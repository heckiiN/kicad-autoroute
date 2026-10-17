"""Net classes of a KiCad project (track width, clearance) read from the .kicad_pro.

pcbnew's Python bindings do not expose the effective net class reliably, so the scripts read the
project file: the class table plus pattern / explicit assignments (KiCad 7+ format).
"""
from __future__ import annotations

import fnmatch
import json
import os
import re


class NetClasses:
    def __init__(self, classes: dict[str, tuple[float, float]], rules: list[tuple[re.Pattern, str]]):
        self.classes = classes
        self.rules = rules

    @classmethod
    def for_board(cls, board_path: str) -> "NetClasses":
        pro = os.path.splitext(board_path)[0] + ".kicad_pro"
        if not os.path.exists(pro):
            return cls({"Default": (0.25, 0.2)}, [])
        project = json.load(open(pro, encoding="utf-8"))
        data = project.get("net_settings", {})
        # a class value below the board minimum (Board Setup > Constraints) fails DRC: raise it
        min_w, min_c = board_minimums(board_path, project)
        classes = {c["name"]: (max(c.get("track_width") or 0.25, min_w), max(c.get("clearance") or 0.2, min_c))
                   for c in data.get("classes", [])}
        classes.setdefault("Default", (max(0.25, min_w), max(0.2, min_c)))
        rules = []
        for net, assigned in (data.get("netclass_assignments") or {}).items():
            name = assigned[0] if isinstance(assigned, list) else assigned
            rules.append((re.compile(re.escape(net) + r"\Z"), name))
        for p in data.get("netclass_patterns") or []:
            pat = p["pattern"]
            rx = re.compile(fnmatch.translate(pat)) if any(ch in pat for ch in "*?[") else \
                re.compile(re.escape(pat) + r"\Z")
            rules.append((rx, p["netclass"]))
        return cls(classes, rules)

    @staticmethod
    def dru_net_clearances(board_path: str, detail: bool = False) -> dict:
        """Per-net clearance minimums from the custom rules in the .kicad_dru, resolved against the
        board's nets. Understood: conditions on A.NetName / A.NetClass (KiCad wildcards * and ?),
        optionally with "B.NetName != A.NetName" or a B.NetName / B.NetClass pair; a pair rule counts
        for the A nets (conservative). Rules that test footprints, courtyards, layers or item types
        are local exceptions and are skipped.
        With detail=True the values are (clearance, group_only): group_only means the rule spares
        nets of the same class ("B.NetClass != 'Iso'"), so inside that class the normal clearance holds."""
        dru = os.path.splitext(board_path)[0] + ".kicad_dru"
        if not os.path.exists(dru):
            return {}
        text = open(dru, encoding="utf-8").read()
        nets = board_net_names(board_path)
        classes = NetClasses.for_board(board_path)

        def matches(kind: str, pattern: str) -> list[str]:
            if kind == "NetClass":
                return [n for n in nets if fnmatch.fnmatchcase(classes.class_of(n), pattern)]
            return [n for n in nets if fnmatch.fnmatchcase(n, pattern)]

        one_sided: dict[str, float] = {}
        group_only: dict[str, bool] = {}
        pairs: list[tuple[list[str], float]] = []
        for rule in re.split(r"\(rule\s", text)[1:]:
            cond = re.search(r'\(condition\s+"([^"]*)"\)', rule)
            clr = re.search(r"\(constraint\s+clearance\s+\(min\s+([\d.]+)\s*mm\)", rule)
            if not cond or not clr:
                continue
            value = float(clr.group(1))
            for alt in cond.group(1).split("||"):
                if re.search(r"[A-Za-z_]\w*\s*\(", alt):  # memberOfFootprint(), intersectsCourtyard(), ...
                    continue
                if re.search(r"\b[AB]\.(?!NetName\b|NetClass\b)\w+", alt):  # Type, Layer, ...
                    continue
                a = re.findall(r"A\.(NetName|NetClass)\s*==\s*'([^']*)'", alt)
                if not a:
                    continue
                targets = sorted({n for kind, pat in a for n in matches(kind, pat)})
                if re.search(r"B\.(NetName|NetClass)\s*==", alt):
                    pairs.append((targets, value))
                else:
                    spares_own = any(kind == "NetClass" and re.search(
                        r"B\.NetClass\s*!=\s*(?:A\.NetClass|'" + re.escape(pat) + "')", alt) for kind, pat in a)
                    for n in targets:
                        if value > one_sided.get(n, 0.0):
                            one_sided[n] = value
                            group_only[n] = spares_own
        for targets, value in pairs:
            for n in targets:
                if value > one_sided.get(n, 0.0):
                    one_sided[n] = value
                    group_only[n] = False
        if detail:
            return {n: (v, group_only.get(n, False)) for n, v in one_sided.items()}
        return one_sided

    def class_of(self, net: str) -> str:
        for rx, name in self.rules:
            if rx.match(net or ""):
                return name
        return "Default"

    def width(self, net: str) -> float:
        return self.classes.get(self.class_of(net), self.classes["Default"])[0]

    def clearance(self, net: str) -> float:
        return self.classes.get(self.class_of(net), self.classes["Default"])[1]


def board_minimums(board_path: str, project: dict | None = None) -> tuple[float, float]:
    """Minimum track width and clearance of the board setup and unconditional .kicad_dru rules."""
    base = os.path.splitext(board_path)[0]
    if project is None:
        project = json.load(open(base + ".kicad_pro", encoding="utf-8")) if os.path.exists(base + ".kicad_pro") else {}
    rules = project.get("board", {}).get("design_settings", {}).get("rules", {})
    min_w, min_c = rules.get("min_track_width") or 0.0, rules.get("min_clearance") or 0.0
    if os.path.exists(base + ".kicad_dru"):
        for rule in re.split(r"\(rule\s", open(base + ".kicad_dru", encoding="utf-8").read())[1:]:
            if re.search(r"\(condition\s", rule):
                continue
            for kind, v in re.findall(r"\(constraint\s+(track_width|clearance)\s+\(min\s+([\d.]+)\s*mm", rule):
                if kind == "track_width":
                    min_w = max(min_w, float(v))
                else:
                    min_c = max(min_c, float(v))
    return min_w, min_c


def board_net_names(board_path: str) -> list[str]:
    """Net names declared in a .kicad_pcb (read from the file, no pcbnew needed)."""
    text = open(board_path, encoding="utf-8").read()
    # KiCad <= 9: (net 12 "GND") table; KiCad 10: items carry (net "GND")
    names = re.findall(r'\(net\s+(?:\d+\s+)?"((?:[^"\\]|\\.)*)"\)', text)
    return sorted({n.replace('\\"', '"') for n in names if n})


class ViaRules:
    """Via size the board's rules allow: the Default net class via, raised to the minimum via
    diameter, drill and annular ring of the board setup and unconditional .kicad_dru rules."""

    def __init__(self, default: tuple[float, float], min_dia: float, min_drill: float, min_annular: float):
        self.min_dia, self.min_drill, self.min_annular = min_dia, min_drill, min_annular
        self.default = self.legal(*default)

    @classmethod
    def for_board(cls, board_path: str) -> "ViaRules":
        base = os.path.splitext(board_path)[0]
        default, min_dia, min_drill, min_annular = (0.6, 0.3), 0.0, 0.0, 0.0
        if os.path.exists(base + ".kicad_pro"):
            data = json.load(open(base + ".kicad_pro", encoding="utf-8"))
            rules = data.get("board", {}).get("design_settings", {}).get("rules", {})
            min_dia = rules.get("min_via_diameter") or 0.0
            min_drill = rules.get("min_through_hole_diameter") or 0.0
            min_annular = rules.get("min_via_annular_width") or 0.0
            for c in data.get("net_settings", {}).get("classes", []):
                if c.get("name") == "Default" and c.get("via_diameter") and c.get("via_drill"):
                    default = (c["via_diameter"], c["via_drill"])
        if os.path.exists(base + ".kicad_dru"):
            text = open(base + ".kicad_dru", encoding="utf-8").read()
            for rule in re.split(r"\(rule\s", text)[1:]:
                cond = re.search(r'\(condition\s+"([^"]*)"\)', rule)
                if cond and not re.fullmatch(r"\s*A\.Type\s*==\s*'[Vv]ia'\s*", cond.group(1)):
                    continue  # conditional rules apply to some items only
                for kind, v in re.findall(r"\(constraint\s+(via_diameter|hole_size|annular_width)\s+\(min\s+([\d.]+)\s*mm", rule):
                    v = float(v)
                    if kind == "via_diameter":
                        min_dia = max(min_dia, v)
                    elif kind == "hole_size":
                        min_drill = max(min_drill, v)
                    else:
                        min_annular = max(min_annular, v)
        return cls(default, min_dia, min_drill, min_annular)

    def legal(self, dia: float, drill: float) -> tuple[float, float]:
        drill = max(drill, self.min_drill)
        dia = max(dia, self.min_dia, drill + 2 * self.min_annular)
        return round(dia + 1e-9, 3), round(drill + 1e-9, 3)

    def options(self, *sizes: tuple[float, float]) -> list[tuple[float, float]]:
        """The given sizes made legal, duplicates removed, order kept."""
        out: list[tuple[float, float]] = []
        for s in sizes:
            v = self.legal(*s)
            if v not in out:
                out.append(v)
        return out
