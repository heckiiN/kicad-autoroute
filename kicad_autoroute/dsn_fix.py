"""Bring a KiCad-exported Specctra DSN in line with the board's own rules (plain python).

  python kicad_autoroute/dsn_fix.py board.dsn board.kicad_pcb

KiCad's exporter leaves rules out that the router needs to know:
  * vias: the net-class via goes out as is, even when it is below the board's minimum via diameter,
    drill or annular ring; every via the router places then fails DRC. Via padstacks are raised to
    the board minimums (ViaRules) and renamed, so KiCad reads the new size back from the SES.
  * board minimums: net-class track width / clearance below the board's minimum go out as is;
    they are raised to the minimum.
  * custom clearance rules (.kicad_dru), e.g. high-voltage nets with 1.3 mm to everything: nets
    with such a rule move to a class of their own whose clearance is the rule value; Freerouting
    keeps at least that much to every other class.
"""
from __future__ import annotations

import os
import re
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dsn  # noqa: E402
from dsn import Atom  # noqa: E402
from kicad_classes import NetClasses, ViaRules, board_minimums  # noqa: E402

VIA_NAME = re.compile(r"Via\[(\d+-\d+)\]_(\d+):(\d+)_um\Z")


def _rename(node, old: str, new: str) -> None:
    for i, x in enumerate(node):
        if isinstance(x, Atom):
            if x.text == old:
                node[i] = Atom(new, x.quoted)
        else:
            _rename(x, old, new)


def fix_vias(tree, rules: ViaRules) -> list[str]:
    notes = []
    library = dsn.child(tree, "library")
    if library is None:
        return notes
    names = {dsn.atoms(p)[1] for p in dsn.children(library, "padstack")}
    for ps in list(dsn.children(library, "padstack")):
        name = dsn.atoms(ps)[1]
        m = VIA_NAME.match(name)
        if not m:
            continue
        dia, drill = int(m.group(2)) / 1000, int(m.group(3)) / 1000
        ldia, ldrill = rules.legal(dia, drill)
        if (ldia, ldrill) == (dia, drill):
            continue
        new = f"Via[{m.group(1)}]_{round(ldia * 1000)}:{round(ldrill * 1000)}_um"
        if new in names:
            library.remove(ps)
        else:
            for shape in dsn.children(ps, "shape"):
                for circle in dsn.children(shape, "circle"):
                    circle[2] = Atom(str(round(ldia * 1000)))
            names.add(new)
        _rename(tree, name, new)
        notes.append(f"{name} -> {new}")
    return notes


def fix_minimums(tree, board: str) -> list[str]:
    """Net-class widths and clearances below the board minimums are raised to them."""
    min_w, min_c = (round(v * 1000) for v in board_minimums(board))
    notes = []
    rules = [("structure", dsn.child(dsn.child(tree, "structure"), "rule"))]
    rules += [(dsn.atoms(c)[1], dsn.child(c, "rule")) for c in dsn.children(dsn.child(tree, "network"), "class")]
    for owner, rule in rules:
        if rule is None:
            continue
        for key, minimum in (("width", min_w), ("clearance", min_c)):
            for item in dsn.children(rule, key):
                if len(item) != 2:  # (clearance 50 (type smd_smd)) and the like stay
                    continue
                if float(item[1].text) < minimum:
                    notes.append(f"{owner}: {key} {item[1].text} -> {minimum} um (board minimum)")
                    item[1] = Atom(str(minimum))
    return notes


def fix_clearances(tree, board: str) -> list[str]:
    extra = NetClasses.dru_net_clearances(board, detail=True)
    if not extra:
        return []
    network = dsn.child(tree, "network")
    notes = []
    group_classes: list[tuple[str, int]] = []  # classes whose clearance holds only towards other classes
    for cls in list(dsn.children(network, "class")):
        name = dsn.atoms(cls)[1]
        rule = dsn.child(cls, "rule")
        clr = dsn.child(rule, "clearance") if rule is not None else None
        base = round(float(dsn.atoms(clr)[1])) if clr is not None else 0
        width = dsn.child(rule, "width") if rule is not None else None
        moved: dict[tuple[int, bool], list[str]] = {}
        for net in dsn.atoms(cls)[2:]:
            value, group_only = extra.get(net, (0.0, False))
            um = round(value * 1000)
            if um > base:
                moved.setdefault((um, group_only), []).append(net)
        if not moved:
            continue
        gone = {n for v in moved.values() for n in v}
        cls[:] = [x for x in cls if not (isinstance(x, Atom) and x.text in gone)]
        for (um, group_only), members in sorted(moved.items()):
            new_name = f"{name}DRU{um}{'G' if group_only else ''}".replace("_", "")
            new = [Atom("class"), Atom(new_name)] + [Atom(n, True) for n in members]
            circuit = dsn.child(cls, "circuit")
            if circuit is not None:
                new.append(circuit)
            new_rule = [Atom("rule")]
            if width is not None:
                new_rule.append(width)
            # inside a group rule ("B.NetClass != 'Iso'") the class keeps its normal clearance
            new_rule.append([Atom("clearance"), Atom(str(base if group_only else um))])
            new.append(new_rule)
            network.append(new)
            if group_only:
                group_classes.append((new_name, um))
            notes.append(f"{len(members)} nets of {name} -> clearance {um / 1000} mm"
                         + (" (to other classes)" if group_only else ""))
        if len(dsn.atoms(cls)) <= 2:
            network.remove(cls)
    # clearance each class demands towards other classes (a pair setting overrides, so take the max)
    towards = {}
    for c in dsn.children(network, "class"):
        rule = dsn.child(c, "rule")
        clr = dsn.child(rule, "clearance") if rule is not None else None
        towards[dsn.atoms(c)[1]] = round(float(dsn.atoms(clr)[1])) if clr is not None else 0
    towards.update(dict(group_classes))
    done = set()
    for name, um in group_classes:
        for other in towards:
            if other == name or frozenset((name, other)) in done:
                continue
            done.add(frozenset((name, other)))
            value = max(um, towards[other])
            network.append([Atom("class_class"), [Atom("classes"), Atom(name, True), Atom(other, True)],
                            [Atom("rule"), [Atom("clearance"), Atom(str(value))]]])
    return notes


def single_layer(src: str, dst: str, layer: str) -> int:
    """Copy of a DSN in which every net class may only use one layer (single-sided routing, the way a
    simple through-hole board is laid out by hand). Returns the number of classes restricted."""
    tree = dsn.read(src)
    n = 0
    for cls in dsn.children(dsn.child(tree, "network"), "class"):
        circuit = dsn.child(cls, "circuit")
        if circuit is None:
            circuit = [Atom("circuit")]
            cls.append(circuit)
        circuit[:] = [c for c in circuit if dsn.head(c) != "use_layer"]
        circuit.append([Atom("use_layer"), Atom(layer, True)])
        n += 1
    Path(dst).write_text(dsn.dumps(tree) + "\n", encoding="latin-1")
    return n


def layer_costs(path: str, main: str, factor: float) -> None:
    """Make traces on every layer but `main` cost `factor` times more, so the router keeps most
    copper on one side and uses the other for crossings, as people lay out 2-layer boards."""
    tree = dsn.read(path)
    structure = dsn.child(tree, "structure")
    structure[:] = [c for c in structure if dsn.head(c) != "autoroute_settings"]
    settings = [Atom("autoroute_settings"), [Atom("autoroute"), Atom("on")], [Atom("postroute"), Atom("off")]]
    for layer in dsn.children(structure, "layer"):
        name = dsn.atoms(layer)[1]
        cost = 1.0 if name == main else factor
        settings.append([Atom("layer_rule"), Atom(name, True),
                         [Atom("preferred_direction_trace_costs"), Atom(f"{cost:.1f}")],
                         [Atom("against_preferred_direction_trace_costs"), Atom(f"{cost * 1.2:.1f}")]])
    structure.append(settings)
    Path(path).write_text(dsn.dumps(tree) + "\n", encoding="latin-1")


def fix(dsn_path: str, board: str) -> list[str]:
    tree = dsn.read(dsn_path)
    notes = fix_vias(tree, ViaRules.for_board(board)) + fix_minimums(tree, board) + fix_clearances(tree, board)
    if notes:
        Path(dsn_path).write_text(dsn.dumps(tree) + "\n", encoding="latin-1")
    return notes


if __name__ == "__main__":
    for line in fix(sys.argv[1], sys.argv[2]) or ["dsn_fix: nothing to change"]:
        print("dsn_fix:", line)
