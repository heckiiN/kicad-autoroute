"""Shrink a board outline and re-place the parts inside it.

  python kicad_autoroute/compact.py board.dsn --left 0 --right 6 --top 4 --bottom 4 -o compact.json
         [--anchor H1,H2,...] [--spacing 0.4] [--iterations 30000]

Edges are named in KiCad's frame (y down): top = smallest KiCad y. Each edge moves inward by the
given millimetres. Anchored parts (connectors, buttons, LEDs, buzzer, mounting holes by default)
that lie within --edge-band mm of an edge move together with that edge, so they keep their place
relative to the case. Other anchored parts stay where they are. Every other part is first mapped
proportionally into the new outline, then legalised (no courtyard overlap, nothing outside) and
finally annealed for wirelength, ratsnest crossings and congestion with legality as a hard
constraint.

Writes a JSON with the new outline and every part's KiCad position (mm, y down, degrees), which
kicad_steps.py apply applies to a KiCad board copy.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import placement as pl  # noqa: E402

DEFAULT_ANCHORS = r"^(H|MH|J|SW|BT|BZ)\d+$"   # mounting holes, connectors, switches, batteries, buzzers
DEFAULT_SLIDERS = r"(?!x)x"                    # none by default; e.g. "^(J4|J7)$"


BASE_OUTSIDE: dict[str, float] = {}  # overhang a part already had by design (edge connectors)


def _overhang(ev: pl.Evaluator, ref: str, pos) -> float:
    return max(0.0, ev._outside(ref, *pos) - BASE_OUTSIDE.get(ref, 0.0) - 1e-6)


def legality(ev: pl.Evaluator, ref: str, pos, only: set[str] | None = None) -> float:
    """Overlap + extra overhang of ref at pos; with `only`, overlap counts against those parts only."""
    if only is None:
        return ev._part_overlap(ref, *pos) + _overhang(ev, ref, pos)
    p = ev.b.parts[ref]
    a = p.world_body(*pos)
    total = _overhang(ev, ref, pos)
    for q_ref in only:
        if q_ref == ref:
            continue
        q = ev.b.parts[q_ref]
        bb = q.world_body(*ev.pos[q_ref])
        w = min(a[2], bb[2]) - max(a[0], bb[0])
        h = min(a[3], bb[3]) - max(a[1], bb[1])
        if w > 0 and h > 0:
            total += w * h
    return total


def legalise(ev: pl.Evaluator, movable: list[pl.Part], grid: float, max_radius: float, rnd: random.Random,
             only: set[str] | None = None, rounds: int = 6) -> int:
    """Greedy: move each illegal part to the nearest legal grid spot (spiral search)."""
    fixed_moves = 0
    for _ in range(rounds):
        bad = [p for p in movable if legality(ev, p.ref, ev.pos[p.ref], only) > 0]
        if not bad:
            break
        bad.sort(key=lambda p: -(p.body[2] - p.body[0]) * (p.body[3] - p.body[1]))
        for p in bad:
            x, y, r = ev.pos[p.ref]
            if legality(ev, p.ref, (x, y, r), only) <= 0:
                continue
            best = None
            steps = int(max_radius / grid)
            for ring in range(1, steps + 1):
                cands = []
                for i in range(-ring, ring + 1):
                    for dx, dy in ((i, -ring), (i, ring), (-ring, i), (ring, i)):
                        if (p.axis == "x" and dy) or (p.axis == "y" and dx):
                            continue
                        cands.append((x + dx * grid, y + dy * grid))
                rnd.shuffle(cands)
                for cx, cy in cands:
                    rots = (r,) if p.axis else (r, (r + 180) % 360) if len(p.pins) > 4 else                         (r, (r + 90) % 360, (r + 180) % 360, (r + 270) % 360)
                    for rot in rots:
                        if legality(ev, p.ref, (cx, cy, rot), only) <= 0:
                            best = (cx, cy, rot)
                            break
                    if best:
                        break
                if best:
                    break
            if best:
                ev.try_move({p.ref: best})
                fixed_moves += 1
    return fixed_moves


def legalise_groups(ev: pl.Evaluator, groups: list[list[str]], grid: float, max_radius: float) -> int:
    """Move each rigid group as a unit to the nearest offset where none of its members collides
    with a part outside the group (overlaps inside a group are the designer's own layout)."""
    # Blocks are placed first and only against fixed parts and the other blocks; the loose small
    # parts are legalised afterwards and make room around them.
    frame = {r for r, p in ev.b.parts.items() if p.fixed} | {r for g in groups for r in g}
    moved = 0
    for refs in sorted(groups, key=len, reverse=True):
        members = [ev.b.parts[r] for r in refs]
        if any(m.fixed for m in members):
            continue
        others = frame - set(refs)
        axis = next((m.axis for m in members if m.axis), None)

        def bad(dx, dy):
            return sum(legality(ev, r, (ev.pos[r][0] + dx, ev.pos[r][1] + dy, ev.pos[r][2]), others) for r in refs)

        if __import__("os").environ.get("COMPACT_DEBUG"):
            print(f"  group {sorted(refs)[:3]}.. axis={axis} start violation {bad(0, 0) / ev.b.unit_mm**2:.2f} mm2")
        if bad(0, 0) <= 0:
            continue
        best = None
        for ring in range(1, int(max_radius / grid) + 1):
            for i in range(-ring, ring + 1):
                for sx, sy in ((i, -ring), (i, ring), (-ring, i), (ring, i)):
                    if (axis == "x" and sy) or (axis == "y" and sx):
                        continue
                    if bad(sx * grid, sy * grid) <= 0:
                        best = (sx * grid, sy * grid)
                        break
                if best:
                    break
            if best:
                break
        if best:
            ev.try_move({r: (ev.pos[r][0] + best[0], ev.pos[r][1] + best[1], ev.pos[r][2]) for r in refs})
            moved += 1
        else:
            print(f"  group without legal spot (axis={axis}): {sorted(refs)} violation {bad(0, 0) / ev.b.unit_mm**2:.1f} mm2")
    return moved


def build_groups(b: pl.Board, ns) -> list[list[str]]:
    """Union parts that share a wide net (power, HV, drive; not GND) and sit close together."""
    wide_nets = set()
    for cls in pl.dsn.children(pl.dsn.child(b.tree, "network") or [], "class"):
        rule = pl.dsn.child(cls, "rule")
        width = next((float(pl.dsn.atoms(w)[1]) for w in pl.dsn.children(rule or [], "width")), 0.0)
        if width >= ns.group_width * b.unit_mm:
            wide_nets.update(pl.dsn.atoms(cls)[2:])
    wide_nets.discard("GND")
    parent = {r: r for r in b.parts}

    def find(r):
        while parent[r] != r:
            parent[r] = parent[parent[r]]
            r = parent[r]
        return r

    limit = ns.group_radius * b.unit_mm
    for net in wide_nets:
        refs = sorted({ref for ref, _ in b.nets.get(net, [])})
        for i, a in enumerate(refs):
            for c in refs[i + 1:]:
                pa, pc = b.parts[a], b.parts[c]
                if math.dist((pa.x, pa.y), (pc.x, pc.y)) <= limit:
                    parent[find(a)] = find(c)
    groups: dict[str, list[str]] = {}
    for r in b.parts:
        groups.setdefault(find(r), []).append(r)
    return [g for g in groups.values() if len(g) > 1]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("board", type=Path)
    ap.add_argument("-o", "--output", type=Path, required=True)
    for e in ("left", "right", "top", "bottom"):
        ap.add_argument(f"--{e}", type=float, default=0.0, help=f"mm the {e} edge moves inward")
    ap.add_argument("--anchor-re", default=DEFAULT_ANCHORS, help="regex of refs that never move freely")
    ap.add_argument("--slide-re", default=DEFAULT_SLIDERS,
                    help="regex of edge parts that follow their edge but may slide along it")
    ap.add_argument("--edge-band", type=float, default=5.0, help="anchored parts this close to an edge follow it")
    ap.add_argument("--spacing", type=float, default=0.4, help="extra mm kept between part courtyards")
    ap.add_argument("--iterations", type=int, default=30000)
    ap.add_argument("--hv-nets", default="", help="comma-separated high-voltage nets (extra spacing around their parts)")
    ap.add_argument("--groups", action="store_true",
                    help="keep parts tied by wide (power/HV/drive) nets as rigid groups")
    ap.add_argument("--group-width", type=float, default=0.5, help="net-class width in mm that makes a net 'wide'")
    ap.add_argument("--group-radius", type=float, default=8.0, help="max mm between parts joined into a group")
    ap.add_argument("--edge-clearance", type=float, default=0.0,
                    help="copper-to-edge clearance of the board rules (mm): movable parts stay that far inside")
    ap.add_argument("--hv-margin", type=float, default=1.2, help="extra mm around parts on high-voltage nets")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--w-cross", type=float, default=0.3)
    ap.add_argument("--w-rudy", type=float, default=0.1)
    ns = ap.parse_args()

    import re
    anchor_re = re.compile(ns.anchor_re)
    slide_re = re.compile(ns.slide_re)
    b = pl.load(ns.board, channel_mm=0.5, spacing_mm=ns.spacing)
    mm = b.unit_mm
    hv_parts = {ref for net in ns.hv_nets.split(",") for ref, _ in b.nets.get(net.strip(), [])}
    for ref in hv_parts:
        p = b.parts[ref]
        m = ns.hv_margin * mm / 2
        p.body = (p.body[0] - m, p.body[1] - m, p.body[2] + m, p.body[3] + m)
    if hv_parts:
        print("high-voltage parts with extra margin:", sorted(hv_parts))
    groups = build_groups(b, ns) if ns.groups else []
    x0, y0, x1, y1 = b.outline                         # DSN frame: y up, KiCad top edge = y1
    nx0, nx1 = x0 + ns.left * mm, x1 - ns.right * mm
    ny0, ny1 = y0 + ns.bottom * mm, y1 - ns.top * mm   # KiCad bottom = DSN y0
    band = ns.edge_band * mm

    orig = {p.ref: (p.x, p.y) for p in b.parts.values()}
    for p in b.parts.values():
        p.fixed = anchor_re.search(p.ref) is not None or slide_re.search(p.ref) is not None
    anchored_log = {}
    for p in b.parts.values():
        if p.fixed:
            bx = p.world_body()
            dx = dy = 0.0
            edges = []
            if bx[0] - x0 <= band:
                dx += ns.left * mm; edges.append("left")
            if x1 - bx[2] <= band:
                dx -= ns.right * mm; edges.append("right")
            if bx[1] - y0 <= band:
                dy += ns.bottom * mm; edges.append("bottom")
            if y1 - bx[3] <= band:
                dy -= ns.top * mm; edges.append("top")
            p.x, p.y = p.x + dx, p.y + dy
            anchored_log[p.ref] = edges or ["stays"]
            if slide_re.search(p.ref) and len(edges) == 1:
                p.fixed = False
                p.axis = "y" if edges[0] in ("left", "right") else "x"
                anchored_log[p.ref] = edges + ["slides"]
        else:
            p.x = nx0 + (p.x - x0) * (nx1 - nx0) / (x1 - x0)
            p.y = ny0 + (p.y - y0) * (ny1 - ny0) / (y1 - y0)
    # parts are checked against the outline inset by the copper-to-edge clearance (anchored edge
    # parts keep the overhang they had by design through BASE_OUTSIDE)
    ec = ns.edge_clearance * mm
    b.outline = (nx0 + ec, ny0 + ec, nx1 - ec, ny1 - ec)
    for refs in groups:
        members = [b.parts[r] for r in refs]
        lead = next((m for m in members if m.fixed), None) or next((m for m in members if m.axis), None)
        # every member gets the lead's shift (anchored/slider) or the group's proportional shift
        if lead is not None:
            dx, dy = lead.x - orig[lead.ref][0], lead.y - orig[lead.ref][1]
        else:
            gx = sum(orig[r][0] for r in refs) / len(refs)
            gy = sum(orig[r][1] for r in refs) / len(refs)
            dx = nx0 + (gx - x0) * (nx1 - nx0) / (x1 - x0) - gx
            dy = ny0 + (gy - y0) * (ny1 - ny0) / (y1 - y0) - gy
        for m in members:
            m.x, m.y = orig[m.ref][0] + dx, orig[m.ref][1] + dy
            if lead is not None and lead.fixed:
                m.fixed = True
            else:
                m.fixed, m.axis, m.group = False, (lead.axis if lead is not None else None), list(refs)
    if groups:
        print(f"rigid groups ({len(groups)}):", [sorted(g) for g in groups])

    ev = pl.Evaluator(b, ns.w_cross, ns.w_rudy, 20.0)
    for p in b.parts.values():
        if p.fixed or p.axis:
            BASE_OUTSIDE[p.ref] = ev._outside(p.ref, *ev.pos[p.ref])
    movable = [p for p in b.parts.values() if not p.fixed]
    rnd = random.Random(ns.seed)
    before = ev.overlap + ev.outside
    # Edge sliders first, against the anchored parts only; then everything else against all parts.
    sliders = [p for p in movable if p.axis and not p.group]
    frame = {p.ref for p in b.parts.values() if p.fixed or p.axis or p.group}
    moved = legalise(ev, sliders, 0.25 * mm, 25 * mm, rnd, only=frame)
    moved += legalise_groups(ev, [g for g in groups if not b.parts[g[0]].fixed], 0.5 * mm, 30 * mm)
    moved += legalise(ev, [p for p in movable if not p.axis and not p.group], 0.25 * mm, 15 * mm, rnd)
    after = ev.overlap + ev.outside
    print(f"outline {(x1 - x0) / mm:.1f} x {(y1 - y0) / mm:.1f} -> {(nx1 - nx0) / mm:.1f} x {(ny1 - ny0) / mm:.1f} mm; "
          f"{len(movable)} movable, legalised {moved} parts, violation area {before / mm**2:.1f} -> {after / mm**2:.1f} mm2")
    illegal_fixed = [p.ref for p in b.parts.values() if p.fixed and ev._outside(p.ref, *ev.pos[p.ref]) > 0
                     and ev._outside(p.ref, *ev.pos[p.ref]) > 0.5 * mm * mm and anchored_log[p.ref] == ["stays"]]
    if illegal_fixed:
        print("warning: anchored parts not on an edge end up outside:", illegal_fixed)
    print("  start metrics:", ev.metrics())
    ev.hp0 = max(sum(ev.net_hpwl.values()), 1e-9)
    ev.cross0 = max(ev.crossings, 1)
    ev.cong0 = max(ev.congestion(), 1e-9)
    pl.anneal(ev, "global", 0, ns.iterations, ns.seed, 0.25, max(1, ns.iterations // 5), max_rotate_pins=4)
    remaining = ev.overlap + ev.outside
    print("  final metrics:", ev.metrics())

    parts = {}
    for p in b.parts.values():
        x, y, r = ev.pos[p.ref]
        parts[p.ref] = {"x": round(x / mm, 4), "y": round(-y / mm, 4), "rot": r % 360,
                        "front": p.front, "anchored": p.fixed, "slides": p.axis}
    out = {"outline_mm": {"left": nx0 / mm, "right": nx1 / mm, "top": -ny1 / mm, "bottom": -ny0 / mm},
           "violation_mm2": round(remaining / mm**2, 2), "anchored_edges": anchored_log, "parts": parts,
           "metrics": ev.metrics()}
    ns.output.write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"wrote {ns.output} (remaining violation {remaining / mm**2:.2f} mm2)")


if __name__ == "__main__":
    main()
