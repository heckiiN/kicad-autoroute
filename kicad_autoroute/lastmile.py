"""Close the last unconnected items the autorouter left (run with KiCad's python).

  <kicad-python> kicad_autoroute/lastmile.py board.kicad_pcb drc.json [--window 5] [--grid 0.05] [--min-width 0.15]

For each unconnected pair in a KiCad DRC report (kicad-cli pcb drc --format json), a small A* maze
search runs on a fine grid in a window around both ends, on every copper layer, with vias. Obstacles
are other-net pads, tracks and vias inflated by the required clearance (net classes from the
.kicad_pro, per-net minimums from the .kicad_dru) plus half the trace width; the board edge keeps its
copper-to-edge clearance. Zones are ignored (they are refilled around the new copper). The net-class
width is tried first, then the minimum width. Found paths are inserted as tracks and vias.
The caller re-runs DRC and keeps the result only if it got better.
"""
import argparse
import functools
import heapq
import json
import math
import os
import re
import sys
import time

import numpy as np
import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kicad_classes import NetClasses, ViaRules  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("board")
ap.add_argument("drc", nargs="?", help="KiCad DRC report (JSON) with the unconnected items to route")
ap.add_argument("--window", type=float, default=5.0)
ap.add_argument("--grid", type=float, default=0.05)
ap.add_argument("--min-width", type=float, default=0.15)
ap.add_argument("--via", type=float, nargs=2, default=None, help="default: 0.6/0.3 raised to the board's via rules")
ap.add_argument("--edge", type=float, default=0.5)
ap.add_argument("--via-cost", type=float, default=3.0, help="via cost in mm of trace")
ap.add_argument("--bend-cost", type=float, default=0.5, help="cost of each 45 degree turn in mm of trace")
ap.add_argument("--hole-clearance", type=float, default=0.25, help="min gap between drill holes")
ap.add_argument("--max-span", type=float, default=0.0, help="skip connections longer than this (mm), 0 = no limit")
ap.add_argument("--max-expansions", type=int, default=400000, help="search nodes per connection before giving up")
ap.add_argument("--reduce-vias", action="store_true",
                help="clean-up mode: re-route every net that has vias with a high via cost, keep it if it needs fewer")
ap.add_argument("-v", "--verbose", action="store_true", help="explain failed connections")
ap.add_argument("--max-rip", type=int, default=6, help="tracks a blocked connection may rip up (0: no rip-up)")
ap.add_argument("--max-rip-nets", type=int, default=2, help="nets whose tracks a rip-up may move")
ap.add_argument("--through", action="store_true",
                help="bus mode on a routed board: the bus nets' own routes are taken up first, and the bundles "
                     "may pass through other nets' unlocked tracks; those are taken out for the autorouter to "
                     "route again")
ap.add_argument("--fan-radius", type=float, default=4.0,
                help="bus mode: around its own pads a net may leave its neighbour's corridor this far (mm)")
ap.add_argument("--demand", type=float, default=0.0,
                help="bus mode: steer bundles away from where many other connections must pass (weight of the "
                     "demand map built from the other nets' air lines; 0: off)")
ap.add_argument("--max-push", type=int, default=25,
                help="--through: tracks/vias of other nets one bus may push aside")
ap.add_argument("--trim-fanout", type=float, default=0.0,
                help="bus mode: leave the bundles this far (mm) away from fine-pitch parts, whose fan-out "
                     "the autorouter then does pin by pin (0: bundles run pad to pad)")
ap.add_argument("--rip-depth", type=int, default=2,
                help="a net moved by a rip-up may itself move others, this many levels deep")
ap.add_argument("--rip-cost", type=float, default=1.0, help="cost of crossing a rippable track, mm of trace per grid cell")
ap.add_argument("--buses", action="store_true",
                help="bus mode: the JSON (from buses.py) lists buses; route each as a parallel bundle and lock it")
ap.add_argument("--rebundle", action="store_true",
                help="clean-up mode: the JSON (from buses.py) lists buses; re-route each routed bus as a bundle "
                     "and keep it when it runs more parallel without losing connections")
ap.add_argument("--unlocked", action="store_true",
                help="bus mode: leave the bundles unlocked, so the autorouter may still move single tracks")
ap.add_argument("--lane-discount", type=float, default=0.4,
                help="cost factor for running alongside the previous net of a bus")
ap.add_argument("--max-cells", type=float, default=600000, help="grid cells per layer before the grid gets coarser")
ap.add_argument("--time-budget", type=float, default=900.0,
                help="seconds for all connections together; short connections go first")
ns = ap.parse_args()
ns.via = ViaRules.for_board(ns.board).legal(*(ns.via or (0.6, 0.3)))
MAX_CELLS = ns.max_cells

TO, FROM = pcbnew.ToMM, pcbnew.FromMM
b = pcbnew.LoadBoard(ns.board)
classes = NetClasses.for_board(ns.board)
extra = NetClasses.dru_net_clearances(ns.board)
layers = [l for l in range(pcbnew.PCB_LAYER_ID_COUNT) if pcbnew.IsCopperLayer(l) and b.IsLayerEnabled(l)]
ebb = b.GetBoardEdgesBoundingBox()
EDGE = (TO(ebb.GetLeft()), TO(ebb.GetTop()), TO(ebb.GetRight()), TO(ebb.GetBottom()))
report = json.load(open(ns.drc, encoding="utf-8")) if ns.drc else {}
if any(u.get("thermal") for u in report.get("unconnected_items", [])):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())   # thermal fixes read the current pour shapes


@functools.lru_cache(maxsize=None)
def need(a, c):
    return max(classes.clearance(a), classes.clearance(c), extra.get(a, 0.0), extra.get(c, 0.0))


# drill holes (x, y, radius) of vias and through-hole pads, for hole-to-hole clearance
holes = []
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetDrillSize().x > 0:
            holes.append((TO(p.GetPosition().x), TO(p.GetPosition().y), TO(p.GetDrillSize().x) / 2))
for t in b.GetTracks():
    if t.GetClass() == "PCB_VIA":
        holes.append((TO(t.GetPosition().x), TO(t.GetPosition().y), TO(t.GetDrillValue()) / 2))

# copper shapes as (net, layers, kind, geometry, ref) in mm; ref is the board item for unlocked
# tracks and vias (these may be ripped up and re-routed), None for everything that stays. Vias of
# nets with a pour stay: they may tie the net to the pour.
poured = {z.GetNetname() for z in b.Zones() if not z.GetIsRuleArea()}
items = []
def pad_shape(p):
    """A pad as an obstacle: its own rectangle, turned with the pad ("rbox"), so a part placed at an
    angle (a QFP at 45 degrees) does not grow into its neighbours' bounding boxes; round pads as discs."""
    c = p.GetPosition()
    sx, sy = TO(p.GetSize().x) / 2, TO(p.GetSize().y) / 2
    if p.GetShape() == pcbnew.PAD_SHAPE_CIRCLE:
        return "disc", (TO(c.x), TO(c.y), sx)
    if p.GetShape() == pcbnew.PAD_SHAPE_CUSTOM:
        bb = p.GetBoundingBox()
        return "box", (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom()))
    a = p.GetOrientation().AsRadians()
    # KiCad's y axis points down; the pad's local x axis is turned by -a in board coordinates
    return "rbox", (TO(c.x), TO(c.y), sx, sy, math.cos(a), -math.sin(a))


for f in b.GetFootprints():
    for p in f.Pads():
        pl = [l for l in layers if p.IsOnLayer(l)]
        kind, geo = pad_shape(p)
        items.append((p.GetNetname(), pl, kind, geo, None))
# copper text and graphics (board level and inside footprints) are fixed copper without a net
copper_art = list(b.GetDrawings())
for f in b.GetFootprints():
    copper_art += list(f.GraphicalItems()) + [f.Reference(), f.Value()]
for d in copper_art:
    if d.GetClass() == "PCB_FIELD" and not d.IsVisible():
        continue
    if pcbnew.IsCopperLayer(d.GetLayer()) and d.GetLayer() in layers:
        bb = d.GetBoundingBox()
        net = d.GetNetname() if hasattr(d, "GetNetname") else ""
        items.append((net, [d.GetLayer()], "box", (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())), None))
# solder mask openings drawn as graphics (exposed copper areas, logos): a track of another net
# under one would be bare next to it (solder mask bridge), so they block the copper layer below;
# an opening around pads of a single net lets that net through
for d in copper_art:
    mask_to_cu = {pcbnew.F_Mask: pcbnew.F_Cu, pcbnew.B_Mask: pcbnew.B_Cu}
    if d.GetClass() in ("PCB_FIELD", "PCB_TEXT") or d.GetLayer() not in mask_to_cu:
        continue
    cu = mask_to_cu[d.GetLayer()]
    if cu not in layers:
        continue
    bb = d.GetBoundingBox()
    box = (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom()))
    inside = {p.GetNetname() for f in b.GetFootprints() for p in f.Pads() if p.IsOnLayer(cu)
              and box[0] <= TO(p.GetPosition().x) <= box[2] and box[1] <= TO(p.GetPosition().y) <= box[3]}
    items.append((inside.pop() if len(inside) == 1 else "", [cu], "box", box, None))
for t in b.GetTracks():
    if t.GetClass() == "PCB_VIA":
        r = TO(t.GetWidth(pcbnew.F_Cu)) / 2
        items.append((t.GetNetname(), layers, "disc", (TO(t.GetPosition().x), TO(t.GetPosition().y), r),
                      t if not t.IsLocked() and t.GetNetname() not in poured else None))
    else:
        items.append((t.GetNetname(), [t.GetLayer()], "seg",
                      (TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y), TO(t.GetWidth()) / 2),
                      t if t.GetClass() == "PCB_TRACK" and not t.IsLocked() else None))


def blocked_map(net, x0, y0, nx, ny, g, half_w, which="all", grid_margin=0.75):
    """Per layer boolean grid: True where a trace centre of half width half_w would violate clearance.
    which: "all" items, only "hard" ones (fixed copper) or only "soft" ones (rippable tracks)."""
    xs = x0 + np.arange(nx) * g
    ys = y0 + np.arange(ny) * g
    X, Y = np.meshgrid(xs, ys)  # [ny, nx]
    maps = {l: np.zeros((ny, nx), bool) for l in layers}
    for it in items:
        if it is None:
            continue
        inet, ilayers, kind, geo, ref = it
        if inet == net or (which == "hard" and ref is not None) or (which == "soft" and ref is None):
            continue
        # paths run straight between free cell centres; such a segment comes at most half a diagonal
        # step (0.71 g) closer to an obstacle than its end points
        m = need(net, inet) + half_w + grid_margin * g
        if kind == "box":
            bx0, by0, bx1, by1 = geo
            lo_x, lo_y, hi_x, hi_y = bx0 - m, by0 - m, bx1 + m, by1 + m
        elif kind == "disc":
            cx, cy, r = geo
            lo_x, lo_y, hi_x, hi_y = cx - r - m, cy - r - m, cx + r + m, cy + r + m
        elif kind == "rbox":
            cx, cy, hx, hy, ca, sa = geo
            rr = math.hypot(hx, hy) + m
            lo_x, lo_y, hi_x, hi_y = cx - rr, cy - rr, cx + rr, cy + rr
        else:
            sx, sy, ex, ey, hw = geo
            lo_x, lo_y = min(sx, ex) - hw - m, min(sy, ey) - hw - m
            hi_x, hi_y = max(sx, ex) + hw + m, max(sy, ey) + hw + m
        # only the cells around the item: its bounding box plus the margin, clipped to the window
        i0, i1 = max(0, int((lo_x - x0) / g)), min(nx, int((hi_x - x0) / g) + 2)
        j0, j1 = max(0, int((lo_y - y0) / g)), min(ny, int((hi_y - y0) / g) + 2)
        if i0 >= i1 or j0 >= j1:
            continue
        Xs, Ys = X[j0:j1, i0:i1], Y[j0:j1, i0:i1]
        if kind == "box":
            d = np.hypot(np.maximum(np.maximum(bx0 - Xs, 0), Xs - bx1), np.maximum(np.maximum(by0 - Ys, 0), Ys - by1))
            hit = d < m
        elif kind == "disc":
            hit = np.hypot(Xs - cx, Ys - cy) < r + m
        elif kind == "rbox":
            u = (Xs - cx) * ca + (Ys - cy) * sa
            v = -(Xs - cx) * sa + (Ys - cy) * ca
            hit = np.hypot(np.maximum(np.abs(u) - hx, 0), np.maximum(np.abs(v) - hy, 0)) < m
        else:
            dx, dy = ex - sx, ey - sy
            L = dx * dx + dy * dy
            t = np.clip(((Xs - sx) * dx + (Ys - sy) * dy) / L, 0, 1) if L > 0 else 0
            hit = np.hypot(Xs - (sx + t * dx), Ys - (sy + t * dy)) < hw + m
        for l in ilayers:
            if l in maps:
                maps[l][j0:j1, i0:i1] |= hit
    if which != "soft":
        edge_m = ns.edge + half_w
        out = (X < EDGE[0] + edge_m) | (X > EDGE[2] - edge_m) | (Y < EDGE[1] + edge_m) | (Y > EDGE[3] - edge_m)
        for l in layers:
            maps[l] |= out
    return maps


def endpoint_layers(desc):
    if "B.Cu" in desc and "F.Cu" not in desc:
        return [pcbnew.B_Cu]
    for name in ("In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu"):
        if name in desc and "F.Cu" not in desc:
            return [b.GetLayerID(name)]
    if desc.startswith(("Via", "Durchkontaktierung")) or "Via" in desc.split(" ")[0]:
        return layers
    if "F.Cu" in desc and ("B.Cu" in desc or "-" in desc):
        return layers
    return [pcbnew.F_Cu]


def resolve(it):
    """(kind, net, layers, position, item) of a DRC item. The board item is looked up by its uuid,
    so neither KiCad's UI language nor custom layer names matter; text parsing is the fallback."""
    pos = (it["pos"]["x"], it["pos"]["y"])
    item = None
    if it.get("uuid"):
        try:
            found = b.ResolveItem(pcbnew.KIID(it["uuid"]))
            item = found.Cast() if found is not None else None
        except Exception:
            item = None
    kind = item.GetClass() if item is not None else ""
    if kind == "ZONE":
        return "zone", item.GetNetname(), [l for l in item.GetLayerSet().Seq() if l in layers], pos, item
    if kind in ("PAD", "PCB_VIA"):
        return kind, item.GetNetname(), [l for l in layers if item.IsOnLayer(l)],             (TO(item.GetPosition().x), TO(item.GetPosition().y)), item
    if kind in ("PCB_TRACK", "PCB_ARC"):
        return "track", item.GetNetname(), [item.GetLayer()], pos, item
    m = re.search(r"\[([^\]]+)\]", it["description"])
    kind = "zone" if it["description"].startswith("Zone") else "other"
    return kind, m.group(1) if m else "", endpoint_layers(it["description"]), pos, None


def zone_target(zone, start, zone_layers):
    """Nearest point of the zone's main fill piece (the largest one) to start, and its layer: a pad
    cut off from a pour connects to the closest piece of the pour, not to the zone's anchor point."""
    best = None
    for layer in zone_layers:
        polys = zone.GetFilledPolysList(layer)
        if polys.OutlineCount() == 0:
            continue
        main = max(range(polys.OutlineCount()), key=lambda i: polys.Outline(i).Area())
        piece = pcbnew.SHAPE_POLY_SET()
        piece.AddOutline(polys.Outline(main))
        for h in range(polys.HoleCount(main)):
            piece.AddHole(polys.Hole(main, h))
        r, step = 0.0, 0.25
        while r < 80 and (best is None or r < best[0]):
            ring = [(start[0] + float(dx), start[1] + dy)
                    for dx in np.arange(-r, r + step / 2, step) for dy in (-r, r)] +                    [(start[0] + dx, start[1] + float(dy))
                    for dy in np.arange(-r + step, r - step / 2, step) for dx in (-r, r)] if r else [start]
            hits = [q for q in ring if piece.Contains(pcbnew.VECTOR2I(FROM(q[0]), FROM(q[1])))]
            if hits:
                q = min(hits, key=lambda q: math.hypot(q[0] - start[0], q[1] - start[1]))
                d = math.hypot(q[0] - start[0], q[1] - start[1])
                if best is None or d < best[0]:
                    best = (d, q, layer)
                break
            r += step if r < 2 else 4 * step
    return (best[1], best[2]) if best else (None, None)


# demand map (1 mm cells over the board outline): how many air lines of other nets cross each cell;
# route() makes a cell dearer by 1 + DEMAND_W * demand when it is set (bus bundles)
DEMAND, DEMAND_W = None, 0.0


def build_demand(exclude):
    """Air lines of every routable net except `exclude` (and nets with a pour), as a minimum spanning
    tree over its pads, rasterised onto 1 mm cells and smoothed."""
    poured_nets = {z.GetNetname() for z in b.Zones() if not z.GetIsRuleArea()}
    x0, y0, x1, y1 = EDGE
    nx, ny = int(x1 - x0) + 2, int(y1 - y0) + 2
    grid = np.zeros((ny, nx))
    pads = {}
    for f in b.GetFootprints():
        for p in f.Pads():
            n = p.GetNetname()
            if n and n not in exclude and n not in poured_nets:
                pads.setdefault(n, []).append((TO(p.GetPosition().x), TO(p.GetPosition().y)))
    for pts in pads.values():
        if len(pts) < 2 or len(pts) > 40:
            continue
        joined, rest = [pts[0]], pts[1:]
        while rest:   # Prim: the shortest edge from the joined pads to a new one
            a, c = min(((u, v) for u in joined for v in rest),
                       key=lambda e: math.hypot(e[0][0] - e[1][0], e[0][1] - e[1][1]))
            joined.append(c)
            rest.remove(c)
            L = math.hypot(c[0] - a[0], c[1] - a[1])
            cells = set()
            for k in range(int(L / 0.5) + 1):
                t = k * 0.5 / L if L else 0
                cells.add((int(a[1] + t * (c[1] - a[1]) - y0), int(a[0] + t * (c[0] - a[0]) - x0)))
            for cy, cx in cells:
                if 0 <= cy < ny and 0 <= cx < nx:
                    grid[cy, cx] += 1
    # air lines within about 2.5 mm of a cell (they are not where the tracks will run exactly)
    padded = np.pad(grid, 2, mode="edge")
    return sum(padded[i:i + ny, j:j + nx] for i in range(5) for j in range(5))


def route(net, a, b_pt, la, lb, width, goal_radius=0.0, deadline=float("inf"), soft=False, fine=False,
          attract=None, pitch=0.0, corridor=0.0, reserve=0.0):
    """Grid A* from a to b_pt. With soft=True unlocked tracks of other nets are passable at a cost
    (rip-up search); otherwise they block like everything else. attract: segments [(layer, x1, y1,
    x2, y2)] of a neighbour (bus); cells at `pitch` from them are cheaper, so the path runs alongside.
    corridor > 0: the path may only leave the neighbour by that much (on the neighbour's layers), except
    around its own two ends where it fans out from the pads; a bus then stays a bundle.
    reserve > 0: away from its ends the path keeps that much more room on both sides (the middle net
    of a bus, so the whole bundle fits next to it)."""
    # long connections search on a coarser grid (at most MAX_CELLS per layer); the clearance margin
    # grows with the grid step, so a coarse path is still legal
    w_mm = abs(a[0] - b_pt[0]) + 2 * ns.window
    h_mm = abs(a[1] - b_pt[1]) + 2 * ns.window
    g = min(0.2, max(ns.grid, math.sqrt(w_mm * h_mm / MAX_CELLS)))
    if fine:  # second try for tight passages (between IC pins): half the grid step
        g = max(ns.grid / 2, math.sqrt(w_mm * h_mm / (4 * MAX_CELLS)))
    x0 = min(a[0], b_pt[0]) - ns.window
    y0 = min(a[1], b_pt[1]) - ns.window
    nx = int((max(a[0], b_pt[0]) + ns.window - x0) / g) + 1
    ny = int((max(a[1], b_pt[1]) + ns.window - y0) / g) + 1
    tmaps = blocked_map(net, x0, y0, nx, ny, g, width / 2, "hard" if soft else "all")
    smaps = blocked_map(net, x0, y0, nx, ny, g, width / 2, "soft") if soft else None
    # a pad in a fine-pitch row (QFP) can only be left along its own axis, where the clearance to its
    # neighbours is just met; the grid margin would close that way on a coarse grid, so right around
    # the two ends the map uses a quarter of that margin (paths leave a pad straight, not diagonally)
    r_esc = 1.5
    for px, py in (a, b_pt):
        i0, i1 = max(0, int((px - r_esc - x0) / g)), min(nx, int((px + r_esc - x0) / g) + 2)
        j0, j1 = max(0, int((py - r_esc - y0) / g)), min(ny, int((py + r_esc - y0) / g) + 2)
        if i0 < i1 and j0 < j1:
            local = blocked_map(net, x0 + i0 * g, y0 + j0 * g, i1 - i0, j1 - j0, g, width / 2,
                                "hard" if soft else "all", grid_margin=0.25)
            for l in layers:
                tmaps[l][j0:j1, i0:i1] &= local[l]
    xs = x0 + np.arange(nx) * g
    ys = y0 + np.arange(ny) * g
    X, Y = np.meshgrid(xs, ys)
    if reserve > 0:
        wide = blocked_map(net, x0, y0, nx, ny, g, width / 2 + reserve, "hard" if soft else "all")
        r_end = 3.0 + 2 * reserve   # out of a pin field first (two rows of a header need ~5 mm)
        ends = (np.hypot(X - a[0], Y - a[1]) <= r_end) | (np.hypot(X - b_pt[0], Y - b_pt[1]) <= r_end)
        tmaps = {l: np.where(ends, tmaps[l], wide[l]) for l in layers}
    rip_cell = ns.rip_cost / g
    vr = ns.via[0] / 2
    vmaps = blocked_map(net, x0, y0, nx, ny, g, vr)
    via_ok = ~np.any(np.stack([vmaps[l] for l in layers]), axis=0)
    for hx, hy, hr in holes:
        reach = hr + ns.via[1] / 2 + ns.hole_clearance
        i0, i1 = max(0, int((hx - reach - x0) / g)), min(nx, int((hx + reach - x0) / g) + 2)
        j0, j1 = max(0, int((hy - reach - y0) / g)), min(ny, int((hy + reach - y0) / g) + 2)
        if i0 < i1 and j0 < j1:
            via_ok[j0:j1, i0:i1] &= np.hypot(X[j0:j1, i0:i1] - hx, Y[j0:j1, i0:i1] - hy) >= reach
    li = {l: i for i, l in enumerate(layers)}
    dem = None
    if DEMAND is not None and DEMAND_W > 0:
        ix = np.clip((X - EDGE[0]).astype(int), 0, DEMAND.shape[1] - 1)
        iy = np.clip((Y - EDGE[1]).astype(int), 0, DEMAND.shape[0] - 1)
        dem = (1.0 + DEMAND_W * DEMAND[iy, ix]).ravel().tolist()
    if attract and corridor > 0:
        tmaps = dict(tmaps)
        inside = {l: np.zeros((ny, nx), bool) for l in layers}
        segs = [s_ for s_ in attract if s_[0] in inside]

        def dist_to(px, py):
            return min((_pt_seg(px, py, *s_[1:]) for s_ in segs), default=float("inf"))
        for layer, sx, sy, ex, ey in segs:
            m = corridor
            i0, i1 = max(0, int((min(sx, ex) - m - x0) / g)), min(nx, int((max(sx, ex) + m - x0) / g) + 2)
            j0, j1 = max(0, int((min(sy, ey) - m - y0) / g)), min(ny, int((max(sy, ey) + m - y0) / g) + 2)
            if i0 >= i1 or j0 >= j1:
                continue
            Xs, Ys = X[j0:j1, i0:i1], Y[j0:j1, i0:i1]
            dx, dy = ex - sx, ey - sy
            L2 = dx * dx + dy * dy
            t = np.clip(((Xs - sx) * dx + (Ys - sy) * dy) / L2, 0, 1) if L2 > 0 else 0
            inside[layer][j0:j1, i0:i1] |= np.hypot(Xs - (sx + t * dx), Ys - (sy + t * dy)) <= m
        for (px, py) in (a, b_pt):   # fan-out room around each pad, on every layer
            r = max(ns.fan_radius, dist_to(px, py) + ns.fan_radius / 2)
            disc = np.hypot(X - px, Y - py) <= r
            for l in layers:
                inside[l] |= disc
        for l in layers:
            tmaps[l] = tmaps[l] | ~inside[l]

    def cell(p):
        return int(round((p[0] - x0) / g)), int(round((p[1] - y0) / g))

    sc, gc = cell(a), cell(b_pt)
    goal = {(gc[0], gc[1], li[l]) for l in lb}
    if goal_radius > 0:  # any cell of the plane layer near the pad will do (the plane is the net itself)
        r = int(goal_radius / g)
        goal = {(gc[0] + dx, gc[1] + dy, li[l]) for l in lb for dx in range(-r, r + 1) for dy in range(-r, r + 1)}
    # the pads themselves are inside other nets' clearance halo only if the design is broken; free the
    # first cells around each end so the search can leave a pad that sits close to a neighbour
    for (cx, cy), ls in ((sc, la), (gc, lb)):
        for l in ls:
            tmaps[l][max(0, cy - 1):cy + 2, max(0, cx - 1):cx + 2] = False
            if smaps is not None:
                smaps[l][max(0, cy - 1):cy + 2, max(0, cx - 1):cx + 2] = False
    # 8 directions in angular order, so |i - j| (mod 8) counts 45 degree steps between them
    dirs = [(1, 0, 1.0), (1, 1, 1.414), (0, 1, 1.0), (-1, 1, 1.414), (-1, 0, 1.0), (-1, -1, 1.414),
            (0, -1, 1.0), (1, -1, 1.414)]
    via_cost = ns.via_cost / g
    turn_cost = ns.bend_cost / g   # per 45 degree of direction change: straight runs, few corners
    NONE = 8                       # no direction yet (start, or right after a via)

    def h(x, y):
        dx, dy = abs(x - gc[0]), abs(y - gc[1])
        return max(dx, dy) + 0.414 * min(dx, dy)

    # flat byte maps: much faster to index in the inner loop than numpy arrays
    L = len(layers)
    hard = [tmaps[layers[k]].tobytes() for k in range(L)]
    near = None
    if attract:
        # the lane next to the neighbour: centre distance pitch .. pitch + 2.5 grid steps
        lanes = {l: np.zeros((ny, nx), bool) for l in layers}
        for layer, sx, sy, ex, ey in attract:
            if layer not in lanes:
                continue
            m = pitch + 2.5 * g   # clearance blocks up to pitch + 0.75 g; leave a lane of ~2 steps
            i0, i1 = max(0, int((min(sx, ex) - m - x0) / g)), min(nx, int((max(sx, ex) + m - x0) / g) + 2)
            j0, j1 = max(0, int((min(sy, ey) - m - y0) / g)), min(ny, int((max(sy, ey) + m - y0) / g) + 2)
            if i0 >= i1 or j0 >= j1:
                continue
            Xs, Ys = X[j0:j1, i0:i1], Y[j0:j1, i0:i1]
            dx, dy = ex - sx, ey - sy
            L2 = dx * dx + dy * dy
            t = np.clip(((Xs - sx) * dx + (Ys - sy) * dy) / L2, 0, 1) if L2 > 0 else 0
            d = np.hypot(Xs - (sx + t * dx), Ys - (sy + t * dy))
            lanes[layer][j0:j1, i0:i1] |= (d >= pitch - 0.5 * g) & (d <= m)
        near = [lanes[layers[k]].tobytes() for k in range(L)]
    soft = [smaps[layers[k]].tobytes() for k in range(L)] if smaps is not None else None
    vias = via_ok.tobytes()

    def search(directional, weight, limit, band=None):
        """A* over (x, y, layer, last direction). Without directions it is a quick feasibility test:
        a ninth of the states, so a connection that cannot be made fails fast."""
        start = [(sc[0], sc[1], li[l], NONE) for l in la]
        dist = {s_: 0.0 for s_ in start}
        prev = {}
        heap = [(weight * h(sc[0], sc[1]), 0.0, s_) for s_ in start]
        heapq.heapify(heap)
        expanded = 0
        while heap:
            expanded += 1
            if expanded > limit or (expanded % 5000 == 0 and time.time() > deadline):
                say(f"  search {net}{' (corners)' if directional else ''}: gave up after {expanded} steps (grid {g:.3f} mm, {nx}x{ny})")
                return None, prev
            f, d, node = heapq.heappop(heap)
            x, y, k, last = node
            if (x, y, k) in goal:
                return node, prev
            if d > dist.get(node, 1e18):
                continue
            hk = hard[k]
            sk = soft[k] if soft is not None else None
            for i, (dx, dy, c) in enumerate(dirs):
                if directional and last != NONE:
                    turn = min((i - last) % 8, (last - i) % 8)
                    if turn > 2:
                        continue        # no acute or reversing corners
                    c += turn * turn_cost
                nx_, ny_ = x + dx, y + dy
                if not (0 <= nx_ < nx and 0 <= ny_ < ny):
                    continue
                cell_ = ny_ * nx + nx_
                if hk[cell_] or (band is not None and not band[cell_]):
                    continue
                if sk is not None and sk[cell_]:
                    c += rip_cell
                if near is not None and near[k][cell_]:
                    c *= ns.lane_discount
                if dem is not None:
                    c *= dem[cell_]
                nb = (nx_, ny_, k, i if directional else NONE)
                nd = d + c
                if nd < dist.get(nb, 1e18):
                    dist[nb] = nd
                    prev[nb] = node
                    heapq.heappush(heap, (nd + weight * h(nx_, ny_), nd, nb))
            here = y * nx + x
            if vias[here]:
                for k2 in range(L):
                    if k2 != k and not hard[k2][here] and not (soft is not None and soft[k2][here]):
                        nb = (x, y, k2, NONE)
                        nd = d + via_cost
                        if nd < dist.get(nb, 1e18):
                            dist[nb] = nd
                            prev[nb] = node
                            heapq.heappush(heap, (nd + weight * h(x, y), nd, nb))
        say(f"  search {net}: no path, {expanded} steps exhausted the window (grid {g:.3f} mm, {nx}x{ny})")
        return None, prev

    found, prev = search(False, 1.0, ns.max_expansions)
    if found is not None:
        # a path exists: search again with corner costs, aimed a little harder at the goal, within a
        # band around the first path (a ninth of the states per cell, so the band keeps it affordable);
        # keep the plain path if that search runs out of budget
        mask = np.zeros((ny, nx), bool)
        r = max(4, int(1.0 / g))
        node = found
        while True:
            x, y = node[0], node[1]
            mask[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1] = True
            if node not in prev:
                break
            node = prev[node]
        found2, prev2 = search(True, 1.2, 2 * ns.max_expansions, mask.tobytes())
        if found2 is not None:
            found, prev = found2, prev2
    if not found:
        return None
    path = [found]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()
    # the path starts and ends on cell centres; pin the ends to the exact pad positions
    pts = [(x0 + x * g, y0 + y * g, layers[k]) for x, y, k, _d in path]
    pts[0] = (a[0], a[1], pts[0][2])
    if goal_radius == 0:
        pts[-1] = (b_pt[0], b_pt[1], pts[-1][2])
    return pts


def _sign(v):
    return 0 if abs(v) < 1e-6 else (1 if v > 0 else -1)


# every board change of insert() and take_out(), so a rip-up can roll back to a checkpoint
journal = []


def take_out(idx):
    """Removes an unlocked track or via from the board (kept for a roll-back)."""
    entry = items[idx]
    b.Remove(entry[4])
    items[idx] = None
    journal.append(("out", entry[4], idx, entry))


def rollback(mark):
    while len(journal) > mark:
        op = journal.pop()
        if op[0] == "in":
            _, obj, idx, via = op
            b.Delete(obj)
            items[idx] = None
            if via:
                holes.pop()
        else:
            _, obj, idx, entry = op
            b.Add(obj)
            items[idx] = entry


def insert(net_name, path, width):
    """Adds the path as tracks and vias; returns the new board items."""
    net = b.FindNet(net_name)
    added = []
    # simplify: keep points where direction or layer changes
    pts = [path[0]]
    for i in range(1, len(path) - 1):
        p0, p1, p2 = pts[-1], path[i], path[i + 1]
        if p1[2] != p0[2] or p2[2] != p1[2]:
            pts.append(p1)
            continue
        d1 = (_sign(p1[0] - p0[0]), _sign(p1[1] - p0[1]))   # step directions, independent of the grid
        d2 = (_sign(p2[0] - p1[0]), _sign(p2[1] - p1[1]))
        if d1[0] * d2[1] != d1[1] * d2[0] or d1[0] * d2[0] + d1[1] * d2[1] < 0:
            pts.append(p1)
    pts.append(path[-1])
    for p, q in zip(pts, pts[1:]):
        if p[2] != q[2]:
            v = pcbnew.PCB_VIA(b)
            v.SetPosition(pcbnew.VECTOR2I(FROM(p[0]), FROM(p[1])))
            v.SetWidth(FROM(ns.via[0])); v.SetDrill(FROM(ns.via[1])); v.SetNet(net)
            b.Add(v)
            holes.append((p[0], p[1], ns.via[1] / 2))
            items.append((net_name, layers, "disc", (p[0], p[1], ns.via[0] / 2), v if net_name not in poured else None))
            journal.append(("in", v, len(items) - 1, True))
            added.append(v)
            continue
        if (p[0], p[1]) == (q[0], q[1]):
            continue
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pcbnew.VECTOR2I(FROM(p[0]), FROM(p[1])))
        t.SetEnd(pcbnew.VECTOR2I(FROM(q[0]), FROM(q[1])))
        t.SetWidth(FROM(width)); t.SetLayer(p[2]); t.SetNet(net)
        b.Add(t)
        items.append((net_name, [p[2]], "seg", (p[0], p[1], q[0], q[1], width / 2), t))
        journal.append(("in", t, len(items) - 1, False))
        added.append(t)
    return added


def say(msg):
    if ns.verbose:
        print(msg, flush=True)


def _pt_seg(px, py, sx, sy, ex, ey):
    dx, dy = ex - sx, ey - sy
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / L))
    return math.hypot(px - sx - t * dx, py - sy - t * dy)


def _seg_seg(a, b_):
    (ax, ay, bx, by), (cx, cy, dx, dy) = a, b_

    def orient(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    o1, o2 = orient(ax, ay, bx, by, cx, cy), orient(ax, ay, bx, by, dx, dy)
    o3, o4 = orient(cx, cy, dx, dy, ax, ay), orient(cx, cy, dx, dy, bx, by)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return 0.0
    return min(_pt_seg(ax, ay, cx, cy, dx, dy), _pt_seg(bx, by, cx, cy, dx, dy),
               _pt_seg(cx, cy, ax, ay, bx, by), _pt_seg(dx, dy, ax, ay, bx, by))


def victims_of(new_items, net):
    """Indices of rippable other-net tracks (in items) that the new tracks/vias violate."""
    shapes = []
    for o in new_items:
        if o.GetClass() == "PCB_VIA":
            shapes.append((None, (TO(o.GetPosition().x), TO(o.GetPosition().y)), TO(o.GetWidth(pcbnew.F_Cu)) / 2))
        else:
            shapes.append((o.GetLayer(), (TO(o.GetStart().x), TO(o.GetStart().y), TO(o.GetEnd().x), TO(o.GetEnd().y)),
                           TO(o.GetWidth()) / 2))
    found = []
    for idx, it in enumerate(items):
        if it is None:
            continue
        inet, ilayers, kind, geo, ref = it
        if ref is None or inet == net or kind not in ("seg", "disc"):
            continue
        clr = need(net, inet) - 1e-4
        for layer, g_, r in shapes:
            if kind == "disc":   # a via of another net: on every layer
                if layer is None:
                    d = math.hypot(g_[0] - geo[0], g_[1] - geo[1]) - r - geo[2]
                else:
                    d = _pt_seg(geo[0], geo[1], *g_) - r - geo[2]
            elif layer is None:
                d = _pt_seg(g_[0], g_[1], *geo[:4]) - r - geo[4]
            elif layer in ilayers:
                d = _seg_seg(g_, geo[:4]) - r - geo[4]
            else:
                continue
            if d < clr:
                found.append(idx)
                break
    return found


def segment_chains(segs):
    """Split segments [(layer, x1, y1, x2, y2)] into groups that touch end to end."""
    key = lambda l, x, y: (l, round(x, 4), round(y, 4))
    groups = []
    for seg in segs:
        ends = {key(seg[0], seg[1], seg[2]), key(seg[0], seg[3], seg[4])}
        touching = [g for g in groups if ends & g[0]]
        merged = ([seg], set(ends))
        for g in touching:
            merged[0].extend(g[1])
            merged[1].update(g[0])
            groups.remove(g)
        groups.append((merged[1], merged[0]))
    return [g[1] for g in groups]


def cut_ends(segs, vias=()):
    """Loose ends of a set of ripped track segments [(layer, x1, y1, x2, y2)]: the points where the
    rest of the net's copper continues. vias: positions of ripped vias, which joined their layers (a
    loose end there has layer None)."""
    def key(layer, x, y):
        for vx, vy in vias:
            if abs(x - vx) < 1e-3 and abs(y - vy) < 1e-3:
                return (None, round(vx, 4), round(vy, 4))
        return (layer, round(x, 4), round(y, 4))
    count = {}
    for layer, x1, y1, x2, y2 in segs:
        for q in (key(layer, x1, y1), key(layer, x2, y2)):
            count[q] = count.get(q, 0) + 1
    return [q for q, n in count.items() if n == 1]


def ripup(net, a, c, la, lc, width, goal_radius, deadline, depth=None):
    """Route a blocked connection through unlocked tracks and vias of other nets, take those out and
    route them again around the new one; a net that then finds no way may itself move others
    (depth levels). Everything is rolled back unless all of it succeeds."""
    depth = ns.rip_depth if depth is None else depth
    mark = len(journal)
    path = route(net, a, c, la, lc, width, goal_radius, deadline, soft=True)
    if not path:
        say(f"rip-up {net}: no path even through rippable tracks")
        return None
    created = insert(net, path, width)
    # the new route must stay where it is while the nets it displaced find new ways
    pinned = {i: items[i] for i in range(len(items) - len(created), len(items))}
    for i in pinned:
        items[i] = items[i][:4] + (None,)
    victims = victims_of(created, net)
    # a ripped via takes the unlocked tracks of its net that end on it along (they would dangle)
    for idx in [i for i in victims if items[i][2] == "disc"]:
        vnet, (vx, vy) = items[idx][0], items[idx][3][:2]
        for j, other in enumerate(items):
            if other is None or j in victims or other[0] != vnet or other[2] != "seg" or other[4] is None:
                continue
            sx, sy, ex, ey = other[3][:4]
            if math.hypot(sx - vx, sy - vy) < 1e-3 or math.hypot(ex - vx, ey - vy) < 1e-3:
                victims.append(j)

    def fail(msg):
        say(f"rip-up {net}: {msg}")
        rollback(mark)
        return None
    if not victims or len(victims) > ns.max_rip:
        return fail(f"{len(victims)} tracks/vias in the way (limit {ns.max_rip})")
    by_net, via_pts, widths = {}, {}, {}
    for idx in victims:
        entry = items[idx]
        if entry[2] == "disc":
            via_pts.setdefault(entry[0], []).append(tuple(entry[3][:2]))
            by_net.setdefault(entry[0], [])
        else:
            by_net.setdefault(entry[0], []).append((entry[4].GetLayer(),) + tuple(entry[3][:4]))
            widths[entry[0]] = entry[3][4] * 2
    if len(by_net) > ns.max_rip_nets:
        return fail(f"{len(by_net)} nets in the way (limit {ns.max_rip_nets})")
    for idx in victims:
        take_out(idx)
    for vnet, segs in by_net.items():
        ends = cut_ends(segs, via_pts.get(vnet, ()))
        if len(ends) < 2 or any(e[0] is None for e in ends) or vnet not in widths:
            return fail(f"ripped tracks of {vnet} leave {len(ends)} loose ends")
        if not rejoin(vnet, ends, widths[vnet], deadline, depth):
            return fail(f"{vnet} could not be routed again")
    for i, entry in pinned.items():   # rippable again for later connections
        if items[i] is not None:
            items[i] = entry
    return len(victims), sorted(by_net)


def rejoin(net, ends, width, deadline, depth):
    """Joins the loose ends [(layer, x, y)] a rip-up left in a net: the first two, then every
    further end to the nearest end already joined (a ripped branch leaves three or more). A join
    that finds no way may move other nets (one level less deep). False if a join fails; the caller
    rolls back."""
    joined = [ends[0]]
    for e in ends[1:]:
        t = min(joined, key=lambda q: math.hypot(q[1] - e[1], q[2] - e[2]))
        p = route(net, (e[1], e[2]), (t[1], t[2]), [e[0]], [t[0]], width, 0.0, deadline)
        if p:
            insert(net, p, width)
        elif depth <= 1 or not ripup(net, (e[1], e[2]), (t[1], t[2]), [e[0]], [t[0]], width, 0.0, deadline,
                                     depth - 1):
            return False
        joined.append(e)
    return True


t_start = time.time()
done, failed, skipped = [], [], []
def span(u):
    its = u.get("items", [])
    if len(its) < 2:
        return 0.0
    return math.hypot(its[0]["pos"]["x"] - its[1]["pos"]["x"], its[0]["pos"]["y"] - its[1]["pos"]["y"])


def main_piece(zone, layer):
    """The largest filled piece of a zone on a layer (the pour the rest of the net hangs on)."""
    polys = zone.GetFilledPolysList(layer)
    if polys.OutlineCount() == 0:
        return None
    i = max(range(polys.OutlineCount()), key=lambda k: polys.Outline(k).Area())
    piece = pcbnew.SHAPE_POLY_SET()
    piece.AddOutline(polys.Outline(i))
    for h in range(polys.HoleCount(i)):
        piece.AddHole(polys.Hole(i, h))
    return piece


def thermal_fix(u):
    """A pad whose thermal relief does not reach the pour properly (too few spokes, or spokes into a
    cut-off piece) gets short tracks into the main pour: KiCad counts a track into the zone as a
    connection. Which case it is follows from the geometry, not from the report's wording."""
    res = [resolve(i) for i in u.get("items", [])]
    pad = next((r for r in res if r[0] == "PAD"), None)
    zone = next((r for r in res if r[0] == "zone"), None)
    if pad is None or zone is None or zone[4] is None:
        return None
    _k, net, pad_layers, a, pad_item = pad
    zone_item = zone[4]
    layers_ = [l for l in zone[2] if l in pad_layers]
    if not layers_:
        return None
    layer = layers_[0]
    bb = pad_item.GetBoundingBox()
    reach = max(TO(bb.GetWidth()), TO(bb.GetHeight())) / 2 + TO(zone_item.GetThermalReliefGap()) + 0.6
    # the fill piece around the pad: when it also holds other copper of the net it is a real part of
    # the pour and only needs another spoke; when the pad is alone in it, it is a cut-off island and
    # the stubs go to the main pour
    fill = zone_item.GetFilledPolysList(layer)
    own = None
    for i in range(fill.OutlineCount()):
        cand = pcbnew.SHAPE_POLY_SET()
        cand.AddOutline(fill.Outline(i))
        for h in range(fill.HoleCount(i)):
            cand.AddHole(fill.Hole(i, h))
        if any(cand.Contains(pcbnew.VECTOR2I(FROM(a[0] + reach * math.cos(math.radians(k * 45))),
                                             FROM(a[1] + reach * math.sin(math.radians(k * 45))))) for k in range(8)):
            own = cand
            break
    others = [o.GetPosition() for f in b.GetFootprints() for o in f.Pads()
              if o.GetNetname() == net and o.IsOnLayer(layer) and o.m_Uuid.AsString() != pad_item.m_Uuid.AsString()]
    others += [t.GetPosition() for t in b.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetNetname() == net]
    island = own is None or not any(own.Contains(o) for o in others)
    piece = main_piece(zone_item, layer) if island else own
    if piece is None:
        return None
    # KiCad counts a track into the pour like a spoke, and wants min_resolved_spokes (normally 2)
    # connections to the main pour. The relief's own spokes run at the spoke angle + k * 90 deg, so
    # stubs go between them: per direction the nearest point of the main pour, then the two closest
    # directions at least 90 deg apart.
    try:
        spoke = float(pad_item.GetThermalSpokeAngle().AsDegrees())
    except Exception:
        spoke = 0.0
    hits = {}
    r = reach
    while r < reach + 20 and len(hits) < 16:
        for k in range(16):
            ang = k * 22.5
            if ang in hits:
                continue
            q = (a[0] + r * math.cos(math.radians(ang)), a[1] + r * math.sin(math.radians(ang)))
            if piece.Contains(pcbnew.VECTOR2I(FROM(q[0]), FROM(q[1]))):
                hits[ang] = (r, q)
        r += 0.5

    def on_spoke(ang):
        return min(abs((ang - spoke) % 90), 90 - abs((ang - spoke) % 90)) < 10

    order = sorted(hits, key=lambda ang: (round(hits[ang][0], 1), on_spoke(ang)))
    candidates = [(ang, hits[ang][1]) for ang in order]
    near, _l = zone_target(zone_item, a, [layer]) if island else (None, None)
    if near is not None:   # the closest point of the main pour, wherever it is
        candidates.insert(0, (math.degrees(math.atan2(near[1] - a[1], near[0] - a[0])) % 360, near))
    made, used = 0, []
    for ang, q in candidates:
        if made == 2 or time.time() > t_start + ns.time_budget:
            break
        if any(min((ang - t) % 360, (t - ang) % 360) < 90 for t in used):
            continue
        for w in sorted({classes.width(net), max(ns.min_width, 0.1)}, reverse=True):
            path = route(net, a, q, [layer], [layer], w, goal_radius=0.25, deadline=t_start + ns.time_budget)
            say(f"thermal {net} {a}: stub towards {ang:.0f} deg {tuple(round(v, 2) for v in q)} w {w}: {'ok' if path else 'no path'}")
            if path and insert(net, path, w):
                made += 1
                used.append(ang)
                break
    return made


def open_connections():
    """Unconnected count after refilling the pours (a re-routed net can cut a pour apart)."""
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    b.BuildConnectivity()
    conn = b.GetConnectivity()
    conn.RecalculateRatsnest()
    return conn.GetUnconnectedCount(False)


def reduce_vias(max_pads=8, via_cost_mm=12.0):
    """Clean-up pass a person would do: every small net that uses vias is taken up and routed again
    pad to pad (minimum spanning tree) with a high via cost. The new route is kept only when it is
    complete, uses fewer vias and is not much longer; otherwise the old one is put back."""
    kept = []
    nets = {}
    for t in b.GetTracks():
        if not t.IsLocked() and t.GetNetname():
            nets.setdefault(t.GetNetname(), []).append(t)
    pads_of = {}
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname():
                pads_of.setdefault(p.GetNetname(), []).append(p)
    order = sorted(nets, key=lambda n: -sum(t.GetClass() == "PCB_VIA" for t in nets[n]))
    saved_cost = ns.via_cost
    baseline = open_connections()
    for net in order:
        if time.time() > t_start + ns.time_budget:
            break
        old = nets[net]
        vias_before = sum(t.GetClass() == "PCB_VIA" for t in old)
        pads = pads_of.get(net, [])
        if vias_before == 0 or not 2 <= len(pads) <= max_pads:
            continue
        length_before = sum(t.GetLength() for t in old if t.GetClass() != "PCB_VIA") / 1e6
        # take the net up (board and obstacle model)
        ids = {o.m_Uuid.AsString() for o in old}
        removed_items = [(i, it) for i, it in enumerate(items) if it is not None and it[0] == net and it[2] in ("seg", "disc")]
        for i, _it in removed_items:
            items[i] = None
        for o in old:
            b.Remove(o)
        n_items, n_holes = len(items), len(holes)
        # minimum spanning tree over the pads
        pts = [((TO(p.GetPosition().x), TO(p.GetPosition().y)), [l for l in layers if p.IsOnLayer(l)]) for p in pads]
        in_tree, edges = {0}, []
        while len(in_tree) < len(pts):
            i, j = min(((i, j) for i in in_tree for j in range(len(pts)) if j not in in_tree),
                       key=lambda e: math.hypot(pts[e[0]][0][0] - pts[e[1]][0][0], pts[e[0]][0][1] - pts[e[1]][0][1]))
            edges.append((i, j))
            in_tree.add(j)
        ns.via_cost = via_cost_mm
        new = []
        ok = True
        for i, j in edges:
            path = None
            for w in sorted({classes.width(net), max(ns.min_width, 0.1)}, reverse=True):
                path = route(net, pts[i][0], pts[j][0], pts[i][1], pts[j][1], w, 0.0, t_start + ns.time_budget)
                if path:
                    new += insert(net, path, w)
                    break
            if not path:
                ok = False
                break
        ns.via_cost = saved_cost
        vias_after = sum(o.GetClass() == "PCB_VIA" for o in new)
        length_after = sum(o.GetLength() for o in new if o.GetClass() != "PCB_VIA") / 1e6
        if ok and vias_after < vias_before and length_after <= 1.5 * length_before + 5:
            now = open_connections()
            if now <= baseline:
                baseline = now
                kept.append(f"{net} vias {vias_before}->{vias_after}")
                continue
        for o in new:
            b.Delete(o)
        for i in range(n_items, len(items)):
            items[i] = None
        del holes[n_holes:]
        for o in old:
            b.Add(o)
        for i, it in removed_items:
            items[i] = it
    open_connections()   # leave the pours filled for the old routes that came back
    return kept


def bundle_order(nets):
    """A bus is laid from its middle net outwards: [(net, neighbour it runs next to)], the middle
    net first with no neighbour, then alternately one further out on each side."""
    m = (len(nets) - 1) // 2
    order = [(nets[m], None)]
    for k in range(1, len(nets)):
        if m - k >= 0:
            order.append((nets[m - k], nets[m - k + 1]))
        if m + k < len(nets):
            order.append((nets[m + k], nets[m + k - 1]))
    return order


def follow(net, a, b_pt, la, lb, width, prev, pitch, reserve=0.0, soft=False):
    """Route one net of a bus. Next to its neighbour (prev): first inside a corridor along it (the
    bundle stays together), then, if the corridor is blocked, anywhere with the lane only preferred.
    The middle net (no neighbour) first keeps room for the rest of the bundle on both sides."""
    deadline = t_start + ns.time_budget
    if prev:
        # a narrow corridor first; a wider one (still well within what reads as running alongside)
        # before giving up the neighbour altogether
        for corridor in (max(1.5, 3 * pitch), 2.4):
            path = route(net, a, b_pt, la, lb, width, 0.0, deadline, attract=prev, pitch=pitch, corridor=corridor,
                         soft=soft)
            if path:
                return path
        say(f"  {net}: corridor along its neighbour blocked, routing freely")
    elif reserve > 0:
        path = route(net, a, b_pt, la, lb, width, 0.0, deadline, reserve=reserve, soft=soft)
        if path:
            return path
        say(f"  {net}: no room for the whole bundle, routing the middle net alone")
    return route(net, a, b_pt, la, lb, width, 0.0, deadline, attract=prev, pitch=pitch, soft=soft)


def bus_pitch(nets):
    return max(classes.width(n) for n in nets) + max(need(x, y) for x, y in zip(nets, nets[1:]))


def _clip_outside(seg, box):
    """Parts of a segment (x1, y1, x2, y2) outside an axis-parallel box (Liang-Barsky)."""
    x1, y1, x2, y2 = seg
    bx0, by0, bx1, by1 = box
    dx, dy = x2 - x1, y2 - y1
    t0, t1 = 0.0, 1.0
    for p_, q_ in ((-dx, x1 - bx0), (dx, bx1 - x1), (-dy, y1 - by0), (dy, by1 - y1)):
        if abs(p_) < 1e-12:
            if q_ < 0:
                return [seg]          # parallel to an edge and outside: no overlap
            continue
        r = q_ / p_
        if p_ < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
    if t0 >= t1:
        return [seg]
    out = []
    if t0 > 1e-9:
        out.append((x1, y1, x1 + t0 * dx, y1 + t0 * dy))
    if t1 < 1 - 1e-9:
        out.append((x1 + t1 * dx, y1 + t1 * dy, x2, y2))
    return out


def fanout_box(ref, nets, margin):
    """The region around a fine-pitch part where its pins fan out: the bounding box of its pads,
    grown by margin; None if the bus pins of the part are not fine-pitch (< 0.8 mm apart)."""
    f = b.FindFootprintByReference(ref)
    if f is None:
        return None
    pads = list(f.Pads())
    bus_pads = [p for p in pads if p.GetNetname() in nets]
    pts = [(TO(p.GetPosition().x), TO(p.GetPosition().y)) for p in bus_pads]
    pitch = min((math.hypot(a[0] - c[0], a[1] - c[1]) for i, a in enumerate(pts) for c in pts[i + 1:]), default=9.0)
    if pitch >= 0.8 or any(p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD for p in bus_pads):
        return None
    boxes = [p.GetBoundingBox() for p in pads]
    return (min(TO(x.GetLeft()) for x in boxes) - margin, min(TO(x.GetTop()) for x in boxes) - margin,
            max(TO(x.GetRight()) for x in boxes) + margin, max(TO(x.GetBottom()) for x in boxes) + margin)


def trim_fanout(objs, boxes):
    """Takes the parts of a laid bundle inside the fan-out boxes out again (vias inside go, tracks
    are cut at the box edge); returns the board items that are left."""
    kept = []
    for o in objs:
        if o.GetClass() == "PCB_VIA":
            x, y = TO(o.GetPosition().x), TO(o.GetPosition().y)
            if any(bx0 <= x <= bx1 and by0 <= y <= by1 for bx0, by0, bx1, by1 in boxes):
                b.Delete(o)
            else:
                kept.append(o)
            continue
        pieces = [(TO(o.GetStart().x), TO(o.GetStart().y), TO(o.GetEnd().x), TO(o.GetEnd().y))]
        for box in boxes:
            pieces = [q for piece in pieces for q in _clip_outside(piece, box)]
        pieces = [q for q in pieces if math.hypot(q[2] - q[0], q[3] - q[1]) > 0.05]
        if not pieces:
            b.Delete(o)
            continue
        for k, (x1, y1, x2, y2) in enumerate(pieces):
            t = o if k == 0 else pcbnew.PCB_TRACK(b)
            if k:
                t.SetWidth(o.GetWidth()); t.SetLayer(o.GetLayer()); t.SetNet(o.GetNet())
                b.Add(t)
            t.SetStart(pcbnew.VECTOR2I(FROM(x1), FROM(y1))); t.SetEnd(pcbnew.VECTOR2I(FROM(x2), FROM(y2)))
            kept.append(t)
    return kept


def join_extra_pads(net, trunk, pads):
    """Joins each pad to the nearest point of the laid trunk tracks; returns the new board items
    (pads that find no way are left to the autorouter)."""
    made = []
    w = classes.width(net)
    for p in pads:
        if not trunk:
            break
        px, py = TO(p.GetPosition().x), TO(p.GetPosition().y)
        best = None
        for t in trunk:
            sx, sy, ex, ey = TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y)
            dx, dy = ex - sx, ey - sy
            L2 = dx * dx + dy * dy
            u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / L2))
            q = (sx + u * dx, sy + u * dy)
            d = math.hypot(q[0] - px, q[1] - py)
            if best is None or d < best[0]:
                best = (d, q, t.GetLayer())
        if best[0] > 15.0:   # far away: a part of its own, not a pull-up beside the bus
            continue
        path = route(net, (px, py), best[1], [l for l in layers if p.IsOnLayer(l)], [best[2]], w, 0.0,
                     t_start + ns.time_budget)
        if not path:
            say(f"  {net}: extra pad at ({px:.1f}, {py:.1f}) left to the autorouter")
            continue
        first = len(items)
        new = insert(net, path, w)
        for k in range(first, len(items)):   # locked with the bundle
            if items[k] is not None:
                items[k] = items[k][:4] + (None,)
        made += new
    return made


def route_buses(bus_data):
    """Lay every bus as a bundle: its nets in pin order, each one drawn to the lane right next to
    the previous one. The bundles are locked, so the autorouter routes around them."""
    pad_of, pads_of = {}, {}
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname():
                pad_of[(f.GetReference(), p.GetNetname())] = p
                pads_of.setdefault(p.GetNetname(), []).append((f.GetReference(), p))
    # parts a bus of the net ends at; any other pad of a bus net (a pull-up, a test point) is joined
    # to the laid trunk, so the autorouter is not tempted to route the net a second time around it
    bus_ends = {}
    for bus_ in bus_data.get("detail", []):
        for n in bus_["nets"]:
            bus_ends.setdefault(n, set()).update((bus_["from"], bus_["to"]))
    report_lines = []

    def landing(net, pad):
        """Where the bus net starts at this pad: the pad itself, or the via its fan-out (laid before,
        locked) already leads to; then the bundle runs to that via on any layer."""
        x, y = TO(pad.GetPosition().x), TO(pad.GetPosition().y)
        if pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD:
            vias = [it[3] for it in items if it is not None and it[0] == net and it[2] == "disc" and it[4] is None
                    and math.hypot(it[3][0] - x, it[3][1] - y) <= 3.0]
            if vias:
                vx, vy, _r = min(vias, key=lambda v: math.hypot(v[0] - x, v[1] - y))
                return (vx, vy), list(layers)
        return (x, y), [l for l in layers if pad.IsOnLayer(l)]

    def ends(bus, net):
        pa, pb = pad_of.get((bus["from"], net)), pad_of.get((bus["to"], net))
        if pa is None or pb is None:
            return None
        (a, la), (c, lc) = landing(net, pa), landing(net, pb)
        return a, la, c, lc

    def span_of(bus):
        e = [ends(bus, n) for n in bus["nets"]]
        e = [x for x in e if x]
        return sum(math.hypot(x[0][0] - x[2][0], x[0][1] - x[2][1]) for x in e) / max(1, len(e))

    # short buses first: they are local, almost always fit, and leave the long ones the most room;
    # a bus that cannot be laid completely is taken up again and left to the autorouter
    joined = {}   # net -> sets of parts already joined by laid bundles (a third bus would close a loop)

    def group(net, ref):
        return next((s_ for s_ in joined.get(net, []) if ref in s_), {ref})

    global DEMAND, DEMAND_W
    if ns.demand > 0:
        DEMAND = build_demand({n for bus in bus_data.get("detail", []) for n in bus["nets"]})
        DEMAND_W = ns.demand
        say(f"demand map: max {DEMAND.max():.1f}, mean {DEMAND.mean():.2f} air lines per mm cell")
    if ns.through:
        import buses as bus_tools
    for bus in sorted(bus_data.get("detail", []), key=span_of):
        if time.time() > t_start + ns.time_budget:
            break
        mark = len(journal)
        if ns.through:
            # on a routed board only buses that are not bundled yet are laid again; their nets' own
            # routes go first (the bundle replaces them)
            before, _l = bus_tools.parallelism(b, bus)
            if before >= 0.6:
                continue
            for i, it in enumerate(items):
                if it is not None and it[0] in bus["nets"] and it[4] is not None:
                    take_out(i)
        if any(bus["to"] in group(n, bus["from"]) for n in bus["nets"]):
            say(f"bus {bus['from']}->{bus['to']}: its parts are already joined by other bundles")
            rollback(mark)
            continue
        n_items, n_holes = len(items), len(holes)
        segs_of, laid, pushed = {}, [], 0
        pitch_all = bus_pitch(bus["nets"])
        skipped_nets = []
        order_ = bus["nets"]
        for net, nb in bundle_order(bus["nets"]):
            e = ends(bus, net)
            if e is None:
                laid = None
                break
            if nb is not None and nb not in segs_of:
                # its inner neighbour did not fit: run next to the nearest laid net on that side
                i_net, i_nb = order_.index(net), order_.index(nb)
                step = 1 if i_nb > i_net else -1
                nb = next((order_[k] for k in range(i_nb, len(order_) if step > 0 else -1, step)
                           if order_[k] in segs_of), None)
            a_, la_, b_, lb_ = e
            w = classes.width(net)
            pitch = w + need(net, nb) if nb else 0.0
            path = follow(net, a_, b_, la_, lb_, w, segs_of.get(nb), pitch,
                          reserve=math.ceil((len(bus["nets"]) - 1) / 2) * pitch_all, soft=ns.through)
            if not path and segs_of and len(bus["nets"]) - len(skipped_nets) > 2:
                # an outer net that does not fit is left to the autorouter; the rest stays a bundle
                say(f"bus {bus['from']}->{bus['to']}: {net} does not fit, left out of the bundle")
                skipped_nets.append(net)
                continue
            if not path:
                say(f"bus {bus['from']}->{bus['to']}: {net} does not fit, bus left to the autorouter")
                laid = None if not laid else laid + [None]
                break
            first = len(items)
            new = insert(net, path, w)
            for k in range(first, len(items)):   # the bundle itself is never taken out again
                if items[k] is not None:
                    items[k] = items[k][:4] + (None,)
            if ns.through:   # whatever of other nets is in the way goes; the autorouter routes it again
                for idx in victims_of(new, net):
                    if items[idx] is not None:
                        take_out(idx)
                        pushed += 1
                if pushed > ns.max_push:
                    say(f"bus {bus['from']}->{bus['to']}: would push {pushed} tracks of other nets aside, left as it is")
                    laid = None if not laid else laid + [None]
                    break
            laid += new
            segs_of[net] = [(o.GetLayer(), TO(o.GetStart().x), TO(o.GetStart().y), TO(o.GetEnd().x),
                             TO(o.GetEnd().y)) for o in new if o.GetClass() == "PCB_TRACK"]
        if laid and None not in laid:
            for n in [n for n in bus["nets"] if n not in skipped_nets]:
                sets = joined.setdefault(n, [])
                ga, gb = group(n, bus["from"]), group(n, bus["to"])
                sets[:] = [s_ for s_ in sets if s_ is not ga and s_ is not gb] + [ga | gb]
            boxes = [x for x in (fanout_box(ref, set(bus["nets"]), ns.trim_fanout) for ref in (bus["from"], bus["to"]))
                     if x] if ns.trim_fanout > 0 else []
            if boxes:
                # the trunk stays, the fan-out at fine-pitch parts is left to the autorouter
                laid = trim_fanout(laid, boxes)
                for i in range(n_items, len(items)):
                    items[i] = None
                del holes[n_holes:]
                for o in laid:
                    if o.GetClass() == "PCB_VIA":
                        x, y = TO(o.GetPosition().x), TO(o.GetPosition().y)
                        holes.append((x, y, TO(o.GetDrillValue()) / 2))
                        items.append((o.GetNetname(), layers, "disc", (x, y, TO(o.GetWidth(pcbnew.F_Cu)) / 2), None))
                    else:
                        items.append((o.GetNetname(), [o.GetLayer()], "seg", (TO(o.GetStart().x), TO(o.GetStart().y),
                                      TO(o.GetEnd().x), TO(o.GetEnd().y), TO(o.GetWidth()) / 2), None))
            for net_ in bus["nets"]:
                laid += join_extra_pads(net_, [o for o in laid if o.GetClass() == "PCB_TRACK" and o.GetNetname() == net_],
                                        [p for ref, p in pads_of.get(net_, []) if ref not in bus_ends.get(net_, ())])
            for o in laid:
                o.SetLocked(not ns.unlocked)
            report_lines.append(f"{bus['from']}->{bus['to']} {len(bus['nets']) - len(skipped_nets)} nets")
        else:   # the bus goes again; whatever it had taken out comes back
            rollback(mark)
    return report_lines


def rebundle(bus_data):
    """Clean-up for a routed board: every bus whose nets join just its two parts is taken up and
    routed again as a bundle (each net in the lane next to the previous one). Kept only when the bus
    runs clearly more parallel and no connection is lost; otherwise the old routes come back."""
    import buses as bus_tools
    kept = []
    pad_of, net_pads = {}, {}
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname():
                pad_of[(f.GetReference(), p.GetNetname())] = p
                net_pads.setdefault(p.GetNetname(), []).append(p)

    def where(p):
        return (TO(p.GetPosition().x), TO(p.GetPosition().y)), [l for l in layers if p.IsOnLayer(l)]
    baseline = open_connections()
    for bus in bus_data.get("detail", []):
        if time.time() > t_start + ns.time_budget:
            break
        nets = [n for n in bus["nets"] if 2 <= len(net_pads.get(n, [])) <= 4
                and (bus["from"], n) in pad_of and (bus["to"], n) in pad_of]
        if len(nets) < 3:
            continue
        sub_bus = dict(bus, nets=nets)
        before, _len = bus_tools.parallelism(b, sub_bus)
        old = [t for t in b.GetTracks() if t.GetNetname() in nets and not t.IsLocked()]
        if not old:
            continue
        removed_items = [(i, it) for i, it in enumerate(items) if it is not None and it[0] in nets and it[2] in ("seg", "disc")]
        for i, _it in removed_items:
            items[i] = None
        for o in old:
            b.Remove(o)
        n_items, n_holes = len(items), len(holes)
        new, ok, segs_of = [], True, {}
        displaced = []   # (index, entry) of other nets' tracks a bundle pushed aside
        pitch_all = bus_pitch(nets)
        for net, nb in bundle_order(nets):
            pa, pb = pad_of[(bus["from"], net)], pad_of[(bus["to"], net)]
            a_ = (TO(pa.GetPosition().x), TO(pa.GetPosition().y))
            b_ = (TO(pb.GetPosition().x), TO(pb.GetPosition().y))
            w = classes.width(net)
            lane_args = dict(attract=segs_of.get(nb), pitch=(w + need(net, nb)) if nb else 0.0)
            la_b, lb_b = [l for l in layers if pa.IsOnLayer(l)], [l for l in layers if pb.IsOnLayer(l)]
            path = follow(net, a_, b_, la_b, lb_b, w, lane_args["attract"], lane_args["pitch"],
                          reserve=math.ceil((len(nets) - 1) / 2) * pitch_all)
            pushed = False
            if not path and ns.max_rip > 0:
                # no free lane: the bundle may push unlocked tracks of other nets aside
                path = route(net, a_, b_, la_b, lb_b, w, 0.0, t_start + ns.time_budget, soft=True, **lane_args)
                pushed = path is not None
            if not path:
                say(f"rebundle {bus['from']}->{bus['to']}: {net} finds no lane")
                ok = False
                break
            first = len(items)
            made = insert(net, path, w)
            new += made
            for k in range(first, len(items)):   # the bundle itself must not be pushed aside
                if items[k] is not None:
                    items[k] = items[k][:4] + (None,)
            if pushed:
                for idx in victims_of(made, net):
                    entry = items[idx]
                    if entry is None or entry[0] in nets or entry[2] != "seg":
                        continue
                    b.Remove(entry[4])
                    displaced.append((idx, entry))
                    items[idx] = None
            segs_of[net] = [(o.GetLayer(), TO(o.GetStart().x), TO(o.GetStart().y), TO(o.GetEnd().x),
                             TO(o.GetEnd().y)) for o in made if o.GetClass() == "PCB_TRACK"]
            # the net's other pads (a pull-up, a second part) join the nearest pad already connected
            joined = [pa, pb]
            for extra_pad in [q for q in net_pads[net] if q.m_Uuid.AsString() not in
                              {pa.m_Uuid.AsString(), pb.m_Uuid.AsString()}]:
                (xa, la_), = [where(extra_pad)]
                target = min(joined, key=lambda q: math.hypot(where(q)[0][0] - xa[0], where(q)[0][1] - xa[1]))
                xt, lt = where(target)
                branch = route(net, xa, xt, la_, lt, w, 0.0, t_start + ns.time_budget)
                if not branch:
                    ok = False
                    break
                new += insert(net, branch, w)
                joined.append(extra_pad)
            if not ok:
                break
        if ok and displaced:
            # the pushed-aside nets are routed again between the loose ends their tracks left
            by_net = {}
            for _idx, entry in displaced:
                by_net.setdefault(entry[0], []).append((entry[4].GetLayer(),) + tuple(entry[3][:4]))
            for vnet, segs in by_net.items():
                width = next(e[3][4] * 2 for _i, e in displaced if e[0] == vnet)
                for chain in segment_chains(segs):
                    loose = cut_ends(chain)
                    if len(loose) != 2:
                        ok = False
                        break
                    (l1, x1, y1), (l2, x2, y2) = loose
                    again = route(vnet, (x1, y1), (x2, y2), [l1], [l2], width, 0.0, t_start + ns.time_budget)
                    if not again:
                        ok = False
                        break
                    new += insert(vnet, again, width)
                if not ok:
                    break
            say(f"rebundle {bus['from']}->{bus['to']}: pushed {len(displaced)} tracks of {len(by_net)} nets aside, "
                f"{'re-routed' if ok else 'could not re-route them'}")
        if ok:
            after, _len = bus_tools.parallelism(b, sub_bus)
            say(f"rebundle {bus['from']}->{bus['to']} ({len(nets)} nets): parallel {before:.2f} -> {after:.2f}")
            if after >= before + 0.1:
                now = open_connections()
                say(f"  open connections {baseline} -> {now}")
                if now <= baseline:
                    baseline = now
                    kept.append(f"{bus['from']}->{bus['to']} parallel {before:.2f}->{after:.2f}")
                    continue
        for o in new:
            b.Delete(o)
        for i in range(n_items, len(items)):
            items[i] = None
        del holes[n_holes:]
        for o in old:
            b.Add(o)
        for i, it in removed_items:
            items[i] = it
        for i, entry in displaced:
            b.Add(entry[4])
            items[i] = entry
    open_connections()
    return kept


if ns.rebundle:
    kept = rebundle(report)
    b.Save(ns.board)
    print(f"rebundle: {len(kept)} buses laid as bundles {kept}")
    sys.exit(0)

if ns.buses:
    lines = route_buses(report)
    b.Save(ns.board)
    print(f"buses: laid as bundles {lines}")
    sys.exit(0)

if ns.reduce_vias:
    kept = reduce_vias()
    b.Save(ns.board)
    print(f"reduce_vias: {len(kept)} nets with fewer vias {kept}")
    sys.exit(0)

def connect_pieces(zone):
    """KiCad reports a zone as unconnected to itself when pieces of its fill do not touch. Every
    piece that holds pads of the net but is not the main piece is tied to the main piece: a track
    from one of its pads to the nearest point of the main pour."""
    net = zone.GetNetname()
    made = 0
    baseline = open_connections()
    for layer in [l for l in zone.GetLayerSet().Seq() if l in layers]:
        fill = zone.GetFilledPolysList(layer)
        if fill.OutlineCount() < 2:
            continue
        main = max(range(fill.OutlineCount()), key=lambda k: fill.Outline(k).Area())
        for i in range(fill.OutlineCount()):
            if i == main:
                continue
            piece = pcbnew.SHAPE_POLY_SET()
            piece.AddOutline(fill.Outline(i))
            # copper of the net in this piece: pads, vias, track ends (start points for the tie)
            inside = [(p.GetPosition(), [l for l in layers if p.IsOnLayer(l)]) for f in b.GetFootprints()
                      for p in f.Pads() if p.GetNetname() == net and p.IsOnLayer(layer)]
            inside += [(t.GetPosition(), list(layers)) for t in b.GetTracks()
                       if t.GetClass() == "PCB_VIA" and t.GetNetname() == net]
            inside += [(pt, [layer]) for t in b.GetTracks() if t.GetClass() == "PCB_TRACK" and t.GetNetname() == net
                       and t.GetLayer() == layer for pt in (t.GetStart(), t.GetEnd())]
            inside = [(pos, ls) for pos, ls in inside if piece.Contains(pos)]
            for pos, start_layers in inside[:1]:
                a_ = (TO(pos.x), TO(pos.y))
                q, _l = zone_target(zone, a_, [layer])
                if q is None:
                    continue
                for w in sorted({classes.width(net), max(ns.min_width, 0.1)}, reverse=True):
                    path = route(net, a_, q, start_layers, [layer], w, 0.3, t_start + ns.time_budget)
                    if path:
                        n_items, n_holes = len(items), len(holes)
                        tie = insert(net, path, w)
                        now = open_connections()
                        if now < baseline:     # the piece really was cut off: keep the tie
                            baseline = now
                            made += 1
                        else:                   # already joined through tracks: no extra copper
                            for o in tie:
                                b.Delete(o)
                            for k in range(n_items, len(items)):
                                items[k] = None
                            del holes[n_holes:]
                        break
    return made


def plane_drop(net, ends, deadline):
    """Last resort for a net with a pour on an inner layer: each end gets its own short way down to
    that pour (a via next to it), the way people tie a decoupling capacitor to a supply plane when
    there is no room to join it on the surface. Kept only if KiCad counts fewer open connections."""
    planes = [(z, l) for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net
              for l in z.GetLayerSet().Seq() if l in layers and l not in (pcbnew.F_Cu, pcbnew.B_Cu)]
    if not planes:
        return 0
    before = open_connections()
    mark = len(journal)
    made = 0
    for pos, lay in ends:
        for zone, l in planes:
            target, tl = zone_target(zone, pos, [l])
            if target is None:
                continue
            widths = sorted({classes.width(net), max(ns.min_width, 0.1)}, reverse=True)
            for w in widths:
                p = route(net, pos, target, lay, [tl], w, goal_radius=0.3, deadline=deadline)
                if p:
                    insert(net, p, w)
                    made += 1
                    break
            else:
                # crowded around the end (a fine-pitch pin, a via cluster): make room
                if ns.max_rip > 0 and ripup(net, pos, target, lay, [tl], widths[-1], 0.3, deadline):
                    made += 1
                else:
                    continue
            break
    if not made or open_connections() >= before:
        rollback(mark)
        return 0
    return made


seen_pairs = set()
zones_joined = set()
for u in sorted(report.get("unconnected_items", []), key=span):
    kinds = [resolve(i) for i in u.get("items", [])[:2]]
    if len(kinds) == 2 and kinds[0][0] == kinds[1][0] == "zone" and kinds[0][4] is not None:
        zone = kinds[0][4]
        if zone.m_Uuid.AsString() not in zones_joined:
            zones_joined.add(zone.m_Uuid.AsString())
            pcbnew.ZONE_FILLER(b).Fill(b.Zones())
            n = connect_pieces(zone)
            (done if n else failed).append(f"{zone.GetNetname()} pour pieces joined: {n}")
        continue
    if u.get("thermal"):
        made = thermal_fix(u)
        (done if made else failed).append(f"thermal relief {made or 0} stubs")
        continue
    its = u.get("items", [])
    pair = tuple(sorted((round(i["pos"]["x"], 3), round(i["pos"]["y"], 3)) for i in its[:2]))
    if pair in seen_pairs:
        continue  # the same connection reported twice would be routed twice
    seen_pairs.add(pair)
    if len(its) < 2:
        continue
    ka, net, la, a, _ia = resolve(its[0])
    kc, net_c, lc, c, zone = resolve(its[1])
    if ka == "zone":  # keep the zone as the second end
        (ka, net, la, a, _ia), (kc, net_c, lc, c, zone) = (kc, net_c, lc, c, zone), (ka, net, la, a, _ia)
    net = net or net_c
    if not net or not la or not lc:
        failed.append(net or "?")
        continue
    goal_radius = 0.0
    if kc == "zone":
        if zone is not None:
            c, layer = zone_target(zone, a, lc)
            if c is None:
                failed.append(net)
                continue
            lc = [layer]
        goal_radius = 0.3   # anywhere in the pour right there
    say(f"{net}: {ka} {a} on {[b.GetLayerName(l) for l in la]} -> {kc} {c} on {[b.GetLayerName(l) for l in lc]}")
    if (ns.max_span and math.hypot(a[0] - c[0], a[1] - c[1]) > ns.max_span) or time.time() - t_start > ns.time_budget:
        skipped.append(net)
        continue
    path = None
    for w in sorted({classes.width(net), max(ns.min_width, 0.1)}, reverse=True):
        path = route(net, a, c, la, lc, w, goal_radius=goal_radius, deadline=t_start + ns.time_budget)
        if path:
            n = len(insert(net, path, w))
            done.append(f"{net} ({w} mm, {n} items)")
            break
    if not path:
        w = max(ns.min_width, 0.1)
        path = route(net, a, c, la, lc, w, goal_radius=goal_radius, deadline=t_start + ns.time_budget, fine=True)
        if path:
            n = len(insert(net, path, w))
            done.append(f"{net} ({w} mm, fine grid, {n} items)")
    if not path and ns.max_rip > 0:
        for w in sorted({classes.width(net), max(ns.min_width, 0.1)}, reverse=True):
            res = ripup(net, a, c, la, lc, w, goal_radius, t_start + ns.time_budget)
            if res:
                path = True
                done.append(f"{net} ({w} mm, ripped and re-routed {res[0]} tracks of {', '.join(res[1])})")
                break
    if not path and kc != "zone":
        n = plane_drop(net, [(a, la), (c, lc)], t_start + ns.time_budget)
        if n:
            path = True
            done.append(f"{net} (both ends tied to its inner-layer pour, {n} ways down)")
    if not path:
        failed.append(net)
b.Save(ns.board)
print(f"lastmile: routed {done}; failed {failed}" + (f"; skipped (too long / out of time) {skipped}" if skipped else ""))
