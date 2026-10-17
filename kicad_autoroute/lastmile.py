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
import heapq
import json
import math
import os
import sys
import time

import numpy as np
import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kicad_classes import NetClasses, ViaRules  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("board")
ap.add_argument("drc")
ap.add_argument("--window", type=float, default=5.0)
ap.add_argument("--grid", type=float, default=0.05)
ap.add_argument("--min-width", type=float, default=0.15)
ap.add_argument("--via", type=float, nargs=2, default=None, help="default: 0.6/0.3 raised to the board's via rules")
ap.add_argument("--edge", type=float, default=0.5)
ap.add_argument("--via-cost", type=float, default=3.0, help="via cost in mm of trace")
ap.add_argument("--hole-clearance", type=float, default=0.25, help="min gap between drill holes")
ap.add_argument("--max-span", type=float, default=0.0, help="skip connections longer than this (mm), 0 = no limit")
ap.add_argument("--max-expansions", type=int, default=400000, help="search nodes per connection before giving up")
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
report = json.load(open(ns.drc, encoding="utf-8"))


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

# copper shapes as (net, layers, kind, geometry) in mm
items = []
for f in b.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox()
        pl = [l for l in layers if p.IsOnLayer(l)]
        items.append((p.GetNetname(), pl, "box", (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom()))))
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
        items.append((net, [d.GetLayer()], "box", (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom()))))
for t in b.GetTracks():
    if t.GetClass() == "PCB_VIA":
        r = TO(t.GetWidth(pcbnew.F_Cu)) / 2
        items.append((t.GetNetname(), layers, "disc", (TO(t.GetPosition().x), TO(t.GetPosition().y), r)))
    else:
        items.append((t.GetNetname(), [t.GetLayer()], "seg",
                      (TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y), TO(t.GetWidth()) / 2)))


def blocked_map(net, x0, y0, nx, ny, g, half_w):
    """Per layer boolean grid: True where a trace centre of half width half_w would violate clearance."""
    xs = x0 + np.arange(nx) * g
    ys = y0 + np.arange(ny) * g
    X, Y = np.meshgrid(xs, ys)  # [ny, nx]
    maps = {l: np.zeros((ny, nx), bool) for l in layers}
    for inet, ilayers, kind, geo in items:
        if inet == net:
            continue
        # one grid step of margin: paths run between cell centres and diagonals cut corners
        m = need(net, inet) + half_w + g
        if kind == "box":
            bx0, by0, bx1, by1 = geo
            if bx1 + m < xs[0] or bx0 - m > xs[-1] or by1 + m < ys[0] or by0 - m > ys[-1]:
                continue
            d = np.hypot(np.maximum(np.maximum(bx0 - X, 0), X - bx1), np.maximum(np.maximum(by0 - Y, 0), Y - by1))
            hit = d < m
        elif kind == "disc":
            cx, cy, r = geo
            if cx + r + m < xs[0] or cx - r - m > xs[-1] or cy + r + m < ys[0] or cy - r - m > ys[-1]:
                continue
            hit = np.hypot(X - cx, Y - cy) < r + m
        else:
            sx, sy, ex, ey, hw = geo
            if max(sx, ex) + hw + m < xs[0] or min(sx, ex) - hw - m > xs[-1] or \
               max(sy, ey) + hw + m < ys[0] or min(sy, ey) - hw - m > ys[-1]:
                continue
            dx, dy = ex - sx, ey - sy
            L = dx * dx + dy * dy
            t = np.clip(((X - sx) * dx + (Y - sy) * dy) / L, 0, 1) if L > 0 else 0
            hit = np.hypot(X - (sx + t * dx), Y - (sy + t * dy)) < hw + m
        for l in ilayers:
            if l in maps:
                maps[l] |= hit
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


def route(net, a, b_pt, la, lb, width, goal_radius=0.0, deadline=float("inf")):
    # long connections search on a coarser grid (at most MAX_CELLS per layer); the clearance margin
    # grows with the grid step, so a coarse path is still legal
    w_mm = abs(a[0] - b_pt[0]) + 2 * ns.window
    h_mm = abs(a[1] - b_pt[1]) + 2 * ns.window
    g = min(0.2, max(ns.grid, math.sqrt(w_mm * h_mm / MAX_CELLS)))
    x0 = min(a[0], b_pt[0]) - ns.window
    y0 = min(a[1], b_pt[1]) - ns.window
    nx = int((max(a[0], b_pt[0]) + ns.window - x0) / g) + 1
    ny = int((max(a[1], b_pt[1]) + ns.window - y0) / g) + 1
    tmaps = blocked_map(net, x0, y0, nx, ny, g, width / 2)
    vr = ns.via[0] / 2
    vmaps = blocked_map(net, x0, y0, nx, ny, g, vr)
    via_ok = ~np.any(np.stack([vmaps[l] for l in layers]), axis=0)
    xs = x0 + np.arange(nx) * g
    ys = y0 + np.arange(ny) * g
    X, Y = np.meshgrid(xs, ys)
    for hx, hy, hr in holes:
        via_ok &= np.hypot(X - hx, Y - hy) >= hr + ns.via[1] / 2 + ns.hole_clearance
    li = {l: i for i, l in enumerate(layers)}

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
    dirs = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    via_cost = ns.via_cost / g
    start = [(sc[0], sc[1], li[l]) for l in la]
    dist = {s: 0.0 for s in start}
    prev = {}
    heap = [(math.hypot(sc[0] - gc[0], sc[1] - gc[1]), 0.0, s) for s in start]
    heapq.heapify(heap)
    found = None
    expanded = 0
    while heap:
        expanded += 1
        if expanded > ns.max_expansions or (expanded % 5000 == 0 and time.time() > deadline):
            break
        f, d, node = heapq.heappop(heap)
        if node in goal:
            found = node
            break
        if d > dist.get(node, 1e18):
            continue
        x, y, k = node
        nbrs = []
        for dx, dy, c in dirs:
            nx_, ny_ = x + dx, y + dy
            if 0 <= nx_ < nx and 0 <= ny_ < ny and not tmaps[layers[k]][ny_, nx_]:
                nbrs.append(((nx_, ny_, k), c))
        if via_ok[y, x]:
            for k2 in range(len(layers)):
                if k2 != k and not tmaps[layers[k2]][y, x]:
                    nbrs.append(((x, y, k2), via_cost))
        for nb, c in nbrs:
            nd = d + c
            if nd < dist.get(nb, 1e18):
                dist[nb] = nd
                prev[nb] = node
                heapq.heappush(heap, (nd + math.hypot(nb[0] - gc[0], nb[1] - gc[1]), nd, nb))
    if not found:
        return None
    path = [found]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()
    # the path starts and ends on cell centres; pin the ends to the exact pad positions
    pts = [(x0 + x * g, y0 + y * g, layers[k]) for x, y, k in path]
    pts[0] = (a[0], a[1], pts[0][2])
    if goal_radius == 0:
        pts[-1] = (b_pt[0], b_pt[1], pts[-1][2])
    return pts


def insert(net_name, path, width):
    net = b.FindNet(net_name)
    added = 0
    # simplify: keep points where direction or layer changes
    pts = [path[0]]
    for i in range(1, len(path) - 1):
        p0, p1, p2 = pts[-1], path[i], path[i + 1]
        if p1[2] != p0[2] or p2[2] != p1[2]:
            pts.append(p1)
            continue
        d1 = (round((p1[0] - p0[0]) / ns.grid), round((p1[1] - p0[1]) / ns.grid))
        d2 = (round((p2[0] - p1[0]) / ns.grid), round((p2[1] - p1[1]) / ns.grid))
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
            items.append((net_name, layers, "disc", (p[0], p[1], ns.via[0] / 2)))
            added += 1
            continue
        if (p[0], p[1]) == (q[0], q[1]):
            continue
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pcbnew.VECTOR2I(FROM(p[0]), FROM(p[1])))
        t.SetEnd(pcbnew.VECTOR2I(FROM(q[0]), FROM(q[1])))
        t.SetWidth(FROM(width)); t.SetLayer(p[2]); t.SetNet(net)
        b.Add(t)
        items.append((net_name, [p[2]], "seg", (p[0], p[1], q[0], q[1], width / 2)))
        added += 1
    return added


import re
t_start = time.time()
done, failed, skipped = [], [], []
def span(u):
    its = u.get("items", [])
    if len(its) < 2:
        return 0.0
    return math.hypot(its[0]["pos"]["x"] - its[1]["pos"]["x"], its[0]["pos"]["y"] - its[1]["pos"]["y"])


for u in sorted(report.get("unconnected_items", []), key=span):
    its = u.get("items", [])
    if len(its) < 2:
        continue
    m = re.search(r"\[([^\]]+)\]", its[0]["description"])
    if not m:
        continue
    net = m.group(1)
    a = (its[0]["pos"]["x"], its[0]["pos"]["y"])
    c = (its[1]["pos"]["x"], its[1]["pos"]["y"])
    la, lc = endpoint_layers(its[0]["description"]), endpoint_layers(its[1]["description"])
    if (ns.max_span and math.hypot(a[0] - c[0], a[1] - c[1]) > ns.max_span) or time.time() - t_start > ns.time_budget:
        skipped.append(net)
        continue
    path = None
    to_plane = its[1]["description"].startswith("Zone") and lc not in ([pcbnew.F_Cu], [pcbnew.B_Cu])
    for w in sorted({classes.width(net), max(ns.min_width, 0.1)}, reverse=True):
        path = route(net, a, c, la, lc, w, goal_radius=1.5 if to_plane else 0.0, deadline=t_start + ns.time_budget)
        if path:
            n = insert(net, path, w)
            done.append(f"{net} ({w} mm, {n} items)")
            break
    if not path:
        failed.append(net)
b.Save(ns.board)
print(f"lastmile: routed {done}; failed {failed}" + (f"; skipped (too long / out of time) {skipped}" if skipped else ""))
