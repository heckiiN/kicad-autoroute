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
import re
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
ap.add_argument("--bend-cost", type=float, default=0.5, help="cost of each 45 degree turn in mm of trace")
ap.add_argument("--hole-clearance", type=float, default=0.25, help="min gap between drill holes")
ap.add_argument("--max-span", type=float, default=0.0, help="skip connections longer than this (mm), 0 = no limit")
ap.add_argument("--max-expansions", type=int, default=400000, help="search nodes per connection before giving up")
ap.add_argument("-v", "--verbose", action="store_true", help="explain failed connections")
ap.add_argument("--max-rip", type=int, default=6, help="tracks a blocked connection may rip up (0: no rip-up)")
ap.add_argument("--max-rip-nets", type=int, default=2, help="nets whose tracks a rip-up may move")
ap.add_argument("--rip-cost", type=float, default=1.0, help="cost of crossing a rippable track, mm of trace per grid cell")
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

# copper shapes as (net, layers, kind, geometry, ref) in mm; ref is the board track for unlocked
# tracks (these may be ripped up and re-routed), None for everything that stays
items = []
for f in b.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox()
        pl = [l for l in layers if p.IsOnLayer(l)]
        items.append((p.GetNetname(), pl, "box", (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())), None))
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
for t in b.GetTracks():
    if t.GetClass() == "PCB_VIA":
        r = TO(t.GetWidth(pcbnew.F_Cu)) / 2
        items.append((t.GetNetname(), layers, "disc", (TO(t.GetPosition().x), TO(t.GetPosition().y), r), None))
    else:
        items.append((t.GetNetname(), [t.GetLayer()], "seg",
                      (TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y), TO(t.GetWidth()) / 2),
                      t if t.GetClass() == "PCB_TRACK" and not t.IsLocked() else None))


def blocked_map(net, x0, y0, nx, ny, g, half_w, which="all"):
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
        m = need(net, inet) + half_w + 0.75 * g
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


def route(net, a, b_pt, la, lb, width, goal_radius=0.0, deadline=float("inf"), soft=False, fine=False):
    """Grid A* from a to b_pt. With soft=True unlocked tracks of other nets are passable at a cost
    (rip-up search); otherwise they block like everything else."""
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
    rip_cell = ns.rip_cost / g
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

    start = [(sc[0], sc[1], li[l], NONE) for l in la]
    dist = {s: 0.0 for s in start}
    prev = {}
    heap = [(h(sc[0], sc[1]), 0.0, s) for s in start]
    heapq.heapify(heap)
    found = None
    expanded = 0
    while heap:
        expanded += 1
        if expanded > ns.max_expansions or (expanded % 5000 == 0 and time.time() > deadline):
            break
        f, d, node = heapq.heappop(heap)
        x, y, k, last = node
        if (x, y, k) in goal:
            found = node
            break
        if d > dist.get(node, 1e18):
            continue
        nbrs = []
        for i, (dx, dy, c) in enumerate(dirs):
            if last != NONE:
                turn = min((i - last) % 8, (last - i) % 8)
                if turn > 2:
                    continue        # no acute or reversing corners
                c += turn * turn_cost
            nx_, ny_ = x + dx, y + dy
            if 0 <= nx_ < nx and 0 <= ny_ < ny and not tmaps[layers[k]][ny_, nx_]:
                if smaps is not None and smaps[layers[k]][ny_, nx_]:
                    c += rip_cell
                nbrs.append(((nx_, ny_, k, i), c))
        if via_ok[y, x]:
            for k2 in range(len(layers)):
                if k2 != k and not tmaps[layers[k2]][y, x] and not (smaps is not None and smaps[layers[k2]][y, x]):
                    nbrs.append(((x, y, k2, NONE), via_cost))
        for nb, c in nbrs:
            nd = d + c
            if nd < dist.get(nb, 1e18):
                dist[nb] = nd
                prev[nb] = node
                heapq.heappush(heap, (nd + h(nb[0], nb[1]), nd, nb))
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
            items.append((net_name, layers, "disc", (p[0], p[1], ns.via[0] / 2), None))
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
        if ref is None or inet == net or kind != "seg":
            continue
        clr = need(net, inet) - 1e-4
        for layer, g_, r in shapes:
            if layer is None:
                d = _pt_seg(g_[0], g_[1], *geo[:4]) - r - geo[4]
            elif layer in ilayers:
                d = _seg_seg(g_, geo[:4]) - r - geo[4]
            else:
                continue
            if d < clr:
                found.append(idx)
                break
    return found


def cut_ends(segs):
    """Loose ends of a set of ripped track segments [(layer, x1, y1, x2, y2)]: the points where the
    rest of the net's copper continues."""
    count = {}
    for layer, x1, y1, x2, y2 in segs:
        for q in ((layer, round(x1, 4), round(y1, 4)), (layer, round(x2, 4), round(y2, 4))):
            count[q] = count.get(q, 0) + 1
    return [q for q, n in count.items() if n == 1]


def ripup(net, a, c, la, lc, width, goal_radius, deadline):
    """Route a blocked connection through unlocked tracks of other nets, rip those tracks and route
    them again around the new one. Everything is undone unless all of it succeeds."""
    path = route(net, a, c, la, lc, width, goal_radius, deadline, soft=True)
    if not path:
        say(f"rip-up {net}: no path even through rippable tracks")
        return None
    n_items, n_holes = len(items), len(holes)
    created = insert(net, path, width)
    victims = victims_of(created, net)
    removed, rerouted = [], []

    def undo():
        for o in created + rerouted:
            b.Delete(o)
        for i in range(n_items, len(items)):
            items[i] = None
        del holes[n_holes:]
        for idx, entry in removed:
            b.Add(entry[4])
            items[idx] = entry
        return None

    if not victims or len(victims) > ns.max_rip:
        say(f"rip-up {net}: {len(victims)} tracks in the way (limit {ns.max_rip})")
        return undo()
    by_net = {}
    for idx in victims:
        entry = items[idx]
        by_net.setdefault(entry[0], []).append((entry[4].GetLayer(),) + tuple(entry[3][:4]))
        b.Remove(entry[4])
        removed.append((idx, entry))
        items[idx] = None
    widths = {e[0]: e[3][4] * 2 for _i, e in removed}
    if len(by_net) > ns.max_rip_nets:
        say(f"rip-up {net}: {len(by_net)} nets in the way (limit {ns.max_rip_nets})")
        return undo()
    for vnet, segs in by_net.items():
        ends = cut_ends(segs)
        if len(ends) != 2:
            say(f"rip-up {net}: ripped tracks of {vnet} leave {len(ends)} loose ends")
            return undo()
        (l1, x1, y1), (l2, x2, y2) = ends
        p = route(vnet, (x1, y1), (x2, y2), [l1], [l2], widths[vnet], 0.0, deadline)
        if not p:
            say(f"rip-up {net}: {vnet} could not be routed again")
            return undo()
        rerouted += insert(vnet, p, widths[vnet])
    return len(victims), sorted(by_net)


t_start = time.time()
done, failed, skipped = [], [], []
def span(u):
    its = u.get("items", [])
    if len(its) < 2:
        return 0.0
    return math.hypot(its[0]["pos"]["x"] - its[1]["pos"]["x"], its[0]["pos"]["y"] - its[1]["pos"]["y"])


seen_pairs = set()
for u in sorted(report.get("unconnected_items", []), key=span):
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
    if not path:
        failed.append(net)
b.Save(ns.board)
print(f"lastmile: routed {done}; failed {failed}" + (f"; skipped (too long / out of time) {skipped}" if skipped else ""))
