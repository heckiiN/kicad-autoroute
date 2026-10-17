"""Stitch isolated pieces of plane-net pours to the rest of the net (run with KiCad's python).

  <kicad-python> kicad_autoroute/zone_stitch.py board.kicad_pcb [--net GND] [--via 0.6 0.3]

After the zones are filled, every filled polygon of the net on every layer is checked: a piece that
contains no via and no through-hole pad of the net only touches copper on its own layer, so it is
usually cut off (a pour boxed in by traces). Such a piece gets a via at a spot inside it, at least
a via radius away from its border, that keeps the required clearance to all other-net copper on
every layer and to the board edge. The via ties it to the pours on the other layers.
"""
import argparse
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kicad_classes import NetClasses, ViaRules  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("board")
ap.add_argument("--net", default="GND")
ap.add_argument("--via", type=float, nargs=2, default=None, help="default: 0.6/0.3 raised to the board's via rules")
ap.add_argument("--small-via", type=float, nargs=2, default=None, help="fallback for small islands (default 0.45/0.2, raised to the rules)")
ap.add_argument("--edge", type=float, default=0.55, help="copper-to-edge clearance + margin (mm)")
ap.add_argument("--step", type=float, default=0.2)
ap.add_argument("--no-rip", dest="rip", action="store_false",
                help="never remove unlocked tracks to make room for an island via")
ns = ap.parse_args()
VIA_RULES = ViaRules.for_board(ns.board)
ns.via = VIA_RULES.legal(*(ns.via or (0.6, 0.3)))
ns.small_via = VIA_RULES.legal(*(ns.small_via or (0.45, 0.2)))

TO, FROM = pcbnew.ToMM, pcbnew.FromMM
b = pcbnew.LoadBoard(ns.board)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
net_item = b.FindNet(ns.net)
classes = NetClasses.for_board(ns.board)
extra = NetClasses.dru_net_clearances(ns.board)
vr = ns.via[0] / 2
VIA_D, DRILL = ns.via
ebb = b.GetBoardEdgesBoundingBox()
EX0, EY0, EX1, EY1 = TO(ebb.GetLeft()), TO(ebb.GetTop()), TO(ebb.GetRight()), TO(ebb.GetBottom())


def need(other):
    return max(classes.clearance(ns.net), classes.clearance(other), extra.get(other, 0.0))


# other-net copper (all layers count for a through via)
obst = []
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetNetname() != ns.net:
            bb = p.GetBoundingBox()
            obst.append(("box", p.GetNetname(), (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())), None))
# copper text and graphics without a net (board level and inside footprints)
copper_art = list(b.GetDrawings())
for f in b.GetFootprints():
    copper_art += list(f.GraphicalItems()) + [f.Reference(), f.Value()]
for d in copper_art:
    if d.GetClass() == "PCB_FIELD" and not d.IsVisible():
        continue
    if pcbnew.IsCopperLayer(d.GetLayer()) and (not hasattr(d, "GetNetname") or d.GetNetname() != ns.net):
        bb = d.GetBoundingBox()
        obst.append(("box", "", (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())), None))
holes = []
rippable_holes = {}  # (x, y) -> uuid of an unlocked other-net via
for t in b.GetTracks():
    if t.GetClass() == "PCB_VIA":
        holes.append((TO(t.GetPosition().x), TO(t.GetPosition().y), TO(t.GetDrillValue()) / 2))
        if t.GetNetname() != ns.net and not t.IsLocked():
            rippable_holes[(round(TO(t.GetPosition().x), 4), round(TO(t.GetPosition().y), 4))] = t.m_Uuid.AsString()
        if t.GetNetname() != ns.net:
            obst.append(("disc", t.GetNetname(), (TO(t.GetPosition().x), TO(t.GetPosition().y), TO(t.GetWidth(pcbnew.F_Cu)) / 2),
                         t.m_Uuid.AsString() if not t.IsLocked() else None))
    elif t.GetNetname() != ns.net:
        obst.append(("seg", t.GetNetname(), (TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y),
                                             TO(t.GetWidth()) / 2), t.m_Uuid.AsString() if not t.IsLocked() else None))
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetDrillSize().x > 0:
            holes.append((TO(p.GetPosition().x), TO(p.GetPosition().y), TO(p.GetDrillSize().x) / 2))


def conflicts(x, y, ignore_unlocked):
    """Obstacles a via at (x, y) would violate; with ignore_unlocked, unlocked tracks are returned
    separately instead of blocking (they can be ripped and re-routed)."""
    rip = []
    for kind, net, g, item in obst:
        m = vr + need(net)
        if kind == "box":
            d = math.hypot(max(g[0] - x, 0, x - g[2]), max(g[1] - y, 0, y - g[3]))
        elif kind == "disc":
            d = math.hypot(x - g[0], y - g[1]) - g[2]
        else:
            sx, sy, ex, ey, hw = g
            dx, dy = ex - sx, ey - sy
            L = dx * dx + dy * dy
            t = 0 if L == 0 else max(0, min(1, ((x - sx) * dx + (y - sy) * dy) / L))
            d = math.hypot(x - sx - t * dx, y - sy - t * dy) - hw
        if d < m:
            if ignore_unlocked and item is not None:
                rip.append(item)
            else:
                return None
    return rip


def free(x, y):
    if min(x - EX0, EX1 - x, y - EY0, EY1 - y) < vr + ns.edge:
        return False
    for hx, hy, hr in holes:
        if math.hypot(x - hx, y - hy) < hr + DRILL / 2 + 0.25:
            return False
    for kind, net, g, _item in obst:
        m = vr + need(net)
        if kind == "box":
            d = math.hypot(max(g[0] - x, 0, x - g[2]), max(g[1] - y, 0, y - g[3]))
        elif kind == "disc":
            d = math.hypot(x - g[0], y - g[1]) - g[2]
        else:
            sx, sy, ex, ey, hw = g
            dx, dy = ex - sx, ey - sy
            L = dx * dx + dy * dy
            t = 0 if L == 0 else max(0, min(1, ((x - sx) * dx + (y - sy) * dy) / L))
            d = math.hypot(x - sx - t * dx, y - sy - t * dy) - hw
        if d < m:
            return False
    return True


# where the net already reaches other layers: its vias and drilled pads
anchors = [(TO(t.GetPosition().x), TO(t.GetPosition().y)) for t in b.GetTracks()
           if t.GetClass() == "PCB_VIA" and t.GetNetname() == ns.net]
anchors += [(TO(p.GetPosition().x), TO(p.GetPosition().y)) for f in b.GetFootprints() for p in f.Pads()
            if p.GetNetname() == ns.net and p.GetDrillSize().x > 0]

# the inner layer that carries a pour of the net: islands that have no room for a via of their own are
# tied to it by the last-mile router (pad -> short trace -> via -> plane)
plane_layer, plane_zone = None, None
best_area = 0
for z in b.Zones():
    if z.GetNetname() == ns.net and not z.GetIsRuleArea():
        for layer in z.GetLayerSet().Seq():
            if layer not in (pcbnew.F_Cu, pcbnew.B_Cu):
                area = z.GetFilledPolysList(layer).Area()
                if area > best_area:
                    plane_layer, plane_zone, best_area = layer, z, area
# filled copper of the net per layer: an island via only helps where it lands in the net's fill on
# another layer (a 2-layer board with a single GND pour has nothing to tie an island to)
fills = {}
for z in b.Zones():
    if z.GetNetname() == ns.net and not z.GetIsRuleArea():
        for layer in z.GetLayerSet().Seq():
            fills.setdefault(layer, []).append(z.GetFilledPolysList(layer))


def reaches_other(x, y, layer):
    v = pcbnew.VECTOR2I(FROM(x), FROM(y))
    return any(l != layer and any(p.Contains(v) for p in polys) for l, polys in fills.items())


requests = []
added, stuck, ripped = 0, 0, 0
for z in b.Zones():
    if z.GetNetname() != ns.net or z.GetIsRuleArea():
        continue
    for layer in z.GetLayerSet().Seq():
        if not any(l != layer for l in fills):
            continue  # the net has no fill on another layer: an island via could not reach anything
        polys = z.GetFilledPolysList(layer)
        for i in range(polys.OutlineCount()):
            piece = pcbnew.SHAPE_POLY_SET()
            piece.AddOutline(polys.Outline(i))
            for h in range(polys.HoleCount(i)):
                piece.AddHole(polys.Hole(i, h))
            if any(piece.Contains(pcbnew.VECTOR2I(FROM(ax), FROM(ay))) for ax, ay in anchors):
                continue
            bb = piece.BBox()
            x0, y0, x1, y1 = TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())
            spot = None
            for VIA_D, DRILL in (tuple(ns.via), tuple(ns.small_via)):
                vr = VIA_D / 2
                if (x1 - x0) < 2 * vr or (y1 - y0) < 2 * vr:
                    continue  # sliver, too small for this via
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                pts = []
                y = y0
                while y <= y1:
                    x = x0
                    while x <= x1:
                        pts.append((x, y))
                        x += ns.step
                    y += ns.step
                pts.sort(key=lambda p: math.hypot(p[0] - cx, p[1] - cy))
                for x, y in pts:
                    v = pcbnew.VECTOR2I(FROM(x), FROM(y))
                    if not piece.Contains(v):
                        continue
                    inner = all(piece.Contains(pcbnew.VECTOR2I(FROM(x + vr * math.cos(a)), FROM(y + vr * math.sin(a))))
                                for a in (0, math.pi / 2, math.pi, 3 * math.pi / 2))
                    if inner and reaches_other(x, y, layer) and free(x, y):
                        spot = (x, y)
                        break
                if spot is None:
                    # make room: a spot that only unlocked other-net tracks block; rip those tracks
                    best_rip = None
                    for x, y in pts:
                        v = pcbnew.VECTOR2I(FROM(x), FROM(y))
                        if not piece.Contains(v):
                            continue
                        if min(x - EX0, EX1 - x, y - EY0, EY1 - y) < vr + ns.edge:
                            continue
                        if any(math.hypot(x - hx, y - hy) < hr + DRILL / 2 + 0.25
                               and (round(hx, 4), round(hy, 4)) not in rippable_holes for hx, hy, hr in holes):
                            continue
                        inner = all(piece.Contains(pcbnew.VECTOR2I(FROM(x + vr * math.cos(a)), FROM(y + vr * math.sin(a))))
                                    for a in (0, math.pi / 2, math.pi, 3 * math.pi / 2))
                        if not inner or not reaches_other(x, y, layer):
                            continue
                        rip = conflicts(x, y, True)
                        if rip is not None and (best_rip is None or len(rip) < len(best_rip[2])):
                            best_rip = (x, y, rip)
                            if len(rip) <= 1:
                                break
                    if best_rip is not None and ns.rip:
                        x, y, rip = best_rip
                        seen = set(rip)
                        # vias whose hole is too close go as well
                        for (hx, hy), uid in rippable_holes.items():
                            if math.hypot(x - hx, y - hy) < 0.6 + DRILL / 2 + 0.25:
                                seen.add(uid)
                        for t in list(b.GetTracks()):
                            if t.m_Uuid.AsString() in seen:
                                b.Delete(t)
                        holes[:] = [h for h in holes if rippable_holes.get((round(h[0], 4), round(h[1], 4))) not in seen]
                        obst[:] = [o for o in obst if o[3] is None or o[3] not in seen]
                        spot = (x, y)
                        ripped += len(seen)
                if spot is not None:
                    break
            if spot is None:
                stuck += 1
                if plane_layer is not None:
                    for f in b.GetFootprints():
                        for p in f.Pads():
                            if p.GetNetname() == ns.net and p.IsOnLayer(layer) and piece.Contains(p.GetPosition()):
                                px, py = TO(p.GetPosition().x), TO(p.GetPosition().y)
                                requests.append({"items": [
                                    {"description": f"Pad {p.GetNumber()} [{ns.net}] von {f.GetReference()} auf {b.GetLayerName(layer)}",
                                     "pos": {"x": px, "y": py}, "uuid": p.m_Uuid.AsString()},
                                    {"description": f"Zone [{ns.net}] auf {b.GetLayerName(plane_layer)}",
                                     "pos": {"x": px, "y": py}, "uuid": plane_zone.m_Uuid.AsString()}]})
                                break
                        else:
                            continue
                        break
                continue
            via = pcbnew.PCB_VIA(b)
            via.SetPosition(pcbnew.VECTOR2I(FROM(spot[0]), FROM(spot[1])))
            via.SetWidth(FROM(VIA_D)); via.SetDrill(FROM(DRILL)); via.SetNet(net_item)
            b.Add(via)
            anchors.append(spot)
            holes.append((spot[0], spot[1], DRILL / 2))
            added += 1
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(ns.board)
import json
req_path = os.path.splitext(ns.board)[0] + "_islands.json"
json.dump({"unconnected_items": requests}, open(req_path, "w", encoding="utf-8"), indent=1)
print(f"zone_stitch: {added} island vias added ({ripped} tracks ripped for room), {stuck} islands without room, "
      f"{len(requests)} pad-to-plane "
      f"requests -> {req_path}")
