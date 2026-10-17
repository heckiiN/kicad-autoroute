"""Automatic pad escapes for wide nets on fine-pitch pads (run with KiCad's python).

  <kicad-python> kicad_autoroute/escape.py board.kicad_pcb [--min-class-width 0.5]

A net class that asks for wide copper (charge / power, 0.5 mm and more) cannot leave a 0.2 mm pad at
0.4 mm pitch: the autorouter either necks the trace below the rules or gives up. Hand-routed boards
solve it with a short narrow stub straight out of the pad to a via, and the wide trace starts at the
via. This script does that for every SMD pad whose narrow side is smaller than its net-class width:

  pad -> stub (pad width, at least --min-stub) outward from the footprint -> via (0.8/0.4 or 0.6/0.3,
  raised to the board's via rules)

Pads of the same net next to each other share one via. Candidate via spots are searched outward and
sideways; a spot is used only if the via and the stub keep the class clearance to every other-net pad,
track and via and the copper-to-edge clearance. Stubs and vias are locked, so Freerouting keeps them.
"""
import argparse
import math
import sys

import pcbnew

ap = argparse.ArgumentParser()
ap.add_argument("board")
ap.add_argument("--min-class-width", type=float, default=0.5)
ap.add_argument("--min-stub", type=float, default=0.2)
ap.add_argument("--edge", type=float, default=0.5)
ap.add_argument("--hole-clearance", type=float, default=0.25, help="min gap between drill holes")
ap.add_argument("--refs", default="", help="only treat these comma-separated references (local repair)")
ns = ap.parse_args()

b = pcbnew.LoadBoard(ns.board)
TO, FROM = pcbnew.ToMM, pcbnew.FromMM
ebb = b.GetBoardEdgesBoundingBox()
EX0, EY0, EX1, EY1 = TO(ebb.GetLeft()), TO(ebb.GetTop()), TO(ebb.GetRight()), TO(ebb.GetBottom())


sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from kicad_classes import NetClasses, ViaRules  # noqa: E402

NETCLASSES = NetClasses.for_board(ns.board)
VIA_RULES = ViaRules.for_board(ns.board)
HV_CLEARANCE = NetClasses.dru_net_clearances(ns.board)


def class_of(item):
    net = item.GetNetname()
    return NETCLASSES.width(net), NETCLASSES.clearance(net)


# obstacles: (net, x0, y0, x1, y1) boxes in mm, refreshed as we add copper
obstacles = []
for fp in b.GetFootprints():
    for pad in fp.Pads():
        bb = pad.GetBoundingBox()
        obstacles.append((pad.GetNetname(), TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())))
for t in b.GetTracks():
    bb = t.GetBoundingBox()
    obstacles.append((t.GetNetname(), TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())))


def box_dist(px, py, x0, y0, x1, y1):
    dx = max(x0 - px, 0, px - x1)
    dy = max(y0 - py, 0, py - y1)
    return math.hypot(dx, dy)


def seg_box_dist(ax, ay, bx, by, box):
    # sample the segment; boxes are small, 0.05 mm steps are exact enough for a clearance check
    n = max(2, int(math.hypot(bx - ax, by - ay) / 0.05))
    return min(box_dist(ax + (bx - ax) * i / n, ay + (by - ay) * i / n, *box) for i in range(n + 1))


DEBUG = __import__("os").environ.get("ESCAPE_DEBUG")
blockers = {}


def clear(net, pts, half_w, via_r, via_pt, clr, own_pad_box):
    vx, vy = via_pt
    if min(vx - EX0, EX1 - vx, vy - EY0, EY1 - vy) < via_r + ns.edge:
        blockers["edge"] = blockers.get("edge", 0) + 1
        return False
    for onet, *box in obstacles:
        if onet == net:
            continue
        clr_o = max(clr, NETCLASSES.clearance(onet), HV_CLEARANCE.get(onet, 0.0))
        if box_dist(vx, vy, *box) < via_r + clr_o - 1e-3:
            blockers[f"via~{onet}"] = blockers.get(f"via~{onet}", 0) + 1
            return False
        for (ax, ay), (bx2, by2) in zip(pts, pts[1:]):
            if seg_box_dist(ax, ay, bx2, by2, box) < half_w + clr_o - 1e-3:
                blockers[f"stub~{onet}"] = blockers.get(f"stub~{onet}", 0) + 1
                return False
    return True


made_vias = {}  # net -> list of (x, y)
holes = []  # (x, y, drill radius) of every via and drilled pad, any net
for _f in b.GetFootprints():
    for _p in _f.Pads():
        if _p.GetDrillSize().x > 0:
            holes.append((TO(_p.GetPosition().x), TO(_p.GetPosition().y), TO(_p.GetDrillSize().x) / 2))
for _t in b.GetTracks():
    if _t.GetClass() == "PCB_VIA":
        holes.append((TO(_t.GetPosition().x), TO(_t.GetPosition().y), TO(_t.GetDrillValue()) / 2))
stubs = vias = skipped = 0
report = []
ONLY = {r for r in ns.refs.split(",") if r}
for fp in b.GetFootprints():
    if ONLY and fp.GetReference() not in ONLY:
        continue
    cx, cy = TO(fp.GetPosition().x), TO(fp.GetPosition().y)
    for pad in fp.Pads():
        net = pad.GetNetname()
        if not net or pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
            continue
        width, clr = class_of(pad)
        if width < ns.min_class_width:
            continue
        bb = pad.GetBoundingBox()
        pw, ph = TO(bb.GetWidth()), TO(bb.GetHeight())
        if min(pw, ph) >= width:
            continue  # the class width fits on the pad: the router can leave it directly
        px, py = TO(pad.GetPosition().x), TO(pad.GetPosition().y)
        # outward = the pad's long axis, pointing away from the footprint centre
        if ph >= pw:
            dirs = [(0, 1 if py >= cy else -1)]
        else:
            dirs = [(1 if px >= cx else -1, 0)]
        dirs += [(-d[0], -d[1]) for d in dirs] + [(d[1], d[0]) for d in dirs] + [(-d[1], -d[0]) for d in dirs]
        r2 = math.sqrt(0.5)
        dirs += [(r2, r2), (r2, -r2), (-r2, r2), (-r2, -r2)]  # diagonal escapes as a last resort
        stub_w = max(ns.min_stub, min(pw, ph, 0.3))
        own_box = (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom()))
        reach = max(pw, ph) / 2

        # reuse a via of this net that is already close
        near = [v for v in made_vias.get(net, []) if math.dist(v, (px, py)) < reach + 1.6]
        done = False
        for vx, vy in near:
            pts = [(px, py), (vx, vy)]
            if clear(net, pts, stub_w / 2, 0.0, (vx, vy), clr, own_box):
                tr = pcbnew.PCB_TRACK(b)
                tr.SetStart(pcbnew.VECTOR2I(FROM(px), FROM(py)))
                tr.SetEnd(pcbnew.VECTOR2I(FROM(vx), FROM(vy)))
                tr.SetWidth(FROM(stub_w)); tr.SetLayer(pad.GetPrincipalLayer()); tr.SetNet(pad.GetNet()); tr.SetLocked(True)
                b.Add(tr)
                obstacles.append((net, min(px, vx), min(py, vy), max(px, vx), max(py, vy)))
                stubs += 1
                done = True
                break
        if done:
            continue
        sizes = VIA_RULES.options((0.8, 0.4), (0.6, 0.3), (0.5, 0.25))
        attempts = [(vd, dr, stub_w) for vd, dr in sizes]
        if stub_w > ns.min_stub:
            attempts += [(vd, dr, ns.min_stub) for vd, dr in sizes[1:] or sizes]
        for via_d, drill, stub_w in attempts:
            for dx, dy in dirs:
                for dist in (0.6, 0.8, 1.0, 1.3, 1.6, 2.0, 2.5, 3.0, 3.5):
                    for side in (0.0, 0.4, -0.4, 0.8, -0.8):
                        ex, ey = px + dx * (reach + dist), py + dy * (reach + dist)
                        vx, vy = ex + dy * side, ey + dx * side
                        pts = [(px, py), (ex, ey), (vx, vy)] if side else [(px, py), (vx, vy)]
                        if any(math.hypot(vx - hx, vy - hy) < hr + drill / 2 + ns.hole_clearance
                               for hx, hy, hr in holes):
                            continue
                        if clear(net, pts, stub_w / 2, via_d / 2, (vx, vy), clr, own_box):
                            for (ax, ay), (bx2, by2) in zip(pts, pts[1:]):
                                tr = pcbnew.PCB_TRACK(b)
                                tr.SetStart(pcbnew.VECTOR2I(FROM(ax), FROM(ay)))
                                tr.SetEnd(pcbnew.VECTOR2I(FROM(bx2), FROM(by2)))
                                tr.SetWidth(FROM(stub_w)); tr.SetLayer(pad.GetPrincipalLayer())
                                tr.SetNet(pad.GetNet()); tr.SetLocked(True)
                                b.Add(tr)
                                obstacles.append((net, min(ax, bx2) - stub_w / 2, min(ay, by2) - stub_w / 2,
                                                  max(ax, bx2) + stub_w / 2, max(ay, by2) + stub_w / 2))
                            v = pcbnew.PCB_VIA(b)
                            v.SetPosition(pcbnew.VECTOR2I(FROM(vx), FROM(vy)))
                            v.SetWidth(FROM(via_d)); v.SetDrill(FROM(drill)); v.SetNet(pad.GetNet()); v.SetLocked(True)
                            b.Add(v)
                            obstacles.append((net, vx - via_d / 2, vy - via_d / 2, vx + via_d / 2, vy + via_d / 2))
                            made_vias.setdefault(net, []).append((vx, vy))
                            holes.append((vx, vy, drill / 2))
                            stubs += len(pts) - 1
                            vias += 1
                            done = True
                            break
                    if done:
                        break
                if done:
                    break
            if done:
                break
        if not done and DEBUG:
            print("DEBUG", fp.GetReference(), pad.GetNumber(), "stub", stub_w, "clr", clr, sorted(blockers.items(), key=lambda kv: -kv[1])[:8])
        blockers.clear()
        if not done:
            skipped += 1
            report.append(f"{fp.GetReference()}.{pad.GetNumber()} ({net})")

b.Save(ns.board)
print(f"escapes: {stubs} stubs, {vias} vias; no room for {skipped}: {report}")
