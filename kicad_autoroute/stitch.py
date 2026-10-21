"""Stitching vias for a plane net next to every SMD pad of that net (run with KiCad's python).

  <kicad-python> kicad_autoroute/stitch.py board.kicad_pcb [--net GND [--net +3V3 ...]] [--skip-refs U4]
                                          [--extra-clearance HV_NET=1.6]

Each SMD pad of the plane nets (default GND) gets a short track to a via that drops into the plane,
or a short link to copper of its net that already reaches the plane. Several nets are done in one
pass, larger pads (capacitors) before fine-pitch pins, so the supply and ground pins of an IC share
the room in front of them instead of the first net taking it all.
Candidate spots are tried outward from the footprint centre, fanning out in 22.5 degree steps; the
first spot that keeps the required clearance to every other-net pad, track and via and to the board
edge is used. Required clearance = the larger net-class clearance of the two nets (read from the
.kicad_pro), at least --min-clearance, or an --extra-clearance override for special nets (high
voltage). Fine-pitch pads get a smaller via and a thinner link. Pads already connected by a track
are skipped. Vias and links are locked so the autorouter keeps them.

"""
import argparse
import math
import os
import sys

import headless  # noqa: F401  (first: KiCad's message boxes go to stderr)
import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kicad_classes import NetClasses, ViaRules, board_edges, keepouts, poly_dist  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("board")
ap.add_argument("--net", action="append", help="plane net (repeatable; default GND)")
ap.add_argument("--via", type=float, nargs=2, default=None, metavar=("DIA", "DRILL"),
                help="default: 0.6/0.3 raised to the board's via rules")
ap.add_argument("--fine-via", type=float, nargs=2, default=None, metavar=("DIA", "DRILL"),
                help="default: 0.45/0.2 raised to the board's via rules")
ap.add_argument("--link-width", type=float, default=0.4)
ap.add_argument("--fine-link-width", type=float, default=0.25)
ap.add_argument("--fine-pad", type=float, default=0.45, help="pads narrower than this (mm) count as fine pitch")
ap.add_argument("--min-clearance", type=float, default=0.2)
ap.add_argument("--edge", type=float, default=0.9, help="mm from via centre to board edge (radius + edge clearance + margin)")
ap.add_argument("--skip-refs", default="", help="comma-separated references to skip (e.g. parts with thermal vias)")
ap.add_argument("--skip-prefixes", default="H,TP,MH", help="reference prefixes to skip")
ap.add_argument("--refs", default="", help="only treat these comma-separated references (local repair)")
ap.add_argument("--share-reach", type=float, default=6.0,
                help="2-layer boards: link a pad to plane-net copper within this many mm instead of a new via")
ap.add_argument("--extra-clearance", action="append", default=[], help="NET=mm, e.g. HV_OUT=1.6")
ns = ap.parse_args()
NETS = ns.net or ["GND"]
VIA_RULES = ViaRules.for_board(ns.board)
ns.via = VIA_RULES.legal(*(ns.via or (0.6, 0.3)))
ns.fine_via = VIA_RULES.legal(*(ns.fine_via or (0.45, 0.2)))

b = pcbnew.LoadBoard(ns.board)
NETS = [n for n in NETS if b.FindNet(n) is not None]
if not NETS:
    raise SystemExit(f"none of the nets {ns.net} found")
TO, FROM = pcbnew.ToMM, pcbnew.FromMM
classes = NetClasses.for_board(ns.board)
# custom-rule clearances from the .kicad_dru (+0.1 mm margin), overridden by --extra-clearance
extra = {k: v + 0.1 for k, v in NetClasses.dru_net_clearances(ns.board).items()}
extra.update({k: float(v) for k, v in (e.split("=", 1) for e in ns.extra_clearance)})
skip_refs = {r for r in ns.skip_refs.split(",") if r}
skip_prefixes = tuple(p for p in ns.skip_prefixes.split(",") if p)
# the net being stitched right now (the checks below compare against it)
cur_net = NETS[0]
own_clear = classes.clearance(cur_net)


def need(other_net, base):
    if other_net in extra:
        return extra[other_net]
    return max(base, own_clear, classes.clearance(other_net), ns.min_clearance)


pads, tracks, vias = [], [], []
# the board outline (notches and cut-outs inside its bounding box): vias and links keep the edge clearance
edge_segs, edge_clr = board_edges(b)
rule_areas = keepouts(b)
for f in b.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox()
        # a clearance set on the pad or its part (1.7 mm around a mounting hole) above its net
        # class: the pad's box grows by the difference
        g = max(0.0, TO(p.GetOwnClearance(pcbnew.F_Cu)) - classes.clearance(p.GetNetname()))
        pads.append((p, p.GetNetname(), TO(bb.GetLeft()) - g, TO(bb.GetTop()) - g, TO(bb.GetRight()) + g,
                     TO(bb.GetBottom()) + g))
for t in b.GetTracks():
    if t.GetClass() == "PCB_VIA":
        vias.append((t.GetNetname(), TO(t.GetPosition().x), TO(t.GetPosition().y), TO(t.GetWidth(pcbnew.F_Cu)) / 2))
    else:
        tracks.append((t.GetNetname(), t.GetLayer(), TO(t.GetStart().x), TO(t.GetStart().y),
                       TO(t.GetEnd().x), TO(t.GetEnd().y), TO(t.GetWidth()) / 2))

ebb = b.GetBoardEdgesBoundingBox()
EX0, EY0 = TO(ebb.GetLeft()) + ns.edge, TO(ebb.GetTop()) + ns.edge
EX1, EY1 = TO(ebb.GetRight()) - ns.edge, TO(ebb.GetBottom()) - ns.edge


def seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    length = dx * dx + dy * dy
    t = 0 if length == 0 else max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def box_dist(px, py, x0, y0, x1, y1):
    return math.hypot(max(x0 - px, 0, px - x1), max(y0 - py, 0, py - y1))


def samples(a, c, n=10):
    return [(a[0] + (c[0] - a[0]) * i / n, a[1] + (c[1] - a[1]) * i / n) for i in range(n + 1)]


def ok(cx, cy, own_pad, link_from, tw, base_clr, vd, layer):
    r = vd / 2
    if not (EX0 < cx < EX1 and EY0 < cy < EY1):
        return False
    link = samples(link_from, (cx, cy))
    for p, net, x0, y0, x1, y1 in pads:
        if p is own_pad or net == cur_net:
            continue
        clr = need(net, base_clr)
        if box_dist(cx, cy, x0, y0, x1, y1) < r + clr:
            return False
        if any(box_dist(px, py, x0, y0, x1, y1) < tw / 2 + clr for px, py in link):
            return False
    for net, tl, x1, y1, x2, y2, hw in tracks:
        if net == cur_net:
            continue
        clr = need(net, base_clr)
        if seg_dist(cx, cy, x1, y1, x2, y2) < r + hw + clr:
            return False
        if tl == layer and any(seg_dist(px, py, x1, y1, x2, y2) < tw / 2 + hw + clr for px, py in link):
            return False
    for net, vx, vy, vr in vias:
        gap = 0.2 if net == cur_net else need(net, base_clr)
        if math.hypot(cx - vx, cy - vy) < r + vr + gap:
            return False
    for x1, y1, x2, y2 in edge_segs:
        if seg_dist(cx, cy, x1, y1, x2, y2) < r + edge_clr or any(
                seg_dist(px, py, x1, y1, x2, y2) < tw / 2 + edge_clr for px, py in link):
            return False
    for pts, lays, no_tracks, no_vias in rule_areas:
        if (no_vias and poly_dist(cx, cy, pts) < r + base_clr) or (no_tracks and layer in lays and any(
                poly_dist(px, py, pts) < tw / 2 + base_clr for px, py in link)):
            return False
    return True


def link_ok(a, c, tw, layer, skip_pads):
    """A straight track from a to c on layer keeps its clearance to everything of other nets."""
    link = samples(a, c, max(10, int(math.hypot(c[0] - a[0], c[1] - a[1]) / 0.2)))
    for p, net, x0, y0, x1, y1 in pads:
        if net == cur_net or p.m_Uuid.AsString() in skip_pads:
            continue
        if not p.IsOnLayer(layer):
            continue
        clr = need(net, own_clear)
        if any(box_dist(px, py, x0, y0, x1, y1) < tw / 2 + clr for px, py in link):
            return False
    for net, tl, x1, y1, x2, y2, hw in tracks:
        if net != cur_net and tl == layer and any(seg_dist(px, py, x1, y1, x2, y2) < tw / 2 + hw + need(net, own_clear)
                                                 for px, py in link):
            return False
    for net, vx, vy, vr in vias:
        if net != cur_net and any(math.hypot(px - vx, py - vy) < tw / 2 + vr + need(net, own_clear) for px, py in link):
            return False
    for x1, y1, x2, y2 in edge_segs:
        if any(seg_dist(px, py, x1, y1, x2, y2) < tw / 2 + edge_clr for px, py in link):
            return False
    for pts, lays, no_tracks, _no_vias in rule_areas:
        if no_tracks and layer in lays and any(poly_dist(px, py, pts) < tw / 2 + own_clear for px, py in link):
            return False
    return True


# Vias are a cost people avoid: a pad first tries a short straight track to copper of the net
# that already reaches the pour (a through-hole pad, a via, a pad linked before), the way
# neighbouring ground pins share one via.
# next to a plane, pads further apart share too (a decoupling capacitor beside its IC pin, ground
# pins of one part; 8 mm by default): people keep the vias into the plane few
share_reach = ns.share_reach if b.GetCopperLayerCount() <= 2 or ns.share_reach <= 0 else ns.share_reach + 2.0
# 2-layer boards: a pad on a side that has a pour of the net is joined by that pour, no via needed
poured = {n: ({l for z in b.Zones() if z.GetNetname() == n and not z.GetIsRuleArea()
               for l in z.GetLayerSet().Seq()} if b.GetCopperLayerCount() <= 2 else set()) for n in NETS}
anchors = {n: [(TO(p.GetPosition().x), TO(p.GetPosition().y), None, p) for f in b.GetFootprints() for p in f.Pads()
               if p.GetNetname() == n and p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD]
              + [(vx, vy, None, None) for net, vx, vy, _r in vias if net == n] for n in NETS}

added, linked, failed = {n: 0 for n in NETS}, {n: 0 for n in NETS}, {n: [] for n in NETS}
todo = []
for f in b.GetFootprints():
    ref = f.GetReference()
    if ref in skip_refs or ref.startswith(skip_prefixes):
        continue
    if ns.refs and ref not in ns.refs.split(","):
        continue
    todo += [(f, p) for p in f.Pads() if p.GetNetname() in NETS and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
# larger pads first (capacitors and the like), fine-pitch pins last: a pin of an IC can then join
# the via its decoupling capacitor already has, the way people wire them
todo.sort(key=lambda fp: -min(fp[1].GetSize().x, fp[1].GetSize().y))
for f, p in todo:
    cur_net = p.GetNetname()
    own_clear = classes.clearance(cur_net)
    net_item = b.FindNet(cur_net)
    ref = f.GetReference()
    fcx, fcy = TO(f.GetPosition().x), TO(f.GetPosition().y)
    px, py = TO(p.GetPosition().x), TO(p.GetPosition().y)
    if any(tn == cur_net and abs(tx - px) < 1e-3 and abs(ty - py) < 1e-3 for tn, _l, tx, ty, _a, _b, _h in tracks):
        continue
    bb = p.GetBoundingBox()
    hw, hh = TO(bb.GetWidth()) / 2, TO(bb.GetHeight()) / 2
    layer = p.GetPrincipalLayer()
    if layer in poured[cur_net]:
        continue
    if share_reach > 0:
        tw0 = ns.link_width if min(hw, hh) * 2 >= ns.fine_pad else ns.fine_link_width
        # sorted by distance and position only: on a tie Python would compare the pads themselves,
        # which KiCad answers with an assert dialog that blocks a headless run
        near = sorted(((math.hypot(ax - px, ay - py), ax, ay, al, ap) for ax, ay, al, ap in anchors[cur_net]
                       if math.hypot(ax - px, ay - py) <= share_reach and (al is None or al == layer)),
                      key=lambda t: t[:3])
        done_link = False
        # a pin in a row leaves straight out of the row first (a knee), then heads for the anchor
        ox, oy = px - fcx, py - fcy
        out = (1.0 if ox > 0 else -1.0, 0.0) if abs(ox) >= abs(oy) else (0.0, 1.0 if oy > 0 else -1.0)
        reach_out = (hw if out[0] else hh) + tw0 / 2 + 0.35
        knee = (px + out[0] * reach_out, py + out[1] * reach_out)
        for _d, ax, ay, _al, ap in near:
            skip = {p.m_Uuid.AsString()} | ({ap.m_Uuid.AsString()} if ap is not None else set())
            for pts in ([(px, py), (ax, ay)], [(px, py), knee, (ax, ay)]):
                if not all(link_ok(u, v, tw0, layer, skip) for u, v in zip(pts, pts[1:])):
                    continue
                for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                    t = pcbnew.PCB_TRACK(b)
                    t.SetStart(pcbnew.VECTOR2I(FROM(x1), FROM(y1))); t.SetEnd(pcbnew.VECTOR2I(FROM(x2), FROM(y2)))
                    t.SetWidth(FROM(tw0)); t.SetLayer(layer); t.SetNet(net_item); t.SetLocked(True)
                    b.Add(t)
                    tracks.append((cur_net, layer, x1, y1, x2, y2, tw0 / 2))
                anchors[cur_net].append((px, py, layer, p))
                linked[cur_net] += 1
                done_link = True
                break
            if done_link:
                break
        if done_link:
            continue
    base = math.atan2(py - fcy, px - fcx) if (px, py) != (fcx, fcy) else 0.0
    fine = min(hw, hh) * 2 < ns.fine_pad
    tw = ns.fine_link_width if fine else ns.link_width
    vd, vdrill = ns.fine_via if fine else ns.via
    placed = False
    for dist_extra in ((0.3, 0.6, 0.9, 1.3, 1.8) if fine else (0.45, 0.7, 1.0, 1.4)):
        for k in range(16):
            ang = base + (k // 2) * (math.pi / 8) * (1 if k % 2 == 0 else -1)
            ux, uy = math.cos(ang), math.sin(ang)
            edge = min(hw / abs(ux) if abs(ux) > 1e-6 else 1e9, hh / abs(uy) if abs(uy) > 1e-6 else 1e9)
            d = edge + vd / 2 + dist_extra
            cx, cy = px + ux * d, py + uy * d
            if ok(cx, cy, p, (px, py), tw, own_clear, vd, layer):
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pcbnew.VECTOR2I(FROM(cx), FROM(cy)))
                v.SetWidth(FROM(vd)); v.SetDrill(FROM(vdrill)); v.SetNet(net_item); v.SetLocked(True)
                b.Add(v)
                t = pcbnew.PCB_TRACK(b)
                t.SetStart(p.GetPosition()); t.SetEnd(pcbnew.VECTOR2I(FROM(cx), FROM(cy)))
                t.SetWidth(FROM(tw)); t.SetLayer(layer); t.SetNet(net_item); t.SetLocked(True)
                b.Add(t)
                vias.append((cur_net, cx, cy, vd / 2))
                tracks.append((cur_net, layer, px, py, cx, cy, tw / 2))
                anchors[cur_net].append((cx, cy, None, None))
                anchors[cur_net].append((px, py, layer, p))
                added[cur_net] += 1
                placed = True
                break
        if placed:
            break
    if not placed:
        failed[cur_net].append(f"{ref}.{p.GetNumber()}")

b.Save(ns.board)
for n in NETS:
    print(f"{n} stitching vias added: {added[n]}, pads linked to nearby {n} copper: {linked[n]}; no room for: {failed[n]}")
