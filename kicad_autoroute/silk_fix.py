"""Silkscreen clean-up after a placement change (run with KiCad's python).

  <kicad-python> kicad_autoroute/silk_fix.py board.kicad_pcb --original original.kicad_pcb [--keep U,J,SW,TP,Q,Y]

1. Board texts (labels such as "PHONE", "! HV 400Vpk", button names) follow the part they label:
   each text is tied to the nearest footprint in the original layout and moved by that footprint's
   displacement.
2. Any board text that overlaps a pad or another text, or leaves the board, moves to the nearest
   free spot within --search mm. Board texts go first because they carry meaning.
3. Reference designators: values off silk; references of small parts hidden (they stay on the fab
   layer) unless their prefix is in --keep; the rest go just outside the courtyard (above, below,
   left, right) where they hit no pad, no text and stay on the board, else they are hidden.
"""
import argparse
import math

import pcbnew

ap = argparse.ArgumentParser()
ap.add_argument("board")
ap.add_argument("--original", required=True)
ap.add_argument("--keep", default="U,J,SW,TP,BZ,Y,Q,F,K,D8,D9")
ap.add_argument("--small-area", type=float, default=8.0, help="mm2 courtyard below which references are hidden")
ap.add_argument("--ref-size", type=float, default=0.8)
ap.add_argument("--search", type=float, default=14.0)
ap.add_argument("--texts-from-original", action="store_true",
                help="replace the board's silk texts by the original board's (when the routed copy came from an "
                     "older stage of the project)")
ns = ap.parse_args()

mm, TO = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(ns.board)
orig = pcbnew.LoadBoard(ns.original)
SILK = (pcbnew.F_SilkS, pcbnew.B_SilkS)
keep = tuple(k for k in ns.keep.split(",") if k)


def board_texts(board):
    return [d for d in board.GetDrawings() if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() in SILK]


if ns.texts_from_original:
    for t in board_texts(b):
        b.Delete(t)
    for src in board_texts(orig):
        t = pcbnew.PCB_TEXT(b)
        t.SetText(src.GetText())
        t.SetLayer(src.GetLayer())
        t.SetPosition(src.GetPosition())
        t.SetTextSize(src.GetTextSize())
        t.SetTextThickness(src.GetTextThickness())
        t.SetTextAngle(src.GetTextAngle())
        t.SetHorizJustify(src.GetHorizJustify())
        t.SetVertJustify(src.GetVertJustify())
        t.SetMirrored(src.IsMirrored())
        b.Add(t)

# 1. move board texts with their nearest footprint
old_pos = {f.GetReference(): f.GetPosition() for f in orig.GetFootprints()}
old_boxes = {f.GetReference(): f.GetBoundingBox(False, False) for f in orig.GetFootprints()}
new_fp = {f.GetReference(): f for f in b.GetFootprints()}
orig_texts = {(t.GetText(), t.GetLayer()): t.GetPosition() for t in board_texts(orig)}


def box_point_dist(box, p):
    dx = max(box.GetLeft() - p.x, 0, p.x - box.GetRight())
    dy = max(box.GetTop() - p.y, 0, p.y - box.GetBottom())
    return math.hypot(dx, dy)


moved_labels = []
for t in board_texts(b):
    p0 = orig_texts.get((t.GetText(), t.GetLayer()), t.GetPosition())
    ref = min(old_boxes, key=lambda r: box_point_dist(old_boxes[r], p0))
    if ref in new_fp:
        d = new_fp[ref].GetPosition() - old_pos[ref]
        t.SetPosition(pcbnew.VECTOR2I(p0.x + d.x, p0.y + d.y))
        moved_labels.append(f"{t.GetText()!r}->{ref}")

# obstacles
pad_boxes = []
for f in b.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox()
        bb.Inflate(mm(0.15))
        pad_boxes.append(bb)
edge = b.GetBoardEdgesBoundingBox()
placed = []
# footprint silkscreen artwork (outlines, polarity marks, pin-1 dots) must not be covered either
for f in b.GetFootprints():
    for g in f.GraphicalItems():
        if g.GetLayer() in SILK and g.IsVisible() if hasattr(g, "IsVisible") else g.GetLayer() in SILK:
            bb = g.GetBoundingBox()
            bb.Inflate(mm(0.1))
            placed.append(bb)


def free(box):
    if not (edge.Contains(box.GetOrigin()) and edge.Contains(box.GetEnd())):
        return False
    return not any(box.Intersects(pb) for pb in pad_boxes) and not any(box.Intersects(o) for o in placed)


# 2. board texts first (they carry meaning): nearest free spot
step = mm(0.25)
unresolved = []
for t in board_texts(b):
    start = t.GetPosition()
    ok = False
    for ring in range(0, int(ns.search / 0.25) + 1):
        offsets = [(0, 0)] if ring == 0 else \
            [(i, -ring) for i in range(-ring, ring + 1)] + [(i, ring) for i in range(-ring, ring + 1)] + \
            [(-ring, i) for i in range(-ring + 1, ring)] + [(ring, i) for i in range(-ring + 1, ring)]
        offsets.sort(key=lambda o: abs(o[0]) + abs(o[1]))
        for dx, dy in offsets:
            t.SetPosition(pcbnew.VECTOR2I(start.x + dx * step, start.y + dy * step))
            tb = t.GetBoundingBox()
            if free(tb):
                placed.append(tb)
                ok = True
                break
        if ok:
            break
    if not ok:
        t.SetPosition(start)
        unresolved.append(t.GetText())

# 3. reference designators, around the texts already placed
hidden, shown = 0, 0
for f in b.GetFootprints():
    f.Value().SetVisible(False)
    ref = f.Reference()
    name = f.GetReference()
    if name.startswith(("H", "MH")):
        ref.SetVisible(False)
        continue
    cy = f.GetCourtyard(pcbnew.F_CrtYd if f.GetLayer() == pcbnew.F_Cu else pcbnew.B_CrtYd)
    cbb = cy.BBox() if cy.OutlineCount() else f.GetBoundingBox(False, False)
    small = TO(cbb.GetWidth()) * TO(cbb.GetHeight()) < ns.small_area
    if small and not name.startswith(keep):
        ref.SetVisible(False)
        hidden += 1
        continue
    ref.SetVisible(True)
    ref.SetLayer(pcbnew.F_SilkS if f.GetLayer() == pcbnew.F_Cu else pcbnew.B_SilkS)
    ref.SetTextSize(pcbnew.VECTOR2I(mm(ns.ref_size), mm(ns.ref_size)))
    ref.SetTextThickness(mm(ns.ref_size * 0.15))
    ref.SetTextAngle(pcbnew.EDA_ANGLE(0, pcbnew.DEGREES_T))
    cx, cyy = cbb.GetCenter().x, cbb.GetCenter().y
    cands = [(cx, cbb.GetTop() - mm(0.6)), (cx, cbb.GetBottom() + mm(0.6)),
             (cbb.GetLeft() - mm(1.2), cyy), (cbb.GetRight() + mm(1.2), cyy),
             (cbb.GetLeft() - mm(0.4), cbb.GetTop() - mm(0.6)), (cbb.GetRight() + mm(0.4), cbb.GetTop() - mm(0.6)),
             (cx, cyy)]
    for x, y in cands:
        ref.SetPosition(pcbnew.VECTOR2I(int(x), int(y)))
        tb = ref.GetBoundingBox()
        if free(tb):
            placed.append(tb)
            shown += 1
            break
    else:
        ref.SetVisible(False)
        hidden += 1

b.Save(ns.board)
print(f"silk: labels {moved_labels}; references shown {shown}, hidden {hidden}; unresolved texts {unresolved}")
