"""Routability analysis and placement optimisation for Specctra DSN boards.

  python kicad_autoroute/placement.py analyze board.dsn
  python kicad_autoroute/placement.py optimize board.dsn -o board_placed.dsn [--mode rotate|local|global]

analyze  prints wirelength (HPWL), ratsnest length and crossings, RUDY congestion, courtyard
         overlaps and parts outside the board.
optimize improves the placement with simulated annealing:
           rotate  only turns parts in 90 degree steps (positions stay)
           local   turns and moves parts at most --radius mm from where they are (default 5 mm)
           global  moves parts anywhere on the board
         Fixed parts (connectors, mounting holes, parts without nets, --fix) never move.
         Writes the new DSN plus <out>.moves.json for applying the changes in KiCad.

The cost the annealer minimises is
    HPWL/HPWL0 + w_cross * crossings/crossings0 + w_rudy * congestion/congestion0
where congestion is the RUDY overflow above the board average. Courtyard overlaps and courtyard
area outside the board are hard constraints: a move that grows them is rejected, so overhang
that the designer chose (edge connectors, mounting holes) is kept but never made worse.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dsn  # noqa: E402
from dsn import Atom  # noqa: E402

FIXED_REF_RE = re.compile(r"^(J|P|CN|X|H|MH|MK|FID|BT|SW|TP)\d", re.I)
FIXED_IMAGE_RE = re.compile(r"MountingHole|Connector|TerminalBlock|Fiducial|TestPoint|USB|Jack|Header", re.I)


# ---- model ----------------------------------------------------------------------------------


@dataclass
class Part:
    ref: str
    image: str
    x: float
    y: float
    rot: float
    front: bool
    pins: dict[str, tuple[float, float]]
    body: tuple[float, float, float, float]  # local courtyard bbox
    through_hole: bool
    fixed: bool = False
    place_node: list = field(default=None, repr=False)
    axis: str | None = None  # "x" or "y": the part may only slide along this axis (edge connectors)
    group: list | None = None  # refs of a rigid group this part belongs to (moved together)

    def transform(self, lx: float, ly: float, x=None, y=None, rot=None) -> tuple[float, float]:
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        if not self.front:
            lx = -lx
        r = math.radians(rot)
        c, s = math.cos(r), math.sin(r)
        return x + lx * c - ly * s, y + lx * s + ly * c

    def world_body(self, x=None, y=None, rot=None) -> tuple[float, float, float, float]:
        x0, y0, x1, y1 = self.body
        pts = [self.transform(px, py, x, y, rot) for px, py in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return min(xs), min(ys), max(xs), max(ys)


@dataclass
class Board:
    tree: list
    parts: dict[str, Part]
    nets: dict[str, list[tuple[str, str]]]
    net_weight: dict[str, float]
    outline: tuple[float, float, float, float]
    unit_mm: float  # DSN units per mm


def _padstack_extents(tree) -> dict[str, tuple[float, float, bool]]:
    """Half width, half height and through-hole flag per padstack."""
    lib = dsn.child(tree, "library") or []
    result = {}
    for ps in dsn.children(lib, "padstack"):
        name = dsn.atoms(ps)[1]
        hw = hh = 0.0
        layers = set()
        for shape in dsn.children(ps, "shape"):
            for geo in shape:
                if not isinstance(geo, list):
                    continue
                kind = dsn.head(geo)
                vals = [float(a) for a in dsn.atoms(geo)[2:] if _is_num(a)]
                layers.add(dsn.atoms(geo)[1] if len(dsn.atoms(geo)) > 1 else "")
                if kind == "circle" and vals:
                    hw = max(hw, vals[0] / 2)
                    hh = max(hh, vals[0] / 2)
                elif kind == "rect" and len(vals) >= 4:
                    hw = max(hw, abs(vals[2] - vals[0]) / 2)
                    hh = max(hh, abs(vals[3] - vals[1]) / 2)
                elif kind in ("polygon", "path") and len(vals) >= 3:
                    pts = vals[1:]
                    xs, ys = pts[0::2], pts[1::2]
                    if xs and ys:
                        hw = max(hw, max(abs(v) for v in xs))
                        hh = max(hh, max(abs(v) for v in ys))
        result[name] = (hw, hh, len(layers) > 1 or "signal" in layers)
    return result


def _is_num(text: str) -> bool:
    try:
        float(text)
        return True
    except ValueError:
        return False


def load(path: Path, channel_mm: float = 0.5, spacing_mm: float = 0.0) -> Board:
    tree = dsn.read(path)
    structure = dsn.child(tree, "structure")
    resolution = dsn.child(tree, "resolution")
    unit = dsn.atoms(dsn.child(tree, "unit") or resolution)[1].lower() if (dsn.child(tree, "unit") or resolution) else "um"
    unit_mm = {"um": 1000.0, "mm": 1.0, "mil": 1 / 0.0254, "inch": 1 / 25.4, "cm": 0.1}.get(unit, 1000.0)

    pads = _padstack_extents(tree)
    rule = dsn.child(structure, "rule") or []
    clearance = next((float(dsn.atoms(c)[1]) for c in dsn.children(rule, "clearance") if len(c) == 2), 0.0)
    channel = channel_mm * unit_mm
    images = {}
    for img in dsn.children(dsn.child(tree, "library") or [], "image"):
        name = dsn.atoms(img)[1]
        pins = {}
        bx = [math.inf, math.inf, -math.inf, -math.inf]
        th = False
        for pin in dsn.children(img, "pin"):
            a = [x for x in pin if isinstance(x, Atom)]
            # (pin padstack [(rotate r)] id x y)
            pid, px, py = a[2].text, float(a[3].text), float(a[4].text)
            pins[pid] = (px, py)
            hw, hh, is_th = pads.get(a[1].text, (0.0, 0.0, False))
            th = th or is_th
            bx = [min(bx[0], px - hw), min(bx[1], py - hh), max(bx[2], px + hw), max(bx[3], py + hh)]
        courtyard = None
        for out in dsn.children(img, "outline"):
            geo = next((g for g in out if isinstance(g, list)), None)
            if geo and dsn.head(geo) == "polygon":
                vals = [float(v) for v in dsn.atoms(geo)[2:] if _is_num(v)][1:]
                if dsn.atoms(geo)[2] in ("0", "0.0") and len(vals) >= 6:
                    xs, ys = vals[0::2], vals[1::2]
                    courtyard = (min(xs), min(ys), max(xs), max(ys))
                    break
        if bx[0] == math.inf:
            bx = [-1.0, -1.0, 1.0, 1.0]
        if courtyard:
            bx = [min(bx[0], courtyard[0]), min(bx[1], courtyard[1]), max(bx[2], courtyard[2]), max(bx[3], courtyard[3])]
        else:
            # No courtyard in the DSN: keep half the clearance plus half a routing channel around
            # the pads, so two neighbours keep at least clearance + one channel between their pads.
            m = (clearance + channel) / 2
            bx = [bx[0] - m, bx[1] - m, bx[2] + m, bx[3] + m]
        if spacing_mm > 0:
            s = spacing_mm * unit_mm / 2  # extra keep-out on every part (compaction needs routing room)
            bx = [bx[0] - s, bx[1] - s, bx[2] + s, bx[3] + s]
        images[name] = (pins, tuple(bx), th)

    parts: dict[str, Part] = {}
    for comp in dsn.children(dsn.child(tree, "placement") or [], "component"):
        image = dsn.atoms(comp)[1]
        pins, body, th = images.get(image, ({}, (-1, -1, 1, 1), False))
        for place in dsn.children(comp, "place"):
            a = dsn.atoms(place)
            if len(a) < 6:
                continue
            locked = any(dsn.head(x) == "lock_type" for x in place if isinstance(x, list))
            parts[a[1]] = Part(a[1], image, float(a[2]), float(a[3]), float(a[5]), a[4] == "front",
                               pins, body, th, fixed=locked, place_node=place)

    planes = {dsn.atoms(p)[1] for p in dsn.children(structure, "plane")}
    nets: dict[str, list[tuple[str, str]]] = {}
    for net in dsn.children(dsn.child(tree, "network") or [], "net"):
        name = dsn.atoms(net)[1]
        pins_node = dsn.child(net, "pins")
        refs = []
        for token in dsn.atoms(pins_node)[1:] if pins_node else []:
            ref, _, pin = token.rpartition("-")
            if ref in parts and pin in parts[ref].pins:
                refs.append((ref, pin))
        if len(refs) >= 2:
            nets[name] = refs
    weights = {}
    for name, refs in nets.items():
        if name in planes:
            weights[name] = 0.0  # connected through the plane, not by traces
        elif len(refs) > 12:
            weights[name] = 0.25  # supply-like nets: many pins, usually poured or wide
        else:
            weights[name] = 1.0

    boundary = dsn.child(structure, "boundary")
    xs, ys = [], []
    for geo in boundary or []:
        if isinstance(geo, list):
            vals = [float(v) for v in dsn.atoms(geo)[2:] if _is_num(v)]
            if dsn.head(geo) == "path":
                vals = vals[1:]
            xs += vals[0::2]
            ys += vals[1::2]
    outline = (min(xs), min(ys), max(xs), max(ys)) if xs else (-1e9, -1e9, 1e9, 1e9)
    return Board(tree, parts, nets, weights, outline, unit_mm)


def default_fixed(board: Board, fix: set[str], free: set[str]) -> None:
    net_parts = {ref for refs in board.nets.values() for ref, _ in refs}
    for p in board.parts.values():
        auto = (FIXED_REF_RE.match(p.ref) is not None or FIXED_IMAGE_RE.search(p.image) is not None
                or p.ref not in net_parts)
        p.fixed = (p.fixed or auto or p.ref in fix) and p.ref not in free


# ---- evaluation -----------------------------------------------------------------------------


class Evaluator:
    """Keeps per-net terms so a move only re-evaluates the nets and parts it touches."""

    def __init__(self, board: Board, w_cross: float, w_rudy: float, w_overlap: float, grid: int = 48):
        self.b = board
        self.w_cross, self.w_rudy, self.w_overlap = w_cross, w_rudy, w_overlap
        self.parts = list(board.parts.values())
        self.pos = {p.ref: (p.x, p.y, p.rot) for p in self.parts}
        self.part_nets: dict[str, list[str]] = {}
        for name, refs in board.nets.items():
            for ref, _ in refs:
                self.part_nets.setdefault(ref, [])
                if name not in self.part_nets[ref]:
                    self.part_nets[ref].append(name)
        x0, y0, x1, y1 = board.outline
        self.gx0, self.gy0 = x0, y0
        self.cell = max(x1 - x0, y1 - y0) / grid
        self.gw = max(1, int(math.ceil((x1 - x0) / self.cell)))
        self.gh = max(1, int(math.ceil((y1 - y0) / self.cell)))
        self.rudy = np.zeros((self.gh, self.gw))
        self.net_hpwl: dict[str, float] = {}
        self.net_segs: dict[str, list] = {}
        self.net_rudy: dict[str, tuple] = {}
        for name in board.nets:
            self._eval_net(name, add=True)
        self.crossings = self._count_all_crossings()
        self.mean_len = max(1.0, sum(self.net_hpwl.values()) / max(1, len(self.net_hpwl)))
        self.hp0 = max(sum(self.net_hpwl.values()), 1e-9)
        self.cross0 = max(self.crossings, 1)
        self.cong0 = max(self.congestion(), 1e-9)
        self.overlap = sum(self._part_overlap(p.ref, *self.pos[p.ref], others_after=True) for p in self.parts)
        self.outside = sum(self._outside(p.ref, *self.pos[p.ref]) for p in self.parts)

    # -- per-net terms
    def pin_xy(self, ref: str, pin: str):
        p = self.b.parts[ref]
        x, y, r = self.pos[ref]
        lx, ly = p.pins[pin]
        return p.transform(lx, ly, x, y, r)

    def _eval_net(self, name: str, add: bool) -> None:
        if not add:
            self.rudy -= self._rudy_patch(*self.net_rudy[name]) if self.net_rudy.get(name) else 0
        pts = [self.pin_xy(r, p) for r, p in self.b.nets[name]]
        w = self.b.net_weight[name]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        hp = (max(xs) - min(xs)) + (max(ys) - min(ys))
        self.net_hpwl[name] = hp * w
        self.net_segs[name] = _mst(pts) if w >= 1.0 else []
        if w > 0:
            box = (min(xs), min(ys), max(xs), max(ys), w)
            self.net_rudy[name] = box
            self.rudy += self._rudy_patch(*box)
        else:
            self.net_rudy[name] = None

    def _rudy_patch(self, x0, y0, x1, y1, w):
        patch = np.zeros_like(self.rudy)
        c0 = int(np.clip((x0 - self.gx0) // self.cell, 0, self.gw - 1))
        c1 = int(np.clip((x1 - self.gx0) // self.cell, 0, self.gw - 1))
        r0 = int(np.clip((y0 - self.gy0) // self.cell, 0, self.gh - 1))
        r1 = int(np.clip((y1 - self.gy0) // self.cell, 0, self.gh - 1))
        width = max(x1 - x0, self.cell * 0.5)
        height = max(y1 - y0, self.cell * 0.5)
        # RUDY: wire demand (half perimeter) spread uniformly over the net's bounding box.
        patch[r0:r1 + 1, c0:c1 + 1] = w * (width + height) / (width * height) * self.cell
        return patch

    def congestion(self) -> float:
        mean = self.rudy.mean() if self.rudy.size else 0.0
        over = np.clip(self.rudy - 2.0 * mean, 0, None)
        return float((over ** 2).sum()) / max(mean, 1e-9)

    # -- crossings
    def _count_all_crossings(self) -> int:
        segs = [(n, s) for n, ss in self.net_segs.items() for s in ss]
        total = 0
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                if segs[i][0] != segs[j][0] and _intersects(segs[i][1], segs[j][1]):
                    total += 1
        return total

    def _crossings_of(self, names: set[str]) -> int:
        """Crossings involving at least one of the nets in names."""
        mine = [(n, s) for n in names for s in self.net_segs[n]]
        others = [(n, s) for n, ss in self.net_segs.items() if n not in names for s in ss]
        total = 0
        for n1, s1 in mine:
            bx = _seg_box(s1)
            for n2, s2 in others:
                if _box_hit(bx, s2) and _intersects(s1, s2):
                    total += 1
        for i in range(len(mine)):
            for j in range(i + 1, len(mine)):
                if mine[i][0] != mine[j][0] and _intersects(mine[i][1], mine[j][1]):
                    total += 1
        return total

    # -- geometry penalties
    def _part_overlap(self, ref, x, y, r, others_after=False) -> float:
        p = self.b.parts[ref]
        a = p.world_body(x, y, r)
        total = 0.0
        seen = False
        for q in self.parts:
            if q.ref == ref:
                seen = True
                continue
            if others_after and not seen:
                continue
            if not (p.through_hole or q.through_hole or p.front == q.front):
                continue
            qx, qy, qr = self.pos[q.ref]
            b = q.world_body(qx, qy, qr)
            w = min(a[2], b[2]) - max(a[0], b[0])
            h = min(a[3], b[3]) - max(a[1], b[1])
            if w > 0 and h > 0:
                total += w * h
        return total

    def _outside(self, ref, x, y, r) -> float:
        a = self.b.parts[ref].world_body(x, y, r)
        o = self.b.outline
        area = (a[2] - a[0]) * (a[3] - a[1])
        w = max(0.0, min(a[2], o[2]) - max(a[0], o[0]))
        h = max(0.0, min(a[3], o[3]) - max(a[1], o[1]))
        return area - w * h

    def cost(self) -> float:
        """Each term relative to the starting placement, so the weights read as trade-offs:
        w_cross = 0.5 means halving the crossings is worth half of the total wirelength."""
        hp = sum(self.net_hpwl.values()) / self.hp0
        return hp + self.w_cross * self.crossings / self.cross0 + self.w_rudy * self.congestion() / self.cong0

    def metrics(self) -> dict:
        mm = self.b.unit_mm
        rat = sum(math.dist(*s) for ss in self.net_segs.values() for s in ss) / mm
        return {
            "hpwl_mm": round(sum(self.net_hpwl.values()) / mm, 1),
            "ratsnest_mm": round(rat, 1),
            "crossings": self.crossings,
            "rudy_peak": round(float(self.rudy.max() / max(self.rudy.mean(), 1e-9)), 2),
            "congestion": round(self.congestion(), 2),
            "overlap_mm2": round(self.overlap / mm ** 2, 2),
            "outside_mm2": round(self.outside / mm ** 2, 2),
        }

    # -- moves
    def try_move(self, changes: dict[str, tuple[float, float, float]]):
        """Applies changes, returns an undo token. Updates all incremental terms."""
        nets = {n for ref in changes for n in self.part_nets.get(ref, [])}
        old_cross_part = self._crossings_of(nets) if nets else 0
        old_ov = sum(self._part_overlap(ref, *self.pos[ref]) for ref in changes)
        old_out = sum(self._outside(ref, *self.pos[ref]) for ref in changes)
        undo = {ref: self.pos[ref] for ref in changes}
        undo_nets = {n: (self.net_hpwl[n], self.net_segs[n], self.net_rudy[n]) for n in nets}
        for n in nets:
            if self.net_rudy[n]:
                self.rudy -= self._rudy_patch(*self.net_rudy[n])
        for ref, v in changes.items():
            self.pos[ref] = v
        for n in nets:
            self._eval_net(n, add=True)
        self.crossings += (self._crossings_of(nets) if nets else 0) - old_cross_part
        # pairs inside `changes` are counted twice on both sides; good enough as a penalty signal
        self.overlap += sum(self._part_overlap(ref, *self.pos[ref]) for ref in changes) - old_ov
        self.outside += sum(self._outside(ref, *self.pos[ref]) for ref in changes) - old_out
        return undo, undo_nets, old_cross_part, nets, old_ov, old_out

    def undo(self, token) -> None:
        undo, undo_nets, old_cross_part, nets, old_ov, old_out = token
        new_cross_part = self._crossings_of(nets) if nets else 0
        new_ov = sum(self._part_overlap(ref, *self.pos[ref]) for ref in undo)
        new_out = sum(self._outside(ref, *self.pos[ref]) for ref in undo)
        for n in nets:
            if self.net_rudy[n]:
                self.rudy -= self._rudy_patch(*self.net_rudy[n])
        for ref, v in undo.items():
            self.pos[ref] = v
        for n, (hp, segs, box) in undo_nets.items():
            self.net_hpwl[n], self.net_segs[n], self.net_rudy[n] = hp, segs, box
            if box:
                self.rudy += self._rudy_patch(*box)
        self.crossings += old_cross_part - new_cross_part
        self.overlap += old_ov - new_ov
        self.outside += old_out - new_out


def _mst(pts):
    """Prim's minimum spanning tree over the pins of a net (the ratsnest)."""
    n = len(pts)
    if n < 2:
        return []
    in_tree = [False] * n
    best = [math.inf] * n
    parent = [-1] * n
    best[0] = 0
    segs = []
    for _ in range(n):
        u = min((i for i in range(n) if not in_tree[i]), key=lambda i: best[i])
        in_tree[u] = True
        if parent[u] >= 0:
            segs.append((pts[parent[u]], pts[u]))
        for v in range(n):
            if not in_tree[v]:
                d = abs(pts[u][0] - pts[v][0]) + abs(pts[u][1] - pts[v][1])
                if d < best[v]:
                    best[v], parent[v] = d, u
    return segs


def _seg_box(s):
    (x1, y1), (x2, y2) = s
    return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)


def _box_hit(b, s) -> bool:
    (x1, y1), (x2, y2) = s
    return not (max(x1, x2) < b[0] or min(x1, x2) > b[2] or max(y1, y2) < b[1] or min(y1, y2) > b[3])


def _intersects(s1, s2) -> bool:
    (ax, ay), (bx, by) = s1
    (cx, cy), (dx, dy) = s2
    if (ax, ay) in ((cx, cy), (dx, dy)) or (bx, by) in ((cx, cy), (dx, dy)):
        return False

    def orient(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    d1 = orient(cx, cy, dx, dy, ax, ay)
    d2 = orient(cx, cy, dx, dy, bx, by)
    d3 = orient(ax, ay, bx, by, cx, cy)
    d4 = orient(ax, ay, bx, by, dx, dy)
    return (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0) and d1 != 0 and d2 != 0 and d3 != 0 and d4 != 0


# ---- optimisation ---------------------------------------------------------------------------


def anneal(ev: Evaluator, mode: str, radius_mm: float, iterations: int, seed: int, grid_mm: float, log_every: int,
           max_rotate_pins: int = 4, max_move_pins: int = 10**9):
    rnd = random.Random(seed)
    b = ev.b
    movable = [p for p in b.parts.values() if not p.fixed and len(p.pins) <= max_move_pins]
    if not movable:
        return
    origin = {p.ref: (p.x, p.y) for p in movable}
    grid = grid_mm * b.unit_mm
    radius = radius_mm * b.unit_mm
    ox0, oy0, ox1, oy1 = b.outline
    span = max(ox1 - ox0, oy1 - oy0)
    cost = ev.cost()
    t0 = cost * 0.005
    best_cost, best_pos = cost, dict(ev.pos)
    accepted = 0
    size = {p.ref: (p.body[2] - p.body[0]) * (p.body[3] - p.body[1]) for p in movable}
    start = time.time()
    for it in range(iterations):
        frac = it / max(1, iterations - 1)
        temp = t0 * (1 - frac) ** 2 + 1e-9
        p = rnd.choice(movable)
        x, y, r = ev.pos[p.ref]
        kind = rnd.random()
        if (p.axis or p.group) and (mode == "rotate" or kind < 0.35 or kind >= 0.85):
            continue  # edge sliders and rigid groups neither turn nor swap
        if mode == "rotate" or kind < 0.35:
            # Turning a many-pin part changes its escape directions, which the proxies cannot see.
            if len(p.pins) > max_rotate_pins:
                continue
            change = {p.ref: (x, y, (r + rnd.choice((90, 180, 270))) % 360)}
        elif kind < 0.85 or len(movable) < 2:
            sigma = (radius if mode == "local" else span * 0.25) * (1 - 0.8 * frac)
            nx = round((x + rnd.gauss(0, sigma)) / grid) * grid if p.axis != "y" else x
            ny = round((y + rnd.gauss(0, sigma)) / grid) * grid if p.axis != "x" else y
            if mode == "local":
                ox, oy = origin[p.ref]
                nx = min(max(nx, ox - radius), ox + radius)
                ny = min(max(ny, oy - radius), oy + radius)
            change = {p.ref: (nx, ny, r)}
            if p.group:
                dx, dy = nx - x, ny - y
                change = {m: (ev.pos[m][0] + dx, ev.pos[m][1] + dy, ev.pos[m][2]) for m in p.group}
        else:
            # swap with a part of similar size (keeps the overlap picture similar)
            # never swap with a rigid-group member or an edge slider: that would tear it out of its block
            cands = [q for q in movable if q is not p and q.front == p.front and not q.group and not q.axis
                     and 0.7 < size[q.ref] / max(size[p.ref], 1e-9) < 1.4]
            if not cands:
                continue
            q = rnd.choice(cands)
            qx, qy, qr = ev.pos[q.ref]
            if mode == "local" and (math.dist((qx, qy), origin[p.ref]) > radius
                                    or math.dist((x, y), origin[q.ref]) > radius):
                continue
            change = {p.ref: (qx, qy, r), q.ref: (x, y, qr)}
        legality = ev.overlap + ev.outside
        token = ev.try_move(change)
        if ev.overlap + ev.outside > legality + 1e-6:
            # Hard constraint: never create or grow courtyard overlaps or overhang.
            ev.undo(token)
            continue
        new_cost = ev.cost()
        delta = new_cost - cost
        if delta <= 0 or rnd.random() < math.exp(-delta / temp):
            cost = new_cost
            accepted += 1
            if cost < best_cost:
                best_cost, best_pos = cost, dict(ev.pos)
        else:
            ev.undo(token)
        if log_every and (it + 1) % log_every == 0:
            print(f"  it {it + 1:>7}  cost {cost:12.0f}  best {best_cost:12.0f}  accepted {accepted}  "
                  f"{time.time() - start:5.0f}s", flush=True)
    # restore best
    changes = {ref: best_pos[ref] for ref in best_pos if best_pos[ref] != ev.pos[ref]}
    if changes:
        ev.try_move(changes)


def write_placement(board: Board, ev: Evaluator, out: Path) -> list[dict]:
    moves = []
    for p in board.parts.values():
        x, y, r = ev.pos[p.ref]
        if (x, y, r) == (p.x, p.y, p.rot):
            continue
        node = p.place_node
        node[2] = Atom(_fmt(x))
        node[3] = Atom(_fmt(y))
        node[5] = Atom(_fmt(r))
        moves.append({"ref": p.ref, "from": [p.x / board.unit_mm, p.y / board.unit_mm, p.rot],
                      "to": [round(x / board.unit_mm, 4), round(y / board.unit_mm, 4), r],
                      "side": "front" if p.front else "back"})
    dsn.write(board.tree, out)
    return moves


def _fmt(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".") if v != int(v) else str(int(v))


# ---- CLI ------------------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("analyze")
    a.add_argument("board", type=Path)
    o = sub.add_parser("optimize")
    o.add_argument("board", type=Path)
    o.add_argument("-o", "--output", type=Path)
    o.add_argument("--mode", choices=["rotate", "local", "global"], default="local")
    o.add_argument("--radius", type=float, default=5.0, help="max move in mm for --mode local")
    o.add_argument("--iterations", type=int, default=20000)
    o.add_argument("--grid", type=float, default=0.25, help="placement grid in mm")
    o.add_argument("--seed", type=int, default=1)
    o.add_argument("--max-rotate-pins", type=int, default=4, help="only parts with at most this many pins turn")
    o.add_argument("--fix", default="", help="comma-separated refs that must not move")
    o.add_argument("--free", default="", help="comma-separated refs that may move although auto-fixed")
    for sp in (a, o):
        sp.add_argument("--channel", type=float, default=0.5,
                        help="mm kept free between pads of parts without courtyard (routing channel)")
        sp.add_argument("--w-cross", type=float, default=0.3)
        sp.add_argument("--w-rudy", type=float, default=0.1)
        sp.add_argument("--w-overlap", type=float, default=20.0)
    ns = ap.parse_args()

    board = load(ns.board, ns.channel)
    fix = {s.strip() for s in getattr(ns, "fix", "").split(",") if s.strip()}
    free = {s.strip() for s in getattr(ns, "free", "").split(",") if s.strip()}
    default_fixed(board, fix, free)
    ev = Evaluator(board, ns.w_cross, ns.w_rudy, ns.w_overlap)
    before = ev.metrics()
    movable = sum(1 for p in board.parts.values() if not p.fixed)
    print(f"{ns.board.name}: {len(board.parts)} parts ({movable} movable), {len(board.nets)} nets")
    print("  before:", before)
    if ns.cmd == "analyze":
        return
    t = time.time()
    anneal(ev, ns.mode, ns.radius, ns.iterations, ns.seed, ns.grid, max(1, ns.iterations // 10),
           ns.max_rotate_pins)
    after = ev.metrics()
    print("  after: ", after, f"({time.time() - t:.0f}s)")
    out = ns.output or ns.board.with_name(ns.board.stem + f"_placed_{ns.mode}.dsn")
    moves = write_placement(board, ev, out)
    report = {"mode": ns.mode, "before": before, "after": after, "moves": moves,
              "fixed": sorted(p.ref for p in board.parts.values() if p.fixed)}
    out.with_suffix(".moves.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    print(f"  {len(moves)} parts changed -> {out}")


if __name__ == "__main__":
    main()
