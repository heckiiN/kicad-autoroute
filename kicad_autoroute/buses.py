"""Buses of a board and how tidily they are routed (run with KiCad's python).

  <kicad-python> kicad_autoroute/buses.py board.kicad_pcb [--json]

A bus is found from the netlist alone: three or more nets that each connect a pad of one footprint
to a pad of another footprint, where the pads on each side sit next to each other (a run of
neighbouring pins). Data lines from a controller to a connector are the typical case.

For a routed board every bus gets a parallelism score: the share of its track length that runs
alongside its neighbour in the bus (same layer, same direction, close by). A hand-laid bus scores
high; nets of a bus routed one by one wander apart and score low.
"""
from __future__ import annotations

import argparse
import json
import math

import pcbnew

TO = pcbnew.ToMM


def pad_runs(pads: list, max_gap: float) -> list[list]:
    """Split pads into runs of neighbours: consecutive pads (along the line they lie on) at most
    max_gap apart."""
    if len(pads) < 2:
        return [pads]
    xs = [p[1] for p in pads]
    ys = [p[2] for p in pads]
    horizontal = (max(xs) - min(xs)) >= (max(ys) - min(ys))
    pads = sorted(pads, key=lambda p: (p[1], p[2]) if horizontal else (p[2], p[1]))
    runs, cur = [], [pads[0]]
    for p in pads[1:]:
        if math.hypot(p[1] - cur[-1][1], p[2] - cur[-1][2]) <= max_gap:
            cur.append(p)
        else:
            runs.append(cur)
            cur = [p]
    runs.append(cur)
    return runs


def find_buses(b, min_nets: int = 3) -> list[dict]:
    """Groups of nets running pad-to-pad between two footprints from neighbouring pins."""
    by_net: dict[str, list] = {}
    for f in b.GetFootprints():
        for p in f.Pads():
            net = p.GetNetname()
            if net:
                by_net.setdefault(net, []).append((f.GetReference(), TO(p.GetPosition().x), TO(p.GetPosition().y),
                                                   TO(min(p.GetSizeX(), p.GetSizeY()))))
    links: dict[tuple, list] = {}
    for net, pads in by_net.items():
        if not 2 <= len(pads) <= 4:     # wide nets (power, ground) are not bus members
            continue
        refs = sorted({p[0] for p in pads})
        for i, ra in enumerate(refs):
            for rb in refs[i + 1:]:
                pa = next(p for p in pads if p[0] == ra)
                pb = next(p for p in pads if p[0] == rb)
                links.setdefault((ra, rb), []).append((net, pa, pb))
    buses = []
    for (ra, rb), conns in links.items():
        if len(conns) < min_nets:
            continue
        pitch = sorted(math.hypot(c1[1][1] - c2[1][1], c1[1][2] - c2[1][2])
                       for c1 in conns for c2 in conns if c1 is not c2)[0] if len(conns) > 1 else 0
        side_a = [(c[0], c[1][1], c[1][2]) for c in conns]
        for run in pad_runs(side_a, max(pitch * 1.6, 0.5)):
            if len(run) < min_nets:
                continue
            members = [c for c in conns if c[0] in {r[0] for r in run}]
            # the other side must form a run as well, else it is just a fan of unrelated nets
            side_b = [(c[0], c[2][1], c[2][2]) for c in members]
            pitch_b = sorted(math.hypot(x[1] - y[1], x[2] - y[2]) for x in side_b for y in side_b if x is not y)[0]
            for run_b in pad_runs(side_b, max(pitch_b * 1.6, 0.5)):
                if len(run_b) >= min_nets:
                    order = [r[0] for r in run]
                    nets = [n for n in order if n in {r[0] for r in run_b}]
                    buses.append({"from": ra, "to": rb, "nets": nets, "pitch": round(pitch, 3)})
    return buses


def _seg_dist(px, py, s):
    sx, sy, ex, ey = s
    dx, dy = ex - sx, ey - sy
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / L))
    return math.hypot(px - sx - t * dx, py - sy - t * dy)


def parallelism(b, bus: dict, near: float = 2.5, step: float = 0.25) -> tuple[float, float]:
    """(share of track length running alongside a bus neighbour, total length) for one bus."""
    segs: dict[str, list] = {n: [] for n in bus["nets"]}
    for t in b.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.GetNetname() in segs:
            segs[t.GetNetname()].append((t.GetLayer(), (TO(t.GetStart().x), TO(t.GetStart().y),
                                                        TO(t.GetEnd().x), TO(t.GetEnd().y))))
    nets = bus["nets"]
    together = total = 0.0
    for i, net in enumerate(nets):
        neighbours = [segs[nets[j]] for j in (i - 1, i + 1) if 0 <= j < len(nets)]
        for layer, s in segs[net]:
            length = math.hypot(s[2] - s[0], s[3] - s[1])
            if length == 0:
                continue
            ux, uy = (s[2] - s[0]) / length, (s[3] - s[1]) / length
            n = max(1, int(length / step))
            for k in range(n):
                px, py = s[0] + ux * (k + 0.5) * length / n, s[1] + uy * (k + 0.5) * length / n
                total += length / n
                for other in neighbours:
                    hit = False
                    for ol, os_ in other:
                        if ol != layer:
                            continue
                        ol_len = math.hypot(os_[2] - os_[0], os_[3] - os_[1])
                        if ol_len == 0:
                            continue
                        cross = abs(ux * (os_[3] - os_[1]) - uy * (os_[2] - os_[0])) / ol_len
                        if cross < 0.15 and _seg_dist(px, py, os_) <= near:
                            hit = True
                            break
                    if hit:
                        together += length / n
                        break
    return (together / total if total else 0.0), total


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("board")
    ap.add_argument("--json", action="store_true")
    ns = ap.parse_args()
    b = pcbnew.LoadBoard(ns.board)
    buses = find_buses(b)
    weighted = length = 0.0
    for bus in buses:
        share, total = parallelism(b, bus)
        bus["parallel"], bus["length_mm"] = round(share, 2), round(total, 1)
        weighted += share * total
        length += total
    summary = {"buses": len(buses), "bus_nets": sum(len(x["nets"]) for x in buses),
               "parallel": round(weighted / length, 2) if length else None, "detail": buses}
    if ns.json:
        print(json.dumps(summary))
    else:
        for bus in buses:
            print(f"{bus['from']} -> {bus['to']}: {len(bus['nets'])} nets, parallel {bus.get('parallel')}, "
                  f"{bus.get('length_mm')} mm  {bus['nets'][:4]}...")
        print(f"buses {summary['buses']}, nets {summary['bus_nets']}, parallel overall {summary['parallel']}")


if __name__ == "__main__":
    main()
