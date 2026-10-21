"""Command line: route a KiCad board in one call.

  kicad-autoroute board.kicad_pcb                 route with defaults, result in ./autoroute-<name>/
  kicad-autoroute board.kicad_pcb -o out --seeds 6 --timeout 00:30:00
  kicad-autoroute my_board.toml                   all settings from a config file (configs/example.toml)
  kicad-autoroute check                           show which KiCad / Java / Freerouting it found

The source project is only read; all work happens on copies in the output directory.
"""
from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path

from . import kicad_classes, pipeline, tools

DEFAULTS = {
    "strip": {"enabled": True, "keep_locked": True, "zones": False},
    "placement": {"mode": "none"},
    "escape": {"enabled": True},
    "stitch": {"enabled": "auto", "net": "GND"},
    "freerouting": {"seeds": 4, "workers": 4, "timeout": "00:20:00"},
    "finish": {"candidates": 3},
    "report": {"renders": False},
}


def merged(base: dict, override: dict) -> dict:
    out = {k: (dict(v) if isinstance(v, dict) else v) for k, v in base.items()}
    for k, v in override.items():
        out[k] = merged(out.get(k, {}), v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def build_config(ns) -> tuple[dict, Path, str]:
    target = Path(ns.target)
    if target.suffix == ".toml":
        cfg = merged(DEFAULTS, tomllib.loads(target.read_text(encoding="utf-8")))
        base, label = target.resolve().parent, str(target.resolve())
    else:
        cfg = merged(DEFAULTS, {"board": str(target.resolve())})
        base, label = Path.cwd(), f"cli: {target}"
    if ns.output:
        cfg["work_dir"] = str(Path(ns.output).resolve())
    fr = cfg["freerouting"]
    for key in ("seeds", "workers", "timeout"):
        if getattr(ns, key) is not None:
            fr[key] = getattr(ns, key)
    if ns.plane_net:
        cfg["stitch"]["net"] = ns.plane_net
    if ns.no_stitch:
        cfg["stitch"]["enabled"] = False
    if ns.no_escape:
        cfg["escape"]["enabled"] = False
    if ns.strip_locked:
        cfg["strip"]["keep_locked"] = False
    if ns.renders:
        cfg["report"]["renders"] = True
    if cfg["stitch"]["enabled"] == "auto":
        # stitching needs a plane net with a pour; use it when the board has that net
        board = (base / cfg["board"]).resolve()
        cfg["stitch"]["enabled"] = cfg["stitch"]["net"] in kicad_classes.board_net_names(str(board))
    return cfg, base, label


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="kicad-autoroute", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help="board (.kicad_pcb), config (.toml) or 'check'")
    ap.add_argument("-o", "--output", help="work/output directory (default: ./autoroute-<board name>)")
    ap.add_argument("--seeds", type=int, help="Freerouting runs with different item orders (default 4)")
    ap.add_argument("--workers", type=int, help="runs in parallel (default 4; each needs up to 2 GB RAM)")
    ap.add_argument("--timeout", help="time limit per Freerouting run, HH:MM:SS (default 00:20:00)")
    ap.add_argument("--plane-net", help="net poured as plane, used for stitching vias (default GND)")
    ap.add_argument("--no-stitch", action="store_true", help="no stitching vias into the plane")
    ap.add_argument("--no-escape", action="store_true", help="no escape stubs for wide nets on fine pads")
    ap.add_argument("--strip-locked", action="store_true", help="also remove locked tracks before routing")
    ap.add_argument("--renders", action="store_true", help="3D renders before/after (slow)")
    ns = ap.parse_args(argv)

    if ns.target == "check":
        report = tools.check()
        for name, value in report.items():
            print(f"{name:16} {value}")
        return 1 if any(v.startswith("MISSING") for v in report.values()) else 0

    if not Path(ns.target).exists():
        ap.error(f"{ns.target} not found")
    try:
        cfg, base, label = build_config(ns)
        report = pipeline.run(cfg, base, label)
    except tools.ToolError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    best = report["best"]
    print(f"\nresult: {report['result']}\n  unconnected {best['unconnected']}, DRC errors {best['error_count']}, "
          f"vias {best['vias']}, track {best['track_mm']} mm, {report['timing_s']} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
