"""Generality benchmark: route KiCad's own demo boards with kicad-autoroute (default settings, no
per-board tuning) and, optionally, with the plain Freerouting plugin workflow, then compare.

  python benchmark/run_benchmark.py                         # pipeline on all boards
  python benchmark/run_benchmark.py --standard STOCK.jar    # plus the plugin workflow with a stock jar
  python benchmark/run_benchmark.py --boards ecc83 sonde    # a subset

The demo boards are read from KiCad's installation (never modified); every result is counted with
KiCad's DRC after refilling zones. Output: benchmark/results/<date>/results.md and results.json.
"""
from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kicad_autoroute import cli, pipeline, tools  # noqa: E402

# name -> path below KiCad's demos directory (boards that come routed, so there is a reference)
BOARDS = {
    "ecc83": "ecc83/ecc83-pp.kicad_pcb",
    "sonde": "sonde xilinx/sonde xilinx.kicad_pcb",
    "pic_programmer": "pic_programmer/pic_programmer.kicad_pcb",
    "interf_u": "interf_u/interf_u.kicad_pcb",
    "multichannel": "multichannel/multichannel_mixer.kicad_pcb",
    "coldfire": "kit-dev-coldfire-xilinx_5213/kit-dev-coldfire-xilinx_5213.kicad_pcb",
}
# Held out: demo boards no setting was ever tuned on; run with --holdout to check that the pipeline
# is not just fitted to the boards above. (vme-wren and jetson are too large for a routine run; the
# microwave and NFC antenna demos are RF structures, not routing jobs.)
HOLDOUT = {
    "stickhub": "stickhub/StickHub.kicad_pcb",
    "complex_hierarchy": "complex_hierarchy/complex_hierarchy.kicad_pcb",
    "video": "video/video.kicad_pcb",
    "one_air_max": "openair-max/One-Air-Max.kicad_pcb",
    "cm5_minima": "cm5_minima/CM5_MINIMA_3.kicad_pcb",
    "feather": "royalblue54L_feather/RoyalBlue54L-Feather.kicad_pcb",
}
ALL_BOARDS = {**BOARDS, **HOLDOUT}


def demos_dir() -> Path:
    b = tools.kicad_bin()
    for d in (b.parent / "share" / "kicad" / "demos",                  # Windows, Linux /usr
              b.parent / "SharedSupport" / "demos",                      # macOS app bundle
              Path("/usr/share/kicad/demos"), Path("/usr/local/share/kicad/demos")):
        if d.is_dir():
            return d
    raise tools.ToolError("KiCad demo boards not found; pass --demos")


def standard(board: Path, out: Path, jar: Path, timeout_s: int) -> dict:
    """The Freerouting plugin workflow: strip tracks (zones stay), plain KiCad DSN export, stock
    Freerouting with up to 100 passes, SES back into KiCad, DRC."""
    t0 = time.time()
    pcb = pipeline.copy_project(board, out, board.stem.replace(" ", "_"))
    pipeline.steps("strip", pcb)
    dsn, ses = out / "board.dsn", out / "board.ses"
    pipeline.steps("export", pcb, dsn, "--raw")
    try:
        with open(out / "freerouting.log", "w") as fh:
            subprocess.run([tools.java(), "-Xmx2g", "-jar", str(jar), "-de", str(dsn), "-do", str(ses), "-mp", "100",
                            "--gui.enabled=false", f"--user_data_path={out / 'ud'}"],
                           stdout=fh, stderr=subprocess.STDOUT, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        pass
    if not ses.exists():
        return {"error": f"no result within {timeout_s} s", "time_s": round(time.time() - t0)}
    pipeline.steps("import", pcb, ses, pcb)
    res = pipeline.drc(pcb)
    res.update(json.loads(pipeline.steps("stats", pcb).splitlines()[-1]))
    res["time_s"] = round(time.time() - t0)
    return res


def cell(r: dict | None) -> str:
    if not r:
        return "–"
    if "error" in r:
        return r["error"]
    return f"{r['unconnected']} / {r['error_count']}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boards", nargs="*", choices=sorted(ALL_BOARDS), help="subset of boards")
    ap.add_argument("--holdout", action="store_true", help="the held-out boards instead of the tuning set")
    ap.add_argument("--demos", type=Path, help="KiCad demos directory (default: found from the KiCad install)")
    ap.add_argument("--standard", type=Path, metavar="JAR", help="also run the plugin workflow with this stock jar")
    ap.add_argument("--standard-timeout", type=int, default=3600, help="seconds for a standard run")
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--timeout", default="00:20:00")
    ap.add_argument("-o", "--output", type=Path, default=ROOT / "benchmark" / "results" / time.strftime("%Y%m%d-%H%M"))
    ns = ap.parse_args()

    demos = ns.demos or demos_dir()
    out = ns.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows = {}
    for name in ns.boards or (HOLDOUT if ns.holdout else BOARDS):
        board = demos / ALL_BOARDS[name]
        print(f"=== {name}: {board}", flush=True)
        row = rows[name] = {}
        if ns.standard:
            row["standard"] = standard(board, out / name / "standard", ns.standard.resolve(), ns.standard_timeout)
            print(f"standard: {cell(row['standard'])}", flush=True)
        args = argparse.Namespace(target=str(board), output=str(out / name / "pipeline"), seeds=ns.seeds,
                                  workers=ns.workers, timeout=ns.timeout, plane_net=None, no_stitch=False,
                                  no_escape=False, strip_locked=False, renders=False)
        cfg, base, label = cli.build_config(args)
        try:
            report = pipeline.run(cfg, base, label)
        except (Exception, SystemExit) as exc:   # one failing board must not end the whole benchmark
            print(f"pipeline failed: {str(exc)[-500:]}", flush=True)
            rows.pop(name)
            continue
        row["reference"] = report["reference"]
        row["pipeline"] = dict(report["best"], time_s=report["timing_s"])
        print(f"pipeline: {cell(row['pipeline'])} in {report['timing_s']} s", flush=True)
        (out / "results.json").write_text(json.dumps(rows, indent=1, default=str), encoding="utf-8")

    lines = ["# kicad-autoroute benchmark", "",
             f"{time.strftime('%Y-%m-%d')}, KiCad {tools.kicad_bin()}, {platform.platform()}, "
             f"{ns.seeds} seeds x {ns.timeout}", "",
             "Cells: unconnected / DRC errors (KiCad DRC after zone refill).", "",
             "| Board | Original (hand-routed) | Freerouting plugin workflow | kicad-autoroute | time |",
             "|---|---|---|---|---|"]
    for name, row in rows.items():
        lines.append(f"| {name} | {cell(row.get('reference'))} | {cell(row.get('standard'))} | "
                     f"{cell(row.get('pipeline'))} | {row['pipeline']['time_s'] / 60:.1f} min |")
    (out / "results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
