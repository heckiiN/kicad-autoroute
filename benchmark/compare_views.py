"""Side-by-side views of the hand-routed demo boards and kicad-autoroute's results.

  python benchmark/compare_views.py benchmark/results/<run> [--standard-from other_run/results.json]

Exports the copper layers and the outline of every board in the run (original and result) as SVG
and writes a self-contained compare.html (images embedded) next to results.json: one row per board with both layouts, the DRC
counts and simple layout metrics (vias, track length, corners per mm).
"""
from __future__ import annotations

import base64
import html
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kicad_autoroute import pipeline, tools  # noqa: E402
from benchmark.run_benchmark import ALL_BOARDS, demos_dir  # noqa: E402

LAYERS = "B.Cu,F.Cu,Edge.Cuts"


def export_svg(board: Path, out: Path) -> str:
    res = subprocess.run([str(tools.kicad_cli()), "pcb", "export", "svg", "--layers", LAYERS, "--mode-single",
                          "--fit-page-to-board", "--exclude-drawing-sheet", "--check-zones", "-o", str(out), str(board)],
                         capture_output=True, text=True, timeout=600)
    if res.returncode != 0 or not out.exists():
        raise RuntimeError(res.stderr[-500:])
    return out.read_text(encoding="utf-8")


def main() -> int:
    run = Path(sys.argv[1]).resolve()
    rows = json.loads((run / "results.json").read_text(encoding="utf-8"))
    if "--standard-from" in sys.argv:  # plugin-workflow numbers from an earlier run
        other = json.loads(Path(sys.argv[sys.argv.index("--standard-from") + 1]).read_text(encoding="utf-8"))
        for name, row in rows.items():
            if "standard" not in row and "standard" in other.get(name, {}):
                row["standard"] = other[name]["standard"]
    demos = demos_dir()
    img = run / "svg"
    img.mkdir(exist_ok=True)
    parts = []
    for name, row in rows.items():
        pipe = row.get("pipeline") or {}
        finals = sorted((run / name / "pipeline" / "result").glob("*.kicad_pcb"))
        final = finals[0] if finals else Path(pipe.get("board", ""))
        human = demos / ALL_BOARDS[name]
        metrics, uri = {}, {}
        for tag, board in (("human", human), ("ai", final)):
            svg = export_svg(board, img / f"{name}_{tag}.svg")
            uri[tag] = "data:image/svg+xml;base64," + base64.b64encode(svg.encode("utf-8")).decode("ascii")
            metrics[tag] = json.loads(pipeline.steps("stats", board).splitlines()[-1])
        ref, std = row.get("reference", {}), row.get("standard", {})

        def cell(r):
            return "–" if not r or "unconnected" not in r else f"{r['unconnected']} / {r['error_count']}"
        parts.append(f"""
<section>
  <h2>{html.escape(name)}</h2>
  <table>
    <tr><th></th><th>hand-routed</th><th>Freerouting plugin</th><th>kicad-autoroute</th></tr>
    <tr><td>unconnected / DRC errors</td><td>{cell(ref)}</td><td>{cell(std)}</td><td>{cell(pipe)}</td></tr>
    <tr><td>vias</td><td>{metrics['human']['vias']}</td><td>{std.get('vias', '–')}</td><td>{metrics['ai']['vias']}</td></tr>
    <tr><td>track length (mm)</td><td>{metrics['human']['track_mm']}</td><td>{std.get('track_mm', '–')}</td><td>{metrics['ai']['track_mm']}</td></tr>
  </table>
  <div class="pair">
    <figure><img src="{uri['human']}" alt="{name} hand-routed"><figcaption>hand-routed</figcaption></figure>
    <figure><img src="{uri['ai']}" alt="{name} kicad-autoroute"><figcaption>kicad-autoroute</figcaption></figure>
  </div>
</section>""")
    page = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Layout Comparison</title>
<style>
body {{ font: 15px/1.5 system-ui, sans-serif; margin: 0 auto; max-width: 1400px; padding: 16px; background: #fff; color: #222; }}
table {{ border-collapse: collapse; margin: 8px 0; }} td, th {{ border: 1px solid #ccc; padding: 4px 10px; text-align: right; }}
td:first-child {{ text-align: left; }} .pair {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }}
figure {{ margin: 0; }} img {{ width: 100%; height: auto; background: #f4f4f4; border: 1px solid #ddd; }}
@media (max-width: 800px) {{ .pair {{ grid-template-columns: 1fr; }} }}
</style></head><body><h1>Hand-routed vs. kicad-autoroute</h1>
<p>Copper (top red, bottom blue) and outline of KiCad's demo boards. Same parts, same placement; only the routing differs.</p>
{''.join(parts)}</body></html>"""
    (run / "compare.html").write_text(page, encoding="utf-8")
    print(run / "compare.html")
    return 0


if __name__ == "__main__":
    sys.exit(main())
