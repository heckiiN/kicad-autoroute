"""Human-level scorecard: every routed demo board against its hand-routed original.

  python benchmark/scorecard.py benchmark/results/<run>

Criteria per board (all must hold for "human level"):
  complete      0 unconnected and 0 DRC errors
  buses         share of bus length laid alongside its neighbour >= the human's - 0.05
  vias          <= 1.1 x the human's + 2
  main side     (2-layer boards) share of copper on the main side >= the human's - 0.10
Writes scorecard.md and scorecard.json into the run directory.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kicad_autoroute import pipeline  # noqa: E402
from benchmark.run_benchmark import BOARDS, demos_dir  # noqa: E402


def measure(board: Path) -> dict:
    stats = json.loads(pipeline.steps("stats", board).splitlines()[-1])
    bus = json.loads(pipeline.kpy(pipeline.HERE / "buses.py", board, "--json").splitlines()[-1])
    layers = stats.get("layer_mm") or {}
    total = sum(layers.values())
    return {"vias": stats["vias"], "track_mm": stats["track_mm"], "parallel": bus.get("parallel"),
            "buses": bus.get("buses", 0), "main_share": round(max(layers.values()) / total, 2) if total else None,
            "copper_layers": len(layers)}


def main() -> int:
    run = Path(sys.argv[1]).resolve()
    rows = json.loads((run / "results.json").read_text(encoding="utf-8"))
    demos = demos_dir()
    card = {}
    for name, row in rows.items():
        finals = sorted((run / name / "pipeline" / "result").glob("*.kicad_pcb"))
        if not finals:
            continue
        human, ai = measure(demos / BOARDS[name]), measure(finals[0])
        p = row["pipeline"]
        checks = {"complete": p["unconnected"] == 0 and p["error_count"] == 0,
                  "vias": ai["vias"] <= 1.1 * human["vias"] + 2}
        if human["buses"]:
            checks["buses"] = (ai["parallel"] or 0) >= (human["parallel"] or 0) - 0.05
        if human["copper_layers"] <= 2 and human["main_share"] is not None:
            checks["main side"] = (ai["main_share"] or 0) >= human["main_share"] - 0.10
        card[name] = {"human": human, "ai": ai, "open": p["unconnected"], "errors": p["error_count"],
                      "checks": checks, "human_level": all(checks.values())}
    lines = ["| Board | open / errors | vias (human / ai) | buses parallel (human / ai) | main side (human / ai) | human level |",
             "|---|---|---|---|---|---|"]
    for name, c in card.items():
        h, a, ch = c["human"], c["ai"], c["checks"]

        def mark(key):
            return "" if key not in ch else (" ok" if ch[key] else " MISS")
        lines.append(f"| {name} | {c['open']} / {c['errors']}{mark('complete')} | {h['vias']} / {a['vias']}{mark('vias')} | "
                     f"{h['parallel'] if h['buses'] else '–'} / {a['parallel'] if h['buses'] else '–'}{mark('buses')} | "
                     f"{h['main_share'] if 'main side' in ch else '–'} / {a['main_share'] if 'main side' in ch else '–'}{mark('main side')} | "
                     f"{'yes' if c['human_level'] else 'no'} |")
    reached = sum(c["human_level"] for c in card.values())
    lines.append(f"\n{reached} of {len(card)} boards at human level.")
    (run / "scorecard.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (run / "scorecard.json").write_text(json.dumps(card, indent=1), encoding="utf-8")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
