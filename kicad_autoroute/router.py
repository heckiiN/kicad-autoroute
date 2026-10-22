"""Run Freerouting headless on Specctra DSN files, several jobs side by side.

Each run uses one router thread and no optimizer (routing quality only), so several seeds can run
in parallel. Results come from Freerouting's --router.result_json manifest plus the pass lines in
its log.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path

from . import tools

PASS_RE = re.compile(r"Auto-routing pass #(\d+) .*? with score ([\d.]+) \((\d+) unrouted and (\d+) violations\)")
STAGE_RE = re.compile(
    r"Auto-routing stage completed:.*?completed in ([\d.]+) seconds.*?using ([\d.]+) total CPU seconds"
)


@dataclass
class RunResult:
    board: str
    config: str
    unrouted: int | None = None
    violations: int | None = None
    vias: int | None = None
    length_mm: float | None = None
    score: float | None = None
    passes: int | None = None
    route_s: float | None = None
    cpu_s: float | None = None
    wall_s: float = 0.0
    final_state: str = ""
    ses: str = ""
    error: str = ""
    args: list[str] = field(default_factory=list)

    def key(self):
        """Sort key: fewer unrouted, then fewer violations, then fewer vias, then shorter."""
        big = 10**9
        return (
            self.unrouted if self.unrouted is not None else big,
            self.violations if self.violations is not None else big,
            self.vias if self.vias is not None else big,
            self.length_mm if self.length_mm is not None else big,
        )


def run_once(board: Path, config: str, extra: list[str], out_dir: Path, timeout: str, max_passes: int) -> RunResult:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{board.stem}__{config}"
    ses = out_dir / f"{stem}.ses"
    manifest = out_dir / f"{stem}.json"
    log = out_dir / f"{stem}.log"
    user_dir = out_dir / f"ud_{stem}"
    cmd = [
        tools.java(), "-Xmx2g", "-Dsun.stdout.buffered=false", "-jar", str(tools.freerouting_jar()),
        f"--user_data_path={user_dir}", "--gui.enabled=false", "--api_server.enabled=false",
        "--mcp_server.enabled=false", "--router.max_threads=1",
        "-de", str(board), "-do", str(ses), "-dct", "0",
        f"--router.result_json={manifest}", "--router.fanout.enabled=true",
        "--router.optimizer.enabled=false", f"--router.job_timeout={timeout}",
        f"--router.autorouter.max_passes={max_passes}",
    ] + extra
    # "env:NAME=value" entries are environment settings of the patched router, not options
    env = dict(os.environ)
    for a in [a for a in extra if a.startswith("env:")]:
        name, _, value = a[4:].partition("=")
        env[name] = value
    cmd = [c for c in cmd if not str(c).startswith("env:")]
    res = RunResult(board=board.stem, config=config, args=extra)
    h, m, s = (int(x) for x in timeout.split(":"))
    hard_timeout = h * 3600 + m * 60 + s + 180
    start = time.time()
    try:
        with open(log, "w", encoding="utf-8", errors="replace") as fh:
            subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT, timeout=hard_timeout, check=False, env=env)
    except subprocess.TimeoutExpired:
        res.error = "hard timeout"
    res.wall_s = round(time.time() - start, 1)
    parse_outputs(res, log, manifest, ses)
    shutil.rmtree(user_dir, ignore_errors=True)
    return res


def parse_outputs(res: RunResult, log: Path, manifest: Path, ses: Path) -> None:
    text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else ""
    passes = PASS_RE.findall(text)
    if passes:
        res.passes = int(passes[-1][0])
    stage = STAGE_RE.search(text)
    if stage:
        res.route_s, res.cpu_s = float(stage.group(1)), float(stage.group(2))
    if manifest.exists():
        try:
            data = json.loads(manifest.read_text(encoding="utf-8"))
            # Top-level statistics describe the board written to the .ses file. The autorouter
            # phase only holds Freerouting's last internal attempt, not the best one it kept.
            stats = data.get("board_statistics") \
                or (data.get("phases", {}).get("autorouter", {}).get("after") or {}).get("board_statistics") or {}
            res.unrouted = stats.get("connections", {}).get("incomplete_count")
            res.violations = stats.get("clearance_violations", {}).get("total_count")
            res.vias = stats.get("vias", {}).get("total_count")
            res.length_mm = stats.get("traces", {}).get("total_length_mm")
            res.score = data.get("normalized_score")
            res.final_state = data.get("final_state", "")
            if res.cpu_s is None:
                res.cpu_s = data.get("resource_usage", {}).get("cpu_time")
        except (OSError, ValueError) as exc:
            res.error = f"bad manifest: {exc}"
    elif not res.error:
        res.error = "no manifest (see log)"
    if ses.exists():
        res.ses = str(ses)


def run_matrix(jobs, workers: int, out_dir: Path, timeout: str, max_passes: int, label: str) -> list[RunResult]:
    results: list[RunResult] = []
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_once, b, c, a, out_dir, timeout, max_passes): (b, c) for b, c, a in jobs}
        for i, fut in enumerate(as_completed(futures), 1):
            r = fut.result()
            results.append(r)
            print(f"[{label} {i}/{len(jobs)}] {r.board:<28} {r.config:<14} unrouted={r.unrouted} viol={r.violations} "
                  f"vias={r.vias} len={r.length_mm} passes={r.passes} route_s={r.route_s} {r.error}", flush=True)
    return results
