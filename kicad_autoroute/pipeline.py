"""Autorouting pipeline for KiCad boards: rule-driven preparation, Freerouting on several seeds,
KiCad-DRC scoring and a finishing pass. Normally started through the CLI (`kicad-autoroute`).

Steps, each optional through the config (see configs/example.toml):
  1. copy     board + project + rules into <work>/board (the source project is only read)
  2. strip    remove existing tracks/vias (keep locked ones if strip.keep_locked)
  3. place    optional placement change: "compact" shrinks the outline and re-places parts
              (anchored parts follow their edge, functional groups move as rigid blocks)
  4. escape   short narrow stubs + vias for wide-net pads that are narrower than the net class
  5. stitch   stitching vias from SMD pads of the plane net (e.g. GND) into the plane
  6. zones    pre-route zones (e.g. GND plane on an inner layer)
  7. route    Freerouting on N item-order seeds in parallel
  8. score    import each result into KiCad, add post-route zones, fill, DRC: rank by
              unconnected, DRC errors, vias, length
  9. finish   best candidates: replace necked tracks, last-mile router for what is still open, drop
              dangling leftovers and too-close vias, silkscreen clean-up, DRC
 10. report   best finished board, report.json, 3D renders before/after
Results: <work>/result/<name>.kicad_pcb (+ .kicad_pro/.kicad_dru), report.json, renders.
"""
from __future__ import annotations

import json
import math
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from . import dsn_fix, render_board, router, tools

HERE = Path(__file__).resolve().parent


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def kpy(script: Path, *args: str, timeout: int = 3600, fatal: bool = True) -> str:
    try:
        res = subprocess.run([str(tools.kicad_python()), str(script), *map(str, args)], capture_output=True, text=True,
                             timeout=timeout)
    except subprocess.TimeoutExpired:
        if fatal:
            raise SystemExit(f"{script.name} {' '.join(map(str, args))} timed out after {timeout} s")
        return f"{script.name}: timed out after {timeout} s, skipped"

    def clean(text: str) -> str:
        return "\n".join(l for l in text.splitlines() if "image handler" not in l and "memory leak" not in l).strip()
    # KiCad's messages (stderr) first: callers read their result from the last line of stdout
    out = "\n".join(x for x in (clean(res.stderr), clean(res.stdout)) if x)
    if res.returncode != 0:
        if not fatal:
            err = clean(res.stderr) or out
            last = err.splitlines()[-1] if err else "no output"
            return f"{script.name}: failed, skipped ({last})"
        raise SystemExit(f"{script.name} {' '.join(map(str, args))} failed:\n{out[-2000:]}")
    return out.strip()


def steps(*args: str) -> str:
    return kpy(HERE / "kicad_steps.py", *args)


def copy_project(src: Path, dst_dir: Path, name: str) -> Path:
    dst_dir.mkdir(parents=True, exist_ok=True)
    for ext in (".kicad_pcb", ".kicad_pro", ".kicad_dru"):
        f = src.with_suffix(ext)
        if f.exists():
            shutil.copyfile(f, dst_dir / f"{name}{ext}")
    for extra in ("fp-lib-table", "sym-lib-table"):
        if (src.parent / extra).exists():
            shutil.copyfile(src.parent / extra, dst_dir / extra)
    # project-local footprint/3D libraries (${KIPRJMOD}/lib), needed for complete 3D renders
    lib = src.parent / "lib"
    if lib.is_dir() and not (dst_dir / "lib").exists():
        size = sum(f.stat().st_size for f in lib.rglob("*") if f.is_file())
        if size < 200 * 1024 * 1024:
            shutil.copytree(lib, dst_dir / "lib")
    return dst_dir / f"{name}.kicad_pcb"


def edge_clearance_um(pro: Path) -> int | None:
    try:
        rules = json.load(open(pro, encoding="utf-8"))["board"]["design_settings"]["rules"]
        v = rules.get("min_copper_edge_clearance")
        return int(round(v * 1000)) if v else None
    except (OSError, KeyError, ValueError):
        return None


def min_track_width(pro: Path, dru: Path) -> float:
    widths = []
    try:
        rules = json.load(open(pro, encoding="utf-8"))["board"]["design_settings"]["rules"]
        if rules.get("min_track_width"):
            widths.append(rules["min_track_width"])
    except (OSError, KeyError, ValueError):
        pass
    if dru.exists():
        import re
        for m in re.finditer(r"constraint\s+track_width\s+\(min\s+([\d.]+)\s*mm\)", dru.read_text(encoding="utf-8")):
            widths.append(float(m.group(1)))
    return min(widths) if widths else 0.15


def drc(pcb: Path) -> dict:
    report = pcb.with_name(pcb.stem + "_drc.json")
    subprocess.run([str(tools.kicad_cli()), "pcb", "drc", "--refill-zones", "--format", "json", "--severity-error",
                    "-o", str(report), str(pcb)], capture_output=True, text=True, timeout=3600)
    data = json.loads(report.read_text(encoding="utf-8"))
    errors: dict[str, int] = {}
    for v in data.get("violations", []):
        errors[v.get("type", "?")] = errors.get(v.get("type", "?"), 0) + 1
    return {"unconnected": len(data.get("unconnected_items", [])), "errors": errors,
            "error_count": sum(errors.values())}


def drc_all(pcb: Path) -> Path:
    report = pcb.with_name(pcb.stem + "_drc_all.json")
    subprocess.run([str(tools.kicad_cli()), "pcb", "drc", "--refill-zones", "--format", "json", "-o", str(report), str(pcb)],
                   capture_output=True, text=True, timeout=3600)
    return report


def finish(pcb: Path, min_w: float, original: Path, texts_from_original: bool, rounds: int = 6,
           buses_json: Path | None = None, unlock_against: Path | None = None, time_limit: float = 2400) -> list[str]:
    """Close what the router left: replace necked tracks, last-mile route open connections, drop
    dangling leftovers and too-close vias, repeat; then clean up the silkscreen. unlock_against: the
    user's board; if connections stay open, what the pipeline locked itself (bus bundles, stitching
    links) may be ripped up after all, the user's own locked routes stay.
    time_limit: seconds for all last-mile router calls together; on large boards every call could
    otherwise use its full budget in every round. The clean-up steps (dropping offenders, silkscreen)
    always run."""
    notes = []
    end = time.time() + time_limit

    def lastmile(*args) -> None:
        left = end - time.time()
        if left < 60:
            notes.append("lastmile: skipped, finishing time used up")
            return
        notes.append(kpy(HERE / "lastmile.py", pcb, *args, "--min-width", min_w or 0.15,
                         "--time-budget", round(min(900, left)), timeout=round(min(1500, left + 300)), fatal=False))

    if min_w:
        notes.append(steps("drop_narrow", pcb, min_w))

    last_state, stuck = [None], [False]

    def one_round() -> bool:
        report = pcb.with_name(pcb.stem + "_drc_err.json")
        subprocess.run([str(tools.kicad_cli()), "pcb", "drc", "--refill-zones", "--format", "json", "--severity-error",
                        "-o", str(report), str(pcb)], capture_output=True, text=True, timeout=3600)
        if json.loads(report.read_text(encoding="utf-8")).get("unconnected_items"):
            lastmile(report)
        full = drc_all(pcb)
        data = json.loads(full.read_text(encoding="utf-8"))
        kinds = {v.get("type") for v in data.get("violations", [])}
        errors = {v.get("type") for v in data.get("violations", []) if v.get("severity") == "error"}
        if not kinds & {"track_dangling", "via_dangling", "hole_to_hole"} and not errors and not data.get("unconnected_items"):
            return True
        state = json.dumps([sorted(v.get("type", "") for v in data.get("violations", [])),
                            sorted(str(sorted(i.get("uuid", "") for i in u.get("items", [])))
                                   for u in data.get("unconnected_items", []))])
        if state == last_state[0]:
            stuck[0] = True    # the same as after the previous round: more rounds would not change it
        last_state[0] = state
        notes.append(steps("drop_dangling", pcb, full))
        notes.append(steps("drop_close_vias", pcb, full))
        notes.append(steps("drop_offenders", pcb, full))
        notes.append(kpy(HERE / "zone_stitch.py", pcb, timeout=1500, fatal=False))
        thermal = [{"items": v.get("items", []), "thermal": True} for v in data.get("violations", [])
                   if v.get("type") == "starved_thermal"]
        if thermal:
            # pads whose thermal relief does not reach the pour: short tracks into the main pour
            req = pcb.with_name(pcb.stem + "_thermal.json")
            req.write_text(json.dumps({"unconnected_items": thermal}), encoding="utf-8")
            lastmile(req)
        islands = pcb.with_name(pcb.stem + "_islands.json")
        if islands.exists() and json.loads(islands.read_text(encoding="utf-8")).get("unconnected_items"):
            lastmile(islands)
        return False

    for _ in range(rounds):
        if one_round() or stuck[0]:
            break
        if time.time() > end:
            one_round()  # clean-up only, the last-mile calls are skipped from now on
            break
    if unlock_against and time.time() < end and drc(pcb)["unconnected"]:
        # completeness before tidiness: bundles and stitching links that block the last connections
        # may be ripped up
        notes.append(steps("unlock_new", pcb, unlock_against))
        stuck[0] = False
        for _ in range(2):
            if one_round() or stuck[0]:
                break
    # clean-up a person would do, each kept only if the board is not worse afterwards (open
    # connections, DRC errors): buses re-laid as bundles, then nets re-routed with fewer vias
    passes = [("rebundle", [buses_json, "--rebundle"])] if buses_json and buses_json.exists() else []
    passes.append(("reduce_vias", ["--reduce-vias"]))
    for label, args in passes:
        if time.time() > end - 60:
            notes.append(f"{label}: skipped, finishing time used up")
            continue
        before = drc(pcb)
        backup = pcb.with_name(pcb.stem + f"_before_{label}.kicad_pcb")
        shutil.copyfile(pcb, backup)
        lastmile(*args)
        after = drc(pcb)
        if (after["unconnected"], after["error_count"]) > (before["unconnected"], before["error_count"]):
            # a re-laid net may have cut a pour apart or left a gap: one repair round first
            one_round()
            after = drc(pcb)
        if (after["unconnected"], after["error_count"]) > (before["unconnected"], before["error_count"]):
            shutil.copyfile(backup, pcb)
            notes.append(f"{label}: undone (board got worse)")
    silk_args = [pcb, "--original", original] + (["--texts-from-original"] if texts_from_original else [])
    notes.append(kpy(HERE / "silk_fix.py", *silk_args))
    return notes


def score_session(pre: Path, ses: Path, out_dir: Path, post_zones: list, min_w: float) -> dict | None:
    """The routed session imported into KiCad and scored; None when KiCad cannot import it (its
    importer can fail or hang on a session; that candidate is then left out, not the whole run)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    pcb = copy_project(pre, out_dir, ses.stem)
    res = kpy(HERE / "kicad_steps.py", "import", pcb, ses.resolve(), pcb, timeout=900, fatal=False)
    if not res.endswith("import: ok"):
        log(f"{ses.stem}: session import failed, left out ({res.splitlines()[-1] if res else 'no output'})")
        return None
    if post_zones:
        steps("zones", pcb, json.dumps(post_zones))
    result = drc(pcb)
    result.update(json.loads(steps("stats", pcb).splitlines()[-1]))
    result["ses"] = str(ses)
    result["board"] = str(pcb)
    return result


def off_share(r: dict, main_layer: str | None) -> float:
    """Share of the track length off the main layer (2-layer boards), in steps of 10 %."""
    lengths = r.get("layer_mm") or {}
    total = sum(lengths.values())
    if not main_layer or not total:
        return 0.0
    return round(1 - lengths.get(main_layer, 0.0) / total, 1)


def bus_parallel(pcb: Path, buses_json: Path) -> float:
    """Share of bus track length running alongside its neighbour (0 when the board has no buses)."""
    if not buses_json.exists():
        return 0.0
    try:
        return json.loads(kpy(HERE / "buses.py", pcb, "--json", fatal=False).splitlines()[-1]).get("parallel") or 0.0
    except (ValueError, IndexError):
        return 0.0


def rank_key(r: dict):
    """Fewest open connections, then DRC errors, then buses laid as bundles (parallel share in steps
    of 10 %), vias, copper off the main layer (people keep 2-layer boards mostly on one side), length."""
    return (r["unconnected"], r["error_count"], -round(r.get("parallel", 0.0), 1), r["vias"],
            r.get("off_share", 0.0), r["track_mm"])


def select_best(results: list[dict]) -> dict:
    """The result a person would keep: among the most complete ones (open, then DRC errors), any
    with at most 20 % (+2) more vias than the fewest count as equal; among those the tidiest wins
    (buses in bundles, copper on the main side), then fewer vias, then length."""
    top = min((r["unconnected"], r["error_count"]) for r in results)
    complete = [r for r in results if (r["unconnected"], r["error_count"]) == top]
    fewest = min(r["vias"] for r in complete)
    fair = [r for r in complete if r["vias"] <= 1.2 * fewest + 2]
    return min(fair, key=lambda r: (-round(r.get("parallel", 0.0), 1), r.get("off_share", 0.0), r["vias"], r["track_mm"]))


def pick_candidates(scored: list[dict], n: int, bus_reach: int = 0) -> list[dict]:
    """The best routed result of every routing variant (both layers, single layer), then the next best
    overall up to n: a single-layer result usually leaves more open before finishing but often ends
    up cleaner after it."""
    ranked = sorted(scored, key=rank_key)
    picked, variants = [], set()
    # a variant that leaves many more connections open is not worth finishing (the last-mile router
    # would spend its whole budget on it); a few more are fine, styles often finish cleaner. The
    # finishing time is bounded, so on larger boards the reach grows with what is open.
    reach = ranked[0]["unconnected"] + max(6, ranked[0]["unconnected"] // 4) if ranked else 0
    for r in ranked:
        stem = Path(r["ses"]).stem
        variant = next((t for t in ("_1L", "_main", "_rebus", "_bus") if stem.endswith(t)), "")
        if r["unconnected"] > reach + (bus_reach if variant == "_bus" else 0):
            continue
        if variant not in variants:
            variants.add(variant)
            picked.append(r)
    for r in ranked:
        if len(picked) >= max(n, len(variants)):
            break
        if r not in picked:
            picked.append(r)
    return picked


MARKER = ".kicad-autoroute"


def prepare_work_dir(work: Path) -> None:
    """Start from an empty work directory. Only a directory this tool created earlier (it holds the
    marker file) is cleared; anything else that is not empty is left alone."""
    if work.exists() and any(work.iterdir()):
        if not (work / MARKER).exists():
            raise SystemExit(f"{work} exists and was not created by kicad-autoroute; choose another output directory")
        shutil.rmtree(work)
    work.mkdir(parents=True, exist_ok=True)
    (work / MARKER).write_text("work directory of kicad-autoroute; safe to delete\n", encoding="utf-8")


def run(cfg: dict, base: Path, label: str = "") -> dict:
    """Run the pipeline for one board. Paths in cfg are relative to base. Returns the report."""
    t0 = time.time()
    src = (base / cfg["board"]).resolve()
    name = cfg.get("name", src.stem)
    work = (base / cfg.get("work_dir", f"autoroute-{name}")).resolve()
    prepare_work_dir(work)
    log(f"board {src} -> {work}")

    # 1-2: copy and strip
    pre = copy_project(src, work / "board", name)
    pro, dru = pre.with_suffix(".kicad_pro"), pre.with_suffix(".kicad_dru")
    reference = drc(copy_project(src, work / "reference", name))
    reference.update(json.loads(steps("stats", work / "reference" / f"{name}.kicad_pcb").splitlines()[-1]))
    log(f"reference: {reference}")
    strip_cfg = cfg.get("strip", {})
    if strip_cfg.get("enabled", True):
        log(steps("strip", pre, *(["--keep-locked"] if strip_cfg.get("keep_locked") else []),
                  *(["--zones"] if strip_cfg.get("zones") else [])))

    # 3: placement
    place = cfg.get("placement", {})
    if place.get("mode", "none") == "compact":
        dsn = work / "place" / f"{name}_bare.dsn"
        dsn.parent.mkdir(parents=True, exist_ok=True)
        steps("export", pre, dsn)
        plan = work / "place" / "placement.json"
        shrink = place.get("shrink_mm", {})
        cmd = [sys.executable, str(HERE / "compact.py"), str(dsn), "-o", str(plan),
               "--spacing", str(place.get("spacing_mm", 0.5)), "--iterations", str(place.get("iterations", 25000))]
        for edge in ("left", "right", "top", "bottom"):
            cmd += [f"--{edge}", str(shrink.get(edge, 0))]
        if place.get("anchor_regex"):
            cmd += ["--anchor-re", place["anchor_regex"]]
        if place.get("slide_regex"):
            cmd += ["--slide-re", place["slide_regex"]]
        if place.get("hv_nets"):
            cmd += ["--hv-nets", ",".join(place["hv_nets"])]
        if place.get("groups", True):
            cmd += ["--groups"]
        if edge_clearance_um(pro):
            cmd += ["--edge-clearance", str(edge_clearance_um(pro) / 1000)]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            raise SystemExit(res.stderr[-2000:])
        log("placement: " + " | ".join(l for l in res.stdout.splitlines() if l.startswith(("outline", "wrote"))))
        log(steps("apply", pre, plan))

    # 4-6: rule-driven pre-routing
    esc = cfg.get("escape", {})
    if esc.get("enabled", True):
        log(kpy(HERE / "escape.py", pre, "--min-class-width", esc.get("min_class_width", 0.5)))
    st = cfg.get("stitch", {})
    if st.get("enabled", False):
        # the plane net, and on multilayer boards every other net with an inner-layer pour (supply
        # planes): their pads get their way into the plane before routing closes it off, as people
        # place the supply vias of a fine-pitch part first
        stitch_nets = [st.get("net", "GND")]
        if st.get("supply_planes", True):
            stitch_nets += [n for n in json.loads(steps("plane_nets", pre).splitlines()[-1]) if n not in stitch_nets]
        args = [pre] + [a for net in stitch_nets for a in ("--net", net)]
        if st.get("skip_refs"):
            args += ["--skip-refs", ",".join(st["skip_refs"])]
        log(kpy(HERE / "stitch.py", *args))
    zones = cfg.get("zones", {})
    pre_zones = list(zones.get("pre", []))
    # Post-route pours go in before routing too: the DSN then carries them as planes, so the router
    # knows those pads are connected and stops instead of retrying them until the time limit.
    if zones.get("post_as_planes", True):
        pre_zones += zones.get("post", [])
    if pre_zones:
        log(steps("zones", pre, json.dumps(pre_zones)))
    dsn = work / "route" / f"{name}.dsn"
    dsn.parent.mkdir(parents=True, exist_ok=True)
    log(steps("export", pre, dsn))

    # Freerouting settings (the fan-out run for the buses below uses them too)
    fr = cfg.get("freerouting", {})
    extra = list(fr.get("args", []))
    edge_um = edge_clearance_um(pro)
    if edge_um and not any("copper_to_edge" in a for a in extra):
        extra.append(f"--router.copper_to_edge_clearance_um={edge_um}")
    if fr.get("stop_at_stagnation", True):
        # the last-mile router closes the few connections a stagnating run leaves; Freerouting's own
        # fallback strategies would otherwise repeat attempts until the time limit
        for flag in ("--router.autorouter.fallback_strategies=none", "--router.autorouter.residual_repair_max_open=0"):
            if not any(flag.split("=")[0] in a for a in extra):
                extra.append(flag)
    # vias cost more than Freerouting's default (50): routes stay in their layer, and on dense boards
    # this even leaves fewer connections open (coldfire: 18 instead of 23 open, 291 instead of 335 vias)
    if fr.get("via_costs", 120) and not any("via_costs" in a for a in extra):
        extra.append(f"--router.scoring.via_costs={fr.get('via_costs', 120)}")
    # rounded up: a neck of 198 um on a 0.19812 mm (7.8 mil) board would fail DRC and be re-routed
    min_w_um = math.ceil(min_track_width(pro, dru) * 1000 - 1e-6)
    if fr.get("min_trace_width", True) and not any("min_trace_width" in a for a in extra):
        extra.append(f"--router.min_trace_width_um={min_w_um}")

    # buses (runs of nets between two parts, found in the netlist) are laid as parallel bundles on a
    # copy of the board before Freerouting routes the rest; a routing variant like the layer styles
    bus_pre, dsn_bus, buses_json = None, None, work / "route" / "buses.json"
    if cfg.get("buses", {}).get("enabled", True):
        found = json.loads(kpy(HERE / "buses.py", pre, "--json", "--pairs").splitlines()[-1])
        if found["buses"]:
            buses_json.write_text(json.dumps(found), encoding="utf-8")
            bus_pre = copy_project(pre, work / "board_bus", name)
            bus_cfg = cfg.get("buses", {})
            if bus_cfg.get("fanout_first", False):
                # people fan a fine-pitch part out first and run the bus to its vias: a short
                # Freerouting run (its fan-out comes first), whose pin-to-via stubs of the bus nets are
                # kept, locked; the bundles then end at those vias instead of walling the part in
                fan = router.run_once(dsn, "fanout", extra, work / "fanout", fr.get("fanout_timeout", "00:04:00"), 1)
                if fan.ses and Path(fan.ses).exists():
                    fan_board = copy_project(pre, work / "fanout_board", name)
                    steps("import", fan_board, Path(fan.ses).resolve(), fan_board)
                    log(steps("keep_fanout", bus_pre, fan_board, buses_json, *(["--all"] if bus_cfg.get("fanout_all", True) else [])))
            # long buses need a larger search budget than single connections; a coarser grid keeps it fast
            log(kpy(HERE / "lastmile.py", bus_pre, buses_json, "--buses", "--min-width", min_track_width(pro, dru),
                    "--max-cells", 200000, "--max-expansions", 1500000, "--time-budget", 1500,
                    "--trim-fanout", bus_cfg.get("trim_fanout", 0.0 if bus_cfg.get("fanout_first", False) else 1.5),
                    "--demand", bus_cfg.get("demand", 0.1),
                    "--fan-radius", bus_cfg.get("fan_radius", 1.5),
                    *([] if bus_cfg.get("lock", True) else ["--unlocked"]),
                    timeout=1800, fatal=False))
            dsn_bus = dsn.with_name(dsn.stem + "_bus.dsn")
            log(steps("export", bus_pre, dsn_bus))

    # 7: route
    seeds = fr.get("seeds", 4)
    # 2-layer boards get routing variants next to the plain one: the main side preferred (most copper
    # on one side, the other for crossings) and single-sided (the finishing pass adds jumpers). Every
    # variant gets seeds; ranking keeps completeness first, so a style never costs connections.
    dsns = [(dsn, "")]
    pref = json.loads(steps("layer_pref", pre, cfg.get("stitch", {}).get("net", "GND")).splitlines()[-1])
    main_layer = pref["layer"] if pref["copper_layers"] == 2 else None
    if main_layer and seeds >= 3:
        cost = fr.get("other_layer_cost", 3.0)
        if cost > 1.0:
            weighted = dsn.with_name(dsn.stem + "_main.dsn")
            shutil.copyfile(dsn, weighted)
            dsn_fix.layer_costs(str(weighted), main_layer, cost)
            dsns.append((weighted, "_main"))
        if fr.get("single_layer", True):
            single = dsn.with_name(dsn.stem + "_1L.dsn")
            dsn_fix.single_layer(str(dsn), str(single), main_layer)
            dsns.append((single, "_1L"))
        log(f"2-layer variants: {', '.join(t or 'plain' for _p, t in dsns)} (main layer {main_layer}, {pref['side']})")
    if dsn_bus is not None:
        dsns.append((dsn_bus, "_bus"))
    # the plain variant gets most runs (it gets furthest on hard boards): half of them, and three in
    # four when there is only one other style variant; the others alternate. Pre-laid buses alone
    # (multilayer boards) get half: their runs are the only ones that can keep the buses bundled.
    others = list(range(1, len(dsns)))
    if len(others) == 1 and dsns[1][1] != "_bus":
        pattern = [0, 0, 0, 1]
    else:
        pattern = [v for other in others for v in (0, other)] or [0]

    def make_jobs(first, last):
        out = []
        for s in range(first, last):
            path, tag = dsns[pattern[s % len(pattern)]]
            args = list(extra)
            if tag in ("_main", "_1L"):
                # the layer styles already shape the routes; with dear vias on top they over-constrain
                args = [a for a in args if "via_costs" not in a]
            out.append((path, f"s{s}{tag}", args + ([] if s == 0 else [f"--router.autorouter.item_order_seed={s}"])))
        return out
    workers, timeout = fr.get("workers", 4), fr.get("timeout", "00:15:00")
    log(f"routing {seeds} seeds, {workers} in parallel, timeout {timeout}")
    results = router.run_matrix(make_jobs(0, seeds), workers, work / "route", timeout, fr.get("max_passes", 100), "route")
    # small boards route in seconds: more seeds then cost little and smooth out Freerouting's run-to-run
    # spread, so add waves while a wave stays quick
    quick = fr.get("quick_wave_s", 120)
    total = fr.get("max_seeds", 12)
    while seeds < total and results and max(r.wall_s for r in results) < quick:
        more = min(total, seeds + max(workers, 4))
        log(f"first runs took under {quick} s: adding seeds {seeds}..{more - 1}")
        results = router.run_matrix(make_jobs(seeds, more), workers, work / "route", timeout,
                                    fr.get("max_passes", 100), "route")
        seeds = more

    # 8: score in KiCad
    min_w = min_track_width(pro, dru)
    post = zones.get("post", [])
    scored = []
    for ses in sorted((work / "route").glob("*.ses")):
        base = bus_pre if ses.stem.endswith("_bus") and bus_pre else pre
        r = score_session(base, ses, work / "score" / ses.stem, post, min_w)
        if r is None:
            continue
        r["off_share"] = off_share(r, main_layer)
        r["parallel"] = bus_parallel(Path(r["board"]), buses_json)
        log(f"{ses.stem}: unconnected={r['unconnected']} errors={r['errors']} vias={r['vias']} "
            f"track_mm={r['track_mm']}")
        scored.append(r)
    if not scored:
        raise SystemExit("no routing result")

    # buses re-laid on the best routed board: its bus nets are taken up and laid as bundles, which
    # may push other nets aside; Freerouting then continues from that nearly complete board and
    # routes only what was pushed aside (it is far better at that than at routing around bundles
    # on an empty board)
    plain = [r for r in scored if not Path(r["ses"]).stem.endswith(("_bus", "_1L", "_main"))]
    if buses_json.exists() and plain and cfg.get("buses", {}).get("reroute", False):
        top = min(plain, key=rank_key)
        rebus = copy_project(Path(top["board"]), work / "board_rebus", name)
        log(kpy(HERE / "lastmile.py", rebus, buses_json, "--buses", "--through", "--min-width", min_w,
                "--max-cells", 200000, "--max-expansions", 1500000, "--time-budget", 1500,
                *([] if cfg.get("buses", {}).get("lock", True) else ["--unlocked"]), timeout=1800, fatal=False))
        dsn_rebus = dsn.with_name(dsn.stem + "_rebus.dsn")
        log(steps("export", rebus, dsn_rebus))
        n = fr.get("reroute_seeds", 2)
        jobs = [(dsn_rebus, f"s{k}_rebus", extra + ([f"--router.autorouter.item_order_seed={k + 1}"] if k else []))
                for k in range(n)]
        router.run_matrix(jobs, min(workers, n), work / "route", timeout, fr.get("max_passes", 100), "rebus")
        for ses in sorted((work / "route").glob("*_rebus.ses")):
            r = score_session(rebus, ses, work / "score" / ses.stem, post, min_w)
            if r is None:
                continue
            r["off_share"] = off_share(r, main_layer)
            r["parallel"] = bus_parallel(Path(r["board"]), buses_json)
            log(f"{ses.stem}: unconnected={r['unconnected']} errors={r['errors']} vias={r['vias']} "
                f"track_mm={r['track_mm']} parallel={r['parallel']}")
            scored.append(r)
    best = min(scored, key=rank_key)

    # 9: finish the best candidates (last mile, clean-up, silkscreen), keep the best finished board
    finish_cfg = cfg.get("finish", {})
    def finish_one(cand: dict) -> dict:
        pcb = Path(cand["board"])
        notes = finish(pcb, min_w, src, place.get("mode", "none") == "compact" or finish_cfg.get("texts_from_original", False),
                       buses_json=buses_json, unlock_against=src, time_limit=finish_cfg.get("time_limit", 2400))
        r = drc(pcb)
        r.update(json.loads(steps("stats", pcb).splitlines()[-1]))
        r["off_share"] = off_share(r, main_layer)
        r["parallel"] = bus_parallel(pcb, buses_json)
        r.update({"ses": cand["ses"], "board": str(pcb), "notes": notes, "raw": {k: cand[k] for k in ("unconnected", "errors", "vias", "track_mm")}})
        log(f"finished {pcb.stem}: unconnected={r['unconnected']} errors={r['errors']} vias={r['vias']} track_mm={r['track_mm']}")
        return r

    # the candidates are separate copies: finish them side by side
    cands = pick_candidates(scored, finish_cfg.get("candidates", 3), finish_cfg.get("bus_reach", 6))
    with ThreadPoolExecutor(max(1, min(len(cands), finish_cfg.get("workers", 3)))) as pool:
        finished = list(pool.map(finish_one, cands))
    best = select_best(finished)

    # 10: result + report
    result_dir = work / "result"
    final = copy_project(Path(best["board"]), result_dir, name)
    renders = {}
    if cfg.get("report", {}).get("renders", False):
        for tag, board in (("before", work / "reference" / f"{name}.kicad_pcb"), ("after", final)):
            try:
                renders[tag] = [str(p) for p in render_board.render_views(board, result_dir / "img", tag)]
            except RuntimeError as exc:
                log(f"render {tag} failed: {exc}")
    report = {"config": label, "reference": reference, "best": best, "finished": finished, "all": scored,
              "renders": renders, "timing_s": round(time.time() - t0)}
    (result_dir / "report.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    log(f"best: {Path(best['ses']).stem} unconnected={best['unconnected']} errors={best['errors']} "
        f"vias={best['vias']} track_mm={best['track_mm']} board_mm={best['board_mm']}")
    log(f"reference: unconnected={reference['unconnected']} errors={reference['errors']} "
        f"vias={reference['vias']} track_mm={reference['track_mm']} board_mm={reference['board_mm']}")
    log(f"result: {final}")
    report["result"] = str(final)
    return report
