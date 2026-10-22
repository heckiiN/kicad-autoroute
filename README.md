# kicad-autoroute

Autorouting for KiCad boards in one command, built around [Freerouting](https://github.com/freerouting/freerouting).

Freerouting does the actual routing. kicad-autoroute does everything around it that decides whether
the result passes KiCad's DRC:

- it hands Freerouting the board's **real rules** (KiCad's Specctra export drops several of them),
- prepares what Freerouting is bad at (escapes from fine-pitch pads, stitching vias into planes,
  buses laid as parallel bundles),
- routes **several seeds in parallel** and scores each result with **KiCad's own DRC**,
- and closes what is left with a small **rule-checking last-mile router** and a clean-up pass.

```bash
kicad-autoroute my_board.kicad_pcb
```

The project is only read. Everything happens on copies in `./autoroute-my_board/`; the routed board
ends up in `autoroute-my_board/result/`.

## Status: honest numbers

This is an early version (0.1). It reliably beats the plain Freerouting plugin workflow and finishes
five of KiCad's six demo boards completely, but it does **not** finish every board: on large dense
boards a few connections can stay open and have to be routed by hand.

KiCad's demo boards, default settings, no per-board tuning, run with `benchmark/run_benchmark.py`
(full output in `docs/benchmark-2026-10-20i.md`, plugin-workflow numbers from `docs/benchmark-2026-10-17.md`); the last row
is a private board, routed in an earlier version with the same settings.
Cells: unconnected / DRC errors, counted by KiCad's DRC after refilling zones.

| Board | Layers / parts | Original (hand-routed) | Freerouting plugin workflow¹ | **kicad-autoroute** | Time |
|---|---|---|---|---|---|
| ecc83 | 2 / 15 | 0 / 0 | 0 / 0 | **0 / 0** | 2 min |
| sonde xilinx | 2 / 25 | 0 / 0 | 2 / 2 | **0 / 0** | 4 min |
| pic_programmer | 2 / 63 | 0 / 0 | 2 / 2 | **0 / 0** | 5 min |
| interf_u | 2 / 25 | 0 / 2 | 13 / 5 | **0 / 0** | 11 min |
| multichannel_mixer | 2 / 114 | 0 / 12 | 160 / 42 | **0 / 0** | 6 min |
| kit-dev-coldfire | 4 / 160 | 0 / 0 | 41 / 130 | **0–2 / 0**² | 85 min |
| private board (HV rules) | 4 / 491 | 0 / 0 | no result in 1 h | **83 / 2**³ | 2.6 h |

¹ Tracks removed, KiCad's Specctra export, Freerouting 2.4.1 with up to 100 passes, session imported back.
² Results vary between runs (Freerouting's passes depend on timing); recent runs left 0–2 open.
³ An earlier version; the remaining errors were "starved thermal" reliefs on the ground pour.

### Does it look hand-made?

Complete is not the same as tidy. `benchmark/scorecard.py` compares every routed board with its
hand-routed original: complete (0 open, 0 errors), at most 1.1 × the human's vias + 2, buses (runs of
neighbouring pins between two parts) at least as bundled as the human laid them (share of bus length
running alongside its neighbour, human − 0.05), and on 2-layer boards as much copper on the main side
(human − 0.10). Five of six demo boards pass all four. kit-dev-coldfire does not yet: its vias are
at the human's count now (234–268 against 253), but 0–2 connections stay open depending on the run,
and its buses are 0.44–0.62 bundled against the human's 0.80. Bus nets attract each other inside
Freerouting (patch `0002-bus-attraction`): a bus net's path costs less alongside its routed
neighbours, which lifted coldfire's bundled share from about 0.3 without changing how many
connections close; most of what is still missing sits at the pins, where Freerouting's fan-out
places each pin's escape on its own.

### Boards it was never tuned on

The six boards above are the ones the router was developed against, so they can hide overfitting.
`run_benchmark.py --holdout` runs six other KiCad demo boards, used only to check changes, never to
tune them; a private board of the author is in the same role. Errors that the hand-routed original
has as well (footprints closer than their courtyards, pads too near the edge, drills outside the
board's own limits) are counted apart.

| Board | Layers / parts | Original | **kicad-autoroute** | own errors | Time |
|---|---|---|---|---|---|
| complex_hierarchy | 2 / 68 | 0 / 0 | **0 / 0** | 0 | 4 min |
| StickHub | 2 / 94 | 0 / 0 | **0–1 / 0** | 0 | 6–13 min |
| CM5_MINIMA_3 | 4 / 112 | 0 / 2 | **1 / 2** | 0 | 60 min |
| One-Air-Max | 2 / 210 | 0 / 226 | **0–3 / 27** | 0 | 50–68 min |
| RoyalBlue54L-Feather | 6 / 71 | 0 / 237 | **11–14 / 51** | 0 | 75–80 min |
| video | 4 / 189 | 0 / 0 | **187 / 0** | 0 | 1.7 h |
| private board | 4 / 158 | 0 / 0 | **0–1 / 0** | 0 | 30–55 min |

The held-out boards found real faults that the tuning boards never showed, all fixed in general
form: pads of parts turned by 45° modelled as their bounding box, solder-mask openings drawn as
graphics, pad-level clearances (1.7 mm around a module's mounting holes), the board outline checked
by its bounding box only, keep-out areas ignored by the last-mile router, KiCad message boxes that
block a headless run, a session import that hung on duplicate references, and legitimately thin
0.127 mm differential pairs removed as "necked" tracks. Large dense boards (video) stay far from
complete: Freerouting does not get past its first pass within the time limit there.

Measured on Windows 10, KiCad 10.0, 4 seeds × 20 min, 16 cores. `tinytapeout-demo` is left out:
KiCad's Python API does not load it within 15 minutes on this machine.

## What it does

| Step | What and why |
|---|---|
| copy | board, project and custom rules go to a work directory; the source is never written |
| strip | existing tracks and vias are removed (locked ones stay) |
| placement (optional) | `compact` mode shrinks the outline and re-places parts; connectors and other case-bound parts stay on their edge, functional groups move as blocks |
| escape | fine-pitch pads of nets that need wide copper get a short narrow stub and a via, where Freerouting would neck the trace below the rules or give up |
| stitch | SMD pads of the plane net (GND by default) get a via into the pour |
| export + rule fix | Specctra export, then corrected: via sizes raised to the board's minimum diameter/drill/annular ring, net-class widths/clearances raised to the board minimums, custom `.kicad_dru` clearance rules (net-name wildcards, net classes) turned into router classes, placement rooms that forbid nothing no longer exported as keepouts |
| buses | runs of neighbouring pins between two parts (found in the netlist) are laid as parallel bundles on a copy of the board, middle net first with room for the rest, the others in a corridor next to their neighbour; the bundled copy is a routing variant of its own |
| route | Freerouting on N item-order seeds in parallel, stopping when passes stop improving; while a wave of runs takes under two minutes, more seeds are added (up to 12), which evens out Freerouting's run-to-run spread on small boards. 2-layer boards also get two layout styles people use: most copper on a main side (the SMD side, or the solder side of a through-hole board) with the other side for crossings, and single-sided with jumpers |
| score | every result is imported into KiCad, zones refilled, KiCad DRC; ranked by unconnected, errors, vias, share of copper off the main side, length. The best result of each style that is within reach gets finished |
| finish | for the best candidates: drop necked/dangling/offending items, last-mile router for what is open, vias for cut-off pour islands, silkscreen clean-up; repeated a few rounds. The last-mile router works on a grid with its own clearance checks (copper text included), prefers straight runs with 45°/90° corners, joins cut-off pads to the nearest piece of their pour, retries tight passages on a finer grid, and can rip up a few unlocked tracks of other nets and route them again (undone unless everything fits). Then the clean-up a person would do, each step kept only if the board does not get worse: buses re-laid as bundles (pushing other tracks aside), nets re-routed with fewer vias |
| report | `result/<board>.kicad_pcb`, `report.json`, optional 3D renders |

## Install

Needs:

- **KiCad 10** (KiCad 9 likely works, untested) – its `kicad-cli` and its Python with `pcbnew`
- **Java 25** (a JDK, to build and run Freerouting)
- **git** and **Python 3.11+**

```bash
git clone https://github.com/heckiiN/kicad-autoroute
cd kicad-autoroute
python -m venv .venv && .venv/bin/pip install -e .        # Windows: .venv\Scripts\pip install -e .
JAVA_HOME=/path/to/jdk-25 python freerouting-patches/build_freerouting.py
kicad-autoroute check
```

`build_freerouting.py` clones Freerouting at a pinned commit, applies the patch in
`freerouting-patches/` and builds the jar into `build/`. `kicad-autoroute check` shows which KiCad,
Java and Freerouting it found. Override with `KICAD_BIN`, `KICAD_PYTHON`, `JAVA` / `JAVA_HOME`,
`FREEROUTING_JAR`.

## Use

```bash
kicad-autoroute board.kicad_pcb                      # defaults: 4 seeds x 20 min, GND stitching if a GND net exists
kicad-autoroute board.kicad_pcb --seeds 8 --workers 4 --timeout 00:40:00
kicad-autoroute board.kicad_pcb --plane-net VSS --renders -o out/
kicad-autoroute my_board.toml                        # everything in a config file
```

`configs/example.toml` documents every option. Each Freerouting run is one JVM with up to 2 GB heap,
so `--workers` × 2 GB should fit into free RAM.

Locked tracks are kept, so hand-route critical nets (RF, differential pairs, high current), lock
them, and let the tool do the rest.

## Limitations

- Large dense boards: one Freerouting pass can take 15+ minutes, so 20 minutes allow only one or two
  passes, and many connections stay open. More time helps but is slow.
- Custom DRC rules that depend on footprints, courtyards, layers or item types are not passed to the
  router (only net-name and net-class clearance rules are).
- No length matching, no differential-pair routing.
- Placement (`compact`) needs to be told which parts are fixed by the case; nothing in a KiCad file says so.
- Tested on Windows only so far.

## Benchmark

```bash
python benchmark/run_benchmark.py                                  # pipeline on KiCad's demo boards
python benchmark/run_benchmark.py --standard path/to/freerouting-2.4.1.jar   # plus the plugin workflow
```

Results land in `benchmark/results/<date>/results.md`. A side-by-side view of the hand-routed and the
routed boards (self-contained HTML with the copper layers embedded):

```bash
python benchmark/compare_views.py benchmark/results/<date>
```

## Freerouting changes

`freerouting-patches/0001-min-trace-width.patch` adds `router.min_trace_width_um`: neither the
automatic neckdown at pins nor the fanout's micro-neckdown (3/4, 3/5, 1/2 of the trace width) goes
below the board's minimum track width (KiCad's export does not carry it, so necked traces failed
KiCad DRC and had to be taken up and routed again). It is meant to go upstream.

`experimental-congestion-history.patch` is a negotiated-congestion (PathFinder-style) history cost
for Freerouting's maze search. It gave **no measurable gain** on the DAC2020 and KiCad demo boards
and is not used; it is kept as a documented negative result.

## Tests

```bash
pip install -e ".[test]" && pytest
```

The unit tests cover the rule reading and the Specctra corrections and need neither KiCad nor Java.

## License

GPL-3.0-or-later, like Freerouting. KiCad and Freerouting are separate projects of their respective
authors; this tool only drives them.
