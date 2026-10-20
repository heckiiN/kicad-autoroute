# kicad-autoroute benchmark

2026-10-20, KiCad 10.0, Windows 10, 4 seeds x 00:20:00 plus extra seeds for quick boards

Cells: unconnected / DRC errors (KiCad DRC after zone refill).

| Board | Original (hand-routed) | kicad-autoroute | time |
|---|---|---|---|
| ecc83 | 0 / 0 | 0 / 0 | 1.7 min |
| sonde | 0 / 0 | 0 / 0 | 3.8 min |
| pic_programmer | 0 / 0 | 0 / 0 | 4.6 min |
| interf_u | 0 / 2 | 0 / 0 | 11.0 min |
| multichannel | 0 / 12 | 0 / 0 | 6.1 min |
| coldfire | 0 / 0 | 3 / 0 | 62.2 min |

## Against the hand-routed originals

`benchmark/scorecard.py`: a board counts as routed at human level when it is complete (0 open, 0 DRC
errors), uses at most 1.1 x the human's vias + 2, keeps its buses (runs of neighbouring pins between two
parts) at least as bundled as the human did (share of bus length running alongside its neighbour,
human - 0.05), and on 2-layer boards keeps as much copper on the main side (human - 0.10).

| Board | open / errors | vias (human / ai) | buses parallel (human / ai) | main side (human / ai) | human level |
|---|---|---|---|---|---|
| ecc83 | 0 / 0 ok | 0 / 0 ok | – / – | 1.0 / 0.96 ok | yes |
| sonde | 0 / 0 ok | 3 / 3 ok | 0.14 / 0.67 ok | 0.69 / 0.75 ok | yes |
| pic_programmer | 0 / 0 ok | 6 / 0 ok | – / – | 0.8 / 0.77 ok | yes |
| interf_u | 0 / 0 ok | 84 / 44 ok | 0.24 / 0.35 ok | 0.58 / 0.56 ok | yes |
| multichannel | 0 / 0 ok | 29 / 22 ok | – / – | 0.9 / 0.9 ok | yes |
| coldfire | 3 / 0 MISS | 253 / 259 ok | 0.8 / 0.37 MISS | – / – | no |

5 of 6 boards at human level.

kit-dev-coldfire varies between runs: a single run with the same code (2026-10-20h) finished with the
bundled variant, 2 open, 284 vias and its buses 0.48 bundled.
