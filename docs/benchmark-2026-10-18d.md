# kicad-autoroute benchmark

2026-10-18 (evening), KiCad 10.0, Windows 10, 4 seeds x 00:20:00 plus extra seeds for quick boards

Cells: unconnected / DRC errors (KiCad DRC after zone refill).

| Board | Original (hand-routed) | kicad-autoroute | vias | copper off main side | time |
|---|---|---|---|---|---|
| ecc83 | 0 / 0 | 0 / 0 | 0 | 0 % | 1.9 min |
| sonde | 0 / 0 | 0 / 0 | 12 | 60 % | 3.4 min |
| pic_programmer | 0 / 0 | 0 / 0 | 4 | 30 % | 4.1 min |
| interf_u | 0 / 2 | 3 / 6 | 54 | 60 % | 10.1 min |
| multichannel | 0 / 12 | 1 / 0 | 36 | 10 % | 9.0 min |
| coldfire | 0 / 0 | 6 / 0 | 376 | 0 % | 23.8 min |
