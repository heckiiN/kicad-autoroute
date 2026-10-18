# kicad-autoroute benchmark

2026-10-18, KiCad 10.0, Windows 10, 4 seeds x 00:20:00, with 2-layer layout styles

Cells: unconnected / DRC errors (KiCad DRC after zone refill).

| Board | Original (hand-routed) | kicad-autoroute | vias | copper off main side | time |
|---|---|---|---|---|---|
| ecc83 | 0 / 0 | 0 / 0 | 0 | 0 % | 0.8 min |
| sonde | 0 / 0 | 0 / 0 | 13 | 20 % | 4.2 min |
| pic_programmer | 0 / 0 | 0 / 0 | 4 | 30 % | 11.0 min |
| interf_u | 0 / 2 | 6 / 4 | 51 | 60 % | 23.0 min |
| multichannel | 0 / 12 | 2 / 0 | 35 | 10 % | 9.0 min |
| coldfire | 0 / 0 | 10 / 0 | 360 | 0 % | 118.6 min |
