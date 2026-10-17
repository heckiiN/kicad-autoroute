# kicad-autoroute benchmark

2026-10-17, KiCad C:\Program Files\KiCad\10.0\bin, Windows-10-10.0.19045-SP0, 4 seeds x 00:20:00

Cells: unconnected / DRC errors (KiCad DRC after zone refill).

| Board | Original (hand-routed) | Freerouting plugin workflow | kicad-autoroute | time |
|---|---|---|---|---|
| ecc83 | 0 / 0 | 0 / 0 | 0 / 0 | 0.7 min |
| sonde | 0 / 0 | 2 / 2 | 1 / 0 | 3.8 min |
| pic_programmer | 0 / 0 | 2 / 2 | 0 / 0 | 2.7 min |
| interf_u | 0 / 2 | 13 / 5 | 3 / 3 | 26.7 min |
| multichannel | 0 / 12 | 160 / 42 | 1 / 0 | 3.5 min |
| coldfire | 0 / 0 | 41 / 130 | 6 / 0 | 28 min |
