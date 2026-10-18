# kicad-autoroute benchmark

2026-10-18, KiCad C:\Program Files\KiCad\10.0\bin, Windows-10-10.0.19045-SP0, 4 seeds x 00:20:00

Cells: unconnected / DRC errors (KiCad DRC after zone refill).

| Board | Original (hand-routed) | Freerouting plugin workflow | kicad-autoroute | time |
|---|---|---|---|---|
| ecc83 | 0 / 0 | – | 0 / 0 | 0.7 min |
| sonde | 0 / 0 | – | 0 / 0 | 1.4 min |
| pic_programmer | 0 / 0 | – | 0 / 0 | 1.7 min |
| interf_u | 0 / 2 | – | 2 / 4 | 11.4 min |
| multichannel | 0 / 12 | – | 2 / 0 | 8.3 min |
| coldfire | 0 / 0 | – | 8 / 0 | 51.7 min |
