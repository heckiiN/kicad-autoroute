# Freerouting patches

kicad-autoroute runs Freerouting from upstream commit
[`5aa1b24`](https://github.com/freerouting/freerouting/commit/5aa1b24dc8e5af7f950c551c1e53de9077a84cd5)
plus the patches here. `build_freerouting.py` fetches that commit, applies them and builds the jar.

| Patch | Status | What |
|---|---|---|
| `0001-min-trace-width.patch` | used, intended for upstream | New setting `router.min_trace_width_um`. Automatic neckdown at fine-pitch pins narrows a trace to half the pin width with no lower bound; KiCad's Specctra export does not carry the board's minimum track width, so necked traces failed KiCad DRC. With the setting, `Pin.getTraceNeckdownHalfwidth` never goes below the minimum. Includes unit tests. |
| `experimental-congestion-history.patch` | not used | Negotiated-congestion (PathFinder-style) history cost in the maze search: regions where earlier passes ripped up items or failed get more expensive. Off by default; on the DAC2020 and KiCad demo boards it gave no measurable gain. Kept as a documented negative result. Also contains the min-trace-width change. |

Everything else kicad-autoroute needs (`fallback_strategies`, `residual_repair_max_open`,
`copper_to_edge_clearance_um`, result manifests) is already in upstream Freerouting.
