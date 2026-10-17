import json
import sys
from pathlib import Path

import pytest

# the KiCad-side modules import their siblings by plain name (they also run as scripts)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "kicad_autoroute"))

PRO = {
    "board": {"design_settings": {"rules": {
        "min_track_width": 0.3, "min_clearance": 0.25,
        "min_via_diameter": 0.8, "min_through_hole_diameter": 0.35, "min_via_annular_width": 0.15,
        "min_copper_edge_clearance": 0.5,
    }}},
    "net_settings": {
        "classes": [
            {"name": "Default", "track_width": 0.2, "clearance": 0.2, "via_diameter": 0.6, "via_drill": 0.3},
            {"name": "Iso", "track_width": 0.4, "clearance": 0.3, "via_diameter": 0.8, "via_drill": 0.4},
        ],
        "netclass_patterns": [{"netclass": "Iso", "pattern": "*_ISO"}],
    },
}

DRU = """(version 1)
(rule "hv_y"
  (condition "A.NetName == 'Y_*' && B.NetName != A.NetName")
  (constraint clearance (min 1.3mm)))
(rule "hv_bd"
  (condition "A.NetName == 'BD?_*' && B.NetName != A.NetName")
  (constraint clearance (min 2.5mm)))
(rule "y_to_bd"
  (condition "A.NetName == 'Y_*' && B.NetName == 'BD?_*'")
  (constraint clearance (min 1.8mm)))
(rule "iso_barrier"
  (condition "A.NetClass == 'Iso' && B.NetClass != 'Iso'")
  (constraint clearance (min 2.0mm)))
(rule "intra_part"
  (condition "A.memberOfFootprint('C1') && B.memberOfFootprint('C1')")
  (constraint clearance (min 0.1mm)))
(rule "neckdown"
  (constraint track_width (min 0.32mm)))
"""

# KiCad 10 style: no net table, items carry (net "name")
PCB = """(kicad_pcb (version 20250000) (generator "test")
  (footprint "R" (pad "1" smd rect (net "Y_1")) (pad "2" smd rect (net "GND")))
  (footprint "R" (pad "1" smd rect (net "BD1_1")) (pad "2" smd rect (net "+5V_ISO")))
  (footprint "R" (pad "1" smd rect (net "GND_ISO")) (pad "2" smd rect (net "SIG")))
)
"""


@pytest.fixture
def project(tmp_path: Path) -> Path:
    """A minimal KiCad project (board, project rules, custom rules); returns the board path."""
    (tmp_path / "t.kicad_pro").write_text(json.dumps(PRO), encoding="utf-8")
    (tmp_path / "t.kicad_dru").write_text(DRU, encoding="utf-8")
    (tmp_path / "t.kicad_pcb").write_text(PCB, encoding="utf-8")
    return tmp_path / "t.kicad_pcb"
