"""Reading the board's own rules: net classes, minimums, via sizes, custom clearance rules."""
from kicad_classes import NetClasses, ViaRules, board_minimums, board_net_names


def test_net_names_kicad10_and_older(project, tmp_path):
    assert board_net_names(str(project)) == ["+5V_ISO", "BD1_1", "GND", "GND_ISO", "SIG", "Y_1"]
    old = tmp_path / "old.kicad_pcb"
    old.write_text('(kicad_pcb (net 0 "") (net 1 "GND") (net 2 "VCC"))', encoding="utf-8")
    assert board_net_names(str(old)) == ["GND", "VCC"]


def test_minimums_from_setup_and_unconditional_rules(project):
    width, clearance = board_minimums(str(project))
    assert width == 0.32          # unconditional .kicad_dru rule beats the board setup (0.3)
    assert clearance == 0.25


def test_classes_are_raised_to_board_minimums(project):
    classes = NetClasses.for_board(str(project))
    assert classes.width("SIG") == 0.32 and classes.clearance("SIG") == 0.25   # Default 0.2/0.2 raised
    assert classes.class_of("GND_ISO") == "Iso"
    assert classes.width("GND_ISO") == 0.4 and classes.clearance("GND_ISO") == 0.3


def test_via_rules_raise_to_minimums(project):
    rules = ViaRules.for_board(str(project))
    assert rules.legal(0.6, 0.3) == (0.8, 0.35)        # min diameter 0.8, min drill 0.35
    assert rules.legal(0.6, 0.6) == (0.9, 0.6)         # annular ring 0.15 on each side
    assert rules.default == (0.8, 0.35)                # Default class 0.6/0.3, made legal
    assert rules.options((0.8, 0.4), (0.6, 0.3), (0.5, 0.25)) == [(0.8, 0.4), (0.8, 0.35)]


def test_dru_clearances_wildcards_classes_and_pairs(project):
    extra = NetClasses.dru_net_clearances(str(project), detail=True)
    assert extra["Y_1"] == (1.8, False)        # the Y-to-BD pair rule (1.8) beats hv_y (1.3)
    assert extra["BD1_1"] == (2.5, False)
    assert extra["GND_ISO"] == (2.0, True)     # spares its own class
    assert extra["+5V_ISO"] == (2.0, True)
    assert "GND" not in extra and "SIG" not in extra   # footprint rule skipped
    assert NetClasses.dru_net_clearances(str(project))["Y_1"] == 1.8
