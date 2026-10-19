"""DSN post-processing: legal vias, board minimums, custom clearance rules as router classes."""
import dsn
import dsn_fix

DSN = """(pcb t.dsn
  (parser (string_quote ")(space_in_quoted_tokens on))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (via "Via[0-1]_600:300_um")
    (rule (width 200) (clearance 200) (clearance 50 (type smd_smd)))
  )
  (library
    (padstack "Via[0-1]_600:300_um"
      (shape (circle F.Cu 600))
      (shape (circle B.Cu 600))
      (attach off)
    )
  )
  (network
    (net GND (pins R1-2))
    (class kicad_default GND SIG Y_1 BD1_1
      (circuit (use_via "Via[0-1]_600:300_um"))
      (rule (width 200) (clearance 200))
    )
    (class Iso "+5V_ISO" GND_ISO
      (circuit (use_via "Via[0-1]_600:300_um"))
      (rule (width 400) (clearance 300))
    )
  )
  (wiring
    (via "Via[0-1]_600:300_um" 1000 1000 (net GND)(type fix))
  )
)
"""


def fixed(project, tmp_path):
    path = tmp_path / "t.dsn"
    path.write_text(DSN, encoding="latin-1")
    notes = dsn_fix.fix(str(path), str(project))
    return dsn.read(path), path.read_text(encoding="latin-1"), notes


def classes(tree):
    return {dsn.atoms(c)[1]: c for c in dsn.children(dsn.child(tree, "network"), "class")}


def rule_value(cls, key):
    return float(dsn.atoms(dsn.child(dsn.child(cls, "rule"), key))[1])


def test_vias_are_resized_and_renamed_everywhere(project, tmp_path):
    tree, text, _ = fixed(project, tmp_path)
    assert "Via[0-1]_600:300_um" not in text
    # structure, padstack, wiring and the use_via of all four classes (two split off for the HV rules)
    assert text.count("Via[0-1]_800:350_um") == 7
    padstack = dsn.child(dsn.child(tree, "library"), "padstack")
    assert [dsn.atoms(dsn.child(s, "circle"))[2] for s in dsn.children(padstack, "shape")] == ["800", "800"]


def test_widths_and_clearances_raised_to_minimums(project, tmp_path):
    tree, _, _ = fixed(project, tmp_path)
    structure_rule = dsn.child(dsn.child(tree, "structure"), "rule")
    assert [dsn.atoms(c)[1] for c in dsn.children(structure_rule, "clearance") if len(c) == 2] == ["250"]
    assert dsn.atoms(dsn.child(structure_rule, "width"))[1] == "320"
    default = classes(tree)["kicad_default"]
    assert rule_value(default, "width") == 320 and rule_value(default, "clearance") == 250


def test_custom_rules_become_classes(project, tmp_path):
    tree, _, notes = fixed(project, tmp_path)
    cls = classes(tree)
    assert dsn.atoms(cls["kicad_default"])[2:] == ["GND", "SIG"]          # HV nets moved out
    assert dsn.atoms(cls["kicaddefaultDRU1800"])[2:] == ["Y_1"]
    assert rule_value(cls["kicaddefaultDRU1800"], "clearance") == 1800
    assert rule_value(cls["kicaddefaultDRU2500"], "clearance") == 2500
    # the Iso rule spares its own class: inside it the normal clearance, towards others 2 mm
    assert "Iso" not in cls
    iso = cls["IsoDRU2000G"]
    assert dsn.atoms(iso)[2:] == ["+5V_ISO", "GND_ISO"]
    assert rule_value(iso, "clearance") == 300
    pairs = {tuple(dsn.atoms(dsn.child(p, "classes"))[1:]): float(dsn.atoms(dsn.child(dsn.child(p, "rule"), "clearance"))[1])
             for p in dsn.children(dsn.child(tree, "network"), "class_class")}
    assert pairs[("IsoDRU2000G", "kicad_default")] == 2000
    assert pairs[("IsoDRU2000G", "kicaddefaultDRU2500")] == 2500     # the stricter side wins
    assert any("Iso" in n for n in notes)


def test_single_layer_variant(tmp_path):
    src, dst = tmp_path / "a.dsn", tmp_path / "b.dsn"
    src.write_text(DSN, encoding="latin-1")
    assert dsn_fix.single_layer(str(src), str(dst), "B.Cu") == 2
    for cls in classes(dsn.read(dst)).values():
        assert [dsn.atoms(u) for u in dsn.children(dsn.child(cls, "circuit"), "use_layer")] == [["use_layer", "B.Cu"]]


def test_layer_costs(tmp_path):
    path = tmp_path / "a.dsn"
    path.write_text(DSN, encoding="latin-1")
    dsn_fix.layer_costs(str(path), "B.Cu", 2.0)
    settings = dsn.child(dsn.child(dsn.read(path), "structure"), "autoroute_settings")
    costs = {dsn.atoms(r)[1]: float(dsn.atoms(dsn.child(r, "preferred_direction_trace_costs"))[1])
             for r in dsn.children(settings, "layer_rule")}
    assert costs == {"F.Cu": 2.0, "B.Cu": 1.0}
    assert dsn.atoms(dsn.child(settings, "postroute")) == ["postroute", "off"]


def test_outer_layer_pours_are_not_planes(project, tmp_path):
    text = DSN.replace('(via "Via[0-1]_600:300_um")\n',
                       '(via "Via[0-1]_600:300_um")\n    (plane GND (polygon B.Cu 0 0 0 1000 0 1000 1000 0 0))\n', 1)
    path = tmp_path / "t.dsn"
    path.write_text(text, encoding="latin-1")
    notes = dsn_fix.fix(str(path), str(project))
    assert not dsn.children(dsn.child(dsn.read(path), "structure"), "plane")
    assert any("GND on B.Cu" in n for n in notes)
