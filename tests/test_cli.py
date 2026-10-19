"""CLI configuration and the safety of the work directory."""
import argparse

import pytest

from kicad_autoroute import cli, pipeline


def args(target, **kw):
    base = dict(output=None, seeds=None, workers=None, timeout=None, plane_net=None, no_stitch=False,
                no_escape=False, strip_locked=False, renders=False)
    base.update(kw)
    return argparse.Namespace(target=str(target), **base)


def test_defaults_for_a_board(project):
    cfg, _, _ = cli.build_config(args(project))
    assert cfg["stitch"]["enabled"] is True           # the board has a GND net
    assert cfg["strip"]["keep_locked"] is True
    assert cfg["freerouting"] == {"seeds": 4, "workers": 4, "timeout": "00:20:00"}
    assert cfg["placement"]["mode"] == "none"


def test_flags_override(project):
    cfg, _, _ = cli.build_config(args(project, seeds=8, timeout="00:05:00", plane_net="VSS", strip_locked=True))
    assert cfg["freerouting"]["seeds"] == 8 and cfg["freerouting"]["timeout"] == "00:05:00"
    assert cfg["stitch"] == {"enabled": False, "net": "VSS"}   # no VSS net on this board
    assert cfg["strip"]["keep_locked"] is False


def test_toml_config_merges_with_defaults(project, tmp_path):
    toml = tmp_path / "b.toml"
    toml.write_text('board = "t.kicad_pcb"\n[freerouting]\nseeds = 2\n[placement]\nmode = "compact"\n', encoding="utf-8")
    cfg, base, _ = cli.build_config(args(toml))
    assert base == tmp_path
    assert cfg["freerouting"]["seeds"] == 2 and cfg["freerouting"]["workers"] == 4
    assert cfg["placement"]["mode"] == "compact"


def test_work_dir_is_only_cleared_when_ours(tmp_path):
    foreign = tmp_path / "mine"
    foreign.mkdir()
    (foreign / "important.txt").write_text("keep", encoding="utf-8")
    with pytest.raises(SystemExit):
        pipeline.prepare_work_dir(foreign)
    assert (foreign / "important.txt").exists()

    ours = tmp_path / "out"
    pipeline.prepare_work_dir(ours)
    (ours / "old.txt").write_text("x", encoding="utf-8")
    pipeline.prepare_work_dir(ours)
    assert not (ours / "old.txt").exists() and (ours / pipeline.MARKER).exists()


def test_candidates_cover_every_variant_within_reach():
    def r(stem, unconnected, vias=0):
        return {"ses": f"{stem}.ses", "unconnected": unconnected, "error_count": 0, "vias": vias, "track_mm": 1.0}
    scored = [r("b__s0", 1, 10), r("b__s3", 2, 5), r("b_main__s1_main", 3), r("b_1L__s2_1L", 12)]
    picked = [c["ses"] for c in pipeline.pick_candidates(scored, 2)]
    assert picked == ["b__s0.ses", "b_main__s1_main.ses"]   # single-layer leaves too much open


def test_off_share_ranks_main_layer_boards_first():
    clean = {"unconnected": 0, "error_count": 0, "vias": 0, "track_mm": 200, "layer_mm": {"B.Cu": 200}}
    mixed = {"unconnected": 0, "error_count": 0, "vias": 0, "track_mm": 160, "layer_mm": {"B.Cu": 100, "F.Cu": 60}}
    for x in (clean, mixed):
        x["off_share"] = pipeline.off_share(x, "B.Cu")
    assert min([mixed, clean], key=pipeline.rank_key) is clean


def test_select_best_trades_a_few_vias_for_a_tidy_board():
    def r(vias, off, unconnected=0, parallel=0.0):
        return {"unconnected": unconnected, "error_count": 0, "vias": vias, "off_share": off,
                "parallel": parallel, "track_mm": 100.0}
    fewest_vias, tidy, too_many, incomplete = r(4, 0.4), r(6, 0.2), r(20, 0.0), r(0, 0.0, unconnected=1)
    assert pipeline.select_best([fewest_vias, tidy, too_many, incomplete]) is tidy
    bundled = r(6, 0.4, parallel=0.8)
    assert pipeline.select_best([fewest_vias, tidy, bundled]) is bundled
