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
