"""3D views of a KiCad board through kicad-cli (top, bottom, perspective)."""
from __future__ import annotations

import subprocess
from pathlib import Path

from . import tools


def render(board: Path, out: Path, *extra: str) -> None:
    cmd = [str(tools.kicad_cli()), "pcb", "render", "--width", "1600", "--height", "1100", "--quality", "high",
           "--background", "opaque", *extra, "-o", str(out), str(board)]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    if res.returncode != 0 or not out.exists():
        raise RuntimeError(f"render failed: {res.stderr[-800:]}")


def render_views(board: Path, out_dir: Path, prefix: str) -> list[Path]:
    """Writes <prefix>_top.png, <prefix>_bottom.png and <prefix>_iso.png; returns the paths."""
    out_dir.mkdir(parents=True, exist_ok=True)
    views = {"top": ("--side", "top", "--zoom", "1.0"),
             "bottom": ("--side", "bottom", "--zoom", "1.0"),
             "iso": ("--side", "top", "--perspective", "--rotate", "-35,0,25", "--zoom", "0.9")}
    paths = []
    for view, args in views.items():
        out = out_dir / f"{prefix}_{view}.png"
        render(board, out, *args)
        paths.append(out)
    return paths
