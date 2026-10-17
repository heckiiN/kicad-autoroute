"""Find the external programs the pipeline drives: KiCad (kicad-cli and its bundled python with
pcbnew), Java and the Freerouting jar.

Each can be set explicitly through an environment variable; otherwise the usual install locations
and PATH are searched.

  KICAD_BIN         directory with kicad-cli and KiCad's python (e.g. C:\\Program Files\\KiCad\\10.0\\bin)
  KICAD_PYTHON      python interpreter that can `import pcbnew` (overrides the one found in KICAD_BIN)
  JAVA              java executable (Java 25 or newer for current Freerouting)
  FREEROUTING_JAR   Freerouting executable jar (built with the patch in freerouting-patches/)
"""
from __future__ import annotations

import glob
import os
import platform
import re
import shutil
import subprocess
from functools import lru_cache
from pathlib import Path

EXE = ".exe" if os.name == "nt" else ""
PACKAGE_DIR = Path(__file__).resolve().parent
REPO_DIR = PACKAGE_DIR.parent


class ToolError(RuntimeError):
    pass


def _version_key(path: Path) -> tuple[int, ...]:
    """(10, 0) for .../KiCad/10.0/bin; () when the path carries no version."""
    for part in reversed(path.parts):
        if re.fullmatch(r"\d+(\.\d+)*", part):
            return tuple(int(x) for x in part.split("."))
    return ()


def _kicad_bin_candidates() -> list[Path]:
    """KICAD_BIN first, then installed versions newest first, then kicad-cli on PATH."""
    found: list[Path] = []
    if os.environ.get("KICAD_BIN"):
        found.append(Path(os.environ["KICAD_BIN"]))
    system = platform.system()
    if system == "Windows":
        installs = []
        for root in (os.environ.get("ProgramFiles", r"C:\Program Files"), os.environ.get("ProgramW6432", "")):
            if root:
                installs += [Path(p) / "bin" for p in glob.glob(os.path.join(root, "KiCad", "*"))]
        found += sorted(set(installs), key=_version_key, reverse=True)
    elif system == "Darwin":
        found.append(Path("/Applications/KiCad/KiCad.app/Contents/MacOS"))
    cli = shutil.which("kicad-cli")
    if cli:
        found.append(Path(cli).resolve().parent)
    if system == "Linux":
        found += [Path("/usr/bin"), Path("/usr/local/bin")]
    return found


@lru_cache(maxsize=None)
def kicad_bin() -> Path:
    for d in _kicad_bin_candidates():
        if (d / f"kicad-cli{EXE}").exists():
            return d
    raise ToolError("KiCad not found: install KiCad 9 or newer, or set KICAD_BIN to the directory with kicad-cli")


@lru_cache(maxsize=None)
def kicad_cli() -> Path:
    return kicad_bin() / f"kicad-cli{EXE}"


@lru_cache(maxsize=None)
def kicad_python() -> Path:
    """A python interpreter that can import pcbnew."""
    candidates = []
    if os.environ.get("KICAD_PYTHON"):
        candidates.append(Path(os.environ["KICAD_PYTHON"]))
    try:
        b = kicad_bin()
        candidates += [b / f"python{EXE}", b / "python3"]
        if platform.system() == "Darwin":  # bundled framework python
            candidates += sorted(Path(p) for p in glob.glob(str(
                b.parent / "Frameworks" / "Python.framework" / "Versions" / "*" / "bin" / "python3")))
    except ToolError:
        pass
    candidates += [Path(p) for p in (shutil.which("python3"), shutil.which("python")) if p]
    for c in candidates:
        if c.exists() and _imports_pcbnew(c):
            return c
    raise ToolError("no python with KiCad's pcbnew module found: set KICAD_PYTHON")


def _imports_pcbnew(python: Path) -> bool:
    try:
        res = subprocess.run([str(python), "-c", "import pcbnew"], capture_output=True, timeout=120)
        return res.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


@lru_cache(maxsize=None)
def java() -> str:
    candidates = []
    if os.environ.get("JAVA"):
        candidates.append(os.environ["JAVA"])
    if os.environ.get("JAVA_HOME"):
        candidates.append(str(Path(os.environ["JAVA_HOME"]) / "bin" / f"java{EXE}"))
    # a JDK unpacked next to the repository (tools/jdk-25...) as build_freerouting.py leaves it
    candidates += [str(Path(p) / "bin" / f"java{EXE}") for p in sorted(glob.glob(str(REPO_DIR / "tools" / "jdk-*")), reverse=True)]
    if shutil.which("java"):
        candidates.append(shutil.which("java"))
    for c in candidates:
        if c and Path(c).exists() and java_major(c) >= 25:
            return c
    raise ToolError("Java 25 or newer not found: install a JDK 25 and set JAVA or JAVA_HOME")


def java_major(executable: str) -> int:
    try:
        res = subprocess.run([executable, "-version"], capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return 0
    text = res.stderr + res.stdout
    m = re.search(r'version "(\d+)', text)
    return int(m.group(1)) if m else 0


@lru_cache(maxsize=None)
def freerouting_jar() -> Path:
    candidates = []
    if os.environ.get("FREEROUTING_JAR"):
        candidates.append(Path(os.environ["FREEROUTING_JAR"]))
    candidates.append(REPO_DIR / "build" / "freerouting" / "build" / "libs" / "freerouting-current-executable.jar")
    for c in candidates:
        if c.exists():
            return c
    raise ToolError("Freerouting jar not found: run `python freerouting-patches/build_freerouting.py` "
                    "or set FREEROUTING_JAR")


def check() -> dict[str, str]:
    """Locate every tool; returns name -> path or the error message."""
    report = {}
    for name, fn in (("kicad-cli", kicad_cli), ("kicad python", kicad_python), ("java", java),
                     ("freerouting jar", freerouting_jar)):
        try:
            report[name] = str(fn())
        except ToolError as exc:
            report[name] = f"MISSING - {exc}"
    return report
