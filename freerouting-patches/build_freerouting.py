"""Build the Freerouting jar the pipeline needs: upstream Freerouting at a pinned commit plus the
patches in this directory.

  python freerouting-patches/build_freerouting.py [--jdk PATH_TO_JDK25]

Needs git and a JDK 25 (JAVA_HOME, --jdk, or `java` on PATH). The jar ends up in
build/freerouting/build/libs/freerouting-current-executable.jar, where kicad-autoroute finds it.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

UPSTREAM = "https://github.com/freerouting/freerouting.git"
COMMIT = "5aa1b24dc8e5af7f950c551c1e53de9077a84cd5"
PATCHES = ["0001-min-trace-width.patch", "0002-bus-attraction.patch"]

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
SRC = REPO / "build" / "freerouting"


def run(cmd: list[str], cwd: Path | None = None, env: dict | None = None) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=cwd, env=env, check=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--jdk", help="JDK 25 home directory (default: JAVA_HOME)")
    ns = ap.parse_args()
    if not shutil.which("git"):
        sys.exit("git is needed to fetch Freerouting")
    env = dict(os.environ)
    if ns.jdk:
        env["JAVA_HOME"] = ns.jdk
    if not env.get("JAVA_HOME"):
        sys.exit("set JAVA_HOME (or --jdk) to a JDK 25")

    if not (SRC / ".git").exists():
        SRC.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "clone", "--filter=blob:none", "--no-checkout", UPSTREAM, str(SRC)])
    run(["git", "fetch", "--filter=blob:none", "origin", COMMIT], cwd=SRC)
    run(["git", "checkout", "--force", "--detach", COMMIT], cwd=SRC)
    run(["git", "clean", "-fdq"], cwd=SRC)
    for patch in PATCHES:
        run(["git", "apply", "--whitespace=nowarn", str(HERE / patch)], cwd=SRC)
    gradlew = SRC / ("gradlew.bat" if os.name == "nt" else "gradlew")
    if os.name != "nt":
        gradlew.chmod(0o755)
    run([str(gradlew), "--no-daemon", "-q", "executableJar"], cwd=SRC, env=env)
    jar = SRC / "build" / "libs" / "freerouting-current-executable.jar"
    if not jar.exists():
        sys.exit("build finished but the jar is missing")
    print(f"built {jar}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
