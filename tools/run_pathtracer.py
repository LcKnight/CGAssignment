import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = ROOT / "build" / "windows-mingw-release"
EXE = BUILD_DIR / "PathTracer.exe"
MINGW_BIN = Path(r"C:/Compilers/msys64/mingw64/bin")


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: python tools/run_pathtracer.py <scene_dir> [spp]")
        return 1

    scene = sys.argv[1]
    spp = sys.argv[2] if len(sys.argv) > 2 else "64"

    env = os.environ.copy()
    env["PATH"] = str(MINGW_BIN) + os.pathsep + env.get("PATH", "")

    result = subprocess.run(
        [str(EXE), scene, spp],
        cwd=str(BUILD_DIR),
        env=env,
    )
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
