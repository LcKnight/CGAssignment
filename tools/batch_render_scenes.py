import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    default_scenes_root = root / "example-scenes-cg25"
    default_output_dir = root / "rayTracing_report"

    parser = argparse.ArgumentParser(
        description="Batch render every scene directory with multiple spp values."
    )
    parser.add_argument(
        "exe_path",
        type=Path,
        help="Path to PathTracer.exe",
    )
    parser.add_argument(
        "--scenes-root",
        type=Path,
        default=default_scenes_root,
        help=f"Scene root directory (default: {default_scenes_root})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=default_output_dir,
        help=f"Directory for rendered outputs (default: {default_output_dir})",
    )
    parser.add_argument(
        "--spp",
        type=int,
        nargs="+",
        default=[16, 64, 128],
        help="Samples per pixel list, e.g. --spp 1 2 8 16",
    )
    parser.add_argument(
        "--keep-ppm",
        action="store_true",
        help="Keep the generated PPM files and rename them alongside PNG files",
    )
    return parser.parse_args()


def list_scene_dirs(scenes_root: Path) -> list[Path]:
    return sorted(path for path in scenes_root.iterdir() if path.is_dir())


def rename_output(output_dir: Path, scene_name: str, spp: int, suffix: str) -> None:
    src = output_dir / f"{scene_name}{suffix}"
    dst = output_dir / f"{scene_name}_{spp}{suffix}"
    if not src.exists():
        raise FileNotFoundError(f"Expected output file not found: {src}")
    if dst.exists():
        dst.unlink()
    shutil.move(str(src), str(dst))


def main() -> int:
    args = parse_args()

    exe_path = args.exe_path.resolve()
    scenes_root = args.scenes_root.resolve()
    output_dir = args.output_dir.resolve()
    spp_values = args.spp

    if not exe_path.is_file():
        print(f"PathTracer.exe not found: {exe_path}", file=sys.stderr)
        return 1
    if not scenes_root.is_dir():
        print(f"Scenes root not found: {scenes_root}", file=sys.stderr)
        return 1
    if any(spp <= 0 for spp in spp_values):
        print("All spp values must be positive integers.", file=sys.stderr)
        return 1

    scene_dirs = list_scene_dirs(scenes_root)
    if not scene_dirs:
        print(f"No scene directories found in: {scenes_root}", file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Using executable: {exe_path}")
    print(f"Scenes root: {scenes_root}")
    print(f"Output dir: {output_dir}")
    print(f"SPP list: {' '.join(str(spp) for spp in spp_values)}")

    total_jobs = len(scene_dirs) * len(spp_values)
    job_index = 0

    for scene_dir in scene_dirs:
        scene_name = scene_dir.name
        for spp in spp_values:
            job_index += 1
            print(f"[{job_index}/{total_jobs}] Rendering {scene_name} @ {spp} spp")
            cmd = [str(exe_path), str(scene_dir), str(spp)]
            result = subprocess.run(cmd, cwd=str(output_dir))
            if result.returncode != 0:
                print(
                    f"Render failed for scene '{scene_name}' at {spp} spp.",
                    file=sys.stderr,
                )
                return result.returncode

            rename_output(output_dir, scene_name, spp, ".png")
            ppm_path = output_dir / f"{scene_name}.ppm"
            if args.keep_ppm:
                rename_output(output_dir, scene_name, spp, ".ppm")
            elif ppm_path.exists():
                ppm_path.unlink()

    print("All renders completed successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
