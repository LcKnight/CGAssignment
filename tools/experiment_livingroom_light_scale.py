import math
import os
import re
import shutil
import struct
import subprocess
import zlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = ROOT / "build" / "windows-mingw-release"
EXE = BUILD_DIR / "PathTracer.exe"
MINGW_BIN = Path(r"C:/Compilers/msys64/mingw64/bin")
SRC_SCENE = ROOT / "example-scenes-cg25/living-room"
REF_IMG = ROOT / "example-scenes-cg25/living-room.png"
TMP_ROOT = ROOT / "tmp" / "living-room-light-scale"


def read_png_rgb(path: Path):
    with path.open("rb") as f:
        assert f.read(8) == b"\x89PNG\r\n\x1a\n"
        width = 0
        height = 0
        color_type = 2
        idat = b""
        while True:
            n = struct.unpack(">I", f.read(4))[0]
            t = f.read(4)
            d = f.read(n)
            f.read(4)
            if t == b"IHDR":
                width, height = struct.unpack(">II", d[:8])
                color_type = d[9]
            elif t == b"IDAT":
                idat += d
            elif t == b"IEND":
                break
    ch = 4 if color_type == 6 else 3
    raw = zlib.decompress(idat)
    stride = width * ch
    idx = 0
    prev = [0] * stride
    out = []
    for _ in range(height):
        ft = raw[idx]
        idx += 1
        row = list(raw[idx:idx + stride])
        idx += stride

        if ft == 1:
            for i in range(ch, len(row)):
                row[i] = (row[i] + row[i - ch]) & 0xFF
        elif ft == 2:
            for i in range(len(row)):
                row[i] = (row[i] + prev[i]) & 0xFF
        elif ft == 3:
            for i in range(len(row)):
                a = row[i - ch] if i >= ch else 0
                row[i] = (row[i] + ((a + prev[i]) // 2)) & 0xFF
        elif ft == 4:
            for i in range(len(row)):
                a = row[i - ch] if i >= ch else 0
                b = prev[i]
                c = prev[i - ch] if i >= ch else 0
                pa = abs(b - c)
                pb = abs(a - c)
                pc = abs(a + b - 2 * c)
                pr = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
                row[i] = (row[i] + pr) & 0xFF

        prev = row
        for x in range(width):
            out.extend(row[x * ch:x * ch + 3])
    return width, height, out


def compare(ref_path: Path, out_path: Path):
    rw, rh, rpx = read_png_rgb(ref_path)
    ow, oh, opx = read_png_rgb(out_path)
    w = min(rw, ow)
    h = min(rh, oh)
    n = w * h
    mse = 0.0
    bias = [0.0, 0.0, 0.0]
    for y in range(h):
        for x in range(w):
            for c in range(3):
                rv = rpx[(y * rw + x) * 3 + c]
                ov = opx[(y * ow + x) * 3 + c]
                d = ov - rv
                mse += d * d
                bias[c] += d
    mse /= n * 3
    rmse = math.sqrt(mse)
    return rmse, bias[0] / n, bias[1] / n, bias[2] / n, (bias[0] + bias[1] + bias[2]) / (3 * n)


def scaled_scene(scale: float) -> Path:
    name = f"living-room-s{str(scale).replace('.', '_')}"
    dst = TMP_ROOT / name
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(SRC_SCENE, dst)

    xml = (dst / "scene.xml").read_text(encoding="utf-8")
    m = re.search(r'radiance\s*=\s*"\s*([0-9.]+)\s*,\s*([0-9.]+)\s*,\s*([0-9.]+)\s*"', xml)
    if not m:
        raise RuntimeError("Could not find radiance in scene.xml")
    r, g, b = [float(m.group(i)) for i in (1, 2, 3)]
    repl = f'radiance="{r * scale:.6f}, {g * scale:.6f}, {b * scale:.6f}"'
    xml = re.sub(r'radiance\s*=\s*"[^"]+"', repl, xml)
    (dst / "scene.xml").write_text(xml, encoding="utf-8")
    return dst


def render(scene_dir: Path, spp: int):
    env = os.environ.copy()
    env["PATH"] = str(MINGW_BIN) + os.pathsep + env.get("PATH", "")
    subprocess.run([str(EXE), str(scene_dir), str(spp)], cwd=str(BUILD_DIR), env=env, check=True)


def main():
    TMP_ROOT.mkdir(parents=True, exist_ok=True)
    scales = [1.0, 0.85, 0.7, 0.6, 0.5, 0.4]
    spp = 64
    rows = []

    for scale in scales:
        scene = scaled_scene(scale)
        render(scene, spp)
        out = BUILD_DIR / f"{scene.name}.png"
        rmse, br, bg, bb, bm = compare(REF_IMG, out)
        rows.append((scale, rmse, br, bg, bb, bm, out))

    rows.sort(key=lambda x: abs(x[5]))
    print("scale, rmse, bias_r, bias_g, bias_b, bias_mean, output")
    for r in rows:
        print(f"{r[0]:.2f}, {r[1]:.2f}, {r[2]:+.2f}, {r[3]:+.2f}, {r[4]:+.2f}, {r[5]:+.2f}, {r[6]}")


if __name__ == "__main__":
    main()
