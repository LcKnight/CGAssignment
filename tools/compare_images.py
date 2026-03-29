import struct, zlib, math, os


def pick_output(root, name):
    candidates = [
        os.path.join(root, 'build', 'windows-mingw-release', name),
        os.path.join(root, 'build', 'windows-mingw-debug', name),
        os.path.join(root, name),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]

def read_png_rgb(path):
    """PNG reader supporting RGB and RGBA, returns (width, height, flat RGB list)"""
    with open(path, 'rb') as f:
        assert f.read(8) == b'\x89PNG\r\n\x1a\n'
        w = h = 0
        color_type = 2
        idat = b''
        while True:
            length = struct.unpack('>I', f.read(4))[0]
            ctype  = f.read(4)
            data   = f.read(length)
            f.read(4)
            if ctype == b'IHDR':
                w, h = struct.unpack('>II', data[:8])
                bit_depth  = data[8]
                color_type = data[9]
            elif ctype == b'IDAT':
                idat += data
            elif ctype == b'IEND':
                break
    ch = 4 if color_type == 6 else 3  # RGBA or RGB
    raw = zlib.decompress(idat)
    stride = w * ch
    prev_row = [0] * stride
    pixels = []
    idx = 0
    for y in range(h):
        ft  = raw[idx]; idx += 1
        row = list(raw[idx:idx+stride]); idx += stride
        if ft == 1:   # Sub
            for i in range(ch, len(row)): row[i] = (row[i] + row[i-ch]) & 0xFF
        elif ft == 2: # Up
            for i in range(len(row)): row[i] = (row[i] + prev_row[i]) & 0xFF
        elif ft == 3: # Average
            for i in range(len(row)):
                a = row[i-ch] if i>=ch else 0
                row[i] = (row[i] + (a + prev_row[i])//2) & 0xFF
        elif ft == 4: # Paeth
            for i in range(len(row)):
                a = row[i-ch] if i>=ch else 0
                b = prev_row[i]
                c = prev_row[i-ch] if i>=ch else 0
                pa=abs(b-c); pb=abs(a-c); pc=abs(a+b-2*c)
                pr = a if pa<=pb and pa<=pc else (b if pb<=pc else c)
                row[i] = (row[i] + pr) & 0xFF
        prev_row = row
        # Extract only RGB
        for x in range(w):
            pixels.extend(row[x*ch:x*ch+3])
    return w, h, pixels

def compare(ref_path, out_path, label):
    print(f"\n=== {label} ===")
    if not os.path.exists(ref_path):  print(f"  MISSING ref: {ref_path}");    return
    if not os.path.exists(out_path):  print(f"  MISSING output: {out_path}"); return
    rw, rh, rpx = read_png_rgb(ref_path)
    ow, oh, opx = read_png_rgb(out_path)
    print(f"  Ref:    {rw}x{rh}")
    print(f"  Output: {ow}x{oh}")
    w = min(rw, ow); h = min(rh, oh)
    if rw!=ow or rh!=oh: print("  WARNING: resolution mismatch")
    mse = 0.0; max_diff = 0; bias = [0.0,0.0,0.0]
    n = w*h
    for y in range(h):
        for x in range(w):
            for c in range(3):
                rv = rpx[(y*rw+x)*3+c]
                ov = opx[(y*ow+x)*3+c]
                d  = ov - rv
                mse += d*d; bias[c] += d
                if abs(d)>max_diff: max_diff=abs(d)
    mse /= n*3
    rmse = math.sqrt(mse)
    psnr = 10*math.log10(255*255/mse) if mse>0 else float('inf')
    print(f"  RMSE:  {rmse:.2f}  (0=perfect, <10=good)")
    print(f"  PSNR:  {psnr:.1f} dB  (>35=good)")
    print(f"  Max pixel diff: {max_diff}/255")
    br,bg,bb = bias[0]/n, bias[1]/n, bias[2]/n
    print(f"  Mean bias R={br:+.1f} G={bg:+.1f} B={bb:+.1f}  (output - ref)")
    o = (br+bg+bb)/3
    print(f"  -> {'BRIGHTER' if o>3 else 'DARKER' if o<-3 else 'close brightness'} than reference (mean diff={o:+.1f})")

root = r'C:\Users\lcknight\Desktop\tmp\CGAssignment1'
ref  = os.path.join(root, 'example-scenes-cg25')
compare(os.path.join(ref,'cornell-box.png'), pick_output(root, 'cornell-box.png'), 'Cornell Box')
compare(os.path.join(ref,'veach-mis.png'),   pick_output(root, 'veach-mis.png'),   'Veach MIS')
compare(os.path.join(ref,'living-room.png'), pick_output(root, 'living-room.png'), 'Living Room')
