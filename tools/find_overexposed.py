import struct, zlib, math, sys

def read_png_rgb(path):
    with open(path, 'rb') as f:
        assert f.read(8) == b'\x89PNG\r\n\x1a\n'
        w = h = 0; color_type = 2; idat = b''
        while True:
            length = struct.unpack('>I', f.read(4))[0]
            ctype = f.read(4); data = f.read(length); f.read(4)
            if ctype == b'IHDR':
                w, h = struct.unpack('>II', data[:8]); color_type = data[9]
            elif ctype == b'IDAT': idat += data
            elif ctype == b'IEND': break
    ch = 4 if color_type == 6 else 3
    raw = zlib.decompress(idat)
    stride = w * ch; prev_row = [0]*stride; pixels = []
    idx = 0
    for y in range(h):
        ft = raw[idx]; idx += 1
        row = list(raw[idx:idx+stride]); idx += stride
        if ft == 1:
            for i in range(ch, len(row)): row[i] = (row[i] + row[i-ch]) & 0xFF
        elif ft == 2:
            for i in range(len(row)): row[i] = (row[i] + prev_row[i]) & 0xFF
        elif ft == 3:
            for i in range(len(row)):
                a = row[i-ch] if i>=ch else 0
                row[i] = (row[i] + (a + prev_row[i])//2) & 0xFF
        elif ft == 4:
            for i in range(len(row)):
                a = row[i-ch] if i>=ch else 0; b = prev_row[i]; c = prev_row[i-ch] if i>=ch else 0
                pa=abs(b-c); pb=abs(a-c); pc=abs(a+b-2*c)
                p = a if pa<=pb and pa<=pc else (b if pb<=pc else c)
                row[i] = (row[i] + p) & 0xFF
        pixels.extend(row[:w*3] if ch==4 else row)
        prev_row = row
    return w, h, pixels

ref_path = sys.argv[1] if len(sys.argv)>1 else 'example-scenes-cg25/living-room.png'
out_path = sys.argv[2] if len(sys.argv)>2 else 'living-room.png'

rw, rh, rpx = read_png_rgb(ref_path)
ow, oh, opx = read_png_rgb(out_path)
print(f'Ref: {rw}x{rh}, Out: {ow}x{oh}')

# Find pixels where output is significantly brighter
bright = []
for y in range(min(rh,oh)):
    for x in range(min(rw,ow)):
        ri = (y*rw+x)*3; oi = (y*ow+x)*3
        rd = (rpx[ri]+rpx[ri+1]+rpx[ri+2])//3
        od = (opx[oi]+opx[oi+1]+opx[oi+2])//3
        diff = od - rd
        if diff > 60:
            bright.append((diff, x, y, od, rd,
                           opx[oi], opx[oi+1], opx[oi+2],
                           rpx[ri], rpx[ri+1], rpx[ri+2]))
bright.sort(reverse=True)
print(f'Pixels brighter by >60: {len(bright)}')
print('Top 30 overexposed pixels:')
print('  diff   x    y   out(R,G,B)      ref(R,G,B)')
for item in bright[:30]:
    d,x,y,ol,rl,or_,og,ob,rr,rg,rb = item
    print(f'  {d:3d}  {x:4d} {y:4d}   ({or_:3d},{og:3d},{ob:3d})   ({rr:3d},{rg:3d},{rb:3d})')

# Also show region density — where are overexposed pixels clustered?
if bright:
    xs = [b[1] for b in bright]; ys = [b[2] for b in bright]
    print(f'\nOverexposed region: x=[{min(xs)},{max(xs)}] y=[{min(ys)},{max(ys)}]')
    # bucket into 8x8 grid cells
    bw, bh = rw//8, rh//8
    grid = {}
    for d,x,y,*_ in bright:
        gx, gy = x//bw, y//bh
        grid[(gx,gy)] = grid.get((gx,gy), 0) + 1
    top_cells = sorted(grid.items(), key=lambda kv: -kv[1])[:5]
    print('Top grid cells (8x8 grid):')
    for (gx,gy), cnt in top_cells:
        print(f'  cell({gx},{gy}) pixel_range x=[{gx*bw},{(gx+1)*bw}] y=[{gy*bh},{(gy+1)*bh}]  count={cnt}')
