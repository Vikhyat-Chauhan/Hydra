#!/usr/bin/env python3
import argparse, math, json, os, sys
from dataclasses import dataclass
import numpy as np

try:
    # pip install noise
    from noise import pnoise2
except ImportError:
    print("Please `pip install noise` (Perlin) first.", file=sys.stderr)
    sys.exit(1)

# --- Map & grid config (matches your world) ---
X_MIN, X_MAX = -100.0, 100.0
Y_MIN, Y_MAX =  -50.0,  50.0
CELL_M   = 5.0   # grid cell ≈ 2 m (your example)
Z_THICK  = 5.0  # obstacle slab thickness
Z_EPS    = 0.00  # small height above ground to avoid z-fighting

# Visual style
DIFFUSE = (1.0, 0.0, 0.0, 1.0)  # light red, semi-transparent

@dataclass
class Args:
    out_sdf: str
    out_meta: str
    density: float   # p in (0,1)
    corr_len_m: float
    seed: int

def build_grid():
    xs = np.arange(X_MIN, X_MAX, CELL_M)  # left edges
    ys = np.arange(Y_MIN, Y_MAX, CELL_M)  # bottom edges
    return xs, ys

def sample_perlin(xs, ys, corr_len_m, seed):
    # Map meters to Perlin units so that spatial correlation ~ corr_len_m
    # Frequency ≈ 1 / corr_len_m (empirical, tweakable)
    freq = 1.0 / max(corr_len_m, 1e-6)
    z = np.zeros((len(xs), len(ys)), dtype=np.float32)
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            # Scale & seed via offsets
            zx = (x - xs[0]) * freq + seed * 1.1337
            zy = (y - ys[0]) * freq - seed * 0.7331
            z[i, j] = pnoise2(zx, zy, octaves=3, repeatx=1024, repeaty=1024, base=seed)
    # normalize to [0,1]
    z = (z - np.min(z)) / max(np.ptp(z), 1e-9)
    return z

def mask_by_density(noise_map, density):
    # Threshold at quantile so that fraction ≈ density
    thr = np.quantile(noise_map, 1.0 - density)
    mask = (noise_map >= thr)
    return mask, float(thr)

def merge_rects(mask):
    """
    Merge contiguous True cells into rectangles.
    Returns list of rectangles in grid indices: (i0, j0, w, h).
    i -> x index, j -> y index
    """
    used = np.zeros_like(mask, dtype=bool)
    rects = []
    nx, ny = mask.shape
    for i in range(nx):
        j = 0
        while j < ny:
            if mask[i, j] and not used[i, j]:
                # grow width in j (y)
                w = 1
                while j + w < ny and mask[i, j + w] and not used[i, j + w]:
                    w += 1
                # grow height in i (x) as long as entire row segment is valid
                h = 1
                good = True
                while i + h < nx and good:
                    for jj in range(j, j + w):
                        if not (mask[i + h, jj] and not used[i + h, jj]):
                            good = False
                            break
                    if good:
                        h += 1
                # mark used
                for ii in range(i, i + h):
                    for jj in range(j, j + w):
                        used[ii, jj] = True
                rects.append((i, j, h, w))
                j += w
            else:
                j += 1
    return rects

def rect_to_world(xs, ys, rect):
    i0, j0, h, w = rect
    # grid cell [i,j] spans [x_i, x_i+CELL_M] × [y_j, y_j+CELL_M]
    x0 = xs[i0]; y0 = ys[j0]
    w_m = w * CELL_M
    h_m = h * CELL_M
    cx = x0 + w_m/2.0
    cy = y0 + h_m/2.0
    return cx, cy, w_m, h_m

def write_sdf(rects, xs, ys, out_sdf_path):
    r,g,b,a = DIFFUSE
    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append('<sdf version="1.9">')
    lines.append('  <model name="restricted_zones">')
    lines.append('    <static>true</static>')
    lines.append('    <pose>0 0 0 0 0 0</pose>')
    lines.append('    <link name="link">')
    for k, rect in enumerate(rects):
        cx, cy, w_m, h_m = rect_to_world(xs, ys, rect)
        # collision
        lines.append(f'      <collision name="c{k}">')
        lines.append(f'        <pose>{cx:.3f} {cy:.3f} {Z_EPS:.3f} 0 0 0</pose>')
        lines.append(f'        <geometry><box><size>{w_m:.3f} {h_m:.3f} {Z_THICK:.3f}</size></box></geometry>')
        lines.append('      </collision>')
        # visual
        lines.append(f'      <visual name="v{k}">')
        lines.append(f'        <pose>{cx:.3f} {cy:.3f} {Z_EPS:.3f} 0 0 0</pose>')
        lines.append(f'        <geometry><box><size>{w_m:.3f} {h_m:.3f} {Z_THICK:.3f}</size></box></geometry>')
        lines.append('        <material>')
        lines.append(f'          <diffuse>{r} {g} {b} {a}</diffuse>')
        lines.append(f'          <ambient>{r} {g} {b} {a}</ambient>')
        lines.append('          <specular>0.1 0.1 0.1 1</specular>')
        lines.append('        </material>')
        lines.append('      </visual>')
    lines.append('    </link>')
    lines.append('  </model>')
    lines.append('</sdf>')
    with open(out_sdf_path, 'w') as f:
        f.write("\n".join(lines))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-sdf",  default=os.path.join("models/generated","generated_nofly.sdf"))
    ap.add_argument("--out-meta", default=os.path.join("models/generated","generated_nofly_meta.json"))
    ap.add_argument("--density",  type=float, default=0.2, help="fraction of blocked area (0..1)")
    ap.add_argument("--corr-len", type=float, default=10.0, help="correlation length in meters")
    ap.add_argument("--seed",     type=int,   default=0)
    args = ap.parse_args()
    A = Args(args.out_sdf, args.out_meta, args.density, args.corr_len, args.seed)

    xs, ys = build_grid()
    nmap = sample_perlin(xs, ys, A.corr_len_m, A.seed)
    mask, thr = mask_by_density(nmap, A.density)
    rects = merge_rects(mask)

    os.makedirs(os.path.dirname(A.out_sdf), exist_ok=True)
    write_sdf(rects, xs, ys, A.out_sdf)

    # Save metadata for the metrics logger (to compute violations in 2D)
    meta = {
        "x_min": X_MIN, "x_max": X_MAX, "y_min": Y_MIN, "y_max": Y_MAX,
        "cell_m": CELL_M, "density": A.density, "corr_len_m": A.corr_len_m,
        "seed": A.seed, "threshold": thr,
        "rectangles_xywh": [rect_to_world(xs, ys, r) for r in rects]  # (cx,cy,w,h)
    }
    with open(A.out_meta, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"Wrote: {A.out_sdf} and {A.out_meta}. Rectangles: {len(rects)}")

if __name__ == "__main__":
    main()
