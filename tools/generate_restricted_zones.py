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
CELL_M   = 5.0    # grid cell size
Z_THICK  = 5.0    # obstacle slab thickness
Z_EPS    = 0.00   # small height above ground to avoid z-fighting

@dataclass
class Args:
    out_sdf: str
    out_meta: str
    density: float          # p in (0,1)
    corr_len_m: float
    seed: int
    pass_through: bool      # if True: NO collision blocks are emitted
    visual_alpha: float     # 0..1 (0 opaque, 1 fully transparent)
    color: tuple            # RGB (no alpha)

def build_grid():
    xs = np.arange(X_MIN, X_MAX, CELL_M)  # left edges
    ys = np.arange(Y_MIN, Y_MAX, CELL_M)  # bottom edges
    return xs, ys

def sample_perlin(xs, ys, corr_len_m, seed):
    # Map meters to Perlin units so that spatial correlation ~ corr_len_m
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
    Returns list of rectangles in grid indices: (i0, j0, h, w).
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

def write_sdf(rects, xs, ys, out_sdf_path, pass_through, visual_alpha, color_rgb):
    """
    pass_through=True  -> VISUAL-ONLY (no <collision>) so the drone can pass through.
    visual_alpha in [0,1]: 0 = opaque, 1 = fully transparent.
    """
    r, g, b = color_rgb
    # SDF uses visual-level <transparency> where 0=opaque, 1=fully transparent.
    transparency = max(0.0, min(1.0, visual_alpha))
    # Many renderers also respect the alpha channel on <diffuse>.
    a = 1.0 - transparency  # 1=opaque, 0=fully transparent

    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append('<sdf version="1.9">')
    lines.append('  <model name="restricted_zones">')
    lines.append('    <static>true</static>')
    lines.append('    <pose>0 0 0 0 0 0</pose>')
    lines.append('    <link name="link">')
    for k, rect in enumerate(rects):
        cx, cy, w_m, h_m = rect_to_world(xs, ys, rect)

        # Only add collision if NOT pass-through
        if not pass_through:
            lines.append(f'      <collision name="c{k}">')
            lines.append(f'        <pose>{cx:.3f} {cy:.3f} {Z_EPS:.3f} 0 0 0</pose>')
            lines.append(f'        <geometry><box><size>{w_m:.3f} {h_m:.3f} {Z_THICK:.3f}</size></box></geometry>')
            lines.append('      </collision>')

        # Visual
        lines.append(f'      <visual name="v{k}">')
        lines.append(f'        <pose>{cx:.3f} {cy:.3f} {Z_EPS:.3f} 0 0 0</pose>')
        lines.append(f'        <geometry><box><size>{w_m:.3f} {h_m:.3f} {Z_THICK:.3f}</size></box></geometry>')
        # Correct place for transparency:
        lines.append(f'        <transparency>{transparency:.4f}</transparency>')
        lines.append('        <material>')
        lines.append(f'          <diffuse>{r:.4f} {g:.4f} {b:.4f} {a:.4f}</diffuse>')
        lines.append(f'          <ambient>{r:.4f} {g:.4f} {b:.4f} {a:.4f}</ambient>')
        lines.append('          <specular>0.1 0.1 0.1 1</specular>')
        lines.append('        </material>')
        lines.append('      </visual>')
    lines.append('    </link>')
    lines.append('  </model>')
    lines.append('</sdf>')
    with open(out_sdf_path, 'w') as f:
        f.write("\n".join(lines))


def parse_color(s):
    # "r,g,b" in 0..1
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Color must be 'r,g,b' with values in [0,1].")
    try:
        r, g, b = map(float, parts)
    except ValueError:
        raise argparse.ArgumentTypeError("Color values must be floats.")
    for v in (r, g, b):
        if not (0.0 <= v <= 1.0):
            raise argparse.ArgumentTypeError("Color values must be in [0,1].")
    return (r, g, b)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-sdf",  default=os.path.join("models/generated","generated_nofly.sdf"))
    ap.add_argument("--out-meta", default=os.path.join("models/generated","generated_nofly_meta.json"))
    ap.add_argument("--density",  type=float, default=0.2, help="fraction of blocked area (0..1)")
    ap.add_argument("--corr-len", type=float, default=10.0, help="correlation length in meters")
    ap.add_argument("--seed",     type=int,   default=0)
    ap.add_argument("--pass-through", action="store_true",
                    help="If set, do NOT emit <collision> blocks (visual-only, drone can pass through).")
    ap.add_argument("--visual-alpha", type=float, default=0.3,
                    help="Visual transparency in [0,1]; 0=opaque, 1=invisible.")
    ap.add_argument("--color", type=parse_color, default=(1.0, 0.0, 0.0),
                    help="Base RGB color as 'r,g,b' in [0,1] (default: 1,0,0 for red).")
    args = ap.parse_args()

    # Bundle dataclass (handy if you extend later)
    A = Args(
        out_sdf=args.out_sdf,
        out_meta=args.out_meta,
        density=args.density,
        corr_len_m=args.corr_len,
        seed=args.seed,
        pass_through=args.pass_through,
        visual_alpha=max(0.0, min(1.0, args.visual_alpha)),
        color=args.color
    )

    xs, ys = build_grid()
    nmap = sample_perlin(xs, ys, A.corr_len_m, A.seed)
    mask, thr = mask_by_density(nmap, A.density)
    rects = merge_rects(mask)

    os.makedirs(os.path.dirname(A.out_sdf), exist_ok=True)
    write_sdf(rects, xs, ys, A.out_sdf, A.pass_through, A.visual_alpha, A.color)

    # Save metadata for the metrics logger (to compute violations in 2D)
    meta = {
        "x_min": X_MIN, "x_max": X_MAX, "y_min": Y_MIN, "y_max": Y_MAX,
        "cell_m": CELL_M, "density": A.density, "corr_len_m": A.corr_len_m,
        "seed": A.seed, "threshold": thr,
        "pass_through": A.pass_through,
        "visual_alpha": A.visual_alpha,
        "color_rgb": A.color,
        "rectangles_xywh": [rect_to_world(xs, ys, r) for r in rects]  # (cx,cy,w,h)
    }
    with open(A.out_meta, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"Wrote: {A.out_sdf} and {A.out_meta}. Rectangles: {len(rects)} | pass_through={A.pass_through} | visual_alpha={A.visual_alpha:.2f}")

if __name__ == "__main__":
    main()
