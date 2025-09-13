#!/usr/bin/env python3
import argparse, json, os, sys, random
from dataclasses import dataclass
from typing import Tuple, List

# --- Map bounds (match your world) ---
X_MIN, X_MAX = -100.0, 100.0
Y_MIN, Y_MAX =  -50.0,  50.0

@dataclass
class Args:
    out_sdf: str
    out_meta: str
    in_meta_nofly: str
    z: float
    radius: float
    color: Tuple[float, float, float, float]
    seed: int
    margin_walls: float
    margin_rect: float
    name: str

def parse_rgba(s: str) -> Tuple[float, float, float, float]:
    parts = [float(x.strip()) for x in s.split(",")]
    if len(parts) != 4:
        print("ERROR: --color must be 'r,g,b,a'", file=sys.stderr)
        sys.exit(2)
    return tuple(parts)  # type: ignore

def inside_rect(x: float, y: float, cx: float, cy: float, w: float, h: float, margin: float=0.0) -> bool:
    return (abs(x - cx) <= w/2.0 - margin) and (abs(y - cy) <= h/2.0 - margin)

def load_restricted_rects(meta_path: str) -> List[Tuple[float, float, float, float]]:
    if not os.path.isfile(meta_path):
        return []
    try:
        with open(meta_path) as f:
            meta = json.load(f)
        rects = meta.get("rectangles_xywh", [])
        # ensure tuples of 4 floats
        return [tuple(map(float, r)) for r in rects]
    except Exception as e:
        print(f"[warn] Failed to read {meta_path}: {e}", file=sys.stderr)
        return []

def pick_safe_xy(rects: List[Tuple[float,float,float,float]],
                 margin_walls: float, margin_rect: float, seed: int | None) -> Tuple[float,float]:
    rnd = random.Random(seed)
    for _ in range(5000):
        x = rnd.uniform(X_MIN + margin_walls, X_MAX - margin_walls)
        y = rnd.uniform(Y_MIN + margin_walls, Y_MAX - margin_walls)
        if all(not inside_rect(x, y, cx, cy, w, h, margin=margin_rect) for (cx,cy,w,h) in rects):
            return x, y
    # fallback if everything is blocked
    return 0.0, 0.0

def make_sphere_sdf(name: str, radius: float, rgba: Tuple[float,float,float,float], x: float, y: float, z: float) -> str:
    r,g,b,a = rgba
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><sphere><radius>{radius:.3f}</radius></sphere></geometry>
        <material>
          <diffuse>{r} {g} {b} {a}</diffuse>
          <ambient>{r} {g} {b} {a}</ambient>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

def write_text(path: str, text: str):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)

def main():
    ap = argparse.ArgumentParser(description="Generate per-run target sphere SDF (avoids restricted zones).")
    ap.add_argument("--out-sdf",  default=os.path.join("models/generated","generated_target.sdf"))
    ap.add_argument("--out-meta", default=os.path.join("models/generated","generated_target_meta.json"))
    ap.add_argument("--in-meta-nofly", default=os.path.join("models/generated","generated_nofly_meta.json"),
                    help="Input metadata produced by generate_restricted_zones.py")
    ap.add_argument("--z", type=float, default=0.0, help="target altitude")
    ap.add_argument("--radius", type=float, default=0.6)
    ap.add_argument("--color", default="1,0.9,0,1", help="RGBA as r,g,b,a")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--margin-walls", type=float, default=5.0)
    ap.add_argument("--margin-rect",  type=float, default=1.0)
    ap.add_argument("--name", default="target_sphere")
    args_ns = ap.parse_args()

    A = Args(
        out_sdf=args_ns.out_sdf,
        out_meta=args_ns.out_meta,
        in_meta_nofly=args_ns.in_meta_nofly,
        z=args_ns.z,
        radius=args_ns.radius,
        color=parse_rgba(args_ns.color),
        seed=args_ns.seed,
        margin_walls=args_ns.margin_walls,
        margin_rect=args_ns.margin_rect,
        name=args_ns.name,
    )

    rects = load_restricted_rects(A.in_meta_nofly)
    x, y = pick_safe_xy(rects, A.margin_walls, A.margin_rect, A.seed)
    sdf_xml = make_sphere_sdf(A.name, A.radius, A.color, x, y, A.z)

    write_text(A.out_sdf, sdf_xml)
    meta = {"name": A.name, "x": x, "y": y, "z": A.z, "radius": A.radius, "color": A.color}
    write_text(A.out_meta, json.dumps(meta, indent=2))

    print(f"Wrote: {A.out_sdf} and {A.out_meta}. Target at ({x:.2f}, {y:.2f}, {A.z:.2f})")

if __name__ == "__main__":
    main()
