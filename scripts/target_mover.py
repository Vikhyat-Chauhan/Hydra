#!/usr/bin/env python3
import sys, argparse, os, json, random

# Cage bounds (match your world)
X_MIN, X_MAX = -100.0, 100.0
Y_MIN, Y_MAX =  -50.0,  50.0

def inside_rect(x, y, cx, cy, w, h, margin=0.0):
    return (abs(x - cx) <= w/2.0 - margin) and (abs(y - cy) <= h/2.0 - margin)

def pick_safe_xy(meta_path, margin_walls=5.0, margin_rect=1.0, seed=None):
    rnd = random.Random(seed)
    rects = []
    if os.path.isfile(meta_path):
        try:
            with open(meta_path) as f:
                meta = json.load(f)
            rects = meta.get("rectangles_xywh", [])
        except Exception:
            pass

    for _ in range(5000):
        x = rnd.uniform(X_MIN + margin_walls, X_MAX - margin_walls)
        y = rnd.uniform(Y_MIN + margin_walls, Y_MAX - margin_walls)
        if all(not inside_rect(x, y, cx, cy, w, h, margin=margin_rect)
               for (cx, cy, w, h) in rects):
            return x, y
    # fallback if everything is blocked
    return 0.0, 0.0

def make_sphere_sdf(name, radius, rgba, x, y, z):
    r, g, b, a = (float(v) for v in rgba.split(","))
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

def main():
    p = argparse.ArgumentParser(description="Generate SDF for a static visual sphere at a safe (x,y).")
    p.add_argument("--name", default="target_sphere")
    p.add_argument("--radius", type=float, default=0.6)
    p.add_argument("--color", default="1,0.9,0,1", help="RGBA as r,g,b,a")
    p.add_argument("--z", type=float, default=15.0, help="altitude")
    p.add_argument("--out", default="/tmp/target.sdf", help="Output SDF path (default: /tmp/target.sdf)")
    p.add_argument("--meta-in", default=os.path.join("worlds","generated_nofly_meta.json"),
                   help="Path to generated_nofly_meta.json")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--margin-walls", type=float, default=5.0)
    p.add_argument("--margin-rect", type=float, default=1.0)
    args = p.parse_args()

    x, y = pick_safe_xy(args.meta_in, args.margin_walls, args.margin_rect, seed=args.seed)
    xml = make_sphere_sdf(args.name, args.radius, args.color, x, y, args.z)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        f.write(xml)

    # Print coords to stderr so stdout stays clean if you redirect output
    print(f"[make_sphere_sdf] Safe pose -> x={x:.3f}, y={y:.3f}, z={args.z:.3f}", file=sys.stderr)
    print(f"[make_sphere_sdf] Wrote {os.path.abspath(args.out)}", file=sys.stderr)

if __name__ == "__main__":
    main()
