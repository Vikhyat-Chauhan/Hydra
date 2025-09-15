"""
Exact match: python3 gz_pose_filter.py --name drone1

Substring match: python3 gz_pose_filter.py --contains rotor_

Regex match: python3 gz_pose_filter.py --regex '^v(3|4|5)$'

JSON output: python3 gz_pose_filter.py --name target_sphere --json

Names in your sample include "drone1", "target_sphere", "X3/rotor_0"… so --contains rotor_ will capture the four rotors, and --name drone1 will capture the model root.

Add --once if you only want the next tick; omit it to keep streaming updates.

"""



#!/usr/bin/env python3
import argparse, json, subprocess, sys, re

def coerce_float(v, default=0.0):
    if v is None: return default
    try:
        return float(v)
    except (ValueError, TypeError):
        return default

def match_name(name, args):
    if args.name and name == args.name:
        return True
    if args.contains and any(s in name for s in args.contains):
        return True
    if args.regex and args._regex and args._regex.search(name):
        return True
    return False

def format_line(p):
    pos = p.get("position", {}) or {}
    ori = p.get("orientation", {}) or {}
    x = coerce_float(pos.get("x", 0.0))
    y = coerce_float(pos.get("y", 0.0))
    z = coerce_float(pos.get("z", 0.0))
    w = coerce_float(ori.get("w", 1.0))
    qx = coerce_float(ori.get("x", 0.0))
    qy = coerce_float(ori.get("y", 0.0))
    qz = coerce_float(ori.get("z", 0.0))
    return x, y, z, w, qx, qy, qz

def main():
    ap = argparse.ArgumentParser(
        description="Filter poses from `gz topic ... --json-output` by name."
    )
    ap.add_argument("--topic", "-t", default="/world/airport/pose/info",
                    help="Gazebo topic (default: /world/airport/pose/info)")
    ap.add_argument("--name", help="Exact name match (e.g., 'drone1')")
    ap.add_argument("--contains", nargs="+",
                    help="Substring match; any token matching passes (e.g., rotor_ X3/)")
    ap.add_argument("--regex", help="Python regex to match names (e.g., '^v[0-9]+$')")
    ap.add_argument("--json", action="store_true",
                    help="Print matching poses as compact JSON (one line per pose)")
    ap.add_argument("--once", action="store_true",
                    help="Exit after printing the first matching pose set")
    args = ap.parse_args()

    args._regex = re.compile(args.regex) if args.regex else None

    # Launch gz stream
    cmd = ["gz", "topic", "-e", "-t", args.topic, "--json-output"]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
    except FileNotFoundError:
        print("Error: `gz` not found in PATH.", file=sys.stderr)
        sys.exit(1)

    try:
        for line in proc.stdout:
            line = line.strip()
            if not line or not line.startswith("{"):
                continue

            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue

            poses = msg.get("pose", [])
            printed_any = False
            for p in poses:
                name = p.get("name", "")
                if not name:
                    continue
                if not match_name(name, args):
                    continue

                x, y, z, w, qx, qy, qz = format_line(p)

                if args.json:
                    out = {
                        "name": name,
                        "position": {"x": x, "y": y, "z": z},
                        "orientation": {"w": w, "x": qx, "y": qy, "z": qz},
                        "id": p.get("id")
                    }
                    print(json.dumps(out, separators=(",", ":")))
                else:
                    print(f"{name:>24s} | pos: ({x:.3f}, {y:.3f}, {z:.3f}) | "
                          f"quat (w,x,y,z): ({w:.6f}, {qx:.6f}, {qy:.6f}, {qz:.6f})")
                printed_any = True

            # If you only want one snapshot of matches, stop after first batch that had any
            if args.once and printed_any:
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            proc.terminate()
        except Exception:
            pass

if __name__ == "__main__":
    main()
