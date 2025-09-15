#!/usr/bin/env python3
"""
Hydra minimal controller (transport-only, no CLI):
- Subscribes to Pose_V on /world/*/pose/info, or falls back to /model/<name>/pose
- Publishes Twist to /model/<name>/cmd_vel
- Straight-line P controller with altitude hold
"""

import argparse, json, math, re, signal, sys, time
from dataclasses import dataclass

# ----------------------------- Topics -----------------------------
WORLD_POSE_TOPICS = [
    "/world/airport/pose/info",
]
MODEL_POSE_TOPIC_TMPL = "/model/{model}/pose"
CMD_VEL_TOPIC_TMPL    = "/model/{model}/cmd_vel"

# ----------------------------- Msgs/APIs --------------------------
try:
    import gz.msgs10.pose_v_pb2 as pose_v_pb2
    import gz.msgs10.pose_pb2   as pose_pb2
    import gz.msgs10.twist_pb2  as twist_pb2
except Exception:
    import gz.msgs9.pose_v_pb2 as pose_v_pb2
    import gz.msgs9.pose_pb2   as pose_pb2
    import gz.msgs9.twist_pb2  as twist_pb2

try:
    from gz.transport13 import Node
except Exception:
    from gz.transport import Node  # backup if 13 isn't present

# ----------------------------- Utils ------------------------------
def clamp(v, lo, hi): return max(lo, min(hi, v))

def yaw_from_quat(w, x, y, z):
    # Z-yaw from quaternion
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

@dataclass
class Pose:
    x: float; y: float; z: float; yaw: float

# -------------------------- Transport -----------------------------
class GzTransportOnly:
    """ Lowercase API: node.subscribe(topic, cb, MsgClass); node.advertise(...). """
    def __init__(self, model: str):
        self.model = model
        self.node = Node()
        # ensure lowercase API exists (your env shows True)
        if not (hasattr(self.node, "subscribe") and hasattr(self.node, "advertise")):
            raise RuntimeError("Lowercase Gazebo Python API not available")
        self.pose: Pose | None = None
        self._pub = None  # publisher handle (kept to avoid GC)
        self._subscribed = False

    def start_pose_subscription(self) -> bool:
        """Try world Pose_V topics first, then /model/<name>/pose."""
        Pose_V = pose_v_pb2.Pose_V
        PoseMsg = pose_pb2.Pose

        def on_pose_v(msg):
            m = None
            if isinstance(msg, (bytes, bytearray)):
                m = Pose_V(); m.ParseFromString(msg)
            else:
                m = msg
            for p in m.pose:
                name = getattr(p, "name", "")
                if name.endswith("/" + self.model) or name == self.model:
                    self.pose = Pose(
                        p.position.x, p.position.y, p.position.z,
                        yaw_from_quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
                    )
                    return

        for topic in WORLD_POSE_TOPICS:
            try:
                self.node.subscribe(topic, on_pose_v, Pose_V)
                print(f"[hydra] Subscribed to {topic}")
                self._subscribed = True
                return True
            except Exception as e:
                print(f"[hydra] !Could not subscribe {topic}: {e}", file=sys.stderr)

        # Fallback: per-model pose
        model_pose_topic = MODEL_POSE_TOPIC_TMPL.format(model=self.model)

        def on_pose(msg):
            p = None
            if isinstance(msg, (bytes, bytearray)):
                p = PoseMsg(); p.ParseFromString(msg)
            else:
                p = msg
            self.pose = Pose(
                p.position.x, p.position.y, p.position.z,
                yaw_from_quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
            )

        try:
            self.node.subscribe(model_pose_topic, on_pose, PoseMsg)
            print(f"[hydra] Subscribed to {model_pose_topic}")
            self._subscribed = True
            return True
        except Exception as e:
            print(f"[hydra] Could not subscribe {model_pose_topic}: {e}", file=sys.stderr)
            return False

    def publish_cmd_vel(self, vx: float, vy: float, vz: float, wz: float = 0.0) -> bool:
        """Advertise once, then publish Twist via publisher object."""
        topic = CMD_VEL_TOPIC_TMPL.format(model=self.model)
        try:
            if self._pub is None:
                self._pub = self.node.advertise(topic, twist_pb2.Twist)
            msg = twist_pb2.Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = float(vx), float(vy), float(vz)
            msg.angular.z = float(wz)
            self._pub.publish(msg)
            return True
        except Exception as e:
            print(f"[hydra] Publish error on {topic}: {e}", file=sys.stderr)
            return False

# --------------------------- Controller ---------------------------
def drive_to_target(get_pose, publish_twist, target_xyz,
                    *, goal_r=1.0, alt=None,
                    k_lin=0.8, vmax=5.0, vz_max=2.0,
                    horizon_s=120.0, dt=0.05):
    """
    Simple P controller toward (tx, ty, tz). If alt is set, override target z.
    """
    tx, ty, tz = target_xyz
    if alt is not None: tz = alt

    print(f"[hydra] Navigating to target: x={tx:.2f} y={ty:.2f} z={tz:.2f} (goal_r={goal_r} m)")
    t0 = time.time()
    last_pub = 0.0
    pose_wait_warned = False

    try:
        while True:
            now = time.time()
            if now - t0 > horizon_s:
                print("[hydra] Time horizon exceeded; stopping.")
                publish_twist(0,0,0,0); return False

            p = get_pose()
            if p is None:
                if not pose_wait_warned and (now - t0) > 2.0:
                    print("[hydra] Waiting for first pose…", file=sys.stderr)
                    pose_wait_warned = True
                time.sleep(0.02)
                continue

            dx, dy, dz = tx - p.x, ty - p.y, tz - p.z
            dist_xy = math.hypot(dx, dy)
            # Accept if inside cylinder around goal and not far off in z
            if dist_xy <= goal_r and abs(dz) <= max(0.5, goal_r*0.5):
                print("[hydra] Reached goal. Stopping.")
                publish_twist(0,0,0,0); return True

            vx = clamp(k_lin * dx, -vmax,  vmax)
            vy = clamp(k_lin * dy, -vmax,  vmax)
            vz = clamp(0.6  * dz, -vz_max, vz_max)

            if now - last_pub >= dt:   # ~20 Hz
                publish_twist(vx, vy, vz, 0.0)
                last_pub = now
            time.sleep(0.01)
    finally:
        try: publish_twist(0,0,0,0)
        except: pass

# ------------------------------ Main ------------------------------
def parse_target_from_stdin(txt: str):
    m = re.search(r"x=([-\d\.eE]+)\s+y=([-\d\.eE]+)\s+z=([-\d\.eE]+)", txt)
    if not m: return None
    return tuple(map(float, m.groups()))

def main():
    ap = argparse.ArgumentParser(description="Hydra controller (transport-only)")
    ap.add_argument("--model", default="drone1", help="Gazebo model name")
    ap.add_argument("--target", type=float, nargs=3, metavar=("X","Y","Z"),
                    help="Target coordinates (meters)")
    ap.add_argument("--target-json", type=str,
                    help="Path to JSON with keys x,y,z")
    ap.add_argument("--alt", type=float, default=None, help="Override target altitude (z)")
    ap.add_argument("--goal-r", type=float, default=1.5, help="Goal acceptance radius (m)")
    ap.add_argument("--vmax", type=float, default=5.0, help="Max planar speed (m/s)")
    ap.add_argument("--vz-max", type=float, default=2.0, help="Max vertical speed (m/s)")
    ap.add_argument("--k", type=float, default=0.8, help="P-gain for planar control")
    ap.add_argument("--horizon-s", type=float, default=180.0, help="Max run time (s)")
    args = ap.parse_args()

    # Resolve target
    if args.target is not None:
        tx, ty, tz = args.target
    elif args.target_json:
        with open(args.target_json, "r") as f:
            j = json.load(f)
        tx, ty, tz = float(j["x"]), float(j["y"]), float(j.get("z", 0.0))
    else:
        stdin_txt = sys.stdin.read()
        t = parse_target_from_stdin(stdin_txt) if stdin_txt else None
        if not t:
            print("Provide --target X Y Z or --target-json FILE (or pipe 'Target => x=.. y=.. z=..').", file=sys.stderr)
            sys.exit(2)
        tx, ty, tz = t

    # Transport (no CLI fallback)
    transport = GzTransportOnly(args.model)
    if not transport.start_pose_subscription():
        print("[hydra] ERROR: Could not subscribe to any pose topic.", file=sys.stderr)
        sys.exit(3)

    # Ctrl+C clean stop
    def _sigint(_a,_b):
        try: transport.publish_cmd_vel(0,0,0,0)
        except: pass
        sys.exit(0)
    signal.signal(signal.SIGINT, _sigint)

    ok = drive_to_target(
        lambda: transport.pose,
        lambda vx,vy,vz,wz: transport.publish_cmd_vel(vx,vy,vz,wz),
        (tx,ty,tz),
        goal_r=args.goal_r, alt=args.alt,
        k_lin=args.k, vmax=args.vmax, vz_max=args.vz_max,
        horizon_s=args.horizon_s, dt=0.05,
    )
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
