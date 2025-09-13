#!/usr/bin/env python3
import sys, re, math, subprocess, time

# ----------------- Config -----------------
TOPIC = "/world/airport/dynamic_pose/info"
TARGET_NAME = "drone1"

# Tolerances: tighter in XY, looser in Z for sim "breathing"
POS_EPS_XY = 0.003   # 3 mm
POS_EPS_Z  = 0.010   # 10 mm
ANG_EPS    = 0.02    # ~1.15 deg

# Require change to persist for K samples before printing (debounce)
PERSIST_K  = 2

# Optional smoothing (EMA) used only for change detection (not what we print)
USE_EMA          = True
EMA_ALPHA        = 0.2   # 0<alpha<=1 ; 0.2 = modest smoothing

# Rate limit prints (max Hz)
MAX_HZ           = 10.0  # set None to disable
# ------------------------------------------

# Allow CLI overrides: topic, target_name
if len(sys.argv) >= 2: TOPIC = sys.argv[1]
if len(sys.argv) >= 3: TARGET_NAME = sys.argv[2]

cmd = ["gz", "topic", "-e", "-t", TOPIC]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

stamp_sec = stamp_nsec = None
in_pose = in_position = in_orientation = False
cur_name = None
px = py = pz = None
qx = qy = qz = qw = None

num_re = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")
str_re = re.compile(r'"(.*)"')

# State for change gating
last_emitted = None             # (px,py,pz,qx,qy,qz,qw)
ema_p = None                    # (x,y,z) smoothed
persist_count = 0
last_emit_time_ns = 0

def now_ns():
    try:
        return time.time_ns()
    except AttributeError:
        return int(time.time() * 1e9)

def quat_angle(q1, q2):
    # angle between orientations (shortest arc), robust to sign
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    dot = 1.0 if dot > 1.0 else (0.0 if dot < 0.0 else dot)
    return 2.0 * math.acos(dot)

def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

def rate_limited():
    global last_emit_time_ns
    if MAX_HZ is None or MAX_HZ <= 0:
        return False
    interval_ns = int(1e9 / MAX_HZ)
    t = now_ns()
    if t - last_emit_time_ns >= interval_ns:
        last_emit_time_ns = t
        return False
    return True

print("sec,nsec,x,y,z,qx,qy,qz,qw,yaw_rad")

def maybe_emit():
    global last_emitted, ema_p, persist_count

    if cur_name != TARGET_NAME:
        return
    if None in (stamp_sec, stamp_nsec, px, py, pz, qx, qy, qz, qw):
        return

    # Smoothing for change decision (not for printed values)
    if USE_EMA:
        if ema_p is None:
            ema_p = (px, py, pz)
        else:
            ax = EMA_ALPHA
            ema_p = (ax*px + (1-ax)*ema_p[0],
                     ax*py + (1-ax)*ema_p[1],
                     ax*pz + (1-ax)*ema_p[2])
        test_px, test_py, test_pz = ema_p
    else:
        test_px, test_py, test_pz = px, py, pz

    # Compare vs last emitted
    if last_emitted is None:
        # First sample—emit immediately
        if not rate_limited():
            yaw = yaw_from_quat(qx, qy, qz, qw)
            print(f"{stamp_sec},{stamp_nsec},{px},{py},{pz},{qx},{qy},{qz},{qw},{yaw}")
            sys.stdout.flush()
        last_emitted = (px, py, pz, qx, qy, qz, qw)
        persist_count = 0
        return

    (ex, ey, ez, eqx, eqy, eqz, eqw) = last_emitted

    # Axis-wise position deadband (XY tighter, Z looser)
    dx = abs(test_px - ex)
    dy = abs(test_py - ey)
    dz = abs(test_pz - ez)

    pos_changed = (dx > POS_EPS_XY) or (dy > POS_EPS_XY) or (dz > POS_EPS_Z)

    # Orientation change (shortest-arc angle)
    ang = quat_angle((eqx, eqy, eqz, eqw), (qx, qy, qz, qw))
    ang_changed = ang > ANG_EPS

    changed = pos_changed or ang_changed

    # Debounce: require change to persist K samples
    if changed:
        persist_count += 1
    else:
        persist_count = 0

    if persist_count >= PERSIST_K:
        if not rate_limited():
            yaw = yaw_from_quat(qx, qy, qz, qw)
            print(f"{stamp_sec},{stamp_nsec},{px},{py},{pz},{qx},{qy},{qz},{qw},{yaw}")
            sys.stdout.flush()
            last_emitted = (px, py, pz, qx, qy, qz, qw)
        persist_count = 0

for raw in proc.stdout:
    line = raw.strip()

    # header stamp
    if line.startswith("sec:"):
        m = num_re.search(line);  stamp_sec = int(float(m.group(0))) if m else stamp_sec
        continue
    if line.startswith("nsec:"):
        m = num_re.search(line);  stamp_nsec = int(float(m.group(0))) if m else stamp_nsec
        continue

    # pose block structure
    if line == "pose {":
        in_pose = True
        cur_name = None
        px = py = pz = None
        qx = qy = qz = qw = None
        continue
    if line == "position {": in_position = True; continue
    if line == "orientation {": in_orientation = True; continue
    if line == "}":
        if in_orientation: in_orientation = False; continue
        if in_position:    in_position = False; continue
        if in_pose:
            maybe_emit()
            in_pose = False
            continue
        continue

    if not in_pose:
        continue

    # inside a pose
    if line.startswith("name:"):
        m = str_re.search(line);  cur_name = m.group(1) if m else cur_name
        continue

    if in_position:
        if line.startswith("x:"):
            m = num_re.search(line);  px = float(m.group(0)) if m else px
        elif line.startswith("y:"):
            m = num_re.search(line);  py = float(m.group(0)) if m else py
        elif line.startswith("z:"):
            m = num_re.search(line);  pz = float(m.group(0)) if m else pz
        continue

    if in_orientation:
        if line.startswith("x:"):
            m = num_re.search(line);  qx = float(m.group(0)) if m else qx
        elif line.startswith("y:"):
            m = num_re.search(line);  qy = float(m.group(0)) if m else qy
        elif line.startswith("z:"):
            m = num_re.search(line);  qz = float(m.group(0)) if m else qz
        elif line.startswith("w:"):
            m = num_re.search(line);  qw = float(m.group(0)) if m else qw
        continue
