#!/usr/bin/env python3
import os
import sys
import time
import signal
import threading
import subprocess
import shlex
from dataclasses import dataclass

# ===================== User Config =====================
WORLD_PATH = "/home/vikhyat-chauhan/Documents/Hydra/worlds/airport_world.sdf"
SIM_CMD = ["gz", "sim", "-r", WORLD_PATH, "--render-engine", "ogre2"]
SIM_ENV = {
    **os.environ,
    "QT_QPA_PLATFORM": "xcb",
    "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
    "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
}

TOPIC = "/model/drone1/cmd_vel"   # Gazebo topic for gz.msgs.Twist

# Teleop dynamics
RATE_HZ   = 30.0            # publishing rate (higher = smoother)
SPEED_X   = 5.0             # m/s forward/back
SPEED_Y   = 5.0             # m/s left/right (strafe)
SPEED_Z   = 5.0             # m/s up/down
YAW_RATE  = 0.8             # rad/s yaw

LAUNCH_SIM = True
SIM_BOOT_SECS = 8
# =======================================================


# ---- Imports for Gazebo Transport & Msgs (expect these to be installed) ----
# sudo apt install python3-gz-transport13 python3-gz-msgs10
try:
    from gz.transport13 import Node, AdvertiseMessageOptions  # type: ignore
except Exception as e:
    print("ERROR: Could not import gz.transport13. Install 'python3-gz-transport13'.", file=sys.stderr)
    raise

# Prefer msgs10; fall back to older ignition/gz if present.
_twist_import_errors = []
Twist = None
Vector3d = None
for base in ("gz.msgs10", "gz.msgs", "ignition.msgs"):
    try:
        Twist = __import__(f"{base}.twist_pb2", fromlist=["Twist"]).Twist
        Vector3d = __import__(f"{base}.vector3d_pb2", fromlist=["Vector3d"]).Vector3d
        print(f"[hydra] Using msgs from: {base}")
        break
    except Exception as e:
        _twist_import_errors.append((base, str(e)))
        continue

if Twist is None or Vector3d is None:
    print("ERROR: Could not import gz/ignition msgs (Twist, Vector3d).", file=sys.stderr)
    print("Tried:", _twist_import_errors, file=sys.stderr)
    print("Install 'python3-gz-msgs10' (recommended).", file=sys.stderr)
    sys.exit(2)


def launch(cmd, env=None):
    # start_new_session=True creates a new process group -> we can kill the whole tree later
    return subprocess.Popen(cmd, env=env, start_new_session=True)


def kill_process_tree(proc: subprocess.Popen | None, name: str, gentle_timeout=5, term_timeout=5):
    """Try SIGINT -> SIGTERM -> SIGKILL on the process group."""
    if not proc or proc.poll() is not None:
        return
    pgid = proc.pid
    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=gentle_timeout)
    except Exception:
        pass
    if proc.poll() is None:
        try:
            os.killpg(pgid, signal.SIGTERM)
            proc.wait(timeout=term_timeout)
        except Exception:
            pass
    if proc.poll() is None:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass


class GzVelPub:
    """
    Minimal, fast velocity publisher:
    - Keeps a persistent Node & Publisher
    - Reuses a pre-allocated Twist protobuf for low GC pressure
    """
    def __init__(self, topic: str, rate_limit_hz: float | None = None):
        self.topic = topic
        self.node = Node()

        # Try the modern advertise() API first; fall back to Node.Advertise() if bound that way.
        self._pub = None
        self._publish_fn = None

        # Modern API: node.advertise(topic, MsgType, opts?) -> Publisher with .publish(msg)
        try:
            if rate_limit_hz:
                opts = AdvertiseMessageOptions()
                # msgs_per_sec is a soft limit; the loop still runs at RATE_HZ
                opts.msgs_per_sec = float(rate_limit_hz)
                self._pub = self.node.advertise(self.topic, Twist, opts)  # type: ignore[attr-defined]
            else:
                self._pub = self.node.advertise(self.topic, Twist)  # type: ignore[attr-defined]
            if not self._pub:
                raise RuntimeError("node.advertise returned None")
            self._publish_fn = lambda msg: self._pub.publish(msg)  # type: ignore[union-attr]
            print("[hydra] Transport API: node.advertise(...).publish(msg)")
        except Exception:
            # Legacy-style API (older py bindings): Node.Advertise(topic, MsgType) + Node.Publish(topic, msg)
            ok = False
            try:
                ok = self.node.Advertise(self.topic, Twist)  # type: ignore[attr-defined]
            except Exception:
                pass
            if not ok:
                raise RuntimeError(f"Failed to advertise publisher on '{self.topic}' via any API.")
            self._publish_fn = lambda msg: self.node.Publish(self.topic, msg)  # type: ignore[attr-defined]
            print("[hydra] Transport API: Node.Advertise(...), Node.Publish(topic, msg)")

        # Pre-allocate a Twist message we’ll mutate in-place
        self._msg = Twist()

    def send(self, lin_xyz: tuple[float, float, float], ang_zyx: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        # Mutate pre-allocated message (cheap; avoids new allocations)
        lx, ly, lz = lin_xyz
        ax, ay, az = ang_zyx
        self._msg.linear.CopyFrom(Vector3d(x=lx, y=ly, z=lz))
        self._msg.angular.CopyFrom(Vector3d(x=ax, y=ay, z=az))
        ok = self._publish_fn(self._msg)
        if ok is False:
            # Some bindings return bool; others None. Only error if explicit False.
            raise RuntimeError("Publish failed")


class GzTeleop:
    def __init__(self, topic: str):
        self.topic = topic
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wz = 0.0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._pub = GzVelPub(self.topic)

        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

        print(
            "Ready. Keys (then Enter):\n"
            "  w/s = forward/back, a/d = left/right, r/f = up/down\n"
            "  v/c = yaw left/right, stop = zero all, q = quit\n"
        )

    def _publish_loop(self):
        period = 1.0 / RATE_HZ
        while not self._stop_event.is_set():
            with self._lock:
                vx, vy, vz, wz = self._vx, self._vy, self._vz, self._wz
            self._pub.send((vx, vy, vz), (0.0, 0.0, wz))
            time.sleep(period)

    def set_cmd(self, vx, vy, vz, wz):
        with self._lock:
            self._vx, self._vy, self._vz, self._wz = vx, vy, vz, wz

    def stop(self):
        self.set_cmd(0.0, 0.0, 0.0, 0.0)

    def shutdown(self):
        self.stop()
        self._stop_event.set()
        # grace period to let publisher thread exit
        time.sleep(0.05)


def input_thread(ctrl: GzTeleop, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            line = input(">>> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            line = "q"

        if line == "":
            ctrl.stop(); print("[stopped]"); continue
        if line == "w":
            ctrl.set_cmd(+SPEED_X, 0.0, 0.0, 0.0)
        elif line == "s":
            ctrl.set_cmd(-SPEED_X, 0.0, 0.0, 0.0)
        elif line == "a":
            ctrl.set_cmd(0.0, +SPEED_Y, 0.0, 0.0)
        elif line == "d":
            ctrl.set_cmd(0.0, -SPEED_Y, 0.0, 0.0)
        elif line == "r":
            ctrl.set_cmd(0.0, 0.0, +SPEED_Z, 0.0)
        elif line == "f":
            ctrl.set_cmd(0.0, 0.0, -SPEED_Z, 0.0)
        elif line == "c":
            ctrl.set_cmd(0.0, 0.0, 0.0, +YAW_RATE)
        elif line == "v":
            ctrl.set_cmd(0.0, 0.0, 0.0, -YAW_RATE)
        elif line == "stop":
            ctrl.stop(); print("[stopped]")
        elif line in ("q", "quit", "exit"):
            ctrl.stop(); stop_event.set()
        else:
            print("Unknown command")


def main():
    sim = None
    stop_event = threading.Event()
    ctrl = None

    def sigint_handler(*_):
        try:
            if ctrl is not None:
                ctrl.shutdown()
        except Exception:
            pass
        stop_event.set()
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        if LAUNCH_SIM:
            print("Launching Gazebo (gz sim)…")
            sim = launch(SIM_CMD, env=SIM_ENV)
            time.sleep(SIM_BOOT_SECS)

        # Create controller and input thread
        ctrl = GzTeleop(TOPIC)
        t = threading.Thread(target=input_thread, args=(ctrl, stop_event), daemon=True)
        t.start()

        # Wait until user quits
        while not stop_event.is_set():
            time.sleep(0.1)

    finally:
        try:
            if ctrl is not None:
                ctrl.shutdown()
        except Exception:
            pass
        kill_process_tree(sim, "gz_sim")


if __name__ == "__main__":
    main()
