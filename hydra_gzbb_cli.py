#!/usr/bin/env python3
import os
import time
import signal
import threading
import subprocess
import shlex
import importlib
import sys

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
RATE_HZ   = 20.0            # publishing rate
SPEED_X   = 5.0             # m/s forward/back
SPEED_Y   = 5.0             # m/s left/right (strafe)
SPEED_Z   = 5.0             # m/s up/down
YAW_RATE  = 0.6             # rad/s yaw

LAUNCH_SIM = True
SIM_BOOT_SECS = 10
# =======================================================


def launch(cmd, env=None):
    # start_new_session=True creates a new process group -> we can kill the whole tree later
    return subprocess.Popen(cmd, env=env, start_new_session=True)


def kill_process_tree(proc: subprocess.Popen, name: str, gentle_timeout=5, term_timeout=5):
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


# ---------- Gazebo Transport + msgs (robust autodetect) ----------
Node = None
for mod in ("gz.transport13", "gz.transport12", "gz.transport11", "gz.transport10", "ignition.transport"):
    try:
        Node = importlib.import_module(mod).Node
        print(f"[hydra] Using transport: {mod}")
        break
    except Exception:
        pass

if Node is None:
    print("ERROR: No Gazebo transport module found (gz.transportXX / ignition.transport).")
    sys.exit(2)

Twist = Vector3d = None
for base in ("gz.msgs", "ignition.msgs"):
    try:
        Twist = importlib.import_module(f"{base}.twist_pb2").Twist
        Vector3d = importlib.import_module(f"{base}.vector3d_pb2").Vector3d
        print(f"[hydra] Using msgs: {base}")
        break
    except Exception:
        pass

USE_SUBPROC_PUB = (Twist is None)
if USE_SUBPROC_PUB:
    print("[hydra] Python gz.msgs not found -> will publish via 'gz topic -p' fallback.")


class GzTeleop:
    def __init__(self, topic: str):
        self.topic = topic
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wz = 0.0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        if not USE_SUBPROC_PUB:
            self._node = Node()
            ok = self._node.Advertise(self.topic, Twist)
            if not ok:
                print(f"ERROR: Failed to advertise publisher on '{self.topic}'.")
                sys.exit(2)

        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

        print(
            "Ready. Keys (then Enter):\n"
            "  w/s = forward/back, a/d = left/right, r/f = up/down\n"
            "  v/c = yaw left/right, stop = zero all, q = quit\n"
        )

    def _publish_once(self, vx, vy, vz, wz):
        if USE_SUBPROC_PUB:
            # Publish using the CLI so we don't need Python protobufs
            payload = f"linear: {{x: {vx}, y: {vy}, z: {vz}}} angular: {{z: {wz}}}"
            cmd = f"gz topic -t {shlex.quote(self.topic)} -m gz.msgs.Twist -p \"{payload}\""
            subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            msg = Twist()
            msg.linear.CopyFrom(Vector3d(x=vx, y=vy, z=vz))
            msg.angular.CopyFrom(Vector3d(x=0.0, y=0.0, z=wz))
            self._node.Publish(self.topic, msg)

    def _publish_loop(self):
        period = 1.0 / RATE_HZ
        while not self._stop_event.is_set():
            with self._lock:
                vx, vy, vz, wz = self._vx, self._vy, self._vz, self._wz
            self._publish_once(vx, vy, vz, wz)
            time.sleep(period)

    def set_cmd(self, vx, vy, vz, wz):
        with self._lock:
            self._vx, self._vy, self._vz, self._wz = vx, vy, vz, wz

    def stop(self):
        self.set_cmd(0.0, 0.0, 0.0, 0.0)

    def shutdown(self):
        self.stop()
        self._stop_event.set()
        time.sleep(0.1)  # allow thread to exit


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
        elif line == "v":
            ctrl.set_cmd(0.0, 0.0, 0.0, +YAW_RATE)
        elif line == "c":
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

