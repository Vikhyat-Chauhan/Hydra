#!/usr/bin/env python3
import os
import sys
import time
import signal
import threading
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ===================== User Config =====================
WORLD_ARG = 'gz_args:=/home/vikhyat-chauhan/Documents/Hydra/worlds/airport_world_V2.sdf --render-engine ogre2'
SIM_CMD = ["ros2", "launch", "ros_gz_sim", "gz_sim.launch.py", WORLD_ARG]
SIM_ENV = {
    **os.environ,
    "QT_QPA_PLATFORM": "xcb",
    "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
    "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
}

BRIDGE_CMD = [
    "ros2", "run", "ros_gz_bridge", "parameter_bridge",
    "/drone1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"
]

TOPIC = "/drone1/cmd_vel"   # change if your sim uses a different topic

# Teleop dynamics
RATE_HZ   = 20.0            # publishing rate
SPEED_X   = 5.0             # m/s forward/back
SPEED_Y   = 5.0             # m/s left/right (strafe)
SPEED_Z   = 5.0             # m/s up/down
YAW_RATE  = 0.6             # rad/s yaw

LAUNCH_SIM_AND_BRIDGE = True
SIM_BOOT_SECS = 10
BRIDGE_BOOT_SECS = 3
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

class TeleopNode(Node):
    def __init__(self):
        super().__init__('hydra_cli_teleop')
        self.pub = self.create_publisher(Twist, TOPIC, 10)

        # desired velocities (shared with input thread)
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wz = 0.0
        self._lock = threading.Lock()

        # publish timer
        self.create_timer(1.0 / RATE_HZ, self._tick)

        self.get_logger().info(
            "Ready. Type a command and press Enter:\n"
            "  forward, back, left, right, up, down, rotate_left, rotate_right\n"
            "  stop (or blank line)\n"
            "  quit"
        )

    def _tick(self):
        msg = Twist()
        with self._lock:
            msg.linear.x  = self._vx
            msg.linear.y  = self._vy
            msg.linear.z  = self._vz
            msg.angular.z = self._wz
        self.pub.publish(msg)

    def set_cmd(self, vx, vy, vz, wz):
        with self._lock:
            self._vx, self._vy, self._vz, self._wz = vx, vy, vz, wz

    def stop(self):
        self.set_cmd(0.0, 0.0, 0.0, 0.0)

def input_thread(node: TeleopNode, stop_event: threading.Event):
    """Reads commands from stdin and updates the node's velocities."""
    while not stop_event.is_set():
        try:
            line = input(">>> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            line = "quit"

        if line == "":
            node.stop()
            print("[stopped]")
            continue

        if line == "w":
            node.set_cmd(+SPEED_X, 0.0, 0.0, 0.0)
        elif line == "s":
            node.set_cmd(-SPEED_X, 0.0, 0.0, 0.0)
        elif line == "a":
            node.set_cmd(0.0, +SPEED_Y, 0.0, 0.0)
        elif line == "d":
            node.set_cmd(0.0, -SPEED_Y, 0.0, 0.0)
        elif line == "r":
            node.set_cmd(0.0, 0.0, +SPEED_Z, 0.0)
        elif line == "f":
            node.set_cmd(0.0, 0.0, -SPEED_Z, 0.0)
        elif line == "v":
            node.set_cmd(0.0, 0.0, 0.0, +YAW_RATE)
        elif line == "c":
            node.set_cmd(0.0, 0.0, 0.0, -YAW_RATE)
        elif line == "stop":
            node.stop()
            print("[stopped]")
        elif line == "q":
            # Final stop for safety, then signal main loop to exit
            node.stop()
            stop_event.set()
        else:
            print("Unknown command")

def main():
    sim = bridge = None
    node = None

    stop_event = threading.Event()

    def cleanup():
        # Stop the sim & bridge cleanly (whole process groups)
        kill_process_tree(bridge, "ros_gz_bridge")
        kill_process_tree(sim, "ros_gz_sim")

    # Make Ctrl+C graceful: stop publishing and exit
    def sigint_handler(*_):
        try:
            if node is not None:
                node.stop()
        except Exception:
            pass
        stop_event.set()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        if LAUNCH_SIM_AND_BRIDGE:
            print("Launching Gazebo…")
            sim = launch(SIM_CMD, env=SIM_ENV)
            time.sleep(SIM_BOOT_SECS)

            print("Launching ROS–Gazebo bridge…")
            bridge = launch(BRIDGE_CMD)
            time.sleep(BRIDGE_BOOT_SECS)

        # Init ROS 2
        rclpy.init()
        node = TeleopNode()

        # Start input thread
        t = threading.Thread(target=input_thread, args=(node, stop_event), daemon=True)
        t.start()

        # Spin while input thread runs
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        # Final stop before shutting down ROS
        try:
            if node is not None:
                node.stop()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        cleanup()

if __name__ == "__main__":
    # Ensure you sourced ROS before running this (e.g., /opt/ros/<distro>/setup.bash)
    main()
