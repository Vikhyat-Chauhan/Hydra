#!/usr/bin/env python3
import time
import threading
from typing import Tuple
from .transport import GzVelPub
from .config import TeleopConfig
from .logger import Logger


class GzTeleop:
    """
    velocity teleop:
      - set_cmd(): update state that the background loop will keep publishing
      - start(): begin periodic publishing at cfg.rate_hz
      - publish_once(): send a single Twist (does NOT affect background state)
      - stop()/shutdown(): zero cmd and cleanly stop the loop
    """

    def __init__(self, topic: str, cfg: TeleopConfig):
        if cfg.rate_hz <= 0:
            raise ValueError(f"rate_hz must be > 0, got {cfg.rate_hz}")
        self.topic = topic
        self.cfg = cfg

        self._logger = Logger(cfg)
        # State published by the loop (hold)
        self._vx = self._vy = self._vz = self._wz = 0.0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        self._pub = GzVelPub(self.topic)

    # ---------- Internal loop ----------
    def _publish_loop(self) -> None:
        period = 1.0 / self.cfg.rate_hz
        next_t = time.time()
        while not self._stop_event.is_set():
            with self._lock:
                vx, vy, vz, wz = self._vx, self._vy, self._vz, self._wz
            # Only yaw on angular for the hold loop; edit if you want wx/wy too.
            self._pub.send((vx, vy, vz), (0.0, 0.0, wz))

            next_t += period
            sleep_dt = max(0.0, next_t - time.time())
            time.sleep(sleep_dt)

    # ---------- Public API ----------
    def start(self) -> None:
        """Start periodic publishing (idempotent)."""
        self._logger.log(source="GzTeleop", event="start", message="Teleop started")
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def set_cmd(self, vx: float, vy: float, vz: float, wz: float) -> None:
        """Update the held velocity that the loop will keep publishing."""
        with self._lock:
            self._vx, self._vy, self._vz, self._wz = vx, vy, vz, wz
            self._logger.log(source="GzTeleop", event="set_cmd", message="", vx = self._vx, vy = self._vy, vz = self._vz, wx = 0, wy = 0, wz = self._wz)

    def publish_once(
        self,
        linear: Tuple[float, float, float],
        angular: Tuple[float, float, float],
    ) -> None:
        """
        Send a one-shot velocity message immediately.
        Does NOT change the held state used by the background loop.
        """
        self._logger.log(source="GzTeleop", event="publish_once", message="", vx = linear[0], vy = linear[1], vz = linear[2], wx = angular[0], wy = angular[1], wz = angular[2])
        self._pub.send(linear, angular)

    def stop(self) -> None:
        """Zero the held command (does not stop the thread)."""
        self._logger.log(source="GzTeleop", event="stop", message="")
        self.set_cmd(0.0, 0.0, 0.0, 0.0)

    def shutdown(self, join_timeout: float = 0.3) -> None:
        # 1) set state to zero
        with self._lock:
            self._vx = self._vy = self._vz = self._wz = 0.0

        # 2) force-send a zero immediately (in case the loop gets stopped early)
        try:
            self._pub.send((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        except Exception:
            pass

        # 3) give the loop a tick to also publish zero from its path
        time.sleep(max(0.02, 2.0 / self.cfg.rate_hz))

        # 4) stop the loop
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=join_timeout)
        self._thread = None


"""
def input_thread(ctrl: GzTeleop, cfg: TeleopConfig, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            line = input(">>> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            line = "q"

        if line == "":
            ctrl.stop(); print("[stopped]"); continue
        if line == "w":
            ctrl.set_cmd(+cfg.speed_x, 0.0, 0.0, 0.0)
        elif line == "s":
            ctrl.set_cmd(-cfg.speed_x, 0.0, 0.0, 0.0)
        elif line == "a":
            ctrl.set_cmd(0.0, +cfg.speed_y, 0.0, 0.0)
        elif line == "d":
            ctrl.set_cmd(0.0, -cfg.speed_y, 0.0, 0.0)
        elif line == "r":
            ctrl.set_cmd(0.0, 0.0, +cfg.speed_z, 0.0)
        elif line == "f":
            ctrl.set_cmd(0.0, 0.0, -cfg.speed_z, 0.0)
        elif line == "c":
            ctrl.set_cmd(0.0, 0.0, 0.0, +cfg.yaw_rate)
        elif line == "v":
            ctrl.set_cmd(0.0, 0.0, 0.0, -cfg.yaw_rate)
        elif line == "stop":
            ctrl.stop(); print("[stopped]")
        elif line in ("q", "quit", "exit"):
            ctrl.stop(); stop_event.set()
        else:
            print("Unknown command")

def run_teleop(cfg: TeleopConfig) -> None:
    
    #Standalone runner for keyboard teleop.
    #Handles optional sim launch and clean shutdown.
    
    from .sim import launch, kill_process_tree
    import threading, time

    sim = None
    stop_event = threading.Event()
    ctrl = None

    def _sigint_handler(*_):
        try:
            if ctrl is not None:
                ctrl.shutdown()
        except Exception:
            pass
        stop_event.set()
    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        if cfg.launch_sim:
            print("Launching Gazebo (gz sim)…")
            sim = launch([*cfg.sim_cmd, cfg.world_path, "--render-engine", "ogre2"], env=cfg.sim_env)
            time.sleep(cfg.sim_boot_secs)

        ctrl = GzTeleop(cfg.topic, cfg)
        t = threading.Thread(target=input_thread, args=(ctrl, cfg, stop_event), daemon=True)
        t.start()

        while not stop_event.is_set():
            time.sleep(0.1)

    finally:
        try:
            if ctrl is not None:
                ctrl.shutdown()
        except Exception:
            pass
        kill_process_tree(sim, "gz_sim")
"""