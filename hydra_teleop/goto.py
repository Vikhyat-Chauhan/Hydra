import time, threading, signal
from .transport import GzVelPub
from .config import TeleopConfig

class GzTeleop:
    def __init__(self, topic: str, cfg: TeleopConfig):
        self.topic = topic
        self.cfg = cfg
        self._vx = self._vy = self._vz = self._wz = 0.0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._pub = GzVelPub(self.topic)
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

    def _publish_loop(self):
        period = 1.0 / self.cfg.rate_hz
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
        time.sleep(0.05)
