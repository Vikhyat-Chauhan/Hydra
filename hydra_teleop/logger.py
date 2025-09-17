#!/usr/bin/env python3
# logger.py
import os, csv, time, threading
from typing import Optional
from .config import TeleopConfig

class Logger:
    HEADERS = ["t_sec","source","event","message","vx","vy","vz","wx","wy","wz"]

    def __init__(self, cfg: TeleopConfig):
        self.t0 = time.time()
        self._lock = threading.Lock()

        # From config
        self._to_csv   = bool(getattr(cfg, "log_to_csv", False))
        self._log_dir  = str(getattr(cfg, "log_dir", "logs"))
        self._log_name = str(getattr(cfg, "log_name", "run"))

        os.makedirs(self._log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(self._log_dir, f"{self._log_name}_{ts}.csv")

        self._csv_file = None
        self._csv_writer = None
        self._csv_ready = False

    def log(self, source: str, event: str, message: str = "",
            vx: Optional[float]=None, vy: Optional[float]=None, vz: Optional[float]=None,
            wx: Optional[float]=None, wy: Optional[float]=None, wz: Optional[float]=None):
        with self._lock:
            row = {
                "t_sec": round(time.time() - self.t0, 6),
                "source": source,
                "event": event,
                "message": message,
                "vx": vx if vx is not None else "",
                "vy": vy if vy is not None else "",
                "vz": vz if vz is not None else "",
                "wx": wx if wx is not None else "",
                "wy": wy if wy is not None else "",
                "wz": wz if wz is not None else "",
            }

            # Console
            kv = " ".join(f"{k}={row[k]}" for k in self.HEADERS if row[k] != "")
            print(f"[{row['t_sec']:.3f}] {kv}", flush=True)

            # CSV
            if self._to_csv:
                self._ensure_csv_ready()
                self._csv_writer.writerow([row[h] for h in self.HEADERS])
                self._csv_file.flush()

    def close(self):
        with self._lock:
            if self._csv_file:
                try: self._csv_file.close()
                except Exception: pass
            self._csv_file = None
            self._csv_writer = None
            self._csv_ready = False

    def __enter__(self): return self
    def __exit__(self, *exc): self.close()

    def _ensure_csv_ready(self):
        if self._csv_ready: return
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.HEADERS)
        self._csv_ready = True

    @property
    def csv_path(self) -> Optional[str]:
        return self._csv_path if self._to_csv else None
