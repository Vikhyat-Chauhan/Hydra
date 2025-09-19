#!/usr/bin/env python3
# hydra_teleop/violations.py
# Violation checker: subscribe to PoseStamped, count entries into red boxes.

import json, os, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

META_PATH = os.path.join("models", "generated", "generated_nofly_meta.json")


def load_rects(meta_path):
    """Load list of (cx, cy, w, h) rectangles from JSON metadata."""
    with open(meta_path) as f:
        meta = json.load(f)
    return [tuple(map(float, r)) for r in meta.get("rectangles_xywh", [])]


def inside_rect(x, y, cx, cy, w, h):
    return (abs(x - cx) <= w / 2.0) and (abs(y - cy) <= h / 2.0)


class ViolationCounter(Node):
    def __init__(self, pose_topic="/model/drone1/pose/info", meta_path=META_PATH):
        super().__init__("violation_counter")
        self._rects = load_rects(meta_path)
        self._violations = 0
        self._inside_any = False

        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self.get_logger().info(f"[violations] Loaded {len(self._rects)} restricted boxes.")

    def _on_pose(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        in_zone = any(inside_rect(x, y, cx, cy, w, h) for (cx, cy, w, h) in self._rects)

        # detect new entry
        if in_zone and not self._inside_any:
            self._violations += 1
            self.get_logger().warn(
                f"Violation {self._violations}: entered restricted zone at ({x:.2f}, {y:.2f})"
            )

        self._inside_any = in_zone

    def get_total(self) -> int:
        """Return total violations so far (does not reset)."""
        return self._violations

    def reset(self):
        """Reset violation counter to zero."""
        self._violations = 0


def start_violation_monitor(pose_topic="/model/drone1/pose/info"):
    """
    Start the ViolationCounter in a background thread.
    Assumes rclpy.init() has already been called elsewhere.
    """
    node = ViolationCounter(pose_topic=pose_topic)

    # spin in background
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    return node
