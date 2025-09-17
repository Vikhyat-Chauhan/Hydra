#Retired Script
"""
A bit of History:
I tried several methods to get drone positions when the simulation is running :
1. I tried the pose publisher plugin with ros bridge with the drone [/model/drone1/pose]
    ->  When I looked at the entries the drone1 kept publishing the the information about the links in the drone1 model not the drone itself. Which was
        impossible to resolve nomatter what config I used in the pose publisher plugin.
2. I tried to use pose publisher for the world, but it was not compatible with the world and was throwing error when loading the world sdf file.
3. I tried to create a ros bridge from the world pose publisher [/world/airport/pose/info] to ros topic
    ->  The problem I ran across was that a bridge from gz pose vector to ros removes all essential information 
        about the entity (like drone1) and give some output thats gibberish to parse for positions. This is apparantly because ros converts the gz.msgs.Pose to something native to ros.
[Current hotfix]
4. Finally I had to create a script that uses the cli command of gz to get the pose infromation about all the entitites in the world and parse them with a python code to then create
    a ros bridge from which I can later fetch this information, I am guessing it might not be very efficient but this is the only option i have now.
"""

#!/usr/bin/env python3
import json, subprocess, threading, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

TOPIC = "/world/airport/pose/info"
ROSTOPIC = "/model/drone1/pose/info"
GZ_CMD = ["gz", "topic", "-e", "-t", TOPIC, "--json-output"]

def fnum(x, default=0.0):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default

class WorldPoseRepublisher(Node):
    def __init__(self):
        super().__init__("drone1_world_pose_republisher")
        self.pub = self.create_publisher(PoseStamped, "/model/drone1/pose/info", 10)
        self._stop = False
        self._proc = None
        threading.Thread(target=self._reader, daemon=True).start()
        self.get_logger().info(f"Republishing {TOPIC} → /model/drone1/pose/info")

    def _reader(self):
        try:
            self._proc = subprocess.Popen(GZ_CMD, stdout=subprocess.PIPE, text=True, bufsize=1)
        except FileNotFoundError:
            self.get_logger().error("`gz` not found in PATH.")
            return

        for line in self._proc.stdout:
            if self._stop:
                break
            line = line.strip()
            if not line or not line.startswith("{"):
                continue

            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue

            # Pose_V message shape: {"header": {...}, "pose": [ {name, id, position, orientation}, ... ] }
            poses = msg.get("pose", [])
            if not poses:
                continue

            for p in poses:
                if p.get("name") != "drone1":
                    continue

                pos = p.get("position", {}) or {}
                ori = p.get("orientation", {}) or {}
                out = PoseStamped()
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = "world"
                out.pose.position.x = fnum(pos.get("x"))
                out.pose.position.y = fnum(pos.get("y"))
                out.pose.position.z = fnum(pos.get("z"))
                out.pose.orientation.w = fnum(ori.get("w"), 1.0)
                out.pose.orientation.x = fnum(ori.get("x"))
                out.pose.orientation.y = fnum(ori.get("y"))
                out.pose.orientation.z = fnum(ori.get("z"))

                self.pub.publish(out)
                # If you only want root model pose, break after publishing once per line
                break

    def destroy_node(self):
        self._stop = True
        try:
            if self._proc:
                self._proc.terminate()
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = WorldPoseRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
