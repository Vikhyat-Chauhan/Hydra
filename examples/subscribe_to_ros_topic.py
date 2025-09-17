#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DronePoseSubscriber(Node):
    def __init__(self, topic='/model/drone1/pose/info'):
        super().__init__('drone_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,                 # <-- PoseStamped
            topic,
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        # stamp is available because it's PoseStamped
        self.get_logger().info(
            f"[{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}] "
            f"frame='{msg.header.frame_id}' "
            f"pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) "
            f"ori=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DronePoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
