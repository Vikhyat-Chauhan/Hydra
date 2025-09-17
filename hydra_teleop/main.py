#!/usr/bin/env python3
import time, rclpy
from .config import TeleopConfig
from .sim import start_sim, stop_sim
from .teleop import GzTeleop
from .pose_republisher import start_pose_bridge
from .basic_navigator import TargetNavigator, GoToConfig

def main():
    cfg = TeleopConfig()
    sim = start_sim(cfg)

    # start the world→ROS pose bridge in background
    bridge = start_pose_bridge(["drone1", "target_sphere"])

    # Related to Drone stuff

    ctrl = GzTeleop(cfg.topic, cfg)
    ctrl.start()

    nav = TargetNavigator(ctrl, cfg, goto_cfg=GoToConfig(
        goal_radius_m=1.0,
        kp_lin=0.8, kp_z=0.8, kp_yaw=1.5,
        max_v=min(5.0, cfg.speed_x),
        max_vz=min(5.0, cfg.speed_z),
        max_wz=min(0.8, cfg.yaw_rate),
        slow_yaw_threshold=0.8,
        rate_hz=20.0,
    ))

    try:
        # Follow /model/target_sphere/pose/info dynamically:
        # reached = nav.go_to(target_xyz=(0.0, 20.0, 5.0), timeout_s=60.0)
        reached = nav.go_to(timeout_s=60.0)
        print(f"[hydra] GoTo result: {'reached' if reached else 'timeout'}")
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop()
        ctrl.shutdown()
        nav.shutdown()
        # stop the bridge and the sim
        bridge.stop()
        stop_sim(sim)

if __name__ == "__main__":
    main()
