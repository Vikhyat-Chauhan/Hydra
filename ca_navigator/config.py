# config.py
from dataclasses import dataclass
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

@dataclass
class TeleopConfig:
    # --- Run Options ---
    simulation_runs = 1
    simulation_timeout = 200
    simulation_world_style = "city" #"perlin|city"
    # --- Sim / world ---
    world_path: str = os.path.join(PROJECT_ROOT, "worlds", "airport_world.sdf")
    sim_cmd: tuple[str, ...] = ("gz", "sim", "-r")#, "-s")
    sim_env: dict | None = None
    sim_boot_secs: float = 8.0
    fixed_seed = False
    world_gen_seed_offset = 112
    target_distance = 150
    # --- Gazebo transport ---
    topic: str = "/model/drone1/cmd_vel"  # gz.msgs.Twist
    world_pose_topic: str = "/world/airport/pose/info"

    # --- Teleop dynamics ---
    rate_hz: float = 100

    # --- Logging ---
    log_path: str = "logs/run_logs.json"

    def __post_init__(self):
        if self.sim_env is None:
            models_path = os.path.join(PROJECT_ROOT, "models")
            existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
            resource_path = (
                f"{models_path}:{existing_resource_path}"
                if existing_resource_path
                else models_path
            )
            self.sim_env = {
                **os.environ,
                "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
                "GZ_SIM_RESOURCE_PATH": resource_path,
            }

    # =======================
    # RESULTS & ANALYZER CFG
    # =======================

    # Where main experiment writes the consolidated results CSV:
    results_csv_path: str = "logs/results/experiment_summary.csv"

    # Analyzer output directory (plots + summaries will go here):
    analyzer_out_dir: str = "logs/results"

    analyzer_strategies = ["APE1", "APE2", "APE3", "CA"]

    # =======================
    # Generated Simulation & Algo Selector
    # =======================

    # Fixed start pose for selector (no ROS needed for this)
    start_x: float = -95.0
    start_y: float =   0.0
    start_z: float =   1.0

    start_yaw:   float = 0.0  # radians
    # Target file path (from your earlier JSON)
    target_json_path: str = "models/generated/generated_target_meta.json"

    # =======================
    # Event Generator (B-style)
    # =======================
    # Topics
    event_topic = "/ca_navigator/event"
    ros_pose_topic = "/model/drone1/pose/info"

    # -----------------------
    # Mixture of event types
    # -----------------------
    # Defines probabilities for each kind.
    event_mix_enemy = 0.33
    event_mix_obstacle = 0.33
    event_mix_lane = 0.34

    # -----------------------
    # Logging
    # -----------------------
    event_log_csv_path = "logs/events_log.csv"

    # -----------------------
    # Deadline model — AUTHORITATIVE SOURCE
    # Propagated into EventCfg via EventCfg.from_teleop_cfg().
    # Must stay consistent with EventDecisionCfg thresholds in nav_algorithm_T.py.
    #
    # RETUNED for MCU-scale budgets (APE1≈15.9ms / APE2≈16.8ms / APE3≈58.6ms,
    # ca_navigator/tools/orin_nx_cycle_model.py APE_LATENCY_US). The old
    # 600-700ms floor was tuned against pre-gem5-MCU budgets three orders
    # of magnitude smaller than today's; with it, CA always had time to
    # wait for APE3, so the speed/accuracy tradeoff never engaged (see
    # docs/ca_architecture_deviations.md — confirmed empirically: 11/11 CA
    # resolutions picked APE3 in a real run under the old floor).
    #
    # Physical grounding — sudden-obstacle reaction window (the tightest
    # real scenario the sim models), not the general LiDAR-safe-boundary
    # case used before:
    #   deadline_min = (sudden_obj_radius_m + vehicle_radius_m +
    #                    sudden_obj_clearance_m) / v_max
    #                = (1.2 + 0.7 + 0.3) / 15.0 = 2.2m / 15m/s ≈ 0.147s
    #   (sudden_obj_radius_m/vehicle_radius_m/sudden_obj_clearance_m are
    #   EventDecisionCfg/RiskCfg defaults in nav_algorithm_T.py; v_max is
    #   GoToConfig.max_v.)
    #   deadline_max = 3500ms → 52m (far-field threat, unchanged — still a
    #   reasonable long-horizon replanning window, not tied to APE budgets)
    # -----------------------
    deadline_alpha: float = 0.85
    deadline_min_s: float = 0.147
    deadline_max_s: float = 3.50
    # -----------------------
    # Physics
    # -----------------------
    cmd_latency_s: float = 0.10
    wind_level_0to1: float = 0.5
    wind_accel_std_base_mps2: float = 0.8
    physics_seed: int = 42  # seed for wind RNG; same seed → identical wind gusts across runs
