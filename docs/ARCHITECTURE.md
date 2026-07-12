# Architecture

This document describes the system design, data flow, and component interactions within CANavigator.

---

## System Overview

CANavigator is a multi-layer system that bridges three domains:

1. **Simulation** -- Gazebo Harmonic provides 6-DOF physics, LiDAR sensor simulation, and 3D rendering
2. **Middleware** -- ROS 2 Jazzy handles inter-process communication via topics
3. **Application** -- Python modules implement navigation algorithms, event handling, energy modeling, and experiment orchestration

All three layers run concurrently. The application layer drives the experiment loop, spawning one Gazebo instance per attempt and teleporting the drone back to start between strategy evaluations.

---

## Component Diagram

```
┌──────────────────────────────────────────────────────────────────────┐
│                        APPLICATION LAYER                            │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │                 Experiment Controller (main.py)              │    │
│  │                                                             │    │
│  │  - Orchestrates good-run loop                               │    │
│  │  - Manages component lifecycle                              │    │
│  │  - Records CSV results                                      │    │
│  │  - Triggers post-experiment analysis                        │    │
│  └───────┬──────────┬──────────┬──────────┬──────────┬────────┘    │
│          │          │          │          │          │              │
│          ▼          ▼          ▼          ▼          ▼              │
│  ┌───────────┐ ┌─────────┐ ┌────────┐ ┌────────┐ ┌──────────┐    │
│  │ Arena     │ │ Event   │ │ CA     │ │ Viol.  │ │ Energy   │    │
│  │ Generator │ │ Emitter │ │ Nav.   │ │ Monitor│ │ Monitor  │    │
│  └───────────┘ └─────────┘ └────────┘ └────────┘ └──────────┘    │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │              Velocity Controller (GzTeleop)                 │    │
│  │              DronePhysics (FlyCart 30 model)                 │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │              Async JSON Logger                               │    │
│  └─────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        MIDDLEWARE LAYER                              │
│                                                                     │
│  ROS 2 MultiThreadedExecutor (3 threads)                           │
│                                                                     │
│  Topics:                                                            │
│    /model/drone1/cmd_vel         (Twist)       Controller → Gazebo  │
│    /model/drone1/front_lidar/scan (LaserScan)  Gazebo → Navigator  │
│    /model/drone1/pose/info       (PoseStamped) Gazebo → Monitors   │
│    /model/target_sphere/pose/info (PoseStamped) Gazebo → Navigator │
│    /ca_navigator/event                  (String/JSON)  Emitter → Navigator │
│                                                                     │
│  Bridges:                                                           │
│    ros_gz_bridge parameter_bridge (LiDAR + cmd_vel)                │
│    MultiWorldPoseRepublisher (gz topic → ROS 2 pose)               │
└──────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        SIMULATION LAYER                             │
│                                                                     │
│  Gazebo Harmonic                                                    │
│    - airport_world.sdf (base world)                                │
│    - x3-uav model (drone with front LiDAR)                        │
│    - Procedurally generated NFZ models (SDF + metadata JSON)       │
│    - Target sphere model                                            │
│    - Physics: 6-DOF, collision, gravity                            │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Execution Flow

### Startup Sequence

```
1. TeleopConfig()                   Load all configuration
2. setup_async_logger()             Initialize queue-based JSON logger
3. rclpy.init()                     Start ROS 2 runtime
4. MultiThreadedExecutor(3)         Create executor with 3 worker threads
5. add_pose_republisher()           Bridge Gazebo world poses → ROS 2
6. start_parameter_bridge()         Bridge LiDAR + cmd_vel topics
7. add_event_emitter()              Create deterministic event source
8. GzTeleop(topic, cfg)             Create velocity publisher
9. add_violation_monitor()          Create NFZ entry tracker
10. add_energy_monitor()            Create EPM energy estimator
11. spin_thread.start()             Begin ROS 2 callback processing
```

### Per-Run Sequence

```
For each attempt (until good_runs == simulation_runs):

  1. ArenaGenerator.run()           Generate NFZ layout + target position
  2. start_sim(cfg)                 Launch one Gazebo instance for this attempt
  3. For each strategy in [APE1, APE2, APE3, CA]:
     a. reset_sim() (if not first)  Teleport drone back to start pose
     b. emitter.reset()             Reset event sequence to same seed
     c. viol_node.mark_run_start()  Reset violation counter
     d. ener_node.mark_run_start()  Reset energy accumulator
     e. LidarTargetNavigatorCA() Create navigator for this strategy
     f. nav.go_to(timeout_s)        Execute navigation to target
     g. nav.shutdown()              Clean up navigator subscriptions
     h. Collect: reached, elapsed, energy, violations, events
  4. stop_sim()                     Kill Gazebo instance
  5. If all strategies reached target:
     - Write one CSV row per strategy
     - Increment good_runs
  6. Else:
     - Discard attempt, continue to next
```

### Teardown Sequence

```
1. spin_stop.set()                  Signal ROS 2 spin loop to exit
2. emitter.stop()                   Stop event generation thread
3. ctrl.stop()                      Stop velocity publisher
4. exec.remove_node(...)            Detach all ROS 2 nodes
5. exec.shutdown()                  Shut down executor
6. rclpy.shutdown()                 Stop ROS 2 runtime
7. rosgz_bridge.stop()              Kill parameter bridge process
8. log_handle.stop()                Flush and close logger
```

---

## Navigation Architecture

### CA Meta-Algorithm

The CA (Time-Resource Optimized Operations Planner) navigator is the core intelligence of CANavigator. It wraps three Adaptive Planning Engines (APEs) with a deadline-aware selector.

```
                    ┌──────────────────────┐
                    │    Event Queue       │
                    │  (from /ca_navigator/event) │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │   Deadline Calculator │
                    │                      │
                    │   t_left = deadline   │
                    │          - elapsed    │
                    └──────────┬───────────┘
                               │
                 ┌─────────────┼─────────────┐
                 │             │             │
            all three APEs run in parallel from event arrival
                 │             │             │
                 ▼             ▼             ▼
            ┌────────┐   ┌────────┐   ┌────────┐
            │  APE3  │   │  APE2  │   │  APE1  │
            │  Full  │   │  Med   │   │  Fast  │
            │~1257ms │   │ ~716ms │   │ ~29ms  │
            └────┬───┘   └────┬───┘   └────┬───┘
                 │             │             │
                 └─────────────┼─────────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │   Heading + Speed    │
                    │   Calculation        │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │   GzTeleop.set_cmd() │
                    │   → DronePhysics     │
                    │   → Gazebo           │
                    └──────────────────────┘
```

Selection is opportunistic, not predicted: take APE3 the moment it's ready (nothing better can arrive); otherwise keep waiting for a better answer until the deadline, then take whichever of APE2/APE1 is ready. Only a true `DEADLINE_MISS` (nothing ready at all when time runs out) is a violation. See `docs/ca_architecture_deviations.md` §1 for the full rationale and what this replaced.

### LiDAR Processing Pipeline

APE1/APE2 and the base avoidance loop use a single-layer `LaserScan`:

```
Raw LaserScan (360° or limited arc)
         │
         ▼
┌────────────────────────────┐
│  Front Window (±5°)        │ → min_front_dist → obstacle detection
├────────────────────────────┤
│  Left Side (30° ± 30°)    │ → left_clearance → corridor width
├────────────────────────────┤
│  Right Side (-30° ± 30°)  │ → right_clearance → corridor width
└────────────────────────────┘
         │
         ▼
┌────────────────────────────┐
│  Stale Data Check          │
│  soft: 0.6s → degrade      │
│  hard: 1.2s → brake        │
└────────────────────────────┘
         │
         ▼
┌────────────────────────────┐
│  Avoidance Decision        │
│  - Heading sweep (±60°)    │
│  - NFZ repulsion vectors   │
│  - TTC braking             │
│  - Breadcrumb tie-breaking │
└────────────────────────────┘
```

APE3/VFH additionally consumes a 5-layer `PointCloud2` (`_CloudSub`) for multi-layer obstacle consensus — `LaserScan` has no vertical dimension, and `ros_gz_bridge`'s `LaserScan` converter silently truncates multi-layer scans to one layer. See `docs/ca_architecture_deviations.md` ("Multi-layer sensing") for why.

### Safety Subsystems

| System | Trigger | Action |
|---|---|---|
| **TTC Soft Brake** | Time-to-collision < 2.2s | Reduce speed proportionally |
| **TTC Hard Brake** | Time-to-collision < 1.4s | Clamp to minimum speed |
| **Stale LiDAR** | No scan for > 0.6s | Degrade to cautious mode |
| **Hard Stale** | No scan for > 1.2s | Full brake |
| **NFZ Repulsion** | Distance to NFZ < threshold | Apply soft repulsion vector |
| **Edge Guard** | Near NFZ boundary | Scale speed by 0.6x |

---

## Physics Model

The `DronePhysics` class models realistic velocity-level dynamics:

```
Desired Velocity (vx, vy, vz, wz)
         │
         ▼
┌────────────────────────────┐
│  Command Latency Buffer    │  FIFO delay of cmd_latency_s
└────────────┬───────────────┘
             ▼
┌────────────────────────────┐
│  Actuator Dynamics         │  2nd-order per-axis
│  (omega_n, zeta)           │  [Cuniato-2022]
└────────────┬───────────────┘
             ▼
┌────────────────────────────┐
│  Jerk Limiting             │  Acceleration slew-rate cap
│  (max_jerk per axis)       │  [Cuniato-2022]
└────────────┬───────────────┘
             ▼
┌────────────────────────────┐
│  Tilt / Thrust Caps        │  Lateral accel limit,
│  (max_tilt_deg, asym z)    │  asymmetric up/down
└────────────┬───────────────┘
             ▼
┌────────────────────────────┐
│  Aerodynamic Drag          │  F = c_lin*v + c_quad*|v|*v
│  (linear + quadratic)      │  [Hattenberger-2023]
└────────────┬───────────────┘
             ▼
┌────────────────────────────┐
│  Wind Disturbance          │  Ornstein-Uhlenbeck process
│  (OU noise per axis)       │  [OU-1930][Obukhov-2021]
└────────────┬───────────────┘
             ▼
      Shaped Velocity Output
      (published to Gazebo)
```

Tuned to DJI FlyCart 30 limits: ~65 kg mass (two DB2000 batteries, MTOW ~95 kg with cargo); max tilt ~30° → lateral accel cap `a_xy_max = g*tan(30°) ≈ 5.66 m/s²`; max horizontal speed 15 m/s typical / 20 m/s absolute; max ascent 5 m/s / descent 3 m/s; max wind resistance ~12 m/s (`wind_level=1.0` approximates this) [DJI-FC30-SPECS][DJI-FC30-UM]. Drag coefficients (`k1`, `k2`) are conservative defaults, intended to be regressed from flight/sim logs per [Hattenberger-2023]. Wind uses OU (tau, sigma) approximating gust autocorrelation/strength [OU-1930]; Dryden/von Kármán spectra are a drop-in alternative for canonical turbulence [Dryden-Std]. 2nd-order tracking with jerk limits follows layered UAV safety/control practice [Cuniato-2022].

---

## Energy Model

CANavigator tracks two independent energy sources:

### CPU Energy (Computational)

```
P(t) = P_idle + (TDP - P_idle) * U_eff * (f / f_base)^alpha

Where:
  TDP        = 25 W (Jetson Orin NX 16GB MAXN TDP, DS-10662-001 Table 1)
  P_idle     = TDP * idle_frac (10%, ~2.5W — TB-10580-001)
  U_eff      = effective CPU utilization = latency_us / (wall_s * N_cores)
  N_cores    = 8 (8x ARM Cortex-A78AE, DS-10662-001 Table 2)
  f / f_base = frequency scaling ratio (f_base = 2.0 GHz, DS-10662-001)
  alpha      = 1.5 (CMOS dynamic-power exponent, Bircher & John, IEEE TC 2012)
```

`(f/f_base)^alpha` evaluates to 1.0 in the current model since gem5's cycle counts already reflect execution at `f_base`, so the term is omitted from the live computation (`orin_nx_cycle_model.py::latency_to_energy_j`).

### Motion Energy

```
E_motion = EPM * horizontal_distance

Where:
  EPM = 208.9 J/m (FlyCart 30 baseline energy per meter)
```

---

## Event System

### Event Generation

The `EventEmitter` generates events with deterministic timing:

```
dt ~ LogUniform(dt_min_s, dt_max_s)
deadline = clamp(alpha * dt, [deadline_min_s, deadline_max_s])
type ~ Categorical(enemy=0.33, obstacle=0.33, lane=0.34)
```

Each event is published as a JSON string on `/ca_navigator/event` containing:
- Event type (enemy / obstacle / lane)
- Timestamp
- Deadline
- Position (within arena bounds)

### Event Lifecycle

```
Generated → Published → Received by Navigator → APE Selected → Resolved/Violated
                                                      │
                                            ┌─────────┼─────────┐
                                            ▼         ▼         ▼
                                         RESOLVED  RESOLVED  DEADLINE_MISS
                                         (APE3)    (APE1)    (no APE ready)
```

---

## Concurrency Model

```
Thread 1: ROS 2 spin loop (5 ms timeout)
  - Processes all ROS 2 callbacks
  - Event emitter callbacks
  - Pose republisher callbacks
  - Violation monitor callbacks
  - Energy monitor callbacks

Thread 2-3: MultiThreadedExecutor workers
  - Navigator LiDAR callbacks
  - Navigator pose callbacks
  - Parallel APE execution

Main Thread: Experiment controller
  - Arena generation
  - Sim start/stop
  - CSV recording
  - Analysis
```

All ROS 2 subscriptions use `ReentrantCallbackGroup` to allow concurrent execution across the executor's 3 threads.

---

## Data Flow Summary

| Data | Source | Sink | Transport |
|---|---|---|---|
| Velocity commands | Navigator → GzTeleop | Gazebo | gz.transport / cmd_vel topic |
| LiDAR scans | Gazebo | Navigator | ros_gz_bridge → ROS 2 topic |
| Drone pose | Gazebo | Navigator, Monitors | PoseRepublisher → ROS 2 topic |
| Target pose | Gazebo | Navigator | PoseRepublisher → ROS 2 topic |
| Events | EventEmitter | Navigator | ROS 2 /ca_navigator/event topic |
| NFZ metadata | ArenaGenerator | ViolationMonitor | JSON file on disk |
| Results | Navigator | main.py | Return values |
| CSV output | main.py | Disk | csv.DictWriter |
| JSON logs | All components | Disk | AsyncLogger queue |
