# gem5 Power Study: Cycle-Accurate Validation of APE Compute Cost

gem5 (cycle-accurate microarchitectural simulation) measures the real,
native APE planners directly, so `budget_ms`/energy figures used by
`nav_algorithm_T.py` are grounded in an actual instruction stream on a
modeled Cortex-A78-class core — not a hand-estimated latency table. The
APE algorithms are real, literature-standard implementations, written
directly in C, and gem5 measures them.

## 1. What each APE actually is

- **APE1** (`ca_navigator/native/ape_ops/src/ape1_bug.c`) — a minimal
  reactive potential-field/Bug-style heading controller. Closed-form, one
  forward sector + two side sectors, no candidate search — deliberately
  the cheapest tier, matching the "fast but least accurate" role.
- **APE2** (`ape2_dwa.c`) — real Dynamic Window Approach (Fox, Burgard &
  Thrun, *The Dynamic Window Approach to Collision Avoidance*, IEEE
  Robotics & Automation Magazine, 1997): a fixed `dwa_n_v × dwa_n_w` grid
  of (v, ω) candidates, each forward-simulated over a kinematic unicycle
  model and scored by clearance + goal-heading alignment + speed.
- **APE3** (`ape3_vfh.c`) — real Vector Field Histogram (Borenstein &
  Koren, *The Vector Field Histogram*, IEEE Trans. Robotics & Automation,
  1991): a polar obstacle-density histogram built from **all** vertical
  LiDAR layers (multi-layer consensus — a sector is blocked if any layer
  sees a close obstacle there), followed by a single-pass valley search
  that selects and scores the best candidate heading.

Both DWA and VFH always walk their full fixed-size grid/histogram
unconditionally — no early exit on scan content — which keeps their op
count independent of live sensor data, so a one-time offline gem5
measurement is valid for every future invocation (see each file's own
"op-count discipline" docstring note).

`ca_navigator/tools/ape_native.py` is the only bridge between Python and
these planners — Python's role in the APE path is just marshaling (raw
scan + config → a ctypes struct → the native call → unpack the result),
no algorithm logic. See `docs/ca_architecture_deviations.md` for the full
architectural reasoning.

## 2. Multi-layer LiDAR

APE3's multi-layer input required extending the simulated LiDAR
(`models/x3-uav/4/model.sdf`, `vertical.samples` 1→5) and a new sensor
path: `sensor_msgs/LaserScan` is structurally single-plane (no vertical
dimension at all), and `ros_gz_bridge`'s `LaserScan` converter silently
truncates a multi-layer `gz.msgs.LaserScan` down to one layer with **no
warning** (confirmed empirically by direct inspection — `gz topic -e`/
`ros2 topic echo` on both the raw Gazebo topic and the bridged ROS 2
topic). Multi-layer data instead goes through the sensor's native
point-cloud output (`gz.msgs.PointCloudPacked` → `sensor_msgs/PointCloud2`,
which the bridge does support correctly — `height`/`width` on the bridged
topic matches the sensor's real layer/sample counts). APE1/APE2 and the
base (non-event) avoidance path are unaffected — they still use the
original single-layer `LaserScan` topic exactly as before.

## 3. Architecture

Everything lives under `ca_navigator/measurements/gem5_power_study/`,
untracked scratch/research infrastructure alongside the pre-existing stock
gem5 clone at `ca_navigator/measurements/gem5/`. Nothing in the main
`ca_navigator/` package is modified by this study.

```
gem5_power_study/
├── bench/                       # cross-compiled microbenchmark binaries
│   ├── src/main.c                # calls the real ape{1,2,3}_*_plan() functions
│   ├── Makefile                  # AArch64 / Cortex-A78 target
│   ├── Makefile.mcu               # 32-bit ARM/Thumb / Cortex-M7-approx target
│   └── build/ape_bench            # cross-compiled static ELF
├── configs/                     # gem5 SE-mode Python configs
│   ├── cortex_a78_o3.py           # ArmO3CPU tuned toward Cortex-A78
│   ├── cortex_m7_inorder.py       # in-order MinorCPU approximating Cortex-M7
│   ├── caches.py
│   ├── se_run.py                  # A78 study driver
│   ├── se_run_mcu.py              # MCU-approximation study driver
│   └── mcpat_base_template.xml
├── mcpat/                       # McPAT power tool (git clone, patched build)
├── scripts/
│   ├── gem5_to_mcpat.py
│   ├── freeze_measured_latencies.py       # writes gem5_measured_latencies.py
│   └── freeze_measured_latencies_mcu.py   # writes gem5_measured_latencies_mcu.py
└── results/, results_mcu/
    ├── ape1/ape2/ape3/            # gem5 stats.txt per profile
    ├── mcpat_xml/, mcpat_out/
    └── FINDINGS.md
```

- **`cortex_a78_o3.py`** — an `ArmO3CPU` subclass. Functional-unit
  latencies (`IntAlu` = 1 cycle, `IntMult` = 3, `IntDiv` = 12) are gem5's
  stock `O3_ARM_v7a` values, which already match the ARM Cortex-A78
  Software Optimization Guide (`ARM 103-0101`). Width/queue parameters —
  4-wide fetch/decode, 6-wide rename, 8-wide issue, 160-entry ROB, ~13
  execution ports — are widened from gem5's default A9-class template
  toward A78, sourced from public secondary reporting (WikiChip, a UIUC
  CS433 course writeup) since ARM has not published exact A78 queue
  sizes; flagged inline as a modeling estimate.
- **`cortex_m7_inorder.py`** — an in-order MinorCPU config (~400 MHz,
  single-issue, gem5's stock `MinorDefaultFUPool`) approximating
  Cortex-M7. This is an explicit approximation, not a true M-profile
  simulation — gem5 has no Cortex-M/M-profile CPU model at all; see that
  file's module docstring for the full caveat list (coarse per-opClass FU
  timings borrowed from gem5's default pool; TCM approximated as L1-only
  cache; single-issue pipeline width).
- **`caches.py`** — private L1I (32 KiB) / L1D (64 KiB) / L2 (512 KiB).
  L2 size is a modeler's estimate — NVIDIA does not publish per-core Orin
  NX cache sizes.
- **`se_run.py`/`se_run_mcu.py`** — assemble the gem5 `System` (2.0 GHz
  clock for the A78 study, from the Jetson Orin NX datasheet
  DS-10662-001; `LPDDR5_5500_1x16_BG_BL32` DRAM model), wire CPU → L1 →
  L2 bus → L2 → membus → DRAM, and load `ape_bench` as an SE-mode
  `Process`.
- **`mcpat/`** — upstream McPAT 1.3, patched to build 64-bit (its
  makefile hardcoded a 32-bit build that fails on this machine's glibc)
  — a well-known, safe workaround with no functional effect on McPAT's
  power model. Tech node is set to 22 nm, the closest node McPAT's
  built-in device models support to Orin NX's actual 8 nm (Samsung 8LPP)
  process — affects absolute power numbers only, not the cycle-count/
  latency results, which come from gem5 alone.
- **`gem5_to_mcpat.py`** — reads the ROI stat block from `stats.txt` and
  maps gem5 25.1.0.1 stat names onto the McPAT template via
  occurrence-indexed regex substitution (several components reuse stat
  names, so a helper locates the Nth match of a given `<stat name=...>`
  tag by document position).

## 4. Reproducing the study

```bash
# 1. Build the real planners for gem5 (both targets share the same
#    native source — see ca_navigator/native/ape_ops/)
cd ca_navigator/measurements/gem5_power_study/bench
make                    # AArch64 / Cortex-A78 study
make -f Makefile.mcu    # 32-bit ARM/Thumb / Cortex-M7-approximation study

# 2. Run gem5 SE-mode for each profile, each study
GEM5=../../gem5/build/ALL/gem5.opt
cd ..
for p in ape1 ape2 ape3; do
  $GEM5 --outdir=results/$p configs/se_run.py $p 200
  $GEM5 --outdir=results_mcu/$p configs/se_run_mcu.py $p 200
done

# 3. Translate stats to McPAT XML and run McPAT (A78 study only —
#    energy accounting uses the Orin-NX/A78 numbers, see docs/ca_architecture_deviations.md)
for p in ape1 ape2 ape3; do
  python3 scripts/gem5_to_mcpat.py results/$p/stats.txt results/mcpat_xml/$p.xml
  mcpat/mcpat -infile results/mcpat_xml/$p.xml -print_level 5 -opt_for_clk 1 \
    > results/mcpat_out/$p.txt
done

# 4. Freeze both studies' results into the live simulator's source of truth
#    (ca_navigator/tools/gem5_measured_latencies.py and _mcu.py — commit the result)
python3 scripts/freeze_measured_latencies.py
python3 scripts/freeze_measured_latencies_mcu.py
```

Each profile runs against one fixed, open-corridor scan fixture defined
directly in `bench/src/main.c` — each planner call is fully
self-contained, so there is no separate "shared" profile to compose.

## 5. Measured results

| profile | A78 cycles/inv | A78 µs/inv | MCU-approx cycles/inv | MCU-approx µs/inv (@400MHz) |
|---|---:|---:|---:|---:|
| APE1 (Bug)  | 2,623    | 1.31   | 11,635    | 29.1   |
| APE2 (DWA)  | 96,071   | 48.04  | 286,389   | 716.0  |
| APE3 (VFH)  | 223,993  | 112.00 | 502,900   | 1,257.3 |

Real DWA/VFH produce an 85× spread between APE1 and APE3 (A78) —
genuinely differentiated compute cost driven by real algorithmic work
(candidate-grid forward simulation, multi-layer histogram + valley
search). On the MCU-approximation target, this puts `budget_ms`
(`= APE_LATENCY_US × DEADLINE_SCALE / 1000`) at roughly **29ms / 716ms /
1257ms** — genuinely comparable in magnitude to real physical event
deadlines (`deadline_min_s`/`deadline_max_s` in `ca_navigator/config.py`,
147ms-3500ms).

## 6. Caveats

- **ROB size (160 entries)** and **private L2 size (512 KiB)** in the A78
  config are modeler estimates, not datasheet values — flagged inline in
  `configs/cortex_a78_o3.py` / `configs/caches.py`.
- **McPAT tech node (22 nm vs. Orin NX's real 8 nm)** affects absolute
  power numbers only; the cycle-count/latency findings do not depend on it.
- The Cortex-M7-approximation study is not a true M-profile simulation —
  gem5 has no Cortex-M CPU model; see `configs/cortex_m7_inorder.py`'s
  docstring for the full caveat list.
- Both studies are **single-thread, solo-planner** SE-mode simulation.
  They do not model the CA parallel-halt scenario's real cache/memory
  contention among three concurrently-launched APE threads running on
  shared cores.

## 7. Live CA-mode win rates

Measured directly from `logs/run_logs.json` (10 good runs, all four
strategies reached target, zero errors): 260 CA-mode event resolutions,
tallied by which planner's proposal the selector actually took.

| Planner | Wins | Win rate |
|---|---:|---:|
| APE1 (Bug) | 169 | 65.0% |
| APE3 (VFH) | 46  | 17.7% |
| APE2 (DWA) | 45  | 17.3% |

Zero `DEADLINE`/`PREEMPTIVE` violations across all 10 runs (526 total
events handled). The split is stable per-run (APE1 60–68%, APE2 15–25%,
APE3 14–19% in every individual run, not just in aggregate). This
confirms the real-algorithm gem5 budgets do produce genuine mixed
APE1/APE2/APE3 selection — a real speed/accuracy tradeoff engages, unlike
the old synthetic-workload model where APE3 won every resolution
(11/11). APE1 dominates because it's fastest and its proposal is often
good enough before APE2/APE3 finish, but APE2 and APE3 each still win
roughly 1-in-6 times.

## 8. Open questions

- Whether `DEADLINE_SCALE=1000` (`ca_navigator/tools/orin_nx_cycle_model.py`)
  still needs adjustment given these larger, real-algorithm budgets is
  unverified — flagged, not resolved, in that file's `DEADLINE_SCALE`
  comment.
