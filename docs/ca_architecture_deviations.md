# CA Architecture: Deviations from Paul & Bettendorf (2024)

## Source

Paul, J. M. and Bettendorf, I. **"Paradox, Conflict and Structural
Intelligence."** *IEEE Computer*, 2024. NSF Grant No. 2204780.
(https://vtechworks.lib.vt.edu/server/api/core/bitstreams/982c4a51-2986-47e1-8374-fd8a2232aa27/content)

This is the origin paper for the Conflict Architecture (CA) concept —
APE1/APE2/APE3, a selector, and event-driven resolution — that
`ca_navigator/navigation/nav_algorithm_T.py` implements. It is also the
source of the "AMD Zen 4" SystemC timing model that
`ca_navigator/tools/orin_nx_cycle_model.py` and
`docs/compute_power_model.md` cite (see "Timing model provenance" below).

---

## What's faithfully preserved

- **MISD structure**: all APEs process the same input in isolation
  (paper Fig. 4), and a selector picks one output based on a physical
  event. CANavigator launches APE1/2/3 as independent threads on the same
  sensor snapshot — matches.
- **Three tiers of speed/accuracy trade-off**: APE1 fast/crude, APE2
  middling, APE3 slow/best — same qualitative shape as the paper's
  Random / Tremaux's / Map Search.
- **Physical parameter (time) resolves the conflict**, not logic — both
  use elapsed/available time as the selection axis rather than a scored
  competition.

---

## Fundamental deviations (contradict the paper's central thesis)

These two are not implementation shortcuts — they invert the specific
philosophical claim the paper opens with: *"Computing does not
traditionally accommodate such conflict. Instead, it forces resolution
... [CA] accommodates ambiguity and conflict by holding multiple answers
at the same time ... until a physical event ... resolves the conflict."*

### 1. Selection is logical/predictive, not physical/opportunistic

Paper: the paradox gate produces genuinely simultaneous, unresolved
outputs, held until a physical event arrives; the MUX (Fig. 3) only
works because *both* answers already exist by the time the event fires —
selection is passive, discovering whichever answer is physically ready.

Code: `_evt_winner_for_deadline()` (`nav_algorithm_T.py:609-630`) is a
pure, deterministic lookup against a declared `deadline_s` — computable
the instant an event arrives, **before any APE has done any work** —
that decides in advance which APE will be used. The code then blocks
waiting only for that pre-chosen APE (`nav_algorithm_T.py:915-920`:
*"self._evt_winner was fixed at intake. Just wait for it. No fallback
logic."*). This is a logical resolution computed ahead of the physical
event, not a resolution performed *by* the physical event — exactly the
conventional, resolution-forcing computing model the paper is arguing
against, wrapped in an MISD-looking three-thread shell. (A cascading
best-available fallback does exist, but only in the separate
event-preemption "salvage" path, lines 784-807 — not in the primary
resolution loop.)

### 2. Computation is discrete/on-demand, not continuous/isolated

Paper: APEs are always running, independently, on a persistent input
stream — this is why partial results survive across events ("if APE3 is
interrupted after mapping only part of the current cell, the partial
mapping is kept in memory"). The brain-inspired MISD argument depends on
this continuity — each region is mid-thought at all times; a physical
event just reads out whatever state that ongoing thought has reached.

Code: APE1/2/3 threads are spawned fresh from a new sensor snapshot on
every event and torn down immediately after (`_evt_clear()`,
`nav_algorithm_T.py:641-652`, `831-836`). There is no ongoing, isolated
computation being interrupted and read out — each event is a race
started from zero. This is a conventional real-time-scheduling pattern
("compute on demand, race a clock"), not the always-on,
parallel-conflicting-models picture central to the paper's argument.

---

## Implementation-level deviations (do not contradict the thesis)

These are legitimate engineering choices or extensions, not violations
of the paper's central claim:

- **Fixed n=3 APEs** vs. the paper's conceptually unbounded APE₁...APEₙ
  (Fig. 4).
- **Execution time is injected, not emergent**: the paper's timing
  differences fall out of actually running different algorithms (Random
  vs. Tremaux's vs. Map Search) instrumented with SystemC operator
  latencies. CANavigator's APE1/2/3 are similar velocity-command
  heuristics with an artificial `time.sleep(budget_ms)` bolted on, where
  `budget_ms` comes from a disconnected op-counting model
  (`orin_nx_cycle_model.py`) rather than real algorithmic cost.
- **Explicit declared deadline** (`evt["deadline_s"]`) vs. the paper's
  implicit, empirically-discovered "how much time has passed since the
  last event."
- **Different evaluation domain**: abstract 2D maze + pursuer (binary
  caught/not-caught) vs. full 3D Gazebo drone physics + LiDAR obstacle
  avoidance with its own violation/safety framework (deadline,
  preemptive, corner-guard, no-fly-zone violations).
- **An entire energy/power-modeling layer with no counterpart in the
  paper** (`OrinNxCycleMeter`, `latency_to_energy_j`, McPAT, the gem5
  validation study in `docs/gem5_power_study.md`). The paper never
  discusses energy or physical hardware calibration.

---

## Timing model provenance (separate finding, not thesis-related)

`orin_nx_cycle_model.py`'s docstring and `docs/compute_power_model.md`
citation [8] describe the paper's SystemC "Zen 4" model as running at
**4500 MHz** (citing "AMD Ryzen 9 7950X Processor Specifications, 2022")
with latencies in **SC_US** (microseconds). The actual paper states:

> "The AMD Zen 4 processor is clocked at 1GHz which has a period of one
> nanosecond. All operator latencies are in relation to the clock
> period... latencies are in nanosecond[s]. **As with all simulations,
> the timing is relative.**"

Two errors follow from this:

1. The clock frequency CANavigator scaled from (4500 MHz, real 7950X
   silicon) does not match what the cited model actually used (1 GHz, an
   arbitrary round-number simulation choice) — the `2.25× = 4500/2000`
   clock-ratio scale factor baked into `_L` in `orin_nx_cycle_model.py`
   has no basis in the source paper.
2. The paper explicitly disclaims absolute-time interpretation ("timing
   is relative") — its numbers were never meant to be read as calibrated
   real-world hardware latency. Treating them as a portable, physically
   real Zen-4-to-Orin-NX latency baseline (as `orin_nx_cycle_model.py`
   does) is a category error independent of, and upstream of, the
   `DEADLINE_SCALE` double-counting bug documented in
   `docs/gem5_power_study.md` §3.1.

This means the entire downstream power/energy modeling effort — not just
the specific double-counting bug — is built on a foundation (the paper's
SystemC numbers) that was never intended to bear that kind of absolute,
physical weight. Revisiting `DEADLINE_SCALE`/`_L` as if a bigger,
literature-backed multiplicative constant could fix this is not
correct: the source paper cannot be used to derive an absolute latency
budget at all, by its own explicit statement.

---

## Current state: real compute, MCU-class timing, two hardware targets

Two follow-up changes replaced the mechanisms above rather than just
documenting their problems:

**1. Genuine computation, genuine parallelism.** `_evt_plan_ape1/2/3()` no
longer stand in for compute cost with `time.sleep()` alone. They call
`ca_navigator/tools/ape_native.py` → `ca_navigator/native/ape_ops/` (a
ctypes-bridged native build of the same op-kernel C code the gem5 studies
use), which genuinely consumes CPU cycles and — because `ctypes` releases
the GIL for foreign calls — genuinely runs across real cores in parallel
with the other APE threads. The real compute duration is tiny relative to
`budget_ms` (real host hardware is far faster than the simulated targets
below), so a top-up `time.sleep()` still makes up the difference to reach
the nominal gem5-derived budget — but the work itself is now real, not
purely simulated.

**2. Planning-deadline timing now models the flight controller, not the
companion computer.** `orin_nx_cycle_model.py`'s `APE_LATENCY_US` (what
`nav_algorithm_T.py`'s `budget_ms` derives from) is now sourced from a
second gem5 study — `configs/cortex_m7_inorder.py`, an in-order MinorCPU
config approximating Cortex-M7 (gem5 has no true M-profile CPU model; see
that file's docstring for the full caveat list) — instead of the
Cortex-A78/Orin-NX study. Real ArduPilot-class flight controllers run on
Cortex-M-class MCUs; using the Orin-NX numbers made every event's
deadline vastly exceed even APE3's budget, so the opportunistic selector
(§ above) always picked APE3 — the speed/accuracy tradeoff never engaged
(confirmed empirically at the time: 11/11 CA resolutions picked APE3 in
a real run). This has since been superseded twice more — first by
retuning `deadline_min_s` to a physically-derived floor (still
insufficient on its own, see §"Real C-native APE algorithms" below), and
then by replacing the synthetic op-count workload with real Bug/DWA/VFH
algorithms whose gem5-measured cost is now large enough
(APE1≈29ms/APE2≈716ms/APE3≈1257ms) to be genuinely comparable to real
event deadlines, without further deadline tightening. See that section
for the current numbers and reasoning; whether `DEADLINE_SCALE=1000` still
needs adjustment on top is still explicitly unverified, not silently
resolved.

**This creates a real, load-bearing split worth being explicit about**:
`OrinNxCycleMeter`/`latency_to_energy_j` (energy/power accounting) are
*unchanged* and still model the Orin NX companion computer specifically
(`_TDP_W`, `_N_CORES`, etc. are all Orin-NX constants) — the codebase now
models two different pieces of hardware for two different purposes:
planning-deadline timing (flight-controller-class MCU) and energy
accounting (companion-computer-class Orin NX). That's a defensible
division of labor if the two subsystems genuinely run on different
hardware in a real deployment, but it's not automatic or free — it should
be stated plainly in any writeup rather than left for a reader to
discover by diffing constants.

---

## Real C-native APE algorithms (Bug / DWA / VFH)

Two more changes closed remaining gaps between this codebase and the
paper's design, and between the "compute cost" model and what the code
actually does.

**1. The APEs are real algorithms now, not simplified heuristics.**
APE1 = a reactive potential-field/Bug-style controller, APE2 = real
Dynamic Window Approach (Fox, Burgard & Thrun 1997), APE3 = real Vector
Field Histogram (Borenstein & Koren 1991) with a valley-search step —
restoring the fidelity gap flagged earlier in this document, where
APE3 was supposed to be a "Map Search algorithm" per the paper but was
actually a confidence-weighted sidestep heuristic. See
`docs/gem5_power_study.md` §5 for the algorithm details and measured
gem5 cycle costs.

**2. Python's role in the APE path is now marshaling only — all
algorithm logic moved to C.** `_evt_plan_ape1/2/3()` in
`nav_algorithm_T.py` used to run real Python decision math
(`_shared_motion_caps`, `_confidence_from_scan`, etc.) *separately* from
a synthetic C op-count workload that only existed to consume CPU cycles
matching a gem5-measured budget — two independent implementations that
had to be kept in sync by hand, which caused two rounds of real bugs
this session (a stale `_APE2_UNIQUE` profile with phantom scans, an
undocumented `sqrt≈div×2` approximation). Both problems are gone at the
root: there is now exactly one implementation of each APE's decision
logic (`ca_navigator/native/ape_ops/src/ape{1,2,3}_*.c`), written in C,
called from Python via `ape_native.py`, and gem5 measures that same real
code directly. Python's remaining job — marshal the raw LiDAR scan +
config into a `ctypes` struct, call the native planner, unpack
`(v, wz, vz, score)`, hand it to the (unchanged) selector — matches the
isolation the paper's MISD framing already called for even more
strictly: each APE is now a genuinely separate, independently-compiled
unit of computation, not three Python closures sharing an interpreter.

**3. This also (mostly) resolves the deadline/budget mismatch — with
real algorithmic cost, not another tuned constant.** Real DWA/VFH do
genuinely more work than the old synthetic op-count profiles: gem5-measured
MCU-approximation budgets are now **APE1≈29ms / APE2≈716ms / APE3≈1257ms**
(up from the synthetic study's 15.9/16.8/58.6ms) — comparable in
magnitude to real event deadlines (147ms-3500ms) for the first time,
rather than needing a separately-justified deadline floor. This was a
consequence of implementing real algorithms, not a target chosen in
advance; whether it produces genuine APE1/APE2 selections under `CA`
mode in live operation (versus APE3 still usually winning) is the
natural next thing to re-measure — see `docs/gem5_power_study.md` §5.4.

**4. Multi-layer sensing.** APE3 also gained genuinely richer input:
the simulated LiDAR now has 5 vertical layers (`models/x3-uav/4/model.sdf`),
feeding APE3's VFH histogram with multi-layer obstacle consensus (a
sector is blocked if *any* layer sees a close obstacle there) — real
altitude-awareness APE1/APE2 don't have, justifying APE3's extra cost
with genuinely richer data, not just more arithmetic on the same input.
Getting this sensor data to APE3 required a real infrastructure decision,
not just an SDF edit: `sensor_msgs/LaserScan` can't represent multi-layer
data at all, and empirical testing (before writing any code, not assumed)
found `ros_gz_bridge`'s `LaserScan` converter silently truncates a
multi-layer scan to one layer with no warning. Multi-layer data instead
flows through the sensor's native point-cloud topic
(`PointCloud2`, which the bridge does support correctly) via a new,
APE3-only subscriber — the original `LaserScan` path, and everything
downstream of it (APE1, APE2, the base avoidance loop), is untouched.

**5. Compute-energy accounting fixed to use the right hardware's
numbers.** `OrinNxCycleMeter.record_event()` (`orin_nx_cycle_model.py`)
previously read the generic, live `APE_LATENCY_US` symbol — which,
since the MCU rewire earlier in this work, is Cortex-M7-approximation
timing (ms-scale). But the surrounding energy formula
(`latency_to_energy_j`) is parameterized with real Orin NX datasheet
constants (TDP=25W, N_cores=8, idle_frac) — physically meaningful only
paired with Orin-NX-measured compute duration (µs-scale). This silently
mixed two hardware targets: Orin-NX power constants integrated over
MCU-scale compute time. `record_event()` now explicitly uses
`_GEM5_A78_LATENCY_US` instead — the deliberate, documented split is:
`budget_ms` (MCU-class) drives planning-deadline timing, since that's
the tightest realistic reaction-time constraint and what makes the CA
tradeoff meaningful; compute energy uses the Orin-NX numbers, matching
a realistic deployment where compute this heavy (real DWA/VFH) runs on
a Jetson-class companion computer onboard the drone. Two different
questions, two correctly-paired models — not an accident, and not
interchangeable. The practical effect on `compute_energy_j` is currently
small either way (compute time is negligible next to mission wall-clock
time regardless of which figure is used, so `U_eff` stays pinned near 0
and energy stays near the idle-power floor) — but the reasoning is now
coherent, which matters more as compute cost grows.
