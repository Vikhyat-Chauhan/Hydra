# CA Architecture: Deviations from Paul & Bettendorf (2024)

## Source

Paul, J. M. and Bettendorf, I. **"Paradox, Conflict and Structural
Intelligence."** *IEEE Computer*, 2024. NSF Grant No. 2204780.
(https://vtechworks.lib.vt.edu/server/api/core/bitstreams/982c4a51-2986-47e1-8374-fd8a2232aa27/content)

This is the origin paper for the Conflict Architecture (CA) concept —
APE1/APE2/APE3, a selector, and event-driven resolution — that
`ca_navigator/navigation/nav_algorithm_T.py` implements. It is also the
source of the "AMD Zen 4" SystemC timing model that
`ca_navigator/tools/orin_nx_cycle_model.py` cites (see "Timing model
provenance" below).

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

`orin_nx_cycle_model.py`'s docstring describes the paper's SystemC "Zen 4" model as running at
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
   `DEADLINE_SCALE` calibration question tracked in
   `docs/gem5_power_study.md` ("Open questions").

This means the entire downstream power/energy modeling effort — not just
the specific double-counting bug — is built on a foundation (the paper's
SystemC numbers) that was never intended to bear that kind of absolute,
physical weight. Revisiting `DEADLINE_SCALE`/`_L` as if a bigger,
literature-backed multiplicative constant could fix this is not
correct: the source paper cannot be used to derive an absolute latency
budget at all, by its own explicit statement.

---

## Event timing floors

`EventCfg` (`ca_navigator/tools/event_emitter.py`) has two independently-tuned
floors, sized so that no APE can win or violate a deadline purely by
coincidence rather than genuine timing pressure:

- **`dt_min_s`** (raw inter-arrival floor) = `0.02s`, comfortably below
  APE2's measured per-cycle budget (~90ms), so the tightest event gaps
  leave only APE1 eligible. (Eligibility is a race, not a guarantee:
  APE2's persistent worker loops continuously, unsynchronized to event
  arrivals, so even a window shorter than its period has some chance of
  catching a tick — `0.02s` keeps that chance to roughly 20%, well above
  APE1's own ~1ms cycle time.)
- **`deadline_min_s`** = `0.073s`, derived from `deadline_min =
  (sudden_obj_radius_m + vehicle_radius_m + sudden_obj_clearance_m) /
  (v_max + v_closing_obj) = (1.2 + 0.7 + 0.3) / (15.0 + 15.0) = 2.2m /
  30m/s ≈ 0.073s`, using `v_closing_obj = 15.0 m/s` — the top of the
  5-15 m/s closing-speed range commonly cited in bird-strike/small-UAV
  sense-and-avoid literature, appropriate for a hard minimum-reaction-time
  floor (which should use the worst case of its own cited range, not a
  typical value). This sits below APE2's cycle time, so the tightest
  events can genuinely exceed APE2's real compute cost.

---

## Current implementation: real algorithms, two hardware targets

**APE1/2/3 are real, literature-standard algorithms, written in C — not
simplified heuristics.** APE1 is a reactive potential-field/Bug-style
controller; APE2 is real Dynamic Window Approach (Fox, Burgard & Thrun
1997); APE3 is real Vector Field Histogram (Borenstein & Koren 1991) with
a valley-search step. This is what restores the "Map Search algorithm"
fidelity the paper calls for in APE3's role. See `docs/gem5_power_study.md`
for algorithm details and measured gem5 cycle costs.

**Python's role in the APE path is marshaling only.** `_evt_plan_ape1/2/3()`
in `nav_algorithm_T.py` call `ca_navigator/tools/ape_native.py` →
`ca_navigator/native/ape_ops/src/ape{1,2,3}_*.c` via `ctypes`: marshal the
raw LiDAR scan + config into a struct, call the native planner, unpack
`(v, wz, vz, score)`, hand it to the selector. There is exactly one
implementation of each APE's decision logic, and gem5 measures that same
real C code directly — each APE is a genuinely separate,
independently-compiled unit of computation, matching the paper's MISD
framing more strictly than three Python closures sharing an interpreter
would. Because `ctypes` releases the GIL for foreign calls, the three
APEs also genuinely run across real cores in parallel during planning.

**Planning-deadline timing models the flight controller; energy
accounting models the companion computer — deliberately two different
hardware targets.** `orin_nx_cycle_model.py`'s `APE_LATENCY_US` (what
`nav_algorithm_T.py`'s `budget_ms` derives from) is sourced from the
Cortex-M7-approximation gem5 study (`configs/cortex_m7_inorder.py`, an
in-order MinorCPU config — gem5 has no true M-profile CPU model; see that
file's docstring for caveats), since real ArduPilot-class flight
controllers run on Cortex-M-class MCUs and that is the tightest realistic
reaction-time constraint — what makes the CA speed/accuracy tradeoff
meaningful. `OrinNxCycleMeter.record_event()` (`orin_nx_cycle_model.py`),
by contrast, uses `_GEM5_A78_LATENCY_US` (the Cortex-A78/Orin-NX gem5
study) for compute-energy accounting, since `latency_to_energy_j` is
parameterized with real Orin NX datasheet constants (TDP=25W, N_cores=8,
idle_frac) that are only physically meaningful paired with
Orin-NX-measured compute duration. This is a defensible division of labor
— compute this heavy (real DWA/VFH) plausibly runs on a Jetson-class
companion computer while planning deadlines are bounded by a
Cortex-M-class flight controller — not an accident or an interchangeable
pair of constants. The practical effect on `compute_energy_j` is
currently small (compute time is negligible next to mission wall-clock
time regardless of which figure is used, so `U_eff` stays pinned near 0
and energy stays near the idle-power floor), but the reasoning matters
more as compute cost grows.

**Measured budgets are now comparable in magnitude to real event
deadlines, and produce genuine mixed selection in live operation.**
gem5-measured MCU-approximation budgets are **APE1≈29ms / APE2≈716ms /
APE3≈1257ms** — comparable to real event deadlines (147ms-3500ms in
`ca_navigator/config.py`) for the first time, a consequence of
implementing real algorithms rather than a target chosen in advance. A
live measurement across 10 CA-mode runs (260 event resolutions) confirms
this actually engages the speed/accuracy tradeoff: APE1 wins 65.0%,
APE3 17.7%, APE2 17.3%, with zero deadline/preemptive violations — a
sharp contrast from the old synthetic-workload model, where APE3 won
every resolution (11/11). See `docs/gem5_power_study.md` for the full
breakdown. Whether `DEADLINE_SCALE=1000` (`orin_nx_cycle_model.py`) still
needs adjustment on top of these budgets remains an open, unverified
question.

**Multi-layer sensing.** APE3 has richer input than APE1/APE2: the
simulated LiDAR has 5 vertical layers (`models/x3-uav/4/model.sdf`),
feeding APE3's VFH histogram with multi-layer obstacle consensus (a
sector is blocked if *any* layer sees a close obstacle there). This
required a real infrastructure decision, not just an SDF edit:
`sensor_msgs/LaserScan` can't represent multi-layer data at all, and
`ros_gz_bridge`'s `LaserScan` converter silently truncates a multi-layer
scan to one layer with no warning (confirmed empirically before writing
any code). Multi-layer data instead flows through the sensor's native
point-cloud topic (`PointCloud2`, which the bridge does support
correctly) via a new, APE3-only subscriber — the original `LaserScan`
path, and everything downstream of it (APE1, APE2, the base avoidance
loop), is untouched.
