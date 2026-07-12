# Event Handling in the Conflict Architecture (CA)

This document describes the event episode lifecycle, violation taxonomy, selector
calibration, commitment hold rationale, and CSV output schema for the CANavigator teleop
CA framework after the Option B fix (explicit execution commitment window).

---

## Section 1 — Event episode lifecycle

A single event episode passes through five phases: emission, arrival, parallel APE
planning, resolution + commitment hold, and return to baseline.

### 1.1 Emission

`EventEmitter` samples inter-arrival times `dt` log-uniformly from [0.02, 4.0] s.
Each event carries a deadline:

```
deadline_s = clamp(alpha * dt, [deadline_min_s, deadline_max_s])
```

with `alpha = 0.85`, `deadline_min_s = 0.073 s`, `deadline_max_s = 3.50 s`.
These are `EventCfg`'s own tuned defaults (`ca_navigator/tools/event_emitter.py`),
independently derived from — not propagated from — `TeleopConfig`'s own
`deadline_min_s` (see `docs/CONFIGURATION.md`).

### 1.2 Arrival and intake

The nav loop polls `_EventSub.pop()` once per tick (~33 ms at 30 Hz).  When a new
event is returned:

1. `_events_handled` is incremented.
2. If an old event is still active, the salvage path runs (see §2 PREEMPTIVE).
3. `_pending_evt`, `_evt_deadline_at`, `_evt_active` are set for the new event;
   `_evt_resolved` is cleared.

### 1.3 Parallel APE launch

In CA mode all three planner threads start immediately.  In solo modes (APE1,
APE2, APE3) only one thread starts.  Each thread writes its result into
`_evt_proposals` under a lock when it finishes (~1 ms for APE1, ~16 ms for APE3).

### 1.4 Selector logic

Selection is opportunistic, not predicted at intake: all three APE threads run
in parallel from event arrival. As soon as APE3 (best quality) posts, it wins —
nothing better can arrive. Otherwise the selector keeps waiting for a better
answer until the deadline, then takes whichever of APE2/APE1 is ready. If no
plan is ready when the deadline passes, that's a `DEADLINE` violation. See
`nav_algorithm_T.py::_evt_cascade_order()` and
`docs/ca_architecture_deviations.md` §1 for the full rationale and what this
replaced (a deadline→tier lookup that picked a winner before any APE had run).

### 1.5 Resolution and commitment hold

When a plan is chosen for the first time (`not self._evt_resolved`):

1. A RESOLVED log record is written.
2. `_cycle_meter.record_event()` is called.
3. `_evt_resolved = True` is set.
4. The winning (v, wz, vz) triple is cached in `_resolved_cmd`.
5. `_evt_resolved_at` records the wall time.
6. `_commit_hold_active = True` is set.
7. `_evt_clear()` closes the event window immediately (resets `_evt_active`,
   `_pending_evt`, proposals, and threads).

On the immediately following ticks the commitment hold path fires:

```python
if _commit_hold_active:
    hold_elapsed = time.time() - _evt_resolved_at
    if hold_elapsed < commit_hold_s:   # 0.9 s
        v_cmd, wz_cmd, vz_cmd = _resolved_cmd
    else:
        _commit_hold_active = False
```

### 1.6 Return to baseline go-to

Once `_commit_hold_active` becomes False the nav loop reverts to the normal
avoidance / heading-select / go-to path with no special-case logic.

### Timeline diagram

```
t_recv                  APE1 ready   APE2 ready   APE3 ready   deadline_at
  |                         |             |             |          |
  |---~29ms---|---~716ms----|---~1257ms---|---up to 3.50s----|
  ^            ^            ^             ^
  event        APE1         APE2          APE3       deadline
  arrives      done         done          done        window ends
                |
                | selector picks best ready plan (APE3 immediately if ready,
                | else best-of-ready at deadline)
                v
          RESOLVED
                |
                |<------- commit_hold_s = 0.9 s ------->|
                |  _resolved_cmd applied every nav tick  |
                                                         |
                                                         v
                                                  return to baseline go-to
```

---

## Section 2 — Violation taxonomy

There are exactly three outcomes for each event:

### RESOLVED

The selector found a ready plan within the deadline.  No violation is recorded.
`_events_handled` was already incremented at arrival; no violation counter changes.

### DEADLINE

`time_left` reached 0 before any plan was selected.  This is a genuine violation:
the drone had no committed evasion plan in time.

- `_evt_violate("DEADLINE")` is called.
- `_events_violated` and `_events_violated_deadline` are both incremented.
- A DEADLINE log record is written.

### PREEMPTIVE

A new event arrived while the old event was still active and no usable plan existed
for the old event.  This is a genuine violation on the old event.

- `_evt_violate("PREEMPTIVE")` is called.
- `_events_violated` and `_events_violated_preemptive` are both incremented.
- A PREMPTIVE log record is written.

### Why ghost violations no longer occur

Before the fix, `_evt_clear()` was commented out in the RESOLVED branch.  This left
`_evt_active = True` and `_pending_evt` set after resolution, so the selector kept
re-applying the APE plan on every nav tick.  When no new event arrived before the
deadline expired, `tl <= 0` fired `_evt_violate("DEADLINE")` on an event that had
already been resolved — a ghost violation.

After the fix, `_evt_clear()` is called immediately upon resolution.  `_evt_active`
becomes False on the same tick, so the deadline check is never reached for a
resolved event.

**Quantification from the 100-run experiment:** 23% of DEADLINE records were ghost
violations on resolved events.  This matches exactly the fraction of inter-arrival
gaps longer than `deadline_min` (1.471 s) in the log-uniform emission distribution,
confirming these violations were not caused by genuine planner latency but by the
open event window.

---

## Section 3 — Selector calibration

There are no static tier boundaries (no `t_hard_s`/`t_med_s`, no deadline→tier
lookup table). The selector is purely opportunistic — see §1.4 and
`nav_algorithm_T.py::_evt_cascade_order()`. APE budgets (`ape1_budget_ms`,
`ape2_budget_ms`, `ape3_budget_ms` in `EventDecisionCfg`) come live from
`orin_nx_cycle_model.py`'s gem5-measured `APE_LATENCY_US`, not a hand-tuned
constant — see `docs/gem5_power_study.md` for how those numbers are produced
and kept in sync with the real native planners.

---

## Section 4 — Commitment hold rationale

### Why one nav tick is insufficient

A single nav tick is ~33 ms (30 Hz).  APE2 event commands move at up to
`v_cap_frac * max_v = 0.75 * 15.0 = 11.25 m/s` (APE3/VFH now shares this same
ceiling — see `ape3_vfh.c`; it no longer uses the old `sidestep_speed_frac`
throttle inherited from the pre-VFH sidestep heuristic).  Against a ~1.2 m
radius obstacle the drone needs to open at least 1.5 m of lateral clearance
before it can safely resume forward flight.  Even at the *lowest* end of the
observed speed range (well below the 11.25 m/s ceiling), that clearance opens
in well under 0.29 s; `commit_hold_s = 0.9 s` remains a comfortable — now more
conservative than strictly required — margin.

### Why the hold is explicit

Before this fix the hold was an accidental side-effect of the commented-out
`_evt_clear()`.  This made the duration implicitly equal to
`deadline_at - t_resolved`, which varied with each event and could be arbitrarily
long.  Making it an explicit `EventDecisionCfg` parameter:

- Appears in the CFG log record, making every run fully auditable.
- Can be tuned independently of the deadline distribution.
- Is documented with its physical motivation in the dataclass.

### Behaviour during the hold

During the hold the commitment hold path overrides `v_cmd`, `wz_cmd`, and `vz_cmd`
with the cached `_resolved_cmd` triple produced by the winning APE.  After
`commit_hold_s` elapses, `_commit_hold_active` is cleared and the nav loop returns
cleanly to the baseline go-to path.  The event episode is then fully closed with no
residual state.

---

## Section 5 — CSV output schema

Each row in `results_csv_path` corresponds to one strategy in one good run
(all strategies reached the target).

### Mission columns

| Column       | Type  | Description |
|--------------|-------|-------------|
| `run`        | int   | 1-based index of the good run (discarded attempts not counted) |
| `strategy`   | str   | Navigator name: APE1, APE2, APE3, or CA |
| `elapse_time`| float | Wall-clock seconds from nav start to target reached |

### NFZ columns

| Column           | Type  | Description |
|------------------|-------|-------------|
| `zone_violations`| int   | Number of no-fly zone boundary crossings detected by ViolationMonitor |

### Compute columns

| Column               | Type  | Description |
|----------------------|-------|-------------|
| `compute_latency_us` | float | Total Orin NX cycle-model latency in microseconds across all nav ticks |
| `compute_energy_j`   | float | Estimated compute energy in joules derived from cycle latency and elapsed time |

### Flight energy columns

| Column         | Type  | Description |
|----------------|-------|-------------|
| `energy_kj`    | float | Flight energy in kilojoules measured by EnergyMonitor |
| `mean_power_kw`| float | Mean flight power in kilowatts over the run |

### Event columns

| Column                     | Type  | What it counts | What it does NOT count |
|----------------------------|-------|----------------|------------------------|
| `events_handled`           | int   | All events received by the nav loop (incremented at arrival, before any outcome) | Nothing excluded |
| `event_violated`           | int   | Total violations = DEADLINE + PREEMPTIVE | Events that resolved successfully |
| `event_violated_deadline`  | int   | Events where `time_left` reached 0 before any plan was selected | Ghost violations (eliminated by this fix) |
| `event_violated_preemptive`| int   | Events where a new event arrived before the old event resolved and no salvage plan existed | Events where a salvage plan was successfully applied |
| `event_violation_rate`     | float | `event_violated / events_handled` (0.0 if events_handled == 0) | — |

`event_violated == event_violated_deadline + event_violated_preemptive` always holds.
