"""
orin_nx_cycle_model.py — APE compute latency (gem5-measured) and Orin NX
compute energy model.

Two independent gem5 cycle-accurate studies feed this module, each
measuring the REAL native APE planners (ape1_bug_plan/ape2_dwa_plan/
ape3_vfh_plan — see ca_navigator/native/ape_ops/) directly, not a
synthetic op-count proxy:

  - `gem5_measured_latencies.py` — ArmO3CPU tuned toward Cortex-A78
    (NVIDIA Jetson Orin NX companion computer, DS-10662-001), feeds
    `_GEM5_A78_LATENCY_US`, which pairs with the energy model below
    (TDP/idle/N_cores are all Orin-NX-specific — this module's energy
    accounting still targets that companion computer specifically).

  - `gem5_measured_latencies_mcu.py` — an in-order MinorCPU
    APPROXIMATION of Cortex-M7 (see
    ca_navigator/measurements/gem5_power_study/configs/
    cortex_m7_inorder.py for the approximation's caveats — gem5 has no
    true M-profile CPU model), feeds `APE_LATENCY_US`, the live
    simulator's actual source of truth for `nav_algorithm_T.py`'s
    `budget_ms`. Real ArduPilot-class flight controllers run on
    Cortex-M-class MCUs, not Cortex-A78-class companion computers — see
    docs/ca_architecture_deviations.md for why planning-deadline timing
    and energy accounting now deliberately model two different pieces
    of hardware for two different purposes.

Both are small, checked-in, regenerable snapshots (regenerate via
gem5_power_study/scripts/freeze_measured_latencies{,_mcu}.py after any
change to the native planners or gem5 configs) so this module — and
everything that imports it, including the live ROS 2 navigator — never
needs the gem5 toolchain installed just to know APE compute cost.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
POWER MODEL PARAMETERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  TDP      = 25 W     NVIDIA Jetson Orin NX 16GB, MAXN (maximum performance)
                      power mode.  Source: NVIDIA DS-10662-001, Table 1
                      "Module Electrical Specifications".

  idle_frac = 0.10    Module idle power is approximately 2–3 W from NVIDIA
                      Jetson Orin Power Estimation and Measurement Application
                      Note (NVIDIA document TB-10580-001).
                      10% = 2.5 W / 25 W TDP.

  N_cores  = 8        8× ARM Cortex-A78AE cores.
                      Source: NVIDIA DS-10662-001, Table 2 "Module Features".

  f_base   = 2.0 GHz  CPU maximum frequency.
                      Source: NVIDIA DS-10662-001.

  α        = 1.5      CMOS dynamic-power frequency exponent.
                      Source: Bircher & John, IEEE Trans. Computers, 2012
                      (α is a material/process property, not
                      processor-specific).

Note: Because gem5's A78 cycle counts already reflect real execution at
f_base = 2.0 GHz, the frequency-ratio term (f/f_base)^α = 1.0 and is
omitted from the energy calculation.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
HISTORY
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

This file previously derived APE latency from a hand-authored op-count
table (`_L`/`_CYCLES`, transcribing nav_algorithm_T.py's old Python
decision-math heuristics op-for-op) rather than measuring real algorithm
execution. That table, and the DEADLINE_SCALE double-counting bug found
in it, are documented in docs/gem5_power_study.md and
docs/ca_architecture_deviations.md. It has been removed outright — once
gem5 measures the real Bug/DWA/VFH planners directly, there is nothing
left for a hand-estimated proxy to validate against, and keeping one
would only reintroduce the same drift-prone transcription problem that
caused that bug (found twice: a stale APE2 profile, and an undocumented
sqrt≈div×2 approximation).
"""

from __future__ import annotations

from ca_navigator.tools.gem5_measured_latencies import (
    ITERATIONS as _GEM5_ITERATIONS,
    TOTAL_CYCLES as _GEM5_TOTAL_CYCLES,
)
from ca_navigator.tools.gem5_measured_latencies_mcu import (
    ITERATIONS as _GEM5_MCU_ITERATIONS,
    TOTAL_CYCLES as _GEM5_MCU_TOTAL_CYCLES,
    CPU_CLOCK_HZ as _MCU_CPU_CLOCK_HZ,
)

# Cycle-time constants for each gem5 study's clock — used only to convert
# each study's raw TOTAL_CYCLES into microseconds below.
_CYCLE_TIME_US_A78 = 1.0 / 2.0e3          # 1 / 2.0 GHz = 0.0005 µs/cycle (DS-10662-001)
_CYCLE_TIME_US_MCU = 1.0e6 / _MCU_CPU_CLOCK_HZ  # 0.0025 µs/cycle @ 400 MHz

# ---------------------------------------------------------------------------
# gem5-measured latency (µs), Cortex-A78 / Orin-NX companion-computer study.
# Pairs with the energy model below (OrinNxCycleMeter/latency_to_energy_j
# are Orin-NX-specific) — NOT the live simulator's timing source of truth,
# see APE_LATENCY_US below for that.
# ---------------------------------------------------------------------------
_GEM5_A78_LATENCY_US: dict[str, float] = {
    "APE1": _GEM5_TOTAL_CYCLES["ape1"] / _GEM5_ITERATIONS * _CYCLE_TIME_US_A78,
    "APE2": _GEM5_TOTAL_CYCLES["ape2"] / _GEM5_ITERATIONS * _CYCLE_TIME_US_A78,
    "APE3": _GEM5_TOTAL_CYCLES["ape3"] / _GEM5_ITERATIONS * _CYCLE_TIME_US_A78,
}

# ---------------------------------------------------------------------------
# gem5-measured latency (µs), Cortex-M7-approximation (flight-controller-
# class MCU) study — THE LIVE SIMULATOR'S SOURCE OF TRUTH. Real
# ArduPilot-class flight controllers run on Cortex-M-class MCUs, not
# Cortex-A78-class companion computers — see
# ca_navigator/measurements/gem5_power_study/configs/cortex_m7_inorder.py
# for the approximation's caveats (gem5 has no true M-profile CPU model)
# and docs/ca_architecture_deviations.md for why this is the number
# nav_algorithm_T.py's budget_ms is derived from, not _GEM5_A78_LATENCY_US.
# ---------------------------------------------------------------------------
APE_LATENCY_US: dict[str, float] = {
    "APE1": _GEM5_MCU_TOTAL_CYCLES["ape1"] / _GEM5_MCU_ITERATIONS * _CYCLE_TIME_US_MCU,
    "APE2": _GEM5_MCU_TOTAL_CYCLES["ape2"] / _GEM5_MCU_ITERATIONS * _CYCLE_TIME_US_MCU,
    "APE3": _GEM5_MCU_TOTAL_CYCLES["ape3"] / _GEM5_MCU_ITERATIONS * _CYCLE_TIME_US_MCU,
}

# DEADLINE_SCALE: the ONLY software/scheduling-overhead multiplier applied
# in this pipeline (Python interpreter + OS/ROS 2 contention on the flight
# controller / companion computer). budget_ms = APE_LATENCY_US[name] *
# DEADLINE_SCALE / 1000 (µs -> ms unit conversion is the trailing /1000;
# DEADLINE_SCALE is the overhead factor). APE_LATENCY_US is real
# MCU-approximation-gem5-measured compute cost of the real Bug/DWA/VFH
# planners — whether DEADLINE_SCALE=1000 is still the right overhead
# multiplier on top of these numbers (now substantially larger than the
# old synthetic-workload figures, since real DWA/VFH do genuinely more
# work) is unverified — a modeling decision, not implied by this file —
# flagged, not resolved, here. See docs/ca_architecture_deviations.md.
DEADLINE_SCALE: float = 1000.0

# ---------------------------------------------------------------------------
# Power model parameters — NVIDIA Jetson Orin NX 16GB (DS-10662-001)
# ---------------------------------------------------------------------------
_TDP_W:     float = 25.0          # Module TDP, MAXN mode (DS-10662-001 Table 1)
_IDLE_FRAC: float = 0.10          # ~10%: ~2.5 W idle / 25 W TDP (TB-10580-001)
_N_CORES:   int   = 8   # 8× Cortex-A78AE (DS-10662-001 Table 2); hardcoded to match
                        # the Orin NX spec regardless of the simulation host's CPU count.


def latency_to_energy_j(total_latency_us: float, wall_s: float) -> float:
    """
    Convert OrinNxCycleMeter total latency to compute energy (Joules).

    Power model (Fan, Weber & Barroso, ISCA 2007):
        P = P_idle + (TDP - P_idle) * U_eff
        E = P * wall_s

    where:
        U_eff = (total_latency_us * 1e-6) / (wall_s * N_cores)

    The frequency-scaling term (f/f_base)^1.5 equals 1.0 because gem5's
    A78 cycle counts already reflect real execution at f_base = 2.0 GHz
    and is therefore omitted.

    Parameters
    ----------
    total_latency_us : total simulated CPU work from OrinNxCycleMeter.end() [µs]
    wall_s           : wall-clock duration of the mission [s]
    """
    if wall_s <= 0.0:
        return 0.0
    p_idle   = _TDP_W * _IDLE_FRAC
    active_s = total_latency_us * 1e-6
    u_eff    = min(1.0, active_s / (wall_s * _N_CORES))
    p_avg    = p_idle + (_TDP_W - p_idle) * u_eff
    return p_avg * wall_s


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

class OrinNxCycleMeter:
    """
    Computes total compute latency (µs) for APE workloads on the NVIDIA
    Jetson Orin NX (ARM Cortex-A78AE @ 2.0 GHz), using the parallel-halt
    execution model:

      All APEs in a selector run start simultaneously.  When the selected APE
      finishes at T_sel, any still-running APE is halted.  Each APE
      contributes min(its_latency, T_sel).

      Solo modes (APE1/APE2/APE3): one APE runs, cost = that APE's full latency.

      CA parallel-halt cost when `sel` is selected:
        sum(min(_GEM5_A78_LATENCY_US[n], _GEM5_A78_LATENCY_US[sel]) for n in running)
      (not hardcoded here — see _GEM5_A78_LATENCY_US at import time, or call
      record_event() and inspect the result, to avoid this docstring
      drifting from the live model again.)

    Deliberately uses _GEM5_A78_LATENCY_US (Cortex-A78/Orin-NX gem5
    measurement), NOT the live APE_LATENCY_US (Cortex-M7-approximation,
    what nav_algorithm_T.py's budget_ms is derived from). This class'
    energy formula (latency_to_energy_j, below) is parameterized with
    real Orin NX datasheet constants (TDP=25W, N_cores=8, idle_frac) —
    those are only physically meaningful paired with Orin-NX-measured
    compute duration. This also matches a realistic deployment split:
    compute this heavy (real DWA/VFH) is the kind of workload that in
    practice runs on a Jetson-class companion computer onboard a real
    drone, while budget_ms's MCU-class timing represents the tightest
    realistic reaction-time constraint driving the CA tradeoff, not the
    hardware compute actually executes on for energy-accounting purposes
    — two distinct, separately-justified models for two different
    questions. See docs/ca_architecture_deviations.md.

    API:
        begin()
        record_event(selected: str, running: list)
        end() -> (total_latency_us: float, per_selected_us: dict[str, float])
    """

    def __init__(self) -> None:
        self._total_us: float = 0.0
        self._per_selected: dict[str, float] = {"APE1": 0.0, "APE2": 0.0, "APE3": 0.0}

    def begin(self) -> None:
        self._total_us = 0.0
        self._per_selected = {"APE1": 0.0, "APE2": 0.0, "APE3": 0.0}

    def record_event(self, selected: str, running: list) -> None:
        """
        Record one event's compute cost under the parallel-halt model.

        selected : APE whose plan was applied ("APE1", "APE2", or "APE3")
        running  : all APEs spawned for this event — ["APE1","APE2","APE3"]
                   for CA, [selected] for solo modes
        """
        t_sel = _GEM5_A78_LATENCY_US.get(selected, 0.0)
        cost  = sum(min(_GEM5_A78_LATENCY_US.get(n, 0.0), t_sel) for n in running)
        self._total_us += cost
        if selected in self._per_selected:
            self._per_selected[selected] += cost

    def end(self) -> tuple:
        return self._total_us, dict(self._per_selected)
