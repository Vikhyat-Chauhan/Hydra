"""
gem5_measured_latencies_mcu.py — GENERATED FILE, do not edit by hand.

Regenerate with:
    cd ca_navigator/measurements/gem5_power_study
    python3 scripts/freeze_measured_latencies_mcu.py

Raw per-invocation cycle counts from gem5 cycle-accurate simulation
using CortexM7ApproxMinorCPU (see configs/cortex_m7_inorder.py — an
in-order MinorCPU APPROXIMATION of Cortex-M7, NOT a true M-profile
simulation; gem5 has no M-profile CPU model) of the REAL native
APE planners (ape1_bug_plan/ape2_dwa_plan/ape3_vfh_plan — see
ca_navigator/native/ape_ops/), not a synthetic op-count proxy.
Read directly from each profile's results_mcu/<profile>/stats.txt
ROI block (see bench/src/main.c for the m5_reset_stats/
m5_dump_stats bracketing, against one fixed open-corridor scan
fixture).

See docs/gem5_power_study.md for the full methodology and
configs/cortex_m7_inorder.py for the approximation's caveats.
"""

ITERATIONS = 200
CPU_CLOCK_HZ = 400000000.0  # cortex_m7_inorder.py's CPU_CLOCK

# Total simulated cycles across ITERATIONS invocations, ROI-only
# (process/loader startup excluded).
TOTAL_CYCLES: dict[str, int] = {
    "ape1": 2326902,
    "ape2": 57277897,
    "ape3": 100580071,
}

