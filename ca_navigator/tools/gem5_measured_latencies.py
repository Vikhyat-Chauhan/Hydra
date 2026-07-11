"""
gem5_measured_latencies.py — GENERATED FILE, do not edit by hand.

Regenerate with:
    cd ca_navigator/measurements/gem5_power_study
    python3 scripts/freeze_measured_latencies.py

Raw per-invocation cycle counts from gem5 cycle-accurate simulation
(ArmO3CPU tuned toward Cortex-A78, see configs/cortex_a78_o3.py) of
the REAL native APE planners (ape1_bug_plan/ape2_dwa_plan/
ape3_vfh_plan — see ca_navigator/native/ape_ops/), not a synthetic
op-count proxy. Read directly from each profile's stats.txt ROI
block (see bench/src/main.c for the m5_reset_stats/m5_dump_stats
bracketing, against one fixed open-corridor scan fixture).

See docs/gem5_power_study.md for the full methodology.
"""

ITERATIONS = 200

# Total simulated cycles across ITERATIONS invocations, ROI-only
# (process/loader startup excluded).
TOTAL_CYCLES: dict[str, int] = {
    "ape1": 524655,
    "ape2": 19214157,
    "ape3": 44798505,
}

