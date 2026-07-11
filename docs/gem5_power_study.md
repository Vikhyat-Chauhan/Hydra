# gem5 Power Study: Cycle-Accurate Validation of the Compute Power Model

> **Superseded methodology notice**: Sections 1-3 below describe the
> *original* study, which measured a **synthetic op-count workload**
> (`op_kernels.c`/`ape_profiles.c`, a hand-transcribed proxy of the old
> Python decision-math heuristics) rather than real APE algorithms. That
> synthetic workload and its Python-side audit baseline
> (`_L`/`_CYCLES`/`_ANALYTICAL_LATENCY_US`/`compare_to_analytical.py`)
> have since been **deleted outright** — see §5 below. gem5 now measures
> the REAL native APE planners (`ape1_bug_plan`/`ape2_dwa_plan`/
> `ape3_vfh_plan`, real Bug/DWA/VFH algorithms — see
> `ca_navigator/native/ape_ops/`) directly. Sections 1-3 are kept for
> historical context (the reasoning for *why* gem5 validation matters
> still applies) but their specific file references (`op_kernels.c`,
> `compare_to_analytical.py`, the "shared" profile) no longer exist in
> the codebase. Read §5 for the current methodology.

## 1. Context and motivation

### 1.1 Why this exists

`docs/compute_power_model.md` documents CANavigator's two-layer compute
power model: `OrinNxCycleMeter` (Layer 1) assigns each APE a static
per-invocation latency in microseconds, built by summing a hand-authored
table of per-operation costs (`ca_navigator/tools/orin_nx_cycle_model.py`,
the `_L` dict); `latency_to_energy_j()` (Layer 2) converts that latency into
energy via `P = P_idle + (TDP - P_idle) * U_eff`.

That model is a **closed-form analytical proxy**. It never executes a real
instruction stream, models no cache hierarchy, no DVFS, and — in the CA
"parallel-halt" selector — accounts for concurrent APE threads with a plain
`Σ min(latency_i, T_sel)` in Python, ignoring real contention for shared
cores, caches, and memory bandwidth. That's a reasonable engineering
estimate, but it is not, on its own, a *scientific measurement* — the goal
of this work is to produce results defensible enough to publish in an IEEE
CAL letter, where a reviewer's first question will be some version of "how
do you know the ARM latency table applies once compiled, scheduled, and
cached on real silicon?"

gem5 (cycle-accurate microarchitectural simulation) is the standard tool
for closing exactly that gap.

### 1.2 The three candidate approaches considered

Before any implementation, three gem5-based experimental designs were
evaluated (full writeup in the session's plan; summarized here for
context):

1. **SE-mode instruction-level gem5 + McPAT** (this document — implemented).
   Reproduce each APE's exact operation profile as a cross-compiled AArch64
   microbenchmark, run it under gem5's Syscall-Emulation mode on a CPU model
   tuned toward Cortex-A78, and feed the resulting cycle/activity stats into
   McPAT for a physically-derived power number. Lowest engineering effort,
   direct replacement of the existing latency table, and does not require
   modeling the ROS 2/threading stack.

2. **Full-system multi-core gem5 with real thread contention** (not yet
   built). Boot AArch64 Linux under gem5 FS mode with 8 A78AE-like cores
   sharing a realistic cache hierarchy, and run the actual CA parallel-halt
   scenario (3 threads racing, loser threads killed) inside a simulated OS
   scheduler. This is the one aspect the current model provably cannot
   capture — cache/memory contention among concurrently-launched APE
   threads — and would be the paper's most novel finding. Substantially
   more engineering effort (FS image, boot process, C++ port of the kernel
   logic) and much slower simulation per data point.

3. **Literature cross-validation** (not yet built). Since no physical
   Jetson Orin NX board is available for hardware-in-the-loop calibration,
   anchor gem5's credibility against independently published Cortex-A78
   CPI figures and Orin NX power numbers instead of new measurements. This
   is the validation layer a CAL reviewer will expect regardless of which
   other option is pursued.

Recommended order was 1 → 3 → 2. **Option 1 is what has been built and run
end-to-end**, described in full below.

### 1.3 What Option 1 set out to answer

Does gem5's cycle-accurate simulation of the *actual* APE arithmetic
(same op types, same counts, same code structure) on a Cortex-A78-tuned
core agree with `orin_nx_cycle_model.py`'s hand-authored latency table? And
if not, by how much, and in which direction?

---

## 2. Architecture

Everything lives under `ca_navigator/measurements/gem5_power_study/`,
untracked scratch/research infrastructure alongside the pre-existing stock
gem5 clone at `ca_navigator/measurements/gem5/`. Nothing in the main
`ca_navigator/` package was modified.

```
gem5_power_study/
├── bench/                      # AArch64 microbenchmark binary
│   ├── include/
│   │   ├── op_kernels.h
│   │   └── ape_profiles.h
│   ├── src/
│   │   ├── op_kernels.c
│   │   ├── ape_profiles.c
│   │   └── main.c
│   ├── Makefile
│   └── build/ape_bench          # cross-compiled static AArch64 ELF
├── configs/                     # gem5 SE-mode Python config
│   ├── cortex_a78_o3.py
│   ├── caches.py
│   ├── se_run.py
│   └── mcpat_base_template.xml
├── mcpat/                       # McPAT power tool (git clone, patched build)
├── scripts/
│   ├── gem5_to_mcpat.py
│   └── compare_to_analytical.py
└── results/
    ├── shared/ape1/ape2/ape3/   # gem5 stats.txt per profile
    ├── mcpat_xml/                # generated McPAT inputs
    ├── mcpat_out/                # McPAT power reports
    └── FINDINGS.md
```

### 2.1 Layer 1 — the workload (`bench/`)

**Design principle:** the instruction mix the CPU actually executes must be
traceable line-by-line back to `orin_nx_cycle_model.py`'s `(op_type,
count)` tables, so the gem5 comparison isn't against a strawman.

- **`op_kernels.c`** — one function per primitive op type (`add`, `sub`,
  `mul`, `mod`, `div`, `compare`, `shift`, `bitwise`, `assign`). Each runs
  `count` iterations pulling operands from a `volatile` 8-entry pool.
  `volatile` is the load-bearing detail: it defeats GCC's constant-folding
  and dead-code elimination without artificially serializing independent
  iterations, so the out-of-order core is free to schedule them the way it
  would in real compiled code. This is what makes the "ILP compresses
  relative APE cost gaps" finding (§3.2) valid rather than a benchmarking
  artifact.

- **`ape_profiles.c`** — transcribes `_SHARED`, `_APE1_UNIQUE`,
  `_APE2_UNIQUE`, `_CONFIDENCE`, `_APE3_UNIQUE`, and the `WINDOW_COST(n)`
  macro (mirroring `_window_cost()`) as static tables of `{op_type, count}`
  structs. A full APE invocation is composed at runtime as "shared kernel,
  then unique kernel" via `op_profile_apply_all()` — exactly mirroring how
  `orin_nx_cycle_model.py` builds
  `APE_LATENCY_US["APE1"] = _SHARED_US + latency_us(_APE1_UNIQUE)`.

- **`main.c`** — CLI entry: `ape_bench <shared|ape1|ape2|ape3> <iterations>`.
  Brackets the invocation loop with gem5's `m5_reset_stats()` /
  `m5_dump_stats()` pseudo-instructions (linked against gem5's own arm64
  `m5op.S`, no dependency on the separate `m5` CLI tool), so `stats.txt`
  contains a clean stat block for *just* the workload, with process/loader
  startup excluded. This is what makes the extracted cycle counts precise
  rather than noisy.

- **Build**: cross-compiled statically —
  `aarch64-linux-gnu-gcc -O2 -static` — linked directly against
  `ca_navigator/measurements/gem5/util/m5/src/abi/arm64/m5op.S`.

### 2.2 Layer 2 — the simulated hardware (`configs/`)

**Design principle:** reuse gem5's own validated functional-unit latencies
where they already match the ARM datasheet this repo cites, and be
explicit — in code comments — about which parameters are estimates versus
sourced values, rather than presenting estimates as fact.

- **`cortex_a78_o3.py`** — an `ArmO3CPU` subclass.
  - Functional-unit latencies (`IntAlu` = 1 cycle, `IntMult` = 3,
    `IntDiv` = 12) are gem5's stock `O3_ARM_v7a` values, kept
    **unmodified** because they already equal the ARM Cortex-A78 Software
    Optimization Guide numbers (`ARM 103-0101`) that
    `orin_nx_cycle_model.py` itself cites. This holds the per-op latency
    assumptions constant between the two models — only the *scheduling*
    (issue width, ROB, queues) differs, which is the variable this study
    is actually testing.
  - Width/queue parameters — 4-wide fetch/decode, 6-wide rename, 8-wide
    issue, 160-entry ROB, ~13 execution ports — are widened from gem5's
    default A9-class template toward A78, sourced from public secondary
    reporting (WikiChip, a UIUC CS433 course writeup), since ARM has not
    published exact queue sizes for A78. Flagged inline as a modeling
    estimate, not a datasheet fact — treated as a sensitivity parameter for
    Option 3's future literature cross-check.
- **`caches.py`** — private L1I (32 KiB) / L1D (64 KiB) / L2 (512 KiB).
  L2 size is explicitly commented as a modeler's estimate: NVIDIA does not
  publish per-core Orin NX cache sizes.
- **`se_run.py`** — assembles the gem5 `System`: 2.0 GHz clock (from the
  Jetson Orin NX datasheet, DS-10662-001, already cited in
  `compute_power_model.md`), `LPDDR5_5500_1x16_BG_BL32` DRAM model
  (matches Orin NX's real memory type), wires CPU → L1 → L2 bus → L2 →
  membus → DRAM, and loads `ape_bench` as an SE-mode `Process` with
  `argv = [profile, iterations]`.

### 2.3 Layer 3 — power (McPAT)

- **`mcpat/`** — upstream McPAT 1.3, git-cloned fresh. Its makefile
  hardcoded a 32-bit (`-m32`) build, which fails on this machine's glibc
  (missing 32-bit multilib headers, `bits/wordsize.h`). Patched to build
  64-bit instead — a well-known, safe workaround with no functional effect
  on McPAT's power model.
- **`mcpat_base_template.xml`** — a copy of McPAT's own bundled
  `ARM_A9_2GHz.xml`, used as the structural skeleton rather than
  hand-building XML from scratch. McPAT's parser expects components in a
  specific positional order tied to its `homogeneous_*` flags (confirmed
  the hard way — a hand-built XML produced parser errors); patching values
  into a known-working file is the safer path.
- **`gem5_to_mcpat.py`** — reads the first (ROI) stat block from
  `stats.txt`, maps real gem5 25.1.0.1 stat names — verified by grepping an
  actual run, not guessed — (`numCycles`, `commitStats0.numInsts`,
  `rob.reads/writes`, `rename.lookups`, `intAluAccesses`, cache
  `overallAccesses`/`overallMisses`, etc.) onto the McPAT template via
  **occurrence-indexed regex substitution**: a helper locates the Nth match
  of a given `<stat name=...>` tag by document position, since several
  components reuse stat names (e.g. `read_accesses` appears once each for
  icache, dcache, BTB, ...). `number_of_L1Directories` / `L2Directories` /
  `L3s` / `NoCs` are set to 0 so McPAT's own "skip if count is 0" logic
  zeroes out structural placeholders this single-core, non-coherent
  workload has no real data for. Tech node is set to 22 nm — the closest
  node McPAT's built-in device models support to Orin NX's actual 8 nm
  (Samsung 8LPP) process — flagged as a caveat affecting *absolute power
  only*, not the cycle-count/latency results (which come from gem5 alone).

### 2.4 Layer 4 — the comparison (`scripts/compare_to_analytical.py`)

Imports `orin_nx_cycle_model.py` **live** (not copy-pasted numbers) so the
comparison can never silently drift out of sync with the model it's
checking against. Computes gem5 cycles/µs per invocation and gem5+McPAT
energy per invocation, and prints both against the analytical model's own
numbers. Every number in `results/FINDINGS.md` is regenerable by rerunning
this script — nothing in it is hand-typed.

### 2.5 End-to-end data flow

```
orin_nx_cycle_model.py op-profiles
        │ (transcribed by hand, auditable line-by-line)
        ▼
bench/*.c  →  aarch64-linux-gnu-gcc  →  ape_bench (static ELF)
        │
        ▼
gem5.opt configs/se_run.py <profile> <iters>   (CortexA78O3CPU + caches + LPDDR5)
        │
        ▼
results/<profile>/stats.txt   (ROI block via m5_reset_stats/m5_dump_stats)
        │
        ├──► compare_to_analytical.py ──► latency table
        │
        ▼
scripts/gem5_to_mcpat.py  →  results/mcpat_xml/<profile>.xml
        │
        ▼
mcpat/mcpat  →  results/mcpat_out/<profile>.txt  (Runtime Dynamic + Leakage)
        │
        ▼
compare_to_analytical.py ──► energy table
        │
        ▼
results/FINDINGS.md
```

### 2.6 How to reproduce

```bash
# 1. Build the microbenchmark (requires gcc-aarch64-linux-gnu / g++-aarch64-linux-gnu)
cd ca_navigator/measurements/gem5_power_study/bench && make

# 2. Run gem5 SE-mode for each profile
GEM5=../../gem5/build/ALL/gem5.opt
for p in shared ape1 ape2 ape3; do
  $GEM5 --outdir=../results/$p ../configs/se_run.py $p 200
done

# 3. Translate stats to McPAT XML and run McPAT
cd ..
for p in shared ape1 ape2 ape3; do
  python3 scripts/gem5_to_mcpat.py results/$p/stats.txt results/mcpat_xml/$p.xml
  mcpat/mcpat -infile results/mcpat_xml/$p.xml -print_level 5 -opt_for_clk 1 \
    > results/mcpat_out/$p.txt
done

# 4. Compare gem5 against the hand-derived analytical baseline
python3 scripts/compare_to_analytical.py

# 5. Freeze these gem5 cycle counts as the live simulator's source of truth
#    (writes ca_navigator/tools/gem5_measured_latencies.py — commit the
#    result). Step 4 compares gem5 to the analytical estimate; this step
#    is what actually makes gem5's numbers reach the ROS 2 navigator.
python3 scripts/freeze_measured_latencies.py
```

As of this study, step 5's frozen gem5 output was `ca_navigator/tools/
orin_nx_cycle_model.py`'s public `APE_LATENCY_US` — the number
`nav_algorithm_T.py` actually uses to size APE planning budgets — instead
of the hand-summed op-count table. The hand-summed table still exists
internally as `_ANALYTICAL_LATENCY_US`, kept only as the audit baseline
step 4 validates gem5 against (see §3.1: gem5 measures ~5-6x *higher*
latency than the hand-summed table once real load/branch/loop overhead is
counted, which the abstract op-count sum never charged for). This was as
close as the live drone simulation could get to "running the APEs on real
hardware" — gem5 can't execute inside the real-time ROS 2 loop itself
(cycle-accurate simulation is many orders of magnitude slower than real
time), but because each APE's op profile is static regardless of live
sensor input, a one-time offline gem5 measurement is the complete and
permanently valid answer for every future invocation. See
docs/ca_architecture_deviations.md for the full reasoning.

**Superseded by §2.7 below**: `APE_LATENCY_US` is no longer sourced from
this Cortex-A78 study — it's now sourced from a second gem5 study
targeting an MCU-class core (real ArduPilot flight controllers run on
Cortex-M-class MCUs, not Cortex-A78-class companion computers). This
study's numbers are kept as `_GEM5_A78_LATENCY_US`, for comparison only.

### 2.7 The MCU-approximation extension (Cortex-M7)

A second gem5 study, `configs/cortex_m7_inorder.py` + `configs/
se_run_mcu.py` + `bench/Makefile.mcu`, targets an in-order MinorCPU
config approximating Cortex-M7 (~400 MHz, single-issue, gem5's stock
`MinorDefaultFUPool`) running the *same* op profiles cross-compiled to
32-bit ARM/Thumb. **This is an explicit approximation, not a true
M-profile simulation** — gem5 has no Cortex-M/M-profile CPU model at all;
see `cortex_m7_inorder.py`'s module docstring for the full list of
caveats (coarse per-opClass FU timings borrowed from gem5's own default
pool rather than hand-tuned per-instruction; TCM approximated as L1-only
cache; single-issue pipeline width).

Regenerate via `scripts/freeze_measured_latencies_mcu.py` (mirrors
`freeze_measured_latencies.py`, reading `results_mcu/` instead of
`results/`), writing `ca_navigator/tools/gem5_measured_latencies_mcu.py`.
This is what `orin_nx_cycle_model.py`'s `APE_LATENCY_US` — and therefore
`nav_algorithm_T.py`'s `budget_ms` — is sourced from now.

Measured cycle counts (200 iterations each, same methodology as §2.6):

| profile | A78 cycles/inv | A78 µs/inv | MCU-approx cycles/inv | MCU-approx µs/inv (@400MHz) |
|---|---:|---:|---:|---:|
| shared | 1265.8 | 0.633 | 5747.9 | 14.37 |
| APE1   | 1380.7 | 0.690 | 6356.0 | 15.89 |
| APE2   | 1492.4 | 0.746 | 6720.9 | 16.80 |
| APE3   | 4510.8 | 2.255 | 23430.6 | 58.58 |

The combined effect of ~4.5x more cycles (in-order, no ILP) at a 5x
slower clock puts MCU-approximation latency ~20-26x higher than the A78
numbers — moving `budget_ms` from sub-millisecond (0.69-2.26ms) to
tens-of-milliseconds (15.9-58.6ms). That's still roughly an order of
magnitude smaller than `deadline_min_s` (700ms, `ca_navigator/config.py`),
down from 300-6000x smaller pre-rewire — a real, hardware-grounded
improvement, though whether `DEADLINE_SCALE=1000` still needs adjustment
on top of these larger numbers is unverified and explicitly flagged (not
resolved) in `orin_nx_cycle_model.py`'s `DEADLINE_SCALE` comment.

Also fixed alongside this rewire (both studies rebuilt from the same
corrected source): `ca_navigator/native/ape_ops/src/ape_profiles.c`'s
`APE2_UNIQUE_OPS` had two phantom `_window_cost(_N_FRONT)` scans not
present in the current `_evt_plan_ape2` (a leftover from an earlier
version of that function) — removed; and `_stopping_limited_speed`'s
`math.sqrt()` call, previously modeled as an undocumented `div(2)`
approximation, now has a real `OP_SQRT` op kernel. See
`orin_nx_cycle_model.py`'s inline `CORRECTED` comments for both.

---

## 3. Findings

Full detail in `ca_navigator/measurements/gem5_power_study/results/FINDINGS.md`;
summarized here.

### 3.1 Latency: gem5 is ~900–1200× faster than the model claims

| profile | gem5 cycles/inv | gem5 µs/inv | model µs (live code) | ratio |
|---|---:|---:|---:|---:|
| shared | 1034.0 | 0.5170 | 511.10  | 989×  |
| APE1   | 1161.5 | 0.5807 | 523.48  | 901×  |
| APE2   | 2289.4 | 1.1447 | 1342.55 | 1173× |
| APE3   | 4010.6 | 2.0053 | 2034.81 | 1015× |

That gap is strikingly *consistent* across all four profiles rather than
varying per-operation-mix — which points to a systematic derivation/unit
issue rather than per-op modeling error. Suggestively, `nav_algorithm_T.py`
separately documents a `DEADLINE_SCALE` of exactly **1000×**, described as
"1µs native Cortex-A78AE compute → 1ms effective budget under Python
interpreter ~100× + system contention ~10× on Orin NX." The gem5-measured
gap lands almost exactly on that same factor, suggesting the `_L` table's
"native compute" values may already carry interpreter-level inflation
before `DEADLINE_SCALE` is applied on top — a real, independently
reportable bug candidate, not just noise.

A second, smaller issue found in passing: `docs/compute_power_model.md`'s
published table (540/1729/3004 µs; shared = 233.64 µs) does not match what
`orin_nx_cycle_model.py` computes today (523.48/1342.55/2034.81 µs;
shared = 511.10 µs) — the docs appear stale relative to the code,
independent of anything gem5-related.

### 3.2 Relative cost shape: the model assumes serial cost; hardware pipelines

|  | model APE2/APE1 | model APE3/APE1 | gem5 APE2/APE1 | gem5 APE3/APE1 |
|---|---:|---:|---:|---:|
| ratio | 2.56× | 3.89× | 1.97× | 3.45× |

The model treats every operation as strictly additive latency. A real
out-of-order core executes independent operations (e.g. the window-scan
loop's index arithmetic) across multiple ports in parallel, so heavier
kernels benefit disproportionately from ILP — the real gap between "light"
and "heavy" APEs is smaller than the model assumes. This directly affects
the CA selector's threshold tuning, which is calibrated against the
model's inflated, disproportionate latency gaps.

### 3.3 Energy: two compounding, independently explainable overestimates

| profile | gem5+McPAT µJ/inv | model µJ/inv (U=1 ceiling) | ratio |
|---|---:|---:|---:|
| shared | 0.2019 | 12777.50 | 63298× |
| APE1   | 0.2245 | 13087.00 | 58296× |
| APE2   | 0.4474 | 33563.75 | 75016× |
| APE3   | 0.7724 | 50870.25 | 65859× |

(The model's energy function is defined over a whole mission's wall-clock
time, so there's no native "energy of one invocation" to read off directly;
the comparison uses the model's own implied ceiling — a single synchronous
invocation at `U_eff = 1`, i.e. `P_avg = TDP = 25 W` for that invocation's
duration — as the only apples-to-apples basis.)

1. The ~900–1200× latency gap (§3.1) propagates directly into energy, since
   E ∝ latency at fixed power.
2. The model attributes the **full 25 W module TDP** — meant for all 8
   cores, GPU, and memory controllers under MAXN full load — to a single
   core doing a few thousand scalar integer ops. McPAT's structural
   estimate for one core's actual dynamic+leakage power on this workload is
   ~0.27–0.39 W, roughly two orders of magnitude below module TDP.

### 3.4 Caveats to carry into the paper

- **ROB size (160 entries)** and **private L2 size (512 KiB)** are modeler
  estimates, not datasheet values — flagged inline in
  `configs/cortex_a78_o3.py` / `configs/caches.py`. Option 3 (literature
  CPI/power cross-check) is the intended way to bound their sensitivity.
- **McPAT tech node (22 nm vs. Orin NX's real 8 nm)** affects absolute
  power numbers only; the cycle-count/latency findings do not depend on it.
- The microbenchmark kernels read a small `volatile` operand pool per
  iteration, adding a memory access the original op-cost table doesn't
  explicitly charge for. L1 hit rate is >99.9% in every run, so this is
  small relative to the ~1000× gap found, but not zero.
- This is **single-thread, solo-kernel** SE-mode simulation. It does not
  model the CA parallel-halt scenario's real cache/memory contention among
  three concurrently-launched APE threads — that is Option 2's job, not
  this one's.

---

## 4. Status and next steps

Option 1 is complete and fully reproducible. Not yet started: Option 3
(literature CPI/power cross-validation, cheap to add on top of this) and
Option 2 (full-system multi-core gem5 with real thread contention, the
most novel but most expensive extension). The `DEADLINE_SCALE` /
interpreter-inflation finding in §3.1 is also independently actionable as a
codebase fix, separate from anything destined for the paper.

---

## 5. Current methodology: gem5 measures the real APE algorithms directly

The synthetic op-count workload described in §1-3 was always a proxy —
a hand-transcribed approximation of what the Python decision logic did,
kept in sync by hand. That transcription drifted twice (a stale APE2
profile with two phantom scans that didn't exist in the real code, and
an undocumented `sqrt≈div×2` approximation) before this rewrite. The
APE algorithms themselves were also simplified heuristics, not the kind
of algorithm the Conflict Architecture paper describes (APE3 was
supposed to be a "Map Search algorithm based on techniques used for
autonomous maze-solving robots" — the old code was a confidence-weighted
sidestep, not a search).

Both problems are fixed the same way: **the APE algorithms are now real,
literature-standard implementations, written directly in C, and gem5
measures them — not a separate proxy.**

### 5.1 What each APE actually is now

- **APE1** (`ca_navigator/native/ape_ops/src/ape1_bug.c`) — a minimal
  reactive potential-field/Bug-style heading controller. Closed-form, one
  forward sector + two side sectors, no candidate search — deliberately
  the cheapest tier, matching the paper's "fast but least accurate" role.
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
  that selects and scores the best candidate heading. This is the "search
  over a local structure" step intended to restore the paper's Map-Search
  framing for APE3.

Both DWA and VFH always walk their full fixed-size grid/histogram
unconditionally — no early exit on scan content — which keeps their op
count independent of live sensor data, preserving the "one offline gem5
measurement is valid forever" property the original synthetic study
relied on (see each file's own "op-count discipline" docstring note).

`ca_navigator/tools/ape_native.py` is the only bridge between Python and
these planners — Python's role in the APE path is now just marshaling
(raw scan + config → a ctypes struct → the native call → unpack the
result), no algorithm logic. See `docs/ca_architecture_deviations.md`
for the full architectural reasoning.

### 5.2 Multi-layer LiDAR

APE3's multi-layer input required extending the simulated LiDAR
(`models/x3-uav/4/model.sdf`, `vertical.samples` 1→5) and a new sensor
path: `sensor_msgs/LaserScan` is structurally single-plane (no vertical
dimension at all), and `ros_gz_bridge`'s `LaserScan` converter was
confirmed empirically (before building anything) to silently truncate a
multi-layer `gz.msgs.LaserScan` down to one layer with **no warning** —
not a documented limitation, discovered by direct inspection
(`gz topic -e`/`ros2 topic echo` on both the raw Gazebo topic and the
bridged ROS 2 topic). Multi-layer data instead goes through the sensor's
native point-cloud output (`gz.msgs.PointCloudPacked` →
`sensor_msgs/PointCloud2`, which the bridge does support correctly —
also verified empirically: `height`/`width` on the bridged topic matched
the sensor's real layer/sample counts exactly). APE1/APE2 and the base
(non-event) avoidance path are unaffected — they still use the original
single-layer `LaserScan` topic exactly as before.

### 5.3 Reproducing the current study

```bash
# 1. Build the real planners for gem5 (both targets share the same
#    native source — see ca_navigator/native/ape_ops/)
cd ca_navigator/measurements/gem5_power_study/bench
make            # AArch64 / Cortex-A78 study
make -f Makefile.mcu   # 32-bit ARM/Thumb / Cortex-M7-approximation study

# 2. Run gem5 SE-mode for each profile, each study
GEM5=../../gem5/build/ALL/gem5.opt
cd ..
for p in ape1 ape2 ape3; do
  $GEM5 --outdir=results/$p configs/se_run.py $p 200
  $GEM5 --outdir=results_mcu/$p configs/se_run_mcu.py $p 200
done
# Note: ape2/ape3 are substantially heavier real workloads than the old
# synthetic profiles (see §5.4) — the MCU/ape3 run in particular can take
# several minutes of gem5 host wall-clock time, not seconds.

# 3. Freeze both studies' results
python3 scripts/freeze_measured_latencies.py
python3 scripts/freeze_measured_latencies_mcu.py
```

Each profile now runs against one fixed, open-corridor scan fixture
defined directly in `bench/src/main.c` (not a CLI-selected named
profile table — there's no more "shared" profile, since each planner
call is fully self-contained).

### 5.4 Measured results (real algorithms)

| profile | A78 cycles/inv | A78 µs/inv | MCU-approx cycles/inv | MCU-approx µs/inv (@400MHz) |
|---|---:|---:|---:|---:|
| APE1 (Bug)  | 2,623    | 1.31   | 11,635    | 29.1   |
| APE2 (DWA)  | 96,071   | 48.04  | 286,389   | 716.0  |
| APE3 (VFH)  | 223,993  | 112.00 | 502,900   | 1,257.3 |

Compare to the old synthetic-workload numbers (§3.1): a modest ~3.3×
spread between APE1 and APE3. Real DWA/VFH produce an **85× spread**
(A78) — genuinely differentiated compute cost driven by real algorithmic
work (candidate-grid forward simulation, multi-layer histogram + valley
search), not injected busywork. On the MCU-approximation target, this
puts `budget_ms` (`= APE_LATENCY_US × DEADLINE_SCALE / 1000`) at
roughly **29ms / 716ms / 1257ms** — for the first time, genuinely
comparable in magnitude to real physical event deadlines
(`deadline_min_s`/`deadline_max_s` in `ca_navigator/config.py`, 147ms-
3500ms), rather than three-to-six orders of magnitude smaller. Whether
this is enough to produce genuine APE1/APE2 selections under `CA` mode
in practice (versus APE3 still always winning) hasn't been re-measured
with a live run since this change — that's the natural next check.
