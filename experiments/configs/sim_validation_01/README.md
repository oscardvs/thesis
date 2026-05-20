# sim_validation_01 — dual elevation mapping validation suite

Sim-doable checks specified in `thesis/docs/01_dual_elevation_mapping.md` §Sim-phase validation suite. Six checks total. This directory holds the configs and runner for the first and most load-bearing one:

## joint_sweep — suppression firing rate × ceiling grid resolution

The architectural claim in 01's z-inversion theory is that the framework's count-thresholded suppression rule (Miki 2022 §II-C, `custom_kernels.py:183`) captures the lowest overhead surface on the negated ceiling layer. The rule requires `num_points > wall_num_thresh` per cell per scan to fire. Overhead returns are sparser than floor returns; if firing rate measures low, the ceiling Kalman fusion degenerates to averaging and the layer publishes a mean-vs-min biased height rather than the lower-bound surface the construction promises.

This sweep answers: at which ceiling cell size does the suppression rule contribute materially to reconstruction accuracy on featured ceilings?

### Matrix design

`4 cell sizes × 2 enable_edge_sharpen states = 8 runs`. The `enable_edge_sharpen` toggle gives a *direct* measure of the suppression-rule contribution per cell size without modifying the framework kernels: with the flag false, the kernel collapses to plain Kalman fusion (the bug-fix correction from 2026-05-20 — see journal); with the flag true, the suppression rule is active. The delta in reconstruction RMSE between the two states at each cell size measures how much suppression is actually doing.

| cell_size (m) | edge_sharpen | rationale |
|---------------|--------------|-----------|
| 0.10          | true / false | current production resolution baseline |
| 0.15          | true / false | mild coarsening; 2.25× cell area |
| 0.20          | true / false | the asymmetric-resolution lever from "Asymmetric configuration" |
| 0.30          | true / false | aggressive coarsening; 9× cell area |

`wall_num_thresh` is held at 100 (production default) for the entire sweep — the cell-size variation is what scales points-per-cell-per-scan. The floor instance is held at production settings (`resolution: 0.10`, all filters enabled, `enable_edge_sharpen: true`) for all runs.

### Expected pattern (the falsifiable prediction)

If suppression fires reliably at the production cell size (0.10 m), `RMSE_boundary[edge_sharpen=true] < RMSE_boundary[edge_sharpen=false]` at 0.10, with the gap *narrowing* as cell size grows (because at larger cells, even plain Kalman fusion averages over enough returns to approximate the lower envelope — though probably not exactly).

If suppression *does not* fire reliably at 0.10 m, `RMSE_boundary[edge_sharpen=true] ≈ RMSE_boundary[edge_sharpen=false]` at 0.10, with the gap *widening* as cell size grows (suppression starts firing at coarser cells with denser points-per-cell). The crossover point — where the on/off delta becomes large — is the operating-point recommendation, and the implication is that the production cell size needs coarsening (or `wall_num_thresh` needs lowering, follow-up experiment).

### Sim world

`hilda_gazebo/worlds/construction_site.world` — already has the feature mix needed: base flat ceiling, hanging pipes, ceiling lamps, lowered ceiling sections, slab with holes. Provides both flat regions (where suppression is irrelevant, `mean ≈ min`) and featured regions (where suppression matters, `mean ≠ min`). The `compute_ceiling_metrics.py` script partitions cells into `core` (interior of surfaces) and `boundary` (edges, transitions, near-feature) — the latter is exactly where the suppression contribution should show.

### Trajectory

A 90-second recorded bag of the robot driving past the varied ceiling features in `construction_site.world`. Recorded once and replayed for all 8 runs to ensure identical input data — this is the existing benchmark-harness pattern (`benchmark_record.sh`). The recording covers (a) approach to the lowered ceiling section, (b) drive under hanging pipes at varied lateral offsets, (c) pass-through under ceiling lamps, (d) pass-near slab-with-holes section, (e) return to a flat-ceiling section as a control baseline.

### Outputs

Per run:
- `timing_floor.csv`, `timing_ceiling.csv` — per-callback latency breakdown (existing instrumentation)
- `tegrastats.csv` — system metrics (GPU %, RAM, temp, power); skipped on desktop
- `accuracy_core.json`, `accuracy_boundary.json` — RMSE/MAE/max/P50/P90/P95/bias/coverage from `compute_ceiling_metrics.py`
- `config.yaml` — exact per-instance configuration used (for provenance)

Summary across 8 runs:
- `sweep_summary.csv` — one row per (cell_size, edge_sharpen) combination with the metrics above
- `sweep_summary.md` — rendered table with the expected-pattern interpretation

### Where to run

Desktop is fine for the firing-rate question — it depends on point density per cell per scan, not compute hardware. Latency and memory numbers will be desktop-class, not Jetson-class; surface in the summary that these are desktop figures and the latency calibration to Jetson lives in the existing benchmark harness (Phase 8 work).

### Open decisions for the user

Before running, three things to confirm:

1. **Bag recording.** Need the 90-second bag from `construction_site.world`. Either record fresh with `benchmark_record.sh 90` while you drive the robot interactively, or reuse the existing `benchmark_bag_20260217_093755` if it covers similar terrain. (Recommend recording fresh for trajectory control over the featured ceiling sections; the existing bag was for a throughput benchmark, not feature coverage.)
2. **Floor instance lock.** The matrix holds the floor at production settings. Confirm this is the right baseline — if floor and ceiling configs should sweep jointly (asymmetric resolution = floor 0.10, ceiling varies; vs symmetric resolution = both vary together), the matrix doubles. Recommend asymmetric for the first pass — the question is specifically about the ceiling instance.
3. **Where to run.** Desktop (this machine) is recommended for iteration speed. The findings transfer to Jetson because the suppression-firing question is density-driven, not compute-driven. The Jetson-specific latency calibration stays in the benchmark harness, not this experiment.
