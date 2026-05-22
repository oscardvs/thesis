# sim_validation_01b — wall_num_thresh threshold sweep

Follow-up to `sim_validation_01/joint_sweep` (ADR 0006 §F2). Yesterday's corrected sweep ([`joint_sweep__04a2e8802fc1_d57b109/sweep_summary.md`](../../results/sim_validation_01/joint_sweep__04a2e8802fc1_d57b109/sweep_summary.md)) established that the framework's count-thresholded suppression rule contributes at production `wall_num_thresh=20`, but the magnitude is small at production cell size (`Δ_b = −4.3 mm` at 0.10 m). The rule's contribution grows with cell coarseness up to 0.30 m, where the rule also starts hurting `RMSE_core` (+18 mm). The sweet spot of the cell-size sweep is 0.15–0.20 m where both boundary and core improve.

This experiment asks the next question: at the sweet-spot cell size, what `wall_num_thresh` maximises the suppression rule's contribution to `RMSE_boundary` without inflating `RMSE_core`?

## Matrix design (5 runs)

```yaml
sweep:
  matrix:
    cell_size: [0.15]
    wall_num_thresh: [5, 10, 20, 50]
    enable_edge_sharpen: [true]
  extra_runs:
    - {cell_size: 0.15, wall_num_thresh: 20, enable_edge_sharpen: false}
```

4 product runs + 1 extra F-baseline = 5 runs total. ~9 min/run × 5 ≈ 45 min wall time.

| label              | cell | wnt | es | role                               |
|--------------------|------|-----|----|------------------------------------|
| `cs015_wnt005_es1` | 0.15 | 5   | T  | rule fires most freely             |
| `cs015_wnt010_es1` | 0.15 | 10  | T  | half of production                 |
| `cs015_wnt020_es1` | 0.15 | 20  | T  | production threshold               |
| `cs015_wnt050_es1` | 0.15 | 50  | T  | rule fires less freely             |
| `cs015_wnt020_es0` | 0.15 | 20  | F  | baseline + reproducibility check   |

## Why 0.15 m, not 0.20 m

Yesterday's cell-size sweep showed near-identical Δ at the two sweet-spot cells (−6.3 mm at 0.15 m, −6.0 mm at 0.20 m), so on the boundary metric they are indistinguishable. The choice between them is conservative-vs-aggressive:

- **0.15 m** sits closer to production (0.10 m), so a recommendation made here generalises more cleanly to deployment cell size. The points-per-cell-per-scan baseline is the natural reference for the threshold-vs-density argument in 01's z-inversion theory.
- **0.20 m** would give ~78% more points-per-cell-per-scan (cell area scales quadratically: 0.04 m² vs 0.0225 m² = 1.78×) and may show a sharper Δ-vs-threshold curve, but it sits further from production and so the operating point may not translate directly back.

0.15 m is the conservative end of the sweet spot. If the run at 0.15 m produces an ambiguous threshold ranking (no clear monotonic trend, or RMSE_core trade-off shapes nothing), `sim_validation_01b_v2` at 0.20 m is the natural follow-up.

## Why only one F baseline

The `enable_edge_sharpen=false` kernel branch at `custom_kernels.py:183` short-circuits the suppression conjunct at the flag, so `wall_num_thresh` has no kernel-level effect when the flag is off. Running F at every threshold would produce four identical results. One F at the production threshold is enough to (a) confirm threshold-independence empirically, and (b) reproduce yesterday's archived `cs015_es0` as a same-day reproducibility check on TF buffer timing + GPU scheduling determinism. See `acceptance.reproducibility_tolerance_m` in `threshold_sweep.yaml` (5 mm; failure ⇒ STOP and investigate).

## Falsifiable prediction

`Δ_b(T−F)` should be monotonic in threshold across the range:

- **Maximally negative at `wnt=5`** — the rule fires on most cells (the threshold is below typical points-per-cell-per-scan even on ceiling returns), capturing the lowest-overhead surface broadly.
- **Diminishing toward zero at `wnt=50`** — the rule barely fires at this density; behaviour approaches the F baseline.

`RMSE_core` trade-off shape:

- **At `wnt=5`** the rule fires aggressively enough on flat interior cells that it drops valid returns and inflates core RMSE.
- **At `wnt=50`** core RMSE matches F (the rule has no kernel-visible effect when it doesn't fire).

The operating point is the largest threshold at which `Δ_b` is still meaningfully negative while `RMSE_core` stays within `acceptance.core_tolerance_m` (5 mm) of the F baseline. The argmin of `RMSE_b(T)` under that constraint is the recommendation.

**Falsification.** Non-monotonic `Δ_b` or flat `RMSE_core` across thresholds would falsify the mean-vs-min argument in 01's z-inversion theory section: either the rule fires at the same rate regardless of threshold (mechanistically unlikely; the kernel gates on `num_points > thresh` directly), or some other mechanism dominates the layer's behaviour.

## Sim world

Reused from sim_validation_01 — `construction_site.world`. Same feature mix (base flat ceiling, hanging pipes, ceiling lamps, lowered ceiling sections, slab with holes). The same bag and same world ensures clean comparability against yesterday's archived sweep.

## Trajectory

Reused bag at `~/ros2_ws/data/sim_validation_01/bag_2026-05-21_construction_site`. 514 s, same trajectory as sim_validation_01.

## Outputs

Per run (same as sim_validation_01):
- `timing_floor.csv`, `timing_ceiling.csv` — per-callback latency (existing instrumentation).
- `metrics_diagnostic.json` — RMSE/MAE/max/P95/bias/coverage from `compute_ceiling_metrics.py`.
- `summary.json` — runner-collected metrics + `run_overrides` (carries cell_size, wall_num_thresh, enable_edge_sharpen).

Per sweep:
- `sweep_summary.csv` / `sweep_summary.md` — runner-emitted per-run table.
- `sweep_summary.md` is then **appended to** by `sim_validation_01b_postprocess.py` with a reproducibility check (F-baseline vs archived `cs015_es0`) and a threshold-ranking table with the operating-point recommendation.

## Read-out

After the sweep finishes, run:

```bash
python3 experiments/runners/sim_validation_01b_postprocess.py \
    experiments/results/sim_validation_01b/threshold_sweep__<cfg_sha>_<git_sha>
```

The recommendation is computed as:

```
threshold_star = argmin over T-runs of RMSE_boundary(T)
                 subject to RMSE_core(T) ≤ RMSE_core(F) + ε
```

where ε = `acceptance.core_tolerance_m` (5 mm by default). If no threshold satisfies the constraint, the recommendation is "stay at production 20" with rationale.

## Where to run

Desktop is fine for the threshold question — same density-driven reasoning as `sim_validation_01/README.md`. The latency calibration to Jetson lives in the existing benchmark harness, not this experiment.

## Out of scope

- The +0.045 m residual bias from yesterday's F1+F2 fix — framework-side investigation deferred until controller-side validation needs absolute accuracy.
- Splitter `z_low` lowering — depends on what this run reveals about upper-band ceiling features; separate ADR if pursued.
- 0.20 m cell-size variant — natural follow-up `sim_validation_01b_v2` if the 0.15 m result is ambiguous.
