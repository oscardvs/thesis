# sim_validation_01d — time_variance sweep

Corroborating experiment for ADR 0009. 01c found that the framework's
published per-cell σ² grows as 12× when α_d grows 100×; the diagnostic
identified the mechanism as a √-floor between Kalman draw-down and the
constant `time_variance × update_variance_fps` inflation. This sweep
varies `time_variance` directly and checks whether σ² tracks
√(time_variance), which is the falsifiable prediction of the
diagnosis.

## Falsifiable prediction

Holding α_d at 0.05 (production), z ≈ 3 m, and observation cadence
Δt ≈ 0.4 s, ADR §F2 derives the steady-state expression

    σ²_published ≈ √(σ²_meas · time_variance · update_variance_fps · Δt)

so σ² should scale as √(time_variance). Predicted values (from the ADR's
quantitative reconstruction):

| time_variance | predicted σ² (post-init mean) |
|--------------:|------------------------------:|
| 0             | → 0 (draw-down only)          |
| 0.0001        | ≈ 0.010                       |
| 0.001         | ≈ 0.030                       |
| 0.01          | ≈ 0.095                       |

The three non-zero cells should sit on a √-line: each 10× step in
time_variance gives a √10 ≈ 3.16× step in σ². The pure-zero cell tests
the limit where the floor vanishes — variance state draws down across the
bag with no inflation to balance it.

## Falsification

- If σ² is flat across time_variance (or grows linearly) → the √-floor
  mechanism is not the dominant contributor and the ADR's F2 is wrong.
  02's calibration design waits until the actual mechanism is named.
- If σ² grows faster than √(time_variance) → there's a coupling the
  algebra missed (e.g. outlier-rejection sensitivity to map_v, since the
  Mahalanobis gate scales with map_v not √map_v).
- If the time_variance=0 cell publishes σ² ≈ time_variance=0.0001 cell's
  σ² → inflation is not what's setting the floor; some other α_d-
  independent source is.

## Operating point

- Bag: shared persistent recording from 01 / 01b / 01c.
- α_d = 0.05 (production, settled by 01c — and the snf050 reproducibility
  point that anchors the variance baseline).
- cell_size = 0.10, wnt = 20, es = true (production).

The time_variance = 0.0001 cell is the in-sweep reproducibility check
against 01c's snf050 cell — same params + same bag + same code → sub-mm
agreement on both elevation RMSE and post-init mean σ².

## Read-out

`sim_validation_01d_postprocess.py` reports four numbers:

1. **Reproducibility** (tv=0.0001 vs archived 01c snf050). PASS = both
   RMSE_all and RMSE_core within 5 mm; σ² post-init mean within
   ~sub-percent of the archived value.
2. **√-scaling on the three non-zero cells.** Three pairwise ratios
   checked against the √-prediction with ±50% tolerance. The wide
   tolerance reflects that the prediction is order-of-magnitude and the
   ratio depends on within-bag cell observation cadence.
3. **Zero-inflation behaviour.** σ² at time_variance=0 should be
   substantially smaller than at 0.0001 (one-sided check, factor ≤ 0.5).
4. **Elevation residual stability.** RMSE_core spread across the sweep
   should be < 5 mm (time_variance changes the variance state, not the
   estimate's equilibrium).

## Cost

4 runs × ~9 min ≈ 36 min wall on the persistent bag.

## What this experiment does NOT do

Does not address whether 02's δ_cal can recover an α_d-linear regime by
parameter tuning — that's a Module 02-side design question that ADR 0009
resolves by accepting the floor regime and re-scoping the calibration
(option D). This experiment only confirms the diagnosis empirically.
