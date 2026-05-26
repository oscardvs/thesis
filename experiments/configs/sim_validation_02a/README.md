# sim_validation_02a — variance-aware ε baseline

First end-to-end exercise of the variance-aware ε prototype kernel
(`hilda_clearance_field/prototype_kernel.py`) on the persistent sim rosbag,
through the wire-in to `compute_ceiling_metrics.py` per [decision 0010](../../docs/decisions/0010-clearance-field-package-boundary.md).
This is the first quantitative read on what the variance-aware ε actually
produces in production conditions; it also serves as a structural check
that the script's wire-in correctly translates published σ² into ε per
the formula in 02 §Theory.

## Falsifiable predictions

The kernel computes per cell

    ε(x, y) = ε_base + δ_cal + λ √max(σ²(x, y), 0)

with the script defaults λ = 3, δ_cal = 0, ε_base = 0.05. Across the
post-init subset of matched cells (cells with σ² ≤ init_thresh = 50, i.e.
where the Kalman state is no longer pinned at `initial_variance = 100`):

1. **Quantile cross-check.** Because √ is monotonic non-decreasing on σ² ≥ 0,
   the median, p90, and p95 commute through the kernel:

       ε_p50  − ε_static  =  λ √σ²_p50
       ε_p90  − ε_static  =  λ √σ²_p90
       ε_p95  − ε_static  =  λ √σ²_p95

   These hold to float-precision end-to-end through the script's wire-in.
   The acceptance tolerance is 1 mm — a violation indicates a wire-in
   defect (e.g. parameters read as defaults differently than declared,
   stats computed on different subsets, the kernel called with wrong σ²).

2. **Jensen bound on the mean.** By Jensen on √ (concave),

       ε_post_init_mean − ε_static  ≤  λ √σ²_post_init_mean

   up to a small slack (5 mm) for numerical noise. The gap (≤ vs equality)
   reflects σ²'s within-bag distribution shape: if σ² were spatially
   uniform, equality would hold; the distribution being right-skewed
   (a few high-σ² cells at FOV edges) pulls the mean apart from the median.

3. **Reproducibility against archived 01d tv=0.0001 cell.** Elevation RMSE
   matches sub-mm, σ² post-init mean matches within ±5 %. Same params,
   same bag, same code modulo the wire-in addition.

4. **Tightening sanity.** `tightening_post_init_mean` should sit between
   2 cm (below this and the wire-in is silently producing 0-margins) and
   50 cm (the Jensen ceiling at production σ² ≈ 0.026, λ = 3). This is the
   load-bearing baseline number — the average extra margin the variance-
   aware kernel demands on well-measured surfaces over the static 5 cm
   ε_safety baseline `constraint_field_node` currently carries.

5. **Consistency.** `epsilon.init_dominated_frac` equals
   `variance.init_dominated_frac` within 0.5 % (the two stats blocks share
   inputs and threshold; they must agree on what fraction of cells is
   init-dominated).

## Falsification

- *Quantile cross-check off by more than 1 mm* → wire-in bug. The kernel
  unit tests pass; the failure is in `compute_ceiling_metrics.py`'s
  invocation path (wrong parameter, wrong subset, wrong layer).
- *Jensen bound violated by more than 5 mm* → either the kernel is buggy
  (the unit tests would have caught it; if it survives, the issue is in
  `epsilon_stats`'s population vs subset arithmetic) or floating-point
  noise is larger than expected at this matrix size.
- *Reproducibility broken* → the wire-in changed something that affects
  upstream elevation computation. Inspect `_compute_metrics` for an
  inadvertent side effect.
- *Tightening_post_init_mean < 2 cm or > 50 cm* → either the production σ²
  distribution has drifted from what 01d measured (sample the variance
  stats first to verify) or the script's ε parameter defaults are not
  being read.

## Operating point

- Bag: shared persistent recording from 01 / 01b / 01c / 01d.
- Matrix: single cell at production — cs=0.10, snf=0.05, es=true, wnt=20,
  tv=0.0001. Identical to 01d's tv=0.0001 cell and 01c's snf050 cell.
- Metric-script ε parameters at canonical defaults: λ=3 (the 99.865 %
  one-sided coverage point per 02 §Theory), δ_cal=0 (sim phase per 02
  §Open questions, no calibration corpus yet), ε_base=0.05 (matches the
  static ε_safety the constraint_field_node carries today).

## Read-out

`sim_validation_02a_postprocess.py` reports five verdicts:

1. **Reproducibility** (elevation RMSE_all + RMSE_core; σ² post-init mean).
2. **ε quantile cross-check** (p50/p90/p95 on full + post-init populations
   against the closed-form λ √σ² target).
3. **Jensen bound** on the post-init mean.
4. **Tightening baseline** (tightening_post_init_mean as the headline
   number; also tightening_post_init_max, tightening_mean, init_dom_frac).
5. **Consistency** (init_dominated_frac match between ε and variance
   blocks).

The headline number — tightening_post_init_mean in metres — is what
the thesis cites when claiming "variance-aware ε tightens the margin by
~X m on well-measured surfaces at the deployed sensor-noise operating
point." That citation rests on this single number landing in the
expected range with the structural checks passing.

## Cost

1 run × ~9 min ≈ 9 min wall on the persistent bag.

## What this experiment does NOT do

- Does not sweep λ. The linearity of `tightening_post_init_mean` in λ is a
  closed-form consequence of the formula and is structurally verified by
  the quantile cross-check (since `ε_p50 − ε_static = λ √σ²_p50` makes
  the λ dependence explicit). A λ-sweep adds nothing here; 02b runs it
  only if there's downstream interest in measuring the spread of
  `tightening_post_init_mean − λ √σ²_post_init_mean` (the Jensen gap)
  across λ values for the thesis's sensitivity analysis.
- Does not address δ_cal. δ_cal is calibrated on the partner-facility
  corpus per 02 §Calibration protocol; the sim-phase value is 0 and the
  variance-driven contribution is what 02a characterises.
- Does not subscribe to the floor map. Per 02 §Calibration protocol's
  ceiling-only working assumption (σ²_zfloor ≪ σ²_zceil at HILDA's
  geometry); the script's wire-in honours that explicitly via the
  `epsilon_assumption` field in the JSON output.
- Does not extend the experiment runner. 02b — when it lands — will add
  a metric-script-param pass-through so λ and δ_cal can be swept from
  the YAML alongside the elevation parameters.
