# sim_validation_02b — variance-aware ε λ-sweep

Sweeps the chance-constraint coverage parameter λ across {1.0, 2.0, 3.0, 4.0}
at production framework geometry, with all other ε kernel parameters held at
the canonical sim-phase defaults (δ_cal = 0, ε_base = 0.05). Builds on 02a's
single-cell baseline by characterising the kernel's structural response to λ
and producing the thesis's headline sensitivity figure for 02 §Theory's
chance-constraint formulation.

The λ=3.0 cell IS the in-sweep reproducibility check against archived 02a:
same bag, same params, same code modulo the runner's metrics-param pass-through
added for 02b. Sub-mm elevation, sub-percent σ², ±5 % on
`tightening_post_init_mean`.

## Falsifiable predictions

The kernel computes per cell ε(x, y) = ε_base + δ_cal + λ √max(σ²(x, y), 0).
At fixed framework parameters (same bag, same code, same `time_variance`,
same `sensor_noise_factor`), σ²(x, y) is λ-independent. The closed-form
predictions across the sweep are therefore **exact**, not approximate:

1. **λ-linearity (cross-cell).** For each cell `i` at coverage λ_i,

       tightening_post_init_mean_i  =  λ_i · E[√σ²_post_init]

   with `E[√σ²_post_init]` invariant across the four cells. Equivalently,
   the slope `tightening_post_init_mean_i / λ_i` is invariant. The acceptance
   tolerance is ±5 % relative — matching the σ²-reproducibility tolerance
   measured by 01d (same-bag same-code Δt cadence variation produces ~1.4 %
   on σ² post-init mean; ±5 % is a generous gate). A violation indicates
   either a wire-in defect (λ not being read), the kernel mis-applying λ, or
   σ² drifting more than expected between cells (which the per-cell
   reproducibility check against 02a/01d would expose).

2. **Per-cell quantile cross-check.** At each cell, the same closed-form
   structural identity 02a verified at λ=3 holds at every λ:

       ε_p50  − ε_static  =  λ √σ²_p50    (full + post-init)
       ε_p90  − ε_static  =  λ √σ²_p90    (full + post-init)
       ε_p95  − ε_static  =  λ √σ²_p95    (full only)

   Five paired checks per cell, 1 mm tolerance. A failure on a single cell
   indicates that cell's λ was not faithfully invoked by the script.

3. **Per-cell Jensen bound.** At each cell,

       tightening_post_init_mean  ≤  λ √σ²_post_init_mean   (slack 5 mm)

   by Jensen on √ (concave). The gap reflects the σ² distribution shape and
   should be similar across cells (since the σ² distribution is the same);
   a cell with anomalously small gap suggests near-degenerate σ² distribution.

4. **In-sweep reproducibility (λ=3 vs archived 02a).** RMSE_all + RMSE_core
   within ±5 mm of archived; σ² post-init mean within ±5 % relative;
   `tightening_post_init_mean` within ±5 % relative of archived 424.2 mm.

5. **Per-cell tightening sanity.** Each cell's `tightening_post_init_mean`
   sits in [20, 800] mm — generous bounds, the load-bearing gate is the
   linearity slope, not the absolute per-cell number.

## Falsification

- *Cross-cell slope variation > 5 % relative* → either the wire-in is not
  reading λ from the metric-cmd `-p` args (verify `params.lam` in each cell's
  `metrics_diagnostic.json` matches the YAML), σ² is drifting more than
  expected (check `variance.post_init_mean` per cell against archived 01d),
  or the kernel is mis-applying λ (a unit-test regression — the prototype's
  test suite covers closed-form λ scaling).
- *Per-cell quantile check off by > 1 mm at any λ* → wire-in bug specific to
  that cell. The kernel unit tests pass at every λ; the failure is in
  `compute_ceiling_metrics.py`'s parameter read or subset selection at runtime.
- *In-sweep reproducibility broken at λ=3* → the runner's metrics-param
  pass-through changed something about the metric script's runtime behaviour
  even when the passed value equals the script default. Inspect the runner's
  `-p` argument construction.
- *Tightening_per_cell_min violated at any λ* → wire-in silent failure
  (defaults read as 0, σ² interpreted as 0, etc.) — the same gate that 02a's
  20 mm floor catches.

## Operating point

- Bag: shared persistent recording from 01 / 01b / 01c / 01d / 02a.
- Matrix: 4 cells at production framework geometry (cs=0.10, snf=0.05,
  es=true, wnt=20, tv=0.0001). λ ∈ {1.0, 2.0, 3.0, 4.0}. The λ=3 cell is the
  in-sweep reproducibility check against archived 02a's λ=3 measurement.
- Metric-script defaults at canonical sim values (δ_cal=0, ε_base=0.05);
  λ is the swept axis routed via the runner's METRICS_PARAM_KEYS plumbing.

## Read-out

`sim_validation_02b_postprocess.py` reports six verdict blocks:

1. **In-sweep reproducibility** (λ=3 cell vs archived 02a; elevation RMSE,
   σ² post-init mean, tightening_post_init_mean).
2. **Per-cell quantile cross-check** (5 quantile identities per cell, 4 cells,
   pass/fail aggregated).
3. **Per-cell Jensen bound** (4 cells, aggregated).
4. **Per-cell consistency** (init_dominated_frac match between ε and variance,
   4 cells).
5. **Cross-cell linearity** (slope `tightening_post_init_mean / λ` across all
   cells; relative spread vs the median slope).
6. **Headline sensitivity figure** (`tightening_post_init_mean` vs λ table,
   plus per-cell `tightening_post_init_max`).

The headline number — the slope `E[√σ²_post_init]` — is what the thesis cites
when claiming "the chance-constraint coverage parameter scales the variance-
aware margin linearly with the per-cell standard-deviation expectation." That
number is decoupled from the absolute margin (which depends on `δ_cal` to be
fitted on the partner facility) and from λ itself (which is a design choice).
It is a property of the sensor + framework + operating cadence.

## Cost

4 runs × ~9 min ≈ 36 min wall on the persistent bag.

## What this experiment does NOT do

- Does not sweep δ_cal. δ_cal calibration is bound to the partner-facility
  corpus per 02 §Calibration protocol; sweeping it in sim measures only the
  kernel's structural response to the offset (which is constant-additive and
  trivial). A future 02c can sweep δ_cal if there's downstream interest in
  the calibration-corpus pre-design figure.
- Does not sweep `ε_base`. Same reasoning — `ε_base` is the irreducible
  margin component (drill assembly, vibration, controller tracking) and is
  not a free parameter for sim characterisation.
- Does not change the framework parameters. σ² is held constant across the
  sweep so the λ-response can be isolated. Sweeping σ² + λ simultaneously
  would conflate two distinct mechanisms (covered separately by 01c, 01d,
  and the unplanned 02a-vs-01d cadence regression in [[variance-layer-regime]]).
- Does not extend the diagnostic plot. JSON is the canonical surface for the
  postprocess; the per-λ slope and tightening table render in
  `sweep_summary.md`. A per-λ ε(x, y) overlay figure is deferred until a
  reviewer asks for it.
- Does not subscribe to the floor map. Per 02 §Calibration protocol's
  ceiling-only working assumption (σ²_zfloor ≪ σ²_zceil at HILDA's geometry);
  the script's wire-in honours that explicitly via the `epsilon_assumption`
  field in the per-cell JSON output.
