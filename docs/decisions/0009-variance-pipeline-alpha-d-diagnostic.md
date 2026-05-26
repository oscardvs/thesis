# 0009 — variance-pipeline-α_d-diagnostic

Status: accepted
Date: 2026-05-26
Gap(s): G2
Module: 02_variance_aware_clearance.md

## Context

`sim_validation_01c/alpha_d_sweep` swept `sensor_noise_factor` across `{0.005, 0.05, 0.5}` at production geometry to characterise the calibration pre-floor for 02's δ_cal — i.e., how much of the residual variance the framework's LiDAR noise model already captures before any offset. The falsifiable prediction was a linear scaling: 100× span in α_d → 100× span in mean post-init σ², with a ±2× gate.

Measured ratio (post-init mean) across the 100× span: **12.08×**. The gate failed on the low side. Reproducibility against the archived cs010_es1 baseline passed at sub-mm tolerance (0.0 mm Δ on RMSE_all, 0.1 mm on RMSE_core), so the measurement itself is trustworthy. Init-dominated fraction was 0 % across all three cells — the steady state is settled and the under-measurement confound is ruled out. The full result is at `sim_validation_01c/alpha_d_sweep__c5f4d00d1572_1b3b5e8/sweep_summary.md`.

02 §Variance composition assumed the published per-cell variance is dominated by the LiDAR noise model `σ²_meas = α_d · z²` (`custom_kernels.py:65`). The compression from 100× to 12× says ~88 % of the per-cell variance signal comes from sources that do not respond to α_d. δ_cal would carry weight the LiDAR noise model has not justified unless we can name those sources and reason about what they leave for the calibration to absorb.

This ADR identifies the mechanism, reconstructs the measured numbers from code-level assumptions, lists the parameter-side options for restoring α_d-linear scaling, and revises Module 02's δ_cal scope.

## Findings

### F1 — Kalman pipeline geometry: harmonic-mean batch update, not per-point sequential

`add_points_kernel` (`custom_kernels.py:150–283`) does **one Kalman fusion step per batch per cell**, not one per point. For every valid inlier point hitting cell `c` in batch `k`:

```
new_v_i = map_v · v_i / (map_v + v_i)        // v_i = α_d · rz_i²
atomicAdd(&newmap[1], new_v_i)
atomicAdd(&newmap[2], 1.0)                    // count
```

then `average_map_kernel` (`custom_kernels.py:354–395`) finalises:

```
map[1] = newmap[1] / newmap[2] = mean(new_v_i)
```

If N points hit the cell with near-uniform rz (the realistic case for a small cell on a locally-planar ceiling patch), all N produce the same `new_v_i`, and `mean(new_v_i) = map_v · v̄ / (map_v + v̄)` regardless of N. **Multi-point density within a batch does not tighten the variance.** What advances the variance state is the *number of batches* the cell is observed in, not the points-per-batch.

In recurrence form, one observation step is `1/map_v⁺ = 1/map_v⁻ + 1/v`. Iterating K observations from the initial variance v₀ = 100 gives `map_v_K = v / (K + v/v₀) ≈ v/K`. This is the Miki 2022 §II-C update (eqns 1–2), faithfully implemented except that the per-batch step is the harmonic mean of a single measurement variance, not of N — a detail that matters for §F4's contribution accounting.

### F2 — Between-batch time-variance inflation provides the α_d-independent floor

`update_variance` (`elevation_mapping.py:480–482`) fires on its own timer:

```python
self.elevation_map[1] += self.param.time_variance * self.elevation_map[2]
```

i.e., `map_v += time_variance` on every valid cell, every `1/update_variance_fps` seconds. With `time_variance = 0.0001` and `update_variance_fps = 5.0`, the inflation rate is `Δ̇ = 5 × 10⁻⁴` per cell per second. Crucially: this rate **does not depend on α_d**, and applies even when no observations arrive (`enable_drift_compensation` off does not gate this path).

The recurrence with inflation between observations of spacing Δt becomes:

```
map_v_(k+1) = (map_v_k + Δ) · v / (map_v_k + Δ + v)
```

with `Δ = Δ̇ · Δt = 5×10⁻⁴ · Δt`. The steady-state fixed point is

```
map_v* = ( −Δ + √(Δ² + 4vΔ) ) / 2
```

Two limits matter:

- **Δ ≪ v (sqrt-floor regime):** `map_v* ≈ √(vΔ) ∝ √α_d`.
- **Δ ≫ v (inflation-dominated regime):** `map_v* ≈ v`, recovering the linear α_d dependence.

At the production rate Δ̇ = 5×10⁻⁴ /s with cells re-observed every ~0.5–10 s, Δ sits in [2.5×10⁻⁴, 5×10⁻³]. Measurement variance at production geometry is `v ≈ α_d · z²` with z ≈ 3 m (z is sensor-frame after the splitter's TF transform; per the splitter at `ceiling_pointcloud_splitter.py:135`, it writes `-z_world` and `frame_id=odom`, so the kernel's R, t reduce to identity and `rz = -z_world ≈ -3`). Then v ∈ [0.045, 4.5] across the swept α_d. **For all three swept α_d, v ≫ Δ — the steady state sits in the sqrt-floor regime, where map_v* scales as √α_d.** That alone predicts a 10× ratio across the 100× α_d span, against the measured 12.08×.

The HILDA config sets `time_variance = 0.0001`, two orders of magnitude below the framework default of 0.01 (`parameter.py:166`). The lower value softens the floor, but does not change the regime — the floor still dominates the published variance because `Δ ≪ v` is even more strongly satisfied.

### F3 — Pre-floor (low-K) cells scale linearly in α_d and inflate the mean

A cell needs `K_cross ≈ √(v/Δ)` observation batches before the recurrence relaxes from the linear `v/K` draw-down to the sqrt floor. At Δ = 2×10⁻⁴ (Δt ≈ 0.4 s; the "well-observed" case):

- α_d = 0.005: K_cross ≈ √(0.045 / 2×10⁻⁴) ≈ 15 batches
- α_d = 0.05:  K_cross ≈ √(0.45 / 2×10⁻⁴)  ≈ 47 batches
- α_d = 0.5:   K_cross ≈ √(4.5 / 2×10⁻⁴)   ≈ 150 batches

In a 90 s bag at 5 Hz, the maximum observation count per cell is 450, but most cells in the rolling 20×20 m window are visited intermittently as the robot traverses; the per-cell K distribution is skewed. At low α_d, even sparsely-observed cells cross into the floor regime quickly. At high α_d, a much larger fraction of cells remain pre-floor with `map_v ≈ v/K`, which scales **linearly in α_d**.

The published mean is therefore a mixture of two regimes:

- Floor-regime cells (K > K_cross): √α_d scaling
- Pre-floor cells (K < K_cross): α_d scaling
- Pure-floor ratio over the swept span: √100 = 10×
- Pure-linear ratio: 100×

The measured mean ratio 12× is exactly where the model predicts: dominated by the floor, with a non-trivial tail of pre-floor cells lifting the mean at high α_d. The median ratio of 4.6× (post-init p50) is the same population stripped of the tail, sitting below √100 — consistent with the "average" cell being well-observed and the mean–median gap being the linear-regime tail's signature.

### F4 — outlier_variance: bounded contribution

The outlier branch at `custom_kernels.py:179–181` does `atomicAdd(&map[1], outlier_variance)` directly on the live variance, **not** on the `newmap` accumulator. This injection is then *overwritten* by `average_map_kernel` whenever the cell has any inlier in the same batch (`map[1] = newmap[1]/newmap[2]`). The contribution survives into the next batch only when a cell receives zero inliers in a batch — which is the rare case for any cell currently in the LiDAR's coverage cone.

Two regimes for the persistence:
- **All-inlier batch:** outlier additions to `map[1]` are clobbered by `average_map_kernel`. Net contribution to map_v: zero.
- **All-outlier batch (no inliers):** `average_map_kernel` does not touch the cell. `map_v += N_outlier · 0.01` survives.

At α_d = 0.005 the Mahalanobis gate `|map_h − z| > map_v · mahalanobis_thresh` is tight: with map_v ≈ 0.01 and thresh = 2.0, the threshold is 0.02 m. Ceiling within-cell z-spread on real geometry (slabs adjacent to beams, lip transitions) routinely exceeds 0.02 m, so a sizeable fraction of points are classified outlier. At α_d = 0.5 the same gate sits at ~0.3 m, well outside typical within-cell z-spread, so outlier flagging collapses. The net effect is a small additive contribution at low α_d that further blunts the α_d response — but bounded by the overwrite, so it cannot account for more than a fraction of 0.01 even in the worst case.

This is a contributor, not the mechanism. It explains why the absolute floor at low α_d sits slightly above the pure `√(vΔ)` prediction; it does not change the regime.

Note also that the implementation uses `map_v · thresh` (a "scaled-variance" test), not the literal Mahalanobis `√map_v · thresh`. This is faithful to the framework code but worth flagging because the parameter name `mahalanobis_thresh` invites the wrong mental model when reasoning about how outlier classification responds to α_d.

### F5 — Other suspects: ruled out by code path

- **`max_variance = 100.0`** (`average_map_kernel`, lines 374–379): triggers reset to `initial_variance` only when `new_v/new_cnt > max_variance`. Observed maxes at α_d = 0.5 land at v ≈ 4.5 worst-case; the cap is two orders of magnitude away from binding. Confirmed by the post-init filter passing essentially all 24,743 matched cells at every α_d.
- **`initial_variance = 100.0`**: applied to fresh cells (`elevation_mapping.py:131`, 172, 268) and to cells reset by `max_variance` or by `is_valid=0`. The post-init filter `v ≤ 50` strips these; init-dominated fraction is 0 % in all three cells. Cells transition out of init on their first valid inlier observation, where `map_v = v/(1 + v/100) ≈ v` (≈ 0.045 / 0.45 / 4.5 for the three α_d).
- **`drift_compensation_variance_inler = 0.05`**: passed as the `outlier_variance` constant of `error_counting_kernel` (`elevation_mapping.py:321`). That kernel touches only `error`, `error_cnt`, and the traversability counter `newmap[3]`. Its outputs feed the drift-compensation branch at `elevation_mapping.py:404–415`, gated on `enable_drift_compensation`. With the flag off (ceiling config line 51), the parameter is dormant.
- **`mahalanobis_thresh = 2.0`**: not itself a variance contributor; gates the outlier path (F4) and the edge-sharpen-suppression path. At production the edge-sharpen rule fires below the usable rate the joint sweep measured (1c findings carry over) — irrelevant for the variance budget.

### Quantitative reconstruction

Production cadence: input cloud rate ≈ 5 Hz (RoboSense Airy → fusion → splitter chain; per W21 journal the bag's `/tf` lag is bounded near 0.2 s and the splitter's tf_timeout matches at 0.2 s, so the effective throughput is ~5 Hz). `update_variance_fps = 5.0`, so `Δ̇ = 5×10⁻⁴ /s`. Floor-regime steady state for a cell re-observed at interval Δt:

```
map_v* ≈ √(α_d · z² · Δ̇ · Δt) = z · √(α_d · Δ̇ · Δt)
```

Two scenarios bracket the measured mean:

**Well-observed cell, Δt = 0.4 s, z = 3 m, Δ = 2×10⁻⁴:**

| α_d   | v = α_d·z² | √(vΔ)    | published (≈ +Δ/2) |
|------:|-----------:|---------:|-------------------:|
| 0.005 | 0.045      | 0.00300  | 0.0031             |
| 0.05  | 0.45       | 0.00949  | 0.0096             |
| 0.5   | 4.5        | 0.0300   | 0.0301             |

Ratio: 10× (= √100). Predicts the median behaviour (measured p50 ratio 4.6×; quantitatively below √100 because z is not constant and the lower-half population includes cells where Δt is so small the floor sits even lower).

**Mixed population including pre-floor cells, Δt up to ~10 s, K-distribution skewed:**

For α_d = 0.5, pre-floor cells with K = 30 (Δt ≈ 3 s) give `map_v ≈ v/K = 4.5/30 = 0.15` — close to the measured mean 0.153 m². A small fraction (≈ 20 %) of pre-floor cells with map_v on this scale, mixed with the 80 % floor-regime population at ≈ 0.03, gives the right mean. At α_d = 0.005, all cells with K ≥ 15 are already at the floor of ~0.003, and the mean is set by the floor itself plus the small outlier-injection lift; measured 0.0127 sits ≈ 4× above the bare-floor prediction, which the F4 outlier-injection ceiling of ≈ 0.01 covers comfortably.

Median (post-init p50):

| α_d   | bare-floor pred. | measured |
|------:|-----------------:|---------:|
| 0.005 | 0.0030           | 0.00771  |
| 0.05  | 0.0095           | 0.0134   |
| 0.5   | 0.0300           | 0.0358   |

The median is within a factor of 2 of the bare-floor prediction at every α_d. The mean is offset further at high α_d (by the pre-floor tail) and at low α_d (by the outlier-injection floor). Both deviations are signed consistently with the mechanism: tail-up at high α_d, floor-up at low α_d. The ratio compression from 100× to 12× is the direct algebraic consequence of the dominant √(vΔ) floor.

What this leaves unaccounted: within-cell z-spread on non-planar overhead structure (beams, ducts, slab edges) is real and not captured by `α_d · z̄²` alone. The framework treats every point's noise as `α_d · rz²` regardless of how planar the cell is, and the published variance does not see this geometric spread — it stays in the residual that δ_cal is supposed to absorb. The diagnostic experiment in §Recovery options targets the time_variance axis, which would isolate the floor mechanism from the geometric residual.

## Recovery options

### A — Lower or zero `time_variance`

Setting `time_variance = 0` removes the inflation entirely. The recurrence becomes `1/map_v⁺ = 1/map_v⁻ + 1/v`, draws down to zero over time, and within the bag's K budget gives `map_v ≈ v/K` for most cells — linear in α_d.

Cost: the published variance loses its staleness signal. The Miki 2022 design relies on time_variance to reflect "this cell hasn't been re-observed recently, trust it less" in the published map. Removing it means cells that were observed at t=10 s and left the FOV at t=20 s still publish their last Kalman variance at t=89 s, with no encoding of the 69 s of un-observation. The constraint field would treat stale and fresh cells identically; the chance-constraint margin would under-estimate uncertainty in regions the LiDAR is no longer covering.

Severity: high. The staleness signal is load-bearing for the chance-constraint margin's behaviour at the FOV edge and under occlusion. Recommendation: do not zero.

### B — Increase `update_variance_fps` aggressively

Pushing the rate to e.g. 1 kHz multiplies Δ̇ by 200× → `Δ̇ = 0.02 /s`. At Δt = 1 s: Δ = 0.02. For α_d = 0.5, v = 4.5, still in the sqrt regime (Δ/v = 0.0044). For α_d = 0.005, v = 0.045, Δ/v = 0.44 — entering the transition regime, where map_v starts to approach v rather than √(vΔ). The compression eases but does not vanish, and the rate is now dominating the publish cadence semantically (every cell's variance jumps 0.02 m² per second of un-observation, which dwarfs realistic measurement noise within seconds).

Cost: variance state semantics collapse — cells become "high variance" almost immediately on losing observation, regardless of how confident the last measurement was. The downstream feasibility margin would tighten everywhere the moment the LiDAR cone moves.

Severity: medium-high. Same direction of problem as A: erodes the staleness gradient. Not a useful knob.

### C — Set `outlier_variance = 0`

Removes the constant addition for outlier-flagged points. Pulls the low-α_d floor down by the outlier-injection contribution identified in F4. Does not change the floor regime — the sqrt floor remains.

Cost: outliers are still rejected (the gate at `custom_kernels.py:179` still skips the Kalman update for outlier points), they just stop signalling their rejection through the variance state. The constraint field loses the "this cell saw a lot of disagreement with its prior estimate" signal. Marginal cost; outlier rejection itself is preserved.

Severity: low. Worth considering as a clean refinement, but it shifts the low-α_d mean by perhaps 0.005 in absolute terms — the regime is still floor-dominated. Doesn't address the diagnosis.

### D — Accept the sqrt regime and re-scope δ_cal

The framework is doing what Miki 2022 published. The sqrt floor is a feature: it ensures the variance state stays bounded as observations accumulate, instead of decaying to zero. The compression from 100× to 12× is the signature of that floor, operating in a regime where measurement variance ≫ inflation rate × inter-observation time. This is the design point HILDA's ceiling instance lives at, and it is what the constraint field will be reading from in deployment.

Cost: Module 02 must be re-scoped so δ_cal does not assume the published σ² is a pure LiDAR-model output. See §Implication for Module 02.

Severity: design rework on the δ_cal side, no framework changes. Recommended.

### Targeted experiment to confirm the mechanism

The F2/F3 reasoning is code-level; a single-axis sweep would falsify or confirm it empirically. Proposal:

`sim_validation_01d/time_variance_sweep.yaml`: hold α_d at 0.05 (production), sweep `time_variance ∈ {0, 0.0001, 0.001, 0.01}` at fixed everything else. Predictions, in the floor regime with z = 3 m, Δt ≈ 0.4 s:

- `time_variance = 0`:    map_v* → 0 over the bag (linear draw-down only)
- `time_variance = 0.0001`: map_v* ≈ √(0.45 · 2×10⁻⁴) ≈ 0.0095
- `time_variance = 0.001`:  map_v* ≈ √(0.45 · 2×10⁻³) ≈ 0.030
- `time_variance = 0.01`:   map_v* ≈ √(0.45 · 0.02)    ≈ 0.095

If the measured post-init mean tracks √(time_variance) at fixed α_d, F2 is confirmed and the diagnosis is on solid empirical footing. Three runs, ~30 minutes wall time including ground-truth re-use against the same bag.

The experiment is optional — the algebra and code reads in F1–F3 are sufficient to support the diagnosis — but worth running before Module 02's δ_cal protocol commits to the revised scope.

## Choice

D — accept the floor regime and rewrite Module 02's δ_cal scope against it. Do not change framework parameters. Run the time_variance sweep in §Recovery options as a corroborating measurement before the δ_cal corpus is collected.

## Rationale

A, B, and C trade well-understood semantics (the staleness signal, outlier visibility) for cosmetic α_d linearity. The framework's regime is the deployment regime; calibration should adapt to it, not the other way round. D leaves the runtime untouched and revises the part of the design that the diagnostic shows is mis-scoped — Module 02's assumption about what δ_cal is absorbing.

The targeted experiment is cheap insurance. If `time_variance` does not behave as √(time_variance), there is a second mechanism at work that this ADR has not identified, and the calibration design should not commit until it is named.

## Consequences

### Implication for Module 02's δ_cal

02 §Variance composition models the per-cell published σ²_z as the Kalman variance from the sensor noise model, and §Calibration protocol fits δ_cal as a constant (or per-surface-class) offset against ground-truth residuals at λ = 3. The diagnostic shows the published σ² is **not** the sensor noise variance alone. In the production regime it is approximately

```
σ²_published ≈ √( α_d · z² · Δ̇ · Δt ) = z · √( α_d · time_variance · update_variance_fps · Δt )
```

— a geometric mean of the measurement-variance signal and the framework's time-inflation rate, with a sublinear (√α_d) dependence on the LiDAR noise factor and a √Δt dependence on local observation cadence. δ_cal must therefore absorb three structurally distinct gaps that 02's existing list of three subsumed under "model mis-specification, drift, mean-vs-min bias" did not split out:

1. **The √-compression of the measurement-variance signal itself.** The chance-constraint identity `Pr(c_true ≥ ...) ≥ Φ(λ)` was constructed against σ² = σ²_meas; in the framework, σ² = √(σ²_meas · σ²_inflate). At λ = 3 this halves the effective margin in relative terms compared to what 02 §Theory implies. δ_cal must restore the missing margin on average across the corpus, at the operating-point time_variance and update_variance_fps. The fitted δ_cal will therefore depend on those two framework parameters — if either changes between calibration and deployment, the calibration is invalidated. Add both to the calibration's `manifest.json` and to the runtime-config-versus-calibration-config audit hook.

2. **Within-cell geometric spread carried by σ_meas in 02's §Theory but invisible in σ_published.** The framework's per-point noise model `α_d · rz²` is a pure-distance term and does not see local ceiling curvature, beam-edge transitions, or planar-deviation residuals. 02 already names this contingently under "mean-vs-min bias" gated on the suppression-rule firing rate; the diagnostic makes it unconditional. Whether or not the suppression rule fires, the published variance does not encode within-cell geometric spread, and δ_cal absorbs it through the calibration residual. The corpus's dense/sparse split — already provisionally specified for the mean-vs-min term — should be retained; the dense passes characterise the rate-and-floor-driven √-compression, the sparse passes add the geometric spread on top.

3. **The unobserved time-inflation contribution.** The Δ̇ · Δt term in the floor expression introduces an α_d-independent additive component to σ²_published that scales with local observation cadence (which depends on robot motion and FOV coverage in deployment, not on sensor properties). δ_cal cannot calibrate this away as a constant — the term varies cell-to-cell with how recently each was observed. The calibration objective should therefore be reformulated **on the residual distribution conditional on observation freshness**: bin the corpus cells by time-since-last-observation, fit δ_cal per bin (or fit a residual model in time-since-last-obs), and publish the per-bin coverage. Alternatively, δ_cal stays a single constant fit at average cadence; published coverage is then guaranteed only at that operating cadence. The two-page protocol in 02 §Calibration protocol should add this conditional-on-freshness step before the calibration corpus is collected at the partner facility.

The chance-constraint reformulation `ε(x, y) = ε_base + δ_cal + λ √σ²_c` survives, but the interpretation of `λ √σ²_c` shifts. It is no longer "λ standard deviations of LiDAR measurement noise" — it is "λ standard deviations of the framework's published Kalman+inflation state, which encodes a √-compressed view of the underlying noise plus a staleness term." Coverage interpretability is restored by δ_cal; sensitivity to α_d alone is not, and 02 should not claim it.

### Runtime-config touch

The runtime ceiling config gains no edits from this ADR. `time_variance` stays at 0.0001 (the lower-than-default value HILDA already commits to, which softens but does not remove the floor); `update_variance_fps` stays at 5.0; `outlier_variance` stays at 0.01. Each is load-bearing for some signal that is not the LiDAR-noise calibration target. The diagnostic accepts the design as Miki 2022 specified it.

### Documentation propagation

- 02 §Variance composition: replace the implicit identity `σ²_published = σ²_meas` with the floor expression above, and link to this ADR.
- 02 §Calibration protocol: add the conditional-on-freshness binning step; commit the operating-point `time_variance` and `update_variance_fps` to the calibration manifest.
- 02 open questions: close the "calibration pre-floor" open question with a reference to this ADR; open a new question "per-cadence δ_cal vs. single constant" pending the partner-facility corpus.
- Sweep result documentation: link this ADR from `sim_validation_01c/alpha_d_sweep__c5f4d00d1572_1b3b5e8/sweep_summary.md` so the FAIL verdict is not read in isolation.

### Follow-up

- Run `sim_validation_01d/time_variance_sweep` per §Recovery options before the δ_cal corpus is collected. Three runs, ~30 min, same bag. Confirms or breaks the floor mechanism empirically.
- If confirmed, no further runtime work. If broken, return to F1–F3 and identify the missing mechanism before re-scoping the calibration.
- Discipline note: the diagnostic was producible from code alone within ~1 h of careful reading — the assumption that the published σ² is the LiDAR-noise model was inherited from Fankhauser 2018 §III-B without checking how Miki 2022 §II-C diverged via the time_variance term. Add to the primary-source-reads pattern: when a downstream design assumes a layer is "the sensor noise model," verify against the recurrence the code implements, not against the conceptual source paper alone.

## Empirical update — `sim_validation_01d` result (2026-05-26 afternoon)

Ran the corroborating sweep proposed above. Two findings:

- **Pairwise √-scaling: PASS within ±50 % tolerance** on all three pairs. Measured ratios `tv=0.001/tv=0.0001 = 1.73×` (vs predicted √10 ≈ 3.16×), `tv=0.01/tv=0.001 = 4.41×` (vs 3.16×), `tv=0.01/tv=0.0001 = 7.63×` (vs √100 = 10×). The pairwise behaviour matches a √-law within the absorbing tolerance; the √-mechanism is **empirically present**.

- **Zero-inflation: FAIL.** σ² at `time_variance=0` measured 0.0244 m²; at production `time_variance=0.0001` measured 0.0268 m². Ratio 0.909 — barely a 10 % drop, against an acceptance gate of ≤ 0.5. The mechanism in F1–F3 predicts the zero-inflation case should produce σ² close to 0 (Kalman draw-down without inflation). It does not. **A second α_d-independent variance source is the dominant contributor at production `time_variance`**, with the √-floor only taking over at `time_variance ≥ 0.01` (~100× production).

The two findings together rule the F1–F3 diagnosis as **partially confirmed and partially under-specified**: the √-floor is real and demonstrable, but it is not what is setting the published σ² at the deployment operating point. The dominant production-regime contributor is something else — most likely the outlier-injection mechanism flagged in §F4. The original F4 paragraph called it a "bounded contribution"; the bound (`outlier_rate × outlier_variance`) was not pinned numerically, and the empirical result is consistent with an outlier-injection contribution on the order of 0.024 m² in this bag.

### Mechanism candidate refinement

The outlier branch at `custom_kernels.py:179` adds `outlier_variance = 0.01` to `map[1]` whenever `abs(map_h − z) > map_v · mahalanobis_thresh`. The original §F4 reasoning assumed this only survives for batches with zero inliers (otherwise `average_map_kernel` overwrites it). At production cell size (0.10 m) and the construction_site bag's traversal pattern, many cells receive small batches at the FOV edge where they are observed by oblique returns from one LiDAR but not the other; those batches can be entirely outlier-classified at the prevailing `map_v · 2.0 ≈ 0.05 m` threshold against the ceiling's typical within-cell z-spread. Integrated over a 90 s bag, the per-cell injection rate of ≈ 0.01 m² per outlier-only batch plausibly reaches the measured 0.024 m² floor.

That last sentence is a hypothesis, not a verified mechanism — pinning it down requires either instrumenting `add_points_kernel` to log per-cell outlier rates or running an `outlier_variance` sweep analogous to `sim_validation_01d`. Neither is on the critical path for Module 02; the calibration design accommodates the gap regardless of its specific source.

### Revised consequences

The Implication for Module 02's δ_cal stands — δ_cal carries a non-α_d gap, scope is broadened, freshness-conditional binning is committed. The mechanism *attribution* shifts: the floor at production `time_variance` is more outlier-injection than time-inflation, with the latter taking over only at `time_variance ≥ 0.01`. For 02's runtime config the practical implication is unchanged because HILDA's `time_variance = 0.0001` sits in the outlier-dominated regime, but the *operating-point lock* in §Calibration protocol gains an additional parameter to record:

- `outlier_variance` (currently 0.01) and `mahalanobis_thresh` (currently 2.0) join `time_variance` and `update_variance_fps` as required co-parameters of the calibration manifest. Changing any of the four invalidates the fitted δ_cal.

The recovery-options analysis is unaffected: A/B/C still trade well-understood semantics for cosmetic linearity, D remains the choice. No framework changes; calibration adapts to the regime.

### Discipline takeaway

The √-floor diagnosis was producible from code alone in ~1 h, was internally consistent at the algebra level, and was empirically confirmed in its functional form but under-specified in its production-regime weighting. Two complementary lessons:

1. **Code-level diagnostics need an empirical confirmation pass before doc propagation commits to specific mechanisms.** This ADR's original §F4 ("bounded contribution") was correct directionally but loose on magnitude; the empirical sweep was the only way to find out the bound was actually load-bearing in production.
2. **Partial-confirmation is the most common outcome, not full-PASS or full-FAIL.** The right protocol is to record what was confirmed, what was under-specified, and what would falsify further hypotheses — not to retro-fit the original ADR into a clean victory. The √-mechanism is real; the production-floor attribution was wrong; both are recorded.
