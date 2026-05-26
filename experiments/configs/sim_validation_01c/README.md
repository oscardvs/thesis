# sim_validation_01c — α_d sensitivity

Third experiment in the sim_validation_01 family. 01 settled geometry, 01b
settled the suppression threshold, 01c characterises the variance pipeline's
response to the LiDAR noise-model parameter.

## Why

02's variance-aware clearance field consumes per-cell σ²_zceil + σ²_zfloor as
chance-constraint tightening. The δ_cal calibration protocol (02 §Calibration
protocol) is the offset that closes the gap between *what the framework
publishes* and *what the surface residuals actually distribute as*. Before
the kernel lands, we need to know how much of the published variance is
α_d-driven (the LiDAR noise model) versus other sources (drift, fusion
math, framework clamps).

The check is: hold everything else at production, sweep
`sensor_noise_factor` across an order of magnitude, measure how the
published `variance` layer responds.

## Falsifiable prediction

Under the framework's model σ²_meas = α_d · z², the Kalman steady state has
per-cell σ²_cell scaling linearly with α_d when fusion counts are large.
On the construction_site bag at cell=0.10 m and production wnt=20, most
ceiling cells are well-measured by sample-end.

Prediction (in `alpha_d_sweep.yaml` as `acceptance.variance_ratio_*`):

    mean(post_init σ²)|α_d=0.5  /  mean(post_init σ²)|α_d=0.005  ∈ [50, 200]

(Linear prediction = 100; ±2× tolerance for fusion-count variation across
the bag.)

**Falsification cases:**

- Below 50 ⇒ framework clamps or floors variance independently of α_d;
  δ_cal carries weight the model hasn't justified.
- Above 200 ⇒ additional variance sources blowing up at large α_d
  (drift coupling, max_variance saturation); α_d is not the full
  calibration knob.

Elevation RMSE is expected flat across α_d. The Kalman gain shifts but the
equilibrium estimate does not move much for 100× variance changes. Record
the spread; only flag as a finding if spread exceeds the 01b-derived
reproducibility noise floor by >10×.

## Operating point

- Bag: shared persistent recording from sim_validation_01 / 01b
  (construction_site.world, 514s traversal). Same trajectory means same
  rolling-window pose at sample-end across runs; α_d is the only variable.
- Cell size: 0.10 m (production).
- `wall_num_thresh`: 20 (production, settled by 01b).
- `enable_edge_sharpen`: true (production).

The α_d=0.05 cell IS the reproducibility check — same configuration as the
archived corrected-sweep cell
`joint_sweep__04a2e8802fc1_d57b109/cs010_es1`. Sub-mm agreement on
RMSE_all and RMSE_core gates the rest of the read-out.

## Read-out

`sim_validation_01c_postprocess.py` reports three numbers:

1. **Reproducibility** (α_d=0.05 vs archived cs010_es1). PASS = both
   RMSE_all and RMSE_core within `reproducibility_tolerance_m` (5 mm,
   1 mm in practice from 01b precedent).
2. **Variance response**. Ratio of post-init mean σ² at α_d=0.5 vs
   α_d=0.005. PASS = inside [`variance_ratio_min`,
   `variance_ratio_max`] = [50, 200]. Outside the band is a finding —
   the postprocess writes which side (clamp / saturation) and what the
   implication for δ_cal is.
3. **Elevation residual stability**. Spread of RMSE_core across the 3
   α_d runs. Reported with a flag if > 10× the 01b noise floor (0.5 mm).

## Cost

3 runs × ~10 min = ~30 min wall, no GPU memory growth versus 01b.

## Follow-ups (not in this config)

- Stationary-recording variant: drive the robot less, let cell counts
  saturate, recheck the linear-scaling claim under near-equilibrium
  conditions. Stronger structural claim; deferred unless the driven-bag
  check is ambiguous.
- 02-side: once the variance-aware kernel exists, sweep α_d again and
  measure how the chance-constraint margin λ√σ²_c moves. That's the
  feasibility-margin half of the 01 doc's prescription, gated by 02
  implementation.
