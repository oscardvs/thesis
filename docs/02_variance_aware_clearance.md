# 02 — Variance-aware clearance field

## Scope

Stage 02 fuses the two elevation maps produced by 01 into a per-cell clearance scalar `c(x, y) = z_ceil − z_floor` and a configuration-aware feasibility scalar `f(x, y, s) = c − h_base − s − ε(x, y)`, where the safety margin ε tightens with the per-cell variance of the underlying observations through a chance-constraint reformulation. The contribution closes G2: the per-cell variance that the dual-instance Kalman pipeline already produces is consumed as the input to an additive constraint-tightening at the ceiling layer, structurally identical to the additive tightening used in stochastic floor-traversability MPC but applied to the second elevation instance and at no additional perception cost. The output is exposed downstream as `f`, its gradient, and a C¹-smooth CasADi B-spline interpolant updated at the elevation-map rate; the interpolant is the controller-agnostic interface consumed by either the lead NMPC or any HMPC drop-in.

## Problem statement

The lit-review survey of overhead-clearance encodings (§6.1, §6.5) reveals a perception-side asymmetry. Per-cell height variance is a published interface of the floor elevation mapping pipeline: Fankhauser 2018 §III-B establishes the Kalman update `σ_h^{2+} = σ_h^{2-} σ_p² / (σ_h^{2-} + σ_p²)` with measurement variance `σ_p² = J_S Σ_S J_S^T + J_Φ Σ_ΦIS J_Φ^T` (sensor-noise plus sensor-rotation contributions; sensor-position uncertainty is excluded by the robot-centric frame choice), and Miki 2022 §II-C ports that update to the GPU (eqns 1–2) with an added constant time-variance increment for un-updated cells. What both frameworks publish in the `variance` layer of `grid_map_msgs/GridMap` is this per-cell Kalman height variance — driven by the measurement noise model and propagated through the recursive update — not the full per-cell spatial covariance Σ_P_i that Fankhauser 2018 §III-D constructs for the map-fusion step (which incorporates accumulated robot-motion drift through eqns 14, 20, 21). The variance available to a downstream consumer at the framework's runtime interface is therefore *sensor-model-driven*, and the distinction matters for what δ_cal in the present construction is being asked to absorb.

The floor-traversability literature has consumed this variance through two distinct mechanisms — CVaR cost shaping with a relaxable hard position-risk constraint (Fan et al. 2021, STEP, where each risk factor is modelled R ~ N(μ, σ²) and the closed-form CVaR enters the SQP cost) and Gaussian-process / conformal calibration of confidence intervals (Muenprasitivej 2025). The analogous construction at the ceiling layer is absent from reviewed work, which uniformly treats the overhead safety margin ε as a static design parameter chosen conservatively to absorb worst-case sensor noise. The construction adopted here replaces that static ε with `ε(x, y)` that tracks the actual confidence of the underlying ceiling observations, structurally a **deterministic chance-constraint reformulation** in the Hewing 2020 / Lorenzen 2016 stochastic-MPC sense — a hard constraint with additive tightening proportional to the per-cell standard deviation — rather than a CVaR-in-cost penalty. This is a perception-side reformulation, not an architectural one, and generalises to any of the controller families admitted by §7.7. The construction must satisfy three downstream requirements: smoothness sufficient for gradient-based consumption (C¹ minimum), latency low enough to refresh at the 10 Hz mapping rate, and exposure as a query interface that the controller can evaluate at arbitrary `(x, y, s)` along the prediction horizon without re-rasterising the field.

## Theory

The clearance field is the cell-wise difference between the two elevation layers:

$$
c(x, y) \;=\; z_{\text{ceil}}(x, y) \;-\; z_{\text{floor}}(x, y).
$$

Under the floor-ceiling observation-independence assumption inherited from 01 (plausible — separate sensor returns, different incidence angles — but unverified on the platform's stream; flagged in 01's open questions), the per-cell clearance variance composes additively:

$$
\sigma^2_c(x, y) \;=\; \sigma^2_{z_{\text{ceil}}}(x, y) \;+\; \sigma^2_{z_{\text{floor}}}(x, y).
$$

The feasibility scalar for a given platform configuration is

$$
f(x, y, s) \;=\; c(x, y) \;-\; h_{\text{base}} \;-\; s \;-\; \varepsilon(x, y),
$$

with `h_base` the platform's base height (configuration-independent) and `s = q_1 + q_2` the combined sledge extension; the feasibility region is `{(x, y, s) : f ≥ 0}`. The safety margin ε is variance-aware:

$$
\varepsilon(x, y) \;=\; \varepsilon_{\text{base}} \;+\; \delta_{\text{cal}} \;+\; \lambda \sqrt{\sigma^2_{z_{\text{ceil}}}(x, y) + \sigma^2_{z_{\text{floor}}}(x, y)}.
$$

The three contributions decompose by source. `ε_base` absorbs irreducible factors that do not vary with map state (drill assembly extent, platform vibration amplitude, controller tracking error). `δ_cal` is a calibration term fitted offline to absorb three distinct shortfalls of the framework's published variance: (i) under-reporting of the per-cell Kalman variance on texture-poor and oblique surfaces, where the distance-quadratic measurement-noise model is optimistic and the residual distribution's spread exceeds what the Kalman channel reports; (ii) the absence of accumulated localisation drift from the published variance — Fankhauser 2018 §III-B excludes sensor position uncertainty from the measurement update by construction, and the full drift-aware spatial covariance Σ_P_i lives in the map-fusion step (§III-D) that the `elevation_mapping_cupy` runtime interface does not expose; and (iii) a contingent **mean-vs-min bias** on the ceiling layer when the count-thresholded suppression rule (01 theory; Miki 2022 §II-C, `custom_kernels.py:183`) fires below a usable rate. Under low overhead point density, the ceiling Kalman fusion degenerates to averaging across all returns in a cell, producing a mean ceiling height rather than the lower-bound surface the construction promises; the residual distribution then has a directional bias proportional to local ceiling curvature and roughness within the cell. δ_cal absorbs all three on average, calibrated against ground-truth residuals on the partner facility's instrumented sequences. λ sets the confidence level: under the Gaussian approximation that the per-cell Kalman update implicitly enforces, λ = 3 corresponds to a 99.7% probability that the true clearance exceeds the estimated clearance minus the margin. This is the deterministic reformulation of the chance constraint `Pr(c_true ≥ h_base + s + ε_base) ≥ 1 − α` with `α = Φ(−λ)`, structurally identical to additive constraint-tightening in stochastic MPC (Hewing 2020, Lorenzen 2016) but applied at the perception layer rather than to the dynamics. The construction is distinct from the CVaR-in-cost mechanism of Fan et al. 2021 (STEP), which keeps the position-risk constraint relaxable and routes risk awareness through the optimiser's objective; here the constraint stays hard and the variance enters as a deterministic margin.

The behaviour of ε with map state follows from the variance term. Over well-observed surfaces with dense LiDAR coverage and near-normal incidence, σ²_c is small and ε relaxes towards `ε_base + δ_cal`; the planner can keep the column partially extended through tight clearances. Over geometrically complex overhead features (ducts, beams) or near the LiDAR field-of-view edge, σ²_c grows and ε tightens locally; the planner is forced to retract or route around. The constraint boundary tracks the map, not the worst case envisaged at design time.

## Design choices

### Controller-agnostic interface

The downstream-facing object of this stage is `f(x, y, s)` and its gradient, not a per-controller specialisation. The package boundary follows: the runtime node publishes `f` as a `grid_map_msgs/GridMap` layer plus an auxiliary clearance message, and exposes a CasADi B-spline interpolant computed from the same field. Either the lead NMPC or any HMPC drop-in consumes through the same interface. This is the architectural commitment that keeps the controller-family decision live (see [controller-family wording](../journal/2026-W21.md) and 03 for the policy).

### Fused single-pass CUDA kernel for the field computation

The field computation is a sequence of cell-wise operations: subtract the two elevation layers to get `c`, sum the two variance layers to get `σ²_c`, square-root and scale to get the variance contribution to ε, add the constants `ε_base + δ_cal`, subtract `c − h_base − s − ε` to get `f`. Each is O(cells), each reads the same two pairs of layers, and the only loop-carried dependency is the running computation of ε before subtracting it into `f`. A fused single-pass kernel runs the whole sequence per cell in registers, with the only global-memory accesses being the four input reads and the two output writes (`f` and an auxiliary `clearance` layer). The lit-review allocation in Table 6 is 0.1 ms per update at 10 Hz; the budget is realistic for a fused kernel on the Orin Nano Super's 200×200 grid (the Kalman update kernels in 01 are bandwidth-bound at similar grid sizes and dominate the per-cycle cost by two orders of magnitude). A separated-pass design — four small kernels, one per arithmetic stage — pays four launch-overhead costs and four global-memory round-trips for no benefit; the fused design is the right default.

The kernel is parameterised on `s` so a sweep over the discrete s-profile values that 03's warm-start emits (3–5 values across the prismatic range) costs as many kernel launches as values, against the same input layers. This is the same kernel reused; no separate construction.

### CasADi B-spline interpolant: cubic, regular knot spacing

The controller queries `f` at arbitrary `(x, y, s)` along the prediction horizon, typically at finer spatial resolution than the elevation grid. The field must be C¹ at minimum (the NMPC requirement; see lit-review §7.4) and gradient-queryable analytically rather than by finite difference (the QP backend needs Jacobians, not finite-difference approximations whose error term enters the constraint linearisation). A cubic tensor-product B-spline over the grid satisfies both: it is C² globally, its first and second partials are also cubic B-splines and exact under CasADi's symbolic differentiation, and CasADi's `interpolant` factory exports the spline as a `CasADi::Function` consumable by the acados problem builder.

The form is committed: tensor-product cubic B-spline, regular knot grid co-located with the elevation grid, periodic boundary conditions disabled (the map is finite). The free parameter — knot spacing relative to the elevation grid resolution — is not committed up front. A 1:1 spacing (one knot per elevation cell) is the natural starting point and matches the spatial density of the underlying observations; coarser knot spacing reduces spline evaluation cost in the controller at the cost of smoothing real features. Pick during phase-3 profiling against the controller solve time.

### Calibration protocol for δ_cal

The lit review (§11.6 phase 2) names δ_cal as fitted offline against held-out indoor ceiling sequences but does not specify the corpus, the ground-truth source, or the statistical objective. The protocol proposed here is:

*Corpus.* Three surface classes covered at the industrial partner facility (the same site used in phase 6 hardware validation): textured concrete (the dominant indoor case), texture-poor finishes (plasterboard, paint), and oblique-incidence sequences with mean ray angle exceeding 60° from surface normal. Ten to twenty minutes of recording per class, with the platform driving a varied trajectory beneath instrumented ceiling reference points. The recording set is held out from any sequence used elsewhere in the pipeline.

*Corpus extension — dense vs sparse observation.* If the sim-phase joint sweep of validation check #3 (suppression-rule firing rate) in 01 returns a low firing rate under the operating cell size, the calibration corpus must additionally split each surface class into densely- and sparsely-observed sequences of the same patch. The densely-observed pass drives slowly past the ceiling reference points with sustained dwell, maximising points-per-cell-per-scan; the sparse pass traverses at normal operating speed. The residual distributions of the two passes characterise the mean-vs-min bias term in δ_cal: if the dense pass residuals show only sensor-model under-reporting and the sparse pass residuals show that plus a directional offset, the offset is the bias contribution. Estimating it directly from the corpus is preferable to inferring it analytically from ceiling-curvature priors. The split is conditional — if the sim check shows suppression fires reliably at the operating resolution, the original three-class corpus is sufficient and the dense/sparse split is omitted. This contingency is pre-stated here so the calibration design does not need to be redone after the sim measurement lands.

*Ground truth.* Total-station measurement of each ceiling reference point against the platform's `odom` frame, using the same total-station rig planned for phase 6. For each reference point and each frame in which the corresponding ceiling cell is observed, the measured `z_ref` is the ground truth; the Kalman estimate `ẑ_ceil` and its variance `σ²_zceil` come from the published ceiling map.

*Objective.* δ_cal is the constant offset (or per-surface-class offset, if the residual distributions differ materially) that makes the empirical residual distribution consistent with the variance-aware margin's nominal coverage at λ = 3. Formally, find δ_cal that minimises `|P(|z_ref − ẑ_ceil| > λ√σ²_zceil + δ_cal) − Φ(−λ)|` on the held-out set. The objective is coverage-based rather than second-moment-based: it makes no Gaussian assumption on the residual distribution beyond what is already implicit in the Kalman update, and it directly matches the chance-constraint semantics the margin is supposed to provide.

*Validation.* The variance-aware tightening ablation in phase 4 of the research plan (δ_cal = 0, λ = 0) tests directly whether the fitted δ_cal closes a real coverage gap or absorbs a non-existent one.

This protocol exists in this document and not in the lit review; it is a substantive deliverable for the thesis-stage methodology beyond what §11.2 carries.

### Soft constraint at the field level, slack at the controller

The field publishes `f(x, y, s)` without a sign-flip; the controller (03) consumes `f ≥ 0` as a soft path constraint with an L1 slack penalty in the OCP. This keeps the field semantics clean — `f` is the feasibility margin, positive everywhere safe, negative where the platform would collide — and isolates the soft-vs-hard policy at the boundary where it can be tuned against solver feasibility under transient perception artefacts. A separate "barrier" channel (the relaxed log-barrier term in 03's cost) is computed in the controller from the same `f`, not exported by this stage; centralising the constraint shape in the controller keeps the perception side controller-agnostic.

## Interface

**Inputs.**

- `/elevation_map/floor` — `grid_map_msgs/GridMap`, from 01 at 10 Hz, with `elevation` and `variance` layers.
- `/elevation_map/ceiling` — `grid_map_msgs/GridMap`, from 01 at 10 Hz, with `ceiling_height` (sign-restored) and `variance` layers.
- Parameters: `h_base`, `eps_base`, `delta_cal` (or `delta_cal[surface_class]` if per-class), `lambda` (default 3), spline knot spacing.

**Outputs.**

- `/constraint_field` — `grid_map_msgs/GridMap` at 10 Hz with `clearance`, `variance` (composed σ²_c), `epsilon`, `feasibility` (= f) layers, plus the same set evaluated at a small set of representative `s` values for diagnostic visualisation.
- `/ceiling_clearance` — `hilda_msgs/CeilingClearance` at 10 Hz: a compact per-cycle summary (minimum clearance in the local map, position of the minimum, indicator of any cells where f < 0 at the current `s`). Consumed by `ceiling_collision_monitor` and the HMI.
- CasADi B-spline export — produced in-process from the latest `f` field, consumed by the controller process either through a shared file handle, an in-process Python import, or a serialised parameter blob over DDS. Mechanism decided in 05 against the cross-distro DDS profile.

**Frames and rates.** All grids in `odom`; outputs share the cell layout of the input elevation grids (no resampling). Publication at 10 Hz to match upstream; the spline update is gated by the same cadence.

## Implementation status

Recorded against the `gazebo` branch of `hilda_ceiling` as of 2026-05-19.

**Skeleton in place; field computation is a TODO.** `hilda_ceiling/ceiling_constraint_field/` contains a working C++ ROS 2 node (`ConstraintFieldNode`) with `message_filters::Synchronizer` over the two GridMap topics, declared parameters for `h_base` and a *single static* `eps_safety`, and publishers for `/constraint_field` (GridMap) and `/ceiling_clearance` (`hilda_msgs/CeilingClearance`). The `syncCallback` method is annotated as a TODO with the intended computation sequence: convert GridMaps, compute clearance, compute feasibility, publish. Nothing in this package yet implements the variance-aware ε(x, y); the current parameter surface assumes the static-margin formulation that §6.2 of the lit review identifies as the open gap.

**Sibling skeletons.** `ceiling_height_lookahead` is a C++ skeleton for the 1D s-sweep that warms the controller — subscribes to the planned path and the constraint field, intends to emit `hilda_msgs/SledgeCommand`. `ceiling_collision_monitor` is a runtime safety layer with a tiered slowdown/halt against the constraint field's clearance summary; operational thresholds (`min_clearance_halt`, `slowdown_clearance`, `slowdown_factor`) live in its config. Both are at first-implementation maturity; the lookahead and collision-monitor sync paths are TODOs.

**Package boundary tension.** The existing C++ runtime skeleton lives in `hilda_ceiling/ceiling_constraint_field/`; THESIS.md and the new package layout commit to `hilda_clearance_field/` as a separate top-level package, scaffolded but empty. The motivation for separation (both NMPC and HMPC must consume from it) is sound, but `ceiling_constraint_field` already exists with a working DDS interface and is positioned within the metapackage that owns the surrounding runtime layer. Surfaced as an open call rather than silently merged: either `hilda_clearance_field` absorbs the C++ node (and the metapackage loses one sibling), or `ceiling_constraint_field` remains the C++ runtime and `hilda_clearance_field` becomes the thesis-stage CasADi-export and calibration scaffolding only. Pick before the variance-aware kernel lands.

**Nothing exists yet on the CasADi side.** No B-spline export, no controller-side interface, no calibration scaffolding. All four are deliverables of phases 2 and 3 of the research plan.

## Open questions

### Sim-phase

- **Package boundary.** `hilda_clearance_field` vs `hilda_ceiling/ceiling_constraint_field` — see implementation status. Decide before phase-2 work starts to avoid moving live code mid-implementation.
- **Spline knot spacing.** Committed form (cubic tensor-product, regular knots); free parameter (knot density relative to grid). Pick during phase-3 profiling against the controller solve time.
- **Controller-side B-spline transport.** Whether the spline reaches the UDOO via shared file, in-process import, or DDS parameter blob is a 05 question; the choice affects the field-update-to-spline-availability latency.
- **Provisional λ and ε_base for sim work.** Until δ_cal is fitted, the chance-constraint identity is parameterised at λ = 3 and δ_cal = 0. This is structurally fine for sim — the construction's behaviour as a function of variance is what the sim tests, not its absolute coverage on the partner facility's surfaces — but every controller experiment that consumes `f` from this stage in sim must explicitly carry the "δ_cal pending hardware calibration" annotation in its results manifest. Not a research open question; a discipline-of-results question.
- **Suppression-firing rate on the ceiling instance.** Measured by the joint sweep in 01's validation suite (check #3 × #4). The result determines whether δ_cal absorbs a mean-vs-min bias term in addition to model mis-specification and unmodelled drift, and whether the hardware-deferred calibration corpus needs the dense/sparse split described under "Corpus extension" above. Sim-side measurement; calibration-side consequence.

### Hardware-deferred

These are calibration-bound: the empirical signal they want comes from real sensor noise interacting with real localisation drift on the partner facility's surfaces. Sim-data substitutes would measure only what Gazebo's noise model + ground-truth poses produce — a self-consistency check, not a validation.

- **Independence of floor-ceiling observation noise at the Kalman-channel level.** The additive variance composition `σ²_c = σ²_zceil + σ²_zfloor` assumes it; the case for independence is stronger at the Kalman channel (separate returns, separate count thresholds, separate incidence-angle models) than for the full per-cell position covariance (which is shared via the robot pose). The right empirical test is a correlation check on rosbag pairs of variance maps under stationary observation on hardware; if material covariance shows up at the Kalman level, σ²_c gets a covariance term and the calibration refits. Defer to the hardware sprint.
- **δ_cal calibration corpus.** Three surface classes at the partner facility (textured concrete, texture-poor finishes, oblique-incidence sequences), total-station ground truth, coverage-based objective at λ = 3 (protocol fully specified above). All four inputs are hardware-bound. Defer to the hardware sprint.
- **Per-surface-class δ_cal vs single constant.** The choice between a single global δ_cal and a per-surface-class lookup is answered by the calibration corpus, not by design. Decide when the corpus is in hand.
- **Empirical chance-constraint coverage.** λ = 3 corresponds to 99.7% nominal coverage under the Kalman-Gaussian assumption. The fitted δ_cal recovers nominal coverage on the calibration corpus by construction, but coverage on out-of-distribution surfaces (specular finishes, glass) is unverified. Sanity run on phase-6 hardware-validation data.

Everything in this hardware-deferred block is real research work, but none of it gates sim-phase progress. The construction has been designed to consume any δ_cal value the calibration eventually produces; the sim experiments stress the *structural* behaviour (does ε(x, y) propagate correctly to the controller? does the C¹ B-spline interface return sensible gradients? does the OCP solve under varying ε? does the controller behave sensibly when σ²_c spikes locally?) without committing specific calibration values.

## Cross-references

- 01 — dual-layer elevation mapping (supplies the two elevation layers and their Kalman variances; the independence assumption originates there).
- 03 — perceptive RHC (consumes the CasADi B-spline of `f`).
- 04 — approach-aware IRM (queries the same `f` for terminal and approach-corridor feasibility).
- 05 — embedded deployment (the 0.1 ms field-computation budget and the spline-transport mechanism land here).
- Decisions: [0004 splitter-as-external-node](decisions/0004-splitter-as-external-node.md) and [0005 disable-ceiling-visibility-cleanup](decisions/0005-disable-ceiling-visibility-cleanup.md) constrain the upstream interface from 01.

Source documents:

- Literature Study, Sections 6.1, 6.2, 6.5, 11.2, 11.6.
- `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/src/constraint_field_node.cpp` and the sibling skeletons (`ceiling_height_lookahead`, `ceiling_collision_monitor`).
