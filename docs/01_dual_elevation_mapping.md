# 01 — Dual-layer elevation mapping

## Scope

Stage 01 maintains two concurrent robot-centric 2.5D elevation grids — one for the floor and one for the ceiling — as two instances of the `elevation_mapping_cupy` (Miki et al. 2022) framework, running in one ROS 2 process on the embedded GPU and sharing one CUDA context. The ceiling layer is encoded by negating the z-coordinate of every overhead LiDAR return before insertion, so that the framework's Kalman update + count-thresholded suppression captures the lowest overhead surface and continues to expose a per-cell variance through the same channel; the sign is restored at readout. The output of this stage is the pair `(z_floor, σ²_zfloor)`, `(z_ceil, σ²_zceil)` published as `grid_map_msgs/GridMap` at 10 Hz, consumed by the clearance field in 02. This stage closes G1: the closest dual-layer published construction (Buchanan et al. 2019) is a single Grid Map with two per-cell Kalman layers and per-point Bayesian classification on a laptop-class legged-robot PC, not two concurrent framework instances on a unified-memory Jetson at the lower end of the family.

## Problem statement

A robot whose effective collision height depends on `s = q_1 + q_2` must reason about the corridor between the floor and the lowest overhead surface, not a single height field. The chosen representation must satisfy four requirements (lit review Table 3): expose both floor and ceiling geometry, support real-time robot-centric updates during motion, fit the unified-memory envelope of an Orin Nano-class module, and present a continuous, gradient-queryable interface to the downstream gradient-based controller. No reviewed representation clears all four: 3D volumetric methods do not separate floor from ceiling, the MLS and PCT pipelines produce discrete per-patch outputs that violate the gradient-continuity requirement, BIM priors are unreliable at the relevant centimetre tolerance, and the closest dual-layer line (Buchanan) is single-instance and legged-platform-only. The dual-instance `elevation_mapping_cupy` design with z-inversion encoding is the construction adopted here.

## Theory

### Per-instance update

Each instance maintains a robot-centric grid of square cells with side `r`. For each cell `(i, j)`, the framework stores an elevation estimate `ẑ_ij`, a variance `σ²_ij`, and a timestamp. Range measurements project to the same `(i, j)` and are fused by a one-dimensional Kalman update with distance-dependent measurement variance (Miki 2022 §II-C, eqn 1, equivalent to Fankhauser 2018 §III-B eqn 6):

$$
K_{ij} = \frac{\sigma^2_{ij}}{\sigma^2_{ij} + \sigma^2_{\text{meas}}}, \quad
\hat z_{ij}^+ = \hat z_{ij} + K_{ij}(z - \hat z_{ij}), \quad
(\sigma^2_{ij})^+ = (1 - K_{ij})\sigma^2_{ij},
$$

with `σ²_meas = α_d · d²` for a measurement at range `d`. Outliers are rejected by a Mahalanobis gate `|ẑ_ij − z| ≤ τ_M · σ_ij`. To handle vertical walls where naive Kalman fusion would average returns at different heights into an artificially low estimate, Miki 2022 §II-C adds a per-update count-thresholded suppression: if more than `n_thresh` points fall in the same cell within one scan, returns below the current estimate are discarded before fusion runs. Cells uncovered by motion initialise to `NaN` with a finite prior variance; a constant time-variance increment `σ²_t` is added per cycle to un-updated cells (Miki 2022 eqn 2), driving stale cells back into the outlier-rejection envelope. The framework already implements this update path on the GPU as a CuPy `ElementwiseKernel`; the dual-layer construction reuses it unchanged.

### Z-inversion encoding for the ceiling layer

The Miki et al. 2022 update is a one-dimensional Kalman fusion per cell (Miki §II-C, eqn 1), augmented by a count-thresholded suppression rule: when multiple points within one scan fall in the same cell and their height span exceeds an outlier-rejection threshold, returns below the current estimate are dropped before fusion. The combined effect on a vertical wall is convergence towards the highest observed surface; the framework is not literally a max kernel, but produces max-like behaviour under dense sampling. For ceiling mapping the quantity of interest is the *lowest* overhead surface — the binding constraint for drilling clearance. Negating z before insertion preserves both mechanisms without modifying the kernel: the Kalman update is symmetric in input sign, and the suppression rule now drops returns *above* the current (negated) estimate, which after sign restoration corresponds to discarding returns above the lowest observed overhead surface:

$$
\max(-z_{\text{ceil}}) = -\min(z_{\text{ceil}}).
$$

The ceiling instance fuses points `(x, y, −z)`, and a thin readout plugin returns `h_ceil = −ẑ`. Variance propagation is unchanged because `Var(−Z) = Var(Z)`, and the Kalman gain depends only on positive variances, so `σ²_zceil(x, y)` exposed by the framework is the standard probabilistic-mapping variance over the (negated) ceiling field.

The closest published construction is Buchanan et al. 2019 (§III-B), reused unchanged in their 2021 JFR extension (Buchanan et al. 2021 §4.1): one Grid Map data structure carrying two layers per cell, `[ĥ_floor, σ²_floor]` and `[ĥ_ceil, σ²_ceil]`, with incoming returns partitioned point-by-point through a Bayesian classifier (Gaussian likelihood `N(μ_E, σ_E)` × body-height-conditioned prior `P(E)` for `E ∈ {floor, ceiling}`) before fusion into independent Kalman filters per layer. The contrast with the present construction is mechanistic rather than topological: Buchanan partitions *returns* through classification within one map, the present design partitions *insertions* through z-inversion into two framework instances and lets each instance's existing suppression rule do the per-layer filtering. The z-inversion encoding has not been used for ceiling mapping in published work, and is the structural enabler for reusing the Miki 2022 GPU update path on the ceiling layer without modifying the framework kernels.

### Variance composition

Both instances expose their per-cell Kalman variance through the standard `variance` layer of `grid_map_msgs/GridMap`. What this layer publishes is the recursively updated sensor-noise-model variance from Miki 2022 eqn (2) — equivalently Fankhauser 2018 §III-B eqn 6 — not the full per-cell spatial covariance Σ_P_i that Fankhauser §III-D constructs in the map-fusion step. The drift-aware fusion step is not exposed by the framework's runtime interface; 02 absorbs the resulting under-reporting through its δ_cal calibration term. Under the assumption that the *Kalman-channel* observation noise of the two layers is independent — separate sensor returns, different incidence angles, no shared structural error — the clearance variance composes additively at that channel:

$$
\sigma^2_c(x, y) \;=\; \sigma^2_{z_{\text{ceil}}}(x, y) \;+\; \sigma^2_{z_{\text{floor}}}(x, y).
$$

This composition is the input to the variance-aware margin in 02. Independence at the Kalman-channel level is plausible; independence on accumulated drift would not be (the two layers share the robot pose). The distinction is what δ_cal in 02 is calibrated to absorb on average; see 02's open questions for the residual empirical check.

## Design choices

### Single ROS 2 process, dual `elevation_mapping_cupy` instances

Two map instances live in one process, share one CUDA context and one CuPy memory pool, and run their update kernels on separate CUDA streams. Two processes would double the CUDA context (300–500 MB each) and double the CuPy runtime footprint, against an 8 GB unified-memory budget that also has to host the constraint field, the rest of the perception graph, and the OS. The shared-context design saves of order 300 MB and removes inter-process serialisation from the hot path. The cost is that a fault in one instance affects the other; mitigated by lifecycle isolation per instance within the process.

### External splitter node vs. internal modification

A pre-fusion splitter consumes the fused LiDAR topic and publishes `/lidar/floor_points` and `/lidar/ceiling_points`; each map instance subscribes to one. The framework stays unpatched, each instance is a stock `elevation_mapping_node` parameterised by its own config, and the splitter is independently testable. Full reasoning chain and the maintenance-posture argument are in [decision 0004](decisions/0004-splitter-as-external-node.md).

The splitter classifies points by a URDF-anchored absolute-height threshold (not robot-relative, to avoid drift on ramps); a dead-band `[z_low, z_high]` discards points in the body region and removes boundary-oscillation artefacts at the cost of a narrow blind annulus around the platform itself. Threshold values live in config, not in committed text.

The splitter runs on the GPU. The reference path uses CuPy boolean indexing (1–2 ms per cloud on the Orin Nano Super), with a custom warp-aggregated `RawKernel` targeted for 0.3–0.5 ms once profiling shows the indexing path is the binding cost. The splitter operates on the raw byte buffer of the `sensor_msgs/PointCloud2` message and preserves all auxiliary fields (intensity, ring, timestamp), so downstream consumers of the same topic are unaffected.

### Disable visibility cleanup on the ceiling instance

`elevation_mapping_cupy` runs a Bresenham-style raycasting cleanup that removes elevation artefacts caused by points seen above an existing cell — designed for floor mapping, where overhanging returns are spurious. Applied to the negated ceiling field, the same logic systematically erases the lowest overhead surfaces it is the ceiling instance's job to capture. The ceiling instance therefore runs with `enable_visibility_cleanup: false`; stale observation expiry is handled by a separate plugin (variance-threshold invalidation or a time-layer expiry; see open questions). The floor instance keeps the framework default. Full reasoning chain in [decision 0005](decisions/0005-disable-ceiling-visibility-cleanup.md).

### Asymmetric configuration

The ceiling instance runs with `scanning_duration` raised relative to the floor instance because overhead returns arrive at more oblique incidence angles and lower density per sweep, so a single update cycle accumulates fewer hits per cell. Plugin chains differ: the floor instance carries the standard geometric traversability filter chain (slope, roughness, step); the ceiling instance carries only the `CeilingDecodePlugin` that restores sign for the published `ceiling_height` layer and any future ceiling-specific filters. Asymmetric grid resolution is held in reserve as a tuning lever — coarsening the ceiling grid to twice the floor resolution halves both memory and update time at the cost of localising small overhead features less precisely. Not committed by default.

### Per-layer feature enablement

The Miki 2022 framework ships a constellation of features that were designed against the floor-mapping use case. The dual-instance construction inherits all of them by default, but several have semantics that fight the ceiling instance and must be disabled or re-parameterised explicitly. The per-feature decisions below are committed in `config/setups/hilda/{floor_complete,ceiling_complete}.yaml`; the rationale lives here.

**Visibility cleanup (Miki §II-E).** Raycasting sweep that removes cells whose height falls below the ray plus a variance margin (`if pzi < hi − σi, remove`). On the floor instance this clears dynamic obstacles that have moved out of the way. Applied to the negated-z ceiling layer, the same logic removes returns below the ray height, which after sign restoration corresponds to discarding the *lowest* overhead surface — exactly the binding constraint the layer is meant to capture. Off on ceiling, on on floor. Full reasoning in [decision 0005](decisions/0005-disable-ceiling-visibility-cleanup.md).

**Exclusion area (Miki §II-C, Fig. 3).** Ramp-parameterised filter (`ramped_height_range_a/b/c`) designed to drop points above an angle-dependent envelope, sold in the original framework as a way to "avoid creating the ceiling or close overhanging obstacle as an artifact". On the ceiling instance this is the most actively hostile feature in the framework — its explicit purpose is to suppress what the ceiling layer is built to capture. Neutralised on ceiling by setting the ramp parameters (`a=0`, `b=1`, `c=100`) so the inequality is trivially satisfied for every overhead return; kept at default Miki values (`a=0.3`, `b=1.0`, `c=0.2`) on floor.

**Height drift compensation (Miki §II-D).** Computes a height error between the latest measurement and the existing map, restricted to cells flagged as "relatively flat structure" via the traversability layer's threshold, and shifts the entire map by the average error. The traversability mask is the load-bearing dependency: it confines drift correction to cells whose geometry the CNN scores as reliable floor. The ceiling instance has no analogous traversability layer (no ceiling-trained CNN, no equivalent geometric stationarity heuristic for overhead), so the masking step has no meaningful inputs and the drift correction would be applied uniformly to the layer most affected by sensor incidence outliers. Off on ceiling, on on floor.

**Overlap clearance (Miki §II-F).** Clears stale height estimates from a previous floor in multi-floor environments — when the robot climbs stairs, the previous level's map data gets cleared from cells whose height differs from the robot's current height by more than a threshold. The mechanism assumes a single dominant ground surface that moves with the robot. The ceiling instance does not have an analogous concept of "previous ceiling" to clear, and applying the floor heuristic to negated overhead heights produces nonsense. Off on ceiling, on on floor.

**Edge sharpening (Miki §II-C, wall-handling clause).** Count-thresholded suppression of returns below the current estimate when multiple points fall in one cell within a single scan — the operative mechanism by which the framework produces sharp wall edges instead of averaging through the wall height. This is the same suppression rule the z-inversion argument relies on; after sign inversion it sharpens lower-bound overhead structure. Useful in principle on both layers; in practice the framework's `enable_edge_sharpen` flag also gates floor-specific Mahalanobis post-processing that is not meaningful on the ceiling layer. Off on ceiling, on on floor; the suppression-during-update step remains active on both regardless of the flag.

**Traversability CNN (Miki §II-G).** A learned filter producing per-cell traversability scores from local geometry, trained against floor terrain. Miki Table I attributes the dominant per-cycle cost to this filter (4.1 ms of 6.9 ms total on the Jetson Xavier in the original benchmarks; the Orin Nano Super proportions are similar). No ceiling-trained equivalent exists; running the floor CNN on negated overhead geometry produces meaningless scores. Off on ceiling, on on floor. Disabling on the ceiling instance is also the largest single per-cycle latency saving available without changing the algorithm.

**Initializer at start (`use_initializer_at_start`).** Seeds the map with a linear plane interpolation under the robot at startup, on the prior that the robot stands on approximately flat ground. There is no analogous prior for the overhead — the platform may operate in environments with no ceiling at all, or with sharply non-planar overhead. Off on ceiling, on on floor.

**Upper-bound layer (Miki §II-H).** Maintains a parallel "maximum possible terrain height" per cell, derived from rays that passed through unobserved cells without striking ground. On the floor layer this distinguishes safe occlusions (shallow upper-bound slope) from unsafe drops (steep slope). Negated on the ceiling layer, the same construction would maintain a "minimum possible ceiling height" per unobserved cell — a useful conservative bound for 04's terminal-pose admissibility decisions. The current configs leave the layer computed on both instances (`use_only_above_for_upper_bound: false`) but the ceiling instance does not yet expose it as a downstream interface. Promotion to a published layer is deferred to 04's IRM work, not 01's.

**Inpainting + smoothing + min-filter chain.** Standard CV post-processing on the floor layer for downstream foothold planning (Miki §II-I, used by Grandia 2022 and Jenelten 2020). The ceiling instance carries an abbreviated chain — `min_filter → smooth → inpaint → ceiling_decode` — that fills holes and smooths before the final sign restoration. The chain is shared because the structure (gap-filling on a sparse 2.5D field) is layer-agnostic, but the rationale on the ceiling differs: inpainting fills gaps where occlusion prevented a return, with the implicit prior that "unobserved" should be conservatively treated as "as low overhead as the surrounding observed cells", which is the safe direction for clearance reasoning.

**Plane segmentation (Miki §II-I, downstream from Grandia 2022).** Decomposes the map into convex planar regions for whole-body MPC footstep planning. Floor-only by use case; no analogous semantics on ceiling. Not part of the ceiling chain.

**Hole detection plugin (HILDA addition, `hole_detection.py`).** Local-minimum / drop-off detection layered onto the floor traversability stack, committed to `elevation_mapping_cupy` on `gazebo` branch. Not relevant to ceiling. Not part of the ceiling chain.

The pattern across all of these is consistent: the framework's defaults encode floor-mapping priors (gravity-aligned terrain, learned floor traversability, dynamic-obstacle visibility) that fail or invert on the ceiling instance. The audit collapses to four hard-disabled features (visibility cleanup, drift compensation, overlap clearance, traversability CNN), one neutralised parameter set (exclusion area ramp), and one inherited chain with a different operational rationale (the inpaint stack). No new code is required to enact any of this — the framework's `enable_*` flags and the per-instance plugin config selection are sufficient.

### Frame and shift synchronisation

Both instances are configured with the same `map_frame` (`odom`) and `base_frame` (`base_link`), share a single `tf2_ros::Buffer`, and shift their grids together: when the base motion crosses the shift threshold, both instances roll their circular buffers by the same cell offset, atomically. Without atomicity the two grids would drift out of register and the cell-wise clearance `c = z_ceil − z_floor` in 02 would mix observations from different windows.

## Interface

**Inputs.**

- `/perception/fused_points` — `sensor_msgs/PointCloud2`, fused 360° cloud from the two LiDARs (approximate density and rate live in the sensor driver config, not here).
- `tf2` — `odom → base_link` from the existing state estimation pipeline; assumed accurate per the lit-review system description.
- URDF — for the splitter dead-band thresholds and the per-instance frame configuration.

**Outputs.**

- `/elevation_map/floor` — `grid_map_msgs/GridMap` at 10 Hz with layers `elevation`, `variance`, `traversability` (when the floor traversability plugin is active), `time`.
- `/elevation_map/ceiling` — `grid_map_msgs/GridMap` at 10 Hz with layers `ceiling_height` (positive, sign-restored), `variance` (raw Kalman variance, unaffected by negation), `time`.

**Frames.** Both maps publish in `odom`; cell `(i, j)` of the two grids corresponds to the same world `(x, y)` patch after the synchronised shift.

**Rates.** 10 Hz publish target, matching the constraint-field consumer in 02 and the warm-start cycle in 03. Internal kernel rate may run higher but is gated by the publish cadence.

**Configuration.** Per-instance YAML under `config/setups/hilda/` (`floor_complete.yaml`, `ceiling_complete.yaml`) plus the dual-process orchestration (`dual_ceiling_mapping.yaml`) and the ceiling plugin chain (`ceiling_plugin_config.yaml`).

## Implementation status

Recorded against the state of the `gazebo` branch on both `elevation_mapping_cupy` and `hilda_ceiling` as of 2026-05-19. The on-disk implementation has progressed substantially past the Phase 7 milestone documented in `ceiling_mapping_implementation.md` (which itself is on the same package and now lags reality).

**Code layout.** Two layers, by design.

- *Prototype layer in `elevation_mapping_cupy/`* — Python, CuPy-based. The dual node (`elevation_mapping_cupy/dual_elevation_mapping_node.py`), the splitter prototype (`ceiling_pointcloud_splitter.py`), the ceiling decode plugin (`plugins/ceiling_decode.py`), and the validation scripts under `scripts/` (`validate_ceiling_z.py`, `extract_ceiling_ground_truth.py`, `compute_ceiling_metrics.py`) all live here. Two `hilda_*` configs (`config/setups/hilda/{floor_complete,ceiling_complete,dual_ceiling_mapping,ceiling_plugin_config}.yaml`) parameterise the dual setup. Two launchers: `hilda_ceiling_mapping.launch.py` (three-node fallback), `hilda_dual_ceiling_mapping.launch.py` (single-process dual).
- *Runtime layer in `hilda_ceiling/`* — C++. Five sibling packages, each with `src/<name>_node.cpp`, `launch/`, and `config/`: `ceiling_pointcloud_splitter`, `ceiling_constraint_field`, `ceiling_collision_monitor`, `ceiling_controller`, `ceiling_height_lookahead`. The C++ splitter has had the topic-naming alignment with the rest of the codebase and an additional sibling for the upward-facing RealSense added (`b8ce4d7`); the other four packages are at first-implementation maturity.

**Sim validation (Phase 1–7) replicated and extended.** The 51 unit tests still pass on the prototype layer. The dual node publishes both grids from the Gazebo Harmonic ceiling-sim worlds; ground-truth extraction from SDF runs end-to-end; the z-negation round-trip validator confirms 100 % of published ceiling cells fall in the expected range. Peak GPU memory on the desktop development host: 686 MB for the dual node.

**Phase 8 — Jetson Orin Nano Super deployment — has started and produced substantive findings.** Branch `jetson` was merged into `gazebo` (commit `4b62979`). Cross-distro DDS bridging through a raw-CDR re-publish trick is in place (`humble_jazzy_relay.py`, ~0.1 ms per message); QoS, discovery, and ARM-specific cleanup work are committed (`a81b5c8`, `12244cf`, `0aad50c`, `e0b81a3`). A benchmarking harness with `tegrastats` logging, sweep configuration, and a report generator is committed under `elevation_mapping_cupy/elevation_mapping_cupy/scripts/benchmark/`. Two artefacts capture the current measurements: `manual_benchmark_results.md` (2026-02-17, nine single-instance floor runs across a 2D resolution/map-size matrix) and `FINDINGS.md` (analysis of a 30-run parameter sweep). Both are on-disk and authoritative; the numbers below are quoted from them.

**Latency gap against the lit-review budget.** Single-instance floor mapping on the Jetson at 0.10 m / 20 m / all filters: `t_gpu` mean 83 ms, p95 104 ms, total mean 89 ms (`manual_benchmark_results.md` Run 01). At 0.10 m / 20 m / no filters the means hold at 77 / 82 ms. Lit-review Table 6 allocates **20 ms for dual elevation map update** at 10 Hz, citing Miki et al. (2022) desktop benchmarks. The Jetson measurement is roughly **four times the budget for a single instance** before the dual-instance multiplier or the constraint-field, B-spline, planner, and controller stages are added. This is a substantive finding — the Table 6 row inherited from desktop figures does not survive contact with the Orin Nano Super. The path forward is the tuning ladder named in `ceiling_mapping_implementation.md` (asymmetric resolution, plugin disable, smaller map, lower publish rate, custom RawKernel splitter) plus the architectural mitigations below; the budget itself needs revision in 05.

**Architectural finding: separate-process dual instances are unstable on Jetson.** The 30-run sweep documented in `FINDINGS.md` reveals (a) random mid-run node terminations affecting 8/30 runs uncorrelated with filter level or map size, hypothesised as CUDA-context corruption from two processes contending for the single iGPU through separate contexts, and (b) callback starvation in 2/30 runs where lazy plugin execution at map-publication time blocks the ROS executor for the full duration of inpainting (GPU→CPU→OpenCV→GPU round-trip on a 402×402 grid). Both failure modes argue for the single-process dual-instance architecture already proposed in the design section; the sweep was effectively a stress-test of the alternative. The single-process dual node is the path forward; the benchmark scaffolding now needs a re-run against it. This is the dominant open task on the Phase 8 side.

**Other developments.** A `hole_detection` plugin for depression/drop-off detection is committed (`ef93c5d`, `0646b76`) — adjacent to the floor traversability stack rather than part of the dual-mapping deliverable, but in the same package. The traversability filter now gracefully degrades when its weights fail to load (`cc0c524`), which matters for the Jetson startup path. Both `elevation_mapping_cupy` and `hilda_ceiling` are on branch `gazebo`; merge to `hilda` after the single-process Jetson re-run.

### Sim-phase validation suite

Validation up to this point has covered (a) per-package unit tests (51 passing on the prototype layer), (b) end-to-end publish from the Gazebo Harmonic ceiling-sim worlds, (c) structural sign-restoration via the z-negation round-trip validator, and (d) single-instance Jetson throughput from the manual benchmark sweep. The throughput numbers are credible (Phase 8 finding above); the correctness and sensitivity dimensions are not yet measured. The following checks are doable in sim with SDF ground truth and belong before any hardware sprint:

- *Reconstruction accuracy against SDF ground truth.* Per-cell `z_ceil − z_ceil_sdf` and `z_floor − z_floor_sdf` over a stationary recording, partitioned by surface class (flat, slanted, with overhead features). Reported as RMSE, p95, and per-class completeness against the SDF footprint. Catches systematic bias and any sign-restoration regression that the round-trip check does not exercise (the round-trip check is structural, not metric).

- *Sensitivity to the LiDAR noise-model parameter `α_d`.* `σ²_meas = α_d · d²` is the only knob in the framework's per-return noise model, and the published per-cell variance scales with it. Sweep three `α_d` values across an order of magnitude, hold all else fixed, measure how the published `variance` layer responds and how downstream feasibility-margin behaviour shifts under 02's λ√σ²_c tightening. Establishes the calibration-pre-floor for δ_cal: how much of the residual variance the model already captures before any offset is added.

- *Suppression rule firing on ceiling returns.* The count-thresholded suppression (Miki §II-C) requires more than `wall_num_thresh` points in a cell within one scan to fire. Overhead returns arrive at lower density than floor returns; under typical operating conditions the suppression rule may rarely or never fire on the ceiling instance, in which case the framework collapses to plain Kalman fusion (which averages across overhead structure rather than preserving the lowest surface). Count fire events on the ceiling instance per sweep and per cell during a representative traversal. If suppression rarely fires, surface as a finding in 01 and consider whether a lowered `wall_num_thresh` is appropriate for the ceiling instance.

- *Asymmetric grid resolution.* Run the ceiling instance at twice the floor cell size (0.20 m vs 0.10 m), hold all else fixed, measure update time, peak GPU memory, and the same accuracy/completeness numbers as the first check. Verifies the latency lever held in reserve under "Asymmetric configuration" actually halves the cost as expected, and characterises the accuracy loss the doubled resolution introduces on small overhead features. The Kalman kernel's bandwidth-bound regime may give less benefit than expected at half the cell count.

- *Cell-correspondence under fast yaw.* Both instances shift their grids together when the base motion exceeds the shift threshold. Under fast yaw rotations (>0.5 rad/s sustained), the synchronised-shift mechanism must preserve cell-wise (x, y) correspondence between the two grids — otherwise the cell-wise clearance `c = z_ceil − z_floor` in 02 mixes observations from misaligned windows. Drive a yaw sweep in sim, sample the two grids at the same wall-clock time, compute the per-cell shift offset (should be identically zero), and confirm.

- *Splitter failure-mode robustness.* Three sim scenarios: (a) no points in the ceiling band for an extended interval (open sky), (b) both LiDARs momentarily blocked (object directly in front), (c) URDF height assumption violated (driving on a 15° ramp where the splitter's absolute-z threshold becomes wrong). For each, characterise what the ceiling instance publishes — should be `NaN` cells with time-inflated variance, not garbage data persisting from before the failure. Catches the kind of edge case the unit-test layer does not exercise.

The first four checks need only existing runners and a stationary-or-driven ceiling-sim world; the latter two need targeted scenario worlds. Configs and a runner skeleton land under `thesis/experiments/configs/sim_validation_01/` and `thesis/experiments/runners/` respectively (per the configs-not-scripts discipline). Not all need to be on the critical path for the colloquium-stage deliverable, but reconstruction accuracy and `α_d` sensitivity are the two that any reviewer will ask about; both are sim-doable and short.

## Open questions

- **Observation independence at the Kalman-channel level.** σ²_c = σ²_zceil + σ²_zfloor assumes the published per-cell Kalman variances of the two layers are independent. Independence is defensible by construction (separate returns, separate incidence-angle models, separate count thresholds) but unverified on the platform's sensor stream. Independence on the *unmodelled-drift* channel is not claimed: drift is shared because the robot pose is shared, and δ_cal in 02 absorbs that on average. A correlation check on rosbag pairs of variance maps under stationary observation belongs in the calibration sequence that feeds 02.
- **Stale observation expiry.** The ROS 2 port of `elevation_mapping_cupy` lacks the `scanning_duration` cleanup of the ROS 1 version; `time_variance` inflates uncertainty over time but never reverts cells to `NaN`. Three options stand: variance-threshold invalidation, a `max_observation_age` parameter with explicit time-layer expiry, or a modified raycasting cleanup adapted for overhead geometry. The right choice depends on the failure mode the planner cares about most.
- **Asymmetric resolution.** Whether the ceiling instance should run at twice the floor cell size has not been decided. Pick during phase 5 of the research plan when the Jetson timing budget is measured under load, not before.
- **Map persistence between missions.** The robot currently re-scans on startup. GridMap serialisation to disk or rosbag replay at boot would close the gap. Out of scope for the colloquium-stage deliverable; flagged for phase 6.

## Cross-references

- 00 — pipeline overview
- 02 — variance-aware clearance field (consumes σ²_zceil, σ²_zfloor and the two height layers)
- 03 — perceptive RHC (downstream of 02)
- 05 — embedded deployment (the 20 ms dual-update budget in Table 6 is set here)

Source documents:

- Literature Study, Sections 4.1, 4.4, 6.2, 9.1, 11.1.
- `~/ros2_ws/src/elevation_mapping_cupy/ceiling_mapping_implementation.md`.
- `~/ros2_ws/src/hilda_ceiling/{ceiling_pointcloud_splitter, ceiling_constraint_field, …}` — runtime packages.
- `~/ros2_ws/src/elevation_mapping_cupy/{scripts, elevation_mapping_cupy/plugins/ceiling_decode.py, config/setups/hilda/}` — experimental scaffolding.
