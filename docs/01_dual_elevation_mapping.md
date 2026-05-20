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

The closest published construction is Buchanan et al. 2019 (§III-B): one Grid Map data structure carrying two layers per cell, `[ĥ_floor, σ²_floor]` and `[ĥ_ceil, σ²_ceil]`, with incoming returns partitioned point-by-point through a Bayesian classifier (Gaussian likelihood `N(μ_E, σ_E)` × body-height-conditioned prior `P(E)` for `E ∈ {floor, ceiling}`) before fusion into independent Kalman filters per layer. The contrast with the present construction is mechanistic rather than topological: Buchanan partitions *returns* through classification within one map, the present design partitions *insertions* through z-inversion into two framework instances and lets each instance's existing suppression rule do the per-layer filtering. The z-inversion encoding has not been used for ceiling mapping in published work, and is the structural enabler for reusing the Miki 2022 GPU update path on the ceiling layer without modifying the framework kernels.

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
