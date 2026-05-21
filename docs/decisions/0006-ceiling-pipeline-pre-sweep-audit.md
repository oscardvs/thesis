# 0006 — ceiling-pipeline-pre-sweep-audit

Status: accepted
Date: 2026-05-21
Gap(s): G1
Module: 01_dual_elevation_mapping.md

## Context

`sim_validation_01/joint_sweep` was set up to test the load-bearing claim in 01 §Z-inversion encoding — that the framework's count-thresholded suppression rule (`enable_edge_sharpen` gating the rule at `custom_kernels.py:183`) captures the lowest overhead surface on the negated-z ceiling layer. The runner, the bag, the ground-truth extractor, and the metrics node were all built and brought to dry-run-clean before the first cell launched. The pipeline they exercise — splitter → ceiling `elevation_mapping_node` → plugin chain → metrics — was **not** itself audited before measurement. The five-flag source audit on 2026-05-20 verified the flag *decisions*, not the rest of the pipeline.

The discipline gap is the same one named in [[feedback-primary-sources]] one level up — "audit the system that produces the measurement before drawing conclusions from the measurement." This ADR records the audit done on 2026-05-21 mid-sweep, after Oscar flagged the gap.

## Options

This is an audit ADR rather than a single-decision ADR. The decisions are recorded per finding below; the meta-decision is whether to act on them before or after the in-progress sweep finishes.

- **A — Act now**, interrupt the sweep, fix the findings, restart from scratch.
- **B — Let the running sweep finish**, document its result as preliminary against the audit, then fix the findings before any further interpretation or follow-up experiment.

## Choice

B — let the sweep finish, then act. The running sweep produces data that is informationally useful for the audit itself (per-cell diagnostic PNGs surface error-spatial-structure that informs how the splitter bug manifests). Re-runs after the fixes are tractable (a fresh sweep is ~70 min). Interrupting now wastes the current ~30 min of remaining replay; finishing it gives one more data point under the documented-misconfiguration condition that the corrected sweep will be compared against.

## Findings

### F1 (CRITICAL) — Splitter frame-of-reference bug

The pipeline that feeds the splitter, traced from `pointcloud_fusion/src/pointcloud_fusion_node.cpp:15`, has `target_frame: "base_link"` (config in `pointcloud_fusion/config/config.yaml:15`, set on the fusion node). Confirmed end-to-end by `ros2 topic echo --field header.frame_id --once /perception/fused_points` returning `base_link`, and by the splitter's own log line at first message: `frame=base_link`.

The splitter at `ceiling_pointcloud_splitter.py:189-190` declares `z_low: 2.05` and `z_high: 2.35`, with the derivation at lines 178-188 stating:

```python
# Dead-band thresholds (world frame, meters above floor).
# Derived from HILDA URDF geometry (drilly_v2):
#   - Robot max z (world frame): 0.233 + 1.65 = 1.88 m
#   - z_low = 1.88 + 0.15 m margin = 2.05 m
#   - z_high = z_low + 0.30 m dead-band = 2.35 m
```

But `_split_pointcloud2_numpy` and `_split_pointcloud2_cupy` (lines 47-148) read the raw z-field of each point without any TF transform and compare directly against `z_low`/`z_high`. The thresholds are world-frame-derived but applied to base_link-frame data.

Quantitative consequence, robot standing (base_link origin at z_world ≈ 0.233 m):
- Ceiling at z_world = 3.0 m arrives as z_base_link = 2.77 m → `z > 2.35` → routes to ceiling ✓
- Lowered slab at z_world = 2.0 m arrives as z_base_link = 1.77 m → `z < 2.05` → routes to **floor** ✗
- Pipe at z_world = 2.4 m arrives as z_base_link = 2.17 m → dead-band → **dropped** ✗

In standing pose, only overhead features above z_world ≈ 2.58 m reach the ceiling layer at all. `construction_site.world` has primitives spanning z_world ∈ [1.0, 3.0] m (per `extract_ceiling_ground_truth.py` rasterisation output: 71 primitives, z_bottom range [1.00, 3.00]). The lowered slab and hanging pipes — the deliberately-introduced low-ceiling features for testing — are not reaching the ceiling map.

Worse, the cutoff is pose-dependent. The robot lays-down for transport (legs collapsed, base_link near floor) and stands-up for drilling (base_link at 0.233 m), changing the effective z_world threshold by 0.20–0.25 m as a function of leg state. Even when the threshold is correct for one pose it becomes wrong for the other.

The splitter's safety property — "the self-filter MUST run before this node to prevent robot body points contaminating the ceiling map" (docstring line 12-14) — is not actually delivered by the threshold-based dead-band. The self-filter handles robot self-occlusion; the splitter's dead-band redundantly tries to catch any leakage. With the frame mismatch, the dead-band's catch-zone slides relative to the robot, so the safety margin against residual robot points isn't even consistent.

**Decision F1**: fix the splitter. Two viable routes:

- **F1-a — TF-aware splitter**: subscribe to TF, transform incoming clouds to a world-anchored frame (`odom` is fine in sim with ground-truth TF; on hardware `odom` carries drift but the splitter only needs *coarse* z-classification so drift below the cm-scale doesn't matter), then apply the existing thresholds. Single-block change in `_typed_callback`.
- **F1-b — Pose-aware thresholds**: lookup base_link → odom transform per cloud, derive `z_low_dynamic = z_low_world - z_base_link_in_odom`, and apply the dynamic threshold to the un-transformed point z-values. Same information content as F1-a, more arithmetic per point.

F1-a is the right call. The TF transform is one library call per cloud; the existing kernel logic stays unchanged; the threshold derivation comment becomes literally true. F1-b avoids the per-point transform but at the cost of a more fragile reasoning chain (anything that touches the cloud has to track which frame it's in).

Tracked as a follow-up that wants its own ADR once the F1-a implementation is written (frame name, fallback behaviour if TF lookup fails, sensor-mount offsets).

### F2 (CRITICAL) — `wall_num_thresh` mismatch with production

`ceiling_complete.yaml:28` and `floor_complete.yaml:23` both set `wall_num_thresh: 20`. This is the production value, applied identically on both instances.

`experiments/configs/sim_validation_01/joint_sweep.yaml:24` declares:
```yaml
ceiling:
  wall_num_thresh: 100              # production default; not swept
```

The runner's overlay (`render_ceiling_yaml` in `sim_validation_01_joint_sweep.py`) applies sweep-config keys on top of the base, so the rendered ceiling YAML carries `wall_num_thresh: 100` — confirmed by `grep wall_num_thresh` on the rendered config inside the running sweep's result dir.

The comment is wrong: 100 is not the production default. The sweep is testing the suppression rule at a threshold **5× higher** than production, making it 5× less likely to fire than it would in deployment. The "rule not firing at production resolution" finding from the in-progress sweep is therefore preordained by the configuration, not measured.

The mechanistic prediction in the README (RoboSense Airy 96 density math) — that the rule cannot fire at production resolution on overhead returns — would still hold at `wall_num_thresh=20`, but the *measurement* of where it starts firing as cell size grows would shift by a factor of 5 in the density-required direction. The crossover cell size could be small enough to be within the sweep range at `wall_num_thresh=20`, where at 100 it's pushed off the top of the range.

**Decision F2**: fix the comment and the value in `joint_sweep.yaml` to track production (`wall_num_thresh: 20`). Re-run the sweep against the corrected config after F1 is also resolved. No ADR needed — this is a comment-and-value correction.

### F3 (IMPORTANT) — Plugin chain produces dead outputs

`ceiling_plugin_config.yaml` declares the chain `min_filter → smooth_filter → inpainting → ceiling_decode`. Tracing what each plugin reads and writes:

- `min_filter` (no `input_layer_name` → defaults to `elevation`) writes layer `min_filter`.
- `smooth_filter` reads `min_filter`, writes layer `smooth`.
- `inpainting` reads default (`elevation`), writes layer `inpaint`.
- `ceiling_decode` reads `elevation` (the raw Kalman-update output), writes layer `ceiling_height`.

So `ceiling_decode` bypasses the chain — it negates the raw `elevation` layer directly, ignoring `min_filter`, `smooth`, and `inpaint`. The chain's three intermediate layers are computed at 5 Hz but consumed by nothing the ceiling instance publishes:

```yaml
publishers:
  ceiling_map_raw:
    layers: ['elevation', 'variance', 'ceiling_height']
    basic_layers: ['elevation', 'ceiling_height']
```

The metrics node confirms (`compute_ceiling_metrics.py:230`):
```python
elev = _gridmap_layer_to_numpy(msg, 'elevation')
live_heights[valid_mask] = -elev[valid_mask]
```

It reads `elevation` (raw) and negates manually. So for both the metric and the consumer-facing `ceiling_height`, the data path is identical: `-elevation`. The plugin chain is dead code.

A secondary observation: `min_filter` on a *negated* elevation layer would propagate the most-negative value = the highest true ceiling. That is the *opposite* direction of what min-of-overhead semantics should mean (lowest obstacle wins). If the chain were ever wired to feed `ceiling_decode` without un-negation, it would silently produce a less-safe map. The dead-code state is fortunate; live-wiring without thought would be dangerous.

For the in-progress sweep this is *fortuitous*: the metric measures the raw framework output without confounding by plugin filtering. So the sweep's RMSE numbers tell us about Kalman fusion alone, not about a downstream filter chain.

**Decision F3**: strip the dead plugin entries from `ceiling_plugin_config.yaml`, leaving only `ceiling_decode`. The chain costs compute at 5 Hz × 200×200 cells per layer × 30 iterations on `min_filter` — meaningful Jetson budget for zero downstream benefit. A separate ADR (0007 candidate) can decide whether to *replace* the chain with a sign-aware filter pipeline that operates on the un-negated `ceiling_height` layer (correct direction for min-of-overhead) — that's a substantive architectural choice and should not happen as part of this audit's reactive fix.

### F4 (MODERATE) — Map rolls with robot, GT does not

`ceiling_complete.yaml:16` sets `map_length: 20.0` in the `odom` frame (`map_frame: 'odom'` at line 67). The framework's map is a 20×20 m window centred on the robot; cells outside the current footprint are dropped. GT is full-extent (40×30 m). The metric can only match cells inside the current live-map footprint.

This is correct framework behaviour, not a bug. But it means coverage is bounded by the trajectory ("did the robot visit this cell's neighbourhood recently?"), not by framework quality. The ~77% coverage measured in the in-progress full-bag sweep is consistent with the robot having visited most-but-not-all of the GT extent over the 514 s bag. Higher coverage would require either a longer trajectory or a larger map_length.

**Decision F4**: no fix needed. Document the framework behaviour in 01's implementation-status section so the coverage number isn't misread as a quality metric. If a future experiment wants to bound the measurement above the trajectory-coverage floor, increase `map_length` or use a `map_accumulator`-style external buffer.

### F5 (MODERATE) — Bias source unexplained

Both early-sampling and full-bag sweep results show a systematic bias of approximately −0.15 m (estimate lower than GT). The bias is computed only on cells where both the live map and GT have a value — i.e., on the high parts of the ceiling (z_world > ~2.58 m, given F1). It is not the missing-low-ceiling artefact from F1 (those don't contribute to bias, they contribute to coverage shortfall).

Candidates not ruled out by the audit:
- Kalman fusion small downward bias on heavy-tailed noise — the framework's variance update is recursive and asymmetric noise can drift the mean.
- Sensor-mount offset between the URDF and the Gazebo plugin frame for RoboSense Airy.
- GT-extraction inconsistency: `extract_ceiling_ground_truth.py` uses each primitive's `z_bottom` (lowest z of the collision AABB). The LiDAR sees the visible surface — bottom of the visual mesh — which may differ from the collision-box bottom by ~0.1-0.15 m for primitives where the mesh is a wireframe inside a padded collision box (`slab_with_holes` and `hallway` in the lookup table are obvious candidates).

**Decision F5**: a single quick diagnostic — load a known-flat-ceiling primitive's SDF, compare collision `z_bottom` against the visual mesh's vertical extent. If they differ by ~0.15 m, the bias is a GT-extraction artefact and the fix is in `extract_ceiling_ground_truth.py` (use visual-mesh bottom instead of collision-AABB `z_bottom`). If they agree, drop to candidate (a) — instrument the framework's per-cell Kalman update on a stationary flat-ceiling test and check for systematic drift.

This is a low-cost check that should happen before the corrected sweep; otherwise we'll just re-measure the same bias in the post-F1+F2 sweep without knowing why.

## Resolution plan

In recommended execution order, with re-sweep gating noted.

| Priority | Finding | Action | Effort | Ordering |
|---|---|---|---|---|
| P0 | F1 (splitter frame bug) | Add TF transform in `_typed_callback`; lookup `header.frame_id → 'odom'` and apply to all points before classification. Verify behaviour at three robot poses (laying, standing, mid-stand). | ~1 h code + 30 min sim verification | **Blocks** re-sweep |
| P0 | F2 (`wall_num_thresh`) | Fix comment + value in `joint_sweep.yaml` to `wall_num_thresh: 20`. Also worth considering: extending the matrix to sweep `wall_num_thresh` ∈ {5, 10, 20, 50} at a fixed cell size (e.g. 0.10 m) once F1 is in. | ~5 min config + decision on matrix shape | **Blocks** re-sweep |
| P1 | F5 (bias diagnostic) | Open a known-flat primitive's SDF + visual mesh; compare collision `z_bottom` against visual-mesh bottom. If different, patch `extract_ceiling_ground_truth.py`. | ~30 min | Should happen before re-sweep |
| P2 | F3 (dead plugin chain) | Strip the three dead plugin entries from `ceiling_plugin_config.yaml`. Keep `ceiling_decode` only. | ~5 min | Independent of re-sweep |
| P3 | F3 follow-up | Decide whether to replace with a sign-aware filter pipeline (operates on un-negated `ceiling_height`). | New ADR (0007 candidate) | Defer until F1+F2 sweep results are read |
| P3 | F4 (rolling map) | Document in 01's implementation-status section as known framework behaviour, not a quality issue. | ~10 min | Documentation only |
| P4 | F1 follow-up | ADR for splitter TF-aware design (frame name, fallback if TF lookup fails, sensor-mount offset handling). | New ADR (0008 candidate) | After F1-a is implemented |

**Re-sweep gating**: P0 (F1 + F2) must complete before any re-sweep is meaningful. P1 (F5) should land before the re-sweep so the bias source is known; if F5 reveals a GT bug, the re-sweep needs the patched GT. P2 (F3) is independent — it improves runtime efficiency but doesn't affect the metric. P3 and P4 are deferred work.

The current in-progress sweep finishes around ~12:30 today (~30 min remaining). Its results are recorded as preliminary in [[2026-W21]] with a pointer to this ADR, and used as a baseline against which the post-fix sweep is compared.

## Consequences

- `joint_sweep.yaml` updated post-sweep: `wall_num_thresh: 20` and corrected comment.
- `ceiling_pointcloud_splitter.py` gains a TF-transform step before the z-classification kernel. Adds tf2_ros dependency to the splitter's `package.xml`. Adds a TF-lookup failure path (drop the message, warn at throttled rate).
- `ceiling_plugin_config.yaml` shortens to just `ceiling_decode`. `01_dual_elevation_mapping.md` implementation-status section updates to record that the plugin chain was pruned and why.
- `extract_ceiling_ground_truth.py` may need a visual-mesh-bottom path (pending F5 diagnostic).
- Two follow-up ADRs queued: splitter TF-aware design (post-implementation), sign-aware filter pipeline (post-corrected-sweep, if filters are actually needed).
- Preliminary sweep result is preserved at `experiments/results/sim_validation_01/archive_2026-05-21_early-sampling/` (early-sampling) and the in-progress dir (full-bag-misconfigured). Both are kept; the corrected sweep lands in a new dir once F1 + F2 are resolved.
- Discipline lesson: **audit the system that produces the measurement before drawing conclusions from the measurement**. This is the experimental-side counterpart to [[feedback-primary-sources]]; it gets its own memory entry once the audit's resolution lands.
