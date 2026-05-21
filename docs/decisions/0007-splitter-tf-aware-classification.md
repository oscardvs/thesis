# 0007 — splitter-tf-aware-classification

Status: accepted
Date: 2026-05-21
Gap(s): G1
Module: 01_dual_elevation_mapping.md

## Context

ADR 0006's audit (§F1) identified that `ceiling_pointcloud_splitter.py` declared world-frame-derived dead-band thresholds (`z_low: 2.05`, `z_high: 2.35`, derived from "robot max z (world frame): 1.88 m") but applied them without TF transform to PointCloud2 messages whose `header.frame_id` is `base_link` (set by the upstream `pointcloud_fusion_node.cpp:15` with `target_frame: "base_link"`). The frame mismatch meant the effective world-frame cutoff drifted with robot pose: with the robot standing (base_link at z_world ≈ 0.233 m), only ceiling features above z_world ≈ 2.58 m reached the ceiling layer; with the robot in any other pose, the cutoff shifted further. Safety-critical low-ceiling features (lowered slab at z_world = 1.5 m, hanging pipes around z_world = 2.0 m) were systematically excluded.

The fix surfaces a design decision: how to transform, and what frame to transform to.

## Options

- **A — TF-transform z only**, keeping x/y in source frame. Pass `(r20, r21, r22, t2)` (third row of the rotation + z translation) into the split kernel; compute `z_world = r20*x + r21*y + r22*z + t2` per point; classify on `z_world`; leave x/y untouched in the output. Cheapest transform (one row dot product per point on top of the existing z extraction). But the output PointCloud2 ends up with z in `target_frame` and x/y in source frame — inconsistent semantics — and the downstream `elevation_mapping_node` would still need to TF-transform x/y to its `map_frame`. The negation step on the ceiling instance still operates on `z_world` correctly.

- **B — TF-transform full (x, y, z)**, write transformed coordinates back into the buffer, and set the output PointCloud2's `header.frame_id` to the target frame. Slightly more compute (three rows of R × t per point), but the output cloud is fully frame-consistent. The downstream elevation node receives a cloud already in its `map_frame`; its TF lookup degenerates to identity. The splitter becomes the *frame-rectification* point of the pipeline.

- **C — Do nothing in the splitter; push the TF responsibility downstream.** Keep the splitter as a pure z-classifier in source frame; require all downstream consumers to TF-transform before consuming. Status quo equivalent — but the splitter's thresholds are inherently world-frame quantities, so applying them in source frame is exactly the bug. Cannot be fixed without an additional TF transform somewhere; consolidating that into the splitter is cleaner than scattering it across consumers.

## Choice

B — full (x, y, z) transform, output is in target frame, `header.frame_id` rewritten.

## Rationale

The cost of B over A is one additional multiplication per point per axis on a transform that the GPU/CPU kernel was already iterating across — bandwidth-bound, the extra arithmetic is in the noise. The benefit is structural: the output cloud is *frame-rectified*, meaning every downstream consumer (both elevation node instances, any RViz visualisation, any logging or diagnostic tool) sees a cloud in a known, predictable frame regardless of where the upstream sensor and fusion node decided to publish. The audit's framing — "the splitter assumes world frame but is fed base_link" — is structurally resolved: the splitter now *delivers* world-anchored data, end of story.

Option A would have left a subtler, longer-lived inconsistency: an output cloud whose z is in `odom` and whose x/y is in `base_link`. Anyone reading the cloud naively (especially a future author who didn't read this ADR) would assume internal frame consistency and produce subtly wrong results. The compute saving is not worth the latent-bug risk.

Option C is wrong on the same grounds as the original code: the dead-band thresholds *are* world-frame quantities. Pushing the responsibility downstream just moves the same arithmetic to a different place while leaving the splitter's contract broken.

**Target frame choice** — `odom`. The framework's both elevation node instances declare `map_frame: 'odom'` (`floor_complete.yaml:60`, `ceiling_complete.yaml:67`). In sim, `gz_sim` ground-truth TF publishes `odom` directly from the simulator's pose; on hardware, Fast-LIO publishes `odom` from `fast_lio_robosenseairy/`. Both paths are always-available, no SLAM dependency. `map` would require SLAM and would not exist in pure-replay scenarios. `corrected_map_frame` is for drift-corrected odometry which the ceiling instance does not use (`enable_drift_compensation: false` per ADR 0005). `odom` is the right level.

**TF-failure fallback** — `drop` as default. Two policies are supported via the `tf_fallback` parameter:

- `drop` (default) — when `lookup_transform` raises `LookupException`, `ExtrapolationException`, or `ConnectivityException`, drop the cloud entirely with a throttled warn-log. Safer: better to publish nothing than to publish points in the wrong frame.
- `pass_through` — apply identity transform, retain source frame_id in the output. Diagnostic mode only; lets the operator see whether the splitter is otherwise healthy when TF is missing (e.g. during initial node bring-up before TF tree has settled).

**TF lookup time** — use the message's `header.stamp`. This matches `pointcloud_fusion_node.cpp:81`'s pattern (the same kind of "transform a cloud" operation). Latest-transform (`Time()`) would be more robust to TF lag but inaccurate under robot motion: at 0.4 m/s × 0.1 s `/tf` period, x/y error reaches 4 cm — borderline at the 0.10 m cell resolution. Message-stamped lookup is more accurate; bag-replay startup drops a small fraction of clouds where `/tf` arrives later than the cloud (recorded artefact of how `rosbag2` schedules per-topic playback). The `tf_timeout_sec` parameter (default 0.2 s, matching `dynamic_ceiling_height_filter_node.py:67`) gives the buffer time to catch up.

**Implementation pattern** — matches the workspace idiom (`hilda_traversability/scripts/dynamic_ceiling_height_filter_node.py:_get_current_tip_position`): `tf2_ros.Buffer` + `TransformListener`, explicit exception handling, throttled log on failure, counter for stats. The C++ pattern in `pointcloud_fusion_node.cpp` (`tf_buffer_.transform(...)` from `tf2_sensor_msgs`) is not the Python idiom in this codebase; manual `lookup_transform` + numpy/cupy kernel keeps the splitter's existing bandwidth-friendly structure intact.

## Consequences

- `ceiling_pointcloud_splitter.py` gains a `tf2_ros.Buffer` + `TransformListener` (lifetime tied to the node), four new parameters (`target_frame`, `tf_timeout_sec`, `tf_fallback`, plus the existing `z_low`/`z_high`/`use_gpu`/`input_topic`/`floor_topic`/`ceiling_topic`), and a small `_lookup_transform(msg)` helper. The split kernels (`_split_pointcloud2_numpy`, `_split_pointcloud2_cupy`) take an additional `(R, t)` argument and apply the full xyz transform before classification.
- Output `header.frame_id` is set to `target_frame`, not preserved from the input. Consumers that assumed the input frame would survive must be updated. As of 2026-05-21 the only downstream consumers in this workspace are the two `elevation_mapping_node` instances, which TF-transform the cloud's frame to their `map_frame` automatically — both should now see `odom → odom` (identity) and proceed unchanged.
- The dead-band derivation comment in the splitter source remains accurate (world-frame robot geometry); the values 2.05 and 2.35 stand. With the splitter now operating in world frame, the cutoff is no longer pose-dependent — a real correction beyond the original "look up world-frame z" intent.
- TF lookup failures are logged with throttled verbosity (first 5 verbose, then every 100th) matching the workspace exemplar. The stats line (`Splitter stats: in=N floor_out=M ceil_out=K deadband_drop=L tf_failures=F`) gains a `tf_failures` counter visible at periodic intervals and on shutdown.
- `package.xml` requires no change — `tf2_ros` is already declared at line 26.
- The audit's CRITICAL F1 finding is resolved. Re-evaluation point for the sweep result: with low-ceiling features now reaching the ceiling layer (z_world ∈ [1.0, 2.05] m → routed to floor, z_world ∈ [2.05, 2.35] m → dead-band-dropped, z_world > 2.35 m → ceiling), the construction_site_ceiling primitives at z_world ∈ [2.0, 3.0] m partition cleanly. The dead-band still drops the [2.05, 2.35] m sliver — those features won't appear; lowering `z_low` to capture them is a separate design decision (the self-filter, not the splitter dead-band, is the canonical defence against robot-body contamination).
- Future work: if the corrected sweep reveals that further lowering `z_low` would capture more safety-critical features, the trade-off against potential robot-body contamination (when the self-filter has a regression) wants its own ADR. Not blocking the corrected sweep.
