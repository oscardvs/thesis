# CeilingClearance-at-robot-pose follow-up — design

*2026-05-27. Single-file follow-up to Phase A of module 02 (variance-aware clearance field). Closes the last C++-side TODO from the constraint_field_node wire-in.*

## Scope

Populate `/ceiling_clearance` (`hilda_msgs/CeilingClearance`) inside the existing `constraint_field_node`, sourced from the four-layer output GridMap the kernel already produces. The runtime node lives in `hilda_ceiling/ceiling_constraint_field/` per [ADR 0010](../../decisions/0010-clearance-field-package-boundary.md); this commit stays on the runtime side of that boundary.

Edit surface: `src/constraint_field_node.cpp` (new method bodies + the syncCallback tail call); four new members in `include/ceiling_constraint_field/constraint_field_node.hpp` (`tf_buffer_`, `tf_listener_`, `robot_base_frame_`, `s_phys_max_`); two new parameters in `config/constraint_field_params.yaml`; new test file `test/test_publish_clearance.cpp` registered as a second `ament_add_gtest` target; `tf2_ros` / `tf2_geometry_msgs` entries in `CMakeLists.txt` / `package.xml`. No new package, no new topic, no new launch file.

## Architecture

Append to the existing `syncCallback` (after the kernel + `/constraint_field` publish):

```
syncCallback(floor_msg, ceiling_msg)
  ├── fromMessage / layer-presence / geometry-alignment      [unchanged]
  ├── compute_variance_aware_field(...) → output_map         [unchanged]
  ├── constraint_field_pub_->publish(output_map)             [unchanged]
  └── publishClearanceAtRobotPose(output_map, ceiling_stamp) [new]
       ├── tf_buffer_->lookupTransform(map_frame, robot_base_frame, ceiling_stamp)
       │     → FailReason::TF_LOOKUP_FAILED on exception      [throttle-WARN, skip]
       ├── grid_map::Position(tx, ty)
       │   if (!output_map.isInside(pos))
       │     → FailReason::OUT_OF_MAP                         [throttle-WARN, skip]
       ├── sample {clearance, epsilon} layers
       │   if (any std::isnan(value))
       │     → FailReason::NAN_CELL_AT_POSE                   [throttle-INFO, skip]
       ├── compute f, populate CeilingClearance msg
       └── clearance_pub_->publish(msg)
```

**New members** (header):

- `std::shared_ptr<tf2_ros::Buffer> tf_buffer_;`
- `std::shared_ptr<tf2_ros::TransformListener> tf_listener_;`
- `std::string robot_base_frame_;`
- `double s_phys_max_;`

**New private methods**:

- `void publishClearanceAtRobotPose(const grid_map::GridMap& field_map, const builtin_interfaces::msg::Time& stamp);` — TF lookup wrapper; does the lookup, dispatches `logSkip` on `tf2::TransformException`, otherwise calls `populateAtPosition` and publishes on success.
- `std::optional<ClearanceFailReason> populateAtPosition(const grid_map::GridMap& field_map, const grid_map::Position& pos, const builtin_interfaces::msg::Time& stamp, hilda_msgs::msg::CeilingClearance& msg) const;` — TF-independent, unit-testable. On success fully populates `msg` and returns `nullopt`; on `OUT_OF_MAP` or `NAN_CELL_AT_POSE` returns the reason and leaves `msg` untouched.

**File-local enum** (anonymous namespace in `.cpp`):

- `enum class ClearanceFailReason { TF_LOOKUP_FAILED, OUT_OF_MAP, NAN_CELL_AT_POSE };`
- Used by a small `logSkip(reason, detail)` helper that switches to the right `RCLCPP_*_THROTTLE` per the severity split below.

**Build deps**: `find_package(tf2_ros REQUIRED)` and `find_package(tf2_geometry_msgs REQUIRED)`; both added to `ament_target_dependencies(constraint_field_node ...)`; matching `<depend>` entries in `package.xml`.

## Data flow + key implementation choices

**Layers read from the map**: `clearance` and `epsilon` only. The kernel's `feasibility` layer is *not* read at robot pose; it stays in the published GridMap for RViz inspection. Rationale: the layer's `f` embeds the kernel-side `h_base` and `s_ext` *at the timestamp it was launched*; reading it would couple the message's `is_safe` to scalar values that may diverge from the node's current `h_base_` and `s_` (e.g. stale kernel launch, mismatched defaults, future `/reference_controller/state` wiring landing in one but not the other). Computing `f` node-side keeps `h_base`, `s`, `epsilon`, and `clearance` each sourced exactly once.

**`f` computation**:

```cpp
const float c   = field_map.atPosition("clearance", pos);
const float eps = field_map.atPosition("epsilon",   pos);
if (std::isnan(c) || std::isnan(eps)) { logSkip(NAN_CELL_AT_POSE, ...); return; }
const double f = static_cast<double>(c) - h_base_ - s_ - static_cast<double>(eps);
```

**Message field semantics**:

- `header.stamp` = `ceiling_msg->header.stamp` (lets downstream measure staleness against the field they consume, not wall-clock).
- `header.frame_id` = `field_map.getFrameId()` (typically `"odom"`; read from the map, not hardcoded — robust to launch-config rename).
- `clearance` = sampled `c` from the map (m).
- `is_safe` = `(f > 0.0)`.
- `feasibility` = `is_safe ? 1.0 : 0.0` (binary mirror of `is_safe`; the kernel's per-cell `f` is a meter-scale signed value, the msg field is `[0, 1]`, no thesis-grounded continuous mapping exists).
- `current_sledge_height` = `s_` (the node parameter; production wires `/reference_controller/state` into `s_` in a later commit).
- `max_feasible_height` =

```cpp
const double s_geom = static_cast<double>(c) - h_base_ - static_cast<double>(eps);
double max_h;
if (s_geom <= 0.0) {
  max_h = std::numeric_limits<double>::quiet_NaN();
} else {
  max_h = std::min(s_geom, s_phys_max_);
}
```

Two design choices in `max_feasible_height` worth surfacing:

- **NaN encoding for infeasibility, not 0.0**. Clamp-at-0 collides "barely feasible, extend to ~0" with "no feasible height exists" and picks the safe-looking value for the infeasible case — a silent false-safe on a planning field. NaN matches the stack's NaN-propagation convention, separates the two meanings cleanly, and fails in the safe direction under comparison: a downstream `s_target <= max_feasible_height` evaluates false against NaN, i.e. "can't deploy here", which is the correct semantics.
- **Sledge-travel cap (`s_phys_max_`)**. Under a high ceiling the geometric headroom `c − h_base − ε` can exceed the sledge's mechanical travel. "Max feasible height" derived from sledge extension implies a *deployable* height; reporting beyond mechanical travel misleads in the opposite direction. Default `s_phys_max = 1.28` m from the URDF's `linear_joint` upper limit (`src/hilda_ros/hilda_common/hilda_description/urdf/application_layers/ceiling_drilling/sledge_drilly_v2.urdf.xacro:55`).

**TF lookup**:

- Target frame: `field_map.getFrameId()`.
- Source frame: `robot_base_frame_` (parameter, default `"base_link"`).
- Time stamp: `ceiling_msg->header.stamp` (spatial-temporal consistency with the sampled map).
- Tolerance: none (`tf2::Duration(0)`). Bag-replay has accurate TF at every msg stamp; production runs the broadcaster faster than the sync cadence. A missing transform at this stamp is a real fault, not a timing race.
- Catch scope: `tf2::TransformException` only (covers extrapolation / connectivity / lookup base classes). Anything else from `lookupTransform` is a bug and should surface, not be swallowed.

**Sampling**: `isInside(pos)` pre-check, then `atPosition(layer, pos)` for the two layers. `atPosition` is the canonical grid_map single-cell-by-world-coordinates accessor and handles the float-to-cell mapping internally.

**Log severity + throttles**:

- `TF_LOOKUP_FAILED` and `OUT_OF_MAP`: `RCLCPP_WARN_THROTTLE(..., 2000, ...)` — genuine faults (publisher stalled, pose diverged, upstream config bug, robot left rolling map). Matches the existing WARN throttle in this file.
- `NAN_CELL_AT_POSE`: `RCLCPP_INFO_THROTTLE(..., 10000, ...)` — expected steady state during exploration and in open areas. A WARN here would fire constantly under normal operation and train the operator to tune out the channel, hiding the genuine faults. The 10 s throttle (vs the WARN path's 2 s) accounts for the expected higher firing rate without losing operator visibility on a persistent problem. See [[feedback-log-severity-by-failure-class]].

**Why infeasibility-at-pose is published, not skipped**: a cell where `f < 0` is exactly what `/ceiling_clearance` exists to signal. Skipping that cycle would leave the consumer's cached `latest_clearance_` at the *previous* (safe) message, defeating the alarm — the spatial counterpart to [[feedback-latching-spatial-samples]]'s last-good-republish anti-pattern. So infeasibility is publish-with-degraded-fields (`is_safe=false`, `max_feasible_height=NaN`); only lookup and sample *failures* skip the cycle.

## New parameter surface

Added to `config/constraint_field_params.yaml` and declared in the node constructor:

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `robot_base_frame` | string | `"base_link"` | — | TF source frame for the robot-pose lookup. |
| `s_phys_max` | double | `1.28` | `[0.0, 5.0]` | Sledge prismatic travel upper bound (m). Sourced from URDF `linear_joint` upper. Caps `max_feasible_height` to mechanically deployable values. |

Existing parameters (`h_base`, `eps_base`, `delta_cal`, `lam`, `s`, `elevation_layer`, `variance_layer`, four topic names) unchanged.

## Error handling

| Condition | Handler | Severity | Effect on `/ceiling_clearance` |
|---|---|---|---|
| `tf2::TransformException` from `lookupTransform` | catch, `logSkip(TF_LOOKUP_FAILED, e.what())` | WARN, 2 s throttle | skipped this cycle |
| `!field_map.isInside(robot_pos)` | `logSkip(OUT_OF_MAP, "pos=(x, y), map_center=(cx, cy)")` | WARN, 2 s throttle | skipped this cycle |
| `std::isnan(c) \|\| std::isnan(eps)` | `logSkip(NAN_CELL_AT_POSE, "pos=(x, y)")` | INFO, 10 s throttle | skipped this cycle |
| `s_geom ≤ 0` (cell at robot pose is infeasible) | not a fault | n/a (silent) | published with `is_safe=false`, `feasibility=0.0`, `max_feasible_height=NaN` |

`/constraint_field` continues to publish in all four cases; it is the liveness witness for the node, so a `/ceiling_clearance` skip does not look like node death to downstream.

**Consumer-side follow-ups** (out of scope today, named so they do not ambush integration):

1. `ceiling_collision_monitor` needs a staleness tolerance of 2–3 cycles (~250–375 ms at 8 Hz) on `/ceiling_clearance`. A one-cycle staleness halt will flap on transient blips (one missed TF, one NaN crossing an unobserved patch); a multi-cycle window absorbs blips and still halts fast on persistent loss.
2. The monitor's halt-on-stale-clearance must be gated on `s > s_thresh` (Mode B — sledge extended). In Mode A (transit, sledge retracted) the ceiling check is irrelevant; an unconditional staleness halt would stall transit under any open or unmapped ceiling.

Both belong in `ceiling_collision_monitor`, not this commit. Recorded in 02's §Implementation status follow-ups list.

## Testing

**Unit tests** — five-case gtest fixture on a new private helper method:

```cpp
std::optional<ClearanceFailReason> populateAtPosition(
    const grid_map::GridMap& field_map,
    const grid_map::Position& pos,
    const builtin_interfaces::msg::Time& stamp,
    hilda_msgs::msg::CeilingClearance& msg) const;
```

The helper takes the position directly (bypassing TF) and on success fully populates `msg` (including `header.stamp` and `header.frame_id` from `field_map.getFrameId()`) and returns `std::nullopt`; on `OUT_OF_MAP` or `NAN_CELL_AT_POSE` it returns the reason and leaves `msg` untouched. The public `publishClearanceAtRobotPose` wraps it with the TF lookup + the publish; the wrapper logic (lookup, catch, logSkip dispatch, publish) is exercised at runtime by the bag-replay verification rather than mocked.

| # | Setup | Expected outcome |
|---|---|---|
| 1 | clearance=1.0, epsilon=0.10, h_base=0.85, s=0.0, s_phys_max=1.28 | returns `nullopt`; msg has `is_safe=true`, `feasibility=1.0`, `clearance=1.0`, `current_sledge_height=0.0`, `max_feasible_height=0.05` |
| 2 | clearance=2.5, epsilon=0.10, h_base=0.85, s=0.0, s_phys_max=1.28 | returns `nullopt`; `max_feasible_height = min(1.55, 1.28) = 1.28` (cap binds) |
| 3 | clearance=0.80, epsilon=0.10, h_base=0.85, s=0.0 → s_geom=−0.15 | returns `nullopt`; `is_safe=false`, `feasibility=0.0`, `max_feasible_height=NaN` |
| 4 | clearance=NaN, epsilon=0.10 (or vice versa) | returns `NAN_CELL_AT_POSE`; msg untouched |
| 5 | position outside `isInside` footprint | returns `OUT_OF_MAP`; msg untouched |

Land in a **new** test file `test/test_publish_clearance.cpp` registered as a second `ament_add_gtest` target. The existing `test_variance_aware_kernel` target is scoped to the kernel's per-cell math; the new tests cover msg population logic and don't belong with it. Both targets share the grid_map_core include propagation already documented in the CMakeLists comment.

**Runtime verification on the persistent sim bag** — same orchestration as Phase A's bag-replay check (journal 2026-05-27):

1. Re-run the existing `sim_validation_02a` config under the runner with `constraint_field_node` orchestrated as a sidecar.
2. Acceptance:
   - `/ceiling_clearance` publishes at ~8 Hz over the bag.
   - ≥ 95% of GridMap-publish cycles produce a `/ceiling_clearance` message (transient TF blips tolerated).
   - WARN throttle log shows zero `TF_LOOKUP_FAILED` / `OUT_OF_MAP` over the bag (sim has clean TF; the robot stays inside the rolling map).
   - INFO throttle log may fire on `NAN_CELL_AT_POSE` (rolling map has edges).
3. Spot-check on one message: `clearance` value sensible (~2.2 m under the construction-site ceiling), `is_safe=true` in normal cells, `max_feasible_height = min(c − h_base − ε, 1.28)`.

**Regression on `/constraint_field`** — the kernel itself is unchanged, so the 6-case kernel gtest still passes by construction. The runner's existing 02a regression result is preserved by the deterministic-naming property (different git_sha → different result dir); the new run creates a new result dir and the elevation pipeline numbers should still pass 5/5.

## Implementation-status sync + journal discipline

Two doc updates land in the same commit as the code (per [[feedback-impl-status-sync]]):

1. `thesis/docs/02_variance_aware_clearance.md` §Implementation status — append a block under the existing 2026-05-27 "C++ kernel + node wire-in landed" entry, recording: the CeilingClearance population, the `f`-recomputation-vs-layer-read decision and why, the `s_phys_max` parameter sourcing from URDF, the NaN-encoded infeasibility convention, the publish-on-infeasible-pose semantics, and the consumer-side follow-up list (staleness tolerance + Mode-B gating).
2. `src/thesis/journal/2026-W22.md` — new sub-entry under 2026-05-27 covering the four corrections the spec received (kernel-feasibility-layer drop, NaN infeasibility, sledge-cap, throttle-split), the two new memory entries ([[feedback-latching-spatial-samples]], [[feedback-log-severity-by-failure-class]]), and the bag-replay verification result.

No ADR. The four design corrections live cleanly in the spec + impl-status block. ADR threshold is "non-obvious reasoning chain that future code may need to re-derive" (per [[feedback-adr-for-subtle-calls]]); here the chain is dominated by `.msg` layout interaction, which is documented in the .msg file itself.

## References

- [`hilda_msgs/msg/CeilingClearance.msg`](../../../../hilda_ros/hilda_common/hilda_msgs/msg/CeilingClearance.msg) — message layout.
- [`ceiling_constraint_field/include/ceiling_constraint_field/constraint_field_node.hpp`](../../../../hilda_ceiling/ceiling_constraint_field/include/ceiling_constraint_field/constraint_field_node.hpp) and [`src/constraint_field_node.cpp`](../../../../hilda_ceiling/ceiling_constraint_field/src/constraint_field_node.cpp) — node skeleton; this commit's edit surface.
- [`ceiling_collision_monitor/src/collision_monitor_node.cpp`](../../../../hilda_ceiling/ceiling_collision_monitor/src/collision_monitor_node.cpp) — consumer (TF buffer + listener pattern at lines 66–67; staleness/Mode-B follow-up lives here).
- [`sledge_drilly_v2.urdf.xacro`](../../../../hilda_ros/hilda_common/hilda_description/urdf/application_layers/ceiling_drilling/sledge_drilly_v2.urdf.xacro) — URDF source for `s_phys_max = 1.28`.
- [`02_variance_aware_clearance.md`](../../02_variance_aware_clearance.md) §Theory + §Implementation status — formula-of-record and the Phase A wire-in record.
- [ADR 0010](../../decisions/0010-clearance-field-package-boundary.md) — package boundary (this commit on the runtime side).
- [ADR 0009](../../decisions/0009-variance-pipeline-alpha-d-diagnostic.md) — variance-pipeline diagnostic (sets the `δ_cal` and operating-point context for the message's existing parameters).
- Memory: [[feedback-latching-spatial-samples]], [[feedback-log-severity-by-failure-class]], [[feedback-impl-status-sync]], [[feedback-adr-for-subtle-calls]].
