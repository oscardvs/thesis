# 0011 — ceiling-layer-selection-and-hbase-datum

Status: accepted
Date: 2026-05-29
Gap(s): G2
Module: 02_variance_aware_clearance.md

## Context

A review of the Phase-A constraint-field runtime (2026-05-29) found two coupled defects in `ceiling_constraint_field/ConstraintFieldNode` that the sign-off did not cover. The signed-off `sim_validation_02a/02b` experiments run through the Python `compute_ceiling_metrics.py` path (which decodes the ceiling correctly via `h_ceil = −elevation`), and the C++ node has no integration test exercising `syncCallback` end to end, so neither caught the node path.

1. **Wrong ceiling layer.** The node reads a single `elevation_layer` parameter (default `elevation`) from *both* the floor and ceiling GridMaps (`constraint_field_node.cpp:220`) and computes `clearance = z_ceil − z_floor`. But the ceiling instance's `elevation` layer holds *negated* z by construction — the splitter negates ceiling points so the max-height framework captures the lowest overhead surface (`ceiling_pointcloud_splitter.py:135`; `ceiling_complete.yaml` header). The decoded positive layer `ceiling_height` (h_ceil > 0) is published explicitly "for downstream consumers (Phase 2 constraint field)" (`ceiling_plugin_config.yaml`) but never read. The node therefore computes `clearance = (−z_ceil) − z_floor` — sign-inverted for every cell.

2. **h_base datum mismatch.** `f = c − h_base − s − ε`. The runtime config sets `h_base = 0.85`, but 03 §Notation defines `h_base ≈ 1.99 m` (mast-top height above floor at nominal standing, folding the column-base-to-mast-top offset). The driving-column collision box alone is 1.64 m tall (`sledge_drilly_v2.urdf.xacro`) and the drill-tip offset is 1.889 m (`calibration_drilly_v2.yaml` `T_drill_center__drill_tip`), so 0.85 m is physically impossible as a mast-top height. It under-counts the platform by ≈ 1.14 m, biasing `f` optimistic — the dangerous direction.

The two are coupled: fixing the layer changes what `z_ceil` denotes, so `h_base` must sit on the same datum or `f` stays biased.

## Options

Layer selection:
- **A — per-map layer parameter.** Add `ceiling_elevation_layer` (default `ceiling_height`); keep `elevation_layer` for the floor. Floor reads `elevation`, ceiling reads `ceiling_height`, presence-checked per map.
- **B — negate in the callback.** Keep one `elevation_layer`; negate the ceiling layer inside the node before the subtraction. Re-implements the decoding the upstream `ceiling_decode` plugin already does; couples the consumer to the splitter's sign convention.
- **C — read both, prefer `ceiling_height`.** Fall back to negating `elevation` if `ceiling_height` is absent. Hidden control flow; masks an upstream-config error as a silent recovery.

h_base datum:
- align the runtime config to the authoritative 03 datum (≈ 1.99 m), versus leaving the 0.85 m sim placeholder.

## Choice

Layer: **A** — separate `ceiling_elevation_layer` parameter defaulting to `ceiling_height`.
Datum: align config `h_base` to the 03 datum (1.99 m), pending precise FK confirmation against `kinematic_model.md` §5.2.

## Rationale

A single shared layer parameter cannot serve both maps: the floor needs the un-negated `elevation`, the ceiling needs the decoded positive `ceiling_height`. Option A makes the per-map contract explicit in config and is presence-checked per map, so an upstream rename fails loud rather than silently mis-reading. Option B duplicates the `ceiling_decode` plugin's job inside the consumer; if the upstream ever publishes an un-negated ceiling, the node silently double-negates. Option C's fallback turns a config error into a silent recovery — the opposite of the fail-loud discipline the rest of the node follows (layer-presence check, geometry check). The `ceiling_height` layer already exists in the ceiling publisher's layer set (`ceiling_complete.yaml`: `['elevation', 'variance', 'ceiling_height']`), so no upstream change is needed.

On the datum: per [authoritative-source precedence] the design docs and lit review outrank a sim config value. 03 §Notation is the authority; the URDF/calibration corroborate the magnitude (1.889 m drill-tip offset + base height ≈ 1.99 m). The config is aligned to the doc, not the reverse. The exact figure traces to `kinematic_model.md` §5.2 and the full base_link→mast-top FK; 1.99 m is the doc-stated nominal and is flagged for precise confirmation. The fix is in the safe (conservative) direction regardless of the exact value: a larger `h_base` makes `f` smaller, never optimistic.

Validated by a new node-level integration test (`test_build_constraint_field.cpp`) feeding a floor + ceiling GridMap pair — ceiling `elevation` negated, `ceiling_height` positive — through the extracted `buildConstraintField`, asserting the output clearance equals the true geometric clearance (positive). The bug was invisible because no test exercised the assembled callback; the test reproduces it (fails on the negated read) before the fix lands.

## Consequences

- New parameter `ceiling_elevation_layer` (default `ceiling_height`) in the node and `constraint_field_params.yaml`. The floor's `elevation_layer` is unchanged. Presence is checked per map (floor: `elevation` + `variance`; ceiling: `ceiling_height` + `variance`).
- `syncCallback`'s map-processing core is extracted into `buildConstraintField(floor, ceiling, out)` to create a testable seam; behaviour is otherwise unchanged. New gtest target `test_build_constraint_field` links `constraint_field_node_lib`.
- `h_base` default and config value change 0.85 → 1.99. **This changes feasibility outputs on the sim bag** — more cells read infeasible, correctly, because the platform is taller than the placeholder implied. Any sim result quoting `is_safe` / `feasibility` / `max_feasible_height` against the old 0.85 is superseded; the variance-margin numbers (02a/02b tightening, ε quantiles) are unaffected, as they never used `h_base`.
- The precise `h_base` remains a flagged reconciliation against `kinematic_model.md` §5.2; 1.99 m is the 03 nominal, conservative, and to be confirmed.
- Phase-A "signed off" is amended: the kernel ε-math and publish rate stood, but geometric clearance through the node was wrong. Re-verification on the bag through the node (not the Python metrics path) is the next experimental step.
- The dual-instance origin-resample TODO (production) and the live-`s` wiring remain open and unchanged.
