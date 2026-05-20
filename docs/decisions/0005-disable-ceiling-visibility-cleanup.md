# 0005 — disable-ceiling-visibility-cleanup

Status: accepted
Date: 2026-05-19 (rationale sharpened 2026-05-20 against kernel source)
Gap(s): G1
Module: 01_dual_elevation_mapping.md

## Context

`elevation_mapping_cupy` includes a Bresenham-style visibility-cleanup step in its update cycle, controlled by `enable_visibility_cleanup`. The operation runs raycasts from sensor origin through cells along each ray and clears cell estimates that are inconsistent with the ray having reached a farther cell — designed to remove spurious "phantom obstacle" elevations caused by points seen above an existing cell, which is the dominant artefact in floor mapping when overhanging structure (cable trays, mezzanines, soffits) intermittently occludes the floor surface. The framework default is `enable_visibility_cleanup: true`. The dual-instance design encodes the ceiling layer by negating point z-coordinates before insertion, so the framework operates on a field whose semantics are inverted relative to its design assumption.

## Options

- **A — Leave visibility cleanup enabled on the ceiling instance** with the default raycasting behaviour.
- **B — Disable visibility cleanup on the ceiling instance** (`enable_visibility_cleanup: false`) and handle stale ceiling observations through a separate mechanism (variance-threshold invalidation or a time-layer expiry).
- **C — Modify the raycasting kernel** to invert its consistency check on the ceiling instance, so it removes phantom *low* surfaces rather than phantom *high* ones, preserving the cleanup behaviour with reversed semantics.

## Choice

B — disable on the ceiling instance; keep the framework default on the floor instance.

## Rationale

Option A is wrong on semantic grounds, and the failure mode is concrete. Inside the framework's `add_points_kernel` at `custom_kernels.py:204`, the cleanup is gated by `enable_visibility_cleanup`. Within the gated block, each ray from sensor origin to a measured point sweeps intermediate grid cells; at each intermediate cell the operative check is

```c
if (nmap_h > nz + 0.01 - min(nmap_v, 1.0) * 0.05) {
    // ... cell is penetrated by the ray; remove it
}
```

where `nmap_h` is the cell's stored elevation and `nz` is the ray's z-coordinate at that grid position. The semantic is: if a cell's stored height is above the ray height at that cell, the ray would have passed through the height we estimated, so the estimate was stale.

The splitter for the dual-instance design publishes ceiling-classified points in the world frame with z-coordinates negated, so the ceiling instance sees `t[2] = 0` (sensor-frame = `odom` = `map_frame` after the splitter; no further transform) and points at z-values in `(−∞, −z_high]` where `z_high` is the splitter's ceiling-band lower bound (2.05 m in the current HILDA config). For an existing ceiling cell with stored estimate `nmap_h ≈ −2.0` (a ceiling at 2.0 m in sign-restored terms), a new ray to a *higher* ceiling measurement at `−2.5` traverses intermediate cells with `nz` decreasing from 0 toward `−2.5`. At the cell where the prior `−2.0` estimate lives, the ray height `nz` passes through some value between 0 and `−2.5`, say `−2.3`. The check becomes `nmap_h > nz`, i.e., `−2.0 > −2.3 = true` — the cell is cleared.

In sign-restored terms: a previously-observed low overhead obstacle at 2.0 m gets erased the moment the LiDAR sees higher ceiling (2.5 m) beyond it. For drilling clearance reasoning this is the safety-conservative wrong direction — the lower obstacle is the binding constraint and clearing it could let the planner push the column to a height that the cleared (real) overhead would obstruct. The cleanup *can* fire on the ceiling instance and *does* clear the cells we most need to preserve.

A secondary cost of Option B is surfaced by the same source audit: the upper-bound update for invalid (unobserved) cells at `custom_kernels.py:235–241` lives *inside* the `enable_visibility_cleanup` conditional. Disabling cleanup also disables that update. After sign restoration, an enabled-cleanup upper-bound layer on the ceiling instance would have given a "highest height reached by rays without finding ceiling" per unobserved cell — a useful lower bound on ceiling for cells the LiDAR has not yet hit, which 04's IRM admissibility step could consume to reason about feasibility at goal poses near unobserved overhead. Choosing B forecloses this interface. The trade-off is real: safety-conservative ceiling preservation versus a useful lower-bound layer for unobserved cells. The safety direction wins for now; revisit when 04's interface is committed.

Option C is a viable engineering route but pays a maintenance cost the project does not need. The cleanup kernel is internal to `elevation_mapping_cupy`; modifying it requires either a fork (forecloses upstream pulls — same argument as [0004](0004-splitter-as-external-node.md)) or upstream submission (out of scope timeline). The benefit — preserving the cleanup *function* on the ceiling with reversed semantics — is real but the same outcome (safe-conservative ceiling preservation) can be achieved by Option B plus an external stale-observation plugin, without touching framework code.

Option B is the right call. The cost is that the ceiling map accumulates stale observations indefinitely under the default `time_variance` mechanism (the ROS 2 port of `elevation_mapping_cupy` lacks the `scanning_duration` cleanup of the ROS 1 version, see 01 open questions). The replacement mechanism — variance-threshold invalidation or a time-layer expiry — is a small additional plugin that lives in the same package as `CeilingDecodePlugin`, does not modify the framework, and can be evaluated independently against rosbag replays.

## Consequences

- Ceiling-instance config (`ceiling_complete.yaml`) sets `enable_visibility_cleanup: false`. Floor-instance config keeps the framework default.
- The upper-bound-update foreclosure is real and adds a re-evaluation point when 04's IRM admissibility interface is committed — if 04 needs a "lower bound on ceiling at unobserved cells" interface, the trade-off here must be reconsidered (options: accept the lower-cell-clears risk; build a separate ray-cast pass that does only the upper-bound update without the clearing check; build the lower bound at the constraint-field stage from another source). Tracked in 01's visibility-cleanup section.
- A stale-observation expiry mechanism for the ceiling instance becomes a project deliverable, not a free property of the framework. Tracked in 01's open-questions list.
- The asymmetric configuration (cleanup on for floor, off for ceiling) is non-obvious to a reader of the configs in isolation; an inline comment in `ceiling_complete.yaml` points at this ADR.
- Any downstream module that assumes uniform behaviour across floor and ceiling instances (e.g. a generic GridMap diagnostics tool) must accommodate the asymmetry.
