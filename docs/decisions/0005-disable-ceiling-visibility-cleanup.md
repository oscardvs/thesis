# 0005 — disable-ceiling-visibility-cleanup

Status: accepted
Date: 2026-05-19
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

Option A is wrong on semantic grounds. The visibility cleanup, applied unchanged to the negated ceiling field, removes the lowest overhead surfaces — exactly the surfaces the ceiling instance exists to capture. The reasoning chain: a ray from the LiDAR through the body region to an overhead duct returns a hit at the duct height z_duct. In the unnegated frame this is z = z_duct (a high cell). In the negated frame it is z' = −z_duct (a low cell). The cleanup logic interprets any cell whose estimated height is *above* the most recent ray-traversed elevation as suspect, because a ray that traverses past such a height implies the height was wrong. With the negation, "above" reverses meaning: the lowest overhead surface (most negative z') becomes the cell most likely to be cleared by a subsequent ray that traverses through a higher (less negative) intermediate cell. The cleanup would systematically erase the binding-constraint surface.

Option C is a viable engineering route but pays a maintenance cost the project does not need. The cleanup kernel is internal to `elevation_mapping_cupy`; modifying it requires either a fork (forecloses upstream pulls — same argument as [0004](0004-splitter-as-external-node.md)) or upstream submission (out of scope timeline). The benefit — preserving the cleanup *function* on the ceiling — is real but not large: ceiling stale-observation behaviour can be handled by a variance-threshold invalidation plugin without touching the framework, and the ceiling sees substantially less occlusion-driven artefact than the floor because the upward LiDAR sees obstacles directly rather than past them.

Option B is the right call. The cost is that the ceiling map accumulates stale observations indefinitely under the default `time_variance` mechanism (the ROS 2 port of `elevation_mapping_cupy` lacks the `scanning_duration` cleanup of the ROS 1 version, see 01 open questions). The replacement mechanism — variance-threshold invalidation or a time-layer expiry — is a small additional plugin that lives in the same package as `CeilingDecodePlugin`, does not modify the framework, and can be evaluated independently against rosbag replays.

## Consequences

- Ceiling-instance config (`ceiling_complete.yaml`) sets `enable_visibility_cleanup: false`. Floor-instance config keeps the framework default.
- A stale-observation expiry mechanism for the ceiling instance becomes a project deliverable, not a free property of the framework. Tracked in 01's open-questions list.
- The asymmetric configuration (cleanup on for floor, off for ceiling) is non-obvious to a reader of the configs in isolation; an inline comment in `ceiling_complete.yaml` should point at this ADR.
- Any downstream module that assumes uniform behaviour across floor and ceiling instances (e.g. a generic GridMap diagnostics tool) must accommodate the asymmetry.
