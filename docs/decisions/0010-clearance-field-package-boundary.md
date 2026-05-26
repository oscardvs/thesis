# 0010 — clearance-field-package-boundary

Status: accepted
Date: 2026-05-26
Gap(s): G2
Module: 02_variance_aware_clearance.md

## Context

Two homes for the variance-aware clearance-field code exist on disk:

- `hilda_ceiling/ceiling_constraint_field/` — a working C++ ROS 2 node (`ConstraintFieldNode`) with `message_filters::Synchronizer` over the two GridMap topics, declared parameters `h_base` and a static `eps_safety`, and publishers for `/constraint_field` and `/ceiling_clearance`. The `syncCallback` field computation is a TODO. The package is listed as `exec_depend` in the `hilda_ceiling` metapackage; the sibling packages `ceiling_height_lookahead`, `ceiling_collision_monitor`, and `ceiling_controller` carry `constraint_field_topic: "/constraint_field"` in their config defaults.
- `hilda_clearance_field/` at workspace root — an empty `ament_python` skeleton (`package.xml`, `setup.py`, no entry points, no Python modules). THESIS.md commits to its existence on the grounds that "both NMPC and HMPC will consume from it"; the lit review §11.2 names it as the home for the variance-aware kernel and the CasADi B-spline export.

Phase-2 work — implementing `ε(x, y) = ε_base + δ_cal + λ √σ²_c` per 02 §Theory, with the per-bin freshness lookup and operating-point manifest of ADR 0009 — needs to start. The package boundary is the gate.

## Options

- **A — Absorb.** Move the C++ runtime node into `hilda_clearance_field`, switch its build type to `ament_cmake` (or hybrid), drop `ceiling_constraint_field` from the `hilda_ceiling` metapackage, and retarget the three sibling packages' configs and launchers. One home for everything clearance-field.
- **B — Split by role.** Keep the C++ runtime node in `ceiling_constraint_field` — its sibling pipeline expects it there, and it owns the GridMap publication + the on-GPU field computation. Narrow `hilda_clearance_field` to the controller-facing tooling: CasADi B-spline export, δ_cal calibration scaffolding, and a Python/CuPy prototype of the variance-aware kernel for sim-phase iteration before the C++ kernel is wired.
- **C — Delete `hilda_clearance_field`.** Promote `ceiling_constraint_field` to be the sole home; export the CasADi B-spline from a Python module nested inside it, host calibration scripts in a `scripts/` directory. Drops the top-level package THESIS.md committed to.

## Choice

B.

## Rationale

The binding question is which *artefacts* each package owns, not which directory they live in. Two distinct artefacts emerge from 02:

1. The runtime publication of `f` as a `grid_map_msgs/GridMap` on `/constraint_field`, plus the per-cycle `CeilingClearance` summary. Consumed over DDS by ceiling-pipeline siblings (lookahead, collision-monitor, ceiling-side controller), all physically co-located in `hilda_ceiling/`. The fused single-pass CUDA kernel of 02 §Design choices computes the field on-GPU inside this node — runtime, in-process, C++.

2. The controller-facing interface: the CasADi B-spline `Function` consumed in-process by the NMPC's acados problem builder or any HMPC drop-in, the δ_cal calibration scaffolding (offline, no ROS at runtime), and the operating-point manifest plus startup audit per ADR 0009 §Consequences. This is Python tooling on a different rebuild cadence from the runtime publisher.

Option A collapses both into one package. Cost: switch the package to `ament_cmake` (or hybrid), edit the metapackage, retarget three sibling configs and launchers, possibly re-publish under a new topic if the rename propagates. Benefit: one home. The benefit reads as cleaner-on-paper, but the runtime node and the offline calibration tooling do not share dependencies, build systems, or rebuild cadence; combining them obscures the boundary between "deployed at 10 Hz" and "fitted once before deployment."

Option C drops the top-level package. The CasADi B-spline could be exported from a Python module inside `ceiling_constraint_field`, and calibration scripts could sit under `scripts/`. Strictly possible. The cost is that controller packages — consumed by either the lead NMPC or any HMPC drop-in — would then `<depend>` on a package nested inside the ceiling metapackage to acquire the spline interface, hiding what the controllers actually consume and making a later swap of the interpolant framework (e.g. CasADi → PyTorch for an HMPC variant) harder to localise.

Option B leaves working code where its DDS siblings expect it and makes `hilda_clearance_field` the home of what is genuinely new (variance-aware kernel prototyping in Python first, CasADi export, calibration). The controllers depend on `hilda_clearance_field` for the interface they consume in-process (the spline + the calibration manifest), not on the runtime publishing node. The publishing node remains the DDS source for downstream ceiling-pipeline consumers — including, in principle, the controllers if they preferred the topic interface, though in practice the in-process spline displaces that subscription.

The variance-aware kernel itself splits the same way. Production form: C++/CUDA inside `ceiling_constraint_field`, fused single-pass per 02 §Design choices. Sim-phase prototype: Python/CuPy inside `hilda_clearance_field`, exercised against `compute_ceiling_metrics.py` and the existing experiment runner. Both implementations must agree on the formula; the YAML config and the calibration manifest are the source of truth. When the C++ kernel lands and matches the prototype, the Python version becomes a regression-test reference. The pattern is the one already used by the splitter — production C++ in `hilda_ceiling/ceiling_pointcloud_splitter/`, Python prototype in `elevation_mapping_cupy/elevation_mapping_cupy/ceiling_pointcloud_splitter.py` (ADR 0004 §Consequences).

## Consequences

- **No code moves.** `hilda_ceiling/ceiling_constraint_field/` stays where it is. Phase-2 variance-aware kernel work goes into its `syncCallback` and config: `eps_safety` widens into the `ε = ε_base + δ_cal + λ √σ²_c` machinery; new parameters `eps_base`, `delta_cal` (or `delta_cal[k]` for the per-bin freshness lookup of ADR 0009 §F2), `lambda`, plus the operating-point manifest path.
- **`hilda_clearance_field/` scope is committed to three deliverables.**
  1. *CasADi B-spline export.* A Python module that builds the cubic tensor-product spline of 02 §Design choices from the latest `/constraint_field` snapshot and emits a `CasADi::Function` consumable by the acados problem builder. Transport mechanism (shared file vs in-process import vs DDS parameter blob) is the 05 question; the export API is fixed here.
  2. *δ_cal calibration scaffolding.* Corpus management (loader, splitter for the dense/sparse and per-surface-class partitions of 02 §Calibration protocol), residual fitter with freshness-conditional binning per ADR 0009, manifest writer recording `time_variance`, `update_variance_fps`, `outlier_variance`, and `mahalanobis_thresh` as locked co-parameters.
  3. *Python/CuPy prototype kernel.* Reference implementation of the variance-aware field, exercised in sim against `compute_ceiling_metrics.py`. Becomes a regression-test reference once the C++ runtime catches up.
- **Operating-point audit straddles the boundary.** The manifest is produced by `hilda_clearance_field`'s calibration scaffolding; the audit consumes it in `ceiling_constraint_field` at startup and refuses to publish `f` until live ceiling-config values match the manifest (ADR 0009 §Consequences). The two packages share a manifest schema, not code paths.
- **Topic interface unchanged.** `/constraint_field` and `/ceiling_clearance` keep their current names and payload contracts. `ceiling_height_lookahead`, `ceiling_collision_monitor`, and `ceiling_controller` need no retargeting.
- **Controller packages depend on `hilda_clearance_field`, not on `ceiling_constraint_field`.** When `hilda_nmpc` lands its `package.xml` carries `<depend>hilda_clearance_field</depend>` for the in-process spline plus calibration manifest, and `<exec_depend>hilda_ceiling</exec_depend>` for the runtime metapackage that transitively brings up the publishing node. The same applies to any HMPC drop-in.
- **Naming friction acknowledged.** `hilda_clearance_field` no longer hosts the clearance-field *computation*, only the controller-facing *interface* to it. The package description in `package.xml` updates to reflect this (the current description overstates the scope by listing the GPU kernel). Analogue: `ros2_numpy` is the ROS-side interface to NumPy without hosting NumPy.
- **Flip condition.** If the embedded measurement (G5) shows the GridMap publication overhead dominates the controller's interface latency, the in-process B-spline export migrates into the runtime node directly, the publication step becomes a debug-only output, and the package boundary dissolves. The flip would also require retargeting the three `hilda_ceiling` siblings that currently consume `/constraint_field` over DDS (`ceiling_height_lookahead`, `ceiling_collision_monitor`, `ceiling_controller`); that retargeting cost is the one Option B *defers*, not eliminates. Not expected at 02 §Table 6's 0.1 ms field budget against ms-scale DDS serialisation, but named so the reasoning can be re-opened against the measurement rather than re-derived.
