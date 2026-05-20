# 0004 — splitter-as-external-node

Status: accepted
Date: 2026-05-19
Gap(s): G1
Module: 01_dual_elevation_mapping.md

## Context

The dual-instance dual-layer mapping needs a per-point classifier that separates incoming LiDAR returns into a floor stream and a ceiling stream, so each `elevation_mapping_cupy` instance subscribes to its own input. Two construction routes are available: a stand-alone pre-fusion node that consumes the fused LiDAR cloud and republishes two filtered clouds, or a patch into the upstream `elevation_mapping_cupy` framework that ingests one cloud and routes points to two internal map instances. The classifier itself is the same in both routes — a URDF-anchored z-threshold with a dead-band; the question is where it executes.

## Options

- **A — External splitter node.** A `ceiling_pointcloud_splitter` node subscribes to `/perception/fused_points`, classifies on the GPU (CuPy boolean indexing as the reference path, custom warp-aggregated `RawKernel` as the optimisation target), and publishes `/lidar/floor_points` and `/lidar/ceiling_points`. Each `elevation_mapping_node` is a stock instance pointed at its own input topic. No upstream patch.
- **B — Internal modification.** Fork `elevation_mapping_cupy` and modify the dual-instance entry point to ingest one cloud, classify inside the framework, and dispatch points to two internal maps. Saves one serialisation pass.

## Choice

A — external splitter node.

## Rationale

The binding cost on this choice is **maintenance posture, not runtime cost**. Option B forks an actively maintained framework: every upstream pull becomes a merge against the dispatch modification, and every framework refactor that touches the input path requires re-doing the patch. On a thesis timeline with a published research direction that depends on upstream evolution (Erni 2023 multi-modal extensions, learned traversability filters), that cost compounds. Option A leaves the framework untouched, so each instance benefits from upstream maintenance without per-pull effort.

The runtime cost of A is small. The splitter runs as a CuPy GPU kernel (1–2 ms in the reference indexing path on the Orin Nano Super, targeting 0.3–0.5 ms with a warp-aggregated `RawKernel`). The serialisation cost of republishing `sensor_msgs/PointCloud2` for ~120K points at 10 Hz is negligible on the Jetson's unified-memory architecture, where `cp.asarray(np_array)` is a logical copy on the same physical memory. The splitter operates on the raw byte buffer and preserves auxiliary fields (intensity, ring, timestamp), so downstream nodes that consume the same fused cloud for other purposes are unaffected.

The architectural benefit is independent testability: the splitter has its own unit-test suite, its own config (`splitter_params.yaml`), and its own lifecycle, decoupled from the framework's. Migration into the elevation-mapping callback remains available if profiling later shows it matters — the externalisation does not foreclose the internal route.

## Consequences

- The runtime layer for production deployment lives under `hilda_ceiling/ceiling_pointcloud_splitter/` as a C++ node; the Python prototype remains in `elevation_mapping_cupy/elevation_mapping_cupy/ceiling_pointcloud_splitter.py` for experimental work. Both must agree on the classifier semantics and the dead-band thresholds; configuration is the source of truth (`splitter_params.yaml` for production, `dual_ceiling_mapping.yaml` for the prototype).
- The two `elevation_mapping_node` instances are stock, so per-instance configuration (e.g. `enable_visibility_cleanup: false` on the ceiling instance, see [0005](0005-disable-ceiling-visibility-cleanup.md)) is YAML-only and survives upstream pulls.
- An adjacent splitter for the upward-facing RealSense (already added in `hilda_ceiling`) follows the same architectural pattern; the choice generalises.
