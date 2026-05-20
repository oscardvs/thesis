# 05 — Embedded deployment characterisation

## Scope

Stage 05 characterises the complete pipeline running on the target hardware: dual elevation mapping, fused-kernel clearance field, B-spline coefficient update, SMAC + 1D `s`-sweep warm-start, and the RHC solve, with cross-distro DDS between the Jetson Orin Nano Super (perception + global planning at 10 Hz) and the UDOO (controller at 20 Hz). The closure target is the budget in lit-review Table 6: roughly 45 ms worst-case across all stages, against a 50 ms latency ceiling and an 8 GB unified-memory budget under concurrent perception load. This stage closes G5: no reviewed work integrates dual-instance mapping, a constraint-field pipeline, and an embedded RHC on a unified-memory module under measured contention.

## Sources

- Literature Study, Section 9 (embedded deployment) and §11.5 (allocation across stages).
- Lit-review Table 6 — the per-stage timing budget on the Orin Nano Super.
- Enrico et al. (2025) — Jetson Orin Nano benchmark for acados NMPC (31 ms median) and GPU-parallelised MPPI (17.6 ms median) on a 12-state UAV model, controller-only.
- `ceiling_mapping_implementation.md` — the Phase 8 deployment plan and acceptance criteria for the perception side.

## Architectural commitments inherited

- Two-computer architecture: Jetson Orin Nano Super (8 GB unified LPDDR5) for perception + global planning; UDOO for state estimation, low-level control, and the RHC. Cross-distro bridging through a Jetson-side ROS 2 Jazzy Docker container.
- Asynchronous rates: mapping + constraint field at 10 Hz, controller at 20 Hz against the most recent constraint data.
- Power and clock pinning on the Jetson: 25 W Super mode, `jetson_clocks` for deterministic latency, `CUPY_GPU_MEMORY_LIMIT` and `CUPY_ACCELERATORS=cub` to bound and accelerate the CuPy workload.
- Profiling discipline: per-component timing, peak unified-memory occupancy, DDS communication latency across the dual-computer bridge — all required outputs of the characterisation phase.

## Open questions

- **DDS bridge.** The lit review references Zenoh on the Jetson-UDOO bridge with a 2 ms safety-monitor + DDS overhead allocation. Whether Zenoh, FastDDS, or CycloneDDS is the right choice under concurrent load on the Jetson is unbenchmarked.
- **Memory contention.** Two `elevation_mapping_cupy` instances, a CUDA kernel for the clearance field, and any learned traversability head all share the same 8 GB. The lit review names this as the open characterisation problem; the per-stage budget assumes contention is not the dominant cost.
- **Thermal envelope.** The implementation plan in `ceiling_mapping_implementation.md` flags a 85 °C threshold for thermal throttling. The 30-minute steady-state thermal behaviour under the full pipeline (not just the perception side) is not characterised.
- **Sim-vs-real timing.** The Gazebo evaluation in research-plan phase 4 will not exhibit the same DDS, CUDA-stream, or unified-memory pressure as the Jetson. The transition from simulation to embedded characterisation needs a defined methodology: which numbers carry over, which do not.
- **Failure modes.** No explicit policy for what the controller does if the perception side misses a 10 Hz cycle (hold last `f`? extrapolate? halt?). Define before the full pipeline runs against a perception fault injector.

## Cross-references

- 01 / 02 / 03 / 04 — every module contributes a row to Table 6. Each module's own profiling section should report its budget against the table.
- Hardware-validation phase of the research plan (§11.6 phase 6) uses an adjustable-height crossbar test rig and total-station ground truth at the industrial partner facility.
