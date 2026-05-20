# 00 — Pipeline overview

## Scope

The thesis builds a perception-to-control pipeline that lets HILDA navigate under variable overhead clearance and position itself for ceiling drilling, on embedded hardware. The pipeline runs in five stages: dual-layer elevation mapping (floor and ceiling, the latter encoded by z-inversion into the same `elevation_mapping_cupy` framework), a variance-aware clearance field that fuses both layers into a configuration-dependent feasibility scalar `f(x, y, s)`, a perceptive receding-horizon controller over the augmented state `(x, y, θ, v, ω, s)` with the sledge `s` as a first-class decision variable, an approach-aware inverse-reachability filter that verifies the entire approach corridor and not only the goal pose, and an embedded deployment on the Jetson Orin Nano Super under a 50 ms latency budget. Modules 01–05 develop each stage in isolation before integration.

## Source

- Literature Study, Section 11 (Proposed methodology) — canonical statement of the pipeline.
- Literature Study, Figure 5 — block diagram with update rates and compute allocation.
- Literature Study, Table 5 — prior work against the five gaps; identifies the joint coverage as open.

## Architectural commitments inherited from the lit review

- Augmented state `x = (x, y, θ, v, ω, s)`, control `u = (a, α, u_s)`. The sledge `s` is a continuous decision variable, not a discrete mode.
- Dual `elevation_mapping_cupy` instances share kernels, allocator, and a single CUDA context on one ROS 2 process.
- The clearance field `f(x, y, s) = c(x, y) − h_base − s − ε(x, y)` is the controller-agnostic interface between perception and any receding-horizon controller. Both the lead controller and any HMPC alternative consume this field.
- ε(x, y) is variance-aware: `ε = ε_base + δ_cal + λ·√(σ²_zceil + σ²_zfloor)` with λ = 3 (99.7 % chance constraint under the Kalman-Gaussian assumption). δ_cal is offline-fit to absorb under-reporting on texture-poor and oblique surfaces.
- Perception runs on the Jetson Orin Nano Super at 10 Hz; the controller runs on the UDOO at 20 Hz against the most recent perception output. Cross-distro DDS is handled inside a Jetson-side Docker container running ROS 2 Jazzy.

## Open questions

- The lit review (§11.3) chose gradient-based NMPC over hierarchical MPC and CBF-MPPI on architectural grounds. The thesis-stage convention in `THESIS.md` is to treat the controller family as the **lead candidate with the alternative kept viable through a controller-agnostic interface**. This wording matters for how 03 is framed and for which abstractions in `hilda_clearance_field` are public.
- Goal selection: a 6D inverse-reachability voxel grid vs. an analytical mast-aware filter exploiting the vertical-line reachable set. Deferred to 04.
- Semantic painting and 3DGS ceiling reconstruction are explicitly out of scope until 01 and 02 land geometrically.
- Whether the 1D `s`-sweep that warms the controller from the SMAC global plan runs as a node on the Jetson or as a method call inside the controller process — affects the cross-distro DDS budget in 05.

## Cross-references

- 01 — dual-layer elevation mapping (G1) — Section 4 + 11.1
- 02 — variance-aware clearance field (G2) — Section 6 + 11.2
- 03 — perceptive RHC over the augmented state (G3) — Section 7 + 11.3
- 04 — approach-aware inverse reachability (G4) — Section 8 + 11.4
- 05 — embedded deployment characterisation (G5) — Section 9 + 11.5
