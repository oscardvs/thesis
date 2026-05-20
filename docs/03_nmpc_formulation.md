# 03 — Perceptive RHC over the augmented state

## Scope

Stage 03 builds a receding-horizon controller that solves at every cycle an optimal control problem over the augmented state `x = (x, y, θ, v, ω, s)` with controls `u = (a, α, u_s)`, where `s` is the combined sledge extension acting both as a transit-clearance variable and a per-target task variable. The clearance field `f(x, y, s) ≥ 0` from 02 enters as a path constraint via a C¹ CasADi B-spline interpolant; the cost balances time-to-goal, separately weighted input-rate penalties on drive acceleration and sledge rate (reflecting the order-of-magnitude actuator-bandwidth gap), a relaxed log-barrier on f, and a terminal task cost on the drilling pose with `s = s_goal` active when a drill target is the current goal. Global guidance comes from the SMAC Lattice planner on a ceiling-aware floor-traversability costmap, post-processed by a 1D forward-backward `s`-sweep that supplies the primal warm start. This stage closes G3.

## Sources

- Literature Study, Section 7 (planning architectures) and §11.3 (the chosen formulation).
- Local kinematic and OCP notes in `~/isaac_ros2_ws/src/hilda_ros/hilda_common/hilda_kinematics/docs/`:
  - `kinematic_model.md` — full 10-DOF configuration space and reduction to the planar augmented model
  - `NMPC.md` / `refined.md` — v3 single-layer NMPC pipeline (Grandia-style multiple-shooting SQP-RTI with relaxed-barrier soft constraints)
  - `HMPC.md` — earlier v2 PMPC+TMPC formulation, retained for reference only
  These files were written for a different workspace and have not been merged into thesis-stage notes. Treat as input, not authority; reconcile against the lit review's §11.3 when populating this module. **They reference an internal platform codename — strip on import; use "the platform" or "HILDA" instead.**

## Architectural commitments inherited

- Augmented state and control as above. Unicycle kinematics with empirically calibrated ICR factors for skid-steer; first-order integrator on `s`.
- Soft path constraint `f ≥ 0` via L1 slack penalty; preserves solver feasibility under transient perception artefacts.
- Cost terms: time-to-goal, input-rate (`||a||²`, `||α||²`, `||u_s||²`) with separate weights, relaxed log-barrier on `f`, terminal `||s(T) − s_goal||²` when a drill target is active.
- Solver: acados SQP-RTI, 20 Hz target, calibrated against the 31 ms median reported for the Orin Nano in Enrico et al. (2025) on a 12-state UAV model.
- Global guidance: SMAC Lattice on the floor costmap with a ceiling-aware lethal layer keyed to the current `s`; 1D `s`-sweep on the resulting waypoints supplies the warm start.
- The controller runs on the UDOO; only the constraint-field interface crosses the Jetson/UDOO boundary. Implication: the OCP definition and the warm-start pipeline live on the UDOO side, not the Jetson.

## Open questions

- **Controller family.** The lit review (§7.7, §11.3) chose gradient-based NMPC over hierarchical MPC and CBF-MPPI. The thesis-stage convention in `THESIS.md` keeps the alternatives viable through a controller-agnostic interface (the C¹ field `f` plus optional plane-segment halfspaces). The OCP scaffolding here should therefore not foreclose an HMPC drop-in. Decide via embedded-load profiling in 05; do not commit by code structure before then.
- **ICR calibration.** The skid-steer ICR factors are "empirically calibrated" in the lit review but no procedure or fitted values are written down. Define the calibration manoeuvre, the optimisation objective, and the validation envelope before the controller is profiled in simulation.
- **Horizon and discretisation.** The lit review names acados SQP-RTI and a 5–20 ms solve target but does not commit to a horizon length or shooting node count. Pick during phase 3 of the research plan.
- **Warm-start fidelity.** Whether the 1D `s`-sweep on the SMAC plan is a good warm start when the sweep and the OCP disagree on which side of a ceiling feature to pass — the sweep is unaware of `(v, ω)` dynamics. Investigate during sim evaluation; may motivate a second warm-start branch.
- **Soft-vs-hard constraint policy.** The proposal uses a soft `f ≥ 0` with L1 slack and a separate log-barrier in the cost. The interaction between the two under heavy transient perception noise is not analysed in the lit review.

## Cross-references

- 02 — variance-aware clearance field (supplies `f` and its CasADi B-spline)
- 04 — approach-aware IRM (consumes the OCP forward-simulation to verify approach-corridor feasibility)
- 05 — embedded deployment (the controller is the dominant latency contributor in Table 6)
