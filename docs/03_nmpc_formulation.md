# 03 — Perceptive RHC over the augmented state

## Scope

Stage 03 builds a receding-horizon controller that solves at every cycle an optimal control problem over the augmented state `x = (x, y, θ, v, ω, s)` with controls `u = (a, α, u_s)`, where `s` is the combined sledge extension acting both as a transit-clearance variable and a per-target task variable. The clearance field `f(x, y, s) ≥ 0` from 02 enters as a path constraint via a C¹ CasADi B-spline interpolant; the cost balances time-to-goal, separately weighted input-rate penalties on drive acceleration and sledge rate (reflecting the order-of-magnitude actuator-bandwidth gap), a relaxed log-barrier on f, and a terminal task cost on the drilling pose with `s = s_goal` active when a drill target is the current goal. Global guidance comes from the SMAC Lattice planner on a ceiling-aware floor-traversability costmap, post-processed by a 1D forward-backward `s`-sweep that supplies the primal warm start. This stage closes G3.

## Sources

- Literature Study, Section 7 (planning architectures) and §11.3 (the chosen formulation). **Authoritative for architecture and the variance-aware-clearance interface from 02.**
- Local kinematic and OCP notes in `~/ros2_ws/src/hilda_ros/hilda_common/hilda_kinematics/docs/` (also mirrored under `~/isaac_ros2_ws/` and `~/hilda_ws/`):
  - `kinematic_model.md` — full 10-DOF configuration space, FK/IK chains, Jacobians, limb + drive + column models
  - `NMPC.md` / `refined.md` (byte-equivalent v3 copies) — single-layer NMPC pipeline (Grandia-style multiple-shooting SQP-RTI with relaxed-barrier soft constraints), plane-segmentation constraints
  - `HMPC.md` — earlier v2 PMPC+TMPC formulation, marked superseded by NMPC.md v3
- **Status of the local notes.** Authored 2026-04-16, predating the lit-review's surviving-three-families synthesis (§7.7) and 02's variance-aware ε. Treat as **math-extraction sources, not architectural authorities**: the §Geometric primitives below promote the coordinate-system-agnostic content (lever arm, floor coupling, ground clearance, stability, kinematic-model hierarchy, controller-hierarchy mapping); the policy items NMPC.md v3 unilaterally picked (7-state-with-h vs 6-state, RANSAC-plane polytopes vs C¹ B-spline of f, hard vs soft ceiling, NMPC vs HMPC) are explicitly **not endorsed here** and stay open per §Open questions. They reference an internal platform codename — strip on import; use "the platform" or "HILDA" instead.

## Architectural commitments inherited

- Augmented state and control as above. Unicycle kinematics with empirically calibrated ICR factors for skid-steer; first-order integrator on `s`.
- Soft path constraint `f ≥ 0` via L1 slack penalty; preserves solver feasibility under transient perception artefacts.
- Cost terms: time-to-goal, input-rate (`||a||²`, `||α||²`, `||u_s||²`) with separate weights, relaxed log-barrier on `f`, terminal `||s(T) − s_goal||²` when a drill target is active.
- Solver: acados SQP-RTI, 20 Hz target, calibrated against the 31 ms median reported for the Orin Nano in Enrico et al. (2025) on a 12-state UAV model.
- Global guidance: SMAC Lattice on the floor costmap with a ceiling-aware lethal layer keyed to the current `s`; 1D `s`-sweep on the resulting waypoints supplies the warm start.
- The controller runs on the UDOO; only the constraint-field interface crosses the Jetson/UDOO boundary. Implication: the OCP definition and the warm-start pipeline live on the UDOO side, not the Jetson.

## Geometric primitives

These six items are platform geometry and physics — they apply regardless of the state-vector choice, constraint-encoding choice, or controller-family choice that the §Architectural commitments + §Open questions still gate. Each is adapted from `kinematic_model.md` and `NMPC.md` v3 with the math intact, reconciled against the lit-review's 6-state commitment by making the dependence on the base-height variable explicit so the same derivations carry through if §Open questions later promotes `h` to a 7th state.

### Notation

Two distinct quantities share the symbol `h` across the local notes and the thesis-stage docs; the clash is non-obvious and worth pinning here once.

- `h` (kinematic_model.md, NMPC.md): the *limb-derived base_link height above floor contact*, range ~0.083–0.233 m, equal to `l sin θ_i + r − h_0` on flat ground.
- `h_base` (02 §Theory, lit-review §11.3 / Eq. 8): the *platform base height used in the clearance balance*, ≈ 1.99 m at nominal standing — this folds the column–base-to-mast-top offset of 1.899 m (kinematic_model.md §5.2) into a single constant.

To keep the primitives below state-vector-agnostic, define

$$H(h, s) \;:=\; h \;+\; 1.899 \;+\; s$$

as the body-frame mast-top height above floor contact. In the 6-state regime (h fixed at the platform nominal `h₀`), `H(h₀, s) = h_base^{(02)} + s` and 02's feasibility scalar can be re-stated as `f = c − H − ε` without coordinate change. In the 7-state regime, `H` depends on the additional decision variable.

### 1. Mast-top world-frame position (lever arm under tilt)

The robot's topmost point in world coordinates is **not** `(x, y, H)`. Under base pitch α and roll β:

$$\mathbf{r}_{\text{top}}^{W} \;=\; \begin{pmatrix} x \;-\; H \sin\alpha \\ y \;+\; H \sin\beta \\ z_{\text{floor}}(x, y) \;+\; H \cos\alpha \cos\beta \end{pmatrix}$$

Quantitative consequence at full extension (`H ≈ 4.08 m`): 1° of pitch displaces the mast top by `H sin(1°) ≈ 7 cm` horizontally. The 02 feasibility check `f ≥ 0` is implicitly evaluated at the *base-link* `(x, y)`; under non-zero tilt the *mast top* is at `(x − H sinα, y + H sinβ)`, so the same `(x, y)` cell of f corresponds to a tilted offset overhead and the encoded margin is off by the lever-arm contribution. The thesis-stage 03 currently does not model this; depending on the encoding chosen in §Open questions, the fix is either (a) a tilt-dependent coordinate shift before the f lookup (B-spline branch) or (b) folding γ_τ-style tilt factors into half-space coefficients (polytope branch — NMPC.md §2.3 Appendix A.3 form).

### 2. Floor-coupling: local linearisation per prediction stage

The clearance balance `c(x, y) = z_ceil(x, y) − z_floor(x, y)` collapses the floor into f only when `z_floor` is a constant. On any non-flat patch (ramps, slab seams, the platform's own ride-height variation as it climbs), `z_floor(x, y)` is non-linear in the decision variables and the constraint loses linearity in (x, y). The principled fix is to linearise per prediction stage around the warm-started position `(x̄_τ, ȳ_τ)`:

$$z_{\text{floor}}(x, y) \;\approx\; \underbrace{z_{\text{floor}}(\bar x_\tau, \bar y_\tau)}_{z_{0,\tau}} \;+\; \underbrace{\left.\frac{\partial z_{\text{floor}}}{\partial x}\right|_{(\bar x_\tau, \bar y_\tau)}}_{m_{x,\tau}}(x - \bar x_\tau) \;+\; \underbrace{\left.\frac{\partial z_{\text{floor}}}{\partial y}\right|_{(\bar x_\tau, \bar y_\tau)}}_{m_{y,\tau}}(y - \bar y_\tau)$$

The gradient `(m_x, m_y)` is computed by finite differences on the same `floor` layer 02 consumes. Effort estimate: one extra layer query + two finite-difference subtractions per `(stage, plane)` pair — negligible against the OCP-solve cost. The B-spline branch absorbs this automatically through the spline's bivariate `(x, y)` dependence; the polytope branch needs the linearisation done explicitly in the coefficient build.

### 3. Ground-clearance corner bound (tilt-dependent lower bound on h)

The base is a rectangle of half-dimensions `L_b/2 = 0.435 m` (forward) and `W_b/2 = 0.34 m` (lateral); the lowest structural surface sits `δ_b` below the `base_link` origin (URDF collision box: 0.039 m; conservative pending physical measurement: 0.050 m). The four corners drop a tilt-dependent offset below the centre, and the minimum corner clearance is

$$c_{\min} \;=\; (h - \delta_b) \;-\; \tfrac{L_b}{2}|\sin\alpha_\tau| \;-\; \tfrac{W_b}{2}|\sin\beta_\tau|.$$

Requiring `c_min ≥ 0.05 m` yields a time-varying lower bound

$$\boxed{\,h \;\geq\; h_{\min}(\alpha_\tau, \beta_\tau) \;:=\; 0.05 \;+\; \delta_b \;+\; \tfrac{L_b}{2}|\sin\alpha_\tau| \;+\; \tfrac{W_b}{2}|\sin\beta_\tau|\,}$$

parameterised by IMU at each stage. Numerical sample (δ_b = 0.050 m): flat-level → 0.100 m; 3° pitch → 0.123 m; 3° pitch + 2° roll → 0.131 m; 5° pitch → 0.138 m. In the 6-state regime where `h` is held fixed, this constraint is enforced **outside the NMPC** by the standing-height arbitrator (`reference_controller` + `BaseStability`); the NMPC just needs to know the active `h` value when computing H. In the 7-state regime, it is a linear box constraint on the h state — trivial for the QP backend.

### 4. Static stability constraint (acceleration–extension coupling)

Extending the mast raises the centre of mass, narrowing the safe acceleration envelope before the zero-moment point exits the support polygon. Quasi-static condition:

$$|a| \;\leq\; a_{\max}(h, s) \;=\; \frac{g \cdot x_{\text{support}}}{H(h, s)},$$

with `x_support` the distance from the projected CoM to the nearest support-polygon edge (≈ wheelbase/2 ≈ 0.25 m for longitudinal tipping; track/2 for lateral). Numerical sample (nominal h, x_support = 0.25 m): retracted (s = 0) → `a_max ≈ 1.23 m/s²`; full extension (s = 2.18 m) → `a_max ≈ 0.59 m/s²` — half. NMPC.md §2.4 records two implementation options: (A) the exact non-linear inequality `|a| · H − g·x_support ≤ 0`, evaluated by acados' SQP Jacobian at every iteration, polytopic structure broken; or (B) a linear over-approximation `|a| ≤ a₀ − (a₀ − a_min)/s_max · s` connecting the (s = 0) and (s = s_max) limits, polytopic structure preserved at modest conservatism cost. Pick during phase-3 implementation — both are admissible.

### 5. Kinematic-model hierarchy (skid-steer planar prediction)

HILDA's skid-steer drive cannot satisfy the no-lateral-slip assumption of the standard unicycle during turns. NMPC.md §1.6 enumerates four model levels, ranked by fidelity and cost:

| Level | Form | States | Suitability |
|---|---|---|---|
| 0 — Pure unicycle | `ẋ = v cos ψ, ẏ = v sin ψ, ψ̇ = ω` | 3 + morph | Differential-drive; physically incorrect for skid-steer during turns |
| **1 — ICR-corrected (leading)** | `ẋ = η_v v cos ψ, ẏ = η_v v sin ψ, ψ̇ = η_ω ω` | 3 + morph | Two scalar correction factors absorb ICR shift; same code path as Level 0; Lipschitz |
| 2 — Extended with explicit v_y | `ẋ = v cos ψ − v_y sin ψ, ψ̇ = ω` | +1 algebraic or state | Captures lateral drift; needs a model or learned mapping for `v_y(v, ω, terrain)` |
| 3 — Full coupled dynamics | Pacejka tire forces, slip ratios, side-slip angles | ≥ 11 + morph | Outdoor / aggressive regime; rejected for HILDA (see below) |

Level 3 is rejected on six reinforcing grounds (NMPC.md §1.6.2): state-dimension explosion (13-state would be ~8× the per-iteration cost via the `O(n_x³)` condensing step), Magic-Formula Jacobian complexity, unknown tire parameters with no characterisation campaign in scope, terrain-model dependency that varies across the construction site, diminishing accuracy returns at `v ≤ 0.5 m/s` and `ω ≤ 0.5 rad/s` on hard flat surfaces, and incompatibility with the soft-constraint framework that doesn't need a Lipschitz-bounded disturbance set. Level 1 is the leading candidate; Level 0 is the fallback if calibration data is unavailable for sim-phase prototyping. Calibration procedure: drive constant-radius arcs at varied speeds on concrete, fit (η_v, η_ω) by least squares; expected `η_v ∈ [0.90, 1.0]`, `η_ω ∈ [0.6, 0.9]`. This calibration is the same item flagged in §Open questions "ICR calibration."

### 6. Controller-hierarchy mapping

Three controller layers exist on the platform at 200 Hz on the UDOO Bolt V8; the NMPC (Jetson Orin Nano Super, 20 Hz target) does not command actuators directly but produces references the existing stack consumes.

| NMPC output | Target controller | Mechanism |
|---|---|---|
| `(v, ω)` from integrating `(a, α_ω)` | `SkidSteerController` | `/cmd_vel` (Twist) at ≥ 2 Hz, hard ramp limits at the controller (1.2 m/s², 0.8 rad/s²) |
| `h_ref` from integrating `ḣ` (7-state) or held constant (6-state) | `reference_controller` → `BaseStability` | `/height_reference`; 4 P-loops resolve to `(θ₁..θ₄)` from IMU |
| `s_ref` from integrating `ṡ` | `q₁` then `q₂` joint controllers | Priority split: q₁ (fast, 2 m/s) until limit, then q₂ (slow, 0.1 m/s); reversed on retraction |

Three NMPC-design consequences worth pinning. (i) NMPC input bounds are **controller-ramp-limited, not motor-limited**: `|α_ω| ≤ 0.8 rad/s²` matches the SkidSteer ramp (NMPC.md v2 had ±1.5; v3 corrects this). (ii) Inner-loop settling times (~50 ms for stability, ~5 ms for drive ramp) sit ~one NMPC cycle below the planning rate — they are treated as instantaneous from the NMPC's view, which is the only justification for the flat-ground constraint that reduces 4 limb DOF to a single `h` per §Open questions' "ICR calibration" sibling discussion. (iii) The standing-gate (refuses motion unless `is_standing = true`) means the NMPC is paused during stand-up / lay-down transitions; the *Mode* state of the platform sits outside the OCP and is arbitrated by `reference_controller`. None of these change the OCP definition but all constrain the input-bound choices and the cycle-rate target.

## Open questions

- **Controller family.** The lit review (§7.7, §11.3) chose gradient-based NMPC over hierarchical MPC and CBF-MPPI. The thesis-stage convention in `THESIS.md` keeps the alternatives viable through a controller-agnostic interface (the C¹ field `f` plus optional plane-segment halfspaces). The OCP scaffolding here should therefore not foreclose an HMPC drop-in. Decide via embedded-load profiling in 05; do not commit by code structure before then. (`NMPC.md` v3 unilaterally dropped HMPC — that pick is not adopted here; per [[authoritative-source-precedence]] the thesis-stage keep-alive policy wins.)
- **State-vector dimension (6 vs 7).** Lit-review §11.3 commits to 6-state `(x, y, θ, v, ω, s)` with `h` held at platform nominal. `NMPC.md` v3 §1.3 promotes `h` to a 7th decision variable with `ḣ` as a 4th input, justified by the controller-hierarchy mapping (§Geometric primitives §6). The trade-off: 7-state lets the NMPC arbitrate stand-up height under ramp-induced ground-clearance tightening (§Geometric primitives §3); 6-state keeps the OCP smaller and pushes h-arbitration to the `reference_controller`. Both encodings carry the same geometric primitives; the question is which side of the Jetson/UDOO boundary the h-decision lives on. Resolve during phase-3 controller implementation, informed by 05's embedded budget.
- **Constraint encoding: C¹ B-spline of f vs RANSAC-plane polytopes.** 02 commits to publishing `f(x, y, s)` as a `grid_map_msgs/GridMap` plus a CasADi B-spline interpolant. `NMPC.md` v3 §2.3 instead extracts ceiling planes by RANSAC and folds them into linear half-spaces with the tilt + floor-coupling coefficients. These are incompatible at the QP-construction level — the controller consumes either bivariate spline lookups or per-plane half-space rows, not both. The B-spline path inherits 02's variance-aware ε directly; the polytope path would need a per-plane ε-shift that loses the spatial structure 02 produces. The lit-review path (B-spline) is the thesis-stage commitment; the polytope path is documented in §Geometric primitives only for the floor-coupling derivation it carries. *Structural argument for the B-spline path:* Grandia 2022 Eq. 32 gives the Gauss-Newton Hessian decomposition for a soft constraint composed with a non-linear inner function, `∇²_w (B(h(w))) ≈ ∇_w h(w)ᵀ ∇²_h B(h(w)) ∇_w h(w)` — the QP curvature factors into the inner gradient (the spline's `∇_w h`, well-defined and continuous because the cubic B-spline is C²) and the relaxed barrier's diagonal Hessian, so a C¹ field is sufficient for a clean RTI step without finite-difference approximation of the constraint Jacobian. This is the formal justification 02 §Theory's "C¹ minimum" requirement leans on; without Eq. 32 it would be a smoothness assumption taken on faith. The polytope path achieves the same property differently (each row is already linear in the decision variables, so `∇_w h` is constant and the same decomposition is trivially satisfied) but at the cost of the variance-aware-ε integration awkwardness above.
- **Soft vs hard ceiling.** 02 §"Soft constraint at the field level" + lit-review §11.3 commit to a soft `f ≥ 0` with L1 slack penalty, preserving solver feasibility under transient perception artefacts. `NMPC.md` v3 §2.7 (revised) explicitly flips the ceiling to a hard QP constraint with rationale "drill column physically collides with concrete beam — no recovery mechanism." Per [[authoritative-source-precedence]] the thesis-stage soft policy wins; the hard-constraint pick from NMPC.md is *not* adopted. The hard-vs-soft question is genuinely substantive — under transient sensor dropouts a hard QP can become infeasible mid-flight — and the resolution depends on whether the variance-aware ε from 02 (with calibrated δ_cal absorbing the residual tail) is strong enough to make the soft-with-slack form numerically equivalent to a hard one in regimes where the perception is honest. Pin a written rationale during phase-3 before either choice ships.
- **Variance-aware ε integration.** `NMPC.md` v3 treats `ε_safety` as a fixed engineering constant (0.10 m transit, 0.05 m approach). 02 produces a spatially varying `ε(x, y) = ε_base + δ_cal + λ √σ²_c` with a calibration protocol. These are not stitched together anywhere yet — the controller-side wiring of 02's interface (variance-aware ε per cell, not a constant offset) is a deliverable of phase 3. Trivial for the B-spline encoding (the spline already carries the per-cell ε contribution since 02 publishes `epsilon` as a GridMap layer and the spline interpolates it); non-trivial for the polytope encoding (each plane half-space would need a per-plane ε-shift, and the spatial structure is lost — another count against the polytope path).
- **ICR calibration.** The skid-steer ICR factors are "empirically calibrated" in the lit review but no procedure or fitted values are written down. Define the calibration manoeuvre, the optimisation objective, and the validation envelope before the controller is profiled in simulation.
- **Horizon and discretisation.** The lit review names acados SQP-RTI and a 5–20 ms solve target but does not commit to a horizon length or shooting node count. Pick during phase 3 of the research plan.
- **Warm-start fidelity.** Whether the 1D `s`-sweep on the SMAC plan is a good warm start when the sweep and the OCP disagree on which side of a ceiling feature to pass — the sweep is unaware of `(v, ω)` dynamics. Investigate during sim evaluation; may motivate a second warm-start branch.
- **Soft-vs-hard constraint policy.** The proposal uses a soft `f ≥ 0` with L1 slack and a separate log-barrier in the cost. The interaction between the two under heavy transient perception noise is not analysed in the lit review.

## Cross-references

- 02 — variance-aware clearance field (supplies `f` and its CasADi B-spline)
- `hilda_clearance_field/clearance_spline.py` — the CasADi B-spline export of `f(x, y, s[, h])` + analytic gradient (ADR-0010 #1) landed 2026-05-30; composes `f = c − ε − H` (`H = h + 1.899 + s`, so a 6↔7-state switch needs no re-fit) and is acados-`con_h_expr`-gate verified. Design: [spec](superpowers/specs/2026-05-30-clearance-spline-export-design.md), [ADR 0013](decisions/0013-clearance-spline-interface.md)
- 04 — approach-aware IRM (consumes the OCP forward-simulation to verify approach-corridor feasibility)
- 05 — embedded deployment (the controller is the dominant latency contributor in Table 6)
