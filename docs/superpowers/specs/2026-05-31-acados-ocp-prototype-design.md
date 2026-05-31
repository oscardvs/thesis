# Standalone acados OCP prototype (module 03) — design

*2026-05-31. New feature work: the perceptive RHC of module 03. Builds the lit-review research plan's Phase 3 ("implement the perceptive NMPC in CasADi with the acados SQP-RTI backend, profiled in simulation against a fixed-`s` baseline to confirm the joint-optimisation hypothesis"). Consumes the CasADi B-spline `cs.f`/`cs.grad_f` landed 2026-05-30 ([spec](2026-05-30-clearance-spline-export-design.md), [ADR 0013](../../decisions/0013-clearance-spline-interface.md)). Theory-doc-before-code: spec first, then TDD (single-solve → closed loop → harness), then the committed experiment YAMLs.*

## Scope

A new ROS-free Python prototype in `hilda_nmpc/` (the package THESIS.md reserves for "acados OCP, Python prototype first, Nav2 controller plugin later") that builds a 6-state SQP-RTI optimal-control problem over the augmented state `x = (x, y, θ, v, ω, s)`, control `u = (a, α, u_s)`, with the variance-aware clearance field entering as a soft path constraint `f(x, y, s) ≥ 0` via the landed `cs.f`. A plant integrator drives the closed loop through a synthetic clearance field containing a deliberately-too-low beam; the controller must lower the sledge `s` to pass it, and a fixed-`s` controller on the same scene must fail. That contrast is the G3 evidence. The whole thing is exercised by committed YAMLs with quantified, falsifiable acceptance gates (configs-not-scripts; the 01b/01c discipline applied to module 03).

**Edit surface.** New `hilda_nmpc/hilda_nmpc/model.py` (the `AcadosModel` builder), `ocp.py` (the OCP/solver builder), `scenarios.py` (synthetic clearance-field builders), `closed_loop.py` (plant + RTI loop + trajectory/metric recorder), `gates.py` (the shared gate-evaluation, called by both the pytest smoke-guard and the experiment runner); new `hilda_nmpc/test/test_ocp.py`; `casadi` + `acados_template` added to `setup.py` `install_requires` (venv-only per [ADR 0012](../../decisions/0012-acados-casadi-toolchain.md)). New `thesis/experiments/configs/sim_validation_03{a,b,c}/` configs + `thesis/experiments/runners/sim_validation_03_*.py`. New [ADR 0014](../../decisions/0014-ocp-constraint-config-axes.md) (constraint-config two-axis taxonomy + barrier cost-type switch) created with the implementation. No ROS node, no Nav2 plugin, no change to `hilda_clearance_field` or the runtime `/constraint_field` publisher.

**Out of scope** (named so they do not ambush integration): the Nav2 controller-plugin wrapping and the venv→workspace packaging flip (module 05 / [ADR 0012](../../decisions/0012-acados-casadi-toolchain.md) flip condition); SMAC Lattice + 1D `s`-sweep warm-start (integration — the standalone uses the RTI shift); online ~10 Hz coefficient refresh and cross-process `cs.f` transport (module 05); the `grid_map_msgs` adapter and fitting against a real `/constraint_field` bag (module 03/05); ICR calibration (the η factors stay placeholder); embedded Jetson profiling and the real-time-feasibility proof (module 05 — solve-time here is dev-machine and descriptive, **not** the lit-review Table 6 / Fig-5 "on Orin" number); the tilt/lever-arm coordinate shift before lookup and the ground-clearance corner bound (doc 03 §Geometric primitives §1, §3 — they legitimately leave the 6-state OCP); the static-stability constraint (doc 03 §4 — OCP-relevant but **deferred for these scenes**, see §Deferred-for-this-scene). **None of the four open architecture decisions is closed here** — soft↔hard, NMPC/HMPC, 6↔7-state, IRM grid-vs-analytical all stay open by construction (§What stays open).

**Implemented now vs reserved (a scope line to confirm at the review gate).** Slack-first means the *baseline* (`soft`+`off`) must be working before anything else, so this deliverable implements: the full two-axis config schema; `ocp.py` with the constraint-mode branch (`soft` and `hard` both built — checkpoint 5 solves `hard`) and the explicit barrier branch *point*; the `soft`+`off` closed loop across all three scenes (03a/03b/03c); and the gate/harness. The **barrier-on cost path** (`CONVEX_OVER_NONLINEAR`) and therefore the `none` constraint mode (which requires `barrier=on`) are **reserved, not built here** — the schema carries both axes and `ocp.py` raises a clear `NotImplementedError` for `barrier=on` (mirroring `clearance_spline.py`'s `knot_stride>1` stub), so the Baseline-C / proposed-method / interaction-study runs are a named follow-up increment that flips a cost path the schema already reserves, with **zero re-architecting** of state, model, constraint, scene, or harness. This keeps the baseline-first TDD clean while honouring "all four configs are clean YAMLs later." If you would rather the barrier-on path be built in this same deliverable (so the follow-up is purely new YAMLs, no new code), say so at the review gate and I will move it from reserved to in-scope.

## Architecture

```
scenarios.build_*            [new]  synthetic GridSnapshot -> build_clearance_spline -> ClearanceSpline (static field)
model.build_model(cfg)       [new]  AcadosModel: x=(x,y,θ,v,ω,s), u=(a,α,u_s), MX, ERK; η_v,η_ω,u_s-rate placeholders
ocp.build_ocp(field, cfg)    [new]  AcadosOcpSolver
  ├── model = build_model(cfg)
  ├── cost  = barrier-OFF  ->  NONLINEAR_LS (goal via yref), Gauss-Newton          [the gate-proven combo]
  │         = barrier-ON   ->  CONVEX_OVER_NONLINEAR  (ψ = ½‖track‖²_W + B(f)),     [Eq. 32 generalised GN]
  │                            goal via parameter; EXTERNAL+hand-GN-Hessian fallback
  ├── constraint mode = soft  ->  con_h_expr = cs.f([x,y,s]) (+ _e), lh=0, uh=+∞, idxsh -> L1 slack zl
  │                   = hard  ->  con_h_expr = cs.f([x,y,s]) (+ _e), lh=0, uh=+∞, NO idxsh
  │                   = none  ->  no con_h_expr row  (f enters only via the cost barrier; requires barrier=ON)
  ├── bounds: v, ω, s, a, α, u_s box constraints (cfg)
  └── solver: SQP_RTI, PARTIAL_CONDENSING_HPIPM, GAUSS_NEWTON, N, tf (cfg)
closed_loop.run(solver, plant, field, cfg)  [new]  RTI shift warm-start; apply u0; integrate plant (RK4, model=plant);
                                                    record state traj, realised f, slack, per-step solve time
gates.evaluate(record, cfg)  [new]  reach / feasibility(soft: f≥−f_tol; hard: f≥0) / s-lowering / solve-time;
                                    returns verdict dict — ONE implementation, called by pytest AND the runner
```

**The two constraint axes (the load-bearing schema choice, ADR 0014).** How `f` enters is *two orthogonal axes*, not a single barrier toggle:

- **Axis 1 — `f` as a constraint:** `soft` (`con_h_expr` + `idxsh`, L1 slack) · `hard` (`con_h_expr`, no `idxsh`) · `none` (no `con_h_expr` row).
- **Axis 2 — relaxed barrier in the cost:** `off` · `on`.

The named methods are corners of this 3×2 space, and capturing both axes in the config schema *now* is what makes the eventual Phase-4 mapping a set of clean YAMLs rather than a refactor:

| Method | Axis 1 (constraint) | Axis 2 (barrier) |
|---|---|---|
| **Prototype baseline** (this deliverable) | `soft` | `off` |
| RQ3 hard arm | `hard` | `off` |
| doc 03 *proposed* method = slack×barrier interaction study | `soft` | `on` |
| Baseline C / Grandia-pure | `none` | `on` |

The correction this records: *"barrier on/off" alone does not reach Baseline C* — Baseline C also drops the slacked row (`none`), so `f` lives only in the barrier. A useful side-effect of naming both axes: the prototype baseline is exactly **proposed-minus-barrier**, the cleanest isolation of the soft mechanism. `none` + `off` is degenerate (`f` does not enter at all) and is rejected at config-validate.

**Barrier-on is a cost-type switch, not an added term (ADR 0014).** acados sets one cost type per stage, so a barrier cannot be *added onto* a `NONLINEAR_LS` stage. Barrier-off keeps the design as the spline acados-gate already proved it: `NONLINEAR_LS`, goal via `yref`, Gauss-Newton, HPIPM. Barrier-on makes the stage cost `CONVEX_OVER_NONLINEAR` with `ψ(r) = ½‖r_track‖²_W + B(r_f)`, where `r = [track_residuals; f]`, `B` is Grandia's relaxed barrier (Eq. 18), and the convex-over-nonlinear structure *is* Eq. 32's generalised Gauss-Newton — so **Grandia Eq. 32 attaches to the barrier term naturally here**, not as a side note. Because `CONVEX_OVER_NONLINEAR` does not use `yref`, the goal moves to an acados parameter in that branch. `EXTERNAL` cost with a hand-supplied Gauss-Newton Hessian is the documented fallback if the spline proves awkward inside `CONVEX_OVER_NONLINEAR`. The YAML flips a cost *path*, not a weight. The slacked baseline's C¹ requirement is met independently of the barrier — through the constraint Jacobian `∇f = cs.grad_f` — so the C¹ field is exercised either way; Eq. 32 is the justification only when the barrier is on.

## Model, cost, constraint, bounds

**Model** (`model.py`, MX, the acados modelling type — matches the gate):

```
ẋ = η_v · v · cos θ      v̇ = a
ẏ = η_v · v · sin θ      ω̇ = α
θ̇ = η_ω · ω             ṡ = u_s
```

ICR-corrected unicycle (doc 03 §Geometric primitives §5, Level 1) with `η_v = η_ω = 1.0` default — numerically Level 0 — exposed as model parameters and **flagged placeholder pending ICR calibration** (doc 03 §Open questions "ICR calibration"). First-order integrator on `s`. ERK integrator, `f_impl = xdot − f_expl`.

**Cost.** Time-to-goal is encoded as the fixed-horizon surrogate: a stage quadratic on position-error-to-goal + heading + light velocity regularisation + separately-weighted input rates `(a, α, u_s)` (the separate `R_a`/`R_α`/`R_us` weights reflect the order-of-magnitude actuator-bandwidth gap, doc 03 §Architectural commitments). Terminal cost on position + heading, plus `s − s_goal` **only when a drill target is active** (scene 03c). Barrier-off: residuals in `NONLINEAR_LS`, goal in `yref`/`yref_e`, so the compiled solver is reused across cycles and configs with no re-codegen. Heading residual is `θ` directly for the corridor scenes (heading stays near the corridor axis); the `[cos θ, sin θ]` vs `[cos θ_goal, sin θ_goal]` form is the documented generalisation if a scene needs wrap-around.

**Constraint.** `con_h_expr = cs.f(vertcat(x, y, s))` and `con_h_expr_e` likewise, `lh = 0`, `uh = +∞` (large finite). Soft mode adds the row index to `idxsh`/`idxsh_e` with an L1-dominant slack penalty `zl` (`Zl` only as small conditioning regularisation). Soft mode is RQ3's lever: drop the index (or drive `zl` high) → the hard arm, no re-architecting.

**Bounds** (box, cfg): `v ∈ [v_min, v_max]`, `ω ∈ [−ω_max, ω_max]`, `s ∈ [0, s_max]` (`s_max ≈ 2.18 m`, doc 03 §4), `a ∈ [−a_ramp, a_ramp]` (`1.2 m/s²`, controller-ramp-limited not motor, doc 03 §6), `α ∈ [−α_ramp, α_ramp]` (`0.8 rad/s²`, doc 03 §6 — v3 ramp, not v2's ±1.5), `u_s ∈ [−u_s_max, u_s_max]`. **`u_s_max` is placeholder.** The single first-order `u_s` integrator folds doc 03 §6's q₁/q₂ priority split (fast 2 m/s, then slow 0.1 m/s) into one combined rate; it governs how fast the robot can retract and therefore the feasibility margin, so it is picked conservatively and flagged placeholder like the η factors.

## Scenes (the falsifiable experiments)

Three scenes, isolation-pattern (01/01b/01c applied to scenes), each a committed YAML with a manifest and a PASS/FAIL verdict. All built as synthetic `GridSnapshot`s → `build_clearance_spline` → a **static** `ClearanceSpline` the robot moves through (online refresh is module 05). `plant = controller model` (no mismatch) in this first prototype: isolation tests the optimisation, not robustness; model-mismatch and perception noise are later config knobs. Because `plant = model` and the field is static, the realised-trajectory `f` is the *true physical clearance*, so the feasibility gate is physically meaningful.

**The beam.** A full-corridor-width low stripe in `z_ceil` (full width so lowering `s` is the only way through — no lateral dodge), placed mid-corridor at distance `d`, sized so that at the nominal extension `c_beam − ε < H(s_nominal)` (mast-top would hit the beam, `f < 0`) but `c_beam − ε ≥ H(s_clear)` for a reachable `s_clear < s_nominal`, with `H = h_stand + z_tip0 + s`. **Comfortably feasible, not marginal** (flag): retraction `s_nominal → s_clear` must be achievable with time and space margin given `u_s_max` and `d` — config-validate asserts `d / v_approach ≥ (s_nominal − s_clear)/u_s_max + t_margin`. So a PASS reads "exploited clearance" and the fixed-`s` FAIL reads "fixed-`s` couldn't," never "the scene was infeasible for anything."

- **03a — transit.** Goal beyond the beam; terminal `s`-task **inactive**. The robot starts at `s(0) = s_nominal`; `R_us` resists gratuitous `s` change, so it arrives at the beam at `s_nominal`, retracts to `s_clear` to keep `f` feasible, and reaches the goal. Isolates the G3 path-constraint→configuration coupling. Gate: reach + soft-feasibility + `s` dips below `s_nominal` in the beam region. Re-extension is **not** required (no incentive in transit-only) — that behaviour belongs to 03c.
- **03b — fixed-`s` ablation.** Same scene as 03a, `s` frozen at `s_nominal` (`u_s ≡ 0`, `s` removed from the decision). Expected: realised `f` violates feasibility under the beam. The experiment **passes its hypothesis when fixed-`s` fails** — the citable Phase-3 number ("fixed-`s` cannot clear the beam; joint-opt reaches the goal at min clearance Y"). Same `closed_loop`/`gates` code as 03a; the verdict is the joint-optimisation evidence.
- **03c — drill-target.** Terminal `s_goal ≠ 0` (a drilling extension) active, goal at a drill pose beyond the beam. The robot dips under the beam then re-extends to `s_goal` — exercises the terminal-task cost and the dip-and-recover behaviour, and sits closer to the full mission. Kept separate from 03a so a failure localises to one mechanism (path constraint vs terminal task), not both.

## Acceptance gates (`gates.py`, YAML-quantified, falsifiable)

One implementation, two entry points: a fast pytest smoke-guard and the full experiment runner (manifest + verdict) call the same `gates.evaluate`.

- **reach** — `‖(x,y)_final − goal_xy‖ ≤ reach_tol`; for 03c also `|s_final − s_goal| ≤ s_tol`.
- **feasibility** — soft arm: `min_t f_realised ≥ −f_tol` (`f_tol` = the permitted slack excursion; **record `max_slack` as a diagnostic** alongside). `min_t f ≥ 0` is the *hard* arm's gate, not the soft baseline's — using `≥ 0` for the soft baseline would silently test a hard constraint.
- **s-lowering** — `min_t s(t)` in the beam region `< s_nominal − s_drop_min` (03a/03c); the G3 behaviour.
- **solve-time** — `p50`/`p95`/`max` recorded; gate `solve_time_p95_max`. **Dev-machine, descriptive — seeds RQ4 but is not the Fig-5 real-time-feasibility proof** (that is module 05 on the Orin). The label stays off it in the write-up.
- **03b ablation** — expected `min_t f_realised < −f_tol`; the verdict records the violation magnitude as the fixed-`s` contrast.

Seeds fixed in the config; runner writes `manifest.json` (config hash + git commit), the realised trajectory, the solve-time histogram, and the verdict.

## New parameter surface

`OcpConfig` / `SceneConfig` (frozen dataclasses + YAML; offline tooling, no ROS parameter declaration):

| Parameter | Default | Notes |
|---|---|---|
| `constraint.mode` | `soft` | `soft` / `hard` / `none` (axis 1; `none` requires `barrier=on`) |
| `cost.barrier` | `off` | `off` / `on` (axis 2; `on` switches stage cost to `CONVEX_OVER_NONLINEAR`) |
| `N`, `tf` | `40`, `4.0 s` | horizon; `dt = 0.1 s`. Closes doc 03 "horizon/discretisation" *with a default, not a commitment* |
| `eta_v`, `eta_w` | `1.0`, `1.0` | ICR factors — **placeholder** pending calibration (≡ Level 0) |
| `u_s_max` | conservative | combined q₁/q₂ folded rate — **placeholder** (governs retraction margin) |
| `v_max`, `w_max`, `s_max` | `0.5`, `0.5`, `2.18` | state box bounds [m/s, rad/s, m] |
| `a_ramp`, `alpha_ramp` | `1.2`, `0.8` | input box bounds (controller ramp, doc 03 §6) |
| `Q_xy`, `Q_th`, `Q_v`, `Q_w` | cfg | stage tracking weights |
| `R_a`, `R_alpha`, `R_us` | cfg | separate input-rate weights (bandwidth gap) |
| `Q_xy_e`, `Q_s_e` | cfg | terminal weights (`Q_s_e` active only in 03c) |
| `zl` (`Zl`) | cfg | L1 slack penalty (small L2 conditioning); RQ3 hard-flip lever |
| `s_nominal`, `s_clear`, `beam_d`, `beam_width` | per scene | scene geometry; comfortable-feasibility asserted |
| gate tolerances | per scene | `reach_tol`, `f_tol`, `s_drop_min`, `solve_time_p95_max`, `s_tol` |

## Deferred-for-this-scene (listed, not dropped)

- **Static-stability constraint** (doc 03 §Geometric primitives §4): `|a| ≤ a_max(s) = g·x_support / H(s)` couples a control (`a`) to a state (`s`), so unlike primitives §1 and §3 it *does* belong in the 6-state OCP. Omitted for the first isolation scenes (which exercise the clearance coupling, not tipping), but recorded here rather than silently dropped — a careful colloquium read would catch its absence. The cheap later toggle is doc 03 §4's **option B linear over-approximation** `|a| ≤ a₀ − (a₀ − a_min)/s_max · s` (polytope-preserving), reachable as two linear constraints behind a `stability.enabled` config flag (default off).

## What stays open (none closed by code structure)

- **Soft↔hard** — the slack `idxsh`/`zl` lever (axis 1); flipping it is a YAML edit.
- **NMPC/HMPC** — only an acados *NMPC* is instantiated, but `cs.f`/`cs.grad_f` and the constraint interface stay controller-agnostic; an HMPC drop-in consumes the same field. No code here forecloses it.
- **6↔7-state** — `cs.f` already carries the `seven_state` flag (`∂f/∂h = −1` exact); the prototype runs 6-state, the model builder leaves the 7th-state promotion a flag.
- **IRM grid-vs-analytical** — untouched.
- `f` stays the B-spline; the polytope path is not built.

## Surfaced, not resolved (the three parked items)

- **h₀ §Notation-vs-§3 contradiction** (ADR 0013): the prototype consumes whatever `h_stand` the `ClearanceSpline` was built with (default `0.10 m`); it does not pin the standing height. Flagged, deferred to Oscar's call.
- **Enrico et al. 2025** (the 31 ms / 12-state / Orin number): **not relied on** for any sim-phase gate — solve-time here is dev-machine descriptive. The embedded calibration and the citation check are module 05.
- **/constraint_field ~5-vs-~8 Hz**: orthogonal — the prototype uses a static field; field-refresh rate is module 05.

## Provenance

Methodology (quantified gates, fixed seeds, solve-time histogram, the RQ3/RQ4 structure, Baseline C) is inherited from the February experiment-design doc, treated as a **methodology source, not an architecture authority** — the same stance doc 03 takes toward NMPC.md. Its hard-ceiling / RANSAC-plane / 7-state picks are *ignored* (doc 03 reopened them). That doc lives outside this repo; the RQ3/RQ4/Baseline-C mapping in §Architecture is transcribed from Oscar's summary and is to be sanity-checked at the spec-review gate.

## Testing

TDD in the order Oscar set — single-solve → close the loop → wrap the harness — so option-3 (the acados-ingests-the-OCP milestone) is the first green checkpoint *inside* the option-1 build, not a separate shipped artefact.

| # | Checkpoint | Expected |
|---|---|---|
| 1 | **Single-solve status-0** — full 6-state OCP (`build_ocp`, baseline `soft`+`off`) on the 03a beam field, one `SQP_RTI` solve | `status == 0` (the de-risking gate, scaled up from the spline gate to the whole model) |
| 2 | **Closed-loop reaches goal** — 03a via `closed_loop` | reach gate passes and `min_t f_realised ≥ −f_tol`; `max_slack` recorded |
| 3 | **s-lowering occurs** — 03a | `s` dips below `s_nominal` in the beam region by ≥ `s_drop_min`, then clears |
| 4 | **Fixed-`s` ablation fails** — 03b, thin smoke-guard calling the same `closed_loop`/`gates` as the runner | feasibility gate FAILS (fixed-`s` violates `f` under the beam) — the joint-optimisation hypothesis |
| 5 | **Soft→hard flip solves** — `constraint.mode = hard`, baseline scene | `status == 0`, RQ3-lever sanity (no re-architecting needed) |
| 6 | **df/ds wiring** — the model's `s`-channel reaches `cs.f` | perturbing `s` moves `f` by `−Δs` (consistency with `cs.grad_f`'s exact `−1`) |

Checkpoint 4 is the thin pytest guard ("fixed-`s` still fails"); the 03b **YAML** produces the citable number via the same code. `pytest.importorskip("acados_template")` skips the solve-dependent checkpoints where acados is absent so the structure tests still run; the solve checkpoints require the sourced venv (`source ~/ros2_ws/.venv-acados/acados_env.sh`).

**Deferred (module 03/05, not this commit):** real `/constraint_field` bag fields through the adapter; embedded Jetson solve-time profiling; model-mismatch and perception-noise robustness configs; the Phase-4 comparative evaluation (always-retract and decoupled baselines, lit-review research-plan Phase 4).

## Implementation-status sync + journal discipline

Land in the same commit as the code (per the impl-status-sync discipline):

1. `thesis/docs/03_nmpc_formulation.md` — record that the standalone OCP prototype landed; note the two-axis constraint config and the four-corner method mapping; mark "horizon/discretisation" defaulted (not committed); cross-link this spec and ADR 0014. Confirm the §Open questions stay open.
2. `thesis/docs/decisions/0014-ocp-constraint-config-axes.md` (new) — the two non-obvious reasoning chains future Phase-4 code would otherwise re-derive: (a) the constraint config is two orthogonal axes (mode × barrier) and "barrier on/off" alone does not reach Baseline C; (b) barrier-on is a cost-type switch (`NONLINEAR_LS`→`CONVEX_OVER_NONLINEAR`, goal `yref`→parameter), with Eq. 32 as its justification and `EXTERNAL`+hand-GN-Hessian the fallback. Clears the ADR threshold per the ADR-for-subtle-calls discipline.
3. `thesis/journal/2026-W22.md` — new entry: the brainstorm decisions (scope = config-driven closed-loop eval; encoding = slack-first, barrier-off-by-default), the scene taxonomy (03a/03b/03c), the soft-gate-tolerance correction, the static-stability deferral, the placeholder flags (η, `u_s_max`), and any acados findings from the build.
4. `thesis/experiments/configs/sim_validation_03{a,b,c}/README.md` — the per-scene hypothesis and gate rationale, matching the 01/02 config-README pattern.

## References

- [`hilda_clearance_field/hilda_clearance_field/clearance_spline.py`](../../../../hilda_clearance_field/hilda_clearance_field/clearance_spline.py) — `cs.f`/`cs.grad_f`, the consumed interface; [its spec](2026-05-30-clearance-spline-export-design.md), [ADR 0013](../../decisions/0013-clearance-spline-interface.md).
- [`03_nmpc_formulation.md`](../../03_nmpc_formulation.md) — §Architectural commitments (state/control/cost/solver), §Geometric primitives §4 (static stability), §5 (ICR levels), §6 (controller hierarchy, ramp bounds, q₁/q₂ split), §Open questions (controller family, 6/7-state, soft/hard, horizon, ICR, warm-start).
- [`02_variance_aware_clearance.md`](../../02_variance_aware_clearance.md) — the field `f` and its `ε`; `prototype_kernel.py` formula-of-record.
- [ADR 0012](../../decisions/0012-acados-casadi-toolchain.md) — venv-only toolchain (`source ~/ros2_ws/.venv-acados/acados_env.sh`), plugin-time packaging flip.
- `literature_review/main.tex` §Proposed methodology (`sec:meth:nmpc`) + §Research plan (`sec:methodology:validation`, Phase 3 = this prototype against a fixed-`s` baseline) — authoritative for architecture.
- Grandia 2022 (`literature_review/papers/grandia_2022.pdf`) — Eq. 18 (relaxed barrier), Eq. 32 (generalised Gauss-Newton Hessian for a convex barrier composed with a nonlinear inner function) — the barrier-on cost justification.
- acados: `CONVEX_OVER_NONLINEAR` / `NONLINEAR_LS` / `EXTERNAL` cost types, `con_h_expr`/`idxsh` soft constraints, `SQP_RTI` + `PARTIAL_CONDENSING_HPIPM` + `GAUSS_NEWTON` (the gate-proven stack).
- February experiment-design doc — methodology source only (RQ3/RQ4, Baseline C, §13.2 YAML, §14 success criteria, solve-time histogram); architecture ignored.
- Memory: [[feedback-authoritative-sources]], [[feedback-impl-status-sync]], [[feedback-adr-for-subtle-calls]], [[feedback-quantified-acceptance-gates]], [[project-open-decisions]], [[project-canonical-facts]], [[feedback-subagent-model]].
```
