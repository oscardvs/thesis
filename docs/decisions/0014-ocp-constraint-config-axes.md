# 0014 — OCP constraint config: two orthogonal axes + barrier cost-type switch

Status: accepted
Date: 2026-05-31
Gap(s): G3
Module: 03_nmpc_formulation.md

## Context

The module-03 OCP prototype consumes the clearance field `f` as a soft constraint now, while keeping the soft-vs-hard decision and the eventual Phase-4 method comparison reachable without a refactor. Two non-obvious reasoning chains future code would otherwise re-derive are pinned here. Design: the [spec](../superpowers/specs/2026-05-31-acados-ocp-prototype-design.md); built by [plan](../superpowers/plans/2026-05-31-acados-ocp-prototype.md).

## Choice

1. **`f` enters via two orthogonal axes, not one barrier toggle.** Axis 1 — constraint: `soft` (`con_h_expr` + `idxsh`, L1 slack) / `hard` (`con_h_expr`, no `idxsh`) / `none` (no `con_h_expr` row). Axis 2 — relaxed barrier in the cost: `off` / `on`. The named methods are corners of this 3×2 space:

   | Method | Axis 1 (constraint) | Axis 2 (barrier) |
   |---|---|---|
   | Prototype baseline (= proposed-minus-barrier) | `soft` | `off` |
   | RQ3 hard arm | `hard` | `off` |
   | doc 03 proposed / slack×barrier interaction study | `soft` | `on` |
   | Baseline C / Grandia-pure | `none` | `on` |

   The correction this records: **"barrier on/off" alone does not reach Baseline C** — it also drops the slacked row (`none`), so `f` lives only in the barrier. `none`+`off` is degenerate (`f` never enters) and is rejected at config-validate.

2. **Barrier-on is a cost-type switch, not an added term.** acados sets one cost type per stage, so a barrier cannot be added onto a `NONLINEAR_LS` stage. Off: `NONLINEAR_LS`, goal via `yref`, Gauss-Newton (the spline-gate-proven combo). On: `CONVEX_OVER_NONLINEAR` with `ψ(r) = ½‖r_track‖²_W + B(r_f)`, which is exactly Grandia Eq. 32's generalised Gauss-Newton (so Eq. 32 attaches to the barrier term, not the slacked baseline); goal moves to a parameter. `EXTERNAL` + a hand-supplied Gauss-Newton Hessian is the documented fallback.

## Consequences

The prototype builds `soft`/`hard` + `barrier=off` (all three scenes PASS); `barrier='on'`, `constraint_mode='none'`, and `stability_enabled` raise `NotImplementedError` (reserved, mirroring `clearance_spline.py`'s `knot_stride>1` stub). So the Baseline-C / proposed / interaction runs are later YAMLs that flip a reserved cost path with no re-architecting of state, model, constraint, scene, or harness. The slacked baseline's C¹ requirement is met through `cs.grad_f` (the constraint Jacobian), independent of the barrier.

The slacked-soft baseline also clarified the **fixed-`s` ablation criterion** (03b, refined during execution): with a high slack penalty `zl`, a fixed-`s` controller does not violate `f` under a too-low beam — it **stalls** at the beam edge (stays ~feasible, never reaches). So the ablation confirms the joint-optimisation hypothesis when fixed-`s` **cannot reach while feasible** (`hypothesis_confirmed = not(reach and feasible)`), robust to stall *or* violation — not the spec's original "fixed-`s` violates feasibility". Recorded in the spec §Scenes and the journal.

Revisit when the barrier path is built for the Phase-4 comparison (and if Phase-4 wants a separate forced-violation config to report a violation magnitude).
