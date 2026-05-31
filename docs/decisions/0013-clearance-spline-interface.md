# 0013 — clearance-spline interface: datum, conservative edges, affine-s decomposition

Status: accepted
Date: 2026-05-30
Gap(s): G3 (enabling), G2 boundary
Module: 03_nmpc_formulation.md (interface defined in 02_variance_aware_clearance.md)

## Context

ADR-0010 deliverable #1 is the CasADi B-spline export of `f(x, y, s)` plus analytic gradient — the controller-agnostic interface the OCP queries. Fixing its shape forces three non-obvious calls that future code would otherwise re-derive, and surfaces one cross-document inconsistency. Design detail lives in `docs/superpowers/specs/2026-05-30-clearance-spline-export-design.md`; this ADR records the reasoning chains.

## Options

- **Datum symbol.** (A) Mint a new umbrella symbol `D` for the configuration datum. (B) Reuse doc 03 §Notation's existing `H(h, s) := h + 1.899 + s`, writing `f = c − ε − H`.
- **What the spline carries vs `s`/`h`.** (A) Two bivariate `(x,y)` splines over `c` and `ε`, with `H` composed as an exact affine term. (B) One combined `(x,y)` spline over `c − ε`. (C) Trivariate `(x,y,s)` spline.
- **Out-of-domain / unobserved cells.** (A) Rely on CasADi's native out-of-grid value. (B) Pad-ring (smooth C² ramp at the data edge) plus an explicit outer `if_else` that forces `f < 0` with an inward sub-gradient beyond the fitted extent; `NaN` cells replaced by a conservative sentinel before the fit.
- **`h₀` value.** (A) Take `0.0905 m` from `kinematic_model.md` §6.1 as FK-exact. (B) Treat `h₀` as an operating-point parameter bounded by doc 03 §3, default `0.10 m`.

## Choice

Datum **B** (`H`, not `D`); spline **A** (two `(x,y)` splines + affine `H`); edges **B** (pad-ring + `if_else`, `NaN`→conservative-infeasible at fit time); `h₀` **B** (operating-point parameter, default `0.10 m`; `1.899 m` stays FK-exact). All four built so NMPC/HMPC, 6-vs-7-state, soft-vs-hard, and IRM grid-vs-analytical stay open.

## Rationale

`H` already exists in doc 03 with the same definition; a new symbol re-creates the clutter the `h_base` clash created (per [[feedback-authoritative-sources]], align to the live contract). Two splines plus an affine `H` is the only form that keeps `s` a genuine decision variable along the horizon — a spline cannot be fitted against a decision variable — and it makes `∂f/∂s = ∂f/∂h = −1` exact and the 6↔7-state switch a flag with no re-fit; combined-`c−ε` was rejected because it bakes `ε` and forecloses a global margin trim, trivariate because `f` is exactly affine in `s` (approximating a known-exact term at extra cost). The pad-ring + `if_else` makes the "`f < 0` outside the map" guarantee independent of version-specific native behaviour: on CasADi 3.7.2 the out-of-grid value is a flat `0` (not the `NaN` of issue #2837, which did not reproduce), but a flat `0` still has zero gradient, so safety is not built on it — the `if_else` supplies both the guarantee and an inward restoring sub-gradient, while the pad-ring removes the value discontinuity at the data edge. `NaN`→conservative-infeasible is the safe direction for a controller-facing field (unobserved must mean not-clearable) and avoids `NaN` poisoning the coefficient solve and the QP; note this catches only *never-observed* cells — observed-but-uncertain cells are already tightened by 02's variance-aware `ε`, which remains the uncertainty mechanism. On `h₀`: `1.899 m` is FK-traceable (`kinematic_model.md` §5.2, matching doc 03's `H`), but `0.0905 m` is only the §6.1 example at `θ = 0.05` and sits below doc 03 §3's flat-level corner-clearance floor `h ≥ 0.05 + δ_b = 0.100 m` (conservative `δ_b`), so it cannot be claimed FK-exact; the operating nominal comes from the standing-height arbitrator. The de-risking gate is the acados solve, not codegen: a minimal SQP-RTI OCP with the spline as an `MX` `con_h_expr` returns `status 0` (casadi 3.7.2 + acados v0.5.4).

**Surfaced, not resolved:** doc 03 §Notation states `h_base ≈ 1.99 m ⇒ h₀ ≈ 0.091 m`, which contradicts §3's `h ≥ 0.100 m` (conservative `δ_b`). Parameterising `h₀` (default `0.10`, datum `≈ 2.00 m`, conservative) defers the call; doc 03 §Notation-vs-§3 should be reconciled, and the live operating standing height pinned, before the controller ships.

## Consequences

`hilda_clearance_field` gains `clearance_spline.py` (ROS-free core) + `gridmap_adapter.py`; `casadi` enters `setup.py`. The export emits a bare `casadi.Function` consumed in-process by the acados problem builder or any HMPC drop-in. Live-`λ` is unavailable under spline-A (`λ` baked into the interpolated `ε`); the option-C variant (interpolate `σ²_c`) is the documented path if it is later wanted. Doc 02 §Implementation status and doc 03 §Notation update in the same commit as the code; the spec's parity suite (incl. the acados gate) is the acceptance bar. Revisit if 05's embedded profiling forces online coefficient refresh into the runtime node (ADR-0010 flip condition), or if the `h₀` reconciliation moves the datum.
