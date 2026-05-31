# Clearance-field CasADi B-spline export — design

*2026-05-30. New feature work opening module 03 (perceptive RHC). Builds ADR-0010 deliverable #1: the controller-agnostic CasADi interface the OCP queries — a `casadi.Function f(x, y, s[, h])` with analytic gradient, fitted from a `/constraint_field` snapshot. Theory-doc-before-code: spec first, then TDD against the prototype kernel, then the standalone acados OCP (separate deliverable).*

## Scope

A new ROS-free Python module in the existing `hilda_clearance_field/` package (top-level `ament_python`, controller-facing per [ADR 0010](../../decisions/0010-clearance-field-package-boundary.md)) that turns a clearance-field snapshot into a `casadi.Function` the acados problem builder — or any HMPC drop-in — queries at arbitrary `(x, y, s[, h])` along the prediction horizon, with an analytic (symbolic) gradient. The math it encodes is the per-cell formula of `prototype_kernel.py`; this module adds **interpolation** (which the kernel deliberately lacks) and the symbolic composition of the configuration-dependent datum.

Edit surface: new `hilda_clearance_field/hilda_clearance_field/clearance_spline.py` (the ROS-free core: `SplineParams`, `GridSnapshot`, `ClearanceSpline`, `build_clearance_spline`); new `hilda_clearance_field/hilda_clearance_field/gridmap_adapter.py` (the `grid_map_msgs/GridMap` → `GridSnapshot` shim, isolating the only ROS dependency); new `hilda_clearance_field/test/test_clearance_spline.py` (parity + gradient + edge + acados-consumability gates); `casadi` added to `setup.py` `install_requires`. No new package, no new topic, no new launch file, no change to `prototype_kernel.py`. The runtime publishing node stays in `hilda_ceiling/ceiling_constraint_field/` and is untouched.

Out of scope (named so they do not ambush integration): cross-process transport of the Function (shared file / in-process import / DDS blob) and online ~10 Hz coefficient refresh → both are module-05 questions; the acados OCP itself → next module-03 deliverable; knot-density tuning → phase-3 profiling (default 1:1); the tilt/lever-arm coordinate shift before lookup → doc 03 §Geometric primitives §1, open; per-bin / per-surface-class `δ_cal` → hardware corpus. None of the four open architecture decisions (NMPC/HMPC, 6-vs-7-state, soft-vs-hard ceiling, IRM grid-vs-analytical) is closed here, and the API is built so it cannot foreclose any of them.

## Architecture

```
build_clearance_spline(snapshot, params, seven_state)                       [new]
  ├── validate(snapshot)
  │     shapes agree · coords strictly increasing · ≥ degree+1 cells/axis
  │       → ValueError                                                       [fail-loud]
  ├── sanitise_and_pad(clearance, epsilon)                                   [new]
  │     ├── NaN cells → (c_oob, eps_oob)              [unobserved ⇒ infeasible, not NaN]
  │     └── pad ring (pad_cells) → (c_oob, eps_oob)   [smooth ramp to infeasible at edge]
  ├── c_spline   = ca.interpolant("c",   "bspline", [xs_pad, ys_pad], c_pad,   {degree})
  ├── eps_spline = ca.interpolant("eps", "bspline", [xs_pad, ys_pad], eps_pad, {degree})
  ├── x, y, s [, h] = MX.sym(...)                                            [MX, not SX]
  │     H = h + z_tip0 + s                            [doc 03 §Notation; h = h_stand if 6-state]
  ├── f_in  = c_spline([x, y]) − eps_spline([x, y]) − H
  ├── f_oob = oob_value − oob_slope · dist_outside(fitted_extent)            [review #3]
  ├── f     = if_else(inside(fitted_extent), f_in, f_oob)
  ├── grad_f = jacobian(f, p)                          [p = [x,y,s] or [x,y,s,h]; analytic]
  └── return ClearanceSpline(f, grad_f, c_spline, eps_spline, domain, fitted_extent, meta)
```

**Everything is built in CasADi `MX`, never `SX`.** `MX` is the acados modelling type; building the OCP model in `MX` is what lets acados ingest a spline-based `con_h_expr` and codegen it (verified: §Testing acados gate, `status = 0`). On CasADi 3.7.2 the `bspline` interpolant *does* also evaluate under `SX`, but `MX` is the documented-robust path with no downside, so the module commits to it rather than relying on a version-specific capability.

**New public types** (`clearance_spline.py`):

- `@dataclass(frozen=True) SplineParams` — see §New parameter surface.
- `@dataclass(frozen=True) GridSnapshot` — frame-agnostic, ROS-free container:
  - `clearance: np.ndarray` `(nx, ny)` per-cell `c = z_ceil − z_floor` [m], `NaN` = unobserved.
  - `epsilon: np.ndarray` `(nx, ny)` per-cell variance-aware margin `ε` [m], `NaN` = unobserved.
  - `x_coords: np.ndarray` `(nx,)`, `y_coords: np.ndarray` `(ny,)` — strictly-increasing cell-centre coordinates [m] in `frame_id`.
  - `frame_id: str` — typically `"odom"`.
- `@dataclass(frozen=True) ClearanceSpline`:
  - `f: ca.Function` — `f(p) → (1,1)`, `p = [x,y,s]` (6-state) or `[x,y,s,h]` (7-state).
  - `grad_f: ca.Function` — `∂f/∂p → (1,n)`, analytic.
  - `c_spline, eps_spline: ca.Function` — standalone `(x,y) → (1,1)` interpolants (IRM / diagnostics / the option-C extension).
  - `domain: tuple` — `(x_min, x_max, y_min, y_max)` of the **real (unpadded)** extent; the trustworthy region, exported so the warm-start / planner can keep the horizon inside it.
  - `fitted_extent: tuple` — padded extent; the `if_else` switch boundary.
  - `meta: dict` — knot vectors, cell resolution, `frame_id`, `params`, `seven_state`, snapshot stamp.

**New functions**:

- `build_clearance_spline(snapshot: GridSnapshot, params: SplineParams = SplineParams(), seven_state: bool = False) -> ClearanceSpline` — the deliverable.
- `snapshot_from_gridmap_msg(msg, clearance_layer="clearance", epsilon_layer="epsilon") -> GridSnapshot` (`gridmap_adapter.py`) — lazily imports `grid_map_msgs`; the only ROS-aware code, kept out of the core so parity tests run in the venv without ROS.

**Build deps**: `casadi` (3.7.2, in `~/ros2_ws/.venv-acados`) added to `setup.py` `install_requires`; `numpy` already present. The `ament_python` colcon build does not import `casadi`; the module is only imported from the sourced venv (per [ADR 0012](../../decisions/0012-acados-casadi-toolchain.md)).

## Data flow + key implementation choices

**Two splines plus an affine datum — `s` is never baked into a spline.** The export fits `c(x,y)` and `ε(x,y)` as two bivariate cubic B-splines and composes

```python
H = h + z_tip0 + s            # doc 03 §Notation: body-frame mast-top height above floor
f = c_spline([x, y]) - eps_spline([x, y]) - H
```

`f` is exactly affine in `s` and `h`, so `∂f/∂s = −1` and `∂f/∂h = −1` are exact and need no spline axis. This is the load-bearing call: `s` is a genuine decision variable that varies along the horizon, and you cannot fit a spline against a decision variable. It is also what makes doc 03 §Geometric primitives §2's claim that the B-spline branch "absorbs floor-coupling automatically" true — `c(x,y) = z_ceil − z_floor` carries `z_floor(x,y)` directly, so non-flat floor enters through the spline's bivariate dependence with no per-stage linearisation. A trivariate `(x,y,s)` spline was rejected: it only *approximates* a term that is known-exact and costs an extra dimension.

**Datum `H`, not a new symbol; `1.899` is FK-exact, `h₀` is an operating-point value.** Doc 03 §Notation already defines `H(h, s) := h + 1.899 + s` and restates 02's scalar as `f = c − H − ε`; this module adopts that symbol rather than minting one. The two internal `SplineParams` pieces map onto it: `z_tip0 = 1.899` m (the mast-top/drill-tip offset above `base_link` at `s = 0`, FK-traceable to `kinematic_model.md` §5.1 table → §5.2, matching doc 03's constant), and `h_stand = h₀` (the nominal base height above floor). The retired `h_base ≈ 1.99 m` equals `h_stand + z_tip0`. **`h_stand` is not `0.0905` m.** That value is the `kinematic_model.md` §6.1 *example* at `θ = 0.05`, and it sits below doc 03 §3's flat-level corner-clearance floor `h ≥ 0.05 + δ_b = 0.100 m` (conservative `δ_b = 0.050`; `0.089 m` at the URDF `δ_b = 0.039`). The operating nominal is set by the standing-height arbitrator (`reference_controller` + `BaseStability`), not by that example. So `h_stand` defaults to `0.10 m` (doc 03 §3 conservative flat-level floor) — making the datum `≈ 2.00 m`, marginally more conservative than `1.99` and in the safe direction — and is documented as an operating-point value to pin from the live standing height. The doc-03-internal tension (§Notation's `h_base ≈ 1.99 ⇒ h₀ ≈ 0.091` vs §3's `h ≥ 0.100`) is surfaced for reconciliation in [ADR 0013](../../decisions/0013-clearance-spline-interface.md), not silently resolved here.

**Interpolate the published `clearance` / `epsilon` layers; do not recompute.** `/constraint_field` already carries `clearance` and `epsilon` as GridMap layers (02 §Interface); the export interpolates those, with `prototype_kernel.py`'s `variance_aware_field_reference` as the formula-of-record the layers must match. Consequence: `λ` (and the spatial variance pattern) are baked into the interpolated `ε` layer — they are not live symbolic knobs. The live symbolic handles under this choice are the affine datum (`h, s, h_stand, z_tip0`) and an optional global additive `ε` offset. If live-`λ` is wanted later, that is the **option-C variant**: interpolate `σ²_c` instead and rebuild `ε = ε_base + δ_cal + λ √σ²_c` symbolically (`σ²_c` is published too). Named, not built — `λ = 3` is a fixed deployment choice for the controller.

**Conservative-infeasible edges via pad-ring + outer `if_else`, not native out-of-grid behaviour.** Two mechanisms compose. (1) A `pad_cells`-wide ring is appended to the fitted grid, filled with `c_oob` (low → infeasible) and `eps_oob` (high → infeasible); the cubic spline then ramps smoothly (C²) from the real-edge value down to infeasible across the ring, avoiding a value discontinuity at the data boundary and giving a real inward-pointing gradient there. (2) An outer `if_else` keyed on the padded (`fitted_extent`) bounds returns, beyond that extent, `f_oob = oob_value − oob_slope · dist_outside`, guaranteeing `f < 0` with a defined inward sub-gradient. The `if_else` is the safety guarantee; it does **not** depend on CasADi's native out-of-grid value. Verified on 3.7.2 (§Testing): the native out-of-grid value is a flat `0.0` (not the `NaN` of CasADi issue #2837, which did not reproduce on this version) — but a flat `0.0` still has zero gradient and is version-specific, so safety is not built on it. `dist_outside(p) = Σ max(0, lo − p) + max(0, p − hi)` over the two axes.

**`NaN` (unobserved) cells → conservative-infeasible at fit time.** A `NaN` in `clearance`/`epsilon` is replaced by `(c_oob, eps_oob)` *before* the coefficient solve. This is a deliberate, fail-safe departure from the kernel's `NaN`-propagation contract: a `NaN` would poison the B-spline coefficient solve (spreading to neighbouring cells) and produce `NaN` QP constraints (which break the solve), and for a controller-facing field *unobserved must mean not-clearable*, not unconstrained. Framing for the ADR: observed-but-uncertain cells are already driven toward infeasible by 02's variance-aware `ε` (high `σ²_c` → large margin → smaller `f`); the spline's `NaN→infeasible` substitution catches only *never-observed* cells. The spline is not the uncertainty mechanism — `ε` is.

**Column-major value ordering.** `ca.interpolant` consumes grid values flattened to match `[x_coords, y_coords]` (first axis varies fastest). The adapter must ravel the GridMap layer in the order matching `grid_map`'s storage; a transpose bug here is silent (the field is plausibly smooth either way) so it is asserted by a parity test on an asymmetric field.

**ROS-free core.** `clearance_spline.py` imports only `numpy` + `casadi`; all `grid_map_msgs` knowledge lives in `gridmap_adapter.py`. This matches `prototype_kernel.py`'s ROS-free design, keeps the parity tests runnable in the bare venv, and honours validate-in-isolation-before-integration.

## New parameter surface

`SplineParams` (frozen dataclass; no ROS parameter declaration — this is offline tooling):

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `h_stand` | float | `0.10` | `[0.05, 0.30]` | Nominal base-link height above floor `h₀` [m]. Operating-point value (standing-height arbitrator), ≥ doc 03 §3 `h_min`. **Not** the `0.0905` kinematic example. |
| `z_tip0` | float | `1.899` | — | Mast-top/drill-tip offset above `base_link` at `s = 0` [m]. FK-exact (`kinematic_model.md` §5.2; doc 03 `H`). |
| `degree` | tuple[int,int] | `(3, 3)` | — | Tensor-product spline degree (cubic; gives C², ≥ the C¹ minimum). |
| `knot_stride` | int | `1` | `[1, 8]` | Knots per N elevation cells (1:1 default; coarser trades fidelity for controller eval cost; phase-3 tunable). |
| `pad_cells` | int | `2` | `[1, 8]` | Conservative-infeasible border-ring width [cells]. At 0.10 m resolution, `2` = 0.20 m. |
| `c_oob` | float | `0.0` | — | Clearance sentinel in pad + `NaN` cells [m]; `0` ⇒ `f = −ε − H < 0`. |
| `eps_oob` | float | `1.0` | `[0.0, 5.0]` | `ε` sentinel in pad + `NaN` cells [m]; large ⇒ reinforces infeasibility. |
| `oob_value` | float | `−5.0` | — | Fixed `f` beyond `fitted_extent` [m] (≪ 0). |
| `oob_slope` | float | `10.0` | `[0.0, ∞)` | Inward sub-gradient of `f` beyond `fitted_extent` [m⁻¹]. |

`pad_cells × resolution` must exceed the tilt lever-arm so a tilt-shifted lookup (doc 03 §1: up to `H sin 1° ≈ 7 cm` at full extension) cannot fall off the fitted extent. Default `0.20 m > 0.07 m` holds with margin; a later `pad_cells` or `resolution` change must preserve `pad_cells × resolution ≥ H_max · sin(α_max)`. `domain`, `pad`, and the deferred tilt-shift interact — recorded together so they do not drift apart.

## Error handling

| Condition | Handler | Severity | Effect |
|---|---|---|---|
| snapshot shape mismatch / coords not strictly increasing / `< degree+1` cells per axis | `raise ValueError` | fail-loud (build aborts) | no Function produced |
| `NaN` cells in `clearance` / `epsilon` | replace with `(c_oob, eps_oob)` before fit | by design (silent) | those cells read infeasible (`f < 0`) |
| query `(x,y)` in the pad ring (real-edge → fitted-extent) | spline ramp | n/a | `f` smoothly decreases to infeasible, C², inward gradient |
| query `(x,y)` beyond `fitted_extent` | `if_else → f_oob` | n/a | `f = oob_value − oob_slope · dist_outside < 0`, inward sub-gradient |
| all-zero `clearance` (fully unobserved local map; `NaN`→`c_oob` then everything `0`) | native bspline returns `≈ 0`; `f = 0 − ε − H < 0` | safe (verified, §Testing) | whole field infeasible (correct) |

The build is a pure function with no liveness obligation; failures abort loudly at construction rather than degrading at query time. The fitted Function never raises — out-of-domain is handled in-band by the conservative branch.

## Testing

TDD against the 13-case prototype kernel, which is the regression reference. Parity is exact at cell centres because an interpolating B-spline passes through its grid values (padding perturbs only between-knot values near the boundary, never the real interior grid points).

**Parity construction.** Run `variance_aware_field_reference(z_floor, z_ceil, var_floor, var_ceil, s, FieldParams(h_base, eps_base, delta_cal, lam))` on a synthetic field → take its `clearance` and `epsilon` arrays into a `GridSnapshot`, then build with `SplineParams(z_tip0=Z, h_stand=h_base−Z)` so the export's `H`-constant equals the kernel's `h_base`.

| # | Setup | Expected outcome |
|---|---|---|
| 1 | smooth synthetic `clearance`/`epsilon`, sample `c_spline`/`eps_spline` at cell centres | `max|spline − layer| ≤ 1e-5 m` on the real interior |
| 2 | `f` at cell centres, `s = 0.4`, `H`-const = kernel `h_base` | `max|f_export − feasibility_kernel| ≤ 1e-5 m` |
| 3 | 7-state build, evaluate `grad_f` | `∂f/∂s = −1`, `∂f/∂h = −1` exact; `∂f/∂x, ∂f/∂y` vs central-difference of `c_spline` ≤ `1e-3` on smooth interior |
| 4 | seed one interior cell `NaN` | `f` finite and `< 0` at that cell; all neighbours finite (no `NaN` poisoning) |
| 5 | query beyond `fitted_extent` | `f = oob_value − oob_slope·dist < 0`; `grad_f` points inward (sign check on both axes) |
| 6 | all-zero `clearance` snapshot (degenerate / fully unobserved) | build succeeds, no `NaN`; `f < 0` everywhere (CasADi issue #2837 non-regression) |
| 7 | asymmetric field (`c[i,j] = i + 10·j`) | cell-centre parity holds → ravel order correct (transpose-bug guard) |
| 8 | **acados consumability gate** | minimal SQP-RTI OCP with `f` as an `MX` `con_h_expr`, `lh = 0`: build → codegen → compile → `solve()` returns `status = 0` |

Case 8 is the real de-risking gate (it, not `f.generate()`, proves acados ingests the spline constraint); already passing on a throwaway probe (casadi 3.7.2 + acados v0.5.4, `status 0`, Gauss-Newton, `u0 ≠ 0`). It requires the sourced venv; `pytest.importorskip("acados_template")` skips it where acados is absent so the parity suite still runs.

**Deferred (module 03 / 05, not this commit):** fitting against a real `/constraint_field` bag snapshot through the adapter; spline-evaluation latency against the 20 Hz / 50 ms budget (lit-review Table 6); online coefficient refresh.

## Implementation-status sync + journal discipline

Land in the same commit as the code (per [[feedback-impl-status-sync]]):

1. `thesis/docs/02_variance_aware_clearance.md` §Implementation status — record that deliverable #1 landed: the two-spline + affine-`H` form, the `NaN`/pad conservative-infeasible convention, the `λ`-baked / option-C note, and the parity-against-kernel gate.
2. `thesis/docs/03_nmpc_formulation.md` — align §Notation usage (`f = c − ε − H`) and note the spline interface now exists with `grad_f`; cross-link the spec.
3. `thesis/journal/2026-W22.md` — new entry: the brainstorm decisions, the CasADi 3.7.2 probe findings (SX works, #2837 not reproduced, acados spline-constraint `status 0`), the `h_stand`/§Notation-vs-§3 reconciliation, and the new ADR.
4. [ADR 0013](../../decisions/0013-clearance-spline-interface.md) — created (Status: proposed). Clears the ADR threshold (per [[feedback-adr-for-subtle-calls]]): three non-obvious reasoning chains future code would otherwise re-derive — the `H`/`h_base` rename + `h₀`-vs-§3 tension, the `NaN`/OOB conservative-infeasible policy (and why not native bspline behaviour), and the 2D-splines-plus-affine-`s` decomposition.

## References

- [`hilda_clearance_field/prototype_kernel.py`](../../../../hilda_clearance_field/hilda_clearance_field/prototype_kernel.py) — formula-of-record (`variance_aware_field_reference`, `FieldParams`); parity reference.
- [`02_variance_aware_clearance.md`](../../02_variance_aware_clearance.md) §Theory + §Design choices — `f = c − h_base − s − ε`, the committed cubic tensor-product B-spline form.
- [`03_nmpc_formulation.md`](../../03_nmpc_formulation.md) §Notation (`H(h,s) := h + 1.899 + s`, `f = c − H − ε`), §3 (ground-clearance lower bound `h ≥ 0.05 + δ_b`), §2 (floor-coupling), §1 (tilt lever-arm), §Open questions (the four open decisions).
- [`kinematic_model.md`](../../../../hilda_ros/hilda_common/hilda_kinematics/docs/kinematic_model.md) §5.1–5.3 (`z_tip = 1.899 + q₁ + q₂`), §6.1 (standing-height example) — math-extraction source, subordinate to doc 03.
- [ADR 0010](../../decisions/0010-clearance-field-package-boundary.md) — deliverable #1 + package boundary; [ADR 0012](../../decisions/0012-acados-casadi-toolchain.md) — toolchain + venv; [ADR 0013](../../decisions/0013-clearance-spline-interface.md) — this interface's decisions.
- CasADi `interpolant('bspline', …)` — analytic Jacobian/Hessian, codegen-to-C, `MX` ingestion (verified, 3.7.2); CasADi issue #2837 (all-zero `NaN`, non-regression on 3.7.2).
- Grandia 2022 Eq. 32 — Gauss-Newton Hessian decomposition justifying a C¹ field for a clean RTI step (doc 03 §Open questions).
- Memory: [[feedback-authoritative-sources]], [[feedback-impl-status-sync]], [[feedback-adr-for-subtle-calls]], [[feedback-latching-spatial-samples]], [[feedback-quantified-acceptance-gates]], [[project-open-decisions]].
