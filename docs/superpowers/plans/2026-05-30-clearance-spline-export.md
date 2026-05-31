# Clearance-field CasADi B-spline export — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `build_clearance_spline(snapshot, params) -> ClearanceSpline`, which fits two bivariate cubic B-splines (`c`, `ε`) from a clearance-field snapshot and emits a `casadi.Function f(x, y, s[, h])` with analytic gradient, conservative-infeasible outside the map, parity-checked against the prototype kernel and proven consumable by acados.

**Architecture:** `f = c_spline(x,y) − eps_spline(x,y) − H`, `H = h + z_tip0 + s` (doc 03 §Notation), exactly affine in `s`/`h` so `∂f/∂s = ∂f/∂h = −1` and no spline `s`-axis. Out-of-map is a pad-ring (smooth C² ramp at the data edge) plus an explicit `if_else` that forces `f < 0` with an inward sub-gradient beyond the padded extent — safety never depends on CasADi's native out-of-grid value. Everything is built in `MX` (acados's modelling type). ROS-free: imports only `numpy` + `casadi`.

**Tech Stack:** Python 3.12, CasADi 3.7.2 + acados v0.5.4 (in `~/ros2_ws/.venv-acados`), NumPy (apt 1.26.4), pytest. Package `hilda_clearance_field` (its own git repo, ament_python).

**Reference spec:** `thesis/docs/superpowers/specs/2026-05-30-clearance-spline-export-design.md`. **Decisions:** ADR 0010 (deliverable + boundary), 0012 (toolchain), 0013 (this interface).

**Scope note (refined from spec):** This plan delivers the ROS-free builder + tests only. `gridmap_adapter.py` (the `grid_map_msgs/GridMap → GridSnapshot` shim) is deferred to the module-03/05 integration deliverable, where it is verified against a real `/constraint_field` bag; building it in isolation would require unverifiable grid_map layout code. The four open architecture decisions (NMPC/HMPC, 6-vs-7-state, soft-vs-hard, IRM grid-vs-analytical) stay open.

**Environment — every test/command runs in the sourced venv:**
```bash
source ~/ros2_ws/.venv-acados/acados_env.sh
cd /home/odesha/ros2_ws/src/hilda_clearance_field
```
Do **not** `pip install numpy` (apt-managed; breaks CuPy/ros2_numpy).

---

## File Structure

- `hilda_clearance_field/hilda_clearance_field/clearance_spline.py` — **new**, the entire deliverable: `SplineParams`, `GridSnapshot`, `ClearanceSpline` dataclasses; `_validate`, `_pad_axis`, `_sanitise_and_pad` helpers; `build_clearance_spline`. ROS-free (numpy + casadi only).
- `hilda_clearance_field/test/test_clearance_spline.py` — **new**, all eight gates (parity, gradient, edges, acados).
- `hilda_clearance_field/setup.py` — **modify** line 13, add `casadi` to `install_requires` (metadata only; actual casadi comes from the venv).
- `hilda_clearance_field/hilda_clearance_field/prototype_kernel.py` — **unchanged**, imported by tests as the parity reference.

---

## Task 0: Feature branch

- [ ] **Step 1: Branch the code repo**

```bash
git -C /home/odesha/ros2_ws/src/hilda_clearance_field checkout -b feature/clearance-spline-export
```
Expected: `Switched to a new branch 'feature/clearance-spline-export'`

---

## Task 1: Dataclasses, validation, casadi dependency

**Files:**
- Create: `hilda_clearance_field/hilda_clearance_field/clearance_spline.py`
- Create: `hilda_clearance_field/test/test_clearance_spline.py`
- Modify: `hilda_clearance_field/setup.py:13`

- [ ] **Step 1: Write the failing validation tests**

Create `test/test_clearance_spline.py`:

```python
"""Tests for the CasADi B-spline export. Run in the sourced .venv-acados."""
import numpy as np
import pytest

from hilda_clearance_field.clearance_spline import (
    SplineParams, GridSnapshot, _validate,
)


def _smooth_snapshot(nx=12, ny=12, res=0.10):
    """A finite, smooth synthetic clearance/epsilon field on a regular grid."""
    xs = np.arange(nx, dtype=float) * res
    ys = np.arange(ny, dtype=float) * res
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    clearance = 3.0 + 0.4 * np.sin(2.0 * X) + 0.3 * np.cos(1.5 * Y)
    epsilon = 0.10 + 0.02 * np.cos(X)
    return GridSnapshot(clearance=clearance, epsilon=epsilon,
                        x_coords=xs, y_coords=ys, frame_id="odom")


def test_validate_accepts_well_formed_snapshot():
    _validate(_smooth_snapshot(), SplineParams())  # must not raise


def test_validate_rejects_shape_mismatch():
    s = _smooth_snapshot()
    bad = GridSnapshot(clearance=s.clearance, epsilon=s.epsilon[:, :-1],
                       x_coords=s.x_coords, y_coords=s.y_coords)
    with pytest.raises(ValueError, match="shape"):
        _validate(bad, SplineParams())


def test_validate_rejects_non_increasing_coords():
    s = _smooth_snapshot()
    bad = GridSnapshot(clearance=s.clearance, epsilon=s.epsilon,
                       x_coords=s.x_coords[::-1], y_coords=s.y_coords)
    with pytest.raises(ValueError, match="increasing"):
        _validate(bad, SplineParams())


def test_validate_rejects_too_few_cells_for_degree():
    xs = np.arange(3, dtype=float) * 0.1
    ys = np.arange(3, dtype=float) * 0.1
    tiny = GridSnapshot(clearance=np.ones((3, 3)), epsilon=np.ones((3, 3)),
                        x_coords=xs, y_coords=ys)
    with pytest.raises(ValueError, match="cells"):
        _validate(tiny, SplineParams())  # cubic needs >= 4 per axis
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
source ~/ros2_ws/.venv-acados/acados_env.sh
cd /home/odesha/ros2_ws/src/hilda_clearance_field
python -m pytest test/test_clearance_spline.py -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'hilda_clearance_field.clearance_spline'`

- [ ] **Step 3: Create the module with dataclasses + validation**

Create `hilda_clearance_field/clearance_spline.py`:

```python
"""CasADi B-spline export of the variance-aware clearance field (ADR-0010 #1).

f(x, y, s[, h]) = c_spline(x,y) - eps_spline(x,y) - H,  H = h + z_tip0 + s
(doc 03 §Notation). A casadi.Function + analytic gradient, fitted from a
/constraint_field snapshot, conservative-infeasible outside the map.

ROS-free: imports only numpy + casadi. See
thesis/docs/superpowers/specs/2026-05-30-clearance-spline-export-design.md
and decisions/0013-clearance-spline-interface.md.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import casadi as ca


@dataclass(frozen=True)
class SplineParams:
    h_stand: float = 0.10        # nominal base-link height above floor h0 [m] (doc 03 §3 floor)
    z_tip0: float = 1.899        # mast-top/drill-tip offset above base_link at s=0 [m] (FK §5.2)
    degree: tuple = (3, 3)       # cubic tensor product -> C2
    knot_stride: int = 1         # knots per N cells; 1:1 only for now (phase-3 hook)
    pad_cells: int = 2           # conservative-infeasible border ring [cells]
    c_oob: float = 0.0           # clearance sentinel in pad + NaN cells [m]
    eps_oob: float = 1.0         # epsilon sentinel in pad + NaN cells [m]
    oob_value: float = -5.0      # fixed f beyond the fitted extent [m]
    oob_slope: float = 10.0      # inward sub-gradient beyond the fitted extent [1/m]


@dataclass(frozen=True)
class GridSnapshot:
    clearance: np.ndarray        # (nx, ny) [m], NaN = unobserved
    epsilon: np.ndarray          # (nx, ny) [m], NaN = unobserved
    x_coords: np.ndarray         # (nx,) strictly increasing cell-centre x [m]
    y_coords: np.ndarray         # (ny,) strictly increasing cell-centre y [m]
    frame_id: str = "odom"


@dataclass(frozen=True)
class ClearanceSpline:
    f: "ca.Function"             # f(p) -> (1,1); p = [x,y,s] or [x,y,s,h]
    grad_f: "ca.Function"        # df/dp -> (1,n)
    c_spline: "ca.Function"      # c(x,y) -> (1,1)
    eps_spline: "ca.Function"    # eps(x,y) -> (1,1)
    domain: tuple                # (x_min,x_max,y_min,y_max) real (unpadded) extent
    fitted_extent: tuple         # padded extent; the if_else switch boundary
    meta: dict


def _validate(snapshot: GridSnapshot, params: SplineParams) -> None:
    c = np.asarray(snapshot.clearance)
    e = np.asarray(snapshot.epsilon)
    xs = np.asarray(snapshot.x_coords)
    ys = np.asarray(snapshot.y_coords)
    if c.ndim != 2:
        raise ValueError(f"clearance must be 2-D, got {c.ndim}-D")
    if c.shape != e.shape:
        raise ValueError(f"clearance {c.shape} and epsilon {e.shape} shapes differ")
    nx, ny = c.shape
    if xs.shape != (nx,) or ys.shape != (ny,):
        raise ValueError(
            f"coords (x{xs.shape}, y{ys.shape}) do not match layer shape {c.shape}")
    if not (np.all(np.diff(xs) > 0) and np.all(np.diff(ys) > 0)):
        raise ValueError("x_coords and y_coords must be strictly increasing")
    if nx < params.degree[0] + 1 or ny < params.degree[1] + 1:
        raise ValueError(
            f"need >= degree+1 cells per axis for degree {params.degree}; got {c.shape}")
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python -m pytest test/test_clearance_spline.py -v
```
Expected: PASS — 4 passed.

- [ ] **Step 5: Add casadi to setup.py**

Modify `setup.py:13`, change:
```python
    install_requires=["setuptools", "numpy"],
```
to:
```python
    install_requires=["setuptools", "numpy", "casadi"],  # casadi resolved from .venv-acados
```

- [ ] **Step 6: Commit**

```bash
git add hilda_clearance_field/clearance_spline.py test/test_clearance_spline.py setup.py
git commit -m "feat(clearance_spline): dataclasses + snapshot validation [G3]

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Padding and sanitise helpers

**Files:**
- Modify: `hilda_clearance_field/hilda_clearance_field/clearance_spline.py`
- Modify: `hilda_clearance_field/test/test_clearance_spline.py`

- [ ] **Step 1: Write the failing helper tests**

Append to `test/test_clearance_spline.py`:

```python
from hilda_clearance_field.clearance_spline import _pad_axis, _sanitise_and_pad


def test_pad_axis_extends_uniformly_both_sides():
    xs = np.array([0.0, 0.1, 0.2, 0.3])
    out = _pad_axis(xs, pad=2, step=0.1)
    assert out.shape == (8,)
    np.testing.assert_allclose(out, [-0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5], atol=1e-12)
    assert np.all(np.diff(out) > 0)


def test_sanitise_replaces_nan_then_pads_with_sentinel():
    a = np.array([[1.0, np.nan], [3.0, 4.0]])
    out = _sanitise_and_pad(a, pad=1, sentinel=-9.0)
    assert out.shape == (4, 4)              # (2+2) x (2+2)
    assert np.all(np.isfinite(out))         # no NaN survives
    assert out[1, 1] == 1.0                 # interior real cell preserved
    assert out[1, 2] == -9.0                # the NaN cell -> sentinel
    assert out[0, 0] == -9.0 and out[-1, -1] == -9.0   # pad ring is sentinel


def test_pad_axis_zero_pad_is_identity():
    xs = np.array([0.0, 0.1, 0.2, 0.3])
    np.testing.assert_array_equal(_pad_axis(xs, pad=0, step=0.1), xs)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python -m pytest test/test_clearance_spline.py -k "pad or sanitise" -v
```
Expected: FAIL — `ImportError: cannot import name '_pad_axis'`

- [ ] **Step 3: Implement the helpers**

Append to `clearance_spline.py`:

```python
def _pad_axis(coords: np.ndarray, pad: int, step: float) -> np.ndarray:
    """Extend a strictly-increasing coordinate axis by `pad` cells of `step` each side."""
    coords = np.asarray(coords, dtype=float)
    if pad <= 0:
        return coords
    lo = coords[0] + step * np.arange(-pad, 0)
    hi = coords[-1] + step * np.arange(1, pad + 1)
    return np.concatenate([lo, coords, hi])


def _sanitise_and_pad(layer: np.ndarray, pad: int, sentinel: float) -> np.ndarray:
    """NaN/inf cells -> sentinel, then a `pad`-wide border ring of sentinel.

    Done before the spline fit so unobserved cells never poison the coefficient
    solve and the field reads conservative-infeasible there (spec §Data flow).
    """
    a = np.array(layer, dtype=float)
    a[~np.isfinite(a)] = sentinel
    if pad <= 0:
        return a
    return np.pad(a, pad_width=pad, mode="constant", constant_values=sentinel)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python -m pytest test/test_clearance_spline.py -k "pad or sanitise" -v
```
Expected: PASS — 3 passed.

- [ ] **Step 5: Commit**

```bash
git add hilda_clearance_field/clearance_spline.py test/test_clearance_spline.py
git commit -m "feat(clearance_spline): NaN->sentinel + pad-ring helpers [G3]

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Builder (6-state) + kernel parity, edges, ravel order

**Files:**
- Modify: `hilda_clearance_field/hilda_clearance_field/clearance_spline.py`
- Modify: `hilda_clearance_field/test/test_clearance_spline.py`

- [ ] **Step 1: Write the failing builder/parity tests**

Append to `test/test_clearance_spline.py`:

```python
from hilda_clearance_field.prototype_kernel import (
    variance_aware_field_reference, FieldParams,
)
from hilda_clearance_field.clearance_spline import build_clearance_spline

# H-constant the spline subtracts in 6-state == kernel h_base, so feasibility lines up.
_Z_TIP0 = 1.899
_KERNEL_HBASE = 1.99
_PARAMS = SplineParams(z_tip0=_Z_TIP0, h_stand=_KERNEL_HBASE - _Z_TIP0)


def _kernel_snapshot(nx=12, ny=12, res=0.10, s=0.0):
    """Run the prototype kernel on a synthetic field; return (snapshot, kernel_out, s)."""
    xs = np.arange(nx, dtype=float) * res
    ys = np.arange(ny, dtype=float) * res
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    z_floor = 0.02 * np.sin(X) + 0.01 * Y
    z_ceil = 3.0 + 0.4 * np.sin(2.0 * X) + 0.3 * np.cos(1.5 * Y)
    var_floor = np.full((nx, ny), 1e-4)
    var_ceil = 2e-3 + 1e-3 * np.cos(X) ** 2
    out = variance_aware_field_reference(
        z_floor, z_ceil, var_floor, var_ceil, s,
        FieldParams(h_base=_KERNEL_HBASE, eps_base=0.05, delta_cal=0.02, lam=3.0))
    snap = GridSnapshot(clearance=out["clearance"], epsilon=out["epsilon"],
                        x_coords=xs, y_coords=ys, frame_id="odom")
    return snap, out, s


def test_cell_centre_parity_clearance_and_epsilon():
    snap, out, _ = _kernel_snapshot()
    cs = build_clearance_spline(snap, _PARAMS)
    for i, x in enumerate(snap.x_coords):
        for j, y in enumerate(snap.y_coords):
            assert abs(float(cs.c_spline([x, y])) - out["clearance"][i, j]) < 1e-6
            assert abs(float(cs.eps_spline([x, y])) - out["epsilon"][i, j]) < 1e-6


def test_cell_centre_parity_feasibility():
    snap, out, s = _kernel_snapshot(s=0.40)
    cs = build_clearance_spline(snap, _PARAMS)
    for i, x in enumerate(snap.x_coords):
        for j, y in enumerate(snap.y_coords):
            f = float(cs.f([x, y, s]))
            assert abs(f - out["feasibility"][i, j]) < 1e-6


def test_out_of_map_is_conservative_negative():
    snap, _, _ = _kernel_snapshot()
    cs = build_clearance_spline(snap, _PARAMS)
    x_hi = cs.fitted_extent[1]
    y_hi = cs.fitted_extent[3]
    f = float(cs.f([x_hi + 1.0, y_hi + 1.0, 0.0]))
    assert f < 0.0
    assert abs(f - (_PARAMS.oob_value - _PARAMS.oob_slope * 2.0)) < 1e-9


def test_interior_nan_cell_is_infeasible_and_does_not_poison():
    snap, out, s = _kernel_snapshot(s=0.0)
    c = snap.clearance.copy()
    i0, j0 = 5, 5
    c[i0, j0] = np.nan
    snap2 = GridSnapshot(clearance=c, epsilon=snap.epsilon,
                         x_coords=snap.x_coords, y_coords=snap.y_coords)
    cs = build_clearance_spline(snap2, _PARAMS)
    f_nan = float(cs.f([snap.x_coords[i0], snap.y_coords[j0], s]))
    assert np.isfinite(f_nan) and f_nan < 0.0           # unobserved -> infeasible, not NaN
    f_nbr = float(cs.f([snap.x_coords[i0 + 2], snap.y_coords[j0], s]))
    assert np.isfinite(f_nbr)                            # neighbour not poisoned
    assert abs(f_nbr - out["feasibility"][i0 + 2, j0]) < 1e-6


def test_all_zero_clearance_builds_and_is_infeasible():
    # CasADi issue #2837 non-regression: all-zero values must not yield NaN.
    nx = ny = 12
    xs = np.arange(nx, dtype=float) * 0.1
    ys = np.arange(ny, dtype=float) * 0.1
    snap = GridSnapshot(clearance=np.zeros((nx, ny)), epsilon=np.full((nx, ny), 0.1),
                        x_coords=xs, y_coords=ys)
    cs = build_clearance_spline(snap, _PARAMS)
    for x in xs:
        for y in ys:
            f = float(cs.f([x, y, 0.0]))
            assert np.isfinite(f) and f < 0.0


def test_ravel_order_is_column_major():
    # Asymmetric field: a transposed flatten would return f(x_i,y_j) for the (j,i) cell.
    nx, ny = 12, 13
    xs = np.arange(nx, dtype=float) * 0.1
    ys = np.arange(ny, dtype=float) * 0.1
    clearance = np.array([[float(i + 10 * j) for j in range(ny)] for i in range(nx)])
    snap = GridSnapshot(clearance=clearance, epsilon=np.full((nx, ny), 0.1),
                        x_coords=xs, y_coords=ys)
    cs = build_clearance_spline(snap, _PARAMS)
    for (i, j) in [(2, 3), (7, 1), (10, 11)]:
        assert abs(float(cs.c_spline([xs[i], ys[j]])) - clearance[i, j]) < 1e-6
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python -m pytest test/test_clearance_spline.py -k "parity or out_of_map or nan or all_zero or ravel" -v
```
Expected: FAIL — `ImportError: cannot import name 'build_clearance_spline'`

- [ ] **Step 3: Implement the builder (6-state)**

Append to `clearance_spline.py`:

```python
def build_clearance_spline(snapshot: GridSnapshot,
                           params: SplineParams = SplineParams(),
                           seven_state: bool = False) -> ClearanceSpline:
    """Fit c(x,y) and eps(x,y) cubic B-splines and compose f = c - eps - H.

    f is exactly affine in s (and h), so df/ds = df/dh = -1; only (x,y) is splined.
    Outside the padded extent, f is forced negative with an inward sub-gradient.
    """
    _validate(snapshot, params)
    if params.knot_stride != 1:
        raise NotImplementedError("knot_stride > 1 is a phase-3 extension; use 1")
    if seven_state:
        raise NotImplementedError("seven_state added in a later step")

    xs = np.asarray(snapshot.x_coords, dtype=float)
    ys = np.asarray(snapshot.y_coords, dtype=float)
    dx = float(xs[1] - xs[0])
    dy = float(ys[1] - ys[0])
    P = int(params.pad_cells)

    xs_pad = _pad_axis(xs, P, dx)
    ys_pad = _pad_axis(ys, P, dy)
    c_pad = _sanitise_and_pad(snapshot.clearance, P, params.c_oob)
    eps_pad = _sanitise_and_pad(snapshot.epsilon, P, params.eps_oob)

    deg = list(params.degree)
    # column-major flatten: first grid axis (x) varies fastest, matching [xs_pad, ys_pad]
    c_spline = ca.interpolant("c_spline", "bspline", [xs_pad, ys_pad],
                              c_pad.ravel(order="F"), {"degree": deg})
    eps_spline = ca.interpolant("eps_spline", "bspline", [xs_pad, ys_pad],
                                eps_pad.ravel(order="F"), {"degree": deg})

    x = ca.MX.sym("x")
    y = ca.MX.sym("y")
    s = ca.MX.sym("s")
    p = ca.vertcat(x, y, s)
    H = params.h_stand + params.z_tip0 + s

    xy = ca.vertcat(x, y)
    f_in = c_spline(xy) - eps_spline(xy) - H

    x_lo, x_hi = float(xs_pad[0]), float(xs_pad[-1])
    y_lo, y_hi = float(ys_pad[0]), float(ys_pad[-1])
    inside = ca.logic_and(ca.logic_and(x >= x_lo, x <= x_hi),
                          ca.logic_and(y >= y_lo, y <= y_hi))
    dout = (ca.fmax(0.0, x_lo - x) + ca.fmax(0.0, x - x_hi)
            + ca.fmax(0.0, y_lo - y) + ca.fmax(0.0, y - y_hi))
    f_oob = params.oob_value - params.oob_slope * dout
    f_expr = ca.if_else(inside, f_in, f_oob)

    f = ca.Function("f", [p], [f_expr], ["p"], ["f"])
    grad_f = ca.Function("grad_f", [p], [ca.jacobian(f_expr, p)], ["p"], ["grad_f"])

    domain = (float(xs[0]), float(xs[-1]), float(ys[0]), float(ys[-1]))
    fitted_extent = (x_lo, x_hi, y_lo, y_hi)
    meta = {"x_knots": xs_pad, "y_knots": ys_pad, "dx": dx, "dy": dy,
            "frame_id": snapshot.frame_id, "seven_state": seven_state, "params": params}
    return ClearanceSpline(f=f, grad_f=grad_f, c_spline=c_spline, eps_spline=eps_spline,
                           domain=domain, fitted_extent=fitted_extent, meta=meta)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python -m pytest test/test_clearance_spline.py -k "parity or out_of_map or nan or all_zero or ravel" -v
```
Expected: PASS — 6 passed. (If parity fails near `1e-6`, the bug is the ravel order or a coord/shape transpose — do not loosen the tolerance.)

- [ ] **Step 5: Commit**

```bash
git add hilda_clearance_field/clearance_spline.py test/test_clearance_spline.py
git commit -m "feat(clearance_spline): builder + kernel parity + conservative edges [G3]

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Analytic gradient and 7-state

**Files:**
- Modify: `hilda_clearance_field/hilda_clearance_field/clearance_spline.py`
- Modify: `hilda_clearance_field/test/test_clearance_spline.py`

- [ ] **Step 1: Write the failing gradient + 7-state tests**

Append to `test/test_clearance_spline.py`:

```python
def test_grad_ds_is_minus_one_and_spatial_matches_fd():
    snap, _, s = _kernel_snapshot(s=0.30)
    cs = build_clearance_spline(snap, _PARAMS)
    x0, y0 = snap.x_coords[6], snap.y_coords[6]      # interior, away from edges
    g = np.array(cs.grad_f([x0, y0, s])).ravel()
    assert abs(g[2] - (-1.0)) < 1e-9                 # df/ds exact
    eps = 1e-4
    fx = (float(cs.f([x0 + eps, y0, s])) - float(cs.f([x0 - eps, y0, s]))) / (2 * eps)
    fy = (float(cs.f([x0, y0 + eps, s])) - float(cs.f([x0, y0 - eps, s]))) / (2 * eps)
    assert abs(g[0] - fx) < 1e-3
    assert abs(g[1] - fy) < 1e-3


def test_out_of_map_gradient_points_inward():
    snap, _, _ = _kernel_snapshot()
    cs = build_clearance_spline(snap, _PARAMS)
    x_hi, y_hi = cs.fitted_extent[1], cs.fitted_extent[3]
    g = np.array(cs.grad_f([x_hi + 1.0, y_hi + 1.0, 0.0])).ravel()
    assert g[0] < 0.0 and g[1] < 0.0                 # push x,y back toward the map
    assert abs(g[2]) < 1e-9                           # f_oob independent of s


def test_seven_state_promotes_h_with_exact_minus_one_gradient():
    snap, out, s = _kernel_snapshot(s=0.20)
    cs = build_clearance_spline(snap, _PARAMS, seven_state=True)
    assert cs.meta["seven_state"] is True
    x0, y0 = snap.x_coords[6], snap.y_coords[6]
    h0 = _PARAMS.h_stand
    # at h = h_stand the 7-state f equals the 6-state feasibility
    assert abs(float(cs.f([x0, y0, s, h0])) - out["feasibility"][6, 6]) < 1e-6
    g = np.array(cs.grad_f([x0, y0, s, h0])).ravel()
    assert g.shape == (4,)
    assert abs(g[2] - (-1.0)) < 1e-9                 # df/ds
    assert abs(g[3] - (-1.0)) < 1e-9                 # df/dh
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python -m pytest test/test_clearance_spline.py -k "grad or seven_state or inward" -v
```
Expected: the two gradient tests PASS (grad_f already exists from Task 3); `test_seven_state_*` FAILS with `NotImplementedError: seven_state added in a later step`.

- [ ] **Step 3: Implement the 7-state branch**

In `clearance_spline.py`, replace these lines in `build_clearance_spline`:
```python
    if seven_state:
        raise NotImplementedError("seven_state added in a later step")
```
... and ...
```python
    x = ca.MX.sym("x")
    y = ca.MX.sym("y")
    s = ca.MX.sym("s")
    p = ca.vertcat(x, y, s)
    H = params.h_stand + params.z_tip0 + s
```
with:
```python
    x = ca.MX.sym("x")
    y = ca.MX.sym("y")
    s = ca.MX.sym("s")
    if seven_state:
        h = ca.MX.sym("h")
        p = ca.vertcat(x, y, s, h)
        H = h + params.z_tip0 + s          # h promoted to a decision variable
    else:
        p = ca.vertcat(x, y, s)
        H = params.h_stand + params.z_tip0 + s
```
(The `if params.knot_stride != 1` guard stays; only the `seven_state` guard and the symbol block change.)

- [ ] **Step 4: Run tests to verify they pass**

```bash
python -m pytest test/test_clearance_spline.py -k "grad or seven_state or inward" -v
```
Expected: PASS — 3 passed.

- [ ] **Step 5: Commit**

```bash
git add hilda_clearance_field/clearance_spline.py test/test_clearance_spline.py
git commit -m "feat(clearance_spline): analytic gradient + 7-state h promotion [G3]

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: acados consumability gate

**Files:**
- Modify: `hilda_clearance_field/test/test_clearance_spline.py`

- [ ] **Step 1: Write the failing acados test**

Append to `test/test_clearance_spline.py`:

```python
def test_spline_constraint_solves_in_acados(tmp_path):
    """The real de-risking gate: f as an MX con_h_expr in a SQP-RTI OCP -> status 0."""
    import os
    import casadi as ca
    acados_template = pytest.importorskip("acados_template")
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

    snap, _, _ = _kernel_snapshot()
    cs = build_clearance_spline(snap, _PARAMS)

    m = AcadosModel()
    m.name = "spline_con"
    px, py, ss = ca.MX.sym("px"), ca.MX.sym("py"), ca.MX.sym("ss")
    vx, vy, vs = ca.MX.sym("vx"), ca.MX.sym("vy"), ca.MX.sym("vs")
    xdot = ca.MX.sym("xdot", 3)
    m.x = ca.vertcat(px, py, ss)
    m.u = ca.vertcat(vx, vy, vs)
    m.xdot = xdot
    m.f_expl_expr = ca.vertcat(vx, vy, vs)
    m.f_impl_expr = xdot - m.f_expl_expr
    m.con_h_expr = cs.f(ca.vertcat(px, py, ss))      # the exported spline as a constraint

    ocp = AcadosOcp()
    ocp.model = m
    N, nx, nu, ny = 10, 3, 3, 6
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = 1.0
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    Vx = np.zeros((ny, nx)); Vx[:nx, :nx] = np.eye(nx)
    Vu = np.zeros((ny, nu)); Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vx, ocp.cost.Vu = Vx, Vu
    ocp.cost.W = np.eye(ny); ocp.cost.yref = np.zeros(ny)
    ocp.cost.Vx_e = np.eye(nx); ocp.cost.W_e = np.eye(nx); ocp.cost.yref_e = np.zeros(nx)
    ocp.constraints.lh = np.array([0.0])             # f >= 0
    ocp.constraints.uh = np.array([1e6])
    # start inside the map where the kernel field is feasible
    ocp.constraints.x0 = np.array([float(snap.x_coords[6]), float(snap.y_coords[6]), 0.2])
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"

    os.chdir(tmp_path)
    solver = AcadosOcpSolver(ocp, json_file=str(tmp_path / "ocp.json"), verbose=False)
    status = solver.solve()
    assert status == 0
```

- [ ] **Step 2: Run the test**

```bash
python -m pytest test/test_clearance_spline.py::test_spline_constraint_solves_in_acados -v
```
Expected: PASS — `status == 0` (build → codegen → t_renderer → C compile → HPIPM). First run compiles, so it is slow (~20-40 s). If `acados_template` is not importable, the test SKIPS (not fails).

- [ ] **Step 3: Commit**

```bash
git add test/test_clearance_spline.py
git commit -m "test(clearance_spline): acados SQP-RTI consumability gate [G3]

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Full suite + doc/journal sync

**Files:**
- Modify (thesis repo): `thesis/docs/02_variance_aware_clearance.md`, `thesis/docs/03_nmpc_formulation.md`, `thesis/docs/decisions/0013-clearance-spline-interface.md`, `thesis/journal/2026-W22.md`

- [ ] **Step 1: Run the full suite (both files)**

```bash
source ~/ros2_ws/.venv-acados/acados_env.sh
cd /home/odesha/ros2_ws/src/hilda_clearance_field
python -m pytest test/ -v
```
Expected: all `test_prototype_kernel.py` (13) + `test_clearance_spline.py` (16) pass; acados test passes or skips. Confirm 0 failures.

- [ ] **Step 2: Update doc 02 §Implementation status**

In `thesis/docs/02_variance_aware_clearance.md` §Implementation status, append a 2026-05-30 entry recording: deliverable #1 landed (two-spline + affine `H` form, `f = c − ε − H`); `NaN`/pad conservative-infeasible convention; `λ` baked into the interpolated `ε` (option-C variant named for live-`λ`); cell-centre parity + acados gate against `prototype_kernel`. Note the adapter + online refresh remain deferred to 03/05.

- [ ] **Step 3: Update doc 03 + flip ADR 0013 to accepted**

In `thesis/docs/03_nmpc_formulation.md`, note the spline interface exists (`f`, `grad_f`) and uses `f = c − ε − H`; cross-link the spec. In `thesis/docs/decisions/0013-clearance-spline-interface.md`, change `Status: proposed` to `Status: accepted`.

- [ ] **Step 4: Journal entry**

Append a 2026-05-30 entry to `thesis/journal/2026-W22.md`: the brainstorm decisions, the CasADi 3.7.2 probe findings (SX works, #2837 not reproduced, acados spline-constraint `status 0`), Grandia Eq. 32 confirmed verbatim against the library, the `h_stand`/§Notation-vs-§3 tension, and that the adapter is deferred. Note the `Enrico et al. 2025` source is not in Zotero (doc 03/05 provenance to-do).

- [ ] **Step 5: Commit the thesis docs**

```bash
git -C /home/odesha/ros2_ws/src/thesis add docs/02_variance_aware_clearance.md \
  docs/03_nmpc_formulation.md docs/decisions/0013-clearance-spline-interface.md \
  journal/2026-W22.md docs/superpowers/specs/2026-05-30-clearance-spline-export-design.md \
  docs/superpowers/plans/2026-05-30-clearance-spline-export.md
git -C /home/odesha/ros2_ws/src/thesis commit -m "03 + ADR 0013 + journal: clearance-spline export landed [G3]

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

(The code lives in the separate `hilda_clearance_field` repo; the impl-status doc updates land in the thesis repo right after the code commits — same session, not deferred. Cross-repo, so not literally the same commit.)

---

## Notes for the executor

- **Tolerances are load-bearing.** If `1e-6` cell-centre parity fails, the cause is ravel order, a coord/shape transpose, or a degree/min-cells mismatch — fix the bug, do not loosen the gate (per [[feedback-quantified-acceptance-gates]]).
- **Never `pip install` anything that drags numpy** — apt numpy must stay on the import path (CLAUDE.md).
- **First acados run is slow** (codegen + compile). The `t_renderer` binary is pre-placed (ADR 0012), so it will not prompt.
- Do not add the GridMap adapter, online refresh, or the acados OCP prototype here — they are separate deliverables (spec §Scope).
