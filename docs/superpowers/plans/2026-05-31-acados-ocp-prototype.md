# Standalone acados OCP prototype (module 03) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A ROS-free Python prototype of HILDA's 6-state SQP-RTI perceptive controller that queries the landed clearance spline `cs.f` as a soft constraint, drives a plant integrator through a synthetic too-low-beam scene where the sledge `s` must lower to pass, and is exercised by committed YAMLs with falsifiable gates — confirming the joint-optimisation hypothesis against a fixed-`s` baseline.

**Architecture:** Code in `hilda_nmpc/` (ament_python, venv-only per ADR 0012). `config.py` holds frozen dataclasses; `scenarios.py` builds synthetic `ClearanceSpline` fields; `model.py` builds the `AcadosModel`; `ocp.py` builds the `AcadosOcpSolver` (cost `NONLINEAR_LS`+Gauss-Newton, soft `con_h_expr=cs.f` via `idxsh` L1 slack); `plant.py` integrates the same dynamics (RK4, no model mismatch); `closed_loop.py` runs the RTI loop; `gates.py` evaluates the verdict. The experiment configs + runner live in the thesis repo's `experiments/`. Two orthogonal constraint axes (mode × barrier) are captured in the schema; only `soft`/`hard` + barrier-off are built (barrier-on / `none` reserved with `NotImplementedError`).

**Tech Stack:** Python 3.12, CasADi 3.7.2, acados v0.5.4 (`acados_template`), NumPy (apt), pytest, PyYAML. Source `~/ros2_ws/.venv-acados/acados_env.sh` before any solve.

---

## Environment

Two repos:
- **Code:** `~/ros2_ws/src/hilda_nmpc/` (its own git repo; branch per module).
- **Experiments + docs:** `~/ros2_ws/src/thesis/` (git repo `main`).

Run recipe (tests and runner both need the venv + the workspace overlay so `hilda_nmpc` *and* `hilda_clearance_field` import):

```bash
source ~/ros2_ws/.venv-acados/acados_env.sh                 # casadi + acados_template + apt numpy
cd ~/ros2_ws && colcon build --symlink-install --packages-select hilda_clearance_field hilda_nmpc
source ~/ros2_ws/install/setup.bash                          # makes both packages importable
cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/ -v    # the gate
```

acados codegen needs the pre-placed `t_renderer` (ADR 0012) — already in `tools/acados/bin/`. Solve-dependent tests use `pytest.importorskip("acados_template")` so the structure tests still run where acados is absent.

## File structure

**`hilda_nmpc/` (code repo, branch `feature/ocp-prototype`):**
- `hilda_nmpc/hilda_nmpc/config.py` — `OcpConfig`, `SceneConfig`, `GateConfig` frozen dataclasses + per-type loaders + `load_experiment(path)`. Validation of the two constraint axes.
- `hilda_nmpc/hilda_nmpc/scenarios.py` — `build_beam_field(scene) -> ClearanceSpline` + the comfortable-feasibility assertion.
- `hilda_nmpc/hilda_nmpc/model.py` — `build_model(cfg) -> AcadosModel` (6 states, 3 controls, MX, ERK).
- `hilda_nmpc/hilda_nmpc/plant.py` — `dynamics(x,u,cfg)`, `plant_step(x,u,dt,cfg)` (RK4).
- `hilda_nmpc/hilda_nmpc/ocp.py` — `build_ocp(field,cfg,scene,json_path) -> AcadosOcpSolver`.
- `hilda_nmpc/hilda_nmpc/closed_loop.py` — `run_closed_loop(solver,field,scene,cfg) -> dict`.
- `hilda_nmpc/hilda_nmpc/gates.py` — `evaluate_gates(record,scene,gate_cfg) -> dict`.
- `hilda_nmpc/setup.py` — add `casadi`, `acados_template` to `install_requires`.
- `hilda_nmpc/test/test_{config,scenarios,model,plant,ocp,closed_loop,gates}.py`.

**`thesis/` (docs repo, `main`):**
- `thesis/experiments/configs/sim_validation_03a/transit.yaml` + `README.md`
- `thesis/experiments/configs/sim_validation_03b/fixed_s_ablation.yaml` + `README.md`
- `thesis/experiments/configs/sim_validation_03c/drill_target.yaml` + `README.md`
- `thesis/experiments/runners/sim_validation_03_runner.py`
- `thesis/docs/decisions/0014-ocp-constraint-config-axes.md`
- `thesis/docs/03_nmpc_formulation.md` (impl-status sync), `thesis/journal/2026-W22.md` (entry).

**Locked signatures** (use verbatim across tasks):
- `OcpConfig`, `SceneConfig`, `GateConfig` — fields fixed in Task 1.
- `build_beam_field(scene: SceneConfig) -> ClearanceSpline`
- `build_model(cfg: OcpConfig) -> AcadosModel`
- `dynamics(x: np.ndarray, u: np.ndarray, cfg: OcpConfig) -> np.ndarray`; `plant_step(x, u, dt: float, cfg: OcpConfig) -> np.ndarray`
- `build_ocp(field: ClearanceSpline, cfg: OcpConfig, scene: SceneConfig, json_path: str) -> AcadosOcpSolver`
- `run_closed_loop(solver, field: ClearanceSpline, scene: SceneConfig, cfg: OcpConfig) -> dict` — record keys `states (n,6)`, `controls (n,3)`, `f (n,)`, `slack (n,)`, `solve_times (n,)`, `status (n,)`, `reached: bool`, `n_steps: int`.
- `evaluate_gates(record: dict, scene: SceneConfig, gate_cfg: GateConfig) -> dict` — verdict keys `reach_err`, `reach`, `f_min`, `feasible`, `max_slack`, `s_in_beam_min`, `s_lowering`, `solve_p50/p95/max`, `solve_ok`, `hypothesis_confirmed`, `verdict`.

---

## Task 0: Branch, deps, empty modules

**Files:**
- Modify: `hilda_nmpc/setup.py`
- Create: `hilda_nmpc/hilda_nmpc/{config,scenarios,model,plant,ocp,closed_loop,gates}.py` (empty)

- [ ] **Step 1: Create the feature branch**

```bash
cd ~/ros2_ws/src/hilda_nmpc
git checkout -b feature/ocp-prototype
```

- [ ] **Step 2: Add solver deps to setup.py**

In `hilda_nmpc/setup.py`, change the `install_requires` line:

```python
    install_requires=["setuptools", "casadi", "acados_template"],
```

- [ ] **Step 3: Create empty module files**

```bash
cd ~/ros2_ws/src/hilda_nmpc/hilda_nmpc
touch config.py scenarios.py model.py plant.py ocp.py closed_loop.py gates.py
```

- [ ] **Step 4: Commit**

```bash
cd ~/ros2_ws/src/hilda_nmpc
git add setup.py hilda_nmpc/
git commit -m "scaffold: ocp-prototype modules + solver deps [G3]"
```

---

## Task 1: Config dataclasses (`config.py`)

**Files:**
- Create: `hilda_nmpc/hilda_nmpc/config.py`
- Test: `hilda_nmpc/test/test_config.py`

- [ ] **Step 1: Write the failing test**

```python
# hilda_nmpc/test/test_config.py
import pytest
from hilda_nmpc.config import (
    OcpConfig, SceneConfig, GateConfig,
    load_ocp_config, load_scene_config, load_gate_config,
)


def test_ocp_config_defaults():
    c = OcpConfig()
    assert c.N == 40 and c.tf == 4.0
    assert c.constraint_mode == "soft" and c.barrier == "off"
    assert c.eta_v == 1.0 and c.eta_w == 1.0


def test_ocp_config_rejects_bad_constraint_mode():
    with pytest.raises(ValueError, match="constraint_mode"):
        OcpConfig(constraint_mode="squishy")


def test_ocp_config_rejects_bad_barrier():
    with pytest.raises(ValueError, match="barrier"):
        OcpConfig(barrier="maybe")


def test_ocp_config_rejects_degenerate_none_off():
    # f does not enter at all -> rejected
    with pytest.raises(ValueError, match="degenerate"):
        OcpConfig(constraint_mode="none", barrier="off")


def test_loaders_pick_only_known_keys():
    c = load_ocp_config({"N": 20, "tf": 2.0, "unknown": 99})
    assert c.N == 20 and c.tf == 2.0
    s = load_scene_config({"name": "x", "beam_x_center": 2.0, "junk": 1})
    assert s.name == "x" and s.beam_x_center == 2.0
    g = load_gate_config({"scene_kind": "ablation"})
    assert g.scene_kind == "ablation"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_config.py -v`
Expected: FAIL with `ModuleNotFoundError`/`ImportError` (config symbols not defined).

- [ ] **Step 3: Write the implementation**

```python
# hilda_nmpc/hilda_nmpc/config.py
"""Frozen config dataclasses for the module-03 OCP prototype.

Offline tooling (no ROS parameter declaration). The constraint surface is two
orthogonal axes (ADR 0014): constraint_mode (soft|hard|none) x barrier (off|on).
Only soft/hard + barrier=off are built here; barrier=on and constraint_mode=none
are reserved (ocp.build_ocp raises NotImplementedError).
"""
from __future__ import annotations

from dataclasses import dataclass, fields

_CONSTRAINT_MODES = ("soft", "hard", "none")
_BARRIER_STATES = ("off", "on")


@dataclass(frozen=True)
class OcpConfig:
    # horizon
    N: int = 40
    tf: float = 4.0                  # s; dt = tf/N = 0.1 s
    # model (ICR factors — PLACEHOLDER pending calibration; 1.0 == Level 0)
    eta_v: float = 1.0
    eta_w: float = 1.0
    # state box bounds
    v_max: float = 0.5               # m/s
    w_max: float = 0.5               # rad/s
    s_max: float = 2.18              # m (doc 03 §4)
    # input box bounds (controller ramp limits, doc 03 §6)
    a_ramp: float = 1.2              # m/s^2
    alpha_ramp: float = 0.8          # rad/s^2
    u_s_max: float = 0.30            # m/s — PLACEHOLDER conservative (folds q1/q2)
    # stage cost weights
    q_xy: float = 10.0
    q_th: float = 1.0
    q_v: float = 0.1
    q_w: float = 0.1
    r_a: float = 0.1
    r_alpha: float = 0.1
    r_us: float = 1.0                # sledge rate penalised harder (bandwidth gap)
    # terminal cost weights
    q_xy_e: float = 50.0
    q_th_e: float = 5.0
    q_s_e: float = 0.0               # active (>0) only for drill-target scene (03c)
    # constraint axes
    constraint_mode: str = "soft"    # soft | hard | none
    barrier: str = "off"             # off | on (on reserved)
    zl: float = 1.0e3                # L1 slack penalty (lower slack on f >= 0)
    Zl: float = 1.0e1                # small L2 conditioning on the slack
    # static-stability constraint (doc 03 §4 |a| <= a_max(s)) — RESERVED, deferred-for-scene
    stability_enabled: bool = False

    def __post_init__(self):
        if self.constraint_mode not in _CONSTRAINT_MODES:
            raise ValueError(
                f"constraint_mode must be one of {_CONSTRAINT_MODES}, got {self.constraint_mode!r}")
        if self.barrier not in _BARRIER_STATES:
            raise ValueError(
                f"barrier must be one of {_BARRIER_STATES}, got {self.barrier!r}")
        if self.constraint_mode == "none" and self.barrier == "off":
            raise ValueError(
                "degenerate config: constraint_mode='none' + barrier='off' -> f never enters")


@dataclass(frozen=True)
class SceneConfig:
    name: str = "scene"
    # grid extent + resolution
    x_min: float = -0.5
    x_max: float = 4.5
    y_min: float = -1.0
    y_max: float = 1.0
    res: float = 0.10
    # field
    z_floor: float = 0.0
    z_ceil_nominal: float = 3.0
    eps_const: float = 0.10
    # beam (full corridor width; lowered ceiling)
    beam_x_center: float = 2.0
    beam_x_halfwidth: float = 0.30
    beam_z_ceil: float = 2.40
    # robot datum (must match the spline build)
    h_stand: float = 0.10
    z_tip0: float = 1.899
    # start / goal
    start_x: float = 0.0
    start_y: float = 0.0
    start_theta: float = 0.0
    s_nominal: float = 0.50
    goal_x: float = 4.0
    goal_y: float = 0.0
    goal_theta: float = 0.0
    s_goal: float = 0.0              # terminal drill extension (03c); 0 + q_s_e=0 -> inactive
    v_approach: float = 0.40         # nominal approach speed for feasibility check
    fixed_s: bool = False            # 03b: freeze s at s_nominal
    sim_time: float = 16.0           # s of closed-loop simulation


@dataclass(frozen=True)
class GateConfig:
    scene_kind: str = "controller"   # "controller" | "ablation"
    reach_tol: float = 0.15          # m
    f_tol: float = 0.05              # m, permitted soft-slack excursion
    s_drop_min: float = 0.05         # m, min s-lowering under beam
    solve_time_p95_max: float = 0.05  # s, descriptive (dev-machine, NOT Fig-5)
    s_tol: float = 0.10              # m, terminal-s tolerance (03c)
    require_terminal_s: bool = False  # 03c sets True


def _filter(d: dict, cls) -> dict:
    known = {f.name for f in fields(cls)}
    return {k: v for k, v in (d or {}).items() if k in known}


def load_ocp_config(d: dict) -> OcpConfig:
    return OcpConfig(**_filter(d, OcpConfig))


def load_scene_config(d: dict) -> SceneConfig:
    return SceneConfig(**_filter(d, SceneConfig))


def load_gate_config(d: dict) -> GateConfig:
    return GateConfig(**_filter(d, GateConfig))


def load_experiment(path) -> tuple[OcpConfig, SceneConfig, GateConfig]:
    """Load an experiment YAML with top-level keys `ocp`, `scene`, `gates`."""
    import pathlib
    import yaml
    data = yaml.safe_load(pathlib.Path(path).read_text())
    return (load_ocp_config(data.get("ocp", {})),
            load_scene_config(data.get("scene", {})),
            load_gate_config(data.get("gates", {})))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_config.py -v`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add hilda_nmpc/config.py test/test_config.py
git commit -m "config: OcpConfig/SceneConfig/GateConfig + two-axis validation [G3]"
```

---

## Task 2: Synthetic beam field (`scenarios.py`)

**Files:**
- Create: `hilda_nmpc/hilda_nmpc/scenarios.py`
- Test: `hilda_nmpc/test/test_scenarios.py`

- [ ] **Step 1: Write the failing test**

```python
# hilda_nmpc/test/test_scenarios.py
import numpy as np
import pytest

from hilda_nmpc.config import SceneConfig
from hilda_nmpc.scenarios import build_beam_field, retraction_feasibility


def test_beam_forces_lowering_but_is_comfortably_feasible():
    scene = SceneConfig()
    cs = build_beam_field(scene)
    # Under the beam at nominal extension: infeasible (must lower s).
    f_nom = float(cs.f([scene.beam_x_center, 0.0, scene.s_nominal]))
    assert f_nom < 0.0
    # Under the beam fully retracted: feasible (lowering clears it).
    f_clear = float(cs.f([scene.beam_x_center, 0.0, 0.0]))
    assert f_clear > 0.0
    # Away from the beam at nominal extension: feasible (free transit).
    f_free = float(cs.f([scene.start_x, 0.0, scene.s_nominal]))
    assert f_free > 0.0


def test_retraction_feasibility_has_time_margin():
    scene = SceneConfig()
    margin = retraction_feasibility(scene, u_s_max=0.30)
    assert margin["feasible"] is True
    assert margin["time_to_beam_s"] > margin["retraction_time_s"]


def test_marginal_beam_is_rejected():
    # Beam very close, deep nominal extension: cannot retract in time.
    scene = SceneConfig(beam_x_center=0.2, s_nominal=2.0)
    margin = retraction_feasibility(scene, u_s_max=0.30)
    assert margin["feasible"] is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_scenarios.py -v`
Expected: FAIL with `ImportError` (`build_beam_field` not defined).

- [ ] **Step 3: Write the implementation**

```python
# hilda_nmpc/hilda_nmpc/scenarios.py
"""Synthetic clearance-field scenes for the module-03 OCP prototype.

Builds a ROS-free, static ClearanceSpline (hilda_clearance_field) with a
full-corridor-width low beam sized so that at the nominal sledge extension the
mast-top would hit the beam (f<0) but a reachable lower s clears it (f>=0).
Comfortable-feasibility is asserted: the robot must be able to retract in time.
"""
from __future__ import annotations

import numpy as np

from hilda_clearance_field.clearance_spline import (
    GridSnapshot, SplineParams, build_clearance_spline, ClearanceSpline,
)
from .config import SceneConfig


def build_beam_field(scene: SceneConfig) -> ClearanceSpline:
    xs = np.arange(scene.x_min, scene.x_max + 0.5 * scene.res, scene.res)
    ys = np.arange(scene.y_min, scene.y_max + 0.5 * scene.res, scene.res)
    nx, ny = xs.size, ys.size
    z_ceil = np.full((nx, ny), scene.z_ceil_nominal, dtype=float)
    beam_lo = scene.beam_x_center - scene.beam_x_halfwidth
    beam_hi = scene.beam_x_center + scene.beam_x_halfwidth
    beam_cols = (xs >= beam_lo) & (xs <= beam_hi)         # full y-width stripe
    z_ceil[beam_cols, :] = scene.beam_z_ceil
    clearance = z_ceil - scene.z_floor
    epsilon = np.full((nx, ny), scene.eps_const, dtype=float)
    snap = GridSnapshot(clearance=clearance, epsilon=epsilon,
                        x_coords=xs, y_coords=ys, frame_id="odom")
    params = SplineParams(h_stand=scene.h_stand, z_tip0=scene.z_tip0)
    return build_clearance_spline(snap, params, seven_state=False)


def retraction_feasibility(scene: SceneConfig, u_s_max: float) -> dict:
    """Comfortable-feasibility check (doc 03 / spec flag 4).

    s_clear is the extension at which the beam is exactly cleared:
        c_beam - eps - (h_stand + z_tip0 + s_clear) = 0
    Require s_clear < s_nominal (lowering needed) and the retraction achievable
    before reaching the beam, with margin.
    """
    c_beam = scene.beam_z_ceil - scene.z_floor
    datum0 = scene.h_stand + scene.z_tip0           # H at s=0
    s_clear = c_beam - scene.eps_const - datum0     # f=0 extension
    must_lower = s_clear < scene.s_nominal
    drop = max(0.0, scene.s_nominal - max(0.0, s_clear))
    retraction_time = drop / u_s_max if u_s_max > 0 else float("inf")
    beam_lo = scene.beam_x_center - scene.beam_x_halfwidth
    dist_to_beam = max(0.0, beam_lo - scene.start_x)
    time_to_beam = dist_to_beam / scene.v_approach if scene.v_approach > 0 else 0.0
    feasible = bool(must_lower and s_clear >= -1e-9
                    and time_to_beam > retraction_time)
    return {
        "s_clear": s_clear,
        "must_lower": must_lower,
        "retraction_time_s": retraction_time,
        "time_to_beam_s": time_to_beam,
        "feasible": feasible,
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws && colcon build --symlink-install --packages-select hilda_clearance_field hilda_nmpc && source install/setup.bash && cd src/hilda_nmpc && python3 -m pytest test/test_scenarios.py -v`
Expected: PASS (3 tests). (Needs the venv + overlay so `hilda_clearance_field` imports.)

- [ ] **Step 5: Commit**

```bash
git add hilda_nmpc/scenarios.py test/test_scenarios.py
git commit -m "scenarios: synthetic low-beam field + comfortable-feasibility check [G3]"
```

---

## Task 3: acados model (`model.py`)

**Files:**
- Create: `hilda_nmpc/hilda_nmpc/model.py`
- Test: `hilda_nmpc/test/test_model.py`

- [ ] **Step 1: Write the failing test**

```python
# hilda_nmpc/test/test_model.py
import numpy as np
import pytest

ca = pytest.importorskip("casadi")
pytest.importorskip("acados_template")

from hilda_nmpc.config import OcpConfig
from hilda_nmpc.model import build_model


def test_model_dimensions():
    m = build_model(OcpConfig())
    assert m.x.shape[0] == 6
    assert m.u.shape[0] == 3
    assert m.f_expl_expr.shape[0] == 6
    assert m.name == "hilda_ocp6"


def test_dynamics_values_at_sample():
    m = build_model(OcpConfig(eta_v=1.0, eta_w=1.0))
    fn = ca.Function("f", [m.x, m.u], [m.f_expl_expr])
    x = [0.0, 0.0, 0.0, 0.5, 0.2, 0.3]   # th=0 -> xdot=v, ydot=0
    u = [0.1, -0.1, 0.05]
    xd = np.array(fn(x, u)).ravel()
    np.testing.assert_allclose(xd, [0.5, 0.0, 0.2, 0.1, -0.1, 0.05], atol=1e-12)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_model.py -v`
Expected: FAIL with `ImportError` (`build_model` not defined).

- [ ] **Step 3: Write the implementation**

```python
# hilda_nmpc/hilda_nmpc/model.py
"""acados model for the 6-state perceptive OCP (doc 03 §Architectural commitments).

x = (x, y, theta, v, omega, s),  u = (a, alpha, u_s).
ICR-corrected unicycle (Level 1) with eta_v=eta_w build-time constants
(PLACEHOLDER, default 1.0 == Level 0); first-order integrator on s. Built in MX
(acados modelling type) to match the spline acados gate.
"""
from __future__ import annotations

import casadi as ca
from acados_template import AcadosModel

from .config import OcpConfig


def build_model(cfg: OcpConfig) -> AcadosModel:
    x = ca.MX.sym("x")
    y = ca.MX.sym("y")
    th = ca.MX.sym("th")
    v = ca.MX.sym("v")
    w = ca.MX.sym("w")
    s = ca.MX.sym("s")
    state = ca.vertcat(x, y, th, v, w, s)

    a = ca.MX.sym("a")
    al = ca.MX.sym("al")
    us = ca.MX.sym("us")
    u = ca.vertcat(a, al, us)

    f_expl = ca.vertcat(
        cfg.eta_v * v * ca.cos(th),
        cfg.eta_v * v * ca.sin(th),
        cfg.eta_w * w,
        a,
        al,
        us,
    )
    xdot = ca.MX.sym("xdot", 6)

    m = AcadosModel()
    m.name = "hilda_ocp6"
    m.x = state
    m.u = u
    m.xdot = xdot
    m.f_expl_expr = f_expl
    m.f_impl_expr = xdot - f_expl
    return m
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_model.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add hilda_nmpc/model.py test/test_model.py
git commit -m "model: 6-state ICR-unicycle AcadosModel (eta placeholders) [G3]"
```

---

## Task 4: Plant integrator (`plant.py`)

**Files:**
- Create: `hilda_nmpc/hilda_nmpc/plant.py`
- Test: `hilda_nmpc/test/test_plant.py`

- [ ] **Step 1: Write the failing test**

```python
# hilda_nmpc/test/test_plant.py
import numpy as np

from hilda_nmpc.config import OcpConfig
from hilda_nmpc.plant import dynamics, plant_step


def test_dynamics_matches_model_form():
    cfg = OcpConfig(eta_v=1.0, eta_w=1.0)
    x = np.array([0.0, 0.0, 0.0, 0.5, 0.2, 0.3])
    u = np.array([0.1, -0.1, 0.05])
    np.testing.assert_allclose(dynamics(x, u, cfg),
                               [0.5, 0.0, 0.2, 0.1, -0.1, 0.05], atol=1e-12)


def test_straight_line_step_advances_x():
    cfg = OcpConfig()
    x = np.array([0.0, 0.0, 0.0, 0.5, 0.0, 0.5])
    x1 = plant_step(x, np.zeros(3), dt=0.1, cfg=cfg)
    assert x1[0] > 0.049 and x1[0] < 0.051   # ~0.5 m/s * 0.1 s
    assert abs(x1[1]) < 1e-9                  # no lateral motion
    assert abs(x1[5] - 0.5) < 1e-12          # s unchanged when u_s=0 (fixed-s)


def test_s_integrates_u_s():
    cfg = OcpConfig()
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.5])
    x1 = plant_step(x, np.array([0.0, 0.0, -0.2]), dt=0.1, cfg=cfg)
    assert abs(x1[5] - (0.5 - 0.02)) < 1e-9  # s -= 0.2 * 0.1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_plant.py -v`
Expected: FAIL with `ImportError` (`dynamics` not defined).

- [ ] **Step 3: Write the implementation**

```python
# hilda_nmpc/hilda_nmpc/plant.py
"""Plant integrator for the closed-loop sim. RK4 of the SAME dynamics the OCP
model uses (plant = model: no mismatch in this first isolation prototype).
"""
from __future__ import annotations

import numpy as np

from .config import OcpConfig


def dynamics(x: np.ndarray, u: np.ndarray, cfg: OcpConfig) -> np.ndarray:
    px, py, th, v, w, s = x
    a, al, us = u
    return np.array([
        cfg.eta_v * v * np.cos(th),
        cfg.eta_v * v * np.sin(th),
        cfg.eta_w * w,
        a,
        al,
        us,
    ])


def plant_step(x: np.ndarray, u: np.ndarray, dt: float, cfg: OcpConfig) -> np.ndarray:
    x = np.asarray(x, dtype=float)
    u = np.asarray(u, dtype=float)
    k1 = dynamics(x, u, cfg)
    k2 = dynamics(x + 0.5 * dt * k1, u, cfg)
    k3 = dynamics(x + 0.5 * dt * k2, u, cfg)
    k4 = dynamics(x + dt * k3, u, cfg)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_plant.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add hilda_nmpc/plant.py test/test_plant.py
git commit -m "plant: RK4 integrator of the OCP dynamics (plant=model) [G3]"
```

---

## Task 5: OCP builder + single-solve gate (`ocp.py`) — checkpoint 1

**Files:**
- Create: `hilda_nmpc/hilda_nmpc/ocp.py`
- Test: `hilda_nmpc/test/test_ocp.py`

- [ ] **Step 1: Write the failing test (single-solve status 0 + reserved-branch guards)**

```python
# hilda_nmpc/test/test_ocp.py
import numpy as np
import pytest

pytest.importorskip("casadi")
pytest.importorskip("acados_template")

from hilda_nmpc.config import OcpConfig, SceneConfig
from hilda_nmpc.scenarios import build_beam_field
from hilda_nmpc.ocp import build_ocp


def test_barrier_on_is_reserved():
    scene = SceneConfig()
    field = build_beam_field(scene)
    with pytest.raises(NotImplementedError, match="barrier"):
        build_ocp(field, OcpConfig(barrier="on"), scene, "/tmp/ocp_resv.json")


def test_none_mode_is_reserved():
    scene = SceneConfig()
    field = build_beam_field(scene)
    with pytest.raises(NotImplementedError, match="none"):
        build_ocp(field, OcpConfig(constraint_mode="none", barrier="on"),
                  scene, "/tmp/ocp_none.json")


def test_stability_constraint_is_reserved():
    scene = SceneConfig()
    field = build_beam_field(scene)
    with pytest.raises(NotImplementedError, match="stability"):
        build_ocp(field, OcpConfig(stability_enabled=True), scene, "/tmp/ocp_stab.json")


def test_single_solve_status_zero(tmp_path, monkeypatch):
    """Checkpoint 1: the full 6-state soft+off OCP solves on the beam field."""
    scene = SceneConfig()
    field = build_beam_field(scene)
    monkeypatch.chdir(tmp_path)
    solver = build_ocp(field, OcpConfig(), scene, str(tmp_path / "ocp.json"))
    status = solver.solve()
    assert status == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_ocp.py -v`
Expected: FAIL with `ImportError` (`build_ocp` not defined).

- [ ] **Step 3: Write the implementation**

```python
# hilda_nmpc/hilda_nmpc/ocp.py
"""acados OCP builder for the module-03 prototype.

Baseline = constraint_mode 'soft' + barrier 'off': NONLINEAR_LS cost (goal via
yref) + Gauss-Newton + HPIPM, soft con_h_expr = cs.f([x,y,s]) via idxsh (L1
slack). 'hard' drops idxsh. barrier='on' (CONVEX_OVER_NONLINEAR) and
constraint_mode='none' are RESERVED (ADR 0014) -> NotImplementedError.
"""
from __future__ import annotations

import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver

from .config import OcpConfig, SceneConfig
from .model import build_model


def build_ocp(field, cfg: OcpConfig, scene: SceneConfig, json_path: str) -> AcadosOcpSolver:
    if cfg.barrier == "on":
        raise NotImplementedError(
            "barrier='on' (CONVEX_OVER_NONLINEAR relaxed log-barrier) is reserved; "
            "see ADR 0014 / spec. Only barrier='off' is built in this prototype.")
    if cfg.constraint_mode == "none":
        raise NotImplementedError(
            "constraint_mode='none' (f only in the cost barrier, Baseline C) requires "
            "barrier='on', which is reserved; see ADR 0014.")
    if cfg.stability_enabled:
        raise NotImplementedError(
            "static-stability constraint (doc 03 §4 option-B linear over-approx |a|<=a_max(s)) "
            "is deferred-for-scene; reserved. See spec §Deferred-for-this-scene.")

    ocp = AcadosOcp()
    ocp.model = build_model(cfg)
    nx, nu = 6, 3
    ocp.solver_options.N_horizon = cfg.N
    ocp.solver_options.tf = cfg.tf

    xs = ocp.model.x
    us = ocp.model.u
    px, py, th, v, w, s = xs[0], xs[1], xs[2], xs[3], xs[4], xs[5]
    a, al, u_s = us[0], us[1], us[2]

    # --- cost: NONLINEAR_LS, goal via yref, Gauss-Newton ---
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr = ca.vertcat(px, py, th, v, w, a, al, u_s)   # ny = 8
    ocp.model.cost_y_expr_e = ca.vertcat(px, py, th, s)                # ny_e = 4
    ocp.cost.W = np.diag([cfg.q_xy, cfg.q_xy, cfg.q_th, cfg.q_v, cfg.q_w,
                          cfg.r_a, cfg.r_alpha, cfg.r_us]).astype(float)
    ocp.cost.W_e = np.diag([cfg.q_xy_e, cfg.q_xy_e, cfg.q_th_e, cfg.q_s_e]).astype(float)
    ocp.cost.yref = np.array([scene.goal_x, scene.goal_y, scene.goal_theta,
                              0.0, 0.0, 0.0, 0.0, 0.0])
    ocp.cost.yref_e = np.array([scene.goal_x, scene.goal_y, scene.goal_theta, scene.s_goal])

    # --- soft/hard clearance constraint: f >= 0 ---
    f_expr = field.f(ca.vertcat(px, py, s))
    ocp.model.con_h_expr = f_expr
    ocp.model.con_h_expr_e = f_expr
    ocp.constraints.lh = np.array([0.0])
    ocp.constraints.uh = np.array([1.0e6])
    ocp.constraints.lh_e = np.array([0.0])
    ocp.constraints.uh_e = np.array([1.0e6])
    if cfg.constraint_mode == "soft":
        ocp.constraints.idxsh = np.array([0])
        ocp.constraints.idxsh_e = np.array([0])
        ocp.cost.zl = np.array([cfg.zl]);  ocp.cost.zu = np.array([cfg.zl])
        ocp.cost.Zl = np.array([cfg.Zl]);  ocp.cost.Zu = np.array([cfg.Zl])
        ocp.cost.zl_e = np.array([cfg.zl]); ocp.cost.zu_e = np.array([cfg.zl])
        ocp.cost.Zl_e = np.array([cfg.Zl]); ocp.cost.Zu_e = np.array([cfg.Zl])
    # 'hard' -> no idxsh: f >= 0 is a hard inequality.

    # --- box bounds ---
    ocp.constraints.idxbx = np.array([3, 4, 5])          # v, w, s
    ocp.constraints.lbx = np.array([-cfg.v_max, -cfg.w_max, 0.0])
    ocp.constraints.ubx = np.array([cfg.v_max, cfg.w_max, cfg.s_max])
    u_s_hi = 0.0 if scene.fixed_s else cfg.u_s_max       # 03b freezes the sledge
    u_s_lo = 0.0 if scene.fixed_s else -cfg.u_s_max
    ocp.constraints.idxbu = np.array([0, 1, 2])          # a, alpha, u_s
    ocp.constraints.lbu = np.array([-cfg.a_ramp, -cfg.alpha_ramp, u_s_lo])
    ocp.constraints.ubu = np.array([cfg.a_ramp, cfg.alpha_ramp, u_s_hi])

    # --- initial state ---
    ocp.constraints.x0 = np.array([scene.start_x, scene.start_y, scene.start_theta,
                                   0.0, 0.0, scene.s_nominal])

    # --- solver ---
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"

    return AcadosOcpSolver(ocp, json_file=json_path, verbose=False)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_ocp.py -v`
Expected: PASS (4 tests: three reserved-branch guards + single-solve status 0).

- [ ] **Step 5: Commit**

```bash
git add hilda_nmpc/ocp.py test/test_ocp.py
git commit -m "ocp: 6-state soft+off SQP-RTI builder; single-solve status 0 [G3]"
```

---

## Task 6: Hard-mode flip (checkpoint 5)

**Files:**
- Modify: `hilda_nmpc/test/test_ocp.py`

- [ ] **Step 1: Write the failing test**

```python
# hilda_nmpc/test/test_ocp.py  (append)
def test_hard_mode_solves(tmp_path, monkeypatch):
    """Checkpoint 5: the RQ3 hard-arm flip still builds + solves (no slacks)."""
    scene = SceneConfig()
    field = build_beam_field(scene)
    monkeypatch.chdir(tmp_path)
    solver = build_ocp(field, OcpConfig(constraint_mode="hard"),
                       scene, str(tmp_path / "ocp_hard.json"))
    status = solver.solve()
    assert status == 0
```

- [ ] **Step 2: Run test to verify it fails or passes**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_ocp.py::test_hard_mode_solves -v`
Expected: PASS (the `hard` branch already exists in `ocp.py`). If it FAILS with an HPIPM infeasibility from the initial guess, set the start strictly feasible in the test scene (`s_nominal` already clears away from the beam at `x0`); no code change needed.

- [ ] **Step 3: (no implementation needed — branch exists)**

The `hard` path was implemented in Task 5. This task only adds its regression test.

- [ ] **Step 4: Run the full ocp test file**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_ocp.py -v`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add test/test_ocp.py
git commit -m "ocp: hard-mode flip regression (RQ3 lever) [G3]"
```

---

## Task 7: Closed loop (`closed_loop.py`) — checkpoints 2 + 3

**Files:**
- Create: `hilda_nmpc/hilda_nmpc/closed_loop.py`
- Test: `hilda_nmpc/test/test_closed_loop.py`

- [ ] **Step 1: Write the failing test**

```python
# hilda_nmpc/test/test_closed_loop.py
import numpy as np
import pytest

pytest.importorskip("casadi")
pytest.importorskip("acados_template")

from hilda_nmpc.config import OcpConfig, SceneConfig
from hilda_nmpc.scenarios import build_beam_field
from hilda_nmpc.ocp import build_ocp
from hilda_nmpc.closed_loop import run_closed_loop


def test_transit_reaches_goal_clears_beam_and_lowers_s(tmp_path, monkeypatch):
    """Checkpoints 2 + 3: reaches goal, f bounded, s dips under the beam."""
    scene = SceneConfig()                      # 03a transit defaults
    cfg = OcpConfig()
    field = build_beam_field(scene)
    monkeypatch.chdir(tmp_path)
    solver = build_ocp(field, cfg, scene, str(tmp_path / "ocp.json"))
    rec = run_closed_loop(solver, field, scene, cfg)

    states = np.asarray(rec["states"])
    final = states[-1]
    assert np.hypot(final[0] - scene.goal_x, final[1] - scene.goal_y) <= 0.15
    assert float(np.asarray(rec["f"]).min()) >= -0.05          # slack bounded
    beam_lo = scene.beam_x_center - scene.beam_x_halfwidth
    beam_hi = scene.beam_x_center + scene.beam_x_halfwidth
    in_beam = (states[:, 0] >= beam_lo) & (states[:, 0] <= beam_hi)
    assert in_beam.any()
    assert states[in_beam, 5].min() < scene.s_nominal - 0.05   # s lowered
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_closed_loop.py -v`
Expected: FAIL with `ImportError` (`run_closed_loop` not defined).

- [ ] **Step 3: Write the implementation**

```python
# hilda_nmpc/hilda_nmpc/closed_loop.py
"""Closed-loop RTI driver: re-solve, apply u0, integrate the plant, record.

Warm start = acados-native RTI shift across steps (no SMAC / 1D s-sweep — that
is integration, out of scope). Records the realised trajectory, the realised
clearance f (true physical clearance since plant=model + static field), the
solver lower-slack diagnostic, per-step solve time, and status.
"""
from __future__ import annotations

from time import perf_counter

import numpy as np

from .config import OcpConfig, SceneConfig
from .plant import plant_step


def _max_lower_slack(solver, N: int) -> float:
    best = 0.0
    for i in range(N):
        try:
            sl = np.asarray(solver.get(i, "sl")).ravel()
        except Exception:
            continue
        if sl.size:
            best = max(best, float(sl.max()))
    return best


def run_closed_loop(solver, field, scene: SceneConfig, cfg: OcpConfig) -> dict:
    dt = cfg.tf / cfg.N
    n_steps = int(round(scene.sim_time / dt))
    soft = (cfg.constraint_mode == "soft")

    x = np.array([scene.start_x, scene.start_y, scene.start_theta,
                  0.0, 0.0, scene.s_nominal])
    goal_xy = np.array([scene.goal_x, scene.goal_y])

    states, controls, f_hist, slack_hist, solve_t, status_hist = [], [], [], [], [], []
    reached = False
    for _ in range(n_steps):
        solver.set(0, "lbx", x)
        solver.set(0, "ubx", x)
        t0 = perf_counter()
        status = solver.solve()
        solve_t.append(perf_counter() - t0)
        status_hist.append(int(status))
        u0 = np.asarray(solver.get(0, "u")).ravel()

        states.append(x.copy())
        controls.append(u0.copy())
        f_hist.append(float(field.f([x[0], x[1], x[5]])))
        slack_hist.append(_max_lower_slack(solver, cfg.N) if soft else 0.0)

        if np.hypot(x[0] - goal_xy[0], x[1] - goal_xy[1]) <= 0.10:
            reached = True
            break
        x = plant_step(x, u0, dt, cfg)

    return {
        "states": np.asarray(states),
        "controls": np.asarray(controls),
        "f": np.asarray(f_hist),
        "slack": np.asarray(slack_hist),
        "solve_times": np.asarray(solve_t),
        "status": np.asarray(status_hist),
        "reached": reached,
        "n_steps": len(states),
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_closed_loop.py -v`
Expected: PASS. (If the robot stops short of the goal, raise `scene.sim_time` in the config defaults or `q_xy`; if it clips the beam, raise `cfg.zl`. Tune in the config, not the loop.)

- [ ] **Step 5: Commit**

```bash
git add hilda_nmpc/closed_loop.py test/test_closed_loop.py
git commit -m "closed_loop: RTI driver; 03a reaches goal, lowers s, f bounded [G3]"
```

---

## Task 8: Gate evaluation (`gates.py`)

**Files:**
- Create: `hilda_nmpc/hilda_nmpc/gates.py`
- Test: `hilda_nmpc/test/test_gates.py`

- [ ] **Step 1: Write the failing test (synthetic records — no solve needed)**

```python
# hilda_nmpc/test/test_gates.py
import numpy as np

from hilda_nmpc.config import SceneConfig, GateConfig
from hilda_nmpc.gates import evaluate_gates


def _record(states, f, slack=None, solve_times=None):
    n = len(states)
    return {
        "states": np.asarray(states, dtype=float),
        "controls": np.zeros((n, 3)),
        "f": np.asarray(f, dtype=float),
        "slack": np.asarray(slack if slack is not None else np.zeros(n), dtype=float),
        "solve_times": np.asarray(solve_times if solve_times is not None
                                  else np.full(n, 0.002), dtype=float),
        "status": np.zeros(n, dtype=int),
        "reached": True,
        "n_steps": n,
    }


def test_controller_pass():
    scene = SceneConfig()
    # robot crosses the beam at lowered s and ends at the goal
    states = [[0, 0, 0, 0.5, 0, 0.5],
              [scene.beam_x_center, 0, 0, 0.5, 0, 0.20],   # in beam, s lowered
              [scene.goal_x, 0, 0, 0.0, 0, 0.20]]
    f = [0.4, 0.02, 0.4]
    v = evaluate_gates(_record(states, f), scene, GateConfig())
    assert v["reach"] and v["feasible"] and v["s_lowering"]
    assert v["verdict"] == "PASS"


def test_controller_fail_when_not_feasible():
    scene = SceneConfig()
    states = [[0, 0, 0, 0.5, 0, 0.5],
              [scene.beam_x_center, 0, 0, 0.5, 0, 0.50],   # never lowered
              [scene.goal_x, 0, 0, 0.0, 0, 0.50]]
    f = [0.4, -0.30, 0.4]                                  # smashes the beam
    v = evaluate_gates(_record(states, f), scene, GateConfig())
    assert v["feasible"] is False and v["s_lowering"] is False
    assert v["verdict"] == "FAIL"


def test_ablation_pass_when_fixed_s_violates():
    scene = SceneConfig(fixed_s=True)
    states = [[0, 0, 0, 0.5, 0, 0.5],
              [scene.beam_x_center, 0, 0, 0.5, 0, 0.50],
              [scene.goal_x, 0, 0, 0.0, 0, 0.50]]
    f = [0.4, -0.30, 0.4]
    g = GateConfig(scene_kind="ablation")
    v = evaluate_gates(_record(states, f), scene, g)
    assert v["hypothesis_confirmed"] is True            # fixed-s fails as predicted
    assert v["verdict"] == "PASS"


def test_solve_ok_is_reported_not_gating():
    scene = SceneConfig()
    states = [[0, 0, 0, 0.5, 0, 0.5],
              [scene.beam_x_center, 0, 0, 0.5, 0, 0.20],
              [scene.goal_x, 0, 0, 0.0, 0, 0.20]]
    f = [0.4, 0.02, 0.4]
    slow = np.full(3, 0.5)                                # over the dev bound
    v = evaluate_gates(_record(states, f, solve_times=slow), scene, GateConfig())
    assert v["solve_ok"] is False
    assert v["verdict"] == "PASS"                         # descriptive, not gating
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_gates.py -v`
Expected: FAIL with `ImportError` (`evaluate_gates` not defined).

- [ ] **Step 3: Write the implementation**

```python
# hilda_nmpc/hilda_nmpc/gates.py
"""Falsifiable gate evaluation, shared by the pytest smoke-guard and the runner.

controller scenes (03a/03c): PASS = reach AND feasible AND s_lowering.
ablation scenes (03b): PASS = the fixed-s controller VIOLATES feasibility
(hypothesis = joint-opt is needed). Solve-time is reported (solve_ok) but is
dev-machine descriptive and does NOT gate the verdict (spec flag 6).
Feasibility is soft: f_min >= -f_tol (NOT >= 0, which would be the hard arm).
"""
from __future__ import annotations

import numpy as np

from .config import SceneConfig, GateConfig


def evaluate_gates(record: dict, scene: SceneConfig, gate_cfg: GateConfig) -> dict:
    states = np.asarray(record["states"], dtype=float)
    final = states[-1]
    reach_err = float(np.hypot(final[0] - scene.goal_x, final[1] - scene.goal_y))
    reach = reach_err <= gate_cfg.reach_tol
    if gate_cfg.require_terminal_s:
        reach = reach and (abs(final[5] - scene.s_goal) <= gate_cfg.s_tol)

    f_arr = np.asarray(record["f"], dtype=float)
    f_min = float(f_arr.min()) if f_arr.size else float("nan")
    feasible = f_min >= -gate_cfg.f_tol

    slack = np.asarray(record["slack"], dtype=float)
    max_slack = float(slack.max()) if slack.size else 0.0

    beam_lo = scene.beam_x_center - scene.beam_x_halfwidth
    beam_hi = scene.beam_x_center + scene.beam_x_halfwidth
    in_beam = (states[:, 0] >= beam_lo) & (states[:, 0] <= beam_hi)
    if in_beam.any():
        s_in_beam_min = float(states[in_beam, 5].min())
        s_lowering = s_in_beam_min < (scene.s_nominal - gate_cfg.s_drop_min)
    else:
        s_in_beam_min = None
        s_lowering = False

    st = np.asarray(record["solve_times"], dtype=float)
    solve_p50 = float(np.percentile(st, 50)) if st.size else float("nan")
    solve_p95 = float(np.percentile(st, 95)) if st.size else float("nan")
    solve_max = float(st.max()) if st.size else float("nan")
    solve_ok = bool(solve_p95 <= gate_cfg.solve_time_p95_max) if st.size else False

    if gate_cfg.scene_kind == "ablation":
        hypothesis_confirmed = bool(not feasible)
    else:
        hypothesis_confirmed = bool(reach and feasible and s_lowering)
    verdict = "PASS" if hypothesis_confirmed else "FAIL"

    return {
        "scene_kind": gate_cfg.scene_kind,
        "reach_err": reach_err, "reach": bool(reach),
        "f_min": f_min, "feasible": bool(feasible),
        "max_slack": max_slack,
        "s_in_beam_min": s_in_beam_min, "s_lowering": bool(s_lowering),
        "solve_p50": solve_p50, "solve_p95": solve_p95, "solve_max": solve_max,
        "solve_ok": solve_ok,
        "hypothesis_confirmed": hypothesis_confirmed,
        "verdict": verdict,
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_gates.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add hilda_nmpc/gates.py test/test_gates.py
git commit -m "gates: falsifiable verdict (controller vs ablation), soft f-tol [G3]"
```

---

## Task 9: Fixed-`s` ablation smoke-guard (checkpoint 4)

**Files:**
- Modify: `hilda_nmpc/test/test_closed_loop.py`

- [ ] **Step 1: Write the failing test**

```python
# hilda_nmpc/test/test_closed_loop.py  (append)
from hilda_nmpc.gates import evaluate_gates
from hilda_nmpc.config import GateConfig


def test_fixed_s_ablation_violates_under_beam(tmp_path, monkeypatch):
    """Checkpoint 4: with s frozen at s_nominal the controller cannot clear the
    beam -> feasibility is violated. Same closed_loop + gates as the 03b runner."""
    scene = SceneConfig(fixed_s=True)
    cfg = OcpConfig()
    field = build_beam_field(scene)
    monkeypatch.chdir(tmp_path)
    solver = build_ocp(field, cfg, scene, str(tmp_path / "ocp_fixed.json"))
    rec = run_closed_loop(solver, field, scene, cfg)
    v = evaluate_gates(rec, scene, GateConfig(scene_kind="ablation"))
    assert v["feasible"] is False               # fixed-s smashes the beam
    assert v["hypothesis_confirmed"] is True     # the joint-opt hypothesis holds
```

- [ ] **Step 2: Run test to verify it fails or passes**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_closed_loop.py::test_fixed_s_ablation_violates_under_beam -v`
Expected: PASS (all production code already exists). If the fixed-`s` solver returns a non-zero status under the beam (expected — the hard box keeps `s` high while soft `f` is violated), that is fine: the realised `f` from the plant is what the gate reads, and it is negative under the beam.

- [ ] **Step 3: (no implementation needed)**

`run_closed_loop` + `evaluate_gates` + the `scene.fixed_s` branch in `build_ocp` already cover this. This task adds the integration assertion only.

- [ ] **Step 4: Run the full closed-loop file**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_closed_loop.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add test/test_closed_loop.py
git commit -m "closed_loop: fixed-s ablation smoke-guard (joint-opt hypothesis) [G3]"
```

---

## Task 10: `df/ds` wiring sanity (checkpoint 6)

**Files:**
- Modify: `hilda_nmpc/test/test_ocp.py`

- [ ] **Step 1: Write the failing test**

```python
# hilda_nmpc/test/test_ocp.py  (append)
def test_field_s_channel_wired_into_constraint():
    """Checkpoint 6: the model's s-channel reaches cs.f -> perturbing s moves f
    by -ds (consistency with cs.grad_f's exact -1 on the s axis)."""
    scene = SceneConfig()
    field = build_beam_field(scene)
    x0, y0 = scene.start_x, scene.start_y
    f_lo = float(field.f([x0, y0, 0.2]))
    f_hi = float(field.f([x0, y0, 0.3]))
    assert abs((f_hi - f_lo) - (-0.1)) < 1e-6          # df/ds == -1 exactly
    g = np.asarray(field.grad_f([x0, y0, 0.25])).ravel()
    assert abs(g[2] - (-1.0)) < 1e-9
```

- [ ] **Step 2: Run test to verify it passes**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws/src/hilda_nmpc && python3 -m pytest test/test_ocp.py::test_field_s_channel_wired_into_constraint -v`
Expected: PASS (this asserts the landed `cs.f`/`cs.grad_f` contract the OCP relies on).

- [ ] **Step 3: (no implementation needed)** — asserts the consumed interface.

- [ ] **Step 4: Run the whole hilda_nmpc suite**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws && colcon build --symlink-install --packages-select hilda_clearance_field hilda_nmpc && source install/setup.bash && cd src/hilda_nmpc && python3 -m pytest test/ -v`
Expected: PASS (config 5, scenarios 3, model 2, plant 3, ocp 6, closed_loop 2, gates 4).

- [ ] **Step 5: Commit**

```bash
git add test/test_ocp.py
git commit -m "ocp: df/ds wiring sanity vs cs.grad_f [G3]"
```

---

## Task 11: Experiment configs + READMEs (thesis repo)

**Files:**
- Create: `thesis/experiments/configs/sim_validation_03a/transit.yaml`, `README.md`
- Create: `thesis/experiments/configs/sim_validation_03b/fixed_s_ablation.yaml`, `README.md`
- Create: `thesis/experiments/configs/sim_validation_03c/drill_target.yaml`, `README.md`

- [ ] **Step 1: Write `03a/transit.yaml`**

```yaml
# sim_validation_03a — transit: the sledge lowers to clear a too-low beam.
# Isolates the G3 path-constraint -> configuration coupling. Terminal s-task
# inactive (q_s_e=0); the robot starts at s_nominal and r_us inertia keeps it
# there until the beam forces a dip. PASS = reach AND feasible AND s_lowering.
ocp:
  N: 40
  tf: 4.0
  constraint_mode: soft
  barrier: off
  zl: 1.0e3
  Zl: 1.0e1
  u_s_max: 0.30        # PLACEHOLDER conservative (folds q1/q2)
  q_s_e: 0.0           # terminal s-task inactive in transit
scene:
  name: transit
  beam_x_center: 2.0
  beam_x_halfwidth: 0.30
  beam_z_ceil: 2.40
  s_nominal: 0.50
  goal_x: 4.0
  s_goal: 0.0
  fixed_s: false
  sim_time: 16.0
gates:
  scene_kind: controller
  reach_tol: 0.15
  f_tol: 0.05
  s_drop_min: 0.05
  solve_time_p95_max: 0.05
```

- [ ] **Step 2: Write `03b/fixed_s_ablation.yaml`**

```yaml
# sim_validation_03b — fixed-s ablation: the joint-optimisation hypothesis.
# Same scene as 03a, but s is frozen at s_nominal (fixed_s: true). The fixed-s
# controller cannot clear the beam -> realised f is violated. This experiment
# PASSES its hypothesis when fixed-s FAILS feasibility (lit-review research-plan
# Phase 3: "profiled against a fixed-s baseline"). The citable contrast number.
ocp:
  N: 40
  tf: 4.0
  constraint_mode: soft
  barrier: off
  zl: 1.0e3
  Zl: 1.0e1
  u_s_max: 0.30
  q_s_e: 0.0
scene:
  name: fixed_s_ablation
  beam_x_center: 2.0
  beam_x_halfwidth: 0.30
  beam_z_ceil: 2.40
  s_nominal: 0.50
  goal_x: 4.0
  s_goal: 0.0
  fixed_s: true        # the ablation
  sim_time: 16.0
gates:
  scene_kind: ablation
  reach_tol: 0.15
  f_tol: 0.05
  s_drop_min: 0.05
  solve_time_p95_max: 0.05
```

- [ ] **Step 3: Write `03c/drill_target.yaml`**

```yaml
# sim_validation_03c — drill-target: exercises the terminal-task cost.
# Terminal s_goal != 0 (a drilling extension) pulls s back up after the beam,
# so the robot dips under the beam then re-extends to s_goal at the goal pose.
# PASS = reach (incl. terminal s) AND feasible AND s_lowering.
ocp:
  N: 40
  tf: 4.0
  constraint_mode: soft
  barrier: off
  zl: 1.0e3
  Zl: 1.0e1
  u_s_max: 0.30
  q_s_e: 20.0          # terminal s-task ACTIVE
scene:
  name: drill_target
  beam_x_center: 2.0
  beam_x_halfwidth: 0.30
  beam_z_ceil: 2.40
  s_nominal: 0.50
  goal_x: 4.0
  s_goal: 0.50         # re-extend to a drilling height past the beam
  fixed_s: false
  sim_time: 16.0
gates:
  scene_kind: controller
  reach_tol: 0.15
  f_tol: 0.05
  s_drop_min: 0.05
  solve_time_p95_max: 0.05
  require_terminal_s: true
  s_tol: 0.10
```

- [ ] **Step 4: Write the three READMEs**

`thesis/experiments/configs/sim_validation_03a/README.md`:

```markdown
# sim_validation_03a — transit (G3 lowering behaviour)

**Hypothesis.** A perceptive 6-state controller drives the sledge `s` down to
clear a too-low beam it cannot pass at the nominal extension, then reaches a
goal beyond it. **PASS** = reach AND soft-feasibility (`min f >= -f_tol`) AND
`s` dips below `s_nominal` in the beam region. Falsifiable: a beam the robot
clears anyway, or a controller that ignores the constraint, FAILS.

**Why these numbers.** `beam_z_ceil=2.40` with `h_stand=0.10`, `z_tip0=1.899`,
`eps=0.10` gives `f(s_nominal=0.50) = 2.40-0.10-(0.10+1.899+0.50) = -0.199 < 0`
(must lower) and `f(s=0) = +0.301 > 0` (lowering clears). The beam at `x=2.0`
with `v_approach=0.40` leaves ~5 s to retract 0.2 m at `u_s_max=0.30` (~0.67 s):
comfortably feasible, not marginal.

**Run.**
`source ~/ros2_ws/.venv-acados/acados_env.sh && source ~/ros2_ws/install/setup.bash`
`cd ~/ros2_ws/src/thesis && python3 experiments/runners/sim_validation_03_runner.py experiments/configs/sim_validation_03a/transit.yaml`
```

`thesis/experiments/configs/sim_validation_03b/README.md`:

```markdown
# sim_validation_03b — fixed-`s` ablation (joint-optimisation hypothesis)

The lit-review research plan (Phase 3) validates the contribution by profiling
against a **fixed-`s` baseline**. Same scene as 03a with `s` frozen at
`s_nominal`. **The experiment PASSES its hypothesis when fixed-`s` FAILS
feasibility** — i.e. the realised `f` is violated under the beam because the
robot cannot retract. The recorded `f_min` (violation magnitude) is the citable
contrast against 03a's "joint-opt reaches the goal at min clearance Y".

**Run.** as 03a with `experiments/configs/sim_validation_03b/fixed_s_ablation.yaml`.
```

`thesis/experiments/configs/sim_validation_03c/README.md`:

```markdown
# sim_validation_03c — drill-target (terminal-task cost)

Adds an active terminal task `s = s_goal != 0` (a drilling extension) at a goal
pose beyond the beam. The robot dips under the beam then re-extends to `s_goal`
— exercising the terminal-task cost and the dip-and-recover behaviour, closer to
the full mission. Kept separate from 03a so a failure localises to one mechanism
(path constraint vs terminal task). **PASS** = reach (incl. `|s_final-s_goal| <=
s_tol`) AND feasible AND `s_lowering`.

**Run.** as 03a with `experiments/configs/sim_validation_03c/drill_target.yaml`.
```

- [ ] **Step 5: Commit (thesis repo)**

```bash
cd ~/ros2_ws/src/thesis
git add experiments/configs/sim_validation_03a experiments/configs/sim_validation_03b experiments/configs/sim_validation_03c
git commit -m "03: sim_validation_03 experiment configs (transit / fixed-s / drill) [G3]"
```

---

## Task 12: Experiment runner (thesis repo)

**Files:**
- Create: `thesis/experiments/runners/sim_validation_03_runner.py`
- Test: `hilda_nmpc/test/test_runner_smoke.py` (a thin importable-logic smoke test that does not require the runner file path)

The runner consumes a config path and writes `results/<area>/<stem>__<cfg_sha>_<git_sha>/` with `manifest.json` (status), `record.npz`, `verdict.json`, and `solve_time_hist.csv` — following `experiments/runners/README.md`.

- [ ] **Step 1: Write the runner**

```python
#!/usr/bin/env python3
"""sim_validation_03 runner — standalone acados OCP prototype (module 03).

Consumes one experiment YAML (ocp / scene / gates blocks), builds the synthetic
beam field + the OCP, runs the closed loop, evaluates the falsifiable gates, and
writes a results dir per experiments/runners/README.md. Pure Python (no ROS
nodes, no bag). Solve-time is dev-machine descriptive (NOT the Fig-5 Orin proof).

Usage (needs the venv + workspace overlay so hilda_nmpc + hilda_clearance_field import):
  source ~/ros2_ws/.venv-acados/acados_env.sh
  source ~/ros2_ws/install/setup.bash
  cd ~/ros2_ws/src/thesis
  python3 experiments/runners/sim_validation_03_runner.py \
      experiments/configs/sim_validation_03a/transit.yaml
"""
import argparse
import hashlib
import json
import pathlib
import subprocess
import time

import numpy as np

from hilda_nmpc.config import load_experiment
from hilda_nmpc.scenarios import build_beam_field, retraction_feasibility
from hilda_nmpc.ocp import build_ocp
from hilda_nmpc.closed_loop import run_closed_loop
from hilda_nmpc.gates import evaluate_gates


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("config", type=pathlib.Path)
    args = ap.parse_args()

    repo_root = pathlib.Path(__file__).resolve().parents[2]   # thesis repo root
    cfg_sha = hashlib.sha256(args.config.read_bytes()).hexdigest()[:12]
    git_sha = subprocess.check_output(
        ["git", "-C", str(repo_root), "rev-parse", "--short", "HEAD"]).decode().strip()
    dirty = bool(subprocess.check_output(
        ["git", "-C", str(repo_root), "status", "--porcelain"]).strip())
    out = (repo_root / "experiments" / "results" / args.config.parent.name
           / f"{args.config.stem}__{cfg_sha}_{git_sha}")
    out.mkdir(parents=True, exist_ok=True)

    manifest = {
        "config": str(args.config), "config_sha": cfg_sha, "git_sha": git_sha,
        "dirty": dirty, "start": time.time(), "status": "running",
        "host": subprocess.check_output(["hostname"]).decode().strip(),
        "runner_version": "1",
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))

    try:
        ocp_cfg, scene, gate_cfg = load_experiment(args.config)
        feas = retraction_feasibility(scene, ocp_cfg.u_s_max)
        if gate_cfg.scene_kind == "controller" and not feas["feasible"]:
            raise RuntimeError(
                f"scene not comfortably feasible (must_lower={feas['must_lower']}, "
                f"t_beam={feas['time_to_beam_s']:.2f}s vs t_retract="
                f"{feas['retraction_time_s']:.2f}s) — the beam tests the wrong thing")

        field = build_beam_field(scene)
        solver = build_ocp(field, ocp_cfg, scene, str(out / "ocp.json"))
        rec = run_closed_loop(solver, field, scene, ocp_cfg)
        verdict = evaluate_gates(rec, scene, gate_cfg)

        np.savez(out / "record.npz", **rec)
        (out / "verdict.json").write_text(json.dumps({**verdict, "feasibility_check": feas}, indent=2))
        st = np.asarray(rec["solve_times"])
        hist_lines = ["solve_time_s"] + [f"{t:.6f}" for t in st]
        (out / "solve_time_hist.csv").write_text("\n".join(hist_lines) + "\n")

        manifest["end"] = time.time()
        manifest["status"] = "ok"
        manifest["verdict"] = verdict["verdict"]
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
        print(f"{verdict['verdict']}  ->  {out}")
        print(json.dumps(verdict, indent=2))
        return 0
    except Exception as exc:                                   # fail-loud, keep audit trail
        manifest["end"] = time.time()
        manifest["status"] = "failed"
        manifest["error"] = repr(exc)
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
        raise


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Write a smoke test for the runner's pure logic**

```python
# hilda_nmpc/test/test_runner_smoke.py
"""The runner is thesis-repo code; here we smoke-test the same pipeline it calls
end-to-end through the public hilda_nmpc API, asserting the 03a controller PASS."""
import pytest

pytest.importorskip("casadi")
pytest.importorskip("acados_template")

from hilda_nmpc.config import OcpConfig, SceneConfig, GateConfig
from hilda_nmpc.scenarios import build_beam_field
from hilda_nmpc.ocp import build_ocp
from hilda_nmpc.closed_loop import run_closed_loop
from hilda_nmpc.gates import evaluate_gates


def test_full_pipeline_03a_passes(tmp_path, monkeypatch):
    scene = SceneConfig()
    cfg = OcpConfig()
    field = build_beam_field(scene)
    monkeypatch.chdir(tmp_path)
    solver = build_ocp(field, cfg, scene, str(tmp_path / "ocp.json"))
    rec = run_closed_loop(solver, field, scene, cfg)
    verdict = evaluate_gates(rec, scene, GateConfig(scene_kind="controller"))
    assert verdict["verdict"] == "PASS"
```

- [ ] **Step 3: Run the smoke test**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws && colcon build --symlink-install --packages-select hilda_clearance_field hilda_nmpc && source install/setup.bash && cd src/hilda_nmpc && python3 -m pytest test/test_runner_smoke.py -v`
Expected: PASS.

- [ ] **Step 4: Run all three experiments end-to-end (the real artefact)**

Run:
```bash
source ~/ros2_ws/.venv-acados/acados_env.sh && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/thesis
python3 experiments/runners/sim_validation_03_runner.py experiments/configs/sim_validation_03a/transit.yaml
python3 experiments/runners/sim_validation_03_runner.py experiments/configs/sim_validation_03b/fixed_s_ablation.yaml
python3 experiments/runners/sim_validation_03_runner.py experiments/configs/sim_validation_03c/drill_target.yaml
```
Expected: `03a -> PASS`, `03b -> PASS` (ablation hypothesis confirmed: fixed-`s` infeasible), `03c -> PASS`. Each writes a results dir with `manifest.json` (`"status":"ok"`), `verdict.json`, `record.npz`, `solve_time_hist.csv`.

- [ ] **Step 5: Commit (both repos)**

```bash
cd ~/ros2_ws/src/hilda_nmpc
git add test/test_runner_smoke.py
git commit -m "test: full-pipeline smoke (03a PASS) [G3]"
cd ~/ros2_ws/src/thesis
git add experiments/runners/sim_validation_03_runner.py
git commit -m "03: sim_validation_03 runner (manifest + verdict + solve-time hist) [G3]"
```

---

## Task 13: Docs sync — doc 03, ADR 0014, journal (thesis repo)

**Files:**
- Create: `thesis/docs/decisions/0014-ocp-constraint-config-axes.md`
- Modify: `thesis/docs/03_nmpc_formulation.md`
- Modify: `thesis/journal/2026-W22.md`

- [ ] **Step 1: Write ADR 0014**

```markdown
# 0014 — OCP constraint config: two orthogonal axes + barrier cost-type switch

Status: accepted
Date: 2026-05-31
Gap(s): G3
Module: 03_nmpc_formulation.md

## Context

The module-03 OCP prototype must consume `f` as a soft constraint now while
keeping the soft-vs-hard decision and the eventual Phase-4 method comparison
reachable without a refactor. Two non-obvious reasoning chains future code would
otherwise re-derive are pinned here. Design: the
[spec](../superpowers/specs/2026-05-31-acados-ocp-prototype-design.md).

## Choice

1. **`f` enters via two orthogonal axes, not one barrier toggle.** Axis 1 —
   constraint: `soft` (`con_h_expr` + `idxsh`, L1 slack) / `hard` (`con_h_expr`,
   no `idxsh`) / `none` (no `con_h_expr` row). Axis 2 — barrier in the cost:
   `off` / `on`. The named methods are corners: prototype baseline = soft+off
   (= proposed-minus-barrier); RQ3 hard arm = hard+off; doc 03 proposed /
   interaction study = soft+on; Baseline C / Grandia-pure = none+on. "Barrier
   on/off" alone does NOT reach Baseline C — it also drops the slacked row.
   `none`+`off` is degenerate (f never enters) and is rejected at config-validate.
2. **Barrier-on is a cost-type switch, not an added term.** acados sets one cost
   type per stage, so a barrier cannot be added onto a `NONLINEAR_LS` stage.
   Off: `NONLINEAR_LS`, goal via `yref`, Gauss-Newton (the spline-gate-proven
   combo). On: `CONVEX_OVER_NONLINEAR` with `psi(r) = 1/2 ||r_track||^2_W + B(r_f)`,
   which is exactly Grandia Eq. 32's generalised Gauss-Newton (so Eq. 32 attaches
   to the barrier term, not the slacked baseline); goal moves to a parameter.
   `EXTERNAL` + hand-supplied Gauss-Newton Hessian is the fallback.

## Consequences

The prototype builds soft/hard + barrier-off; `barrier='on'` and
`constraint_mode='none'` raise `NotImplementedError` (reserved), so Baseline C /
proposed / interaction runs are later YAMLs flipping a reserved cost path with no
re-architecting. The slacked baseline's C1 requirement is met through
`cs.grad_f` (the constraint Jacobian), independent of the barrier. Revisit when
the barrier path is built for the Phase-4 comparison.
```

- [ ] **Step 2: Update doc 03 §Cross-references**

In `thesis/docs/03_nmpc_formulation.md`, append to the §Cross-references list (after the `hilda_clearance_field/clearance_spline.py` bullet):

```markdown
- `hilda_nmpc/` — standalone acados OCP prototype (6-state SQP-RTI, soft `f≥0` via `cs.f`) landed 2026-05-31; closes the "horizon/discretisation" open question *with a default* (N=40, tf=4.0 s), keeps NMPC/HMPC, 6/7-state, soft/hard, IRM all open. Constraint config is two axes (mode × barrier); barrier-on / Baseline C reserved. Design: [spec](superpowers/specs/2026-05-31-acados-ocp-prototype-design.md), [ADR 0014](decisions/0014-ocp-constraint-config-axes.md). Validated by `sim_validation_03{a,b,c}` (transit / fixed-`s` ablation / drill-target).
```

Also, in §Open questions under "Horizon and discretisation", append: `Defaulted (not committed) in the 03 prototype: N=40, tf=4.0 s (dt=0.1 s); tunable per config.`

- [ ] **Step 3: Add the journal entry**

Append to `thesis/journal/2026-W22.md`:

```markdown
## 2026-05-31 (standalone acados OCP prototype — module 03 landed)

The 6-state (x,y,θ,v,ω,s) SQP-RTI controller that queries `cs.f` as a soft
clearance constraint. Built theory-doc-before-code: brainstorm → spec → ADR 0014
→ TDD (single-solve status 0 → closed loop → harness).

**Decisions.** Scope = config-driven closed-loop eval with falsifiable gates
(the 01b/01c discipline applied to module 03). Encoding = L1 slack first, relaxed
log-barrier reserved. Constraint config = two orthogonal axes (mode × barrier),
ADR 0014: soft+off baseline = proposed-minus-barrier; RQ3 hard arm = hard+off;
proposed/interaction = soft+on; Baseline C = none+on (drops the slacked row, not
just barrier-off). Barrier-on is a cost-type switch (NONLINEAR_LS →
CONVEX_OVER_NONLINEAR, goal yref → param), reserved here.

**Scenes.** 03a transit (sledge lowers to clear a too-low beam — the G3 coupling,
terminal s-task inactive), 03b fixed-`s` ablation (s frozen → infeasible under the
beam: the joint-optimisation hypothesis, lit-review Phase 3), 03c drill-target
(terminal s_goal ≠ 0, dip-and-recover). Beam comfortably feasible by construction
(retraction time ≪ time-to-beam), asserted in the runner.

**Gates.** Soft feasibility = `min f ≥ −f_tol` (not ≥ 0, which is the hard arm);
`max_slack` recorded. s-lowering required in the beam region. Solve-time recorded
but dev-machine descriptive — seeds RQ4, NOT the Fig-5 Orin proof. 03b passes its
hypothesis when fixed-`s` fails.

**Placeholders flagged.** η_v=η_w=1.0 (≡ Level 0) pending ICR calibration;
`u_s_max` conservative (folds q1/q2 split). Static-stability primitive-4
(|a|≤a_max(s)) deferred-for-scene with doc 03 §4 option-B as the later toggle.

**Open / parked.** Four architecture decisions kept open by construction. Parked,
surfaced not touched: h₀ §Notation-vs-§3 (prototype consumes whatever h_stand the
spline was built with), Enrico 2025 (not relied on — solve-time descriptive),
/constraint_field 5-vs-8 Hz (static field here, orthogonal).

**Changed.** `hilda_nmpc/` (repo, branch `feature/ocp-prototype`): config/scenarios/
model/plant/ocp/closed_loop/gates + tests. `thesis/docs/decisions/0014-*.md` (new),
`docs/03_nmpc_formulation.md` (impl-status sync), `experiments/configs/
sim_validation_03{a,b,c}/` + `experiments/runners/sim_validation_03_runner.py`.

Next: build the barrier-on path (CONVEX_OVER_NONLINEAR) for the Phase-4 method
comparison; wire the GridMap adapter when integration begins.
```

- [ ] **Step 4: Verify the docs render and links resolve**

Run: `cd ~/ros2_ws/src/thesis && ls docs/decisions/0014-ocp-constraint-config-axes.md && grep -n "sim_validation_03" docs/03_nmpc_formulation.md && grep -n "2026-05-31" journal/2026-W22.md`
Expected: the ADR exists; doc 03 references the prototype; the journal entry is present.

- [ ] **Step 5: Commit (thesis repo)**

```bash
cd ~/ros2_ws/src/thesis
git add docs/decisions/0014-ocp-constraint-config-axes.md docs/03_nmpc_formulation.md journal/2026-W22.md
git commit -m "03 + ADR 0014 + journal: standalone acados OCP prototype landed [G3]"
```

---

## Final verification

- [ ] **Full hilda_nmpc suite green:**

Run: `source ~/ros2_ws/.venv-acados/acados_env.sh && cd ~/ros2_ws && colcon build --symlink-install --packages-select hilda_clearance_field hilda_nmpc && source install/setup.bash && cd src/hilda_nmpc && python3 -m pytest test/ -v`
Expected: PASS — config 5, scenarios 3, model 2, plant 3, ocp 6, closed_loop 2, gates 4, runner_smoke 1.

- [ ] **Three experiments produce PASS verdicts** (Task 12 Step 4) with `manifest.json "status":"ok"` and a `verdict.json` in each results dir.

- [ ] **Merge decision:** the `feature/ocp-prototype` branch passes its isolation test (the suite + the three experiment verdicts) — ready to merge per THESIS.md branch-per-module discipline. (Leave the merge to Oscar, as with the clearance-spline branch.)
```
