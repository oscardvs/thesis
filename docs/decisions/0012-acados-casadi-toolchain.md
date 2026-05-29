# 0012 — acados-casadi-toolchain

Status: accepted
Date: 2026-05-29
Gap(s): G3 (enabling)
Module: 03_nmpc_formulation.md

## Context

G3 needs two Python packages neither of which was importable in the workspace interpreter: **casadi** (the C¹ B-spline `f` interface of 02/03 and acados's modelling layer) and **acados** (the SQP-RTI OCP solver of 03). Two constraints shape the install:

- Ubuntu 24.04 marks the system Python as externally-managed (PEP 668); `pip install` into it is refused without `--break-system-packages`.
- CLAUDE.md: numpy is **apt-managed** (`/usr/lib/python3/dist-packages`, 1.26.4) and must not be touched — a pip-installed numpy breaks CuPy and ros2_numpy. acados_template depends on numpy/scipy, so a careless install could drag a shadowing numpy in.

## Options

- **A — `pip install --user --break-system-packages`.** Simplest; packages land in `~/.local`, visible to the system interpreter (and thus to ROS-launched nodes). Risk: `acados_template`'s numpy/scipy deps can pull a newer numpy into `~/.local` that shadows the apt numpy for *all* system Python — the exact CLAUDE.md failure mode.
- **B — venv with `--system-site-packages`.** A dedicated venv that inherits the apt numpy + CuPy + rclpy read-only; casadi/acados install inside the venv only, never overwriting system packages. Cost: the venv must be sourced for G3 work, and a future ROS-node integration must decide packaging then.
- **C — conda/mamba environment.** Fully isolated, but a parallel package manager over a ROS 2 + CuPy stack that is otherwise entirely apt/pip; heaviest and most divergent from the rest of the workspace.

## Choice

**B.** Venv at `~/ros2_ws/.venv-acados` (`--system-site-packages`). casadi `3.7.2` via pip; acados built from source in `~/ros2_ws/tools/acados` (tag `v0.5.4`, commit `dc6668f`), with its `acados_template` Python interface `pip install -e`'d into the venv. `t_renderer` `v0.2.0` pre-placed in `tools/acados/bin/`. Environment via `~/ros2_ws/.venv-acados/acados_env.sh` (sets `ACADOS_SOURCE_DIR` + `LD_LIBRARY_PATH`, activates the venv).

## Rationale

The binding constraint is "do not disturb the apt numpy." Option B satisfies it structurally: with `--system-site-packages` the venv *sees* numpy 1.26.4 / CuPy / rclpy but `pip` inside the venv installs only into the venv's own `site-packages`; the system tree is never written. Verified after install — `numpy.__file__` resolves to `/usr/lib/python3/dist-packages` from both the venv and the system interpreter, and CuPy imports in the venv. Option A's `~/.local` is on the system import path, so any numpy it pulls would shadow apt numpy globally — the failure CLAUDE.md names. Option C adds a second package manager for two packages; not worth the divergence.

Choosing B also fits isolation-before-integration: 03's first artefact is a standalone acados OCP prototype, not yet a ROS node, so a sourced venv is the natural home. When the controller later becomes a Nav2 plugin running in a ROS process, packaging is revisited (see Flip condition).

Verified functional, not merely importable: a throwaway double-integrator OCP went casadi → acados codegen → `t_renderer` → C compile → HPIPM solve and returned `status 0` with the expected control. The first run also exposed that `acados_template.utils.get_tera()` calls `input()` to offer an interactive `t_renderer` download — which hangs any non-interactive/background invocation; pre-placing the binary avoids the prompt entirely (`utils.py:278` returns early).

## Consequences

- **G3 work sources the toolchain:** `source ~/ros2_ws/.venv-acados/acados_env.sh` before running anything that imports casadi/acados. ROS nodes launched without it will not see the packages — fine for the standalone prototype; flagged for integration.
- **Local build artefacts, not committed.** `~/ros2_ws/.venv-acados/` and `~/ros2_ws/tools/acados/` sit alongside `training_runs/` and `log/` — gitignored-by-convention build state. This ADR + the pinned versions (casadi 3.7.2, acados `v0.5.4`/`dc6668f`, t_renderer 0.2.0) are the committed, reproducible spec.
- **numpy/CuPy safe.** Confirmed unchanged post-install; no pip touched the apt tree.
- **Flip condition.** When the controller is wrapped as a Nav2 controller plugin (a ROS-process import, not a standalone script), the venv-only install no longer suffices: either the launching process activates the venv, or acados/casadi move to a workspace-visible install. Decide at the integration step, not now — the OCP prototype does not need it.
