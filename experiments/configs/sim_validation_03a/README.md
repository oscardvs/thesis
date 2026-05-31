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
comfortably feasible, not marginal. Observed: joint-opt reaches the goal
(dist ~0.08 m) lowering `s` to ~0.10, min clearance +0.138 m.

**Run.**
`source ~/ros2_ws/.venv-acados/acados_env.sh && source ~/ros2_ws/install/setup.bash`
`cd ~/ros2_ws/src/thesis && python3 experiments/runners/sim_validation_03_runner.py experiments/configs/sim_validation_03a/transit.yaml`
