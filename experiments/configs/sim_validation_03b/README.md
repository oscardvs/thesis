# sim_validation_03b — fixed-`s` ablation (joint-optimisation hypothesis)

The lit-review research plan (Phase 3) validates the contribution by profiling
against a **fixed-`s` baseline**. Same scene as 03a with `s` frozen at
`s_nominal`. With the soft controller's high slack penalty the fixed-`s`
controller does **not** smash through the beam — it **stalls at the beam edge**
(final `x ≈ 1.76`, ~2.24 m short of the goal), unable to proceed without
violating clearance, and never reaches the goal.

**The experiment PASSES its hypothesis when fixed-`s` cannot reach while
feasible** (gate `scene_kind: ablation` → `hypothesis_confirmed =
not(reach and feasible)`). This is robust to both failure modes (stall or
violate). The contrast vs 03a is the citable result: **joint-opt reaches the
goal by lowering `s` to ~0.10 (min clearance +0.138 m); fixed-`s` stalls 2.24 m
short** — the configuration variable `s` is what makes the goal reachable.

**Run.** as 03a with `experiments/configs/sim_validation_03b/fixed_s_ablation.yaml`.
