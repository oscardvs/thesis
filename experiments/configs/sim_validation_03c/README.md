# sim_validation_03c — drill-target (terminal-task cost)

Adds an active terminal task `s = s_goal != 0` (a drilling extension) at a goal
pose beyond the beam. The robot dips under the beam then re-extends to `s_goal`
— exercising the terminal-task cost and the dip-and-recover behaviour, closer to
the full mission. Kept separate from 03a so a failure localises to one mechanism
(path constraint vs terminal task). **PASS** = reach (incl. `|s_final-s_goal| <=
s_tol`) AND feasible AND `s_lowering`.

**Run.** as 03a with `experiments/configs/sim_validation_03c/drill_target.yaml`.
