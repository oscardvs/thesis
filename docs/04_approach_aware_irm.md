# 04 — Approach-aware inverse reachability

## Scope

Stage 04 selects base poses for ceiling-drilling targets and verifies that the platform can reach each target without violating clearance along the way. For each drill target derived from a BIM model, candidate base stances are offset along the drill axis and filtered by two checks against the clearance field from 02: a terminal-pose check `c − h_base − s_goal − ε ≥ δ_margin` at the candidate, and an approach-corridor check that forward-simulates the controller's intended retract-and-extend profile along a short corridor leading to that pose. Multi-target sequencing falls out as a greedy nearest-neighbour search whose inter-target cost integrates reconfiguration against the same clearance field, addressing the inter-target perception-coupling absence identified in the IRM literature. This stage closes G4.

## Sources

- Literature Study, Section 8 (positioning and goal selection, including §8.7 on upward-facing IRMs) and §11.4 (the proposed approach-aware filter).
- The IRM lineage is reviewed in §8.1–§8.5; the BIM-driven construction-robotics context is in §8.6.

## Architectural commitments inherited

- Two filters per candidate base pose: terminal feasibility and approach-corridor feasibility (forward-simulated).
- Multi-target sequencing through greedy nearest-neighbour with reconfiguration cost integrated against the clearance field.
- Owns its own package `hilda_irm`, separate from `hilda_navigation` and `hilda_clearance_field`.
- The mast topology (a single prismatic DOF supplying the upward reachable set) makes the reachable set above a candidate base a vertical line segment, not a 6D volume.

## Open questions

- **Representation.** The lit review leaves the IRM representation open: an explicit 6D voxel grid (Vahrenkamp-style, RM4D, Reuleaux), an analytical mast-aware filter exploiting the line-segment topology, or a Cavelli-style ellipsoidal approximation. The line-segment topology is a strong argument for closed-form filtering; commit only after a back-of-envelope on the discretisation cost of the voxel route.
- **Approach corridor definition.** The "short approach corridor" is named in §11.4 without a length, a sampling density, or a stopping criterion for the forward simulation. Define these before integration with 03.
- **Forward simulation source.** The corridor check forward-simulates "the controller's intended retract-and-extend profile" — i.e. it depends on 03's warm-start sweep or on a separate simplified model. Decide which, and the dependency direction; relevant for 05's latency budget.
- **Tolerance vs as-built deviation.** Construction-site as-built deviations are of order centimetres (lit review §4.5, citing Blum 2020); the upward reachable line segment is narrow. δ_margin must absorb both ε and the BIM-vs-as-built error budget; that combination is not analysed in §11.4.
- **Sequencing.** Greedy nearest-neighbour is named as the sequencing rule, but the inter-target cost shape (Euclidean, time-to-traverse against the costmap, reconfiguration-weighted) is not pinned down.

## Cross-references

- 02 — variance-aware clearance field (the same `f` is queried for both filters)
- 03 — perceptive RHC (supplies the retract-and-extend profile used in the corridor check)
- 05 — embedded deployment (offline IRM construction must not bleed into the online budget; online filter goes through the same DDS interface as 03)
