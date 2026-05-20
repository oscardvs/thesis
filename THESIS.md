# THESIS.md — project intent and working methodology

Read this together with `CLAUDE.md`. `CLAUDE.md` covers the workspace (ROS 2 environment, packages, launchers, conventions). This file covers **what we're building and how we work**. Read both at session start.

## What we're building

A perception-to-control pipeline that lets HILDA navigate autonomously under variable overhead clearance and position itself for ceiling drilling, on embedded hardware. The pipeline is:

**Dual-layer elevation mapping** (floor + ceiling via z-inversion encoding into the same `elevation_mapping_cupy` framework) → **variance-aware clearance field** f(x, y, s) where s is the combined sledge extension → **perceptive NMPC** over augmented state (x, y, θ, v, ω, s) with the morphological reconfiguration variable as a first-class decision variable → **approach-aware inverse reachability** for base-pose selection that verifies the entire approach corridor, not only the terminal pose.

Target hardware: Jetson Orin Nano Super (8 GB unified memory) running the perception + planning pipeline; UDOO Bolt V8 running low-level control, state estimation, and DDS networking. Both on ROS 2 Jazzy.

## The five gaps (use these handles everywhere)

The literature study identified five gaps the thesis closes. Reference them by tag in commits, journal entries, and docs.

- **G1** — dual-layer 2.5D mapping on embedded GPU (no published dual-instance benchmark on Orin Nano-class hardware)
- **G2** — variance-aware ceiling clearance: per-cell σ²_zceil + σ²_zfloor consumed as chance-constraint tightening at the ceiling layer
- **G3** — joint planar-and-configuration control: the morphological variable s carrying both a transit-rate penalty and a per-target terminal task value, driven internally by clearance
- **G4** — approach-aware inverse reachability: verify configuration feasibility along the entire approach corridor, not only at the goal pose
- **G5** — end-to-end embedded characterisation: dual mapping + constraint field + RHC controller under concurrent perception load on a unified-memory module

G3 and G4 are the deepest contribution; G1, G2, G5 are enabling. The architectural choice of controller family for G3 is **deliberately open** — see "Decisions not yours to close" below.

## How we work — non-negotiable disciplines

**Theory document before code, per module.** Each pipeline module has a doc at `thesis/docs/NN_<module>.md`: formal problem statement, math derivation, design choices and rationale, interfaces in and out, open questions. The doc precedes the code. When code teaches us something new, the doc updates in the same change. Before touching module code, read the doc. If the doc doesn't cover what's being asked, draft the update first, surface the question, then code.

**Engineering journal.** Weekly file at `thesis/journal/YYYY-Www.md`, with dated entries. After non-trivial work, add an entry without asking. Format: date, what was tried, what worked, what didn't, what changed. Brief and factual — not narrative.

**Experiments are configs, not scripts.** Every experiment has a YAML at `thesis/experiments/configs/<area>/<id>_<name>.yaml`. A runner consumes a config path and writes to a results directory named from the config hash and git commit. The runner emits a `manifest.json` next to outputs. Never hard-code parameters in scripts. Never run an experiment without a config committed first.

**Decision records.** Non-obvious choices get a short note at `thesis/docs/decisions/NNNN-<slug>.md`: context, options considered, choice, rationale, consequences. Two paragraphs is enough. Examples: why first-order integrator on s, why λ = 3 in the safety margin, why SQP-RTI not full SQP.

**Branch per module.** Feature branches named after the module. Merge to `main` only after the module passes its isolation test. Existing branches stay as-is: `hilda` for pointcloud packages, `traversability` for nav, `adjustable_trav_height` for the traversability package.

**Validate in isolation before integration.** G1 (dual mapping) and G2 (variance tightening) are perception-side and testable with rosbag in, clearance field out, ground truth comparison — no controller needed. Build those benchmarks before integrating with NMPC.

## Repo additions

The existing layout stays put. Three new packages at workspace root, plus `thesis/` restructured:

- `hilda_clearance_field/` — clearance + feasibility field node, GPU kernel, CasADi B-spline export. Kept separate from `hilda_ceiling` because both NMPC and HMPC will consume from it.
- `hilda_nmpc/` — acados OCP, Python prototype first, Nav2 controller plugin later. Separate from `hilda_navigation` so controller swaps don't touch the existing nav stack.
- `hilda_irm/` — offline IRM construction + online approach-aware filtering against the clearance field.

`thesis/` reorganises to:
```
thesis/
├── docs/                   theory + design per module
│   ├── 00_pipeline_overview.md
│   ├── 01_dual_elevation_mapping.md
│   ├── 02_variance_aware_clearance.md
│   ├── 03_nmpc_formulation.md
│   ├── 04_approach_aware_irm.md
│   ├── 05_embedded_deployment.md
│   └── decisions/          ADR-style notes
├── journal/                weekly engineering log
├── experiments/
│   ├── configs/            one YAML per experiment
│   ├── runners/            scripts that consume a config path
│   └── results/            gitignored; never deleted manually
└── literature_review/      (existing — leave as is)
```

## Decisions not yours to close

These remain open until experimental evidence resolves them. Don't pre-commit by code structure or default arguments.

- **NMPC vs HMPC** for the controller. Lit review left both viable; the call depends on Jetson latency under load. Build the constraint-field interface (C¹ field f, gradients, plane segments as halfspaces) to be controller-agnostic. Both consume from `hilda_clearance_field`.
- **Semantic painting and 3DGS for the ceiling.** Out of scope until G1 + G2 are validated geometrically.
- **Goal selection: explicit IRM grid vs analytical (mast-aware) reachability.** The mast topology makes the upward-reachable set a vertical line segment; this may allow a closed-form filter rather than a 6D voxel grid. Don't commit either way before the docs say so.

## NDA and supervisors

- The robot's internal Hilti project codename is confidential. Use "the platform," "the robot," or "HILDA" only — in code, docs, and commit messages.
- Confidential numerical values (specific dimensions, weights, internal targets) do not go in committed files. Use symbolic constants pulled from a non-committed config; surface what needs to be supplied.
- Thesis supervisors are **Dr. Laura Ferranti (TU Delft, CoR)** and **Riccardo Balbi (Hilti AG)**. Robert Babuška supervised an earlier internship only — do not include him on thesis-stage deliverables.

## Writing style

- Plain, direct, compressed prose. No bureaucratic hedging ("it should be noted that," "in order to," "we can see that").
- Trust symbols and context; let equations carry the technical content.
- British English in thesis-facing docs and writing.
- Tables only when prose doesn't serve. Default to prose, even for comparisons.
- No emoji, no decorative markdown headers, no padding.

## When uncertain

Ask. If `thesis/docs/` doesn't cover what's needed, draft the doc update first and surface the question — don't infer from generic robotics patterns or close an open decision.
