# 0008 — experiment-runner-matrix-generalisation

Status: accepted
Date: 2026-05-22
Gap(s): G1
Module: 01_dual_elevation_mapping.md

## Context

`sim_validation_01_joint_sweep.py` was built to test a specific two-axis question: ceiling cell size × `enable_edge_sharpen`. The matrix iteration is a literal `for cell_size, edge_sharpen in product(cfg["sweep"]["matrix"]["cell_size"], cfg["sweep"]["matrix"]["enable_edge_sharpen"])` at line 465; the YAML renderer (`render_ceiling_yaml`) takes those two values as positional kwargs and hard-aliases `cell_size → resolution`; the per-run label format `cs{int(cell_size*100):03d}_es{int(edge_sharpen)}` and the summary's "Suppression contribution" table are both shaped around the same two axes.

ADR 0006 §F2 named the natural next experiment — sweep `wall_num_thresh ∈ {5, 10, 20, 50}` at a fixed cell size — and the corrected-sweep result on 2026-05-21 (`joint_sweep__04a2e8802fc1_d57b109/sweep_summary.md`) re-affirmed the need: the rule contributes at production threshold=20 but the magnitude is small at production cell size; the operating-point question is whether lowering the threshold maximises -ΔRMSE_boundary without inflating RMSE_core. Threshold is *a different ceiling YAML key* than the two the current runner sweeps; extending the experiment without changing the runner is not possible.

A further structural observation: asymmetric matrices recur naturally in this experiment family. The `wall_num_thresh` sweep wants four threshold values with `enable_edge_sharpen: true` plus *one* `enable_edge_sharpen: false` baseline at the production threshold. Forcing the matrix to a Cartesian product would mean four redundant F-baseline runs (one per threshold) at ~9 min each — 27 min wasted on duplicates of a measurement that yesterday's archived `cs015_es0` already provides.

## Options

- **A — generalise the runner in place.** Replace the two-positional-arg matrix with an N-axis dict (axis name → list of values), with each axis name mapping directly to a ceiling YAML key (with `cell_size → resolution` retained as the only alias). Add an `extra_runs` list for explicit non-product additions (the asymmetric-baseline case). Backward-compatible with `sim_validation_01/joint_sweep.yaml` because the existing axes (`cell_size`, `enable_edge_sharpen`) become dict entries by the same names.
- **B — duplicate the runner per experiment.** Copy `sim_validation_01_joint_sweep.py` to `sim_validation_01b_threshold_sweep.py` and edit the matrix iteration in place. Faster to ship today; long-term every new experiment carries a fresh copy of ~500 lines of common scaffolding (manifest, GT extraction, process management, metrics collection, summary writing).
- **C — do nothing; keep all sweep axes as positional args.** Add `wall_num_thresh` as a third positional arg, three-arg product loop, three-segment label. Mechanically smallest; structurally regresses — every future axis perpetuates the same per-axis hardcoding.

## Choice

A — generalise to N-dim matrix dict with axis-to-YAML-key mapping, plus an `extra_runs` list for asymmetric matrices.

## Rationale

The marginal cost of A over B is small in code volume (~30 lines net change in the runner) and structurally meaningful in expressivity. Any axis present in the ceiling YAML becomes sweepable by adding its key to `cfg["sweep"]["matrix"]`; nothing in the runner needs touching for the next experiment of this shape. The `cell_size` alias is the only special case (the existing GT-extraction code keys on cell size, so renaming the YAML key would ripple further than is worth).

`extra_runs` is the substantive new expressivity. Asymmetric matrices — one F-baseline plus N points along another axis — are the natural shape when one axis is a control (rule on/off) and the others are dose-response variables. Without `extra_runs`, the alternatives are (i) full Cartesian product with redundant baselines, (ii) two separate sweep runs (one matrix, one one-off), (iii) per-experiment custom runners. All three either waste compute or fragment the audit trail. `extra_runs` keeps each experiment's full set of runs in one result directory with one manifest, one `sweep_summary`, one config provenance.

Option B was rejected on the long-term-cost axis. The experiment family (sim_validation_01, 01b, 02b, 03, …) is growing — duplicating 500 lines per experiment is the kind of structural debt that compounds. Option C was rejected because three positional args is no better than two on the abstraction floor; the next reviewer hits the same wall ADR 0006 hit.

**Experiment-specific commentary moves to per-experiment postprocess scripts.** The current runner's inline `write_summary` carries a sim_validation_01-specific "Interpretation" paragraph referencing `README.md §Expected pattern`. After the refactor, the inline summary stays generic (per-run table + on/off comparison block driven by what's in the matrix); experiment-specific recommendations live in scripts like `sim_validation_01b_postprocess.py`. This keeps the runner runner-shaped and the interpretation interpretation-shaped.

**Labels and provenance.** Per-run labels are generated by a short-encoding map: `cell_size → cs{int(v*100):03d}`, `enable_edge_sharpen → es{int(v)}`, `wall_num_thresh → wnt{int(v):03d}`, falling back to `{key}{value}` for unknown axes. Each run's `summary.json` carries the full `run_overrides` dict so postprocess scripts can read by-axis without parsing labels — the maintenance trap (regex per encoding scheme) is avoided.

**GT extraction pre-deduplication.** GT grids are keyed by cell size only. `extra_runs` may share a cell size with the matrix (the F-baseline case does). The runner builds `unique_cell_sizes = {r['cell_size'] for r in product(matrix) ++ extra_runs}` before extraction; one GT per unique value, no re-extraction or key collision.

## Consequences

- `sim_validation_01_joint_sweep.py` is refactored in place. The two-axis loop becomes an N-axis loop; `render_ceiling_yaml` takes a `run_overrides` dict; the on/off block is data-driven by the matrix shape.
- `sim_validation_01/joint_sweep.yaml` requires no changes — the existing matrix (`cell_size: [...], enable_edge_sharpen: [...]`) maps to the new dict form by the same key names. A regression dry-run verifies label and rendered-config equivalence to yesterday's archived results before any new experiment lands.
- `sim_validation_01_postprocess.py` switches from label-parsing (`CELL_RE`) to reading `summary.json` per run directory — postprocess no longer needs to know the label encoding.
- `sim_validation_01b/threshold_sweep.yaml` uses the new `matrix:` + `extra_runs:` form. Future experiments follow suit.
- New experiments add their own `<experiment>_postprocess.py` for experiment-specific recommendation logic. The runner stays generic.
- The discipline lesson — **experiment-specific commentary belongs in postprocess scripts, not in the runner** — generalises: future experiments that want a domain-specific summary block add their own postprocess rather than threading a flag through `write_summary`.
- The refactor is the substantive code change of 2026-05-22. If it fails the regression dry-run and cannot be resolved in scope, the discipline is to land the refactor cleanly in its own commit and defer the downstream experiment (`sim_validation_01b`) rather than shipping a half-baked generalisation against a live experiment.
