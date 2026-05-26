#!/usr/bin/env python3
"""sim_validation_02b — variance-aware ε λ-sweep postprocess.

Reads the four per-cell summary.json files (λ ∈ {1, 2, 3, 4}) and appends
six verdict blocks to sweep_summary.md:

  1. In-sweep reproducibility — λ=3 cell vs archived 02a tightening +
     elevation RMSE + σ² post-init mean.
  2. Per-cell quantile cross-check — closed-form ε_q − ε_static = λ √σ²_q
     verification at each cell.
  3. Per-cell Jensen bound — tightening_post_init_mean ≤ λ √σ²_post_init_mean
     at each cell.
  4. Per-cell consistency — init_dominated_frac match between ε and variance.
  5. Cross-cell linearity — slope `tightening_post_init_mean / λ` across all
     cells must be invariant within ±5 % relative (the σ²-reproducibility
     tolerance). This is the headline structural gate that verifies the
     kernel's λ scaling is exactly linear.
  6. Headline sensitivity figure — table of tightening_post_init_mean,
     tightening_post_init_max, and σ² post-init mean per λ.

Usage:
  python3 experiments/runners/sim_validation_02b_postprocess.py \\
      experiments/results/sim_validation_02b/lambda_sweep__<sha>_<git> \\
      [--archived-02a experiments/results/sim_validation_02a/variance_aware_epsilon_baseline__<sha>_<git>]
"""

import argparse
import json
import math
import pathlib
import statistics
import sys


DEFAULT_ARCHIVED_02A = pathlib.Path(
    "experiments/results/sim_validation_02a/"
    "variance_aware_epsilon_baseline__5abb8400ad08_d34c74e"
)


def _load_summary(cell_dir: pathlib.Path) -> dict:
    p = cell_dir / "summary.json"
    if not p.exists():
        return {}
    return json.loads(p.read_text())


def _fmt(v, prec=4):
    if v is None or v == "":
        return "—"
    if isinstance(v, float):
        return f"{v:.{prec}f}"
    return str(v)


def _fmt_mm(v):
    if v is None:
        return "—"
    return f"{v * 1000:+.1f}"


def _verdict(passed: bool) -> str:
    return "**PASS**" if passed else "**FAIL**"


def _section(title: str) -> list[str]:
    return ["", f"## {title}", ""]


def _cell_lam(cell: dict) -> float | None:
    """Extract the λ this cell was run at. Prefers the run_overrides value
    (authoritative, set by the runner) and falls back to the metric script's
    params block (which reflects what was actually applied at runtime)."""
    ov = cell.get("run_overrides") or {}
    if "lam" in ov:
        return float(ov["lam"])
    eps = cell.get("epsilon") or {}
    lam = (eps.get("params") or {}).get("lam")
    return float(lam) if lam is not None else None


def _cell_label(cell: dict) -> str:
    return cell.get("label", "?")


def in_sweep_reproducibility_block(
    cells_by_lam: dict, archived: dict,
    rmse_tol: float, var_tol_rel: float, tight_tol_rel: float,
) -> tuple[list[str], bool]:
    lines = _section("In-sweep reproducibility (λ=3 cell vs archived 02a)")
    cur = cells_by_lam.get(3.0)
    if cur is None:
        lines.append("λ=3 cell not present in sweep — reproducibility check cannot run.")
        return lines, False
    if not archived:
        lines.append("Archived 02a baseline not loadable — reproducibility check skipped.")
        return lines, True

    cur_rmse_all = cur.get("RMSE_all")
    cur_rmse_core = cur.get("RMSE_core")
    arc_rmse_all = archived.get("RMSE_all")
    arc_rmse_core = archived.get("RMSE_core")
    d_all = (cur_rmse_all - arc_rmse_all) if (cur_rmse_all is not None and arc_rmse_all is not None) else None
    d_core = (cur_rmse_core - arc_rmse_core) if (cur_rmse_core is not None and arc_rmse_core is not None) else None

    cur_var = (cur.get("variance") or {}).get("post_init_mean")
    arc_var = (archived.get("variance") or {}).get("post_init_mean")
    if cur_var is not None and arc_var is not None and arc_var > 0:
        var_rel = abs(cur_var - arc_var) / arc_var
    else:
        var_rel = None

    cur_tight = (cur.get("epsilon") or {}).get("tightening_post_init_mean")
    arc_tight = (archived.get("epsilon") or {}).get("tightening_post_init_mean")
    if cur_tight is not None and arc_tight is not None and arc_tight > 0:
        tight_rel = abs(cur_tight - arc_tight) / arc_tight
    else:
        tight_rel = None

    rmse_pass = (d_all is not None and abs(d_all) <= rmse_tol
                 and d_core is not None and abs(d_core) <= rmse_tol)
    var_pass = var_rel is None or var_rel <= var_tol_rel
    tight_pass = tight_rel is None or tight_rel <= tight_tol_rel
    passed = rmse_pass and var_pass and tight_pass

    lines += [
        f"- RMSE_all:  current = {_fmt(cur_rmse_all)} m, "
        f"archived = {_fmt(arc_rmse_all)} m, Δ = {_fmt_mm(d_all)} mm "
        f"(tol ±{int(rmse_tol*1000)} mm)",
        f"- RMSE_core: current = {_fmt(cur_rmse_core)} m, "
        f"archived = {_fmt(arc_rmse_core)} m, Δ = {_fmt_mm(d_core)} mm "
        f"(tol ±{int(rmse_tol*1000)} mm)",
        f"- σ² post-init mean: current = {_fmt(cur_var, prec=6)}, "
        f"archived = {_fmt(arc_var, prec=6)}, "
        f"relative Δ = {(_fmt(var_rel*100, prec=1) + ' %') if var_rel is not None else '—'} "
        f"(tol ±{int(var_tol_rel*100)} %)",
        f"- tightening_post_init_mean: current = {_fmt(cur_tight, prec=5)} m, "
        f"archived = {_fmt(arc_tight, prec=5)} m, "
        f"relative Δ = {(_fmt(tight_rel*100, prec=1) + ' %') if tight_rel is not None else '—'} "
        f"(tol ±{int(tight_tol_rel*100)} %)",
        "",
        f"Verdict: {_verdict(passed)}.",
    ]
    return lines, passed


def per_cell_quantile_block(
    cells_by_lam: dict, tol_m: float,
) -> tuple[list[str], bool]:
    """ε_q − ε_static = λ √σ²_q at each (cell, quantile) pair."""
    lines = _section("Per-cell ε quantile cross-check (wire-in structural)")
    all_pass = True
    rows = []
    for lam in sorted(cells_by_lam.keys()):
        cell = cells_by_lam[lam]
        eps = cell.get("epsilon") or {}
        var = cell.get("variance") or {}
        eps_static = eps.get("eps_static")
        lam_runtime = (eps.get("params") or {}).get("lam")
        checks = [
            ("p50,full", eps.get("p50"), var.get("p50")),
            ("p90,full", eps.get("p90"), var.get("p90")),
            ("p95,full", eps.get("p95"), var.get("p95")),
            ("p50,post", eps.get("post_init_p50"), var.get("post_init_p50")),
            ("p90,post", eps.get("post_init_p90"), var.get("post_init_p90")),
        ]
        if eps_static is None or lam_runtime is None:
            rows.append((lam, "—", "(missing eps_static or params.lam)", False))
            all_pass = False
            continue
        cell_pass = True
        worst_dev_mm = 0.0
        for label, eps_q, var_q in checks:
            if eps_q is None or var_q is None:
                cell_pass = False
                continue
            predicted = eps_static + lam_runtime * math.sqrt(max(var_q, 0.0))
            dev = eps_q - predicted
            if abs(dev) > tol_m:
                cell_pass = False
            worst_dev_mm = max(worst_dev_mm, abs(dev) * 1000)
        rows.append((lam, lam_runtime, f"max |Δ| = {worst_dev_mm:.3f} mm", cell_pass))
        if not cell_pass:
            all_pass = False

    lines.append("| λ (YAML) | λ (runtime) | worst quantile Δ | pass |")
    lines.append("|---------:|------------:|:-----------------|:----:|")
    for lam, lam_runtime, dev_str, ok in rows:
        lam_runtime_str = _fmt(lam_runtime, prec=2) if isinstance(lam_runtime, float) else str(lam_runtime)
        lines.append(
            f"| {lam:.1f} | {lam_runtime_str} | {dev_str} | {'✓' if ok else '✗'} |"
        )
    lines += [
        "",
        f"Tolerance: ±{int(tol_m*1000)} mm (5 paired quantile identities per cell).",
        f"Verdict: {_verdict(all_pass)}.",
    ]
    if not all_pass:
        lines.append(
            "Per-cell failure: wire-in is not faithfully invoking the kernel "
            "at the failing λ — check that `-p lam:=...` in the runner's "
            "metrics cmd reaches `declare_parameter('lam', ...)` in "
            "compute_ceiling_metrics.py for that cell.")
    return lines, all_pass


def per_cell_jensen_block(
    cells_by_lam: dict, slack_m: float,
) -> tuple[list[str], bool]:
    lines = _section("Per-cell Jensen bound on post-init mean")
    all_pass = True
    lines.append("| λ | tightening [mm] | λ √σ²_mean [mm] | gap [mm] | pass |")
    lines.append("|--:|---------------:|---------------:|--------:|:----:|")
    for lam in sorted(cells_by_lam.keys()):
        cell = cells_by_lam[lam]
        eps = cell.get("epsilon") or {}
        var = cell.get("variance") or {}
        eps_post_mean = eps.get("post_init_mean")
        var_post_mean = var.get("post_init_mean")
        eps_static = eps.get("eps_static")
        lam_runtime = (eps.get("params") or {}).get("lam")
        if (eps_post_mean is None or var_post_mean is None
                or eps_static is None or lam_runtime is None):
            lines.append(f"| {lam:.1f} | — | — | — | — |")
            continue
        tightening = eps_post_mean - eps_static
        ceiling = lam_runtime * math.sqrt(max(var_post_mean, 0.0))
        gap = ceiling - tightening
        ok = gap >= -slack_m
        if not ok:
            all_pass = False
        lines.append(
            f"| {lam:.1f} | {tightening*1000:.1f} | {ceiling*1000:.1f} | "
            f"{gap*1000:+.1f} | {'✓' if ok else '✗'} |"
        )
    lines += [
        "",
        f"Tolerance: gap ≥ −{int(slack_m*1000)} mm (non-negative is expected by Jensen).",
        f"Verdict: {_verdict(all_pass)}.",
    ]
    return lines, all_pass


def per_cell_consistency_block(
    cells_by_lam: dict, tol: float,
) -> tuple[list[str], bool]:
    lines = _section("Per-cell ε vs variance init_dominated_frac consistency")
    all_pass = True
    lines.append("| λ | ε.init_dom % | var.init_dom % | |Δ| % | pass |")
    lines.append("|--:|-------------:|--------------:|-----:|:----:|")
    for lam in sorted(cells_by_lam.keys()):
        cell = cells_by_lam[lam]
        eps = cell.get("epsilon") or {}
        var = cell.get("variance") or {}
        f_eps = eps.get("init_dominated_frac")
        f_var = var.get("init_dominated_frac")
        if f_eps is None or f_var is None:
            lines.append(f"| {lam:.1f} | — | — | — | — |")
            continue
        diff = abs(f_eps - f_var)
        ok = diff <= tol
        if not ok:
            all_pass = False
        lines.append(
            f"| {lam:.1f} | {f_eps*100:.3f} | {f_var*100:.3f} | "
            f"{diff*100:.4f} | {'✓' if ok else '✗'} |"
        )
    lines += [
        "",
        f"Tolerance: ±{tol*100:.1f} %.",
        f"Verdict: {_verdict(all_pass)}.",
    ]
    return lines, all_pass


def cross_cell_linearity_block(
    cells_by_lam: dict, tol_rel: float,
) -> tuple[list[str], bool]:
    """tightening_post_init_mean / λ should be invariant across cells.

    This is the headline structural gate: at fixed framework params σ² is
    λ-independent, so E[√σ²_post_init] = tightening_post_init_mean / λ is
    invariant. Tolerance is ±5 % relative against the median slope across
    cells (matches σ²-reproducibility tolerance).
    """
    lines = _section("Cross-cell linearity (tightening / λ invariance — headline structural gate)")
    slopes: list[tuple[float, float, float]] = []  # (lam, tightening, slope)
    for lam in sorted(cells_by_lam.keys()):
        cell = cells_by_lam[lam]
        eps = cell.get("epsilon") or {}
        tightening = eps.get("tightening_post_init_mean")
        lam_runtime = (eps.get("params") or {}).get("lam")
        if tightening is None or lam_runtime is None or lam_runtime == 0:
            continue
        slopes.append((lam_runtime, tightening, tightening / lam_runtime))

    if len(slopes) < 2:
        lines.append("Fewer than 2 cells with valid slopes — linearity check skipped.")
        return lines, False

    slope_values = [s for _, _, s in slopes]
    median_slope = statistics.median(slope_values)
    spread = max(slope_values) - min(slope_values)
    spread_rel = spread / median_slope if median_slope > 0 else None

    lines += [
        f"Predicted: tightening_post_init_mean(λ) = λ · E[√σ²_post_init], "
        f"with E[√σ²_post_init] invariant across cells.",
        "",
        "| λ | tightening_pim [mm] | slope = tpim/λ [mm] | rel dev vs median |",
        "|--:|-------------------:|--------------------:|-----------------:|",
    ]
    all_pass = True
    for lam_runtime, tightening, slope in slopes:
        rel_dev = (slope - median_slope) / median_slope if median_slope > 0 else None
        if rel_dev is None or abs(rel_dev) > tol_rel:
            all_pass = False
        rel_str = f"{rel_dev*100:+.2f} %" if rel_dev is not None else "—"
        lines.append(
            f"| {lam_runtime:.1f} | {tightening*1000:.2f} | "
            f"{slope*1000:.3f} | {rel_str} |"
        )
    lines += [
        "",
        f"Median slope E[√σ²_post_init] = {median_slope*1000:.3f} mm "
        f"(spread = {spread*1000:.3f} mm = "
        f"{(spread_rel*100) if spread_rel is not None else float('nan'):.2f} % rel).",
        f"Tolerance: each cell's slope within ±{tol_rel*100:.1f} % of median.",
        f"Verdict: {_verdict(all_pass)}.",
    ]
    if not all_pass:
        lines.append(
            "Cross-cell linearity violated: either the wire-in is not "
            "reading λ correctly at the offending cell (verify "
            "`params.lam` in its metrics_diagnostic.json), σ² is drifting "
            "more than σ²-reproducibility predicts (check per-cell "
            "variance.post_init_mean against the median), or the kernel "
            "is mis-applying λ (regression — the prototype unit tests "
            "would catch this in isolation)."
        )
    return lines, all_pass


def headline_sensitivity_block(
    cells_by_lam: dict, per_cell_min_m: float, per_cell_max_m: float,
) -> tuple[list[str], bool]:
    lines = _section("Headline sensitivity figure (tightening_post_init vs λ)")
    lines += [
        "| λ | σ² post-init mean | tightening_post_init_mean [mm] | "
        "tightening_post_init_max [mm] | in-range |",
        "|--:|------------------:|------------------------------:|"
        "------------------------------:|:--------:|",
    ]
    all_in_range = True
    for lam in sorted(cells_by_lam.keys()):
        cell = cells_by_lam[lam]
        eps = cell.get("epsilon") or {}
        var = cell.get("variance") or {}
        tpim = eps.get("tightening_post_init_mean")
        tpimax = eps.get("tightening_post_init_max")
        var_pim = var.get("post_init_mean")
        if tpim is None:
            lines.append(f"| {lam:.1f} | — | — | — | — |")
            continue
        in_range = per_cell_min_m <= tpim <= per_cell_max_m
        if not in_range:
            all_in_range = False
        lines.append(
            f"| {lam:.1f} | {_fmt(var_pim, prec=6)} | "
            f"{tpim*1000:.1f} | "
            f"{_fmt((tpimax or 0)*1000, prec=1)} | "
            f"{'✓' if in_range else '✗'} |"
        )
    lines += [
        "",
        f"Per-cell sanity band: [{int(per_cell_min_m*1000)}, "
        f"{int(per_cell_max_m*1000)}] mm.",
        f"Verdict: {_verdict(all_in_range)}.",
    ]
    return lines, all_in_range


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result_dir", type=pathlib.Path)
    ap.add_argument(
        "--archived-02a", type=pathlib.Path, default=None,
        help="Archived 02a baseline dir for in-sweep reproducibility check.",
    )
    args = ap.parse_args()

    rd = args.result_dir
    if not rd.is_dir():
        sys.exit(f"not a directory: {rd}")

    manifest_path = rd / "manifest.json"
    if not manifest_path.exists():
        sys.exit(f"manifest.json missing in {rd}")
    manifest = json.loads(manifest_path.read_text())

    cfg_path = pathlib.Path(manifest["config"])
    if not cfg_path.is_absolute():
        repo_root = pathlib.Path(__file__).resolve().parents[2]
        cfg_path = repo_root / cfg_path
    import yaml
    cfg = yaml.safe_load(cfg_path.read_text())
    accept = cfg.get("acceptance", {}) or {}
    repro_tol = float(accept.get("reproducibility_tolerance_m", 0.005))
    var_tol_rel = float(
        accept.get("variance_post_init_mean_tolerance_relative", 0.05))
    tight_tol_rel = float(
        accept.get("tightening_lam3_tolerance_relative", 0.05))
    quantile_tol = float(
        accept.get("epsilon_quantile_cross_check_tolerance_m", 0.001))
    jensen_slack = float(accept.get("jensen_slack_m", 0.005))
    init_dom_tol = float(accept.get("init_dom_frac_match_tolerance", 0.005))
    lin_tol_rel = float(
        accept.get("linearity_slope_tolerance_relative", 0.05))
    per_cell_min = float(accept.get("tightening_per_cell_min_m", 0.02))
    per_cell_max = float(accept.get("tightening_per_cell_max_m", 0.80))

    cell_dirs = sorted(
        d for d in rd.iterdir()
        if d.is_dir() and (d / "summary.json").exists()
        and d.name not in ("rendered_configs", "logs")
    )
    if not cell_dirs:
        sys.exit(f"no summary.json files under {rd}")

    cells_by_lam: dict[float, dict] = {}
    for cd in cell_dirs:
        cell = _load_summary(cd)
        if not cell:
            continue
        lam = _cell_lam(cell)
        if lam is None:
            print(
                f"warning: cell {cd.name} has no λ in run_overrides or params — skipping",
                file=sys.stderr,
            )
            continue
        cells_by_lam[lam] = cell

    if not cells_by_lam:
        sys.exit(f"no cells with a valid λ extracted from {rd}")

    arc_dir = args.archived_02a
    if arc_dir is None:
        repo_root = pathlib.Path(__file__).resolve().parents[2]
        arc_dir = repo_root / DEFAULT_ARCHIVED_02A
    # The 02a result has a single cell subdirectory; find it.
    archived = {}
    if arc_dir.is_dir():
        sub = [d for d in arc_dir.iterdir()
               if d.is_dir() and (d / "summary.json").exists()
               and d.name not in ("rendered_configs", "logs")]
        if sub:
            archived = _load_summary(sub[0])

    repro_lines, repro_pass = in_sweep_reproducibility_block(
        cells_by_lam, archived, repro_tol, var_tol_rel, tight_tol_rel)
    quantile_lines, quantile_pass = per_cell_quantile_block(
        cells_by_lam, quantile_tol)
    jensen_lines, jensen_pass = per_cell_jensen_block(
        cells_by_lam, jensen_slack)
    consistency_lines, consistency_pass = per_cell_consistency_block(
        cells_by_lam, init_dom_tol)
    linearity_lines, linearity_pass = cross_cell_linearity_block(
        cells_by_lam, lin_tol_rel)
    headline_lines, headline_pass = headline_sensitivity_block(
        cells_by_lam, per_cell_min, per_cell_max)

    md_path = rd / "sweep_summary.md"
    existing = md_path.read_text() if md_path.exists() else ""
    marker = "<!-- sim_validation_02b_postprocess BEGIN -->"
    end_marker = "<!-- sim_validation_02b_postprocess END -->"
    if marker in existing and end_marker in existing:
        before = existing.split(marker)[0].rstrip() + "\n"
        after_split = existing.split(end_marker, 1)
        after = after_split[1] if len(after_split) > 1 else ""
        existing = before + after.lstrip()

    block = (
        "\n" + marker + "\n"
        + "\n".join(repro_lines)
        + "\n" + "\n".join(quantile_lines)
        + "\n" + "\n".join(jensen_lines)
        + "\n" + "\n".join(consistency_lines)
        + "\n" + "\n".join(linearity_lines)
        + "\n" + "\n".join(headline_lines)
        + "\n" + end_marker + "\n"
    )
    md_path.write_text(existing + block)
    print(f"wrote postprocess block to {md_path}")

    structural_pass = (
        quantile_pass and jensen_pass and consistency_pass
        and linearity_pass and headline_pass)
    if not structural_pass:
        print(
            "STRUCTURAL CHECK FAILED — see sweep_summary.md",
            file=sys.stderr,
        )
        return 2
    if not repro_pass:
        print(
            "REPRODUCIBILITY CHECK FAILED — see sweep_summary.md",
            file=sys.stderr,
        )
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
