#!/usr/bin/env python3
"""sim_validation_02a — variance-aware ε baseline postprocess.

Reads the single per-run summary.json and appends five verdict blocks to
sweep_summary.md:

  1. Reproducibility — elevation RMSE + σ² post-init mean vs archived
     01d tv=0.0001 cell.
  2. ε quantile cross-check — closed-form (ε_q − ε_static = λ √σ²_q)
     verification on p50, p90, p95 for the full and post-init populations.
     This is the structural integration check that the wire-in path
     correctly translates published σ² into ε.
  3. Jensen bound on the post-init mean (tightening_post_init_mean ≤
     λ √σ²_post_init_mean within a small slack).
  4. Tightening baseline — the headline numbers
     (tightening_post_init_mean, tightening_post_init_max,
     tightening_mean, init_dominated_frac on ε).
  5. Consistency — init_dominated_frac match between ε and variance.

Usage:
  python3 experiments/runners/sim_validation_02a_postprocess.py \\
      experiments/results/sim_validation_02a/variance_aware_epsilon_baseline__<sha>_<git> \\
      [--archived-baseline experiments/results/sim_validation_01d/time_variance_sweep__9d153d07e4dd_1b3b5e8/cs010_es1_snf050_tv00100_wnt020]
"""

import argparse
import json
import math
import pathlib
import sys


DEFAULT_ARCHIVED_BASELINE = pathlib.Path(
    "experiments/results/sim_validation_01d/"
    "time_variance_sweep__9d153d07e4dd_1b3b5e8/"
    "cs010_es1_snf050_tv00100_wnt020"
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


def reproducibility_block(
    summary: dict, archived: dict, tol_m: float, var_tol_rel: float,
) -> tuple[list[str], bool]:
    lines = _section("Reproducibility (vs archived 01d tv=0.0001)")
    if not archived:
        lines.append(
            "Archived baseline not loadable — reproducibility check skipped."
        )
        return lines, True
    if not summary:
        lines.append("This run's summary.json is empty — cannot compare.")
        return lines, False

    # Runner-produced summary.json carries RMSE flat at top level
    # (not nested under 'all'/'core' as the metric script's
    # metrics_diagnostic.json does — the runner re-projects).
    cur_rmse_all = summary.get("RMSE_all")
    cur_rmse_core = summary.get("RMSE_core")
    arc_rmse_all = archived.get("RMSE_all")
    arc_rmse_core = archived.get("RMSE_core")

    if cur_rmse_all is not None and arc_rmse_all is not None:
        d_all = cur_rmse_all - arc_rmse_all
    else:
        d_all = None
    if cur_rmse_core is not None and arc_rmse_core is not None:
        d_core = cur_rmse_core - arc_rmse_core
    else:
        d_core = None

    cur_var = (summary.get("variance") or {}).get("post_init_mean")
    arc_var = (archived.get("variance") or {}).get("post_init_mean")
    if cur_var is not None and arc_var is not None and arc_var > 0:
        var_rel = abs(cur_var - arc_var) / arc_var
    else:
        var_rel = None

    rmse_pass = (
        d_all is not None and abs(d_all) <= tol_m
        and d_core is not None and abs(d_core) <= tol_m)
    var_pass = var_rel is None or var_rel <= var_tol_rel
    passed = rmse_pass and var_pass

    lines += [
        f"- RMSE_all:  current = {_fmt(cur_rmse_all)} m, "
        f"archived = {_fmt(arc_rmse_all)} m, Δ = {_fmt_mm(d_all)} mm "
        f"(tol ±{int(tol_m*1000)} mm)",
        f"- RMSE_core: current = {_fmt(cur_rmse_core)} m, "
        f"archived = {_fmt(arc_rmse_core)} m, Δ = {_fmt_mm(d_core)} mm "
        f"(tol ±{int(tol_m*1000)} mm)",
        f"- σ² post-init mean: current = {_fmt(cur_var, prec=6)}, "
        f"archived = {_fmt(arc_var, prec=6)}, "
        f"relative Δ = {(_fmt(var_rel*100, prec=1) + ' %') if var_rel is not None else '—'} "
        f"(tol ±{int(var_tol_rel*100)} %)",
        "",
        f"Verdict: {_verdict(passed)}.",
    ]
    return lines, passed


def quantile_cross_check_block(
    summary: dict, tol_m: float,
) -> tuple[list[str], bool]:
    """ε_q − ε_static = λ √σ²_q for q ∈ {p50, p90, p95} on full +
    post-init populations. Structural integration check."""
    lines = _section("ε quantile cross-check (wire-in structural verification)")
    eps = summary.get("epsilon") or {}
    var = summary.get("variance") or {}
    if not eps or not var:
        lines.append(
            "ε or variance block missing from summary.json — wire-in did not "
            "produce expected output. Verify hilda_clearance_field is "
            "installed and the script's import succeeded."
        )
        return lines, False

    eps_static = eps.get("eps_static")
    lam = (eps.get("params") or {}).get("lam")
    if eps_static is None or lam is None:
        lines.append(
            "ε block missing eps_static or params.lam — postprocess "
            "cannot run quantile check."
        )
        return lines, False

    checks = [
        ("p50, full",  eps.get("p50"),  var.get("p50")),
        ("p90, full",  eps.get("p90"),  var.get("p90")),
        ("p95, full",  eps.get("p95"),  var.get("p95")),
        ("p50, post-init", eps.get("post_init_p50"), var.get("post_init_p50")),
        ("p90, post-init", eps.get("post_init_p90"), var.get("post_init_p90")),
    ]
    all_pass = True
    lines.append("| quantile          | ε measured | λ √σ²    | Δ [mm] | tol [mm] | pass |")
    lines.append("|-------------------|-----------:|---------:|-------:|---------:|:----:|")
    for label, eps_q, var_q in checks:
        if eps_q is None or var_q is None:
            lines.append(f"| {label:<17} | — | — | — | {int(tol_m*1000)} | — |")
            continue
        predicted = eps_static + lam * math.sqrt(max(var_q, 0.0))
        delta = eps_q - predicted
        ok = abs(delta) <= tol_m
        if not ok:
            all_pass = False
        lines.append(
            f"| {label:<17} | {eps_q:.5f} | {predicted:.5f} | "
            f"{delta*1000:+.2f} | {int(tol_m*1000)} | {'✓' if ok else '✗'} |"
        )
    lines += [
        "",
        f"Verdict: {_verdict(all_pass)}.",
        (
            ""
            if all_pass
            else "Failure indicates the wire-in is not faithfully invoking the "
            "kernel — inspect compute_ceiling_metrics.py's parameter "
            "reads, subset selection, or call signature."
        ),
    ]
    return lines, all_pass


def jensen_bound_block(
    summary: dict, slack_m: float,
) -> tuple[list[str], bool]:
    lines = _section("Jensen bound on post-init mean")
    eps = summary.get("epsilon") or {}
    var = summary.get("variance") or {}
    eps_post_mean = eps.get("post_init_mean")
    var_post_mean = var.get("post_init_mean")
    eps_static = eps.get("eps_static")
    lam = (eps.get("params") or {}).get("lam")
    if (eps_post_mean is None or var_post_mean is None
            or eps_static is None or lam is None):
        lines.append("Required fields missing — Jensen check skipped.")
        return lines, True

    tightening_mean = eps_post_mean - eps_static
    jensen_ceiling = lam * math.sqrt(max(var_post_mean, 0.0))
    gap = jensen_ceiling - tightening_mean
    passed = gap >= -slack_m  # tightening_mean ≤ jensen_ceiling + slack

    lines += [
        f"- tightening_post_init_mean = ε_post_init_mean − ε_static "
        f"= {tightening_mean*1000:.1f} mm",
        f"- Jensen ceiling λ √(σ²_post_init_mean) = {jensen_ceiling*1000:.1f} mm",
        f"- Gap (ceiling − measured) = {gap*1000:+.1f} mm "
        f"(slack ±{int(slack_m*1000)} mm; non-negative is expected by Jensen)",
        "",
        f"Verdict: {_verdict(passed)}.",
        "" if passed else
        "Jensen bound violated. Either the σ² distribution shape is unusual "
        "in this bag or the kernel is mis-aggregating; recheck epsilon_stats.",
    ]
    return lines, passed


def tightening_baseline_block(
    summary: dict, min_m: float, max_m: float,
) -> tuple[list[str], bool]:
    lines = _section("Tightening baseline (headline numbers)")
    eps = summary.get("epsilon") or {}
    if not eps:
        lines.append("ε block missing — no baseline to report.")
        return lines, False

    headline = eps.get("tightening_post_init_mean")
    if headline is None:
        lines.append("tightening_post_init_mean missing — cannot evaluate.")
        return lines, False
    in_range = min_m <= headline <= max_m

    lines += [
        f"- **tightening_post_init_mean = {headline*1000:.1f} mm** "
        f"(in-range [{int(min_m*1000)}, {int(max_m*1000)}] mm: "
        f"{'yes' if in_range else 'no'}). "
        f"Average extra margin the variance-aware kernel demands on "
        f"well-measured surfaces, vs the static ε_safety = {eps.get('eps_static', 0.0)*1000:.0f} mm baseline.",
        f"- tightening_post_init_max  = {_fmt((eps.get('tightening_post_init_max') or 0)*1000, prec=1)} mm",
        f"- tightening_mean (full pop) = {_fmt((eps.get('tightening_mean') or 0)*1000, prec=1)} mm "
        f"(includes init-dominated cells; not the load-bearing figure)",
        f"- tightening_max (full pop)  = {_fmt((eps.get('tightening_max') or 0)*1000, prec=1)} mm",
        f"- init_dominated_frac (ε)    = {_fmt((eps.get('init_dominated_frac') or 0)*100, prec=1)} %",
        f"- n_post_init / n_valid (ε)  = {eps.get('n_post_init')} / {eps.get('n_valid')}",
        "",
        f"Verdict: {_verdict(in_range)}.",
        "" if in_range else
        "Tightening_post_init_mean outside the expected band. "
        "Verify σ² post-init mean against 01d's measurement; if σ² is "
        "in-range, the ε parameters at script defaults are likely not "
        "being read (check the metric node's ROS param state).",
    ]
    return lines, in_range


def consistency_block(
    summary: dict, tol: float,
) -> tuple[list[str], bool]:
    lines = _section("Consistency (ε vs variance init_dominated_frac)")
    eps = summary.get("epsilon") or {}
    var = summary.get("variance") or {}
    f_eps = eps.get("init_dominated_frac")
    f_var = var.get("init_dominated_frac")
    if f_eps is None or f_var is None:
        lines.append("init_dominated_frac missing in one of the blocks.")
        return lines, False
    diff = abs(f_eps - f_var)
    passed = diff <= tol
    lines += [
        f"- ε.init_dominated_frac   = {f_eps*100:.2f} %",
        f"- variance.init_dominated_frac = {f_var*100:.2f} %",
        f"- |Δ| = {diff*100:.3f} % (tol {tol*100:.1f} %)",
        "",
        f"Verdict: {_verdict(passed)}.",
    ]
    return lines, passed


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result_dir", type=pathlib.Path)
    ap.add_argument(
        "--archived-baseline", type=pathlib.Path, default=None,
        help="Archived 01d tv=0.0001 cell for reproducibility check.",
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
    quantile_tol = float(
        accept.get("epsilon_quantile_cross_check_tolerance_m", 0.001))
    jensen_slack = float(accept.get("jensen_slack_m", 0.005))
    tightening_min = float(
        accept.get("tightening_post_init_mean_min_m", 0.02))
    tightening_max = float(
        accept.get("tightening_post_init_mean_max_m", 0.50))
    init_dom_tol = float(accept.get("init_dom_frac_match_tolerance", 0.005))

    cell_dirs = sorted(
        d for d in rd.iterdir()
        if d.is_dir() and (d / "summary.json").exists()
        and d.name not in ("rendered_configs", "logs")
    )
    if not cell_dirs:
        sys.exit(f"no summary.json files under {rd}")
    if len(cell_dirs) != 1:
        print(
            f"warning: expected 1 cell, found {len(cell_dirs)}; using first",
            file=sys.stderr,
        )
    summary = _load_summary(cell_dirs[0])

    arc_dir = args.archived_baseline
    if arc_dir is None:
        repo_root = pathlib.Path(__file__).resolve().parents[2]
        arc_dir = repo_root / DEFAULT_ARCHIVED_BASELINE
    archived = _load_summary(arc_dir) if arc_dir.is_dir() else {}

    repro_lines, repro_pass = reproducibility_block(
        summary, archived, repro_tol, var_tol_rel)
    quantile_lines, quantile_pass = quantile_cross_check_block(
        summary, quantile_tol)
    jensen_lines, jensen_pass = jensen_bound_block(summary, jensen_slack)
    tightening_lines, tightening_pass = tightening_baseline_block(
        summary, tightening_min, tightening_max)
    consistency_lines, consistency_pass = consistency_block(
        summary, init_dom_tol)

    md_path = rd / "sweep_summary.md"
    existing = md_path.read_text() if md_path.exists() else ""
    marker = "<!-- sim_validation_02a_postprocess BEGIN -->"
    end_marker = "<!-- sim_validation_02a_postprocess END -->"
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
        + "\n" + "\n".join(tightening_lines)
        + "\n" + "\n".join(consistency_lines)
        + "\n" + end_marker + "\n"
    )
    md_path.write_text(existing + block)
    print(f"wrote postprocess block to {md_path}")

    structural_pass = (
        quantile_pass and jensen_pass
        and tightening_pass and consistency_pass)
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
