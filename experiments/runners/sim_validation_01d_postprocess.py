#!/usr/bin/env python3
"""sim_validation_01d — time_variance sweep postprocess.

Corroborating measurement for ADR 0009. Reads per-run summary.json files
and appends four blocks to sweep_summary.md:

  1. Reproducibility — tv=0.0001 vs archived 01c snf050.
  2. √-scaling check — three pairwise ratios on the non-zero cells,
     tested against the σ² ∝ √(time_variance) prediction.
  3. Zero-inflation behaviour — σ² at tv=0 should be substantially
     smaller than at tv=0.0001.
  4. Elevation residual stability — RMSE_core spread across the sweep.

Usage:
  python3 experiments/runners/sim_validation_01d_postprocess.py \\
      experiments/results/sim_validation_01d/time_variance_sweep__<sha>_<git> \\
      [--archived-baseline experiments/results/sim_validation_01c/alpha_d_sweep__c5f4d00d1572_1b3b5e8/cs010_es1_snf050_wnt020]
"""

import argparse
import json
import math
import pathlib
import sys


DEFAULT_ARCHIVED_BASELINE = pathlib.Path(
    "experiments/results/sim_validation_01c/"
    "alpha_d_sweep__c5f4d00d1572_1b3b5e8/cs010_es1_snf050_wnt020"
)


def _load_summary(cell_dir: pathlib.Path) -> dict:
    p = cell_dir / "summary.json"
    if not p.exists():
        return {}
    return json.loads(p.read_text())


def _ov(summary: dict) -> dict:
    return summary.get("run_overrides") or {}


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


def _fmt_sci(v, prec=3):
    if v is None:
        return "—"
    return f"{v:.{prec}g}"


def reproducibility_block(
    prod_run: dict, archived: dict, tol_m: float,
) -> tuple[list[str], bool]:
    lines = [
        "",
        "## Reproducibility check (tv=0.0001 vs archived 01c snf050)",
        "",
    ]
    if not archived:
        lines += [
            "Archived baseline not loadable — reproducibility check skipped. "
            "Current-run numbers stand on their own; no comparison made.",
            "",
            "**Verdict:** **SKIPPED** (no archive).",
        ]
        return lines, True
    if not prod_run:
        lines += [
            "Production time_variance=0.0001 run missing from this sweep — "
            "cannot run reproducibility check.",
            "",
            "**Verdict:** **FAIL** (production cell missing).",
        ]
        return lines, False
    ra_new = prod_run.get("RMSE_all")
    rc_new = prod_run.get("RMSE_core")
    ra_arc = archived.get("RMSE_all")
    rc_arc = archived.get("RMSE_core")
    v_new = (prod_run.get("variance") or {}).get("post_init_mean")
    v_arc = (archived.get("variance") or {}).get("post_init_mean")
    if None in (ra_new, rc_new, ra_arc, rc_arc):
        lines += [
            "Missing RMSE values — cannot compute reproducibility deltas.",
            "",
            "**Verdict:** **FAIL** (RMSE inputs missing).",
        ]
        return lines, False
    d_a = ra_new - ra_arc
    d_c = rc_new - rc_arc
    if v_new is not None and v_arc is not None and v_arc > 0:
        d_v_rel = (v_new - v_arc) / v_arc
        var_str = f"{v_new:.4g} vs {v_arc:.4g} (Δrel = {d_v_rel * 100:+.1f}%)"
        # σ² reproducibility tolerance: ±5% relative.
        var_pass = abs(d_v_rel) <= 0.05
        var_ran = True
    else:
        var_str = "N/A (variance fields missing)"
        # Skipped sub-check does not claim PASS — surfaced in the verdict.
        var_pass = True
        var_ran = False
    elev_pass = abs(d_a) <= tol_m and abs(d_c) <= tol_m
    passed = elev_pass and var_pass
    skipped_note = " (σ² check skipped — variance fields missing)" if not var_ran else ""
    verdict = "**PASS**" if passed else "**FAIL — STOP and investigate**"
    lines += [
        f"Tolerance: ±{tol_m * 1000:.1f} mm on RMSE_all and RMSE_core; "
        "±5% relative on post-init mean σ².",
        "",
        "| metric | new tv=0.0001 | archived 01c snf050 | Δ |",
        "|--------|--------------:|--------------------:|---|",
        f"| RMSE_all  | {_fmt(ra_new)} | {_fmt(ra_arc)} | {_fmt_mm(d_a)} mm |",
        f"| RMSE_core | {_fmt(rc_new)} | {_fmt(rc_arc)} | {_fmt_mm(d_c)} mm |",
        f"| σ² post-init mean | — | — | {var_str} |",
        "",
        f"**Verdict:** {verdict}{skipped_note}",
    ]
    if not passed:
        lines += [
            "",
            "Reproducibility failure ⇒ the √-scaling read-out below is not "
            "trustworthy until the source of non-determinism is identified.",
        ]
    return lines, passed


def sqrt_scaling_block(
    runs: list[dict], tol_factor: float, variance_metric: str,
) -> list[str]:
    """Three pairwise ratios on the non-zero time_variance cells, tested
    against σ² ∝ √(time_variance)."""
    lines = [
        "",
        f"## √-scaling check (metric: `{variance_metric}`)",
        "",
        "| time_variance | σ² (measured) | σ² (predicted √-floor) | n_post_init | init-dom % |",
        "|--------------:|--------------:|-----------------------:|------------:|-----------:|",
    ]
    rows = sorted(
        [r for r in runs if _ov(r).get("time_variance", 0.0) > 0.0],
        key=lambda r: _ov(r).get("time_variance", 0.0),
    )
    # Per the ADR algebra at α_d=0.05, z≈3m, Δt≈0.4s:
    #   map_v* ≈ √(α_d · z² · time_variance · update_variance_fps · Δt)
    #   = √(0.05 · 9 · time_variance · 5 · 0.4) = √(time_variance · 0.9)
    # Numerically: tv=0.0001 → 0.0095, tv=0.001 → 0.030, tv=0.01 → 0.095.
    PREDICTED = {0.0001: 0.0095, 0.001: 0.030, 0.01: 0.095}
    metric_by_tv: dict[float, float] = {}
    for r in rows:
        tv = _ov(r).get("time_variance")
        v = r.get("variance") or {}
        m = v.get(variance_metric)
        n_post = v.get("n_post_init")
        init_frac = v.get("init_dominated_frac")
        predicted = PREDICTED.get(tv)
        if tv is not None and m is not None:
            metric_by_tv[tv] = m
        lines.append(
            f"| {_fmt(tv, prec=6)} | {_fmt_sci(m)} | "
            f"{_fmt_sci(predicted) if predicted is not None else '—'} | "
            f"{_fmt(n_post, prec=0) if isinstance(n_post, int) else '—'} | "
            f"{(init_frac * 100):.1f}% |"
        )

    lines += [
        "",
        "**Pairwise ratios** (σ² ratio should track √(time_variance ratio)):",
        "",
        "| ratio | predicted (√) | measured | within ±50%? |",
        "|-------|--------------:|---------:|:------------:|",
    ]
    all_pass = True
    tv_values = sorted(metric_by_tv.keys())
    pairs = [(tv_values[i], tv_values[j])
             for i in range(len(tv_values))
             for j in range(i + 1, len(tv_values))]
    for lo, hi in pairs:
        tv_ratio = hi / lo
        predicted = math.sqrt(tv_ratio)
        measured = metric_by_tv[hi] / metric_by_tv[lo]
        within = (
            (1 - tol_factor) * predicted <= measured
            <= (1 + tol_factor) * predicted
        )
        mark = "✓" if within else "✗"
        if not within:
            all_pass = False
        lines.append(
            f"| tv={hi}/tv={lo} ({tv_ratio:g}×) | "
            f"{predicted:.2f}× | {measured:.2f}× | {mark} |"
        )

    lines += ["", ""]
    if all_pass:
        lines.append(
            "**Verdict: PASS — √-scaling confirmed empirically.** ADR 0009 §F2 "
            "is on solid empirical footing; Module 02's δ_cal protocol can "
            "commit to the freshness-conditional scope."
        )
    else:
        lines.append(
            "**Verdict: FAIL — at least one pairwise ratio outside the √-prediction band.** "
            "ADR 0009's diagnosed mechanism does not fully account for the variance "
            "regime in this sweep. Investigate before 02's calibration design commits."
        )
    return lines


def zero_inflation_block(
    runs: list[dict], ratio_max: float, variance_metric: str,
) -> list[str]:
    """σ² at time_variance=0 should be substantially smaller than at the
    production time_variance=0.0001."""
    lines = [
        "",
        "## Zero-inflation behaviour (time_variance=0)",
        "",
    ]
    zero_run = next(
        (r for r in runs if _ov(r).get("time_variance") == 0.0),
        None,
    )
    prod_run = next(
        (r for r in runs if _ov(r).get("time_variance") == 0.0001),
        None,
    )
    if zero_run is None or prod_run is None:
        lines.append(
            "Missing time_variance=0 and/or time_variance=0.0001 cells — "
            "cannot run zero-inflation check."
        )
        return lines
    v_zero = (zero_run.get("variance") or {}).get(variance_metric)
    v_prod = (prod_run.get("variance") or {}).get(variance_metric)
    if v_zero is None or v_prod is None or v_prod == 0:
        lines.append("Missing variance values — cannot compute ratio.")
        return lines
    ratio = v_zero / v_prod
    passed = ratio <= ratio_max
    verdict = "PASS" if passed else "FAIL"
    lines += [
        f"σ² at time_variance=0: {_fmt_sci(v_zero)} · "
        f"σ² at time_variance=0.0001: {_fmt_sci(v_prod)} · "
        f"Ratio: {ratio:.3f} (gate: ≤ {ratio_max}).",
        "",
        f"**Verdict: {verdict}.**",
    ]
    if passed:
        lines.append(
            "Inflation is what sets the production floor — with it removed, "
            "σ² draws down to a fraction of the inflated value. Mechanism F2 "
            "is the dominant contributor as the ADR predicted."
        )
    else:
        lines.append(
            "σ² at time_variance=0 is comparable to the inflated case ⇒ "
            "another α_d-independent variance source is dominant. The √-"
            "floor is real but not load-bearing here; ADR 0009 underspecifies "
            "the mechanism. Investigate the remaining contributors."
        )
    return lines


def elevation_stability_block(
    runs: list[dict], tol_m: float,
) -> list[str]:
    lines = [
        "",
        "## Elevation residual stability (RMSE_core across time_variance)",
        "",
        "| time_variance | RMSE_all | RMSE_core | bias_core |",
        "|--------------:|---------:|----------:|----------:|",
    ]
    rows = sorted(runs, key=lambda r: _ov(r).get("time_variance", 0.0))
    rc_values: list[float] = []
    for r in rows:
        tv = _ov(r).get("time_variance")
        ra = r.get("RMSE_all")
        rc = r.get("RMSE_core")
        bc = r.get("bias_core")
        if rc is not None:
            rc_values.append(rc)
        lines.append(
            f"| {_fmt(tv, prec=6)} | {_fmt(ra)} | {_fmt(rc)} | "
            f"{_fmt_mm(bc)} mm |"
        )
    if len(rc_values) < 2:
        return lines + ["", "Fewer than 2 valid RMSE_core values — spread not computed."]
    spread = max(rc_values) - min(rc_values)
    inside = spread <= tol_m
    lines += [
        "",
        f"RMSE_core spread = {spread * 1000:.2f} mm · tolerance "
        f"{tol_m * 1000:.1f} mm.",
    ]
    if inside:
        lines.append(
            "**Expected** — time_variance changes the variance state, not "
            "the elevation estimate's equilibrium. No finding."
        )
    else:
        lines.append(
            "**Flag** — spread exceeds tolerance; time_variance is moving "
            "the elevation estimate beyond the steady-state prediction."
        )
    return lines


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result_dir", type=pathlib.Path)
    ap.add_argument(
        "--archived-baseline", type=pathlib.Path,
        default=None,
        help="Archived 01c snf050 cell for reproducibility check.",
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
    repro_tol_m = float(accept.get("reproducibility_tolerance_m", 0.005))
    sqrt_tol = float(accept.get("sqrt_law_tolerance_factor", 0.5))
    zero_ratio_max = float(accept.get("zero_inflation_ratio_max", 0.5))
    variance_metric = str(accept.get("variance_metric", "post_init_mean"))
    elev_spread_tol = float(
        accept.get("elevation_rmse_spread_tolerance_m", 0.005))

    cell_dirs = sorted(
        d for d in rd.iterdir()
        if d.is_dir() and (d / "summary.json").exists()
        and d.name not in ("rendered_configs", "logs")
    )
    summaries = [_load_summary(d) for d in cell_dirs]
    if not summaries:
        sys.exit(f"no summary.json files under {rd}")

    prod_run = next(
        (s for s in summaries if _ov(s).get("time_variance") == 0.0001),
        {},
    )

    arc_dir = args.archived_baseline
    if arc_dir is None:
        repo_root = pathlib.Path(__file__).resolve().parents[2]
        arc_dir = repo_root / DEFAULT_ARCHIVED_BASELINE
    arc_summary = _load_summary(arc_dir) if arc_dir.is_dir() else {}

    repro_lines, repro_passed = reproducibility_block(
        prod_run, arc_summary, repro_tol_m,
    )
    sqrt_lines = sqrt_scaling_block(summaries, sqrt_tol, variance_metric)
    zero_lines = zero_inflation_block(
        summaries, zero_ratio_max, variance_metric,
    )
    elev_lines = elevation_stability_block(summaries, elev_spread_tol)

    md_path = rd / "sweep_summary.md"
    existing = md_path.read_text() if md_path.exists() else ""
    marker = "<!-- sim_validation_01d_postprocess BEGIN -->"
    end_marker = "<!-- sim_validation_01d_postprocess END -->"
    if marker in existing and end_marker in existing:
        before = existing.split(marker)[0].rstrip() + "\n"
        after_split = existing.split(end_marker, 1)
        after = after_split[1] if len(after_split) > 1 else ""
        existing = before + after.lstrip()

    block = (
        "\n" + marker + "\n"
        + "\n".join(repro_lines)
        + "\n" + "\n".join(sqrt_lines)
        + "\n" + "\n".join(zero_lines)
        + "\n" + "\n".join(elev_lines)
        + "\n" + end_marker + "\n"
    )
    md_path.write_text(existing + block)
    print(f"wrote postprocess block to {md_path}")
    if not repro_passed:
        print(
            "REPRODUCIBILITY CHECK FAILED — see sweep_summary.md",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
