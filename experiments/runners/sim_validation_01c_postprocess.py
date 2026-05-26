#!/usr/bin/env python3
"""sim_validation_01c — α_d (sensor_noise_factor) sweep postprocess.

Reads per-run summary.json files in a sim_validation_01c result directory
and appends three blocks to sweep_summary.md:

  1. Reproducibility check — α_d=0.05 cell vs archived corrected-sweep
     cs010_es1. Acceptance gate per the YAML's
     acceptance.reproducibility_tolerance_m.

  2. Variance response — ratio of post-init mean σ² at the two extreme
     α_d values against the [variance_ratio_min, variance_ratio_max] gate.
     Below ⇒ framework clamp / floor; above ⇒ additional variance source.
     Either is a finding for 02's δ_cal mechanism.

  3. Elevation residual stability — spread of RMSE_core across the three
     α_d runs vs elevation_rmse_spread_tolerance_m. Expected flat;
     reported for completeness.

Usage:
  python3 experiments/runners/sim_validation_01c_postprocess.py \\
      experiments/results/sim_validation_01c/alpha_d_sweep__<sha>_<git> \\
      [--archived-baseline experiments/results/sim_validation_01/joint_sweep__04a2e8802fc1_d57b109/cs010_es1]
"""

import argparse
import json
import pathlib
import sys


DEFAULT_ARCHIVED_BASELINE = pathlib.Path(
    "experiments/results/sim_validation_01/"
    "joint_sweep__04a2e8802fc1_d57b109/cs010_es1"
)

# Production α_d on both floor_complete.yaml and ceiling_complete.yaml.
PRODUCTION_ALPHA_D = 0.05


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
    """Build the reproducibility-check section and a pass/fail flag."""
    lines = [
        "",
        "## Reproducibility check (α_d=0.05 vs archived cs010_es1)",
        "",
    ]
    if not archived:
        lines.append(
            "Archived baseline not loadable — reproducibility check skipped. "
            "The α_d=0.05 numbers stand on their own."
        )
        return lines, True
    if not prod_run:
        lines.append(
            "Production α_d=0.05 run missing from this sweep — cannot run "
            "reproducibility check. Verify the sweep matrix includes α_d=0.05."
        )
        return lines, False
    ra_new = prod_run.get("RMSE_all")
    rc_new = prod_run.get("RMSE_core")
    ra_arc = archived.get("RMSE_all")
    rc_arc = archived.get("RMSE_core")
    if None in (ra_new, rc_new, ra_arc, rc_arc):
        lines.append(
            "Missing RMSE values — cannot compute reproducibility deltas. "
            "Check inputs manually."
        )
        return lines, False
    d_a = ra_new - ra_arc
    d_c = rc_new - rc_arc
    passed = abs(d_a) <= tol_m and abs(d_c) <= tol_m
    verdict = "PASS" if passed else "**FAIL — STOP and investigate**"
    lines += [
        f"Tolerance: ±{tol_m * 1000:.1f} mm on both RMSE_all and RMSE_core.",
        "",
        "| metric | new α_d=0.05 | archived cs010_es1 | Δ (mm) |",
        "|--------|-------------:|-------------------:|-------:|",
        f"| RMSE_all  | {_fmt(ra_new)} | {_fmt(ra_arc)} | {_fmt_mm(d_a)} |",
        f"| RMSE_core | {_fmt(rc_new)} | {_fmt(rc_arc)} | {_fmt_mm(d_c)} |",
        "",
        f"**Verdict:** {verdict}",
    ]
    if not passed:
        lines += [
            "",
            "Reproducibility failure ⇒ the variance-response read-out below "
            "is not trustworthy until non-determinism is identified. Likely "
            "culprits: TF buffer timing, GPU scheduling, framework state "
            "from prior runs, or a code change between the archive and now.",
        ]
    return lines, passed


def variance_response_block(
    runs: list[dict], ratio_min: float, ratio_max: float,
    variance_metric: str,
) -> list[str]:
    """Per-run variance summary + scaling-ratio check.

    `variance_metric` selects which key under `summary.variance` drives the
    headline ratio. Default is `post_init_mean` — cells whose σ² has
    dropped below `init_thresh`, excluding under-measured cells pinned
    near `initial_variance`.
    """
    lines = [
        "",
        f"## Variance response (metric: `{variance_metric}`)",
        "",
        "| α_d | n_post_init | init-dom % | mean σ² | post-init mean σ² | post-init p50 |",
        "|----:|------------:|-----------:|--------:|------------------:|--------------:|",
    ]
    rows = sorted(runs, key=lambda r: _ov(r).get("sensor_noise_factor", 0.0))
    metric_by_alpha: dict[float, float] = {}
    for r in rows:
        a = _ov(r).get("sensor_noise_factor")
        v = r.get("variance") or {}
        post = v.get("post_init_mean")
        all_mean = v.get("mean")
        post_p50 = v.get("post_init_p50")
        n_post = v.get("n_post_init")
        init_frac = v.get("init_dominated_frac")
        if a is not None and v.get(variance_metric) is not None:
            metric_by_alpha[a] = v[variance_metric]
        lines.append(
            f"| {_fmt(a, prec=4)} | "
            f"{_fmt(n_post, prec=0) if isinstance(n_post, int) else '—'} | "
            f"{(init_frac * 100):.1f}% | "
            f"{_fmt_sci(all_mean)} | {_fmt_sci(post)} | {_fmt_sci(post_p50)} |"
        )

    if len(metric_by_alpha) < 2:
        lines += [
            "",
            "Fewer than 2 α_d values reported a valid `"
            f"{variance_metric}` — cannot compute scaling ratio.",
        ]
        return lines

    a_lo = min(metric_by_alpha)
    a_hi = max(metric_by_alpha)
    v_lo = metric_by_alpha[a_lo]
    v_hi = metric_by_alpha[a_hi]
    if v_lo <= 0:
        lines += [
            "",
            f"`{variance_metric}` at α_d={a_lo} is non-positive — cannot "
            "compute scaling ratio. Check the variance pipeline.",
        ]
        return lines
    alpha_ratio = a_hi / a_lo
    measured = v_hi / v_lo
    predicted = alpha_ratio  # linear scaling

    inside = ratio_min <= measured <= ratio_max
    if inside:
        verdict = (
            f"**PASS** — σ² scales linearly with α_d within the "
            f"[{ratio_min:g}, {ratio_max:g}] band (predicted {predicted:g}× "
            f"for an {alpha_ratio:g}× α_d span)."
        )
        implication = (
            "Implication for 02: the published variance pipeline is "
            "α_d-responsive as the model claims. δ_cal absorbs *residual* "
            "mis-specification on top of an α_d-driven σ², not the bulk of it."
        )
    elif measured < ratio_min:
        verdict = (
            f"**FAIL (below)** — measured ratio {measured:.2f}× is below "
            f"the lower gate {ratio_min:g}× for an {alpha_ratio:g}× α_d span "
            f"(predicted {predicted:g}×)."
        )
        implication = (
            "Implication for 02: the framework clamps or floors σ² "
            "independently of α_d (check `max_variance`, `initial_variance`, "
            "outlier-rejection paths). δ_cal would carry weight the LiDAR "
            "noise model has not justified — the calibration pre-floor is "
            "*not* α_d-driven in the regime tested."
        )
    else:
        verdict = (
            f"**FAIL (above)** — measured ratio {measured:.2f}× exceeds the "
            f"upper gate {ratio_max:g}× for an {alpha_ratio:g}× α_d span "
            f"(predicted {predicted:g}×)."
        )
        implication = (
            "Implication for 02: additional variance sources are blowing "
            "up at large α_d (drift coupling, fusion math non-linearity, "
            "max_variance saturation). α_d is not the full calibration "
            "knob — δ_cal would need an α_d-dependent component."
        )

    lines += [
        "",
        f"Mean σ² at α_d={a_lo}: {_fmt_sci(v_lo)} · "
        f"Mean σ² at α_d={a_hi}: {_fmt_sci(v_hi)} · "
        f"Ratio (high/low): {measured:.2f}× · "
        f"Predicted (linear): {predicted:g}×.",
        "",
        verdict,
        "",
        implication,
    ]
    return lines


def elevation_stability_block(
    runs: list[dict], tol_m: float,
) -> list[str]:
    """Spread of RMSE_core across α_d runs."""
    lines = [
        "",
        "## Elevation residual stability (RMSE_core across α_d)",
        "",
        "| α_d | RMSE_all | RMSE_core | bias_core |",
        "|----:|---------:|----------:|----------:|",
    ]
    rows = sorted(runs, key=lambda r: _ov(r).get("sensor_noise_factor", 0.0))
    rc_values: list[float] = []
    for r in rows:
        a = _ov(r).get("sensor_noise_factor")
        ra = r.get("RMSE_all")
        rc = r.get("RMSE_core")
        bc = r.get("bias_core")
        if rc is not None:
            rc_values.append(rc)
        lines.append(
            f"| {_fmt(a, prec=4)} | {_fmt(ra)} | {_fmt(rc)} | "
            f"{_fmt_mm(bc)} mm |"
        )
    if len(rc_values) < 2:
        lines.append("")
        lines.append("Fewer than 2 valid RMSE_core values — spread not computed.")
        return lines
    spread = max(rc_values) - min(rc_values)
    inside = spread <= tol_m
    lines += [
        "",
        f"RMSE_core spread across α_d = {spread * 1000:.2f} mm · "
        f"tolerance {tol_m * 1000:.1f} mm.",
    ]
    if inside:
        lines.append(
            "**Expected** — Kalman gain shifts under α_d but the equilibrium "
            "elevation estimate does not. No finding."
        )
    else:
        lines.append(
            "**Flag** — spread exceeds tolerance; α_d is moving the "
            "elevation estimate by more than the Kalman steady-state would "
            "predict. Investigate (likely: fusion math under large α_d, "
            "or under-measured cells dominating one of the runs)."
        )
    return lines


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result_dir", type=pathlib.Path)
    ap.add_argument(
        "--archived-baseline", type=pathlib.Path,
        default=None,
        help="Archived sim_validation_01 cs010_es1 dir for the reproducibility "
             "check (defaults to joint_sweep__04a2e8802fc1_d57b109/cs010_es1).",
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
    ratio_min = float(accept.get("variance_ratio_min", 50.0))
    ratio_max = float(accept.get("variance_ratio_max", 200.0))
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
        (s for s in summaries
         if _ov(s).get("sensor_noise_factor") == PRODUCTION_ALPHA_D),
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
    var_lines = variance_response_block(
        summaries, ratio_min, ratio_max, variance_metric,
    )
    elev_lines = elevation_stability_block(summaries, elev_spread_tol)

    md_path = rd / "sweep_summary.md"
    existing = md_path.read_text() if md_path.exists() else ""
    marker = "<!-- sim_validation_01c_postprocess BEGIN -->"
    end_marker = "<!-- sim_validation_01c_postprocess END -->"
    if marker in existing and end_marker in existing:
        before = existing.split(marker)[0].rstrip() + "\n"
        after_split = existing.split(end_marker, 1)
        after = after_split[1] if len(after_split) > 1 else ""
        existing = before + after.lstrip()

    block = (
        "\n" + marker + "\n"
        + "\n".join(repro_lines)
        + "\n" + "\n".join(var_lines)
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
