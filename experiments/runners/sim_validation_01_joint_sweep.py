#!/usr/bin/env python3
"""sim_validation_01 — joint sweep of ceiling cell size × enable_edge_sharpen.

See thesis/experiments/configs/sim_validation_01/README.md for the design.

This runner consumes one YAML at thesis/experiments/configs/sim_validation_01/
joint_sweep.yaml and produces a results directory with 8 sub-runs (4 cell sizes
× 2 enable_edge_sharpen states), each containing per-callback timing CSVs and
per-region accuracy JSON from compute_ceiling_metrics.py, plus a sweep summary.

Status: skeleton. The orchestration loop is wired up; the per-run launch /
metrics-collection blocks are stubs that print what they would do. Filling
those in is the next pass once the design decisions in README.md §"Open
decisions for the user" are resolved (bag source, floor lock, desktop vs
Jetson).
"""

import argparse
import csv
import hashlib
import json
import pathlib
import subprocess
import sys
import time
from itertools import product

import yaml


def sha12(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:12]


def git_state(repo: pathlib.Path) -> tuple[str, bool]:
    head = subprocess.check_output(
        ["git", "-C", str(repo), "rev-parse", "--short", "HEAD"]
    ).decode().strip()
    dirty = bool(subprocess.check_output(
        ["git", "-C", str(repo), "status", "--porcelain"]
    ).strip())
    return head, dirty


def render_per_run_yaml(template: dict, cell_size: float, edge_sharpen: bool,
                       out: pathlib.Path) -> pathlib.Path:
    """Render the ceiling-instance YAML for one (cell_size, edge_sharpen) cell."""
    ceiling_params = dict(template["sweep"]["ceiling"])
    ceiling_params["resolution"] = cell_size
    ceiling_params["enable_edge_sharpen"] = edge_sharpen
    rendered = {
        "ceiling_elevation_mapping_node": {
            "ros__parameters": ceiling_params,
        }
    }
    out.write_text(yaml.safe_dump(rendered, sort_keys=False))
    return out


def stub_run_one(label: str, cfg_path: pathlib.Path, bag_path: pathlib.Path,
                 gt_path: pathlib.Path, out_dir: pathlib.Path) -> dict:
    """Run one (cell_size, edge_sharpen) configuration end-to-end.

    Stub: prints what would happen. Real implementation:
      1. Launch hilda_dual_ceiling_mapping with the rendered cfg_path
      2. Start tegrastats logger (skip on desktop)
      3. Start compute_ceiling_metrics node with ground_truth_path=gt_path
         and result write hooks
      4. Replay bag with --clock
      5. Wait for bag end + drain window
      6. Tear down, collect timing CSVs from /tmp/, accuracy JSON from the
         metrics node, copy into out_dir
      7. Return summary dict (RMSE_core, RMSE_boundary, P95_boundary, t_total_p50,
         t_total_p95, fps, GPU_mem_peak_mb, coverage)
    """
    print(f"[stub] would launch: {label}")
    print(f"  config:    {cfg_path}")
    print(f"  bag:       {bag_path}")
    print(f"  gt:        {gt_path}")
    print(f"  out:       {out_dir}")
    return {
        "label": label,
        "RMSE_core": None,
        "RMSE_boundary": None,
        "P95_boundary": None,
        "t_total_p50_ms": None,
        "t_total_p95_ms": None,
        "fps": None,
        "GPU_mem_peak_mb": None,
        "coverage": None,
        "status": "stub",
    }


def write_summary(rows: list[dict], summary_csv: pathlib.Path,
                  summary_md: pathlib.Path) -> None:
    if not rows:
        return
    fieldnames = ["label", "cell_size", "enable_edge_sharpen", "RMSE_core",
                  "RMSE_boundary", "P95_boundary", "t_total_p50_ms",
                  "t_total_p95_ms", "fps", "GPU_mem_peak_mb", "coverage",
                  "status"]
    with summary_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k, "") for k in fieldnames})

    md_lines = [
        "# sim_validation_01 joint sweep — summary",
        "",
        "| cell_size | edge_sharpen | RMSE_core | RMSE_boundary | P95_boundary | t_p50 | t_p95 | fps | GPU_mem |",
        "|-----------|--------------|-----------|---------------|--------------|-------|-------|-----|---------|",
    ]
    for row in rows:
        md_lines.append(
            f"| {row['cell_size']} | {row['enable_edge_sharpen']} | "
            f"{row['RMSE_core']} | {row['RMSE_boundary']} | "
            f"{row['P95_boundary']} | {row['t_total_p50_ms']} | "
            f"{row['t_total_p95_ms']} | {row['fps']} | {row['GPU_mem_peak_mb']} |"
        )
    md_lines += [
        "",
        "## Interpretation",
        "",
        "Compare `RMSE_boundary[edge_sharpen=true]` vs `[edge_sharpen=false]` at each cell size.",
        "Suppression fires reliably at cell sizes where the on/off delta is non-trivial.",
        "If the production cell size (0.10 m) shows ≈0 delta, the suppression rule is not",
        "firing at production resolution and the construction's lowest-overhead-surface",
        "claim relies on the asymmetric-resolution recovery (coarser ceiling cells).",
        "",
        "See README.md §Expected pattern for the falsifiable prediction.",
    ]
    summary_md.write_text("\n".join(md_lines) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("config", type=pathlib.Path,
                    help="Path to joint_sweep.yaml")
    ap.add_argument("--dry-run", action="store_true",
                    help="Don't actually run; just print the plan")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    repo_root = pathlib.Path(__file__).resolve().parents[2]
    cfg_sha = sha12(args.config)
    git_sha, dirty = git_state(repo_root)

    area = args.config.parent.name           # 'sim_validation_01'
    name = args.config.stem                  # 'joint_sweep'
    out = repo_root / "experiments" / "results" / area / f"{name}__{cfg_sha}_{git_sha}"
    out.mkdir(parents=True, exist_ok=True)

    manifest = {
        "config": str(args.config),
        "config_sha": cfg_sha,
        "git_sha": git_sha,
        "dirty": dirty,
        "start": time.time(),
        "status": "running",
        "host": subprocess.check_output(["hostname"]).decode().strip(),
        "matrix_size": (
            len(cfg["sweep"]["matrix"]["cell_size"])
            * len(cfg["sweep"]["matrix"]["enable_edge_sharpen"])
        ),
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))

    # Bag resolution -- placeholder until the user resolves the source decision
    bag_source = cfg["bag"]["source"]
    if bag_source == "record_fresh":
        bag_path = pathlib.Path("/tmp/sim_validation_01_bag")
        if not args.dry_run and not bag_path.exists():
            print(f"[error] bag source set to 'record_fresh' but {bag_path} does not exist.")
            print(f"[error] run scripts/benchmark/benchmark_record.sh {cfg['bag']['duration_s']} first.")
            return 2
    elif bag_source.startswith("reuse_path:"):
        bag_path = pathlib.Path(bag_source.split(":", 1)[1])
    else:
        print(f"[error] unrecognised bag source: {bag_source}")
        return 2

    # Ground truth resolution per cell size -- placeholder
    gt_dir = pathlib.Path(cfg["ground_truth"]["output_dir"])
    if not args.dry_run:
        gt_dir.mkdir(parents=True, exist_ok=True)
        # TODO: invoke extract_ceiling_ground_truth.py per cell_size

    rows: list[dict] = []
    rendered_dir = out / "rendered_configs"
    rendered_dir.mkdir(exist_ok=True)

    for cell_size, edge_sharpen in product(
        cfg["sweep"]["matrix"]["cell_size"],
        cfg["sweep"]["matrix"]["enable_edge_sharpen"],
    ):
        label = f"cs{int(cell_size * 100)}_es{int(edge_sharpen)}"
        cfg_out = rendered_dir / f"{label}.yaml"
        render_per_run_yaml(cfg, cell_size, edge_sharpen, cfg_out)
        gt_path = gt_dir / f"gt_cs{int(cell_size * 100)}.npy"
        run_dir = out / label
        run_dir.mkdir(exist_ok=True)

        if args.dry_run:
            print(f"[dry-run] {label}: cfg={cfg_out} gt={gt_path} -> {run_dir}")
            continue

        result = stub_run_one(label, cfg_out, bag_path, gt_path, run_dir)
        result["cell_size"] = cell_size
        result["enable_edge_sharpen"] = edge_sharpen
        rows.append(result)

    if not args.dry_run:
        write_summary(rows, out / "sweep_summary.csv", out / "sweep_summary.md")

    manifest["end"] = time.time()
    manifest["status"] = "stub_complete" if any(r["status"] == "stub" for r in rows) else "ok"
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))

    print(f"\nresults: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
