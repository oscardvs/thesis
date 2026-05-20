#!/usr/bin/env python3
"""sim_validation_01 — joint sweep of ceiling cell size × enable_edge_sharpen.

See thesis/experiments/configs/sim_validation_01/README.md for design.

Per (cell_size, enable_edge_sharpen) the runner:
  1. Ensures a ground-truth grid exists at the requested resolution.
  2. Renders a per-run ceiling-instance YAML by overriding the matrix axes.
  3. Launches splitter + floor + ceiling + compute_ceiling_metrics as
     subprocess.Popens in their own process group.
  4. Replays the recorded bag with --clock.
  5. Waits for the metrics node to exit (it self-terminates when it has
     accumulated enough samples to evaluate) or for the bag to finish + drain.
  6. Sends SIGINT to the process group, collects timing CSVs and the metrics
     JSON, writes a per-run subdirectory.
After the matrix completes, a sweep_summary.csv and sweep_summary.md compare
the on/off contribution per cell size — the falsifiable prediction in README.md.

Usage:
  cd ~/ros2_ws/src/thesis
  python3 experiments/runners/sim_validation_01_joint_sweep.py \\
      experiments/configs/sim_validation_01/joint_sweep.yaml [--dry-run]
"""

import argparse
import csv
import hashlib
import json
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import time
from itertools import product

import yaml


WORKSPACE = pathlib.Path.home() / "ros2_ws"
EM_PKG_SHARE = WORKSPACE / "install" / "elevation_mapping_cupy" / "share" / "elevation_mapping_cupy"
EM_SRC = WORKSPACE / "src" / "elevation_mapping_cupy" / "elevation_mapping_cupy"


def sha12(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:12]


def git_state(repo: pathlib.Path) -> tuple[str, bool]:
    head = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "--short", "HEAD"]).decode().strip()
    dirty = bool(subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain"]).strip())
    return head, dirty


def verify_bag(bag_dir: pathlib.Path, required_topics: list[str]) -> None:
    if not bag_dir.exists():
        raise FileNotFoundError(f"bag dir not found: {bag_dir}")
    out = subprocess.check_output(["ros2", "bag", "info", str(bag_dir)]).decode()
    missing = [t for t in required_topics if t not in out]
    if missing:
        raise RuntimeError(f"bag missing required topics: {missing}\n--- ros2 bag info ---\n{out}")


def extract_ground_truth(cell_size: float, world_name: str, out_path: pathlib.Path) -> None:
    if out_path.exists():
        return
    extract_script = EM_SRC / "scripts" / "extract_ceiling_ground_truth.py"
    world = next(WORKSPACE.glob(f"src/hilda_ros/**/{world_name}"), None)
    if world is None:
        raise FileNotFoundError(f"world {world_name} not found under {WORKSPACE}/src/hilda_ros")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.check_call([
        "python3", str(extract_script),
        "--world", str(world),
        "--resolution", str(cell_size),
        "--output", str(out_path),
    ])


def render_ceiling_yaml(template: dict, cell_size: float, edge_sharpen: bool,
                        out: pathlib.Path) -> pathlib.Path:
    p = dict(template["sweep"]["ceiling"])
    p["resolution"] = cell_size
    p["map_length"] = template["sweep"]["ceiling"]["map_length"]
    p["enable_edge_sharpen"] = edge_sharpen
    p["wall_num_thresh"] = template["sweep"]["ceiling"]["wall_num_thresh"]
    # Required upstream params we always want set on the ceiling instance.
    for k in ("enable_visibility_cleanup", "enable_drift_compensation",
              "enable_overlap_clearance", "ramped_height_range_a",
              "ramped_height_range_b", "ramped_height_range_c"):
        p[k] = template["sweep"]["ceiling"][k]
    rendered = {"ceiling_elevation_mapping_node": {"ros__parameters": p}}
    out.write_text(yaml.safe_dump(rendered, sort_keys=False))
    return out


def render_floor_yaml(template: dict, out: pathlib.Path) -> pathlib.Path:
    p = dict(template["sweep"]["floor"])
    rendered = {"floor_elevation_mapping_node": {"ros__parameters": p}}
    out.write_text(yaml.safe_dump(rendered, sort_keys=False))
    return out


def popen_node(cmd: list[str], log_path: pathlib.Path) -> subprocess.Popen:
    log = log_path.open("w")
    return subprocess.Popen(
        cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )


def kill_group(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def run_one(label: str, ceiling_yaml: pathlib.Path, floor_yaml: pathlib.Path,
            bag_path: pathlib.Path, gt_path: pathlib.Path,
            splitter_z_low: float, splitter_z_high: float,
            out_dir: pathlib.Path, drain_s: float = 5.0) -> dict:
    """Launch + replay + collect for one matrix cell."""
    out_dir.mkdir(parents=True, exist_ok=True)
    logs = out_dir / "logs"
    logs.mkdir(exist_ok=True)
    timing_floor = out_dir / "timing_floor.csv"
    timing_ceiling = out_dir / "timing_ceiling.csv"
    plot_path = out_dir / "metrics_diagnostic.png"
    metrics_json = plot_path.with_suffix(".json")

    weights = EM_PKG_SHARE / "config" / "core" / "weights.dat"
    floor_plugin = EM_PKG_SHARE / "config" / "core" / "plugin_config.yaml"
    ceiling_plugin = EM_PKG_SHARE / "config" / "setups" / "hilda" / "ceiling_plugin_config.yaml"

    cmds = {
        "splitter": [
            "ros2", "run", "elevation_mapping_cupy", "ceiling_pointcloud_splitter.py",
            "--ros-args",
            "-p", "use_sim_time:=true",
            "-p", "input_topic:=/perception/fused_points",
            "-p", f"z_low:={splitter_z_low}",
            "-p", f"z_high:={splitter_z_high}",
        ],
        "floor": [
            "ros2", "run", "elevation_mapping_cupy", "elevation_mapping_node.py",
            "--ros-args",
            "-r", "__node:=floor_elevation_mapping_node",
            "--params-file", str(floor_yaml),
            "-p", "use_sim_time:=true",
            "-p", f"weight_file:={weights}",
            "-p", f"plugin_config_file:={floor_plugin}",
            "-p", f"benchmark_csv_path:={timing_floor}",
        ],
        "ceiling": [
            "ros2", "run", "elevation_mapping_cupy", "elevation_mapping_node.py",
            "--ros-args",
            "-r", "__node:=ceiling_elevation_mapping_node",
            "--params-file", str(ceiling_yaml),
            "-p", "use_sim_time:=true",
            "-p", f"weight_file:={weights}",
            "-p", f"plugin_config_file:={ceiling_plugin}",
            "-p", f"benchmark_csv_path:={timing_ceiling}",
        ],
        "metrics": [
            "ros2", "run", "elevation_mapping_cupy", "compute_ceiling_metrics.py",
            "--ros-args",
            "-p", "use_sim_time:=true",
            "-p", f"ground_truth_path:={gt_path}",
            "-p", f"diagnostic_plot_path:={plot_path}",
        ],
    }

    procs: dict[str, subprocess.Popen] = {}
    try:
        for name, cmd in cmds.items():
            procs[name] = popen_node(cmd, logs / f"{name}.log")
        time.sleep(8.0)  # node init + topic discovery

        bag_play = subprocess.Popen(
            ["ros2", "bag", "play", "--clock", str(bag_path)],
            stdout=(logs / "bag_play.log").open("w"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        deadline = time.time() + 600
        while time.time() < deadline:
            if bag_play.poll() is not None:
                break
            if procs["metrics"].poll() is not None:
                break
            time.sleep(0.5)

        # Drain
        time.sleep(drain_s)
    finally:
        for name in ("metrics", "ceiling", "floor", "splitter"):
            if name in procs:
                kill_group(procs[name])
        try:
            if "bag_play" in locals():
                kill_group(bag_play)
        except NameError:
            pass

    # Collect metrics JSON if it landed.
    summary: dict = {"label": label}
    if metrics_json.exists():
        m = json.loads(metrics_json.read_text())
        summary["coverage"] = m.get("coverage")
        summary["RMSE_all"] = m.get("rmse")
        summary["bias_all"] = m.get("bias")
        core = m.get("core", {})
        boundary_pseudo = {k: m.get(k) for k in ("rmse", "max_error", "p95")}
        summary["RMSE_core"] = core.get("rmse")
        summary["P95_core"] = core.get("p95")
        summary["max_core"] = core.get("max_error")
        # The "all" metrics include both core + boundary; the "boundary" delta
        # is RMSE_all weighted toward edges (compute_ceiling_metrics partitions
        # only core; the rest are interpreted as boundary-influenced).
        summary["RMSE_all_minus_core"] = (
            (summary["RMSE_all"] or 0) - (summary["RMSE_core"] or 0)
        )
        summary["status"] = "ok"
    else:
        summary["status"] = "no_metrics_json"

    # Latency from timing CSV (compact).
    for tag, path in (("floor", timing_floor), ("ceiling", timing_ceiling)):
        if path.exists() and path.stat().st_size > 0:
            with path.open() as f:
                reader = csv.DictReader(f)
                totals = [float(r["t_total_ms"]) for r in reader if r.get("t_total_ms")]
            if totals:
                totals.sort()
                p50 = totals[len(totals) // 2]
                p95 = totals[int(len(totals) * 0.95)]
                summary[f"t_total_p50_{tag}_ms"] = round(p50, 3)
                summary[f"t_total_p95_{tag}_ms"] = round(p95, 3)
                summary[f"n_callbacks_{tag}"] = len(totals)

    return summary


def write_summary(rows: list[dict], summary_csv: pathlib.Path, summary_md: pathlib.Path) -> None:
    if not rows:
        return
    keys = ["label", "cell_size", "enable_edge_sharpen", "status",
            "RMSE_all", "RMSE_core", "RMSE_all_minus_core", "P95_core",
            "max_core", "coverage", "bias_all",
            "t_total_p50_floor_ms", "t_total_p95_floor_ms", "n_callbacks_floor",
            "t_total_p50_ceiling_ms", "t_total_p95_ceiling_ms", "n_callbacks_ceiling"]
    with summary_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})

    md = [
        "# sim_validation_01 joint sweep — summary",
        "",
        "Columns: RMSE_all = full-map RMSE; RMSE_core = interior cells only; ",
        "RMSE_all_minus_core ≈ boundary-influenced contribution; t_total_* are p50/p95 ms.",
        "",
        "| cell | edge_sharpen | RMSE_all | RMSE_core | Δboundary | cov | t_p50_c | t_p95_c | n_c |",
        "|------|--------------|----------|-----------|-----------|-----|---------|---------|-----|",
    ]
    for r in rows:
        md.append(
            f"| {r.get('cell_size','?')} | {r.get('enable_edge_sharpen','?')} | "
            f"{r.get('RMSE_all','?')} | {r.get('RMSE_core','?')} | "
            f"{r.get('RMSE_all_minus_core','?')} | {r.get('coverage','?')} | "
            f"{r.get('t_total_p50_ceiling_ms','?')} | {r.get('t_total_p95_ceiling_ms','?')} | "
            f"{r.get('n_callbacks_ceiling','?')} |"
        )
    md += [
        "",
        "## Interpretation",
        "",
        "Compare `RMSE_all_minus_core` (≈ boundary contribution) for `edge_sharpen=true` vs",
        "`false` at each cell size. Suppression is firing materially where the on/off delta is",
        "non-trivial. See README.md §Expected pattern for the falsifiable prediction.",
    ]
    summary_md.write_text("\n".join(md) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("config", type=pathlib.Path)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    repo_root = pathlib.Path(__file__).resolve().parents[2]
    cfg_sha = sha12(args.config)
    git_sha, dirty = git_state(repo_root)
    out = (repo_root / "experiments" / "results" / args.config.parent.name
           / f"{args.config.stem}__{cfg_sha}_{git_sha}")
    out.mkdir(parents=True, exist_ok=True)

    manifest = {
        "config": str(args.config), "config_sha": cfg_sha, "git_sha": git_sha,
        "dirty": dirty, "start": time.time(), "status": "running",
        "host": subprocess.check_output(["hostname"]).decode().strip(),
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))

    # Bag verification
    bag_src = cfg["bag"]["source"]
    bag_path = pathlib.Path(bag_src.split(":", 1)[1] if bag_src.startswith("reuse_path:") else bag_src)
    required = cfg["bag"]["topics"]
    if not args.dry_run:
        verify_bag(bag_path, required)

    # GT extraction per cell size
    gt_dir = pathlib.Path(cfg["ground_truth"]["output_dir"])
    gt_paths: dict[float, pathlib.Path] = {}
    for cs in cfg["sweep"]["matrix"]["cell_size"]:
        gt_paths[cs] = gt_dir / f"gt_cs{int(cs * 100):03d}.npy"
        if not args.dry_run:
            extract_ground_truth(cs, cfg["sim_world"], gt_paths[cs])

    # Render floor YAML once (production lock)
    rendered_dir = out / "rendered_configs"
    rendered_dir.mkdir(exist_ok=True)
    floor_yaml = render_floor_yaml(cfg, rendered_dir / "floor.yaml")

    z_low = cfg["sweep"]["ceiling"]["z_low"]
    z_high = cfg["sweep"]["ceiling"]["z_high"]

    rows: list[dict] = []
    for cell_size, edge_sharpen in product(
        cfg["sweep"]["matrix"]["cell_size"],
        cfg["sweep"]["matrix"]["enable_edge_sharpen"],
    ):
        label = f"cs{int(cell_size * 100):03d}_es{int(edge_sharpen)}"
        ceiling_yaml = render_ceiling_yaml(
            cfg, cell_size, edge_sharpen, rendered_dir / f"{label}.yaml",
        )
        run_out = out / label

        if args.dry_run:
            print(f"[dry-run] {label}: ceiling_yaml={ceiling_yaml} gt={gt_paths[cell_size]} -> {run_out}")
            continue

        print(f"\n=== {label} ===")
        result = run_one(
            label, ceiling_yaml, floor_yaml, bag_path, gt_paths[cell_size],
            z_low, z_high, run_out,
        )
        result["cell_size"] = cell_size
        result["enable_edge_sharpen"] = edge_sharpen
        rows.append(result)
        (run_out / "summary.json").write_text(json.dumps(result, indent=2))

    if not args.dry_run:
        write_summary(rows, out / "sweep_summary.csv", out / "sweep_summary.md")

    manifest["end"] = time.time()
    manifest["status"] = "ok" if rows or args.dry_run else "no_runs"
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))

    print(f"\nresults: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
