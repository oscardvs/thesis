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


def verify_bag(bag_dir: pathlib.Path, required_topics: list[str],
               aliases: dict | None = None) -> dict[str, str]:
    """Verify the bag has each required topic (or an accepted alias).

    Returns a remap dict {actual_topic_in_bag: canonical_topic} for any aliased
    topic where the canonical name isn't in the bag but an alias is. Pass this
    to `ros2 bag play --remap` so downstream consumers see the canonical name.
    """
    if not bag_dir.exists():
        raise FileNotFoundError(f"bag dir not found: {bag_dir}")
    out = subprocess.check_output(["ros2", "bag", "info", str(bag_dir)]).decode()
    aliases = aliases or {}
    missing: list[str] = []
    remap: dict[str, str] = {}
    for canonical in required_topics:
        candidates = aliases.get(canonical, [canonical])
        present = [c for c in candidates if c in out]
        if not present:
            missing.append(canonical)
            continue
        # If the canonical name isn't in the bag but an alias is, remap.
        if canonical not in present:
            remap[present[0]] = canonical
    if missing:
        raise RuntimeError(
            f"bag missing required topics (no canonical or alias found): {missing}\n"
            f"--- ros2 bag info ---\n{out}"
        )
    return remap


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


# Keys in the sweep config under floor/ceiling that are NOT elevation node params
# (e.g. splitter knobs). Excluded from the elevation YAML overlay.
NON_ELEVATION_KEYS = {"z_low", "z_high"}


def _load_base_yaml(base_path: pathlib.Path, root_key: str) -> dict:
    if not base_path.exists():
        raise FileNotFoundError(f"base YAML not found: {base_path}")
    data = yaml.safe_load(base_path.read_text())
    try:
        return dict(data[root_key]["ros__parameters"])
    except (KeyError, TypeError):
        raise RuntimeError(
            f"base YAML {base_path} missing {root_key}.ros__parameters"
        )


def _overlay_params(base: dict, overrides: dict) -> dict:
    """Shallow overlay — every key in overrides wins, NON_ELEVATION_KEYS skipped."""
    merged = dict(base)
    for k, v in overrides.items():
        if k in NON_ELEVATION_KEYS:
            continue
        merged[k] = v
    return merged


def render_ceiling_yaml(template: dict, cell_size: float, edge_sharpen: bool,
                        out: pathlib.Path) -> pathlib.Path:
    base = _load_base_yaml(
        EM_PKG_SHARE / "config" / "setups" / "hilda" / "ceiling_complete.yaml",
        "ceiling_elevation_mapping_node",
    )
    overrides = dict(template["sweep"]["ceiling"])
    overrides["resolution"] = cell_size
    overrides["enable_edge_sharpen"] = edge_sharpen
    merged = _overlay_params(base, overrides)
    rendered = {"ceiling_elevation_mapping_node": {"ros__parameters": merged}}
    out.write_text(yaml.safe_dump(rendered, sort_keys=False))
    return out


def render_floor_yaml(template: dict, out: pathlib.Path) -> pathlib.Path:
    base = _load_base_yaml(
        EM_PKG_SHARE / "config" / "setups" / "hilda" / "floor_complete.yaml",
        "floor_elevation_mapping_node",
    )
    merged = _overlay_params(base, template["sweep"]["floor"])
    rendered = {"floor_elevation_mapping_node": {"ros__parameters": merged}}
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
            out_dir: pathlib.Path, drain_s: float = 5.0,
            bag_remap: dict[str, str] | None = None) -> dict:
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
            # Default is /dual_elevation_mapping_node/ceiling_map_raw; we run
            # two separate elevation nodes, so point at the ceiling node's topic.
            "-p", "gridmap_topic:=/ceiling_elevation_mapping_node/ceiling_map_raw",
            # Full-bag sampling — set high so the node never self-terminates;
            # finalize() runs on SIGINT at end-of-bag and writes the final JSON
            # from the last accumulated sample. The default 10 samples covers
            # only ~2 s of replay and misses most of the trajectory.
            "-p", "num_samples:=100000",
        ],
    }

    procs: dict[str, subprocess.Popen] = {}
    try:
        for name, cmd in cmds.items():
            procs[name] = popen_node(cmd, logs / f"{name}.log")
        time.sleep(8.0)  # node init + topic discovery

        # Bag path first; --clock is nargs='?' and would otherwise eat the bag path.
        play_cmd = ["ros2", "bag", "play", str(bag_path), "--clock"]
        for src, dst in (bag_remap or {}).items():
            play_cmd += ["--remap", f"{src}:={dst}"]
        bag_play = subprocess.Popen(
            play_cmd,
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

    # Collect metrics JSON if it landed. compute_ceiling_metrics partitions
    # cells into "all" (matched live ∩ GT) and "core" (interior, dilated away
    # from feature boundaries). Both are nested dicts in the JSON.
    summary: dict = {"label": label}
    if metrics_json.exists():
        m = json.loads(metrics_json.read_text())
        a = m.get("all", {}) or {}
        c = m.get("core", {}) or {}
        summary["coverage"] = a.get("coverage")
        summary["RMSE_all"] = a.get("rmse")
        summary["P95_all"] = a.get("p95")
        summary["max_all"] = a.get("max_error")
        summary["bias_all"] = a.get("bias")
        summary["n_all"] = a.get("n_compared")
        summary["RMSE_core"] = c.get("rmse")
        summary["P95_core"] = c.get("p95")
        summary["max_core"] = c.get("max_error")
        summary["bias_core"] = c.get("bias")
        summary["n_core"] = c.get("n_compared")
        # Derive boundary stats algebraically: SSE_all = SSE_core + SSE_boundary.
        # boundary = all − core (set-wise), so n_b = n_all − n_core.
        try:
            n_a, n_c_ = summary["n_all"], summary["n_core"]
            r_a, r_c_ = summary["RMSE_all"], summary["RMSE_core"]
            n_b = n_a - n_c_
            if n_b > 0 and r_a is not None and r_c_ is not None:
                sse_b = n_a * r_a * r_a - n_c_ * r_c_ * r_c_
                summary["RMSE_boundary"] = (sse_b / n_b) ** 0.5 if sse_b >= 0 else None
                summary["n_boundary"] = n_b
            else:
                summary["RMSE_boundary"] = None
                summary["n_boundary"] = n_b if n_b else 0
        except (TypeError, ValueError):
            summary["RMSE_boundary"] = None
            summary["n_boundary"] = None
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


def _fmt(v, prec=4):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{prec}f}"
    return str(v)


def write_summary(rows: list[dict], summary_csv: pathlib.Path, summary_md: pathlib.Path) -> None:
    if not rows:
        return
    keys = ["label", "cell_size", "enable_edge_sharpen", "status",
            "RMSE_all", "RMSE_core", "RMSE_boundary", "P95_all", "P95_core",
            "max_all", "max_core", "coverage", "bias_all", "bias_core",
            "n_all", "n_core", "n_boundary",
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
        "RMSE_all = full matched-cell RMSE; RMSE_core = interior cells (dilated away from",
        "feature edges); RMSE_boundary = derived algebraically from SSE_all − SSE_core.",
        "t_total_* are ceiling-node per-callback latency in ms.",
        "",
        "| cell | es | RMSE_all | RMSE_core | RMSE_boundary | cov% | bias | n_a | n_c | n_b | t_p50_c | t_p95_c |",
        "|------|----|----------|-----------|---------------|------|------|-----|-----|-----|---------|---------|",
    ]
    for r in rows:
        md.append(
            "| {cell} | {es} | {ra} | {rc} | {rb} | {cov} | {bias} | {na} | {nc} | {nb} | {tp50} | {tp95} |".format(
                cell=_fmt(r.get("cell_size"), 2),
                es="T" if r.get("enable_edge_sharpen") else "F",
                ra=_fmt(r.get("RMSE_all")),
                rc=_fmt(r.get("RMSE_core")),
                rb=_fmt(r.get("RMSE_boundary")),
                cov=_fmt(r.get("coverage"), 1),
                bias=_fmt(r.get("bias_all"), 3),
                na=_fmt(r.get("n_all")),
                nc=_fmt(r.get("n_core")),
                nb=_fmt(r.get("n_boundary")),
                tp50=_fmt(r.get("t_total_p50_ceiling_ms"), 2),
                tp95=_fmt(r.get("t_total_p95_ceiling_ms"), 2),
            )
        )
    # On/off comparison per cell size
    md += [
        "",
        "## Suppression contribution (RMSE_boundary, edge_sharpen on − off)",
        "",
        "| cell | RMSE_b(T) | RMSE_b(F) | Δ(T−F) | RMSE_c(T) | RMSE_c(F) | Δ(T−F) |",
        "|------|-----------|-----------|--------|-----------|-----------|--------|",
    ]
    by_cell: dict = {}
    for r in rows:
        by_cell.setdefault(r.get("cell_size"), {})[bool(r.get("enable_edge_sharpen"))] = r
    for cs in sorted(by_cell.keys()):
        t = by_cell[cs].get(True, {})
        f = by_cell[cs].get(False, {})
        rb_t, rb_f = t.get("RMSE_boundary"), f.get("RMSE_boundary")
        rc_t, rc_f = t.get("RMSE_core"), f.get("RMSE_core")
        d_b = (rb_t - rb_f) if (rb_t is not None and rb_f is not None) else None
        d_c = (rc_t - rc_f) if (rc_t is not None and rc_f is not None) else None
        md.append(
            f"| {_fmt(cs, 2)} | {_fmt(rb_t)} | {_fmt(rb_f)} | {_fmt(d_b)} | "
            f"{_fmt(rc_t)} | {_fmt(rc_f)} | {_fmt(d_c)} |"
        )
    md += [
        "",
        "## Interpretation",
        "",
        "Per README.md §Expected pattern: if suppression fires materially at a given cell",
        "size, `RMSE_boundary[T] < RMSE_boundary[F]` (negative Δ). A non-negative Δ at",
        "cell=0.10 m would indicate the count-thresholded rule is not contributing at",
        "production resolution — see the `wall_num_thresh` follow-up experiment.",
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

    # Bag verification (with alias support — bag may contain /pointcloud/fused_points
    # or /perception/fused_points; runner remaps to the canonical name at replay).
    bag_src = cfg["bag"]["source"]
    bag_path = pathlib.Path(bag_src.split(":", 1)[1] if bag_src.startswith("reuse_path:") else bag_src)
    required = cfg["bag"]["topics"]
    aliases = cfg["bag"].get("topic_aliases", {})
    bag_remap: dict[str, str] = {}
    if not args.dry_run:
        bag_remap = verify_bag(bag_path, required, aliases)
        if bag_remap:
            print(f"bag remap at replay: {bag_remap}")

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
            z_low, z_high, run_out, bag_remap=bag_remap,
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
