#!/usr/bin/env python3
"""sim_validation sweep runner — generic N-axis matrix over the ceiling YAML.

Originally built for sim_validation_01 (cell_size × enable_edge_sharpen).
Generalised on 2026-05-22 per ADR 0008: `sweep.matrix` is an axis-name →
list-of-values dict; `sweep.extra_runs` is a list of explicit per-run override
dicts appended after the Cartesian product. The runner is sim-validation-shaped
(launches splitter + floor + ceiling + compute_ceiling_metrics against a bag);
experiment-specific commentary lives in per-experiment postprocess scripts.

Per matrix row + per extra_runs entry the runner:
  1. Ensures a ground-truth grid exists at the run's cell_size (one extraction
     per unique cell_size across all runs — pre-deduplicated).
  2. Renders a per-run ceiling-instance YAML by overlaying the run's axes onto
     the base ceiling_complete.yaml (cell_size aliases to YAML key `resolution`).
  3. Launches splitter + floor + ceiling + compute_ceiling_metrics as
     subprocess.Popens in their own process group.
  4. Replays the recorded bag with --clock (+ --remap if `bag.topic_aliases`).
  5. Waits for the metrics node to self-terminate, or for the bag to finish + drain.
  6. Sends SIGINT to the process group, collects timing CSVs + metrics JSON,
     writes a per-run subdirectory with summary.json (incl. run_overrides).

After all runs complete, a sweep_summary.csv + sweep_summary.md compare the
per-run metrics; the on/off block triggers automatically when `enable_edge_sharpen`
is an axis with at least one (T, F) pair after grouping by the other axes.

Usage:
  cd ~/ros2_ws/src/thesis
  python3 experiments/runners/sim_validation_01_joint_sweep.py \\
      experiments/configs/<experiment>/<config>.yaml [--dry-run]
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


# Short-encoding map for per-run labels. Unknown axes fall back to f"{key}{value}".
# See ADR 0008 for the rationale (avoid regex maintenance trap; postprocess
# scripts read run_overrides from summary.json rather than parsing labels).
_LABEL_ENCODERS = {
    "cell_size": lambda v: f"cs{int(v * 100):03d}",
    "enable_edge_sharpen": lambda v: f"es{int(bool(v))}",
    "wall_num_thresh": lambda v: f"wnt{int(v):03d}",
    # snf005 = 0.005, snf050 = 0.05, snf500 = 0.5 (×1000 for sub-unit values).
    "sensor_noise_factor": lambda v: f"snf{int(round(v * 1000)):03d}",
    # tv00000 = 0, tv00100 = 0.0001, tv01000 = 0.001, tv10000 = 0.01 (×1e6 padded).
    "time_variance": lambda v: f"tv{int(round(v * 1e6)):05d}",
}


def encode_label(run_overrides: dict) -> str:
    parts = []
    for k in sorted(run_overrides.keys()):
        enc = _LABEL_ENCODERS.get(k)
        parts.append(enc(run_overrides[k]) if enc else f"{k}{run_overrides[k]}")
    return "_".join(parts)


def expand_matrix(sweep: dict) -> list[dict]:
    """Expand `sweep["matrix"]` (axis name → list of values) into the Cartesian
    product, then append any `sweep["extra_runs"]` items unchanged.

    Each returned item is a dict of axis-name → value, ready to be passed to
    render_ceiling_yaml as run_overrides.
    """
    matrix = sweep.get("matrix", {}) or {}
    axis_names = list(matrix.keys())
    runs: list[dict] = []
    if axis_names:
        for values in product(*[matrix[a] for a in axis_names]):
            runs.append(dict(zip(axis_names, values)))
    for extra in sweep.get("extra_runs", []) or []:
        runs.append(dict(extra))
    return runs


def render_ceiling_yaml(template: dict, run_overrides: dict,
                        out: pathlib.Path) -> pathlib.Path:
    base = _load_base_yaml(
        EM_PKG_SHARE / "config" / "setups" / "hilda" / "ceiling_complete.yaml",
        "ceiling_elevation_mapping_node",
    )
    overrides = dict(template["sweep"]["ceiling"])
    # Apply run-specific axis values. `cell_size` aliases to the ceiling YAML's
    # `resolution`; every other key is taken as a direct YAML param name.
    for k, v in run_overrides.items():
        if k == "cell_size":
            overrides["resolution"] = v
        else:
            overrides[k] = v
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
        # Additive variance block (None for older metric tools).
        summary["variance"] = m.get("variance")
        summary["variance_core"] = m.get("variance_core")
        # Additive variance-aware ε block (None if hilda_clearance_field
        # is not installed in the workspace — the metric script's optional
        # import path gracefully degrades to null).
        summary["epsilon"] = m.get("epsilon")
        summary["epsilon_core"] = m.get("epsilon_core")
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


def _fmt_axis_value(v) -> str:
    if isinstance(v, bool):
        return "T" if v else "F"
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


def write_summary(rows: list[dict], summary_csv: pathlib.Path,
                  summary_md: pathlib.Path) -> None:
    """Write a per-run table + (if applicable) an on/off comparison block.

    The output is matrix-shape-agnostic — the axes that appear in the per-run
    `run_overrides` dict drive the columns. The on/off block triggers whenever
    `enable_edge_sharpen` appears as an axis and at least one (T, F) pair exists
    when grouping by the other axes. Experiment-specific interpretation lives
    in per-experiment postprocess scripts (ADR 0008).
    """
    if not rows:
        return

    # Discover which axes vary across the runs (preserve a stable order).
    axis_order: list[str] = []
    seen: set[str] = set()
    for r in rows:
        for k in (r.get("run_overrides") or {}):
            if k not in seen:
                seen.add(k)
                axis_order.append(k)

    fixed_keys = [
        "status", "RMSE_all", "RMSE_core", "RMSE_boundary",
        "P95_all", "P95_core", "max_all", "max_core",
        "coverage", "bias_all", "bias_core",
        "n_all", "n_core", "n_boundary",
        "t_total_p50_floor_ms", "t_total_p95_floor_ms", "n_callbacks_floor",
        "t_total_p50_ceiling_ms", "t_total_p95_ceiling_ms", "n_callbacks_ceiling",
    ]
    csv_keys = ["label"] + axis_order + fixed_keys
    with summary_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=csv_keys)
        w.writeheader()
        for r in rows:
            ov = r.get("run_overrides") or {}
            row = {k: r.get(k, "") for k in fixed_keys}
            row["label"] = r.get("label", "")
            for a in axis_order:
                row[a] = ov.get(a, "")
            w.writerow(row)

    md = [
        "# sweep summary",
        "",
        "RMSE_all = full matched-cell RMSE; RMSE_core = interior cells (dilated away",
        "from feature edges); RMSE_boundary = derived algebraically from SSE_all −",
        "SSE_core. t_total_* are ceiling-node per-callback latency in ms.",
        "",
    ]
    header_axes = " | ".join(axis_order) if axis_order else ""
    head_left = "label | " + (header_axes + " | " if axis_order else "")
    md.append(
        "| " + head_left
        + "RMSE_all | RMSE_core | RMSE_boundary | cov% | bias | n_a | n_c | n_b | t_p50_c | t_p95_c |"
    )
    md.append("|" + "---|" * (1 + len(axis_order) + 10))
    for r in rows:
        ov = r.get("run_overrides") or {}
        axis_cells = [_fmt_axis_value(ov.get(a, "")) for a in axis_order]
        md.append(
            "| " + " | ".join(
                [r.get("label", "?")]
                + axis_cells
                + [
                    _fmt(r.get("RMSE_all")),
                    _fmt(r.get("RMSE_core")),
                    _fmt(r.get("RMSE_boundary")),
                    _fmt(r.get("coverage"), 1),
                    _fmt(r.get("bias_all"), 3),
                    _fmt(r.get("n_all")),
                    _fmt(r.get("n_core")),
                    _fmt(r.get("n_boundary")),
                    _fmt(r.get("t_total_p50_ceiling_ms"), 2),
                    _fmt(r.get("t_total_p95_ceiling_ms"), 2),
                ]
            ) + " |"
        )

    # On/off comparison block — triggers whenever enable_edge_sharpen is an axis
    # and at least one group has both T and F. Groups by the remaining axes.
    if "enable_edge_sharpen" in axis_order:
        other_axes = [a for a in axis_order if a != "enable_edge_sharpen"]
        groups: dict[tuple, dict[bool, dict]] = {}
        for r in rows:
            ov = r.get("run_overrides") or {}
            key = tuple(ov.get(a) for a in other_axes)
            groups.setdefault(key, {})[bool(ov.get("enable_edge_sharpen"))] = r
        paired = sorted(
            [(k, td) for k, td in groups.items() if True in td and False in td],
            key=lambda kv: tuple((v if v is not None else 0) for v in kv[0]),
        )
        if paired:
            md += [
                "",
                "## Suppression contribution (RMSE_boundary / RMSE_core, edge_sharpen on − off)",
                "",
            ]
            header_axes_md = " | ".join(other_axes) if other_axes else "group"
            md.append(
                "| " + header_axes_md
                + " | RMSE_b(T) | RMSE_b(F) | Δ_b(T−F) | RMSE_c(T) | RMSE_c(F) | Δ_c(T−F) |"
            )
            md.append("|" + "---|" * (max(len(other_axes), 1) + 6))
            for key, td in paired:
                t, f = td[True], td[False]
                rb_t, rb_f = t.get("RMSE_boundary"), f.get("RMSE_boundary")
                rc_t, rc_f = t.get("RMSE_core"), f.get("RMSE_core")
                d_b = (rb_t - rb_f) if (rb_t is not None and rb_f is not None) else None
                d_c = (rc_t - rc_f) if (rc_t is not None and rc_f is not None) else None
                key_cells = [_fmt_axis_value(v) for v in key] or ["—"]
                md.append(
                    "| " + " | ".join(
                        key_cells + [_fmt(rb_t), _fmt(rb_f), _fmt(d_b),
                                     _fmt(rc_t), _fmt(rc_f), _fmt(d_c)]
                    ) + " |"
                )

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

    # Expand the matrix (Cartesian product of axes) and append extra_runs.
    # Each `run_overrides` is a dict of axis-name → value applied to the ceiling
    # YAML (with cell_size → resolution alias). See ADR 0008.
    runs = expand_matrix(cfg["sweep"])
    if not runs:
        raise RuntimeError(
            f"no runs to execute — `sweep.matrix` empty and no `sweep.extra_runs` "
            f"in {args.config}"
        )

    # GT extraction: one grid per unique cell_size across both the matrix and
    # any extra_runs (pre-deduplicate to avoid re-extracting or mis-keying).
    gt_dir = pathlib.Path(cfg["ground_truth"]["output_dir"])
    unique_cell_sizes = sorted({r["cell_size"] for r in runs if "cell_size" in r})
    gt_paths: dict[float, pathlib.Path] = {}
    for cs in unique_cell_sizes:
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
    for run_overrides in runs:
        if "cell_size" not in run_overrides:
            raise RuntimeError(
                f"run is missing required `cell_size` key: {run_overrides}"
            )
        label = encode_label(run_overrides)
        ceiling_yaml = render_ceiling_yaml(
            cfg, run_overrides, rendered_dir / f"{label}.yaml",
        )
        run_out = out / label

        if args.dry_run:
            print(
                f"[dry-run] {label}: ceiling_yaml={ceiling_yaml} "
                f"gt={gt_paths[run_overrides['cell_size']]} overrides={run_overrides} -> {run_out}"
            )
            continue

        print(f"\n=== {label} ===")
        result = run_one(
            label, ceiling_yaml, floor_yaml, bag_path,
            gt_paths[run_overrides["cell_size"]],
            z_low, z_high, run_out, bag_remap=bag_remap,
        )
        result["label"] = label
        result["run_overrides"] = dict(run_overrides)
        # Top-level mirrors for backward-compat consumers (CSV column readers).
        for k, v in run_overrides.items():
            result.setdefault(k, v)
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
