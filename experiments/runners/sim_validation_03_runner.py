#!/usr/bin/env python3
"""sim_validation_03 runner — standalone acados OCP prototype (module 03).

Consumes one experiment YAML (ocp / scene / gates blocks), builds the synthetic
beam field + the OCP, runs the closed loop, evaluates the falsifiable gates, and
writes a results dir per experiments/runners/README.md. Pure Python (no ROS
nodes, no bag). Solve-time is dev-machine descriptive (NOT the Fig-5 Orin proof).

acados codegen is isolated by chdir-ing into the (gitignored) per-config results
dir before build_ocp, so separate experiments never reuse each other's compiled
solver (same model name hilda_ocp6 would otherwise collide in a shared CWD).

Usage (needs venv + workspace overlay so hilda_nmpc + hilda_clearance_field import):
  source ~/ros2_ws/.venv-acados/acados_env.sh
  source ~/ros2_ws/install/setup.bash
  cd ~/ros2_ws/src/thesis
  python3 experiments/runners/sim_validation_03_runner.py \
      experiments/configs/sim_validation_03a/transit.yaml
"""
import argparse
import hashlib
import json
import os
import pathlib
import subprocess
import time

import numpy as np

from hilda_nmpc.config import load_experiment
from hilda_nmpc.scenarios import build_beam_field, retraction_feasibility
from hilda_nmpc.ocp import build_ocp
from hilda_nmpc.closed_loop import run_closed_loop
from hilda_nmpc.gates import evaluate_gates


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("config", type=pathlib.Path)
    args = ap.parse_args()

    config = args.config.resolve()                      # absolute, before any chdir
    repo_root = pathlib.Path(__file__).resolve().parents[2]   # thesis repo root
    cfg_sha = hashlib.sha256(config.read_bytes()).hexdigest()[:12]
    git_sha = subprocess.check_output(
        ["git", "-C", str(repo_root), "rev-parse", "--short", "HEAD"]).decode().strip()
    dirty = bool(subprocess.check_output(
        ["git", "-C", str(repo_root), "status", "--porcelain"]).strip())
    out = (repo_root / "experiments" / "results" / config.parent.name
           / f"{config.stem}__{cfg_sha}_{git_sha}")
    out.mkdir(parents=True, exist_ok=True)

    manifest = {
        "config": str(config), "config_sha": cfg_sha, "git_sha": git_sha,
        "dirty": dirty, "start": time.time(), "status": "running",
        "host": subprocess.check_output(["hostname"]).decode().strip(),
        "runner_version": "1",
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))

    try:
        ocp_cfg, scene, gate_cfg = load_experiment(config)
        feas = retraction_feasibility(scene, ocp_cfg.u_s_max)
        if gate_cfg.scene_kind == "controller" and not feas["feasible"]:
            raise RuntimeError(
                f"scene not comfortably feasible (must_lower={feas['must_lower']}, "
                f"t_beam={feas['time_to_beam_s']:.2f}s vs t_retract="
                f"{feas['retraction_time_s']:.2f}s) — the beam tests the wrong thing")

        field = build_beam_field(scene)
        os.chdir(out)                                   # isolate acados codegen here
        solver = build_ocp(field, ocp_cfg, scene, str(out / "ocp.json"))
        rec = run_closed_loop(solver, field, scene, ocp_cfg)
        verdict = evaluate_gates(rec, scene, gate_cfg)

        np.savez(out / "record.npz", **rec)
        (out / "verdict.json").write_text(
            json.dumps({**verdict, "feasibility_check": feas}, indent=2))
        st = np.asarray(rec["solve_times"])
        (out / "solve_time_hist.csv").write_text(
            "\n".join(["solve_time_s"] + [f"{t:.6f}" for t in st]) + "\n")

        manifest["end"] = time.time()
        manifest["status"] = "ok"
        manifest["verdict"] = verdict["verdict"]
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
        print(f"{verdict['verdict']}  ->  {out}")
        print(json.dumps(verdict, indent=2))
        return 0
    except Exception as exc:                            # fail-loud, keep audit trail
        manifest["end"] = time.time()
        manifest["status"] = "failed"
        manifest["error"] = repr(exc)
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
        raise


if __name__ == "__main__":
    raise SystemExit(main())
