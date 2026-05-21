#!/usr/bin/env python3
"""Rebuild sweep_summary.{csv,md} from existing per-cell metrics_diagnostic.json
and timing CSVs in a result dir. Used when the original runner had bugs in its
JSON-depth reads or its summary structure (pre-2026-05-21 results).

Usage:
  python3 experiments/runners/sim_validation_01_postprocess.py \\
      experiments/results/sim_validation_01/joint_sweep__<sha>_<git>
"""

import argparse
import csv
import json
import pathlib
import re
import sys

# Re-use the runner's summary writer so the format stays in sync.
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from sim_validation_01_joint_sweep import write_summary  # noqa: E402


CELL_RE = re.compile(r"^cs(\d{3})_es([01])$")


def collect_row(cell_dir: pathlib.Path) -> dict:
    m = CELL_RE.match(cell_dir.name)
    if not m:
        return {}
    cell_size = int(m.group(1)) / 100.0
    enable_edge_sharpen = bool(int(m.group(2)))
    row = {
        "label": cell_dir.name,
        "cell_size": cell_size,
        "enable_edge_sharpen": enable_edge_sharpen,
    }

    diag = cell_dir / "metrics_diagnostic.json"
    if not diag.exists():
        row["status"] = "no_metrics_json"
        return row
    m = json.loads(diag.read_text())
    a = m.get("all", {}) or {}
    c = m.get("core", {}) or {}
    row["coverage"] = a.get("coverage")
    row["RMSE_all"] = a.get("rmse")
    row["P95_all"] = a.get("p95")
    row["max_all"] = a.get("max_error")
    row["bias_all"] = a.get("bias")
    row["n_all"] = a.get("n_compared")
    row["RMSE_core"] = c.get("rmse")
    row["P95_core"] = c.get("p95")
    row["max_core"] = c.get("max_error")
    row["bias_core"] = c.get("bias")
    row["n_core"] = c.get("n_compared")

    n_a, n_c = row["n_all"], row["n_core"]
    r_a, r_c = row["RMSE_all"], row["RMSE_core"]
    if n_a and n_c and r_a is not None and r_c is not None and n_a > n_c:
        n_b = n_a - n_c
        sse_b = n_a * r_a * r_a - n_c * r_c * r_c
        row["RMSE_boundary"] = (sse_b / n_b) ** 0.5 if sse_b >= 0 else None
        row["n_boundary"] = n_b
    else:
        row["RMSE_boundary"] = None
        row["n_boundary"] = 0

    row["status"] = "ok"

    for tag in ("floor", "ceiling"):
        p = cell_dir / f"timing_{tag}.csv"
        if p.exists() and p.stat().st_size > 0:
            with p.open() as f:
                reader = csv.DictReader(f)
                totals = [float(r["t_total_ms"]) for r in reader if r.get("t_total_ms")]
            if totals:
                totals.sort()
                row[f"t_total_p50_{tag}_ms"] = round(totals[len(totals) // 2], 3)
                row[f"t_total_p95_{tag}_ms"] = round(totals[int(len(totals) * 0.95)], 3)
                row[f"n_callbacks_{tag}"] = len(totals)
    return row


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result_dir", type=pathlib.Path)
    args = ap.parse_args()
    if not args.result_dir.is_dir():
        sys.exit(f"not a directory: {args.result_dir}")

    cells = sorted(d for d in args.result_dir.iterdir()
                   if d.is_dir() and CELL_RE.match(d.name))
    if not cells:
        sys.exit(f"no cs*es* subdirs found in {args.result_dir}")

    rows = [collect_row(d) for d in cells]
    # Sort by (cell_size, edge_sharpen) for stable table ordering
    rows.sort(key=lambda r: (r.get("cell_size") or 0,
                             not r.get("enable_edge_sharpen", False)))

    write_summary(rows, args.result_dir / "sweep_summary.csv",
                  args.result_dir / "sweep_summary.md")
    # Also keep per-cell summary.json in sync with the new structure
    for d, r in zip(cells, rows):
        (d / "summary.json").write_text(json.dumps(r, indent=2))
    print(f"wrote {args.result_dir}/sweep_summary.{{csv,md}}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
