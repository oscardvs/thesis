#!/usr/bin/env python3
"""Rebuild sweep_summary.{csv,md} from existing per-cell outputs in a result dir.

Used to recover summaries when the runner's in-flight write failed, or when an
older result dir was produced by a runner whose summary structure has since
been superseded.

Per ADR 0008, the canonical source of a run's parameters is `summary.json`'s
`run_overrides` field. For older result dirs (pre-2026-05-22) the run-overrides
field is absent; this script falls back to reconstructing the per-run param
dict from the per-cell directory name by short-encoding lookup
(cs → cell_size, es → enable_edge_sharpen, wnt → wall_num_thresh, …).

Usage:
  python3 experiments/runners/sim_validation_01_postprocess.py \\
      experiments/results/<area>/<run_dir>
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


# Inverse of sim_validation_01_joint_sweep._LABEL_ENCODERS. Each entry maps a
# short prefix to (axis_name, decoder). The decoder takes the post-prefix
# string and returns the typed value.
_LABEL_DECODERS = {
    "cs": ("cell_size", lambda s: int(s) / 100.0),
    "es": ("enable_edge_sharpen", lambda s: bool(int(s))),
    "wnt": ("wall_num_thresh", lambda s: int(s)),
}
_LABEL_SEGMENT_RE = re.compile(r"^([a-zA-Z]+)(.+)$")


def parse_label(label: str) -> dict:
    """Decode a per-run label (e.g. 'cs015_wnt020_es1') back to a param dict.

    Unknown prefixes are skipped silently — the caller treats them as missing
    axes rather than failing the read.
    """
    out: dict = {}
    for seg in label.split("_"):
        m = _LABEL_SEGMENT_RE.match(seg)
        if not m:
            continue
        prefix, rest = m.group(1), m.group(2)
        spec = _LABEL_DECODERS.get(prefix)
        if spec is None:
            continue
        name, decoder = spec
        try:
            out[name] = decoder(rest)
        except (TypeError, ValueError):
            continue
    return out


def collect_row(cell_dir: pathlib.Path) -> dict:
    """Build one row for write_summary from a cell directory.

    Source priority:
      1. summary.json → its `run_overrides` field (current runner output).
      2. summary.json with top-level cell_size / enable_edge_sharpen
         (older runner output; reconstruct run_overrides from those fields).
      3. metrics_diagnostic.json + label-decoded run_overrides (the fallback
         when the per-cell summary.json is missing).
    """
    label = cell_dir.name
    summary_path = cell_dir / "summary.json"
    diag_path = cell_dir / "metrics_diagnostic.json"

    row: dict = {"label": label}
    run_overrides: dict = {}

    if summary_path.exists():
        existing = json.loads(summary_path.read_text())
        # Copy everything from the existing summary (metrics, timing, etc.)
        row.update(existing)
        row["label"] = label
        if "run_overrides" in existing and isinstance(existing["run_overrides"], dict):
            run_overrides = dict(existing["run_overrides"])
        else:
            # Older summary.json: top-level axis keys without run_overrides nest.
            for k in ("cell_size", "enable_edge_sharpen", "wall_num_thresh"):
                if k in existing:
                    run_overrides[k] = existing[k]

    # Fall back / supplement from the label if we still don't know the axes.
    if not run_overrides:
        run_overrides = parse_label(label)

    # If summary.json was missing, repopulate metrics from metrics_diagnostic.json.
    if not summary_path.exists():
        if not diag_path.exists():
            row["status"] = "no_metrics_json"
            row["run_overrides"] = run_overrides
            for k, v in run_overrides.items():
                row.setdefault(k, v)
            return row
        m = json.loads(diag_path.read_text())
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

    row["run_overrides"] = run_overrides
    for k, v in run_overrides.items():
        row.setdefault(k, v)
    return row


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result_dir", type=pathlib.Path)
    args = ap.parse_args()
    if not args.result_dir.is_dir():
        sys.exit(f"not a directory: {args.result_dir}")

    # Cell dirs are subdirectories with a parseable label and at least one of
    # summary.json / metrics_diagnostic.json present.
    cells = sorted(
        d for d in args.result_dir.iterdir()
        if d.is_dir() and (
            (d / "summary.json").exists() or (d / "metrics_diagnostic.json").exists()
        )
        and d.name != "rendered_configs"
        and d.name != "logs"
    )
    if not cells:
        sys.exit(f"no cell subdirs found under {args.result_dir}")

    rows = [collect_row(d) for d in cells]
    # Sort by run_overrides for stable table order (cell_size, then any other axis).
    rows.sort(key=lambda r: tuple(
        (r.get("run_overrides") or {}).get(k, 0)
        for k in ("cell_size", "wall_num_thresh", "enable_edge_sharpen")
    ))

    write_summary(rows, args.result_dir / "sweep_summary.csv",
                  args.result_dir / "sweep_summary.md")
    # Keep per-cell summary.json in sync with the new structure.
    for d, r in zip(cells, rows):
        (d / "summary.json").write_text(json.dumps(r, indent=2))
    print(f"wrote {args.result_dir}/sweep_summary.{{csv,md}}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
