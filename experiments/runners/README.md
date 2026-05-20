# Experiment runners

Runners consume a single config path and produce a results directory.

## Contract

A runner:

1. Loads a YAML at `thesis/experiments/configs/<area>/<id>_<name>.yaml`.
2. Computes a results directory at `thesis/experiments/results/<area>/<id>_<name>__<config_sha>_<git_sha>/`.
3. Writes outputs into that directory.
4. Emits `manifest.json` alongside outputs with: config path, config SHA, git SHA, dirty flag, start/end timestamps, hardware (host, GPU model, ROS distro), runner version.

## Rules

- No parameters hard-coded in runner code. Every tunable lives in the config.
- A runner with uncommitted changes in tracked files writes `"dirty": true` into `manifest.json`. Do not silently ignore.
- Results are gitignored. Never delete a results directory manually — leave the audit trail.
- A runner that fails halfway still writes a `manifest.json` with `"status": "failed"` and the partial outputs.

## Skeleton

```python
# thesis/experiments/runners/<name>.py
import argparse, hashlib, json, pathlib, subprocess, time

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config", type=pathlib.Path)
    args = ap.parse_args()
    cfg_bytes = args.config.read_bytes()
    cfg_sha = hashlib.sha256(cfg_bytes).hexdigest()[:12]
    git_sha = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode().strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain"]).strip())
    name = args.config.stem
    area = args.config.parent.name
    out = pathlib.Path("thesis/experiments/results") / area / f"{name}__{cfg_sha}_{git_sha}"
    out.mkdir(parents=True, exist_ok=True)
    manifest = {"config": str(args.config), "config_sha": cfg_sha, "git_sha": git_sha,
                "dirty": dirty, "start": time.time(), "status": "running"}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    # ... run experiment, write outputs into `out` ...
    manifest["end"] = time.time(); manifest["status"] = "ok"
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))

if __name__ == "__main__":
    main()
```
