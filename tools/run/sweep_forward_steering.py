#!/usr/bin/env python3
"""Run a small deterministic steering tuning sweep for a canonical forward lap.

This utility orchestrates the existing isolated runner; it does not alter the
acceptance gates or declare a candidate usable. Outputs are separate attempts.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("planar-7dof", "14dof-simple"), default="planar-7dof")
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=float, default=600.)
    parser.add_argument("--preview", type=float, nargs="+", default=[0.15, 0.2, 0.25])
    parser.add_argument("--heading-gain", type=float, nargs="+", default=[1.5, 2.0, 2.5])
    parser.add_argument("--lateral-gain", type=float, nargs="+", default=[1.5, 2.0, 2.5])
    parser.add_argument("--maximum-runs", type=int, default=12)
    args = parser.parse_args(argv)
    configs = [(p, h, l) for p in args.preview for h in args.heading_gain for l in args.lateral_gain]
    if len(configs) > args.maximum_runs:
        parser.error(f"{len(configs)} candidates exceed --maximum-runs={args.maximum_runs}")
    args.output_root.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, (preview, heading, lateral) in enumerate(configs, 1):
        command = [sys.executable, "tools/run/run_iem24_forward_driver.py", "--model", args.model,
                   "--output-root", str(args.output_root), "--timeout-seconds", str(args.timeout_seconds),
                   "--preview-seconds", str(preview), "--heading-gain", str(heading),
                   "--lateral-gain", str(lateral)]
        completed = subprocess.run(command, check=False)
        attempts = sorted(args.output_root.glob("attempt_*/summary.json"), key=lambda p: p.stat().st_mtime)
        summary = json.loads(attempts[-1].read_text()) if attempts else {}
        row = {"candidate": index, "preview_seconds": preview, "heading_gain": heading,
               "lateral_gain": lateral, "returncode": completed.returncode,
               "attempt": str(attempts[-1].parent) if attempts else None,
               "status": summary.get("status"), "lap_completed": summary.get("lap_completed"),
               "max_lateral_path_error_m": summary.get("maximum_lateral_path_error_m"),
               "path_tracking_valid": summary.get("path_tracking_valid"), "usable": summary.get("usable")}
        rows.append(row)
        print(json.dumps(row, sort_keys=True), flush=True)
    result = {"model": args.model, "candidates": rows,
              "selection_rule": "diagnostic only; require completed lap and <= existing 0.25 m path gate; no other gates waived"}
    output = args.output_root / "steering_sweep.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
