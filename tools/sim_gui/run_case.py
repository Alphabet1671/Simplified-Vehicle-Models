"""Run one point-mass GUI case through the existing forward runner.

Point-mass consumes a point-mass view of the selected vehicle; this remains
its Task 2 bridge until Task 3 formalizes adaptation.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path
import sys


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--case-dir", type=Path, required=True)
    known, rest = pre.parse_known_args(argv)
    case = json.loads((known.case_dir / "case.json").read_text())
    if case.get("solver_kind") != "point-mass":
        raise SystemExit("run_case.py accepts point-mass cases only")
    import tools.run.run_iem24_forward_driver as runner
    parsed = runner.parse_args(rest)
    if parsed.child_attempt is None:
        raise SystemExit("run_case.py runs only as a child: pass --child-attempt")
    if parsed.model not in runner.MODELS:
        raise SystemExit(f"unsupported selected source model: {parsed.model}")
    spec = runner.MODELS[parsed.model]
    runner.MODELS[parsed.model] = dataclasses.replace(
        spec, formulation=runner.canonical_path(known.case_dir / case["formulation"]),
        vehicle=runner.canonical_path(known.case_dir / case["vehicle"]))
    return runner.child(parsed)


if __name__ == "__main__":
    raise SystemExit(main())
