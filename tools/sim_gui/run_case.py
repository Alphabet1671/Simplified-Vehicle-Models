"""Run one GUI study case: the runner child with the case's variant vehicle."""
from __future__ import annotations
import argparse, dataclasses, json, sys
from pathlib import Path


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--case-dir", type=Path, required=True)
    known, rest = pre.parse_known_args(argv)
    case = json.loads((known.case_dir / "case.json").read_text())
    import tools.run.run_iem24_forward_driver as runner
    args = runner.parse_args(rest)
    if args.child_attempt is None:
        raise SystemExit("run_case.py runs only as a child: pass --child-attempt")
    spec = runner.MODELS[args.model]
    runner.MODELS[args.model] = dataclasses.replace(
        spec, formulation=runner.canonical_path(known.case_dir / case["formulation"]),
        vehicle=runner.canonical_path(known.case_dir / case["vehicle"]))
    return runner.child(args)


if __name__ == "__main__":
    raise SystemExit(main())
