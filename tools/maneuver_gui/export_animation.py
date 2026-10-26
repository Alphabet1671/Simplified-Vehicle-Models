"""Bridge validated maneuver artifacts to the existing animation exporter."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
for _path in (REPOSITORY_ROOT, REPOSITORY_ROOT / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from simplified_vehicle_models.results.maneuver_run_record import write_maneuver_run_record
from tools.maneuver_gui.artifacts import load_artifact_evidence
from tools.anim import export_animation as animator


def _record_path(output: Path) -> Path:
    return output.with_name(f"{output.stem}.runrec")


def _definition(artifact: Path) -> dict:
    try:
        source, arrays, _channels = load_artifact_evidence(artifact)
    except (OSError, ValueError, KeyError) as error:
        raise ValueError(f"invalid artifact evidence: {error}") from error
    if source.get("execution_outcome") != "completed":
        raise ValueError("only completed maneuver artifacts may be exported")
    if source.get("model_kind") != "14dof-simple":
        raise ValueError("artifact must identify exact 14dof-simple model_kind")
    from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
    names = tuple(VerticalForwardPlant.state_names)
    time = arrays.get("time_s")
    states, rates = arrays.get("state"), arrays.get("state_rate")
    if time is None or states is None or rates is None or states.shape != (len(time), len(names)) or rates.shape != states.shape:
        raise ValueError("artifact must contain canonical 38-state samples and matching rates")
    return source["definition"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--output", type=Path, help="MP4 output path (required unless --png-sequence is used)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--png-sequence", type=Path)
    parser.add_argument("--frame-rate", type=animator.positive_float, default=120.)
    parser.add_argument("--width", type=animator.positive_even_int, default=1920)
    parser.add_argument("--height", type=animator.positive_even_int, default=1080)
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.png_sequence is None and args.output is None:
        parser.error("--output is required unless --png-sequence is used")
    if args.png_sequence is not None and args.output is not None:
        parser.error("--output cannot be combined with --png-sequence")
    output = args.output if args.png_sequence is None else args.png_sequence
    run_record = _record_path(args.output) if args.output is not None else args.png_sequence / "animation-run-record.npz"
    try:
        definition = _definition(args.artifact)
        if output.exists():
            raise ValueError(f"output destination already exists: {output}")
        if args.png_sequence is not None and output.is_dir():
            raise ValueError(f"PNG sequence destination already exists: {output}")
        if run_record.exists():
            raise ValueError(f"run record destination already exists: {run_record}")
        command = ["--record", str(run_record), "--output", str(args.output),
                   "--frame-rate", str(args.frame_rate), "--width", str(args.width),
                   "--height", str(args.height)]
        if args.png_sequence is not None:
            command = ["--record", str(run_record), "--png-sequence", str(args.png_sequence),
                       "--frame-rate", str(args.frame_rate), "--width", str(args.width),
                       "--height", str(args.height)]
        if args.dry_run:
            print(f"artifact: {args.artifact}")
            print(f"run record: {run_record}")
            print(f"output destination: {output}")
            print("animator command: python -m tools.anim.export_animation " + " ".join(command))
            return 0
        write_maneuver_run_record(run_record, definition, args.artifact)
        return animator.main(command)
    except (OSError, ValueError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
