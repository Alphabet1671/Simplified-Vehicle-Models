"""Build an exclusive opt-in signed transient capability archive.

The ``linear_program`` profile is sampled conservative evidence, not an
unrestricted physical minimum-time optimum.  It refuses a failed dense
original-physics validation rather than saving placeholders.
"""
from __future__ import annotations
import argparse
import hashlib
import os
from pathlib import Path
import sys
import numpy as np
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.signed_capability import build_signed_transient_capability, save_signed_transient_capability


def _sha256(path: Path) -> str:
    """Return a manifest hash of an input source file."""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formulation", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--curvature-limit", type=float, default=.24)
    parser.add_argument("--curvature-count", type=int, default=49)
    parser.add_argument("--u-count", type=int, default=17)
    parser.add_argument("--maximum-speed", type=float, default=None,
                        help="optional physical top speed; default uses the selected vehicle hardware limit")
    parser.add_argument("--interval-fit", choices=("linear_program", "common_interval"), default="linear_program")
    args=parser.parse_args(argv)
    if not np.isfinite(args.curvature_limit) or args.curvature_limit <= 0 or args.curvature_count < 2 or args.u_count < 2:
        parser.error("curvature limit must be finite and positive and both axis counts at least two")
    # Fail before compiling/scanning physics.  lexists also rejects a dangling
    # symlink, preserving exclusive output semantics without needless work.
    if os.path.lexists(args.output):
        parser.error(f"refusing to overwrite existing output: {args.output}")
    source_manifest = {"formulation_path": str(args.formulation.resolve()),
                       "formulation_sha256": _sha256(args.formulation)}
    vehicle=point_mass_view_of_planar(compile_formulation(str(args.formulation)).model)
    common = dict(curvature=np.linspace(-args.curvature_limit,args.curvature_limit,args.curvature_count),
        u=np.linspace(0.,1.,args.u_count), maximum_speed=args.maximum_speed,
        scan_points=81, refinement_steps=16, speed_scan_points=25,
        endpoint_buffer=.15, domain_backoff=1.)
    lp = dict(minimum_interval_width=.02, construction_subdivisions=2,
        validation_grid=(113,211), heldout_validation_grid=(107,199), cut_plane_attempts=2)
    chart=build_signed_transient_capability(vehicle, interval_fit=args.interval_fit,
        **(common | (lp if args.interval_fit == "linear_program" else {})))
    chart.build_settings["source_manifest"] = source_manifest
    save_signed_transient_capability(chart,args.output)
    print(f"wrote {args.output} representation={chart.representation} vehicle_physics_hash={chart.provenance_hash} source_sha256={source_manifest['formulation_sha256']} fit={chart.build_settings.get('interval_fit', 'common_interval')} LP={chart.validation.get('LP_status', 'not-applicable')}")
    return 0
if __name__ == "__main__":
    raise SystemExit(main())
