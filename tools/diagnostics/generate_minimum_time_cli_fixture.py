"""Create a fresh self-contained synthetic fixture for public spline CLI smoke runs.

Example (from repository root, pinned runtime):
  PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python \
    tools/diagnostics/generate_minimum_time_cli_fixture.py \
    --output runs/no_tv_reliability/task1A2a2_tiny_cli_repro

This creates a 32-segment, 10 m-radius closed circle and a compact ideal
capability chart whose provenance hash is computed from the selected planar
formulation. The chart is an integration fixture, not a validated production
capability surface. Existing output paths are never overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.point_mass_capability import (
    CapabilitySurface, capability_hash, save_capability_surface,
)

DEFAULT_FORMULATION = Path("models/formulations/iem24_murphy_real_planar_7dof.xml")


def build_fixture(output: Path, formulation: Path = DEFAULT_FORMULATION) -> dict:
    output = Path(output).resolve()
    formulation = Path(formulation).resolve()
    output.mkdir(parents=True, exist_ok=False)
    xml_path, csv_path, surface_path = output / "tiny_track.xml", output / "geometry.csv", output / "surface.npz"
    radius, segments = 10.0, 32
    theta = np.linspace(0.0, 2.0 * np.pi, segments + 1)
    distance = radius * theta
    with csv_path.open("x", encoding="utf-8") as stream:
        stream.write("distance,x_north,y_east,heading_rad,curvature\n")
        for s, angle in zip(distance, theta):
            stream.write(f"{s:.12g},{radius*np.sin(angle):.12g},{radius*(1-np.cos(angle)):.12g},{angle:.12g},{1/radius:.12g}\n")
    xml_path.write_text(
        '<?xml version="1.0"?><model schema-version="1.0" id="tiny" type="track-model" '
        'subtype="closed-xy-profile"><metadata><title>Tiny synthetic closed circle</title></metadata>'
        '<data ref="geometry.csv"/><parameters><parameter name="is_closed" value="true"/>'
        '<parameter name="terminal_segment_is_duplicate" value="true"/>'
        '<parameter name="maximum_radius" value="300"/></parameters></model>\n', encoding="utf-8")
    compiled = compile_formulation(formulation)
    vehicle = point_mass_view_of_planar(compiled.model)
    speed = np.array([0.0, 5.0, 10.0, 20.0, 30.0])
    curvature = np.array([-0.2, -0.1, 0.0, 0.1, 0.2])
    shape = (speed.size, curvature.size)
    chart = CapabilitySurface(speed, curvature, np.full(shape, 4.0), np.full(shape, -4.0),
                              np.full(curvature.size, 30.0), capability_hash(vehicle))
    save_capability_surface(chart, surface_path)
    return {
        "output": str(output), "formulation": str(formulation),
        "inputs": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                   for path in (xml_path, csv_path, surface_path)},
        "chart_provenance_hash": chart.provenance_hash,
        "chart_is_physical_validation": False,
        "geometry_parameters": {"radius_m": radius, "segments": segments},
        "chart_parameters": {"speeds_mps": speed.tolist(), "curvatures_per_m": curvature.tolist(),
                             "acceleration_max_mps2": 4.0, "acceleration_min_mps2": -4.0,
                             "speed_ceiling_mps": 30.0},
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--formulation", type=Path, default=DEFAULT_FORMULATION)
    args = parser.parse_args(argv)
    import json
    print(json.dumps(build_fixture(args.output, args.formulation), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
