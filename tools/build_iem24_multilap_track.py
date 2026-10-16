#!/usr/bin/env python3
"""Build a *separate* representative racing-line geometry from measured flying laps.

Each lap receives its own translational alignment to the existing closed track.
The existing path is used only to assign distance phase and resolve normal
position. A robust median of individually smoothed signed offsets defines a
new closed path. Never overwrite the original model with this hypothesis.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d

from tools.build_closed_track import arc_length, resample, signed_geometry, write_csv
from tools.compare_iem24_murphy_stint import (
    fit_position_offset, flying_lap_indices, load_table, project_onto_track,
)
from simplified_vehicle_models.tracks.profile import load_track


def robust_lap_selection(offsets: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Retain similar lines using median RMS deviation and 2.5 robust sigmas."""
    center = np.median(offsets, axis=0)
    rms = np.sqrt(np.mean((offsets - center) ** 2, axis=1))
    baseline = np.median(rms)
    threshold = baseline + 2.5 * 1.4826 * np.median(np.abs(rms - baseline))
    keep = rms <= max(threshold, baseline + 0.15)
    if keep.sum() < 3:
        raise ValueError("fewer than three consistent laps")
    return keep, rms


def periodic_lap_offset(phase: np.ndarray, offset: np.ndarray,
                        grid: np.ndarray, length: float, sigma_m: float) -> np.ndarray:
    """Interpolate a lap's signed normal offset then smooth across the seam."""
    order = np.argsort(phase)
    x, indices = np.unique(phase[order], return_index=True)
    y = offset[order][indices]
    if x.size < 30 or np.max(np.diff(np.r_[x, x[0] + length])) > 12.:
        raise ValueError("lap has a missing section longer than 12 m")
    # A residual integrated-velocity closure drift would otherwise make the
    # signed offset discontinuous exactly at the periodic seam.
    interpolated = np.interp(grid, np.r_[x[-30:] - length, x, x[:30] + length],
                             np.r_[y[-30:], y, y[:30]])
    return gaussian_filter1d(interpolated, sigma_m / (length / grid.size), mode="wrap")


def build(measured: dict[str, np.ndarray], path, *, offset_sigma_m: float = 3.,
          geometry_sigma_m: float = 3.) -> tuple[dict, dict]:
    global_offset, fit_rmse = fit_position_offset(measured, path)
    projected, residual = project_onto_track(measured, path, global_offset)
    grid = path.distance[:-1]
    old_xy = np.column_stack((path.x_north[:-1], path.y_east[:-1]))
    normal = np.column_stack((-np.sin(path.heading[:-1]), np.cos(path.heading[:-1])))
    lap_offsets, identifiers, translations = [], [], []
    for lap in flying_lap_indices(measured):
        selected = measured["lap_index"] == lap
        phase = projected[selected]
        base = np.column_stack((np.interp(phase, path.distance, path.x_north),
                                np.interp(phase, path.distance, path.y_east)))
        n = np.column_stack((-np.sin(np.interp(phase, path.distance, path.heading)),
                             np.cos(np.interp(phase, path.distance, path.heading))))
        sample = np.column_stack((measured["position_north_m"][selected],
                                  measured["position_east_m"][selected])) + global_offset
        correction = np.median(base - sample, axis=0)
        lateral = np.sum((sample + correction - base) * n, axis=1)
        try:
            values = periodic_lap_offset(phase, lateral, grid, path.lap_length, offset_sigma_m)
        except ValueError:
            continue
        identifiers.append(int(lap))
        translations.append(correction.tolist())
        lap_offsets.append(values)
    stack = np.stack(lap_offsets)
    keep, errors = robust_lap_selection(stack)
    representative = np.median(stack[keep], axis=0)
    raw_xy = old_xy + representative[:, None] * normal
    # Smoothing the XY points (not curvature) leaves a geometrically coherent
    # result: both heading and signed curvature come from the same path.
    dx = float(grid[1] - grid[0])
    smooth_xy = gaussian_filter1d(raw_xy, geometry_sigma_m / dx, axis=0, mode="wrap")
    # Reparameterize repeatedly: stored distance should be the new XY arclength.
    nodes = int(grid.size)
    distance, north, east = grid, smooth_xy[:, 0], smooth_xy[:, 1]
    for _ in range(3):
        cyclic_n = np.r_[north, north[0]]
        cyclic_e = np.r_[east, east[0]]
        accumulated = arc_length(cyclic_n, cyclic_e)
        distance, north, east = resample(accumulated, cyclic_n, cyclic_e,
                                         accumulated[-1] / nodes)
    heading, curvature = signed_geometry(distance, north, east)
    spacing = float(distance[1] - distance[0])
    geometry = {"distance": distance, "north": north, "east": east,
                "heading": heading, "curvature": curvature, "length": nodes * spacing}
    diagnostics = {
        "lap_ids": identifiers, "selected_lap_ids": [int(value) for value in np.asarray(identifiers)[keep]],
        "rms_deviation_from_median_m": errors.tolist(),
        "per_lap_translation_m": translations,
        "old_track_fit_rmse_m": fit_rmse,
        "old_track_projected_residual_p95_m": float(np.percentile(residual, 95)),
        "old_length_m": path.lap_length, "candidate_length_m": geometry["length"],
        "maximum_median_normal_offset_m": float(np.max(np.abs(representative))),
        "median_lap_spread_p95_m": float(np.percentile(np.std(stack[keep], axis=0), 95)),
        "smoothing_offset_m": offset_sigma_m, "smoothing_geometry_m": geometry_sigma_m,
        "signed_winding_rad": float(np.sum(curvature) * spacing),
        "minimum_radius_m": float(1. / np.max(np.abs(curvature))),
    }
    return geometry, diagnostics


def write_candidate_xml(destination: Path, csv_name: str, diagnostics: dict) -> None:
    # A distinct model id avoids representing a median driven line as the
    # surveyed physical track centreline.
    from xml.etree import ElementTree as ET
    root = ET.Element("model", {"schema-version": "1.0", "id": "iem24-multilap-driven-line-candidate",
                                "type": "track-model", "subtype": "closed-xy-profile"})
    metadata = ET.SubElement(root, "metadata")
    ET.SubElement(metadata, "title").text = "IEM24 median driven line (experimental)"
    ET.SubElement(root, "data", {"ref": csv_name})
    parameters = ET.SubElement(root, "parameters")
    for name, value in (("is_closed", "true"), ("terminal_segment_is_duplicate", "true"),
                        ("maximum_radius", "300"), ("node_spacing", ".5"),
                        ("smoothing_length", str(diagnostics["smoothing_geometry_m"]))):
        ET.SubElement(parameters, "parameter", {"name": name, "value": value})
    ET.indent(root, space="    ")
    ET.ElementTree(root).write(destination, encoding="UTF-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--measured", type=Path, default=Path("docs/validation/iem24-murphy/measured_stint.csv"))
    parser.add_argument("--track", type=Path, default=Path("models/tracks/2024_michigan_endurance_closed.xml"))
    parser.add_argument("--output", type=Path, default=Path("docs/validation/iem24-murphy/multilap_track_candidate"))
    options = parser.parse_args()
    path = load_track(options.track).path_coordinates
    if path is None:
        raise ValueError("reference track must have signed XY coordinates")
    geometry, diagnostics = build(load_table(options.measured), path)
    options.output.parent.mkdir(parents=True, exist_ok=True)
    write_csv(options.output.with_suffix(".csv"), geometry)
    write_candidate_xml(options.output.with_suffix(".xml"), options.output.with_suffix(".csv").name, diagnostics)
    options.output.with_name(options.output.name + "_diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2) + "\n")
    load_track(options.output.with_suffix(".xml"))
    print(json.dumps(diagnostics, indent=2))


if __name__ == "__main__":
    main()
