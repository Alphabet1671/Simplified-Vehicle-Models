"""Run the stage 1 minimum-time racing-line acceptance pipeline."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.point_mass_capability import capability_hash, load_capability_surface
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.solvers.qss_line_ocp import (
    QssSettings, build_mesh, capability_envelope_time, launch_time, solve_line,
    solve_ladder, track_sample,
)
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.tracks.corridor import Corridor, corridor_from_track
from simplified_vehicle_models.tracks.profile import load_track, _read_width_csv
from simplified_vehicle_models.tracks.racing_line import path_from_solution
from tools.gates.check_minimum_time_line import (
    check_corridor_containment, check_free_line_not_slower, check_mesh_convergence,
    check_replay_agreement, check_zero_corridor_matches_envelope,
)


def _validate_track_for_line_pipeline(track, half_width: float | None = None) -> None:
    """Reject inputs outside the track scope before solver work."""
    if track.has_edges and half_width is not None:
        raise ValueError("half_width is only valid for a track without edges")
    if not track.has_edges and half_width is None:
        raise ValueError("minimum-time line pipeline requires a lateral corridor or half_width")


def _validate_output_paths(source: Path, output: Path) -> None:
    """Reject generated artifacts that would overwrite source track assets."""
    root = ET.parse(source).getroot()
    generated = {
        (output / name).resolve()
        for name in (
            "line.csv", "racing_line.csv", "racing_line_track.csv",
            "racing_line_track.xml", "rebuilt_width.csv", "summary.json",
        )
    }
    protected = {source.resolve()}
    for element in root.findall("./data"):
        reference = element.get("ref")
        if not reference:
            continue
        source_asset = (source.parent / reference).resolve()
        protected.add(source_asset)
        if element.get("role") == "width":
            generated.add((output / f"{source_asset.stem}_rebuilt.csv").resolve())
    for candidate in generated:
        for source_asset in protected:
            same_file = candidate == source_asset
            if not same_file and candidate.exists() and source_asset.exists():
                same_file = candidate.samefile(source_asset)
            if same_file:
                raise ValueError(
                    f"output artifact would overwrite source track asset: {source_asset}"
                )


def _write_rebuilt_track(source: Path, output: Path, path) -> tuple[Path, Path]:
    """Write rebuilt geometry with source widths projected onto new normals."""
    _validate_output_paths(source, output)
    root = ET.parse(source).getroot()
    width_elements = root.findall("./data[@role='width']")
    if len(width_elements) > 1:
        raise ValueError("source track has multiple width-role data references")
    geometry_data = next((element for element in root.findall("./data") if element.get("role") is None), None)
    if geometry_data is None or not geometry_data.get("ref"):
        raise ValueError("source track has no unroled geometry data reference")
    csv_path = output / "racing_line_track.csv"
    xml_path = output / "racing_line_track.xml"
    output.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("distance", "x_north", "y_east", "heading_rad", "curvature"))
        for row in zip(path.distance, path.x_north, path.y_east, path.heading, path.curvature):
            writer.writerow((f"{float(v):.12g}" for v in row))
    geometry_data.set("ref", csv_path.name)
    source_track = load_track(source)
    if source_track.path_coordinates is None or path.distance.size != source_track.path_coordinates.distance.size:
        raise ValueError("rebuilt geometry must retain the source cross-section nodes")
    original = source_track.path_coordinates
    heading_delta = np.asarray(path.heading) - np.asarray(original.heading)
    heading_delta = (heading_delta + np.pi) % (2.0 * np.pi) - np.pi
    if not np.all(np.isfinite(heading_delta)) or np.max(np.abs(heading_delta)) > 0.25:
        raise ValueError("rebuilt path heading differs by more than 0.25 rad; width projection is unreliable")
    dn = np.asarray(path.x_north) - np.asarray(original.x_north)
    de = np.asarray(path.y_east) - np.asarray(original.y_east)

    parameters = root.find("./parameters")
    constant_element = None if parameters is None else parameters.find("./parameter[@name='half_width']")
    if not width_elements and constant_element is not None:
        half_width = float(constant_element.get("value"))
        left = np.full(path.distance.shape, half_width)
        right = np.full(path.distance.shape, half_width)
        width_name = "rebuilt_width.csv"
        width_data = None
    else:
        left = right = None
        width_data = None
        width_name = ""
    for element in width_elements:
        source_width = (source.parent / element.get("ref", "")).resolve()
        if not source_width.is_file():
            raise ValueError(f"source width CSV does not exist: {source_width}")
        _, left, right = _read_width_csv(source_width)
        if left.size != path.distance.size:
            raise ValueError("source widths and rebuilt geometry must have matching node counts")
        width_name = f"{source_width.stem}_rebuilt.csv"
        width_data = element
        break
    if left is not None:
        # Project the old cross-section edge vectors onto the rebuilt path's
        # normals. This preserves nodewise normal clearance as a small-angle
        # approximation, not the exact 2D boundary point. The heading guard above
        # limits the tangential component omitted by this representation.
        left_x = np.asarray(original.x_north) + left * np.sin(original.heading)
        left_y = np.asarray(original.y_east) - left * np.cos(original.heading)
        right_x = np.asarray(original.x_north) - right * np.sin(original.heading)
        right_y = np.asarray(original.y_east) + right * np.cos(original.heading)
        new_left_normal = np.column_stack((np.sin(path.heading), -np.cos(path.heading)))
        new_right_normal = np.column_stack((-np.sin(path.heading), np.cos(path.heading)))
        rebuilt_left = (left_x - path.x_north) * new_left_normal[:, 0] + (left_y - path.y_east) * new_left_normal[:, 1]
        rebuilt_right = (right_x - path.x_north) * new_right_normal[:, 0] + (right_y - path.y_east) * new_right_normal[:, 1]
        if np.any(rebuilt_left <= 0.0) or np.any(rebuilt_right <= 0.0):
            raise ValueError("racing line leaves nonpositive clearance to a source track edge")
        width_csv = output / width_name
        with width_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("distance", "half_width_left", "half_width_right"))
            for row in zip(path.distance, rebuilt_left, rebuilt_right):
                writer.writerow((f"{float(value):.12g}" for value in row))
        if width_data is None:
            if parameters is None:
                parameters = ET.SubElement(root, "parameters")
            parameters.remove(constant_element)
            width_data = ET.SubElement(root, "data", {"role": "width"})
        width_data.set("ref", width_csv.name)
    title = root.find("./metadata/title")
    if title is not None:
        title.text = f"{title.text or 'Track'} — rebuilt racing line"
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(xml_path, encoding="utf-8", xml_declaration=True)
    return xml_path, csv_path


def _write_line(path: Path, solution) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("distance", "offset", "heading_error", "speed", "acceleration", "curvature_rate"))
        for row in zip(solution.distance, solution.offset, solution.heading_error, solution.speed, solution.acceleration, solution.curvature_rate):
            writer.writerow((f"{float(value):.12g}" for value in row))


def _parse_spacings(value) -> tuple[float, ...]:
    """Validate a coarse-to-fine mesh ladder."""
    if isinstance(value, str):
        parts = value.split(",")
        if any(not part.strip() for part in parts):
            raise ValueError("spacings must be a comma-separated list of numbers")
        spacings = tuple(float(part) for part in parts)
    else:
        spacings = tuple(float(item) for item in value)
    if len(spacings) < 2:
        raise ValueError("at least two mesh spacings are required")
    if any(not np.isfinite(item) or item <= 0.0 for item in spacings):
        raise ValueError("mesh spacings must be finite and positive")
    if any(a <= b for a, b in zip(spacings, spacings[1:])):
        raise ValueError("mesh spacings must be strictly decreasing")
    return spacings


def _open_envelope_seed(track, mesh, envelope, settings: QssSettings) -> np.ndarray:
    """Project an open-course envelope speed profile onto an OCP mesh."""
    mesh = np.asarray(mesh, dtype=float)
    nodes = mesh.size
    count = nodes - 1
    distance = np.asarray(track.distance, dtype=float)
    envelope_speed = np.asarray(envelope.speed, dtype=float)
    speed = np.maximum(np.interp(mesh, distance, envelope_speed), settings.minimum_speed)
    speed_rate = np.gradient(envelope_speed, distance)
    acceleration = np.interp(mesh, distance, envelope_speed * speed_rate)
    zeros = np.zeros(nodes)
    mids = np.zeros(count)
    return np.concatenate([zeros, zeros, speed, acceleration, zeros, mids, mids])


def run(args) -> dict:
    output = args.output.resolve()
    _validate_output_paths(Path(args.track).resolve(), output)
    track = load_track(args.track)
    half_width = getattr(args, "half_width", None)
    _validate_track_for_line_pipeline(track, half_width)
    corridor = corridor_from_track(track, margin=args.margin, half_width=half_width)
    output.mkdir(parents=True, exist_ok=True)
    compiled = compile_formulation(str(args.formulation))
    vehicle = point_mass_view_of_planar(compiled.model)
    surface = load_capability_surface(args.surface)
    expected_hash = capability_hash(vehicle)
    if surface.provenance_hash != expected_hash:
        raise ValueError(f"capability surface provenance hash mismatch: expected {expected_hash}, got {surface.provenance_hash}")

    lap_mode = LapMode.FLYING if track.is_closed else LapMode.STANDING_START
    print("Solving centreline envelope...", flush=True)
    centreline_lap = solve_lap(vehicle, track, lap_mode)
    seed_lap = (
        centreline_lap if track.is_closed
        else solve_lap(vehicle, track, lap_mode, conservative_transition=True)
    )
    zero_corridor = Corridor(corridor.distance, np.zeros_like(corridor.minimum), np.zeros_like(corridor.maximum), corridor.provenance, corridor.is_closed)
    settings = QssSettings()
    spacings = _parse_spacings(getattr(args, "spacings", (5.0, 2.0, 1.0)))
    launch = 0.0 if track.is_closed else launch_time(track, surface, settings)
    zero_mesh = build_mesh(track, 1.0 if track.is_closed else 2.0)
    zero_seed = None if track.is_closed else _open_envelope_seed(track, zero_mesh, seed_lap, settings)
    print("Solving zero-corridor control...", flush=True)
    zero_solution = solve_line(track, zero_corridor, surface, zero_mesh, settings, seed=zero_seed)
    print(f"Zero-corridor control completed: {zero_solution.lap_time:.6f} s", flush=True)
    stage_wall_seconds = []

    def record_stage(spacing, solution, wall_seconds):
        print(f"Solved free line at {spacing:g} m spacing in {wall_seconds:.2f} s", flush=True)
        stage_wall_seconds.append(wall_seconds)

    print(f"Solving free-line mesh ladder ({'/'.join(f'{s:g}' for s in spacings)} m)...", flush=True)
    ladder_seed = None
    if not track.is_closed:
        ladder_seed = _open_envelope_seed(track, build_mesh(track, spacings[0]), seed_lap, settings)
    stages = solve_ladder(
        track, corridor, surface, settings, spacings=spacings,
        seed=ladder_seed, stage_callback=record_stage,
    )
    finest = stages[-1]
    _write_line(output / "line.csv", finest)
    rebuilt = path_from_solution(track, finest)
    _write_line(output / "racing_line.csv", finest)
    xml_path, csv_path = _write_rebuilt_track(Path(args.track).resolve(), output, rebuilt)
    replay_track = load_track(xml_path)
    replay = solve_lap(vehicle, replay_track, lap_mode)

    coarse, fine = stages[-2], stages[-1]
    mesh_ok = check_mesh_convergence(coarse=coarse.lap_time, fine=fine.lap_time, tolerance=0.005)
    if track.is_closed:
        sample_count = finest.distance.size - 1
        unique_mesh = finest.distance[:sample_count]
        containment_offset = finest.offset[:sample_count]
        minimum = track_sample(corridor.minimum[:-1], corridor.distance[:-1], unique_mesh,
                               closed=True, period=float(track.lap_length))
        maximum = track_sample(corridor.maximum[:-1], corridor.distance[:-1], unique_mesh,
                               closed=True, period=float(track.lap_length))
    else:
        unique_mesh = finest.distance
        containment_offset = finest.offset
        minimum = track_sample(corridor.minimum, corridor.distance, unique_mesh,
                               closed=False, period=float(track.lap_length))
        maximum = track_sample(corridor.maximum, corridor.distance, unique_mesh,
                               closed=False, period=float(track.lap_length))
    containment = check_corridor_containment(containment_offset, minimum, maximum)
    zero_total = zero_solution.lap_time + launch
    free_total = finest.lap_time + launch
    zero_reference = (
        centreline_lap.lap_time if track.is_closed
        else capability_envelope_time(track, surface, settings, start_speed=0.0)
    )
    checks = {
        "zero_corridor_matches_envelope": check_zero_corridor_matches_envelope(open_loop=zero_reference, ocp=zero_total, tolerance=0.01),
        "free_line_not_slower": check_free_line_not_slower(
            centreline=centreline_lap.lap_time, free=free_total,
            tolerance=0.0 if track.is_closed else 0.002,
        ),
        "corridor_containment": containment,
        "mesh_convergence": mesh_ok,
        "replay_agreement": check_replay_agreement(ocp=free_total, replay=replay.lap_time, tolerance=0.01),
    }
    summary = {
        "track": str(Path(args.track)), "track_closed": bool(track.is_closed),
        "corridor_provenance": corridor.provenance,
        "constant_half_width_m": half_width,
        "conditional_result": track.width_provenance != "surveyed", "margin_m": args.margin,
        "mesh_spacing_m": float(spacings[-1]), "centreline_envelope_s": centreline_lap.lap_time,
        "zero_corridor_reference_s": zero_reference,
        "launch_time_s": launch,
        "ocp_zero_corridor_s": zero_solution.lap_time, "ocp_free_line_s": finest.lap_time,
        "ocp_zero_corridor_including_launch_s": zero_total,
        "ocp_free_line_including_launch_s": free_total,
        "replay_lap_s": replay.lap_time,
        "free_line_gain_s": centreline_lap.lap_time - free_total,
        "free_line_gain_percent": 100.0 * (centreline_lap.lap_time - free_total) / centreline_lap.lap_time,
        "stages": [{"spacing_m": spacing, "iterations": stage.iterations, "wall_seconds": wall_seconds, "lap_time_s": stage.lap_time} for spacing, stage, wall_seconds in zip(spacings, stages, stage_wall_seconds)],
        "maximum_violation": finest.maximum_violation, "checks": checks,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print("Acceptance check                  Result")
    for name, passed in checks.items():
        print(f"{name:34} {str(passed).lower()}")
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", type=Path, required=True)
    parser.add_argument("--formulation", type=Path, required=True)
    parser.add_argument("--surface", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--margin", type=float, default=0.10)
    parser.add_argument(
        "--half-width", type=float, default=None,
        help="constant reference-point half-width for a track without width data",
    )
    parser.add_argument(
        "--spacings", type=str, default="5,2,1",
        help="comma-separated mesh ladder spacings in metres, coarse to fine",
    )
    args = parser.parse_args(argv)
    try:
        summary = run(args)
    except Exception as exc:
        print(f"minimum-time-line pipeline failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0 if all(summary["checks"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
