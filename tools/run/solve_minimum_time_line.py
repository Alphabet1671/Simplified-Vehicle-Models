"""Run the stage 1 minimum-time racing-line acceptance pipeline."""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.controls.planning import (
    build_policy, controller_hash as policy_controller_hash,
)
from simplified_vehicle_models.controls.replay import REPLAY_ADAPTERS, build_replay
from simplified_vehicle_models.formulations.capability_backend import (
    CapabilityGrid, IdealCapabilityBackend, SampledCapability,
)
from simplified_vehicle_models.formulations.controller_capability import (
    AllocationCapabilityBackend,
)
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.point_mass_capability import (
    capability_hash, load_capability_surface, save_capability_surface,
)
from simplified_vehicle_models.solvers.closed_loop_line import correct_to_replay
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.solvers.qss_line_ocp import (
    QssSettings, build_mesh, capability_envelope_time, launch_time, solve_line,
    solve_ladder, track_sample, _offset_basis,
)
from simplified_vehicle_models.solvers.qss_spline_ocp import _geometry, solve_spline_line
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.tracks.corridor import Corridor, corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track, _read_width_csv
from simplified_vehicle_models.tracks.racing_line import path_from_solution
from tools.gates.check_minimum_time_line import (
    check_corridor_containment, check_free_line_not_slower, check_mesh_convergence,
    check_replay_agreement, check_zero_corridor_matches_envelope,
)


#: Controller modes accepted by the runner.
CONTROL_MODES = ("ideal", "allocation", "closed_loop")
#: Offset transcriptions accepted by the runner.
SOLVERS = ("legacy", "spline")
#: Default control spacing for the Task 4 offset spline when none is given.
DEFAULT_SPLINE_CONTROL_SPACING = 10.0
#: Generated artifacts whose reuse would overwrite a previous run.
GENERATED_LINE_ARTIFACTS = (
    "line.csv", "racing_line.csv", "racing_line_track.csv",
    "racing_line_track.xml", "rebuilt_width.csv", "summary.json",
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
        for name in GENERATED_LINE_ARTIFACTS
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


def _write_line(path: Path, solution, curvature_rate=None) -> None:
    """Write the per-node line artifact.

    A spline solution does not carry the linear-basis curvature rate, so the
    caller supplies it from the same offset spline; legacy solutions carry it
    directly. The fallback differentiates the reported curvature rather than
    inventing a column.
    """
    if curvature_rate is None:
        curvature_rate = getattr(solution, "curvature_rate", None)
    if curvature_rate is None:
        curvature_rate = np.gradient(
            np.asarray(solution.curvature, dtype=float),
            np.asarray(solution.distance, dtype=float),
        )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("distance", "offset", "heading_error", "speed", "acceleration", "curvature_rate"))
        for row in zip(solution.distance, solution.offset, solution.heading_error, solution.speed, solution.acceleration, curvature_rate):
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
    basis, _ = _offset_basis(track, mesh, settings.offset_control_spacing)
    offset_size = nodes if basis is None else basis.shape[1]
    offset = np.zeros(offset_size)
    zeros = np.zeros(nodes)
    mids = np.zeros(count)
    return np.concatenate([offset, zeros, speed, acceleration, zeros, mids, mids])


def _control_summary(mode, controller_hash, capability_mode):
    """Return the control-provenance fields shared by every run and comparison."""
    if mode not in CONTROL_MODES:
        raise ValueError(f"unknown control mode {mode!r}")
    return {
        "control_mode": mode,
        "controller_hash": controller_hash,
        "capability_mode": capability_mode,
    }


def _validate_control_args(args):
    """Validate the requested controller mode before any output is created."""
    mode = getattr(args, "control_mode", None) or "ideal"
    if mode not in CONTROL_MODES:
        raise ValueError(f"unknown control mode {mode!r}")
    if mode == "closed_loop" and not getattr(args, "replay_adapter", None):
        raise ValueError("closed_loop requires a registered replay adapter")
    if mode == "allocation" and not getattr(args, "control_policy", None):
        raise ValueError("allocation requires a registered controller policy")
    return mode


def _resolve_solver(args):
    """Resolve the offset transcription, keeping legacy as the default.

    A bare run (the default ``ideal`` mode) keeps the established
    ``solve_line``/``solve_ladder`` path so existing results do not move. A
    controller-aware mode selects the spline transcription unless the caller
    explicitly chose one. ``--legacy-solver`` is the one-release alias for
    ``--solver legacy``.
    """
    mode = getattr(args, "control_mode", None) or "ideal"
    solver = getattr(args, "solver", None)
    if getattr(args, "legacy_solver", False):
        if solver not in (None, "legacy"):
            raise ValueError("--legacy-solver conflicts with --solver spline")
        solver = "legacy"
    if solver is None:
        solver = "legacy" if mode == "ideal" else "spline"
    if solver not in SOLVERS:
        raise ValueError(f"unknown solver {solver!r}")
    if mode == "closed_loop" and solver != "spline":
        raise ValueError("closed_loop requires the spline solver; pass --solver spline")
    return solver


def _load_policy(args):
    """Load the registered controller policy, if one was requested."""
    name = getattr(args, "control_policy", None)
    if not name:
        return None
    try:
        return build_policy(name)
    except KeyError as exc:
        raise ValueError(f"unknown control policy {name!r}; register it before running") from exc


def _build_replay_adapter(args, vehicle, lap_mode):
    """Resolve ``--replay-adapter`` through the registry; never fall back."""
    name = getattr(args, "replay_adapter", None)
    if not name:
        return None
    if name not in REPLAY_ADAPTERS:
        registered = ", ".join(sorted(REPLAY_ADAPTERS)) or "none"
        raise ValueError(
            f"unknown replay adapter {name!r}; registered adapters: {registered}"
        )
    try:
        return build_replay(name, vehicle=vehicle, lap_mode=lap_mode)
    except TypeError as exc:
        raise ValueError(
            f"replay adapter {name!r} must accept vehicle and lap_mode keyword arguments"
        ) from exc


class _CachedIdealCapabilityBackend(IdealCapabilityBackend):
    """Ideal backend backed by the already-validated capability cache.

    ``IdealCapabilityBackend`` rebuilds the surface from the vehicle. The
    runner has already checked the cache provenance against that vehicle, so the
    cache is the ideal surface for exactly these axes; reusing it preserves the
    current surface-cache behaviour and avoids repeating the expensive build
    while keeping the backend's result contract.
    """

    def __init__(self, vehicle, surface):
        super().__init__(vehicle)
        self._cached_surface = surface

    def sample(self, grid: CapabilityGrid, *, curvature_limit: float) -> SampledCapability:
        if not (
            np.array_equal(grid.speed, self._cached_surface.speed)
            and np.array_equal(grid.curvature, self._cached_surface.curvature)
        ):
            raise ValueError("cached ideal surface is not aligned with the requested grid")
        mask = np.ones(self._cached_surface.acceleration_max.shape, dtype=bool)
        return SampledCapability(
            self._cached_surface, self._cached_surface.provenance_hash, "",
            self.mode, feasible_mask=mask,
        )


@dataclass(frozen=True, slots=True)
class _CapabilitySelection:
    """Capability surface plus the provenance that must reach the summary."""

    surface: object
    capability_mode: str
    controller_hash: str
    saturation: tuple[str, ...]
    metadata: dict
    cache_path: Path | None = None


def _curvature_limit(surface) -> float:
    """Return a positive curvature-domain label for a loaded surface."""
    curvature = np.asarray(surface.curvature, dtype=float)
    limit = float(np.max(np.abs(curvature))) if curvature.size else 0.0
    if not np.isfinite(limit) or limit <= 0.0:
        return 1.0e-6
    return limit


def _reject_infeasible_cells(sampled) -> None:
    """Fail loudly instead of treating a zeroed cell as a hold-speed cell."""
    mask = sampled.feasible_mask
    if mask is None or bool(np.all(mask)):
        return
    cells = np.argwhere(~mask)
    speed = np.asarray(sampled.surface.speed, dtype=float)
    curvature = np.asarray(sampled.surface.curvature, dtype=float)
    examples = ", ".join(
        f"(v={speed[i]:.6g} m/s, kappa={curvature[j]:.6g} 1/m)"
        for i, j in cells[:8]
    )
    raise ValueError(
        f"allocation capability marks {int(cells.shape[0])} (v, kappa) cell(s) "
        f"infeasible: {examples}; refusing to treat the zero-acceleration "
        "placeholder as a hold-speed capability"
    )


def _select_capability(args, mode, policy, vehicle, ideal_surface) -> _CapabilitySelection:
    """Choose and, for allocation, sample the capability surface for ``mode``."""
    has_axes = hasattr(ideal_surface, "speed") and hasattr(ideal_surface, "curvature")
    # Ideal sampling, or a stateful controller whose hidden state makes static
    # allocation invalid: in both cases the controller correction is the replay,
    # not a modified surface.
    uses_allocation = mode == "allocation" or (
        mode == "closed_loop" and policy is not None
        and not bool(getattr(policy, "is_stateful", False))
    )
    if not uses_allocation:
        controller_hash = ""
        if mode != "ideal" and policy is not None:
            controller_hash = policy_controller_hash(policy)
        if has_axes:
            grid = CapabilityGrid(
                np.asarray(ideal_surface.speed, dtype=float),
                np.asarray(ideal_surface.curvature, dtype=float),
            )
            sampled = _CachedIdealCapabilityBackend(vehicle, ideal_surface).sample(
                grid, curvature_limit=_curvature_limit(ideal_surface),
            )
            if sampled.mode != "ideal":
                raise ValueError(
                    f"requested ideal capability but the backend recorded {sampled.mode!r}"
                )
            surface = sampled.surface
        else:
            # Duck-typed surface used by the focused runner tests; the legacy
            # solvers consume only the surface object they are given.
            surface = ideal_surface
        return _CapabilitySelection(surface, "ideal", controller_hash, (), {})

    if not has_axes:
        raise ValueError(
            "allocation mode requires a capability surface with speed and curvature axes"
        )
    grid = CapabilityGrid(
        np.asarray(ideal_surface.speed, dtype=float),
        np.asarray(ideal_surface.curvature, dtype=float),
    )
    sampled = AllocationCapabilityBackend(vehicle, policy).sample(
        grid, curvature_limit=_curvature_limit(ideal_surface),
        ideal_surface=ideal_surface,
    )
    if sampled.mode != "allocation":
        raise ValueError(
            f"requested allocation capability but the backend recorded {sampled.mode!r}"
        )
    _reject_infeasible_cells(sampled)
    metadata = dict(sampled.metadata)
    saturation = tuple(metadata.get("saturation_tags", ()))
    cache_path = (
        Path(args.output).resolve()
        / f"capability_allocation_{sampled.controller_hash}.npz"
    )
    return _CapabilitySelection(
        sampled.surface, "allocation", sampled.controller_hash, saturation,
        metadata, cache_path,
    )


def _spline_for(track, args) -> OffsetSpline:
    """Build the Task 4 offset spline used by the spline transcription."""
    spacing = getattr(args, "offset_spacing", None)
    if spacing is None:
        spacing = DEFAULT_SPLINE_CONTROL_SPACING
    return OffsetSpline(track, control_spacing=float(spacing))


def _solve_spline_ladder(track, corridor, surface, settings, spacings, spline,
                         stage_callback=None):
    """Solve the spline OCP on each rung of the coarse-to-fine ladder."""
    stages = []
    for spacing in spacings:
        start = time.perf_counter()
        solution = solve_spline_line(
            track, corridor, surface, build_mesh(track, spacing), spline, settings,
        )
        wall_seconds = time.perf_counter() - start
        stages.append(solution)
        if stage_callback is not None:
            stage_callback(spacing, solution, wall_seconds)
    return stages


def _spline_curvature_rate(track, spline, solution):
    """Return the curvature rate of a spline solution at its mesh nodes."""
    _, _, rate, _, _ = _geometry(
        track, spline, solution.coefficients, solution.distance,
    )
    return rate


def _solve_correction(track, corridor, surface, mesh, spline, settings,
                      controller, replay):
    """Run the Task 6 correction loop while capturing its OCP solutions.

    ``CorrectionResult`` reports the replay outcome but not the accepted
    solution. The loop's module-level solver is wrapped for the duration of the
    call so the last solved line can be written without an extra solve; the
    wrapper is always restored. Replay samples are captured for saturation
    reporting.
    """
    import simplified_vehicle_models.solvers.closed_loop_line as closed_loop

    solutions = []
    samples = []
    original_solve = closed_loop.solve_spline_line

    def recording_solve(*args, **kwargs):
        solution = original_solve(*args, **kwargs)
        solutions.append(solution)
        return solution

    def recording_replay(path, controller_arg):
        sample = replay(path, controller_arg)
        samples.append(sample)
        return sample

    closed_loop.solve_spline_line = recording_solve
    try:
        result = correct_to_replay(
            track, corridor, surface, mesh, spline, settings, controller,
            recording_replay,
        )
    finally:
        closed_loop.solve_spline_line = original_solve
    return result, solutions, samples


def run(args) -> dict:
    output = args.output.resolve()
    _validate_output_paths(Path(args.track).resolve(), output)
    track = load_track(args.track)
    half_width = getattr(args, "half_width", None)
    _validate_track_for_line_pipeline(track, half_width)
    spacings = _parse_spacings(getattr(args, "spacings", (5.0, 2.0, 1.0)))
    corridor = corridor_from_track(track, margin=args.margin, half_width=half_width)

    # Controller configuration is validated before any artifact or solver work.
    mode = _validate_control_args(args)
    solver = _resolve_solver(args)
    path_smoothing = float(getattr(args, "path_smoothing", 0.0))
    if mode == "closed_loop" and path_smoothing > 0.0:
        raise ValueError(
            "closed_loop replays the unsmoothed rebuilt path; --path-smoothing "
            "must be 0 for a controller-accepted result"
        )
    policy = _load_policy(args)

    compiled = compile_formulation(str(args.formulation))
    vehicle = point_mass_view_of_planar(compiled.model)
    ideal_surface = load_capability_surface(args.surface)
    expected_hash = capability_hash(vehicle)
    if ideal_surface.provenance_hash != expected_hash:
        raise ValueError(f"capability surface provenance hash mismatch: expected {expected_hash}, got {ideal_surface.provenance_hash}")
    lap_mode = LapMode.FLYING if track.is_closed else LapMode.STANDING_START
    replay_adapter = _build_replay_adapter(args, vehicle, lap_mode)

    base_settings = QssSettings(offset_control_spacing=getattr(args, "offset_spacing", None))
    settings = base_settings
    if solver == "spline" and not track.is_closed:
        # The spline transcription owns the standing start through its speed
        # boundary; the legacy transcription reaches it through launch_time.
        settings = replace(base_settings, start_speed=base_settings.minimum_speed)
    spline = _spline_for(track, args) if solver == "spline" else None

    selection = _select_capability(args, mode, policy, vehicle, ideal_surface)
    surface = selection.surface
    if surface.provenance_hash != expected_hash:
        raise ValueError(
            "selected capability surface provenance hash mismatch: "
            f"expected {expected_hash}, got {surface.provenance_hash}"
        )
    output.mkdir(parents=True, exist_ok=True)
    if selection.cache_path is not None:
        save_capability_surface(surface, selection.cache_path)

    print("Solving centreline envelope...", flush=True)
    centreline_lap = solve_lap(vehicle, track, lap_mode)
    seed_lap = None
    if not track.is_closed:
        seed_lap = (
            centreline_lap if solver == "spline"
            else solve_lap(vehicle, track, lap_mode, conservative_transition=True)
        )
    zero_corridor = Corridor(corridor.distance, np.zeros_like(corridor.minimum), np.zeros_like(corridor.maximum), corridor.provenance, corridor.is_closed)
    launch = 0.0 if track.is_closed else launch_time(track, surface, settings)
    zero_mesh = build_mesh(track, 1.0 if track.is_closed else 2.0)
    print("Solving zero-corridor control...", flush=True)
    if solver == "legacy":
        zero_seed = None if track.is_closed else _open_envelope_seed(track, zero_mesh, seed_lap, settings)
        zero_solution = solve_line(track, zero_corridor, surface, zero_mesh, settings, seed=zero_seed)
    else:
        zero_solution = solve_spline_line(
            track, zero_corridor, surface, zero_mesh, spline, settings,
        )
    print(f"Zero-corridor control completed: {zero_solution.lap_time:.6f} s", flush=True)
    stage_wall_seconds = []

    def record_stage(spacing, solution, wall_seconds):
        print(f"Solved free line at {spacing:g} m spacing in {wall_seconds:.2f} s", flush=True)
        stage_wall_seconds.append(wall_seconds)

    print(f"Solving free-line mesh ladder ({'/'.join(f'{s:g}' for s in spacings)} m)...", flush=True)
    if solver == "legacy":
        ladder_seed = None
        if not track.is_closed:
            ladder_seed = _open_envelope_seed(track, build_mesh(track, spacings[0]), seed_lap, settings)
        stages = solve_ladder(
            track, corridor, surface, settings, spacings=spacings,
            seed=ladder_seed, stage_callback=record_stage,
        )
    else:
        stages = _solve_spline_ladder(
            track, corridor, surface, settings, spacings, spline,
            stage_callback=record_stage,
        )
    finest = stages[-1]

    rounds = 0
    accepted = True
    replay_time = float("nan")
    saturation = list(selection.saturation)
    if mode == "closed_loop":
        correction_mesh = build_mesh(track, spacings[-1])
        correction, captured, samples = _solve_correction(
            track, corridor, surface, correction_mesh, spline, settings,
            policy, replay_adapter,
        )
        if captured:
            finest = captured[-1]
        rounds = int(correction.rounds)
        accepted = bool(correction.accepted)
        replay_time = float(correction.replay_time)
        for sample in samples:
            saturation.extend(sample.saturation)
        saturation = sorted(set(saturation))

    line_rate = (
        _spline_curvature_rate(track, spline, finest) if solver == "spline"
        else None
    )
    _write_line(output / "line.csv", finest, line_rate)
    rebuilt = path_from_solution(
        track, finest, smoothing=path_smoothing,
    )
    _write_line(output / "racing_line.csv", finest, line_rate)
    xml_path, csv_path = _write_rebuilt_track(Path(args.track).resolve(), output, rebuilt)
    if mode != "closed_loop":
        replay_track = load_track(xml_path)
        replay_time = float(solve_lap(vehicle, replay_track, lap_mode).lap_time)

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
        # Closed loop's own acceptance is a one-sided replay comparison; the
        # symmetric point-mass gate would reject a legitimately faster replay.
        "replay_agreement": (
            bool(accepted) if mode == "closed_loop"
            else check_replay_agreement(ocp=free_total, replay=replay_time, tolerance=0.01)
        ),
    }
    summary = {
        "track": str(Path(args.track)), "track_closed": bool(track.is_closed),
        "corridor_provenance": corridor.provenance,
        "constant_half_width_m": half_width,
        "offset_control_spacing_m": settings.offset_control_spacing,
        "path_smoothing_m": path_smoothing,
        "conditional_result": track.width_provenance != "surveyed", "margin_m": args.margin,
        "mesh_spacing_m": float(spacings[-1]), "centreline_envelope_s": centreline_lap.lap_time,
        "zero_corridor_reference_s": zero_reference,
        "launch_time_s": launch,
        "ocp_zero_corridor_s": zero_solution.lap_time, "ocp_free_line_s": finest.lap_time,
        "ocp_zero_corridor_including_launch_s": zero_total,
        "ocp_free_line_including_launch_s": free_total,
        "replay_lap_s": replay_time,
        "free_line_gain_s": centreline_lap.lap_time - free_total,
        "free_line_gain_percent": 100.0 * (centreline_lap.lap_time - free_total) / centreline_lap.lap_time,
        "stages": [{"spacing_m": spacing, "iterations": stage.iterations, "wall_seconds": wall_seconds, "lap_time_s": stage.lap_time} for spacing, stage, wall_seconds in zip(spacings, stages, stage_wall_seconds)],
        "maximum_violation": finest.maximum_violation, "checks": checks,
    }
    control = _control_summary(mode, selection.controller_hash, selection.capability_mode)
    if control["control_mode"] != mode:
        raise ValueError(
            f"requested control mode {mode!r} but recorded {control['control_mode']!r}"
        )
    summary.update(control)
    summary.update({
        "requested_control_mode": mode,
        "solver": solver,
        "replay_adapter": getattr(args, "replay_adapter", None),
        "replay_time_s": replay_time,
        "rounds": rounds,
        "accepted": bool(accepted),
        "saturation": sorted(set(saturation)),
    })
    if solver == "spline":
        summary["spline_control_spacing_m"] = float(spline.control_spacing)
    if selection.cache_path is not None:
        summary["capability_cache"] = str(selection.cache_path)
    if selection.metadata:
        summary["capability_metadata"] = selection.metadata
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print("Acceptance check                  Result")
    for name, passed in checks.items():
        print(f"{name:34} {str(passed).lower()}")
    return summary


def _replace_args(args, **changes):
    """Return a shallow copy of a CLI namespace with selected fields replaced."""
    values = dict(vars(args))
    values.update(changes)
    from types import SimpleNamespace
    return SimpleNamespace(**values)


def _read_line_offsets(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read the distance/offset columns of a line artifact."""
    distance = []
    offset = []
    with Path(path).open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or not {"distance", "offset"} <= set(reader.fieldnames):
            raise ValueError(f"line artifact has no distance/offset columns: {path}")
        for row in reader:
            distance.append(float(row["distance"]))
            offset.append(float(row["offset"]))
    return np.asarray(distance, dtype=float), np.asarray(offset, dtype=float)


def _unique_line(distance: np.ndarray, offset: np.ndarray):
    """Drop the repeated terminal node of a closed-track line."""
    keep = np.r_[True, np.diff(distance) > 0.0]
    return distance[keep], offset[keep]


def _line_offset_rms(reference: Path, candidate: Path) -> float:
    """Return the RMS offset difference of ``candidate`` against ``reference``."""
    reference_distance, reference_offset = _unique_line(*_read_line_offsets(reference))
    candidate_distance, candidate_offset = _unique_line(*_read_line_offsets(candidate))
    if reference_distance.size == 0 or candidate_distance.size < 2:
        raise ValueError("line artifacts must contain at least two nodes")
    values = np.interp(reference_distance, candidate_distance, candidate_offset)
    return float(np.sqrt(np.mean((values - reference_offset) ** 2)))


def compare_control_modes(args, comparison_path: Path | None = None) -> dict:
    """Run ideal/allocation/closed_loop on identical inputs and write a report.

    The comparison always uses the spline transcription for all three modes so
    the offset representation and mesh are identical. It refuses to overwrite an
    existing comparison file or a non-empty per-mode output directory.
    """
    # Validate the requested mode's own requirements before doing any work; the
    # comparison then deliberately runs every mode.
    _validate_control_args(args)
    if not getattr(args, "control_policy", None):
        raise ValueError(
            "control-mode comparison requires --control-policy for allocation mode"
        )
    if not getattr(args, "replay_adapter", None):
        raise ValueError(
            "control-mode comparison requires --replay-adapter for closed_loop mode"
        )
    solver = getattr(args, "solver", None)
    if getattr(args, "legacy_solver", False):
        solver = "legacy"
    if solver is None:
        solver = "spline"
    if solver != "spline":
        raise ValueError(
            "--compare-control-modes requires the spline solver; omit --solver "
            "or pass --solver spline"
        )

    comparison_path = (
        Path(comparison_path) if comparison_path is not None
        else Path(args.output) / "control_mode_comparison.json"
    ).resolve()
    if comparison_path.exists():
        raise FileExistsError(
            f"comparison output already exists; refusing to overwrite: {comparison_path}"
        )
    for mode in CONTROL_MODES:
        mode_output = Path(args.output) / mode
        if mode_output.exists() and any(mode_output.iterdir()):
            raise FileExistsError(
                f"mode output directory is not empty; refusing to overwrite: {mode_output}"
            )
    generated = {
        (Path(args.output) / mode / name).resolve()
        for mode in CONTROL_MODES for name in GENERATED_LINE_ARTIFACTS
    }
    if comparison_path in generated:
        raise ValueError(
            f"comparison output would overwrite a mode artifact: {comparison_path}"
        )

    summaries = {}
    for mode in CONTROL_MODES:
        mode_args = _replace_args(
            args,
            control_mode=mode,
            solver="spline",
            compare_control_modes=None,
            output=Path(args.output) / mode,
        )
        summaries[mode] = run(mode_args)

    reference = Path(args.output) / "ideal" / "line.csv"
    modes = {}
    for mode in CONTROL_MODES:
        summary = summaries[mode]
        line = Path(args.output) / mode / "line.csv"
        rms = 0.0 if mode == "ideal" else _line_offset_rms(reference, line)
        checks = summary.get("checks", {})
        accepted = bool(summary.get("accepted", False))
        mesh_ok = bool(checks.get("mesh_convergence", False))
        modes[mode] = {
            "control_mode": summary.get("control_mode", mode),
            "controller_hash": summary.get("controller_hash", ""),
            "capability_mode": summary.get("capability_mode", "ideal"),
            "solver": summary.get("solver", "spline"),
            "lap_time_s": float(summary.get("ocp_free_line_including_launch_s", float("nan"))),
            "replay_time_s": float(summary.get("replay_time_s", float("nan"))),
            "line_offset_rms_m": rms,
            "maximum_violation": float(summary.get("maximum_violation", float("nan"))),
            "mesh_converged": mesh_ok,
            "rounds": int(summary.get("rounds", 0)),
            "accepted": accepted,
            "saturation": list(summary.get("saturation", [])),
            "status": "ok" if accepted and mesh_ok else "failed",
        }
    comparison = {
        "track": str(Path(args.track)),
        "formulation": str(Path(args.formulation)),
        "surface": str(Path(args.surface)),
        "margin_m": getattr(args, "margin", None),
        "half_width_m": getattr(args, "half_width", None),
        "spacings_m": list(_parse_spacings(getattr(args, "spacings", (5.0, 2.0, 1.0)))),
        "solver": "spline",
        "modes": modes,
        "lap_time_s": {mode: modes[mode]["lap_time_s"] for mode in CONTROL_MODES},
        "replay_time_s": {mode: modes[mode]["replay_time_s"] for mode in CONTROL_MODES},
        "line_offset_rms_m": {mode: modes[mode]["line_offset_rms_m"] for mode in CONTROL_MODES},
        "saturation": {mode: modes[mode]["saturation"] for mode in CONTROL_MODES},
        "accepted": {mode: modes[mode]["accepted"] for mode in CONTROL_MODES},
        "mesh_converged": {mode: modes[mode]["mesh_converged"] for mode in CONTROL_MODES},
    }
    comparison["all_accepted"] = all(
        entry["status"] == "ok" for entry in modes.values()
    )
    comparison_path.parent.mkdir(parents=True, exist_ok=True)
    if comparison_path.exists():
        raise FileExistsError(
            f"comparison output already exists; refusing to overwrite: {comparison_path}"
        )
    comparison_path.write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    return comparison


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
    parser.add_argument(
        "--offset-spacing", type=float, default=None,
        help="control-point spacing for a reduced linear offset basis, or the "
             "offset-spline control spacing for --solver spline",
    )
    parser.add_argument(
        "--path-smoothing", type=float, default=0.0,
        help="post-reconstruction path smoothing in metres",
    )
    parser.add_argument(
        "--control-mode", choices=CONTROL_MODES, default="ideal",
        help="controller-aware capability mode (default: ideal)",
    )
    parser.add_argument(
        "--control-policy", default=None, metavar="NAME",
        help="registered controller policy name; required by allocation mode",
    )
    parser.add_argument(
        "--replay-adapter", default=None, metavar="NAME",
        help="registered replay adapter name; required by closed_loop mode",
    )
    parser.add_argument(
        "--solver", choices=SOLVERS, default=None,
        help="offset transcription; default is legacy for ideal mode and "
             "spline for allocation/closed_loop",
    )
    parser.add_argument(
        "--legacy-solver", action="store_true",
        help="alias for --solver legacy, kept for one release",
    )
    parser.add_argument(
        "--compare-control-modes", nargs="?", const="", default=None,
        metavar="PATH",
        help="run ideal/allocation/closed_loop on the same inputs and write a "
             "comparison JSON (default PATH: <output>/control_mode_comparison.json)",
    )
    args = parser.parse_args(argv)
    try:
        if args.compare_control_modes is not None:
            comparison = compare_control_modes(args)
            return 0 if comparison["all_accepted"] else 1
        summary = run(args)
    except Exception as exc:
        print(f"minimum-time-line pipeline failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0 if all(summary["checks"].values()) and summary.get("accepted", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
