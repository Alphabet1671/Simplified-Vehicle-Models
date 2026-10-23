"""Run the stage 1 minimum-time racing-line acceptance pipeline."""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
import hashlib
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
from simplified_vehicle_models.solvers.line_validation import (
    REQUIRED_NUMERICAL_CHECKS, strict_validation, validate_line,
)
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
    "racing_line_track.xml", "rebuilt_width.csv", "trajectory.npz", "summary.json",
)
#: Acceptance checks that every controller-mode comparison must report.
GATE_NAMES = (
    "zero_corridor_matches_envelope", "free_line_not_slower",
    "corridor_containment", "mesh_convergence", "replay_agreement",
)


def _validate_track_for_line_pipeline(track, half_width: float | None = None) -> None:
    """Reject inputs outside the track scope before solver work."""
    if track.has_edges and half_width is not None:
        raise ValueError("half_width is only valid for a track without edges")
    if not track.has_edges and half_width is None:
        raise ValueError("minimum-time line pipeline requires a lateral corridor or half_width")


def _validate_output_paths(source: Path, output: Path, *, reject_existing: bool = False) -> None:
    """Reject output artifacts that collide with sources or prior results."""
    root = ET.parse(source).getroot()
    # Keep lexical paths as well as resolved paths.  A dangling symlink does not
    # exist(), but a later writer would follow it and write outside ``output``.
    generated = {output / name for name in GENERATED_LINE_ARTIFACTS}
    protected = {source.resolve()}
    for element in root.findall("./data"):
        reference = element.get("ref")
        if not reference:
            continue
        source_asset = (source.parent / reference).resolve()
        protected.add(source_asset)
        if element.get("role") == "width":
            generated.add(output / f"{source_asset.stem}_rebuilt.csv")
    for candidate in generated:
        resolved = candidate.resolve()
        for source_asset in protected:
            same_file = resolved == source_asset
            if not same_file and candidate.exists() and source_asset.exists():
                same_file = candidate.samefile(source_asset)
            if same_file:
                raise ValueError(
                    f"output artifact would overwrite source track asset: {source_asset}"
                )
        if reject_existing and (candidate.exists() or candidate.is_symlink()):
            raise FileExistsError(
                f"output artifact already exists; refusing to overwrite: {candidate}"
            )


def _json_safe(value):
    """Replace non-finite floating diagnostics with JSON null recursively."""
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.floating):
        value = float(value)
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def _source_hashes(source: Path) -> dict[str, str]:
    """Return role-keyed SHA-256 hashes for the track XML and its assets.

    The role keys follow the repository's ``track_xml``/``track_csv``/
    ``width_csv`` convention. Missing references are omitted rather than
    raising: the runner has already validated the inputs it consumes, and a hash
    manifest is reporting metadata.
    """
    source = Path(source)
    root = ET.parse(source).getroot()
    entries = [("track_xml", source)]
    counts: dict[str, int] = {}
    for element in root.findall("./data"):
        reference = element.get("ref")
        if not reference:
            continue
        role = element.get("role")
        key = "track_csv" if role is None else f"{role}_csv"
        counts[key] = counts.get(key, 0) + 1
        if counts[key] > 1:
            key = f"{key}_{counts[key]}"
        entries.append((key, source.parent / reference))
    hashes = {}
    for key, path in entries:
        if not path.is_file():
            continue
        hashes[key] = hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def _write_rebuilt_track(source: Path, output: Path, path) -> tuple[Path, Path]:
    """Write rebuilt geometry with widths sampled by explicit reference station."""
    _validate_output_paths(source, output, reject_existing=False)
    root = ET.parse(source).getroot()
    width_elements = root.findall("./data[@role='width']")
    if len(width_elements) > 1:
        raise ValueError("source track has multiple width-role data references")
    geometry_data = next((element for element in root.findall("./data") if element.get("role") is None), None)
    if geometry_data is None or not geometry_data.get("ref"):
        raise ValueError("source track has no unroled geometry data reference")
    csv_path = output / "racing_line_track.csv"
    xml_path = output / "racing_line_track.xml"
    # ``run`` has already reserved the complete artifact set, but this writer
    # remains safe when called directly: never replace its own output targets.
    for target in (csv_path, xml_path):
        if target.exists() or target.is_symlink():
            raise FileExistsError(
                f"rebuilt track artifact already exists; refusing to overwrite: {target}"
            )
    source_track = load_track(source)
    if source_track.path_coordinates is None:
        raise ValueError("rebuilt geometry requires source signed coordinates")
    original = source_track.path_coordinates
    reference_station = getattr(path, "reference_distance", None)
    if reference_station is None:
        if path.distance.size != original.distance.size:
            raise ValueError("adaptive rebuilt geometry requires explicit reference station")
        reference_station = np.asarray(original.distance, dtype=float)
    reference_station = np.asarray(reference_station, dtype=float)
    if reference_station.shape != path.distance.shape or np.any(np.diff(reference_station) <= 0.0):
        raise ValueError("rebuilt path reference station must be strictly increasing and aligned")
    if (reference_station[0] != original.distance[0]
            or reference_station[-1] != original.distance[-1]):
        raise ValueError("rebuilt path reference station must cover source endpoints")
    # Preserve source cross-sections by sampling them at reference station,
    # never at rebuilt path arclength. Unwrap first so heading interpolation is
    # meaningful across a closed seam.
    original_heading = np.interp(reference_station, original.distance,
                                 np.unwrap(np.asarray(original.heading)))
    original_north = np.interp(reference_station, original.distance,
                               np.asarray(original.x_north))
    original_east = np.interp(reference_station, original.distance,
                              np.asarray(original.y_east))
    heading_delta = np.asarray(path.heading) - original_heading
    heading_delta = (heading_delta + np.pi) % (2.0 * np.pi) - np.pi
    if not np.all(np.isfinite(heading_delta)) or np.max(np.abs(heading_delta)) > 0.25:
        raise ValueError("rebuilt path heading differs by more than 0.25 rad; width projection is unreliable")

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
        width_station, left, right = _read_width_csv(source_width)
        left = np.interp(reference_station, width_station, left)
        right = np.interp(reference_station, width_station, right)
        width_name = f"{source_width.stem}_rebuilt.csv"
        width_data = element
        break
    if left is not None:
        # Project the old cross-section edge vectors onto the rebuilt path's
        # normals. This preserves nodewise normal clearance as a small-angle
        # approximation, not the exact 2D boundary point. The heading guard above
        # limits the tangential component omitted by this representation.
        left_x = original_north + left * np.sin(original_heading)
        left_y = original_east - left * np.cos(original_heading)
        right_x = original_north - right * np.sin(original_heading)
        right_y = original_east + right * np.cos(original_heading)
        new_left_normal = np.column_stack((np.sin(path.heading), -np.cos(path.heading)))
        new_right_normal = np.column_stack((-np.sin(path.heading), np.cos(path.heading)))
        rebuilt_left = (left_x - path.x_north) * new_left_normal[:, 0] + (left_y - path.y_east) * new_left_normal[:, 1]
        rebuilt_right = (right_x - path.x_north) * new_right_normal[:, 0] + (right_y - path.y_east) * new_right_normal[:, 1]
        if np.any(rebuilt_left <= 0.0) or np.any(rebuilt_right <= 0.0):
            raise ValueError("racing line leaves nonpositive clearance to a source track edge")
        width_csv = output / width_name
        if width_csv.exists() or width_csv.is_symlink():
            raise FileExistsError(
                f"rebuilt track artifact already exists; refusing to overwrite: {width_csv}"
            )
        # All input geometry and width arrays have been preflighted before this
        # first output write.
        output.mkdir(parents=True, exist_ok=True)
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
    # All geometry and clearance checks above precede every output write.
    output.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("distance", "x_north", "y_east", "heading_rad", "curvature"))
        for row in zip(path.distance, path.x_north, path.y_east, path.heading, path.curvature):
            writer.writerow((f"{float(v):.12g}" for v in row))
    geometry_data.set("ref", csv_path.name)
    title = root.find("./metadata/title")
    if title is not None:
        title.text = f"{title.text or 'Track'} — rebuilt racing line"
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(xml_path, encoding="utf-8", xml_declaration=True)
    return xml_path, csv_path


def _write_trajectory_metadata(path: Path, solution) -> None:
    """Persist exact spline trajectory state needed to reproduce an export."""
    trajectory = getattr(solution, "trajectory", None)
    if trajectory is None:
        return
    midpoint = getattr(solution, "midpoint_acceleration", None)
    if midpoint is None:
        raise ValueError("analytic spline solution is missing midpoint acceleration")
    reference_station = np.asarray(trajectory.reference.source_station, dtype=float)
    reference = trajectory.reference.evaluate(reference_station)
    arrays = {
        "source_reference_station": reference_station,
        "reference_north": np.asarray(reference.north, dtype=float),
        "reference_east": np.asarray(reference.east, dtype=float),
        "map_reference_station": np.asarray(trajectory.station_map.reference_station, dtype=float),
        "path_station": np.asarray(trajectory.station_map.path_station, dtype=float),
        "control_station": np.asarray(trajectory.spline.control_distance, dtype=float),
        "coefficients": np.asarray(solution.coefficients, dtype=float),
        "node_station": np.asarray(solution.distance, dtype=float),
        "node_speed": np.asarray(solution.speed, dtype=float),
        "node_acceleration": np.asarray(solution.acceleration, dtype=float),
        "midpoint_acceleration": np.asarray(midpoint, dtype=float),
    }
    if any(value.ndim != 1 or not np.all(np.isfinite(value)) for value in arrays.values()):
        raise ValueError("analytic spline trajectory metadata must be finite one-dimensional arrays")
    if (arrays["map_reference_station"].shape != arrays["path_station"].shape
            or arrays["reference_north"].shape != arrays["source_reference_station"].shape
            or arrays["reference_east"].shape != arrays["source_reference_station"].shape):
        raise ValueError("trajectory reference and station map arrays must have matching shape")
    if arrays["control_station"].shape != arrays["coefficients"].shape:
        raise ValueError("trajectory controls and coefficients must have matching shape")
    if (arrays["node_station"].shape != arrays["node_acceleration"].shape
            or arrays["node_speed"].shape != arrays["node_station"].shape):
        raise ValueError("trajectory node arrays must have matching shape")
    if arrays["midpoint_acceleration"].size != arrays["node_station"].size - 1:
        raise ValueError("trajectory midpoint accelerations must match mesh intervals")
    reference_fit_periodic = getattr(
        trajectory.reference, "interpolation_periodic", bool(trajectory.reference.is_closed),
    )
    if not isinstance(reference_fit_periodic, (bool, np.bool_)):
        raise ValueError("reference interpolation periodicity must be boolean")
    reference_fit_periodic = bool(reference_fit_periodic)
    reference_closed = bool(trajectory.reference.is_closed)
    schema_version = 2 if reference_fit_periodic != reference_closed else 1
    metadata_fields = {
        "schema_version": schema_version,
        "state_representation": str(getattr(solution, "state_representation", "legacy_speed_hs_v1")),
        # Missing control metadata identifies old artifacts as their established
        # physical-acceleration quadratic representation; it is never inferred
        # as the opt-in metric-weighted rate representation.
        "control_representation": str(getattr(
            solution, "control_representation", "quadratic_acceleration_v1",
        )),
        "reference_interpolant": "scipy.make_interp_spline",
        "reference_degree": 5 if reference_station.size >= 6 else 3,
        "is_closed": bool(trajectory.reference.is_closed),
        "offset_control_spacing_m": float(trajectory.spline.control_spacing),
        "time_quadrature": str(getattr(solution, "time_quadrature", "legacy_unspecified")),
        "time_quadrature_metadata": getattr(solution, "time_quadrature_metadata", None),
    }
    if schema_version == 2:
        metadata_fields["source_interpolation_periodic"] = reference_fit_periodic
    metadata = json.dumps(metadata_fields, sort_keys=True)
    # Do not overwrite a reproducibility artifact even if called outside runner
    # preflight.
    with path.open("xb") as handle:
        np.savez_compressed(handle, metadata=np.array(metadata), **arrays)


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

    A bare run and both the ``ideal`` and ``allocation`` modes keep the
    established ``solve_line``/``solve_ladder`` path; the spline transcription
    is opt-in through ``--solver spline``. ``closed_loop`` uses the Task 6
    correction loop, which requires the spline solver, so it must be requested
    explicitly. ``--legacy-solver`` is the one-release alias for
    ``--solver legacy``.
    """
    mode = getattr(args, "control_mode", None) or "ideal"
    solver = getattr(args, "solver", None)
    if getattr(args, "legacy_solver", False):
        if solver not in (None, "legacy"):
            raise ValueError("--legacy-solver conflicts with --solver spline")
        solver = "legacy"
    if solver is None:
        solver = "legacy"
    if solver not in SOLVERS:
        raise ValueError(f"unknown solver {solver!r}")
    if mode == "closed_loop" and solver != "spline":
        raise ValueError(
            "closed_loop requires an explicit --solver spline; the correction "
            "loop does not have a legacy path"
        )
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
        if getattr(self._cached_surface, "mode", "ideal") != "ideal":
            raise ValueError(
                "cached ideal surface is not an ideal capability cache: "
                f"mode is {getattr(self._cached_surface, 'mode', 'ideal')!r}"
            )
        cached_mask = getattr(self._cached_surface, "feasible_mask", None)
        if cached_mask is not None and not np.all(cached_mask):
            raise ValueError(
                "cached ideal surface records infeasible cells; refusing to "
                "relabel a zeroed placeholder as an ideal capability"
            )
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


def _validate_surface_cache(surface, mode, policy) -> None:
    """Reject a cache whose recorded provenance is not usable for this run.

    ``--surface`` names the capability source. It is normally an ideal cache,
    which every mode may use as its base. An allocation cache is accepted only
    for the controller mode and policy that produced it; an allocation cache
    must never be relabelled as ideal.
    """
    cache_mode = str(getattr(surface, "mode", "ideal"))
    cache_hash = str(getattr(surface, "controller_hash", ""))
    if cache_mode not in CONTROL_MODES:
        raise ValueError(
            f"capability cache records unknown mode {cache_mode!r}"
        )
    if cache_mode == "ideal":
        if cache_hash:
            raise ValueError(
                "capability cache records mode 'ideal' with a non-empty "
                f"controller hash {cache_hash!r}"
            )
        mask = getattr(surface, "feasible_mask", None)
        if mask is not None and not np.all(mask):
            raise ValueError(
                "ideal capability cache records infeasible cells; an ideal "
                "surface must not carry a zeroed placeholder"
            )
        return
    if cache_mode == "allocation":
        if mode == "ideal":
            raise ValueError(
                "--surface is an allocation capability cache; refusing to "
                "launder an allocation cache as the ideal surface"
            )
        mask = getattr(surface, "feasible_mask", None)
        if mask is not None and not np.all(mask):
            raise ValueError(
                "allocation capability cache records infeasible cells; "
                "refusing to reuse a zeroed placeholder"
            )
        if policy is None:
            raise ValueError(
                "--surface is an allocation capability cache, which requires "
                "the --control-policy that produced it"
            )
        if mode == "closed_loop" and bool(getattr(policy, "is_stateful", False)):
            raise ValueError(
                "closed_loop with a stateful policy requires an ideal --surface "
                "cache; an allocation cache cannot represent its hidden state"
            )
        expected = policy_controller_hash(policy)
        if cache_hash != expected:
            raise ValueError(
                "capability cache controller hash mismatch: expected "
                f"{expected}, got {cache_hash}"
            )
        return
    raise ValueError(
        f"capability cache mode {cache_mode!r} is not valid for --surface"
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
    trajectory = getattr(solution, "trajectory", None)
    if trajectory is not None:
        return np.asarray(trajectory.evaluate(solution.distance).heading_error_s)
    _, _, rate, _, _ = _geometry(track, spline, solution.coefficients, solution.distance)
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
    # Controller/output configuration is validated before loading a track,
    # solver work, cache writes, or output-directory creation.
    mode = _validate_control_args(args)
    solver = _resolve_solver(args)
    path_smoothing = float(getattr(args, "path_smoothing", 0.0))
    if mode == "closed_loop" and path_smoothing > 0.0:
        raise ValueError(
            "closed_loop replays the unsmoothed rebuilt path; --path-smoothing "
            "must be 0 for a controller-accepted result"
        )
    if solver == "spline" and path_smoothing != 0.0:
        raise ValueError("smoothing is not permitted for a spline trajectory export")
    _validate_output_paths(Path(args.track).resolve(), output, reject_existing=True)
    track = load_track(args.track)
    if mode == "closed_loop" and not track.is_closed:
        raise ValueError(
            "closed_loop is not supported on an open course: the correction "
            "loop compares a launch-free OCP time with a launch-inclusive "
            "replay; use the ideal/allocation mode, or an adapter that "
            "supports launch-inclusive comparison"
        )
    half_width = getattr(args, "half_width", None)
    _validate_track_for_line_pipeline(track, half_width)
    spacings = _parse_spacings(getattr(args, "spacings", (5.0, 2.0, 1.0)))
    corridor = corridor_from_track(track, margin=args.margin, half_width=half_width)
    policy = _load_policy(args)

    compiled = compile_formulation(str(args.formulation))
    vehicle = point_mass_view_of_planar(compiled.model)
    ideal_surface = load_capability_surface(args.surface)
    expected_hash = capability_hash(vehicle)
    if ideal_surface.provenance_hash != expected_hash:
        raise ValueError(f"capability surface provenance hash mismatch: expected {expected_hash}, got {ideal_surface.provenance_hash}")
    _validate_surface_cache(ideal_surface, mode, policy)
    lap_mode = LapMode.FLYING if track.is_closed else LapMode.STANDING_START
    replay_adapter = _build_replay_adapter(args, vehicle, lap_mode)
    if (mode == "closed_loop" and policy is not None
            and not bool(getattr(replay_adapter, "consumes_controller", False))):
        raise ValueError(
            "closed_loop with --control-policy requires a replay adapter that "
            "consumes the controller; the registered point-mass adapter ignores "
            "it and cannot produce a controller-aware result"
        )
    controller_aware = bool(mode != "ideal" and policy is not None)

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
    # The mode that the executed branch records, not the value used to build
    # the summary. They must agree; the check below is deliberately not a
    # comparison of a value with itself.
    recorded_mode = selection.capability_mode

    rounds = 0
    replay_time = float("nan")
    correction = None
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
        replay_time = float(correction.replay_time)
        for sample in samples:
            saturation.extend(sample.saturation)
        saturation = sorted(set(saturation))
        recorded_mode = "closed_loop"

    line_rate = (
        _spline_curvature_rate(track, spline, finest) if solver == "spline"
        else None
    )
    _write_line(output / "line.csv", finest, line_rate)
    if solver == "spline":
        _write_trajectory_metadata(output / "trajectory.npz", finest)
    rebuilt = path_from_solution(track, finest, smoothing=path_smoothing)
    _write_line(output / "racing_line.csv", finest, line_rate)
    xml_path, csv_path = _write_rebuilt_track(Path(args.track).resolve(), output, rebuilt)
    if mode != "closed_loop":
        replay_track = load_track(xml_path)
        replay_time = float(solve_lap(vehicle, replay_track, lap_mode).lap_time)

    coarse, fine = stages[-2], stages[-1]
    mesh_ok = check_mesh_convergence(coarse=coarse.lap_time, fine=fine.lap_time, tolerance=0.005)
    mesh_unavailable_reason = None
    if mode == "closed_loop":
        # The ladder above is the pre-correction free line; closed_loop writes
        # the corrected line solved at the finest mesh only, so reusing the
        # ladder result would claim convergence for a different line.
        mesh_ok = None
        mesh_unavailable_reason = (
            "closed_loop corrects the finest line after the pre-correction "
            "mesh ladder; that ladder does not establish the corrected line's "
            "mesh convergence"
        )
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
    check_unavailable = {}
    if mode == "ideal":
        zero_reference = (
            centreline_lap.lap_time if track.is_closed
            else capability_envelope_time(track, surface, settings, start_speed=0.0)
        )
        zero_gate = check_zero_corridor_matches_envelope(
            open_loop=zero_reference, ocp=zero_total, tolerance=0.01,
        )
        free_reference = centreline_lap.lap_time
        free_gate = check_free_line_not_slower(
            centreline=free_reference, free=free_total,
            tolerance=0.0 if track.is_closed else 0.002,
        )
        replay_gate = check_replay_agreement(
            ocp=free_total, replay=replay_time, tolerance=0.01,
        )
    elif mode == "allocation":
        # Allocation has no replay caps on the free line, so a same-surface
        # zero reference is meaningful. Never compare against the ideal
        # solve_lap envelope.
        replay_gate = None
        check_unavailable["replay_agreement"] = (
            "allocation mode has no controller replay adapter; the "
            "rebuilt-path point-mass replay is not the allocation controller"
        )
        if track.is_closed:
            zero_reference = None
            zero_gate = None
            check_unavailable["zero_corridor_matches_envelope"] = (
                "closed-track allocation has no independent zero-corridor "
                "reference: the zero OCP is solved on the same allocation "
                "surface and is not an independent check"
            )
            free_reference = zero_total
            free_gate = check_free_line_not_slower(
                centreline=free_reference, free=free_total, tolerance=0.0,
            )
        else:
            # The capability envelope starts from rest on the same surface and
            # is independent of the zero OCP mesh.
            zero_reference = capability_envelope_time(
                track, surface, settings, start_speed=0.0,
            )
            zero_gate = check_zero_corridor_matches_envelope(
                open_loop=zero_reference, ocp=zero_total, tolerance=0.01,
            )
            free_reference = zero_reference
            free_gate = check_free_line_not_slower(
                centreline=free_reference, free=free_total, tolerance=0.002,
            )
    else:
        # closed_loop writes a replay-capped corrected line. The zero OCP is
        # uncapped and on a different mesh, so neither comparison has a valid
        # same-cap reference yet. Do not turn that mismatch into a pass/fail.
        zero_reference = None
        zero_gate = None
        check_unavailable["zero_corridor_matches_envelope"] = (
            "closed_loop records a replay-capped corrected line; the "
            "zero-corridor OCP is uncapped, so no same-cap zero reference exists"
        )
        free_reference = zero_total
        free_gate = None
        check_unavailable["free_line_not_slower"] = (
            "closed_loop records a replay-capped corrected line; the "
            "zero-corridor OCP is uncapped and on a different mesh, so no "
            "same-cap zero reference exists yet"
        )
        feasible = bool(correction.feasible) if correction is not None else False
        replay_gate = (
            check_replay_agreement(
                ocp=free_total, replay=replay_time, tolerance=0.01,
            )
            and feasible
        )
    checks = {
        "zero_corridor_matches_envelope": zero_gate,
        "free_line_not_slower": free_gate,
        "corridor_containment": containment,
        "mesh_convergence": mesh_ok,
        "replay_agreement": replay_gate,
    }
    # Preserve legacy gates as evidence, but strict acceptance additionally
    # requires independently-dense geometry/state evidence and all mandated
    # refinement/baseline/replay evidence.  Never infer unavailable evidence.
    solver_succeeded = str(getattr(finest, "status", "")) in {
        "Solve_Succeeded", "success", "succeeded",
    }
    dense_report = validate_line(finest, corridor, surface, settings, vehicle=vehicle)
    checks.update(dense_report["checks"])
    # The existing mesh gate is relevant to the mandated mesh evidence.  Basis
    # refinement and matched same-geometry baseline are not yet run by runner;
    # record them explicitly unavailable rather than making a green claim.
    checks["mesh_convergence"] = mesh_ok
    checks["basis_convergence"] = None
    checks["matched_baseline"] = None
    strict = strict_validation(solver_succeeded, checks, REQUIRED_NUMERICAL_CHECKS)
    accepted = strict["accepted"]
    gain_reference = (
        centreline_lap.lap_time if mode == "ideal" else free_reference
    )
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
        "free_line_gain_s": gain_reference - free_total,
        "free_line_gain_percent": 100.0 * (gain_reference - free_total) / gain_reference,
        "stages": [{"spacing_m": spacing, "iterations": stage.iterations, "wall_seconds": wall_seconds, "lap_time_s": stage.lap_time} for spacing, stage, wall_seconds in zip(spacings, stages, stage_wall_seconds)],
        "maximum_violation": finest.maximum_violation, "checks": checks,
        "dense_validation": dense_report,
        "track_source_hashes": _source_hashes(Path(args.track)),
        "solver_tolerance": float(settings.tolerance),
        "spline_type": (
            ("closed-periodic" if track.is_closed else "open-natural")
            if spline is not None else None
        ),
    }
    if recorded_mode != mode:
        raise ValueError(
            f"requested control mode {mode!r} but recorded {recorded_mode!r}"
        )
    summary.update(_control_summary(
        recorded_mode, selection.controller_hash, selection.capability_mode,
    ))
    summary.update({
        "requested_control_mode": mode,
        "solver": solver,
        "replay_adapter": getattr(args, "replay_adapter", None),
        "replay_kind": (
            "controller_adapter"
            if mode == "closed_loop" and controller_aware
            else "point_mass_parity"
        ),
        "controller_aware": controller_aware,
        "replay_time_s": replay_time,
        "rounds": rounds,
        "accepted": bool(accepted),
        "saturation": sorted(set(saturation)),
        **strict,
    })
    if mesh_unavailable_reason is not None:
        check_unavailable["mesh_convergence"] = mesh_unavailable_reason
    if check_unavailable:
        summary["check_unavailable"] = check_unavailable
    if solver == "spline":
        summary["spline_control_spacing_m"] = float(spline.control_spacing)
        objective = getattr(finest, "objective_value", None)
        trajectory = getattr(finest, "trajectory", None)
        summary["objective_value"] = float(finest.lap_time if objective is None else objective)
        summary["physical_lap_time_s"] = float(finest.lap_time)
        if trajectory is not None:
            summary["reference_fit_discrepancy"] = trajectory.reference.fit_discrepancy
            summary["station_map_error"] = trajectory.station_map_error
            summary["path_length_m"] = float(trajectory.station_map.path_end)
    if selection.cache_path is not None:
        summary["capability_cache"] = str(selection.cache_path)
    if selection.metadata:
        summary["capability_metadata"] = selection.metadata
    summary = _json_safe(summary)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
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

    The comparison requires an explicit ``--solver spline`` because it runs the
    closed-loop mode, and uses that transcription for all three modes so the
    offset representation and mesh are identical. It refuses to overwrite an
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
        raise ValueError(
            "--compare-control-modes requires an explicit --solver spline; it "
            "runs the closed_loop mode, which has no legacy path"
        )
    if solver != "spline":
        raise ValueError(
            "--compare-control-modes requires the spline solver; pass "
            "--solver spline"
        )

    if comparison_path is None:
        requested_path = getattr(args, "compare_control_modes", None)
        comparison_path = (
            Path(requested_path) if requested_path
            else Path(args.output) / "control_mode_comparison.json"
        )
    comparison_path = Path(comparison_path).resolve()
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
        raw_checks = summary.get("checks", {})
        checks = {}
        failed_gates = []
        unavailable_gates = []
        for name in GATE_NAMES:
            # A missing gate is a failure; an explicit None means the gate is
            # recorded as unavailable with a check_unavailable reason.
            value = raw_checks[name] if name in raw_checks else False
            checks[name] = value
            if value is True:
                continue
            if value is None:
                unavailable_gates.append(name)
            else:
                failed_gates.append(name)
        accepted = bool(summary.get("accepted", False))
        # Do not reconstruct acceptance from the legacy gates.  The runner's
        # strict tier accounts for mandatory dense/basis/baseline evidence.
        status_ok = summary.get("validation_status") == "numerically_validated" and accepted
        modes[mode] = {
            "control_mode": summary.get("control_mode", mode),
            "controller_hash": summary.get("controller_hash", ""),
            "capability_mode": summary.get("capability_mode", "ideal"),
            "solver": summary.get("solver", "spline"),
            "lap_time_s": float(summary.get("ocp_free_line_including_launch_s", float("nan"))),
            "replay_time_s": float(summary.get("replay_time_s", float("nan"))),
            "line_offset_rms_m": rms,
            "maximum_violation": float(summary.get("maximum_violation", float("nan"))),
            "checks": checks,
            "failed_gates": failed_gates,
            "unavailable_gates": unavailable_gates,
            "mesh_converged": checks["mesh_convergence"],
            "rounds": int(summary.get("rounds", 0)),
            "accepted": accepted,
            "saturation": list(summary.get("saturation", [])),
            "status": "ok" if status_ok else "failed",
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
        "failed_gates": {mode: modes[mode]["failed_gates"] for mode in CONTROL_MODES},
        "unavailable_gates": {mode: modes[mode]["unavailable_gates"] for mode in CONTROL_MODES},
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
        json.dumps(_json_safe(comparison), indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8",
    )
    return comparison


def _checks_pass(checks) -> bool:
    """Compatibility helper: only an explicit strict numerical tier passes."""
    return bool(checks) and all(value is True for value in checks.values())


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
        help="offset transcription; default is legacy for ideal and allocation; "
             "closed_loop requires the explicit --solver spline",
    )
    parser.add_argument(
        "--legacy-solver", action="store_true",
        help="alias for --solver legacy, kept for one release",
    )
    parser.add_argument(
        "--compare-control-modes", nargs="?", const="", default=None,
        metavar="PATH",
        help="run ideal/allocation/closed_loop on the same inputs and write a "
             "comparison JSON (default PATH: <output>/control_mode_comparison.json); "
             "requires --solver spline",
    )
    args = parser.parse_args(argv)
    try:
        if args.compare_control_modes is not None:
            comparison_path = (
                Path(args.compare_control_modes)
                if args.compare_control_modes else None
            )
            comparison = compare_control_modes(args, comparison_path)
            return 0 if comparison["all_accepted"] else 1
        summary = run(args)
    except Exception as exc:
        print(f"minimum-time-line pipeline failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0 if summary.get("accepted") is True and summary.get("numerically_validated") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
