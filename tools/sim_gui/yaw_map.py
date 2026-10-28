"""GUI-side yaw-map inspection, cache selection, and build command helpers."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import sys
import uuid

import numpy as np
from simplified_vehicle_models.formulations.yaw_map_artifact import build_identity, load_yaw_map


@dataclass(frozen=True)
class MapBuildRequest:
    formulation: Path
    candidate: Path | None
    output: Path
    identity: str
    reason: str


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _normalize(path: Path | str) -> Path:
    path = Path(path).expanduser()
    return path.resolve() if path.is_absolute() else (_repository_root() / path).resolve()


def _compiled(formulation: Path):
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
    compiled = compile_formulation(str(formulation)).model
    if isinstance(compiled, VerticalVehicleParameters):
        compiled = compiled.planar
    return compiled


def _settings():
    # Match yaw_map_options.resolve_yaw_map and builder CLI defaults verbatim.
    from simplified_vehicle_models.formulations.yaw_map_domain import YawMapDomainSettings
    from simplified_vehicle_models.formulations.yaw_map_workflow import YawMapWorkflowSettings
    return YawMapWorkflowSettings(
        discovery=YawMapDomainSettings(probe_budget=250000, scan_spacing=.5,
            bracket_tolerance=.05, lateral_interpolation_loss_tolerance=.25),
        maximum_lateral_cell_width=.5, maximum_map_nodes=3000000,
        maximum_audit_probes=500000, maximum_refinement_rounds=768,
        time_budget_seconds=1200.)


def _identity(formulation: Path):
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from tools.run.run_yaw_aware_forward_driver import anchored_axis, physical_map_speed_limit
    compiled = _compiled(formulation)
    vehicle = point_mass_view_of_planar(compiled)
    speed = np.linspace(0., physical_map_speed_limit(vehicle), 12)
    ax = anchored_axis(-6., 3., 9)
    return build_identity(vehicle, compiled.yaw_inertia, _settings(), speed, ax)


def _valid(path: Path, identity: str) -> bool:
    capability, _ = load_yaw_map(path, expected_identity=identity)
    axes = (capability.speed, capability.longitudinal_acceleration, capability.lateral_acceleration)
    if any(np.asarray(axis).size < 2 for axis in axes):
        raise ValueError('yaw-map profile requires at least two values per axis')
    return True


def inspect_yaw_map(formulation: Path, candidate: str, cache_root: Path) -> Path | MapBuildRequest:
    """Return an accepted candidate/cache artifact or describe a replacement build."""
    formulation = _normalize(formulation)
    cache_root = _normalize(cache_root)
    identity = _identity(formulation)
    requested = _normalize(candidate) if candidate else None
    failures = []
    if requested is not None:
        if not requested.is_file():
            failures.append(f'missing candidate: {requested}')
        else:
            try:
                _valid(requested, identity)
                return requested
            except Exception as error:
                details = str(error)
                kind = 'stale' if 'stale' in details.lower() else ('incomplete' if 'incomplete' in details.lower() else 'corrupt or invalid')
                failures.append(f'{kind} candidate: {details}')
    cache_root.mkdir(parents=True, exist_ok=True)
    for cached in sorted(cache_root.glob(f'{identity}-*.npz')):
        try:
            _valid(cached, identity)
            return cached.resolve()
        except Exception as error:
            failures.append(f'invalid cached attempt {cached.name}: {error}')
    reason = '; '.join(failures) if failures else 'no matching completed yaw-map candidate or cache artifact'
    output = cache_root / f'{identity}-{uuid.uuid4().hex}.npz'
    return MapBuildRequest(formulation, requested, output, identity, reason)


def accept_yaw_map(request: MapBuildRequest, artifact: Path) -> Path:
    """Strictly load and verify a finished build before returning its path."""
    path = _normalize(artifact)
    capability, _ = load_yaw_map(path, expected_identity=request.identity)
    if any(np.asarray(axis).size < 2 for axis in
           (capability.speed, capability.longitudinal_acceleration, capability.lateral_acceleration)):
        raise ValueError('yaw-map profile requires at least two values per axis')
    return path


def default_yaw_map_workers() -> int:
    """Use four build workers by default; explicit CLI overrides remain available."""
    return 4


def map_build_command(request: MapBuildRequest, workers: int | None = None) -> list[str]:
    workers = default_yaw_map_workers() if workers is None else int(workers)
    if workers < 1:
        raise ValueError('workers must be positive')
    return [sys.executable, '-u', '-m', 'tools.run.build_vehicle_yaw_map',
            '--formulation', str(request.formulation), '--output', str(request.output),
            '--workers', str(workers)]
