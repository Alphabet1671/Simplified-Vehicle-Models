"""Canonical fail-closed source identity for opt-in forward execution."""
from __future__ import annotations
import hashlib
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]

# Explicit reviewed execution closure. Paths are repository-relative; absent
# entries are errors, never silently dropped. Keep preparation and child calls
# on this single manifest.
_REQUIRED = (
    "tools/sim_gui/source_identity.py",
    "tools/sim_gui/forward_execute.py",
    "tools/sim_gui/telemetry_schema.py",
    "tools/sim_gui/execute.py",
    "tools/sim_gui/__init__.py",
    "tools/sim_gui/run_case.py",
    "tools/sim_gui/settings.py",
    "tools/sim_gui/study.py",
    "tools/sim_gui/catalogue.py",
    "src/simplified_vehicle_models/solvers/forward_driver.py",
    "src/simplified_vehicle_models/solvers/hybrid_setup.py",
    "src/simplified_vehicle_models/solvers/hybrid_driver.py",
    "src/simplified_vehicle_models/solvers/hybrid_speed_plan.py",
    "src/simplified_vehicle_models/solvers/longitudinal_capability.py",
    "src/simplified_vehicle_models/solvers/envelope_driver.py",
    "src/simplified_vehicle_models/solvers/brake_authority.py",
    "src/simplified_vehicle_models/controls/brake_inverse.py",
    "src/simplified_vehicle_models/controls/brakes.py",
    "src/simplified_vehicle_models/controls/steering.py",
    "src/simplified_vehicle_models/solvers/forward_powertrain.py",
    "src/simplified_vehicle_models/solvers/yaw_transition.py",
    "src/simplified_vehicle_models/solvers/planar_forward.py",
    "src/simplified_vehicle_models/solvers/vertical_forward.py",
    "src/simplified_vehicle_models/solvers/braking_envelope.py",
    "src/simplified_vehicle_models/results/plant_channels.py",
    "src/simplified_vehicle_models/tires/unitire_simple.py",
    "src/simplified_vehicle_models/compiler/compile.py",
    "src/simplified_vehicle_models/compiler/manifest.py",
    "src/simplified_vehicle_models/formulations/vertical_14dof.py",
    "src/simplified_vehicle_models/suspension/network.py",
    "src/simplified_vehicle_models/tires/unitire_transient.py",
    "src/simplified_vehicle_models/tires/vertical.py",
    "src/simplified_vehicle_models/xml/model.py",
    "src/simplified_vehicle_models/xml/schema.py",
    "src/simplified_vehicle_models/compiler/adapters.py",
    "src/simplified_vehicle_models/compiler/capabilities.py",
    "src/simplified_vehicle_models/formulations/planar_7dof.py",
    "src/simplified_vehicle_models/tracks/path_coordinates.py",
    "src/simplified_vehicle_models/results/run_record.py",
    "src/simplified_vehicle_models/parameters/vehicle.py",
    "src/simplified_vehicle_models/powertrain/energy_storage.py",
    "src/simplified_vehicle_models/tracks/profile.py",
    "tools/sim_gui/variant.py",
    "tools/sim_gui/vehicle.py",
)

def canonical_repo_path(path: str | Path, *, root: Path = REPOSITORY_ROOT) -> str:
    """Normalize an existing path to a root-relative POSIX key; reject escape."""
    root = Path(root).resolve()
    candidate = Path(path)
    absolute = candidate.resolve() if candidate.is_absolute() else (root / candidate).resolve()
    try:
        return absolute.relative_to(root).as_posix()
    except ValueError:
        raise ValueError(f"source path escapes repository root: {path}") from None

def execution_source_identity(*, root: Path = REPOSITORY_ROOT) -> dict[str, str]:
    """Hash every required execution source, independent of process cwd."""
    root = Path(root).resolve()
    result = {}
    for rel in _REQUIRED:
        path = root / rel
        if not path.is_file():
            raise FileNotFoundError(f"required forward execution source missing: {rel}")
        key = canonical_repo_path(path, root=root)
        result[key] = "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
    if len(result) != len(_REQUIRED):
        raise ValueError("forward execution source manifest contains duplicate canonical paths")
    return result

def identity_digest(identity: dict[str, str]) -> str:
    import json
    return hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()

def combined_source_closure(component_hashes: dict[str, str], execution_hashes: dict[str, str], *, root: Path = REPOSITORY_ROOT) -> dict[str, str]:
    """Combine model/input closure and code closure without key collisions."""
    if not isinstance(component_hashes, dict) or not component_hashes:
        raise ValueError("compiled model/input source closure is required")
    root = Path(root).resolve()
    expected = {canonical_repo_path(root / item, root=root) for item in _REQUIRED}
    if not isinstance(execution_hashes, dict) or set(execution_hashes) != expected:
        raise ValueError("execution source closure must exactly match the required manifest")
    return ({"component:" + key: value for key, value in component_hashes.items()} |
            {"execution:" + key: value for key, value in execution_hashes.items()})
