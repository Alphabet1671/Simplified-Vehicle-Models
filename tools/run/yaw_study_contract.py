"""Canonical, self-contained provenance and comparison identity for yaw studies."""
from __future__ import annotations

from dataclasses import asdict, is_dataclass
from enum import Enum
import hashlib
import json
import math
from pathlib import Path

CONTRACT_VERSION = "yaw-study-contract-v1"
_DIMENSIONS = {"geometry": "geometry_spacing_m", "control": "control_spacing_m", "replay": "replay_treatment"}
# Explicit identity schema: unknown fields never become accidental compatibility keys.
# Plan file/array digests are verified against each run, but excluded from the
# cross-run identity because distinct study coordinates produce distinct plans.
_COMMON = {"yaw_moment_tolerance", "interval_q_tolerance", "acceleration_tolerance", "map_identity", "map_file_sha256", "map_source_identity",
    "track_identity", "track_data_hashes", "vehicle_configuration_hashes", "vehicle_source_identity",
    "boundary", "boundary_type", "maximum_speed", "maximum_speed_mps", "solver", "solver_settings",
    "profile_settings", "profile_line_settings", "controller_values", "controller", "warm_start_policy", "warm_start_digest",
    "package_versions", "thread_settings", "initial_state_soc", "initial_state", "source_revision",
    "source_dirty_hashes", "source_provenance_status", "watchdog_budget_seconds",
    "mode", "duration", "maximum_step", "rtol", "atol", "method"}
_REPLAY = {"line_treatment", "controller_treatment", "replay_treatment"}
# Minimum attestations the real runner can supply for every comparable run.
# Geometry/control are the only coordinates permitted to vary independently.
_REQUIRED = {
    "geometry": {"geometry_spacing_m", "control_spacing_m", "yaw_moment_tolerance", "interval_q_tolerance", "acceleration_tolerance",
        "map_identity", "map_file_sha256", "map_source_identity", "track_identity",
        "track_data_hashes", "vehicle_configuration_hashes", "boundary_type",
        "maximum_speed_mps", "warm_start_policy", "warm_start_digest"},
    "control": {"geometry_spacing_m", "control_spacing_m", "yaw_moment_tolerance", "interval_q_tolerance", "acceleration_tolerance",
        "map_identity", "map_file_sha256", "map_source_identity", "track_identity",
        "track_data_hashes", "vehicle_configuration_hashes", "boundary_type",
        "maximum_speed_mps", "warm_start_policy", "warm_start_digest"},
    "replay": {"geometry_spacing_m", "control_spacing_m", "yaw_moment_tolerance", "interval_q_tolerance", "acceleration_tolerance",
        "map_identity", "map_file_sha256", "map_source_identity", "track_identity",
        "track_data_hashes", "vehicle_configuration_hashes", "boundary_type",
        "maximum_speed_mps", "warm_start_policy", "warm_start_digest", "replay_treatment",
        "solver", "duration", "maximum_step", "rtol", "atol", "method",
        "initial_state", "initial_state_soc"},
}
_OUTPUT_FIELDS = {"output_dir", "output_path", "plan", "warm_plan", "timestamp", "runtime_seconds",
    "plan_file_digest", "plan_array_digest",
    "wall_time_seconds", "elapsed_seconds", "canonical_json_sha256", "contract_version"}


def _jsonable(value):
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "tolist"):
        return _jsonable(value.tolist())
    if hasattr(value, "item"):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("study contract values must be finite")
    return value


def _canonical(value):
    return json.dumps(_jsonable(value), sort_keys=True, separators=(",", ":"), allow_nan=False)


def _attested(value):
    """Whether a scalar evidence value is non-placeholder and JSON-safe."""
    return isinstance(value, (str, int, float)) and not isinstance(value, bool) and value not in ("", "unknown") and (not isinstance(value, float) or math.isfinite(value))


def _source_path_key(value):
    """Accept normalized filesystem path keys like source_provenance() emits."""
    if not isinstance(value, str) or not value or value != value.strip() or "\\" in value:
        return False
    path = Path(value)
    if not path.is_absolute() and ("/" not in value or any(part in {"", ".", ".."} for part in value.split("/"))):
        return False
    if path.is_absolute() and (str(path) != value or any(part in {".", ".."} for part in value.split("/"))):
        return False
    return bool(path.name) and value not in {".", ".."}


def _hash_mapping(value):
    """A transitive-source/hash attestation is normalized path→canonical SHA-256 hex."""
    return (isinstance(value, dict) and bool(value) and
            all(_source_path_key(key) and
                isinstance(digest, str) and len(digest) == 64 and
                all(char in "0123456789abcdef" for char in digest)
                for key, digest in value.items()))


def _required_attested(key, value):
    if key in {"track_data_hashes", "vehicle_configuration_hashes", "source_dirty_hashes"}:
        return _hash_mapping(value)
    if key == "initial_state":
        return value == "unknown" or (isinstance(value, dict) and
            isinstance(value.get("names"), list) and isinstance(value.get("values"), list) and
            len(value["names"]) == len(value["values"]) and bool(value["names"]) and
            all(isinstance(name, str) and name for name in value["names"]) and
            all(_attested(item) for item in value["values"]))
    return _attested(value)


def study_contract(args, resolved_settings, provenance) -> dict:
    """Return frozen settings and provenance, preserving missing history as unknown."""
    args = vars(args) if hasattr(args, "__dict__") else args
    payload = {
        "contract_version": CONTRACT_VERSION,
        "arguments": _jsonable(args or {}),
        "resolved_settings": _jsonable(resolved_settings or {}),
        "provenance": _jsonable(provenance or {}),
    }
    prov = payload["provenance"]
    prov.setdefault("source_revision", "unknown")
    prov.setdefault("source_dirty_hashes", "unknown")
    complete_source = (_attested(prov["source_revision"]) and
                       _required_attested("source_dirty_hashes", prov["source_dirty_hashes"]))
    # This is derived evidence, never a caller-controlled self-attestation.
    prov["source_provenance_status"] = "attested" if complete_source else "historical_unknown"
    encoded = _canonical(payload)
    payload["canonical_json_sha256"] = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    return payload


def finalized_plan_inputs(args, resolved_settings, provenance, plan_path, array_digest, *, map_identity, track_identity):
    """Build inputs manifest bound to an actual saved or replayed plan."""
    if not array_digest or array_digest == "pending":
        raise ValueError("a finalized study contract requires a plan array digest")
    plan_path = Path(plan_path)
    if not plan_path.is_file():
        raise ValueError(f"plan artifact does not exist: {plan_path}")
    finalized_provenance = dict(provenance)
    finalized_provenance.update(
        plan_file_digest=hashlib.sha256(Path(plan_path).read_bytes()).hexdigest(),
        plan_array_digest=array_digest,
    )
    return {
        "args": _jsonable(args),
        "map_identity": map_identity,
        "track_identity": track_identity,
        "continuous_feasibility_certified": False,
        "study_contract": study_contract(args, resolved_settings, finalized_provenance),
    }


def update_replay_initial_state(inputs_path, state, state_names):
    """Bind the actual initialized plant state to inputs.json, never the plan."""
    inputs_path = Path(inputs_path)
    manifest = json.loads(inputs_path.read_text())
    contract = manifest.get("study_contract")
    if not isinstance(contract, dict):
        raise ValueError("replay inputs lack a study contract")
    state_values = _jsonable(state)
    names = list(state_names) if state_names is not None else []
    if len(names) == len(state_values):
        recorded = {"names": names, "values": state_values}
        soc = state_values[names.index("soc")] if "soc" in names else "unknown"
    else:
        recorded, soc = "unknown", "unknown"
    provenance = contract.setdefault("provenance", {})
    provenance["initial_state"] = recorded
    provenance["initial_state_soc"] = soc
    contract.pop("canonical_json_sha256", None)
    contract["canonical_json_sha256"] = hashlib.sha256(_canonical(contract).encode("utf-8")).hexdigest()
    inputs_path.write_text(json.dumps(manifest, sort_keys=True, indent=2, allow_nan=False) + "\n")
    return manifest


def _comparison_values(contract, dimension):
    """Flatten a contract without letting one envelope layer overwrite another."""
    if dimension not in _DIMENSIONS:
        raise ValueError(f"invalid comparison dimension: {dimension}")
    if isinstance(contract, str):
        contract = json.loads(contract)
    envelope = _jsonable(contract)
    if not isinstance(envelope, dict):
        raise ValueError("study contract must be an object")
    # Flat records are retained for the documented helper interface; envelopes
    # separate argument/settings/provenance namespaces to prevent overwrites.
    top = {key: value for key, value in envelope.items()
           if key not in ({"arguments", "resolved_settings", "provenance"} | _OUTPUT_FIELDS)}
    layers = (("top_level", top), ("arguments", envelope.get("arguments", {})),
              ("resolved_settings", envelope.get("resolved_settings", {})),
              ("provenance", envelope.get("provenance", {})))
    values = {}
    for name, layer in layers:
        if not isinstance(layer, dict):
            raise ValueError(f"{name} must be an object")
        for key, value in layer.items():
            key = {"spacing": "geometry_spacing_m", "control_spacing": "control_spacing_m"}.get(key, key)
            if key in values and values[key] != value:
                raise ValueError(f"conflicting contract value for {key}")
            values[key] = value
    allowed = set(_COMMON)
    if dimension == "replay": allowed |= _REPLAY
    allowed |= {"geometry_spacing_m", "control_spacing_m"}
    # Paths, runtime observations, and per-run plan digests are not compatibility
    # data; map/source digests and warm-start lineage remain binding.
    values = {key: value for key, value in values.items() if key in allowed}
    # The bundled profile-line settings contain both study coordinates. Remove
    # those coordinates here; they are compared explicitly below, with only the
    # requested dimension allowed to differ.
    if isinstance(values.get("profile_line_settings"), dict):
        values["profile_line_settings"] = {key: value for key, value in values["profile_line_settings"].items()
            if key not in {"spacing", "control_spacing", "geometry_spacing_m", "control_spacing_m"}}
    missing = sorted(_REQUIRED[dimension] - values.keys())
    if missing:
        raise ValueError(f"incomplete {dimension} comparison contract; missing required fields: {', '.join(missing)}")
    for key in _REQUIRED[dimension]:
        if not _required_attested(key, values[key]):
            raise ValueError(f"incomplete {dimension} comparison contract; required field {key} lacks attestation")
    # The exact warm-plan artifact must be recorded for each run, but successive
    # refinements necessarily use different preceding plans.  Policy matching,
    # rather than artifact equality, is the cross-run compatibility requirement.
    if dimension in {"geometry", "control"}:
        values.pop("warm_start_digest", None)
    return values


def comparison_identity(contract, dimension: str) -> str:
    """Hash the explicit compatibility schema, omitting only its study coordinate."""
    values = _comparison_values(contract, dimension)
    values.pop(_DIMENSIONS[dimension], None)
    if dimension == "replay":
        for key in _REPLAY: values.pop(key, None)
    return hashlib.sha256(_canonical({"dimension": dimension, "values": values}).encode("utf-8")).hexdigest()


def validate_comparison_pair(first, second, dimension: str) -> None:
    """Reject any non-study-coordinate or warm-start-policy change."""
    a, b = _comparison_values(first, dimension), _comparison_values(second, dimension)
    allowed = {_DIMENSIONS[dimension]}
    if dimension == "replay": allowed |= _REPLAY
    differing = {key for key in a.keys() | b.keys() if a.get(key) != b.get(key)}
    if differing - allowed:
        key = sorted(differing - allowed)[0]
        raise ValueError(f"{key} must remain fixed for {dimension} comparison")
