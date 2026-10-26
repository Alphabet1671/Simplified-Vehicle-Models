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
_COMMON = {"yaw_moment_tolerance", "map_identity", "map_file_sha256", "map_source_identity",
    "track_identity", "track_data_hashes", "vehicle_configuration_hashes", "vehicle_source_identity",
    "boundary", "boundary_type", "maximum_speed", "maximum_speed_mps", "solver", "solver_settings",
    "profile_settings", "profile_line_settings", "controller_values", "controller", "warm_start_policy", "warm_start_digest",
    "package_versions", "thread_settings", "initial_state_soc", "initial_state", "source_revision",
    "source_dirty_hashes", "source_provenance_status", "watchdog_budget_seconds", "plan_file_digest",
    "plan_array_digest", "mode", "duration", "maximum_step", "rtol", "atol", "method"}
_REPLAY = {"line_treatment", "controller_treatment", "replay_treatment"}
_OUTPUT_FIELDS = {"output_dir", "output_path", "plan", "warm_plan", "timestamp", "runtime_seconds",
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
    complete_source = (prov["source_revision"] != "unknown" and
                       prov["source_dirty_hashes"] != "unknown")
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
    # Paths and runtime observations are intentionally not compatibility data;
    # content digests and the explicit warm-start policy remain binding.
    values = {key: value for key, value in values.items() if key in allowed}
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
