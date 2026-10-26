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
    prov.setdefault("source_provenance_status", "historical_unknown" if prov["source_revision"] == "unknown" else "attested")
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


def comparison_identity(contract, dimension: str) -> str:
    """Hash compatibility fields; only the requested study coordinate may vary."""
    if dimension not in _DIMENSIONS:
        raise ValueError(f"invalid comparison dimension: {dimension}")
    if isinstance(contract, str):
        contract = json.loads(contract)
    data = _jsonable(contract)
    # Contracts can either be flat parameter records or full contract envelopes.
    values = dict(data)
    if "arguments" in values:
        arguments = dict(values.pop("arguments") or {})
        for source, target in (("spacing", "geometry_spacing_m"), ("control_spacing", "control_spacing_m")):
            if source in arguments: arguments[target] = arguments.pop(source)
        settings = values.pop("resolved_settings", {}) or {}
        provenance = values.pop("provenance", {}) or {}
        values = {**arguments, **settings, **provenance, **values}
    values.pop("canonical_json_sha256", None)
    values.pop("output_dir", None)
    values.pop("output_path", None)
    values.pop("timestamp", None)
    values.pop("runtime_seconds", None)
    values.pop("wall_time_seconds", None)
    varying = _DIMENSIONS[dimension]
    if dimension == "geometry":
        invariant = "control_spacing_m"
    elif dimension == "control":
        invariant = "geometry_spacing_m"
    else:
        invariant = None
    # Geometry/control refinement is a paired comparison: reject differing settings
    # through a shared invariant baseline marker when a cohort declares one.
    cohort = values.pop("comparison_cohort", None)
    if invariant and cohort is not None and invariant in cohort:
        values[invariant] = cohort[invariant]
    values.pop(varying, None)
    # Explicitly exclude declared warm-start lineage from compatibility identity.
    values.pop("warm_start_lineage", None)
    values.pop("warm_start_plan", None)
    return hashlib.sha256(_canonical({"dimension": dimension, "values": values}).encode("utf-8")).hexdigest()


def validate_comparison_pair(first, second, dimension: str) -> None:
    """Check that a pair varies only along its declared comparison coordinate."""
    if dimension not in _DIMENSIONS:
        raise ValueError(f"invalid comparison dimension: {dimension}")
    if isinstance(first, str): first = json.loads(first)
    if isinstance(second, str): second = json.loads(second)
    def flatten(item):
        result = dict(item)
        for key in ("arguments", "resolved_settings", "provenance"):
            nested = result.pop(key, {}) or {}
            result.update(nested)
        result.pop("canonical_json_sha256", None)
        for key in ("output_dir", "output_path", "timestamp", "runtime_seconds", "wall_time_seconds", "warm_start_lineage", "warm_start_plan"):
            result.pop(key, None)
        return result
    a, b = flatten(_jsonable(first)), flatten(_jsonable(second))
    allowed = {_DIMENSIONS[dimension]}
    differing = {key for key in a.keys() | b.keys() if a.get(key) != b.get(key)}
    if differing - allowed:
        key = sorted(differing - allowed)[0]
        raise ValueError(f"{key} must remain fixed for {dimension} comparison")
