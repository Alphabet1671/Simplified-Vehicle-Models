"""Evidence-only export of accepted maneuver samples into run-record-v1."""
from __future__ import annotations

from pathlib import Path
from collections.abc import Mapping
import numpy as np

from simplified_vehicle_models.results.plant_channels import state_unit
from simplified_vehicle_models.results.run_record import CHANNEL_CATALOGUE, RunRecorder

CONTROL_NAMES = ("swa-rad", "accelerator-travel", "brake-travel")
CONTROL_UNITS = ("rad", "1", "1")


def _mapping(value):
    if hasattr(value, "to_dict"):
        value = value.to_dict()
    elif hasattr(value, "_data"):
        value = value._data
    if not isinstance(value, Mapping):
        raise TypeError("definition and source_manifest must be mappings or expose to_dict()")
    return value


def _json_value(value):
    """Copy provenance recursively into values supported by manifest JSON."""
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(f"provenance contains non-JSON value {type(value).__name__}")


def _validated_artifact(manifest, definition, model_kind):
    """Check the validated maneuver-artifact manifest identity and result digest."""
    manifest = _mapping(manifest)
    if manifest.get("schema") != "maneuver-artifacts-v1" or manifest.get("version") != 1:
        raise ValueError("source_manifest must be a maneuver-artifacts-v1 version 1 manifest")
    if manifest.get("execution_outcome") not in {"completed", "stopped"}:
        raise ValueError("source_manifest execution_outcome must be completed or stopped")
    if model_kind != "14dof-simple" or manifest.get("model_kind") != model_kind:
        raise ValueError("source_manifest and model_kind must identify 14dof-simple")
    if _json_value(manifest.get("definition")) != _json_value(definition):
        raise ValueError("source_manifest definition does not match definition argument")
    hashes = manifest.get("hashes")
    digest = hashes.get("result_sha256") if isinstance(hashes, Mapping) else None
    if (not isinstance(digest, str) or len(digest) != 64
            or any(ch not in "0123456789abcdefABCDEF" for ch in digest)):
        raise ValueError("source_manifest hashes.result_sha256 must be a SHA-256 hex digest")
    return "sha256:" + digest.lower()


def write_maneuver_run_record(path, definition, result, *, model_kind, source_manifest) -> Path:
    """Write accepted evidence verbatim; this function never instantiates a plant.

    The source manifest is provenance supplied by the artifact producer, not a
    path to re-open or a request to recompute/hash a mutable definition.
    """
    data = _mapping(definition)
    source = _mapping(source_manifest)
    from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
    names = tuple(VerticalForwardPlant.state_names)
    if model_kind != "14dof-simple":
        raise ValueError("model_kind must be exactly '14dof-simple'")
    digest = _validated_artifact(source, data, model_kind)
    time = np.asarray(result.time_s)
    states = np.asarray(result.state)
    rates = np.asarray(result.state_rate)
    commands = tuple(result.applied_commands)
    channels = tuple(result.channels)
    if tuple(source.get("state_names", names)) != names or tuple(data.get("state_names", names)) != names:
        raise ValueError("definition and source manifest must use canonical VerticalForwardPlant.state_names")
    if states.shape != (len(time), len(names)) or rates.shape != states.shape:
        raise ValueError("maneuver result must contain matching canonical 38-state accepted samples")
    if len(commands) != len(time) or len(channels) != len(time):
        raise ValueError("commands and channels must have one entry per accepted sample")
    channel_names = tuple(name for name in CHANNEL_CATALOGUE
                          if any(isinstance(row, Mapping) and name in row for row in channels))
    recorder = RunRecorder(path, state_names=names,
                           state_units=tuple(state_unit(name) for name in names),
                           control_names=CONTROL_NAMES, control_units=CONTROL_UNITS,
                           channel_names=channel_names, precision="float64")
    for i, (t, state, rate, command, supplied) in enumerate(zip(
            time, states, rates, commands, channels)):
        if not isinstance(supplied, Mapping):
            raise ValueError("each accepted channel row must be a mapping")
        values = (command.swa_rad, command.accelerator, command.brake)
        recorder.append(t, i, state, rate, values,
                        {name: supplied[name] for name in channel_names if name in supplied})
    geometry = source.get("vehicle_geometry", data.get("vehicle_geometry", {}))
    if not geometry and isinstance(source.get("vehicle"), Mapping):
        geometry = source["vehicle"].get("geometry", {})
    metadata = {
        "producer": {"name": "maneuver_run_record", "version": 1},
        "model": {"kind": str(model_kind), "name": str(source.get("model_name", model_kind)),
                  "provenance": _json_value(source)},
        "sources": _json_value(source.get("sources", {})),
        "segment": {"definition": _json_value(data)},
        "vehicle_geometry": _json_value(geometry) if isinstance(geometry, Mapping) else {},
        "notes": [f"maneuver-artifact-sha256={digest}"],
    }
    return recorder.close(metadata)
