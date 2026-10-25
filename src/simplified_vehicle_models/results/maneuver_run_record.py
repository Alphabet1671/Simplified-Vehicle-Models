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


def _artifact_digest(manifest):
    """Return the SHA-256 of the source artifact, without hashing converted data."""
    manifest = _mapping(manifest)
    # Manifest spellings used by artifact producers; accept only explicit SHA-256.
    hashes = manifest.get("hashes", {})
    if not isinstance(hashes, Mapping):
        hashes = {}
    for key in ("sha256", "source_sha256", "artifact_sha256", "result_sha256"):
        value = manifest.get(key, hashes.get(key))
        if isinstance(value, str) and value.startswith("sha256:"):
            return value
        if isinstance(value, str) and len(value) == 64:
            try:
                int(value, 16)
                return "sha256:" + value.lower()
            except ValueError:
                pass
    raise ValueError("source_manifest must provide a SHA-256 digest")


def write_maneuver_run_record(path, definition, result, *, model_kind, source_manifest) -> Path:
    """Write accepted evidence verbatim; this function never instantiates a plant.

    The source manifest is provenance supplied by the artifact producer, not a
    path to re-open or a request to recompute/hash a mutable definition.
    """
    data = _mapping(definition)
    source = _mapping(source_manifest)
    digest = _artifact_digest(source)
    time = np.asarray(result.time_s)
    states = np.asarray(result.state)
    rates = np.asarray(result.state_rate)
    commands = tuple(result.applied_commands)
    channels = tuple(result.channels)
    names = tuple(source.get("state_names", ()))
    if not names:
        names = tuple(data.get("state_names", ()))
    if not names:
        # The accepted 14-DoF ordering is defined by the vertical plant class;
        # accessing its static names does not construct/evaluate it.
        from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
        names = tuple(VerticalForwardPlant.state_names)
    if len(names) != 38 or states.shape != (len(time), len(names)) or rates.shape != states.shape:
        raise ValueError("maneuver result must contain matching 38-state accepted samples")
    if len(commands) != len(time) or len(channels) != len(time):
        raise ValueError("commands and channels must have one entry per accepted sample")
    channel_names = tuple(name for name in CHANNEL_CATALOGUE
                          if any(isinstance(row, dict) and name in row for row in channels))
    recorder = RunRecorder(path, state_names=names,
                           state_units=tuple(state_unit(name) for name in names),
                           control_names=CONTROL_NAMES, control_units=CONTROL_UNITS,
                           channel_names=channel_names, precision="float64")
    for i, (t, state, rate, command, supplied) in enumerate(zip(
            time, states, rates, commands, channels)):
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
