"""Evidence-only export of completed maneuver artifacts into run-record-v1."""
from __future__ import annotations

from pathlib import Path
from collections.abc import Mapping
import csv
import json
import numpy as np

from simplified_vehicle_models.results.plant_channels import state_unit
from simplified_vehicle_models.results.run_record import CHANNEL_CATALOGUE, RunRecorder

CONTROL_NAMES = ("swa-rad", "accelerator-travel", "brake-travel")
CONTROL_UNITS = ("rad", "1", "1")


def load_maneuver_run_record(path, artifact_directory):
    """Validate a canonical result against the exact immutable maneuver evidence."""
    from tools.maneuver_gui.artifacts import load_artifact_evidence
    from simplified_vehicle_models.results.run_record import load_run_record
    source, arrays, _channels = load_artifact_evidence(Path(artifact_directory))
    if source.get("execution_outcome") != "completed":
        raise ValueError("maneuver evidence is not completed")
    record = load_run_record(path, verify_sources=False)
    if record.manifest.model.get("kind") != source.get("model_kind"):
        raise ValueError("simulation result model kind does not match maneuver evidence")
    if record.manifest.model.get("provenance") != source:
        raise ValueError("simulation result provenance does not match maneuver evidence")
    if record.manifest.segment.get("definition") != source.get("definition"):
        raise ValueError("simulation result definition does not match maneuver evidence")
    for actual, expected, label in ((record.time, arrays["time_s"], "time"),
                                    (record.state, arrays["state"], "state"),
                                    (record.state_rate, arrays["state_rate"], "state rate")):
        if not np.array_equal(actual, expected):
            raise ValueError(f"simulation result {label} does not match maneuver evidence")
    return record


def write_maneuver_run_record(path, definition, artifact_directory, *, model_kind="14dof-simple") -> Path:
    """Convert only immutable, integrity-checked artifact evidence; never rerun physics."""
    from tools.maneuver_gui.artifacts import load_artifact_evidence
    directory = Path(artifact_directory)
    source, arrays, channels = load_artifact_evidence(directory)
    data = source.get("definition")
    if not isinstance(data, Mapping) or json.loads(json.dumps(data)) != json.loads(json.dumps(definition, default=list)):
        raise ValueError("artifact definition does not match definition argument")
    if source.get("execution_outcome") != "completed":
        raise ValueError("only completed maneuver artifacts may be exported")
    if model_kind not in {"14dof-simple", "planar-7dof"} or source.get("model_kind") != model_kind:
        raise ValueError("artifact must identify an exactly supported model_kind")
    if model_kind == "14dof-simple":
        from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
        names = tuple(VerticalForwardPlant.state_names)
    else:
        from simplified_vehicle_models.solvers.maneuver_planar import ManeuverPlanarPlant
        names = tuple(ManeuverPlanarPlant.state_names)
    time = np.asarray(arrays["time_s"])
    states, rates = np.asarray(arrays["state"]), np.asarray(arrays["state_rate"])
    if time.ndim != 1 or len(time) < 1 or not np.all(np.isfinite(time)) or np.any(np.diff(time) <= 0):
        raise ValueError("artifact time must be finite, one-dimensional, and strictly increasing")
    if states.shape != (len(time), len(names)) or rates.shape != states.shape:
        raise ValueError(f"artifact result must contain matching {model_kind} state samples")
    if not np.all(np.isfinite(states)) or not np.all(np.isfinite(rates)):
        raise ValueError("artifact state and state-rate samples must be finite")
    if len(channels) != len(time):
        raise ValueError("accepted channel rows must match time samples")
    for row in channels:
        for key, value in row.items():
            if key not in CHANNEL_CATALOGUE:
                continue  # metadata carried with accepted sample, not a RunRecord channel
            array = np.asarray(value, dtype=float)
            width = CHANNEL_CATALOGUE[key].width
            if array.size != width or not np.all(np.isfinite(array)):
                raise ValueError(f"accepted channel {key!r} has invalid dimensions or values")
    commands = []
    with (directory / "commands.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            commands.append((float(row["swa_rad"]), float(row["accelerator"]), float(row["brake"])))
    if len(commands) != len(time):
        raise ValueError("commands.csv must contain one command per accepted sample")
    # Maneuver accepted channels may be sparse; only synthesize pose/speed from
    # explicit Cartesian model states (never forces or other plant outputs).
    state_index = {name: i for i, name in enumerate(names)}
    for i, row in enumerate(channels):
        pose_names = (("position-x", "x"), ("position-y", "y"), ("heading-world", "heading")) if model_kind == "planar-7dof" else ()
        for channel, state_name in pose_names:
            row.setdefault(channel, [float(states[i, state_index[state_name]])])
        if "speed" not in row and "v_x" in state_index and "v_y" in state_index:
            row["speed"] = [float(np.hypot(states[i, state_index["v_x"]], states[i, state_index["v_y"]]))]
    channel_names = tuple(name for name in CHANNEL_CATALOGUE if any(name in row for row in channels))
    required_display = {"position-x", "position-y", "heading-world"}
    if not required_display.issubset(channel_names):
        raise ValueError("artifact accepted channels lack required animation world-pose evidence")
    recorder = RunRecorder(path, state_names=names,
        state_units=tuple(state_unit(name) for name in names), control_names=CONTROL_NAMES,
        control_units=CONTROL_UNITS, channel_names=channel_names, precision="float64")
    for i, (t, state, rate, control, supplied) in enumerate(zip(time, states, rates, commands, channels)):
        recorder.append(t, i, state, rate, control,
                        {name: supplied[name] for name in channel_names if name in supplied})
    formulation = data.get("vehicle", {}).get("formulation") or data.get("vehicle", {}).get("model")
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.results.plant_channels import vehicle_geometry
    vehicle = compile_formulation(formulation).model
    geometry = vehicle_geometry(getattr(vehicle, "planar", vehicle), vehicle)
    digest = source["hashes"]["result_sha256"]
    metadata = {"producer": {"name": "maneuver_run_record", "version": 2},
        "model": {"kind": model_kind, "provenance": source},
        "sources": {}, "segment": {"definition": data},
        "track": {"file": "models/tracks/fsae_2026_acceleration.xml"},
        "vehicle_geometry": geometry, "notes": [
            f"maneuver-artifact-sha256=sha256:{digest}",
            "Animation uses the display-only straight-reference track geometry."],
    }
    return recorder.close(metadata)
