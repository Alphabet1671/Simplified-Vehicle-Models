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
    if model_kind != "14dof-simple" or source.get("model_kind") != model_kind:
        raise ValueError("artifact must identify exact 14dof-simple model_kind")
    from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
    names = tuple(VerticalForwardPlant.state_names)
    time = np.asarray(arrays["time_s"])
    states, rates = np.asarray(arrays["state"]), np.asarray(arrays["state_rate"])
    if states.shape != (len(time), len(names)) or rates.shape != states.shape:
        raise ValueError("artifact result must contain matching canonical 38-state samples")
    commands = []
    with (directory / "commands.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            commands.append((float(row["swa_rad"]), float(row["accelerator"]), float(row["brake"])))
    if len(commands) != len(time):
        raise ValueError("commands.csv must contain one command per accepted sample")
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
    geometry = vehicle_geometry(vehicle.planar, vehicle)
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
