"""Reproducible matched continuous-driver throttle experiment preparation and arms.

Prepare a fixed-plan directory, obtain review, then invoke each arm separately:
  python tools/run/compare_continuous_driver_throttle.py prepare --output runs/...
  python tools/run/compare_continuous_driver_throttle.py arm runs/... heuristic
  python tools/run/compare_continuous_driver_throttle.py arm runs/... physical_mapper
No vehicle run occurs during preparation. Each arm is externally timeoutable.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import math
import os
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np

FORMULATION = Path("models/formulations/iem24_murphy_real_planar_ttc_r20_7dof.xml")
ASSEMBLY = Path("models/vehicles/iem24_murphy_real_planar_ttc_r20.xml")
DISTANCE_M, PRECURSOR_DISTANCE_M, MAXIMUM_SPEED_MPS = 150., 250., 60.
INITIAL_SPEED_MPS, INITIAL_PEDAL = 6., 0.
PEDAL_TRAVEL_TIME, PEDAL_TIME_CONSTANT = .175, .02
MAXIMUM_STEP, RTOL, ATOL, SOLVER_WALL_SECONDS = .02, 1.e-6, 1.e-8, 300.
DRIVER_GAINS = {"preview_seconds": .15, "speed_gain": 2., "heading_gain": 1.5,
                "lateral_gain": 1.5, "steering_time_constant": .08,
                "brake_governor_margin": .85}
SOURCE_PATHS = (FORMULATION, ASSEMBLY,
    Path("models/tires/ttc-r20-unitire-steady.xml"),
    Path("models/aero/iem24_murphy_real_aero.xml"), Path("models/powertrains/iem24_murphy_real_planar.xml"),
    Path("models/drivetrains/iem24_murphy_real.xml"),
    Path("models/power-plants/iem24_murphy_real_planar_motor.xml"),
    Path("models/energy-storages/iem24_murphy_real_battery.xml"),
    Path("models/data/iem24_murphy_parallel_steering.csv"), Path("models/data/murphy_pedal_ratio.csv"),
    Path(__file__), Path("tools/run/run_iem24_forward_driver.py"),
    Path("src/simplified_vehicle_models/compiler/compile.py"),
    Path("src/simplified_vehicle_models/compiler/manifest.py"),
    Path("src/simplified_vehicle_models/solvers/forward_driver.py"),
    Path("src/simplified_vehicle_models/solvers/forward_powertrain.py"),
    Path("src/simplified_vehicle_models/solvers/planar_forward.py"),
    Path("src/simplified_vehicle_models/solvers/physical_longitudinal_mapper.py"),
    Path("src/simplified_vehicle_models/formulations/planar_7dof.py"),
    Path("src/simplified_vehicle_models/formulations/point_mass.py"))


@dataclass(frozen=True)
class FrozenPlan:
    distance: np.ndarray
    speed: np.ndarray
    curvature: np.ndarray
    acceleration: np.ndarray
    digest: str
    precursor_distance: np.ndarray | None = None
    precursor_speed: np.ndarray | None = None
    crop_origin_m: float | None = None
    precursor_curvature: np.ndarray | None = None
    precursor_acceleration: np.ndarray | None = None


def freeze_plan_arrays(distance, speed, curvature, acceleration):
    arrays = [np.array(a, dtype=float, copy=True) for a in
              (distance, speed, curvature, acceleration)]
    if any(a.ndim != 1 or not np.all(np.isfinite(a)) for a in arrays):
        raise ValueError("plan arrays must be finite one-dimensional arrays")
    if len({a.shape for a in arrays}) != 1 or len(arrays[0]) < 2:
        raise ValueError("plan arrays must have matching lengths")
    if (arrays[0][0] != 0. or np.any(np.diff(arrays[0]) <= 0.)
            or np.any(arrays[1] <= 0.) or np.any(np.abs(arrays[2]) > 1.e-14)):
        raise ValueError("plan must start at zero, increase, and have positive speed/zero curvature")
    digest = hashlib.sha256()
    for name, array in zip(("distance", "speed", "curvature", "acceleration"), arrays):
        array.setflags(write=False)
        digest.update(name.encode() + b"\0")
        digest.update(np.asarray(array, dtype="<f8").tobytes())
    return FrozenPlan(*arrays, digest.hexdigest())


def interval_weighted_metrics(trace):
    t, throttle, deficit = (np.asarray(trace[name], dtype=float)
                             for name in ("time", "throttle", "speed_deficit"))
    if (t.ndim != 1 or t.size < 2 or throttle.shape != t.shape or deficit.shape != t.shape
            or not np.all(np.isfinite(t)) or not np.all(np.isfinite(throttle))
            or not np.all(np.isfinite(deficit)) or np.any(np.diff(t) <= 0.)):
        raise ValueError("trace needs finite equal-length channels and strictly increasing time")
    dt = np.diff(t)
    duration = float(dt.sum())
    return {"duration_s": duration,
            "full_throttle_fraction": float(np.sum(dt * (throttle[:-1] >= 1. - 1.e-12)) / duration),
            "mean_speed_deficit": float(np.sum(dt * deficit[:-1]) / duration)}


def require_matching_manifests(first, second):
    if first != second:
        raise ValueError("model/input manifest mismatch between comparison arms")


def create_output_directory(path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=False)
    return path


def collect_source_manifest(compiled_manifest=None):
    # SOURCE_PATHS is the fixed runner dependency graph. When compilation exposes
    # loaded model paths, include that complete graph as well.
    paths = {path.resolve() for path in SOURCE_PATHS}
    # Bound source coverage to the complete library Python package, avoiding
    # claims that a hand-picked import list is dependency-complete.
    paths.update(path.resolve() for path in Path("src/simplified_vehicle_models").rglob("*.py"))
    if compiled_manifest is not None:
        paths.update(Path(record.path).resolve() for record in compiled_manifest.loaded_components)
        paths.add(Path(compiled_manifest.formulation.path).resolve())
        paths.add(Path(compiled_manifest.assembly.path).resolve())
    revision = subprocess.run(["git", "rev-parse", "HEAD"], check=True,
                              capture_output=True, text=True).stdout.strip()
    raw = subprocess.run(["git", "status", "--porcelain=v1", "-z", "--untracked-files=all"],
                         check=True, capture_output=True).stdout
    entries = (raw.decode("utf-8", errors="surrogateescape") if isinstance(raw, bytes) else raw).split("\0")
    dirty_paths, candidates = [], set(paths)
    index = 0
    while index < len(entries) and entries[index]:
        entry = entries[index]
        status, name = entry[:2], entry[3:]
        candidate = Path(name).resolve()
        # Runtime artifacts are outputs, not provenance changes between arms.
        if candidate in paths or any(str(candidate).startswith(str(root.resolve()) + "/")
                                     for root in (Path("src"), Path("models"), Path("tools"), Path("tests"))):
            dirty_paths.append(status + " " + name)
            candidates.add(candidate)
        index += 1
        # In -z porcelain, a rename/copy has a second NUL-delimited source path.
        if "R" in status or "C" in status:
            if index < len(entries) and entries[index]:
                source_name = entries[index]
                source_path = Path(source_name).resolve()
                if any(str(source_path).startswith(str(root.resolve()) + "/")
                       for root in (Path("src"), Path("models"), Path("tools"), Path("tests"))):
                    dirty_paths.append("source " + source_name)
                    candidates.add(source_path)
                index += 1
    missing = sorted(str(path) for path in paths if not path.is_file())
    if missing:
        raise FileNotFoundError("required provenance inputs missing: " + ", ".join(missing))
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in candidates if path.is_file()}
    return {"revision": revision, "dirty": bool(dirty_paths), "dirty_paths": dirty_paths,
            "hashes": hashes, "coverage": "bounded: declared fixed graph, library Python package, and compiler-reported model graph; external/native dependency closure is not claimed",
            "runtime": {"python": __import__("platform").python_version(),
                        "numpy": np.__version__,
                        "scipy": importlib.metadata.version("scipy")}}


def build_speed_plan(*args, **kwargs):
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan as build
    return build(*args, **kwargs)


def solve_forward(*args, **kwargs):
    from simplified_vehicle_models.solvers.forward_driver import solve_forward as solve
    return solve(*args, **kwargs)


def compile_selected_model():
    from simplified_vehicle_models.compiler.compile import compile_formulation
    return compile_formulation(FORMULATION)


def make_plan(vehicle):
    """Crop a rolling 150 m section from the live-limit standing-start profile.

    build_speed_plan(open_course=True) has no initial-speed parameter. This
    documented slice begins at the first interpolated 6 m/s crossing; it is a
    mechanism-test course, not a launch-from-rest experiment.
    """
    from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
    distance = np.linspace(0., PRECURSOR_DISTANCE_M, 501)
    precursor = build_speed_plan(distance, np.zeros_like(distance), maximum_speed=MAXIMUM_SPEED_MPS,
                                 vehicle=vehicle, live_limit=True, open_course=True)
    crossings = np.flatnonzero((precursor.speed[:-1] < INITIAL_SPEED_MPS)
                               & (precursor.speed[1:] >= INITIAL_SPEED_MPS))
    if precursor.speed[0] == INITIAL_SPEED_MPS:
        origin = 0.
    elif crossings.size:
        i = int(crossings[0])
        fraction = ((INITIAL_SPEED_MPS - precursor.speed[i]) /
                    (precursor.speed[i + 1] - precursor.speed[i]))
        origin = float(precursor.distance[i] + fraction * (precursor.distance[i + 1] - precursor.distance[i]))
    else:
        raise ValueError("250 m live-limit precursor never reaches 6 m/s")
    end = origin + DISTANCE_M
    if end > precursor.distance[-1]:
        raise ValueError("250 m live-limit precursor lacks 150 m after first 6 m/s crossing")
    stations = np.r_[origin, precursor.distance[(precursor.distance > origin) &
                                                  (precursor.distance < end)], end]
    speeds = np.interp(stations, precursor.distance, precursor.speed)
    speeds[0] = INITIAL_SPEED_MPS
    curvatures = np.interp(stations, precursor.distance, precursor.curvature)
    selected = SpeedPlan(stations - origin, speeds, curvatures, is_closed=False)
    frozen = freeze_plan_arrays(selected.distance, selected.speed, selected.curvature,
                                selected.acceleration)
    precursor_distance = np.array(precursor.distance, dtype=float, copy=True)
    precursor_speed = np.array(precursor.speed, dtype=float, copy=True)
    precursor_distance.setflags(write=False)
    precursor_speed.setflags(write=False)
    precursor_curvature = np.array(precursor.curvature, dtype=float, copy=True)
    precursor_acceleration = np.array(precursor.acceleration, dtype=float, copy=True)
    precursor_curvature.setflags(write=False)
    precursor_acceleration.setflags(write=False)
    return FrozenPlan(frozen.distance, frozen.speed, frozen.curvature, frozen.acceleration,
                      frozen.digest, precursor_distance, precursor_speed, origin,
                      precursor_curvature, precursor_acceleration)


def _reconstruct_crop(precursor_arrays, crop_origin):
    distance = precursor_arrays["precursor_distance"]
    speed = precursor_arrays["precursor_speed"]
    curvature = precursor_arrays["precursor_curvature"]
    if (not np.all(np.isfinite(crop_origin)) or crop_origin < 0 or
            np.any(np.diff(distance) <= 0) or np.any(speed < 0) or
            np.any(np.abs(curvature) > 1.e-14)):
        raise ValueError("invalid precursor/crop metadata")
    if speed[0] == INITIAL_SPEED_MPS:
        expected_origin = 0.
    else:
        crossings = np.flatnonzero((speed[:-1] < INITIAL_SPEED_MPS) & (speed[1:] >= INITIAL_SPEED_MPS))
        if not crossings.size:
            raise ValueError("frozen precursor has no 6 m/s crop crossing")
        i = int(crossings[0])
        fraction = (INITIAL_SPEED_MPS - speed[i]) / (speed[i + 1] - speed[i])
        expected_origin = float(distance[i] + fraction * (distance[i + 1] - distance[i]))
    if crop_origin != expected_origin or crop_origin + DISTANCE_M > distance[-1]:
        raise ValueError("crop origin/extent does not match frozen precursor")
    end = crop_origin + DISTANCE_M
    stations = np.r_[crop_origin, distance[(distance > crop_origin) & (distance < end)], end]
    speeds = np.interp(stations, distance, speed)
    speeds[0] = INITIAL_SPEED_MPS
    curvatures = np.interp(stations, distance, curvature)
    from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
    cropped = SpeedPlan(stations - crop_origin, speeds, curvatures, is_closed=False)
    return freeze_plan_arrays(cropped.distance, cropped.speed, cropped.curvature, cropped.acceleration)


def _speed_plan(frozen):
    from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
    plan = SpeedPlan(frozen.distance, frozen.speed, frozen.curvature, is_closed=False)
    reconstructed = np.asarray(plan.acceleration, dtype=float)
    if not np.array_equal(reconstructed, frozen.acceleration):
        raise ValueError("reconstructed SpeedPlan acceleration differs from frozen artifact")
    return plan


def _canonical_config(compiled_manifest_text=""):
    return {"model": str(FORMULATION), "compiled_manifest": compiled_manifest_text,
        "plan_settings": {"precursor_length_m": PRECURSOR_DISTANCE_M, "course_length_m": DISTANCE_M,
            "curvature": 0., "maximum_speed_mps": MAXIMUM_SPEED_MPS,
            "initial_speed_mps": INITIAL_SPEED_MPS,
            "type": "cropped open standing-start live-limit profile; not launch"},
        "driver": dict(DRIVER_GAINS),
        "solver": {"duration_s": 300., "wall_seconds": SOLVER_WALL_SECONDS,
            "maximum_wall_seconds": SOLVER_WALL_SECONDS, "maximum_step": MAXIMUM_STEP,
            "rtol": RTOL, "atol": ATOL, "method": "Radau", "initial_pedal": INITIAL_PEDAL}}


def _config_digest(config):
    encoded = json.dumps(config, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def _json_digest(value):
    return _config_digest(value)


def _array_digest(array):
    contiguous = np.ascontiguousarray(np.asarray(array, dtype="<f8"))
    return hashlib.sha256(contiguous.tobytes()).hexdigest()


def _artifact_digest(manifest, arrays):
    return _json_digest({"manifest": manifest, "array_hashes": {
        name: _array_digest(value) for name, value in sorted(arrays.items())}})


def _sanitize_json(value):
    if isinstance(value, dict):
        return {key: _sanitize_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize_json(item) for item in value]
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        return None
    if isinstance(value, np.integer):
        return int(value)
    return value


def _exclusive_json(path, value):
    payload = (json.dumps(_sanitize_json(value), indent=2, allow_nan=False) + "\n").encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(fd, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _write_claimed_json(path, value):
    _exclusive_json(path, value)


def make_plant_class(vehicle):
    from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
    return PlanarForwardPlant


def make_driver(plan, vehicle, gains=None):
    from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
    return PreviewDriver(plan, vehicle.wheelbase, vehicle.steering_maximum_angle,
                         vehicle.steering_maximum_rate, vehicle.maximum_pedal_force,
                         **(DRIVER_GAINS if gains is None else gains),
                         pedal_travel_time=PEDAL_TRAVEL_TIME,
                         pedal_time_constant=PEDAL_TIME_CONSTANT, live_limit=True)


def make_mapper(vehicle, plan, names):
    from simplified_vehicle_models.solvers.physical_longitudinal_mapper import PhysicalLongitudinalMapper
    return PhysicalLongitudinalMapper(vehicle, plan, state_names=names)


class TraceSink:
    """Accepted-step telemetry using existing evaluation, without extra RHS physics."""
    def __init__(self, plant):
        self.plant, self.rows = plant, []

    def __call__(self, sample):
        observation = self.plant.observe(sample.state)
        requested = sample.requested_command or sample.command
        mapped = sample.mapped_command or requested
        dynamics, power = self.plant.last_evaluation, self.plant.last_powertrain
        mapped_stage = ("identity_no_mapper" if sample.mapped_command is None
                        else "physical_mapper")
        mapped_value = requested if sample.mapped_command is None else sample.mapped_command
        row = {"index": sample.index, "time": float(sample.time),
            "state": np.asarray(sample.state).tolist(), "rates": np.asarray(sample.rates).tolist(),
            "requested_throttle": float(requested.throttle), "mapped_stage": mapped_stage,
            "mapped_throttle": float(mapped_value.throttle),
            "pedal_lagged_applied_throttle": float(sample.command.throttle),
            "requested_brake_force": float(requested.brake_force),
            "mapped_brake_force": float(mapped_value.brake_force),
            "pedal_lagged_applied_brake_force": float(sample.command.brake_force),
            "steering_rate": float(sample.command.steering_rate),
            "target_speed": float(self.plant.plan.sample(observation.distance)[0]),
            "feedforward_acceleration": float(self.plant.plan.sample(observation.distance)[2]),
            "current_speed": float(observation.speed), "distance": float(observation.distance),
            "lateral_error": float(observation.lateral_error),
            "heading_error": float(observation.heading_error), "pedal": sample.pedal,
            "soc": float(sample.state[10]), "wheel_speeds": np.asarray(sample.state[6:10]).tolist()}
        if dynamics is not None:
            for field in ("longitudinal_acceleration", "lateral_acceleration", "yaw_acceleration",
                          "normal_loads", "wheel_accelerations", "slip_ratios", "slip_angles", "drag_force"):
                value = getattr(dynamics, field, None)
                if value is not None:
                    row[field] = np.asarray(value).tolist() if np.ndim(value) else float(value)
        if power is not None:
            for field in ("current", "voltage", "bus_power", "wheel_torques",
                          "motor_wheel_torques", "power_residual", "limited"):
                value = getattr(power, field, None)
                if value is not None:
                    row[field] = np.asarray(value).tolist() if np.ndim(value) else float(value)
        self.rows.append(row)


def prepare_comparison(output_dir, configuration=None):
    """Freeze model/plan provenance only; does not run vehicle simulation."""
    started = time.perf_counter()
    if configuration not in (None, {}):
        raise ValueError("runner configuration is fixed; unknown or overridden settings are rejected")
    compiled = compile_selected_model()
    manifest = collect_source_manifest(compiled.manifest)
    plan_data = make_plan(compiled.model)
    directory = create_output_directory(output_dir)
    for name, array in (("distance", plan_data.distance), ("speed", plan_data.speed),
                        ("curvature", plan_data.curvature), ("acceleration", plan_data.acceleration),
                        ("precursor_distance", plan_data.precursor_distance),
                        ("precursor_speed", plan_data.precursor_speed),
                        ("precursor_curvature", plan_data.precursor_curvature),
                        ("precursor_acceleration", plan_data.precursor_acceleration)):
        if array is not None:
            np.save(directory / f"plan_{name}.npy", array, allow_pickle=False)
    config = _canonical_config(compiled.manifest.as_text())
    data = {**config, "config_digest": _config_digest(config),
        "sources": manifest, "plan_hash": plan_data.digest,
        "plan_arrays_hashes": {name: hashlib.sha256(np.asarray(getattr(plan_data, name), dtype="<f8").tobytes()).hexdigest()
                                for name in ("distance", "speed", "curvature", "acceleration")},
        "crop_origin_m": plan_data.crop_origin_m,
        "tire_law": "steady combined-slip UniTire with dynamic wheel/body states; not transient tire relaxation",
        "archived_ttc_lap_equivalence": False,
        "setup_seconds": time.perf_counter() - started,
        "runtime": manifest["runtime"]}
    data["artifact_digest"] = _artifact_digest(dict(data), {
        **{name: getattr(plan_data, name) for name in ("distance", "speed", "curvature", "acceleration")},
        **{"precursor_" + name: getattr(plan_data, "precursor_" + name)
           for name in ("distance", "speed", "curvature", "acceleration")}})
    _exclusive_json(directory / "manifest.json", data)
    return directory


def _run_arm(run_dir, arm):
    """Run exactly one arm; caller provides external 360 s process timeout."""
    if arm not in ("heuristic", "physical_mapper"):
        raise ValueError("arm must be heuristic or physical_mapper")
    started = time.perf_counter()
    directory = Path(run_dir)
    prepared = json.loads((directory / "manifest.json").read_text())
    for name in ("distance", "speed", "curvature", "acceleration"):
        array = np.load(directory / f"plan_{name}.npy", allow_pickle=False)
        if name == "distance": arrays = [array]
        else: arrays.append(array)
    plan_data = freeze_plan_arrays(*arrays)
    precursor_arrays = {}
    for name in ("precursor_distance", "precursor_speed", "precursor_curvature", "precursor_acceleration"):
        values = np.load(directory / f"plan_{name}.npy", allow_pickle=False)
        if values.ndim != 1 or values.size < 2 or not np.all(np.isfinite(values)):
            raise ValueError(f"invalid frozen precursor array: {name}")
        precursor_arrays[name] = values
    if (len({value.shape for value in precursor_arrays.values()}) != 1 or
            np.any(np.diff(precursor_arrays["precursor_distance"]) <= 0)):
        raise ValueError("frozen precursor arrays do not have valid matching shapes/order")
    recomputed_crop = _reconstruct_crop(precursor_arrays, prepared.get("crop_origin_m", float("nan")))
    if recomputed_crop.digest != plan_data.digest:
        raise ValueError("frozen selected plan is not the recorded precursor crop")
    complete_arrays = {name: getattr(plan_data, name) for name in ("distance", "speed", "curvature", "acceleration")}
    complete_arrays.update({name: precursor_arrays[name]
                            for name in ("precursor_distance", "precursor_speed", "precursor_curvature", "precursor_acceleration")})
    artifact_body = dict(prepared)
    supplied_artifact_digest = artifact_body.pop("artifact_digest", None)
    if supplied_artifact_digest != _artifact_digest(artifact_body, complete_arrays):
        raise ValueError("prepared artifact digest mismatch")
    if plan_data.digest != prepared["plan_hash"]:
        raise ValueError("plan hash does not match prepared manifest")
    expected_hashes = prepared.get("plan_arrays_hashes", {})
    actual_hashes = {name: hashlib.sha256(np.asarray(getattr(plan_data, name), dtype="<f8").tobytes()).hexdigest()
                     for name in ("distance", "speed", "curvature", "acceleration")}
    if expected_hashes != actual_hashes:
        raise ValueError("plan array hashes do not match prepared manifest")
    compiled = compile_selected_model()
    expected_config = _canonical_config(compiled.manifest.as_text())
    allowed_keys = set(expected_config) | {"config_digest", "sources", "plan_hash", "plan_arrays_hashes",
        "crop_origin_m", "tire_law", "archived_ttc_lap_equivalence", "setup_seconds", "runtime",
        "artifact_digest"}
    if (set(prepared) != allowed_keys or
            any(prepared.get(key) != value for key, value in expected_config.items()) or
            prepared.get("config_digest") != _config_digest(expected_config)):
        raise ValueError("prepared complete configuration is missing, modified, or incompatible")
    current_manifest = collect_source_manifest(compiled.manifest)
    require_matching_manifests(prepared["sources"], current_manifest)
    if (directory / f"{arm}.json").exists():
        raise FileExistsError(f"arm result already exists: {arm}")
    vehicle = compiled.model
    plan = _speed_plan(plan_data)
    control = getattr(vehicle, "planar", vehicle)
    plant = make_plant_class(vehicle)(vehicle, plan)
    driver = make_driver(plan, control, prepared["driver"])
    initial = plant.initial_state(INITIAL_SPEED_MPS)
    mapper = None if arm == "heuristic" else make_mapper(vehicle, plan, plant.state_names)
    sink = TraceSink(plant)
    settings = prepared["solver"]
    kwargs = {"duration": settings["duration_s"], "wall_seconds": settings["wall_seconds"],
        "maximum_wall_seconds": settings["maximum_wall_seconds"], "maximum_step": settings["maximum_step"],
        "rtol": settings["rtol"], "atol": settings["atol"], "method": settings["method"],
        "initial_pedal": settings["initial_pedal"], "step_sink": sink, "command_mapper": mapper}
    summary = {"arm": arm, "status": "starting", "plan_hash": plan_data.digest,
        "manifest": prepared["sources"], "solver_settings": {k: v for k, v in kwargs.items()
            if k not in ("step_sink", "command_mapper")}, "initial_state": initial.tolist(),
        "initial_pedal": INITIAL_PEDAL, "driver": prepared["driver"],
        "mapper": "absent" if mapper is None else
            "physical quasi-steady mapper; aggregate counters include trial/validation callbacks"}
    # Persist a starting marker first; interruption/termination cannot masquerade as completion.
    result_path = directory / f"{arm}.json"
    _exclusive_json(directory / f"{arm}.started.json", summary)
    try:
        result = solve_forward(plant, driver, initial, **kwargs)
        if len(sink.rows) != len(result.time):
            raise RuntimeError("accepted telemetry row count differs from solver accepted samples")
        summary.update(status="completed", wall_seconds=float(result.wall_seconds),
                       accepted_samples=len(sink.rows), telemetry=sink.rows,
                       diagnostics=result.diagnostics)
        if len(sink.rows) > 1:
            summary["metrics"] = interval_weighted_metrics({
                "time": [r["time"] for r in sink.rows],
                "throttle": [r["pedal_lagged_applied_throttle"] for r in sink.rows],
                "speed_deficit": [r["target_speed"] - r["current_speed"] for r in sink.rows]})
    except Exception as error:
        summary.update(status="failed", error=f"{type(error).__name__}: {error}",
            accepted_samples=len(sink.rows), telemetry=sink.rows,
            prefix_kind="accepted step-sink prefix; integration failure does not assert accepted-state validation",
            elapsed_seconds=time.perf_counter() - started)
    if mapper is not None:
        summary["mapper_counters"] = mapper.snapshot_diagnostics()
    try:
        _write_claimed_json(result_path, summary)
    except Exception as error:
        # Keep all accepted rows even if a terminal metric/trace value is invalid.
        fallback = {"arm": arm, "status": "failed", "error": f"terminal serialization failed: {type(error).__name__}: {error}",
                    "accepted_samples": len(sink.rows), "telemetry": sink.rows,
                    "prefix_kind": "accepted step-sink prefix preserved; nonfinite values normalized to null",
                    "elapsed_seconds": time.perf_counter() - started}
        if mapper is not None:
            fallback["mapper_counters"] = mapper.snapshot_diagnostics()
        _write_claimed_json(result_path, fallback)
        return fallback
    return summary


def run_arm(run_dir, arm):
    """Run one arm and persist a structured failure even for setup exceptions."""
    if arm not in ("heuristic", "physical_mapper"):
        raise ValueError("arm must be heuristic or physical_mapper")
    directory = Path(run_dir)
    if not directory.is_dir():
        raise FileNotFoundError(f"prepared run directory does not exist: {directory}")
    claim = directory / f"{arm}.claim.json"
    path = directory / f"{arm}.json"
    if any(candidate.exists() for candidate in (path, claim, directory / f"{arm}.started.json")):
        raise FileExistsError(f"arm evidence already exists: {arm}")
    _exclusive_json(claim, {"arm": arm, "status": "claimed", "started_at": time.time()})
    try:
        return _run_arm(run_dir, arm)
    except Exception as error:
        failure = {"arm": arm, "status": "failed", "error": f"{type(error).__name__}: {error}",
                   "accepted_samples": 0, "telemetry": [],
                   "prefix_kind": "none; failure before solve sink was established"}
        try:
            _write_claimed_json(path, failure)
        except FileExistsError:
            raise
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser("prepare")
    prepare.add_argument("--output", type=Path, required=True)
    arm = commands.add_parser("arm")
    arm.add_argument("run_dir", type=Path)
    arm.add_argument("arm", choices=("heuristic", "physical_mapper"))
    args = parser.parse_args(argv)
    if args.command == "prepare":
        print(prepare_comparison(args.output))
        return 0
    result = run_arm(args.run_dir, args.arm)
    print(json.dumps({"arm": args.arm, "status": result["status"],
                      "accepted_samples": result["accepted_samples"]}))
    return 0 if result["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
