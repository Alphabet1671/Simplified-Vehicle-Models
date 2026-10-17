"""Run an isolated forward-driver attempt under a parent wall deadline."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib
import json
import math
from numbers import Real
from pathlib import Path
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ElementTree

import numpy as np

#: Canonical ten-lap MIS endurance track: the active default for new runs.
DEFAULT_TRACK = Path("models/tracks/2024_michigan_endurance_multilap_closed.xml")
#: Track of every attempt that predates --track (no settings["track"] recorded).
#: Historical warm-start matching must keep resolving to this, not DEFAULT_TRACK.
LEGACY_TRACK = Path("models/tracks/2024_michigan_endurance_closed.xml")
#: Planar 7DoF formulation. ``FORMULATION`` is the historical name other tools
#: import; it keeps meaning the planar formulation.
PLANAR_FORMULATION = Path("models/formulations/iem24_murphy_real_planar_7dof.xml")
FORMULATION = PLANAR_FORMULATION
#: Vertical 14DoF formulation over the same shared assembly.
VERTICAL_FORMULATION = Path("models/formulations/iem24_murphy_real_14dof_simple.xml")
#: Shared IEM24 assembly both formulations compile.
VEHICLE = Path("models/vehicles/iem24_murphy_real_planar.xml")
#: The established 7DoF acceptance limit; also the default wall ceiling of
#: ``solve_forward``. A longer wall budget needs ``--model 14dof-simple``.
PLANAR_MAXIMUM_WALL_SECONDS = 300.
#: Maximum path spacing: a coarser plan is not a resolution the driver may run.
MAXIMUM_PATH_SPACING_METRES = .5

#: Sources every model depends on. The powertrain closure moved to
#: ``forward_powertrain.py`` in Task 7 and both plants call it, so both hash it:
#: for the planar rung that changes recorded provenance, not any value.
_SOLVER_SOURCES = (
    Path("src/simplified_vehicle_models/solvers/forward_driver.py"),
    Path("src/simplified_vehicle_models/solvers/forward_powertrain.py"),
)
_PLANAR_SOURCES = _SOLVER_SOURCES + (
    Path("src/simplified_vehicle_models/solvers/planar_forward.py"),
    Path("src/simplified_vehicle_models/solvers/dense_domain.py"),
    Path("src/simplified_vehicle_models/solvers/planar_dense_certificate.py"),
    Path("src/simplified_vehicle_models/solvers/load_interval.py"),
    Path("src/simplified_vehicle_models/formulations/planar_7dof.py"),
    Path("src/simplified_vehicle_models/tires/unitire_simple.py"),
)
_VERTICAL_SOURCES = _SOLVER_SOURCES + (
    Path("src/simplified_vehicle_models/solvers/vertical_forward.py"),
    Path("src/simplified_vehicle_models/solvers/vertical_equilibrium.py"),
    Path("src/simplified_vehicle_models/formulations/vertical_14dof.py"),
    Path("src/simplified_vehicle_models/tires/unitire_transient.py"),
    Path("src/simplified_vehicle_models/tires/vertical.py"),
    Path("src/simplified_vehicle_models/tires/unitire_simple.py"),
    Path("src/simplified_vehicle_models/suspension/corner.py"),
    Path("src/simplified_vehicle_models/suspension/damper_table.py"),
    Path("src/simplified_vehicle_models/suspension/monotone_spline.py"),
)


@dataclass(frozen=True)
class ModelSpec:
    """One forward model: formulation, plant, hashed sources and wall policy."""

    name: str
    formulation: Path
    vehicle: Path
    plant_module: str
    plant_class: str
    solver_sources: tuple[Path, ...]
    #: Extra assembly roles whose component XML and data tables define the run.
    component_roles: tuple[str, ...]
    #: True when the planar dense-domain certificate and its load-interval
    #: contact conditions apply (planar-7dof only).
    planar_dense_certificate: bool
    #: Fraction of the parent timeout held back from the solver, and its floor.
    wall_margin_fraction: float
    wall_margin_floor_seconds: float = 5.

    @property
    def plant(self):
        """Return this model's plant class, imported on first use."""
        return getattr(importlib.import_module(self.plant_module), self.plant_class)


MODELS: dict[str, ModelSpec] = {
    "planar-7dof": ModelSpec(
        name="planar-7dof",
        formulation=PLANAR_FORMULATION,
        vehicle=VEHICLE,
        plant_module="simplified_vehicle_models.solvers.planar_forward",
        plant_class="PlanarForwardPlant",
        solver_sources=_PLANAR_SOURCES,
        component_roles=("tire",),
        planar_dense_certificate=True,
        wall_margin_fraction=0.,
    ),
    "14dof-simple": ModelSpec(
        name="14dof-simple",
        formulation=VERTICAL_FORMULATION,
        vehicle=VEHICLE,
        plant_module="simplified_vehicle_models.solvers.vertical_forward",
        plant_class="VerticalForwardPlant",
        solver_sources=_VERTICAL_SOURCES,
        component_roles=("tire", "tire-transient", "suspension",
                         "damper-front", "damper-rear"),
        planar_dense_certificate=False,
        wall_margin_fraction=.02,
    ),
}
#: Model names in CLI order.
MODEL_NAMES: tuple[str, ...] = tuple(MODELS)
#: Plant state-name patterns that expose the four wheel speeds, most specific
#: first: the planar plant names them ``omega_*``, the vertical plant
#: ``wheel_speed_*``.
_WHEEL_SPEED_PATTERNS = ("omega_{}", "wheel_speed_{}")
_WHEELS = ("fl", "fr", "rl", "rr")


def model_spec(model: str) -> ModelSpec:
    """Return the specification of one forward model."""
    try:
        return MODELS[model]
    except KeyError:
        raise ValueError(
            f"unknown forward model {model!r}; expected one of {MODEL_NAMES}") from None


def wall_margin_seconds(timeout_seconds: float, model: str) -> float:
    """Return the parent-timeout margin held back from the solver.

    ``planar-7dof`` keeps its established flat 5 s margin. ``14dof-simple``
    holds back the larger of 2 % of the parent timeout and 5 s, because its
    setup, accepted-state replay and reporting are proportionally heavier.
    """
    spec = model_spec(model)
    return max(spec.wall_margin_fraction * float(timeout_seconds),
               spec.wall_margin_floor_seconds)


def solver_wall_budget(timeout_seconds: float, model: str) -> float:
    """Return the wall budget handed to ``solve_forward`` for one model."""
    return float(timeout_seconds) - wall_margin_seconds(timeout_seconds, model)


def canonical_path(path):
    """Cwd-relative POSIX-style path when inside the cwd, else absolute."""
    resolved = Path(path).expanduser().resolve()
    try:
        return resolved.relative_to(Path.cwd().resolve())
    except ValueError:
        return resolved


def component_xml(vehicle_xml, role):
    """Resolve one component role of an assembly to its XML path."""
    vehicle_xml = Path(vehicle_xml)
    root = ElementTree.parse(vehicle_xml).getroot()
    for component in root.findall("./components/component"):
        if component.get("role") == role:
            return canonical_path(vehicle_xml.parent / component.get("ref"))
    raise ValueError(f"{vehicle_xml}: no component with role {role!r}")


def component_data_paths(component_xml_path):
    """Return every external data file one component XML references."""
    component_xml_path = Path(component_xml_path)
    root = ElementTree.parse(component_xml_path).getroot()
    return tuple(canonical_path(component_xml_path.parent / data.get("path"))
                 for data in root.iter("data") if data.get("path"))


def model_inputs(model: str, track_xml):
    """Return the named physical inputs of one model: role name -> path.

    The track, its referenced CSV, the vehicle assembly, the formulation and
    the steady tire XML are always present. A model with extra component roles
    (the vertical 14DoF suspension, both dampers and the transient tire) also
    records each of those component XMLs and every data table they reference.
    """
    spec = model_spec(model)
    track_xml, track_csv = track_files(track_xml)
    inputs = {"track_xml": track_xml, "track_csv": track_csv,
              "tire_xml": component_xml(spec.vehicle, "tire"),
              "formulation_xml": spec.formulation, "vehicle_xml": spec.vehicle}
    for role in spec.component_roles:
        if role == "tire":
            continue  # already recorded as ``tire_xml``
        xml = component_xml(spec.vehicle, role)
        key = role.replace("-", "_")
        inputs[f"{key}_xml"] = xml
        for index, data in enumerate(component_data_paths(xml)):
            inputs[f"{key}_data" if index == 0 else f"{key}_data_{index}"] = data
    return inputs


def source_files(model: str, track_xml):
    """Return every file hashed and snapshotted for one run, in order.

    The file names must stay unique because the attempt's ``source_snapshot``
    copies each source by base name.
    """
    spec = model_spec(model)
    sources = [Path(__file__), *spec.solver_sources, *model_inputs(model, track_xml).values()]
    unique = []
    for path in sources:
        if path not in unique:
            unique.append(path)
    names = [path.name for path in unique]
    if len(set(names)) != len(names):
        raise ValueError("source snapshot file names must be unique")
    return tuple(unique)


def trace_header(plant):
    """Return the trace CSV header: ``time`` then the plant's state layout."""
    names = tuple(plant.state_names)
    if len(names) != plant.state_size or len(set(names)) != len(names):
        raise ValueError("plant state names must be unique and match its state size")
    return ("time",) + names


def validated_periodic_tolerances(plant):
    """Return the plant's periodic tolerances after checking them for the lap.

    A plant that publishes a tolerance vector whose length does not match its
    periodic index list cannot define a closed-lap comparison, so this raises
    instead of silently zipping to the shorter vector. The returned array is a
    copy, so a caller can scale or clip it without mutating the plant class.
    """
    indices = tuple(plant.periodic_indices)
    tolerances = np.array(plant.periodic_tolerances, dtype=float, copy=True)
    if tolerances.shape != (len(indices),):
        raise ValueError("plant periodic tolerances must match its periodic indices")
    if not np.all(np.isfinite(tolerances)) or np.any(tolerances < 0.):
        raise ValueError("plant periodic tolerances must be finite and nonnegative")
    for index in indices:
        if not isinstance(index, (int, np.integer)) or not 0 <= int(index) < plant.state_size:
            raise ValueError("plant periodic indices must lie inside the state layout")
    return tolerances


def state_index(plant, name: str) -> int:
    """Return the index of one named plant state."""
    try:
        return tuple(plant.state_names).index(name)
    except ValueError:
        raise ValueError(f"plant has no state named {name!r}") from None


def wheel_speed_indices(plant):
    """Return the indices of the four wheel-speed states, in FL, FR, RL, RR order."""
    names = tuple(plant.state_names)
    for pattern in _WHEEL_SPEED_PATTERNS:
        candidate = tuple(pattern.format(wheel) for wheel in _WHEELS)
        if all(name in names for name in candidate):
            return tuple(names.index(name) for name in candidate)
    raise ValueError("plant state names do not expose four wheel speeds")


def jounce_indices(plant):
    """Return the indices of the four jounce states, in FL, FR, RL, RR order."""
    names = tuple(plant.state_names)
    candidate = tuple(f"jounce_{wheel}" for wheel in _WHEELS)
    if not all(name in names for name in candidate):
        raise ValueError("plant state names do not expose four jounce states")
    return tuple(names.index(name) for name in candidate)


def accepted_jounce_extremes(result, plant):
    """Return ``(minimum, maximum, scope)`` jounce over the accepted trajectory.

    The plant's own ``minimum_jounce``/``maximum_jounce`` are diagnostics over
    *every* evaluation, including rejected Newton trials, and a trial may
    legally sit inside the vertical plant's ±10 mm trial margin around the
    travel limits. The travel gate must therefore read the accepted states of
    ``result`` (and, when the method produced a verifiable interpolant, the
    per-interval dense bounds that cover every midpoint).
    """
    indices = list(jounce_indices(plant))
    values = np.asarray(result.state, dtype=float)[indices]
    minimum, maximum = float(np.min(values)), float(np.max(values))
    scope = "accepted states only (no verifiable dense interpolant)"
    bounds = getattr(result, "dense_bounds", None)
    if bounds is not None:
        bounds = np.asarray(bounds, dtype=float)[:, indices, :]
        minimum = min(minimum, float(np.min(bounds[:, :, 0])))
        maximum = max(maximum, float(np.max(bounds[:, :, 1])))
        scope = ("accepted states and the per-interval dense bounds, which cover "
                 "every midpoint")
    return minimum, maximum, scope


def jounce_travel_limits(vehicle):
    """Return the ``[lower, upper]`` jounce travel every corner must satisfy.

    ``vehicle`` is the compiled vertical 14DoF model. The interval is the
    intersection of the front and rear axle limits, so a state inside it is
    inside every corner's travel without needing per-corner extremes.
    """
    axles = (vehicle.suspension.front.jounce_limits, vehicle.suspension.rear.jounce_limits)
    return (max(float(limits[0]) for limits in axles),
            min(float(limits[1]) for limits in axles))


def finite_or_none(value):
    """Return ``value`` as a float, or None when it is not finite.

    A diagnostic that was never updated (an attempt aborted before its first
    successful plant evaluation) still holds its ``±inf`` sentinel, which
    ``json.dumps(..., allow_nan=False)`` rejects. Mapping it to None keeps the
    summary strict and still written.
    """
    value = float(value)
    return value if math.isfinite(value) else None


#: DOI recorded in a hashed model XML (for example the transient tire component).
_DOI_PATTERN = re.compile(r"DOI\s+(10\.\d{4,9}/[^\s,;)]+)")


def recorded_doi(path):
    """Return the DOI recorded in one model XML, or None when it records none."""
    match = _DOI_PATTERN.search(Path(path).read_text(encoding="utf-8"))
    return match.group(1) if match else None


def vertical_model_provenance(vehicle, inputs, hashes):
    """Return the machine-readable provenance of the vertical tire layer.

    The full parameter table, the reduced-model limits and the paper discussion
    belong to Task 9's ``docs/validation/14dof-simple-parameters.md``; this
    records the essentials with the attempt: the hashed tire XMLs, the DOI as
    recorded in the transient XML, and the PROPOSED coefficients actually
    compiled.
    """
    transient_xml, steady_xml = inputs["tire_transient_xml"], inputs["tire_xml"]
    parameters = vehicle.transient_tire
    return {
        "steady_tire_xml": str(steady_xml),
        "steady_tire_xml_sha256": hashes[str(steady_xml)],
        "transient_tire_xml": str(transient_xml),
        "transient_tire_xml_sha256": hashes[str(transient_xml)],
        "transient_tire_doi": recorded_doi(transient_xml),
        "transient_tire_parameters": {
            name: float(getattr(parameters, name))
            for name in ("longitudinal_length_scale", "lateral_length_scale",
                         "trail_pole_ratio", "trail_lead_ratio")},
        "note": ("PROPOSED transient coefficients, not fitted to this tire; equations, "
                 "reduced-model limits and the full parameter table are Task 9's "
                 "docs/validation/14dof-simple-parameters.md"),
    }


def peak_memory_bytes():
    """Return this process's peak resident set size in bytes, or None.

    ``resource.getrusage(RUSAGE_SELF).ru_maxrss`` is reported in bytes on macOS
    and in kibibytes on Linux, so the raw value is converted here. The value is
    a process high-water mark, not the solver's allocation alone.
    """
    try:
        import resource
    except ImportError:  # pragma: no cover - resource is unavailable on Windows
        return None
    value = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    if not math.isfinite(value) or value < 0.:
        return None
    return value if sys.platform == "darwin" else value * 1024.


def track_files(track_xml):
    """Return (track XML, referenced CSV) paths; raise ValueError if unusable.

    Only a signed ``closed-xy-profile`` track is accepted; a legacy
    distance-radius profile has no turn direction and cannot drive the 7DoF.
    """
    track_xml = Path(track_xml)
    try:
        root = ElementTree.parse(track_xml).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read track XML {track_xml}: {error}") from error
    if root.get("type") != "track-model" or root.get("subtype") != "closed-xy-profile":
        raise ValueError(f"{track_xml}: track must be a signed closed-xy-profile track-model")
    data = root.find("./data")
    if data is None or not data.get("ref"):
        raise ValueError(f"{track_xml}: missing track data reference")
    csv_path = (track_xml.parent / data.get("ref")).resolve()
    if not csv_path.is_file():
        raise ValueError(f"{track_xml}: referenced track CSV {csv_path} not found")
    return track_xml, canonical_path(csv_path)


def tire_xml_path(vehicle_xml=VEHICLE, role="tire"):
    """Component XML of one tire role referenced by the vehicle assembly."""
    return component_xml(vehicle_xml, role)


def warm_start_matches(previous, source_hashes, args):
    """True when a previous attempt may seed this run (same plan, track and sources).

    Source hashes include the track XML/CSV, tire XML and formulation, so a
    historical attempt without them cannot warm-start a track-aware run.
    """
    settings = previous.get("settings", {})
    return bool(previous.get("status") == "Completed" and previous.get("lap_completed")
                and previous.get("source_hashes") == source_hashes
                and all(settings.get(key) == getattr(args, key)
                        for key in ("planner", "maximum_speed", "lateral_acceleration",
                                    "pedal_travel_time", "spacing"))
                and canonical_path(settings.get("track") or LEGACY_TRACK)
                == canonical_path(args.track))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=MODEL_NAMES, default="planar-7dof",
                        help="Forward model: the planar 7DoF rung (300 s wall limit) or the "
                             "vertical 14DoF rung (a longer, user-selected wall limit)")
    parser.add_argument("--output-root", type=Path,
        default=Path("docs/validation/iem24-murphy/forward_driver_trials"))
    parser.add_argument("--timeout-seconds", type=float, default=300.)
    parser.add_argument("--duration", type=float, default=300.)
    parser.add_argument("--maximum-speed", type=float, default=8.)
    parser.add_argument("--lateral-acceleration", type=float, default=None,
                        help="Cornering cap (m/s^2). Conservative planner default 2; "
                             "live planner default: no cap beyond the vehicle's own limits")
    parser.add_argument("--pedal-travel-time", type=float, default=.175,
                        help="Seconds for a full pedal travel (0-100%% throttle or 0-max brake)")
    parser.add_argument("--maximum-step", type=float, default=.02)
    parser.add_argument("--rtol", type=float, default=1.e-6)
    parser.add_argument("--method", choices=("Radau", "BDF"), default="Radau")
    parser.add_argument("--spacing", type=float, default=.5)
    parser.add_argument("--planner", choices=("conservative", "live"), default="conservative",
                        help="Speed planner: conservative (constant limits) or live (point-mass)")
    parser.add_argument("--track", type=Path, default=DEFAULT_TRACK,
                        help="Signed closed-xy-profile track XML (default: the canonical "
                             f"ten-lap MIS track {DEFAULT_TRACK})")
    parser.add_argument("--warm-start", type=Path,
                        help="Completed previous attempt whose final physical/pedal state starts this lap")
    parser.add_argument("--record", action="store_true",
                        help="also write run_record.npz: the full vehicle state, state rate, "
                             "realized controls and tire/attitude channels at every accepted "
                             "step (input of tools/anim/export_animation.py)")
    parser.add_argument("--record-precision", choices=("float32", "float64"),
                        default="float32", help="payload precision of run_record.npz")
    parser.add_argument("--quasi-static-attitude", action="store_true",
                        help="with --record on a planar model, also store the labelled "
                             "APPROXIMATE roll/pitch estimates (never simulated states)")
    parser.add_argument("--child-attempt", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    import math
    for name in ("timeout_seconds", "duration", "maximum_speed", "lateral_acceleration",
                 "maximum_step", "rtol", "spacing", "pedal_travel_time"):
        value = getattr(args, name)
        if value is None:
            continue
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")
    try:
        track_files(args.track)
    except ValueError as error:
        parser.error(f"--track: {error}")
    args.track = canonical_path(args.track)
    if args.warm_start is not None and args.pedal_travel_time is None:
        parser.error("warm start requires pedal dynamics")
    if args.quasi_static_attitude and not args.record:
        parser.error("--quasi-static-attitude needs --record")
    if args.spacing > MAXIMUM_PATH_SPACING_METRES:
        parser.error(f"path spacing must be <= {MAXIMUM_PATH_SPACING_METRES} m")
    if args.timeout_seconds > PLANAR_MAXIMUM_WALL_SECONDS and args.model == "planar-7dof":
        parser.error(f"the planar-7dof wall limit must be <= {PLANAR_MAXIMUM_WALL_SECONDS:g} s; "
                     "use --model 14dof-simple for a longer, user-selected wall limit")
    return args


#: File name of the optional run record inside an attempt directory.
RUN_RECORD_FILE = "run_record.npz"


def write_run_record(sink, args, summary, physical_inputs, spec, result, initial_pedal):
    """Close the sink into ``run_record.npz`` and note it in the summary.

    The manifest hashes only the physical inputs (track, tire, formulation,
    vehicle and component tables). Solver source is deliberately left out: it
    changes with every commit, and the record's values are already final.
    """
    import datetime
    import platform

    import scipy

    from simplified_vehicle_models.results.plant_channels import source_hashes

    def relative(path):
        path = Path(path)
        try:
            return path.resolve().relative_to(Path.cwd().resolve())
        except ValueError:
            return path

    record_path = sink.close({
        "producer": {"tool": "tools/run/run_iem24_forward_driver.py",
                     "python": platform.python_version(), "numpy": np.__version__,
                     "scipy": scipy.__version__},
        "model": {"model": args.model, "plant_class": spec.plant_class,
                  "formulation_xml": str(relative(spec.formulation)),
                  "mechanical_dof": 7 if spec.planar_dense_certificate else 14},
        "sources": source_hashes(relative(path) for path in physical_inputs.values()),
        "integrator": {"method": args.method, "rtol": args.rtol, "atol": args.rtol * .01,
                       "maximum_step": args.maximum_step,
                       "wall_seconds": float(result.wall_seconds)},
        "segment": {"duration_s": float(result.time[-1] - result.time[0]),
                    "lap_completed": bool(result.lap_completed),
                    "warm_started": args.warm_start is not None,
                    "initial_pedal": initial_pedal},
        "track": {"file": str(args.track)},
    }, created_utc=datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"))
    summary["run_record_file"] = record_path.name
    summary["run_record_rows"] = int(sink.rows)


def acceptance_gate(summary):
    """Return true only when every required lap-validity gate is established."""
    delta = summary.get("periodic_state_delta")
    tolerances = summary.get("periodic_state_tolerances")
    def finite_real_vector(values):
        return (isinstance(values, (list, tuple)) and bool(values)
                and all(isinstance(value, Real) and math.isfinite(value)
                        for value in values))

    periodic = (summary.get("lap_completed") is True
                and finite_real_vector(delta)
                and finite_real_vector(tolerances)
                and len(delta) == len(tolerances)
                and all(tolerance >= 0 and abs(value) <= tolerance
                        for value, tolerance in zip(delta, tolerances)))
    return bool(periodic
                and summary.get("periodicity_valid") is True
                and summary.get("integration_completed") is True
                and summary.get("path_tracking_valid") is True
                and summary.get("resolution_valid") is True
                and summary.get("checked_rhs_physics_valid") is True
                and summary.get("between_step_validation_complete") is True
                and summary.get("refinement_validated") is True
                and summary.get("contact_valid") is True)


def child(args):
    started = time.perf_counter()
    root = args.child_attempt
    spec = model_spec(args.model)
    summary = {"status": "Starting", "lap_completed": False, "usable": False,
               "method": "transient_forward_driver", "minimum_time_optimum": False,
               "model": args.model, "requested_wall_seconds": args.timeout_seconds,
               "planner": f"{args.planner}_{'live' if args.planner == 'live' else 'conservative'}_acceleration_envelope",
               "settings": {k: str(v) if isinstance(v, Path) else v
                            for k, v in vars(args).items()}}
    plant = None
    try:
        import numpy as np
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.tracks.profile import load_track
        from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
        from simplified_vehicle_models.solvers.forward_driver import (
            build_speed_plan, PreviewDriver, solve_forward,
        )

        import hashlib
        import platform
        import scipy
        # Track, referenced CSV, tire XMLs, component tables and formulation
        # define the run's physical inputs; they are hashed and snapshotted with
        # the code that this model depends on.
        sources = source_files(args.model, args.track)
        physical_inputs = model_inputs(args.model, args.track)
        summary["source_hashes"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in sources}
        summary["model_inputs"] = {key: str(path) for key, path in physical_inputs.items()}
        snapshot = root / "source_snapshot"
        snapshot.mkdir()
        for path in sources:
            (snapshot / path.name).write_bytes(path.read_bytes())
        summary["environment"] = {"python": sys.version, "numpy": np.__version__,
                                  "scipy": scipy.__version__, "platform": platform.platform()}
        vehicle = compile_formulation(str(spec.formulation)).model
        # The vertical 14DoF model composes the planar parameters; the driver and
        # the live planner read the shared planar layer.
        control = getattr(vehicle, "planar", vehicle)
        if not spec.planar_dense_certificate:
            summary["model_provenance"] = vertical_model_provenance(
                vehicle, physical_inputs, summary["source_hashes"])
        track = load_track(args.track)
        count = int(np.ceil(track.lap_length / args.spacing))
        distance = np.linspace(0., track.lap_length, count + 1)
        _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
        live_limit = args.planner == "live"
        lateral_cap = args.lateral_acceleration
        if lateral_cap is None and not live_limit:
            lateral_cap = 2.
        plan = build_speed_plan(distance, curvature, args.maximum_speed,
                                lateral_cap, 1., 2.,
                                vehicle=control if live_limit else None,
                                live_limit=live_limit)
        plant_class = spec.plant
        # The plant publishes its layout; every column index below is derived
        # from it instead of being repeated as a literal.
        names = tuple(plant_class.state_names)
        header = trace_header(plant_class)

        class LoggedPlant(plant_class):
            next_log = 0.

            def evaluate(self, time_value, state, command):
                value = super().evaluate(time_value, state, command)
                elapsed = time.perf_counter() - started
                if elapsed >= self.next_log:
                    print(json.dumps({"wall_seconds": elapsed, "rhs_time": time_value,
                                      "rhs_distance": float(state[names.index("s")]),
                                      "rhs_lateral_error": float(state[names.index("n")]),
                                      "evaluations": self.evaluations}), flush=True)
                    self.next_log = elapsed + 5.
                return value

        plant = LoggedPlant(vehicle, plan)
        driver = PreviewDriver(plan, control.wheelbase, control.steering_maximum_angle,
                               control.steering_maximum_rate, control.maximum_pedal_force,
                               live_limit=live_limit,
                               pedal_travel_time=args.pedal_travel_time)
        initial = plant.initial_state(float(plan.speed[0]))
        initial_pedal = None
        if args.warm_start is not None:
            previous = json.loads((args.warm_start / "summary.json").read_text())
            previous_settings = previous.get("settings", {})
            if not warm_start_matches(previous, summary["source_hashes"], args):
                raise ValueError("warm start requires a completed lap with matching planner, speed cap, track and source hashes")
            trace = np.loadtxt(args.warm_start / "trace.csv", delimiter=",", skiprows=1)
            pedals = np.loadtxt(args.warm_start / "pedal.csv", delimiter=",", skiprows=1)
            if (trace.ndim != 2 or trace.shape[1] != 1 + plant.state_size
                    or pedals.shape != (len(trace), 2)
                    or not np.array_equal(trace[:, 0], pedals[:, 0])):
                raise ValueError("warm-start trace and pedal samples must match")
            initial = np.array(trace[-1, 1:], copy=True)
            initial[state_index(plant, "s")] = 0.
            initial_pedal = float(pedals[-1, 1])
            if not np.all(np.isfinite(initial)):
                raise ValueError("warm-start final state must be finite")
            summary["warm_start_source"] = str(args.warm_start)
        summary.update(wheel_inertia=control.wheel_inertia, state_size=plant.state_size,
                       track_length=track.lap_length, setup_seconds=time.perf_counter() - started)
        print(json.dumps(summary), flush=True)
        # The parent watchdog keeps the requested timeout; the solver gets a
        # slightly shorter budget so setup, accepted-state replay and reporting
        # fit inside the same parent deadline.
        budget = solver_wall_budget(args.timeout_seconds, args.model)
        summary["solver_wall_budget_seconds"] = budget
        remaining = budget - (time.perf_counter() - started)
        if remaining <= 0:
            raise TimeoutError("setup exhausted the wall budget")
        # The sink is passed only when recording, so a run without --record calls
        # solve_forward exactly as before.
        recording = {}
        sink = None
        if args.record:
            from simplified_vehicle_models.results.plant_channels import RunRecordSink
            sink = RunRecordSink(plant, track.path_coordinates, root / RUN_RECORD_FILE,
                                 precision=args.record_precision,
                                 quasi_static_attitude=args.quasi_static_attitude)
            recording["step_sink"] = sink
        result = solve_forward(plant, driver, initial, duration=args.duration,
                               wall_seconds=remaining,
                               maximum_wall_seconds=args.timeout_seconds,
                               maximum_step=args.maximum_step,
                               rtol=args.rtol, atol=args.rtol * .01, method=args.method,
                                initial_pedal=initial_pedal, **recording)
        summary["solver_elapsed_seconds"] = float(result.wall_seconds)
        if sink is not None:
            write_run_record(sink, args, summary, physical_inputs, spec, result, initial_pedal)
        np.savetxt(root / "trace.csv", np.column_stack((result.time, result.state.T)),
                   delimiter=",", comments="",
                   header=",".join(header))
        if result.pedal is not None:
            np.savetxt(root / "pedal.csv", np.column_stack((result.time, result.pedal)),
                       delimiter=",", comments="", header="time,pedal")
        periodic_indices = tuple(plant.periodic_indices)
        tolerances = validated_periodic_tolerances(plant)
        maximum_spacing = float(np.max(np.diff(result.state[state_index(plant, "s")])))
        path_error = float(np.max(np.abs(result.state[state_index(plant, "n")])))
        closure = (result.state[list(periodic_indices), -1]
                   - initial[list(periodic_indices)])
        periodic = bool(result.lap_completed and np.all(np.abs(closure) <= tolerances))
        summary.update(status="Completed" if result.lap_completed else "Duration_Limit",
                       lap_completed=result.lap_completed,
                       simulated_seconds=float(result.time[-1]),
                       distance_completed=float(result.state[state_index(plant, "s"), -1]),
                       lap_time=float(result.time[-1]) if result.lap_completed else None,
                       accepted_steps=result.time.size - 1, evaluations=result.evaluations,
                       rejected_domain_trials=result.rejected_domain_trials,
                       maximum_lateral_error=path_error, path_tracking_valid=path_error <= .25,
                       maximum_accepted_spatial_step=maximum_spacing,
                       resolution_valid=maximum_spacing <= MAXIMUM_PATH_SPACING_METRES,
                       periodicity_valid=periodic, periodic_state_delta=closure.tolist(),
                       periodic_state_tolerances=tolerances.tolist(),
                       integration_completed=True, checked_rhs_physics_valid=True,
                       midpoint_sample_validation_complete=result.midpoint_sample_validation_complete,
                        between_step_validation_complete=False, refinement_validated=False,
                       steering_peak=float(np.max(np.abs(
                           result.state[state_index(plant, "steering")]))),
                       minimum_wheel_speed=float(np.min(
                           result.state[list(wheel_speed_indices(plant))])),
                       final_soc=float(result.state[state_index(plant, "soc"), -1]),
                       planned_lap_time=float(np.sum(np.diff(plan.distance)
                                                     / (.5 * (plan.speed[1:] + plan.speed[:-1])))),
                       planned_peak_lateral_acceleration=float(np.max(
                           plan.speed**2 * np.abs(plan.curvature))))
        # A completed transient is not usable unless every independent acceptance
        # gate (including periodicity and between-step/refinement checks) passes.
        # Every accepted state has been checked by solve_forward. Zero contact
        # load is permitted (wheel lift); negative loads are rejected by plant.
        summary["minimum_contact_load_n"] = plant.minimum_contact_load
        if spec.planar_dense_certificate:
            summary["contact_valid"] = bool(result.lap_completed
                and math.isfinite(plant.minimum_contact_load)
                and plant.minimum_contact_load >= 0.)
        else:
            # The vertical plant has no inner load loop, so its contact evidence
            # is the accepted trajectory's minimum tire load, the table-domain
            # counter (damper, spring, bump-stop and jounce-steer exits) and the
            # travel bound. The travel gate reads the accepted states and the
            # dense interval bounds, never the plant's all-evaluation jounce
            # extremes: a rejected Newton trial may legally sit inside the
            # ±10 mm trial margin around the limits.
            lower, upper = jounce_travel_limits(vehicle)
            accepted_low, accepted_high, scope = accepted_jounce_extremes(result, plant)
            travel_valid = bool(accepted_low >= lower and accepted_high <= upper)
            summary["jounce_travel_limits_m"] = [lower, upper]
            summary["jounce_gate_scope"] = scope
            summary["jounce_accepted_minimum_m"] = accepted_low
            summary["jounce_accepted_maximum_m"] = accepted_high
            summary["jounce_evaluation_minimum_m"] = finite_or_none(plant.minimum_jounce)
            summary["jounce_evaluation_maximum_m"] = finite_or_none(plant.maximum_jounce)
            summary["out_of_range_evaluations"] = plant.out_of_range_evaluations
            summary["jounce_travel_valid"] = travel_valid
            summary["contact_valid"] = bool(result.lap_completed
                and math.isfinite(plant.minimum_contact_load)
                and plant.minimum_contact_load >= 0.
                and plant.out_of_range_evaluations == 0
                and travel_valid)
        # Interval-wide evidence that is narrower than between-step validation:
        # it bounds the BDF dense interpolant (not the true trajectory) for the
        # explicit state domain, and checks parameter-dependent contact
        # conditions. Load-closure and powertrain closure between nodes stay
        # uncertified, so these flags never set between_step_validation_complete.
        # Both checks are planar-7dof only: the vertical plant has no planar
        # dense-domain certificate, so its summary records them as None.
        if spec.planar_dense_certificate:
            from simplified_vehicle_models.solvers.planar_dense_certificate import (
                certify_planar_dense_domain, contact_nonnegativity_conditions)
            if result.dense_bounds is not None:
                dense = certify_planar_dense_domain(vehicle, plan, result.dense_bounds, True)
            else:
                dense = {"certified": False, "reason": "no verifiable dense interpolant"}
            contact = contact_nonnegativity_conditions(vehicle)
        else:
            dense = None
            contact = None
        # Save the exact interpolant so independent refinements can be compared
        # continuously (see tools/gates/check_iem24_forward_tube.py).
        if result.dense_polynomials is not None:
            poly = result.dense_polynomials
            np.savez_compressed(root / "dense_polynomials.npz", times=poly.times, t_end=poly.t_end,
                                step=poly.step, s_lower=poly.s_lower, s_upper=poly.s_upper,
                                order=poly.order, coefficients=poly.coefficients,
                                bounds=result.dense_bounds)
            summary["dense_polynomials_file"] = "dense_polynomials.npz"
        summary["dense_midpoint_samples"] = int(result.dense_samples)
        summary["dense_defect_scaled_max"] = float(result.dense_defect_scaled_max)
        summary["dense_polynomial_domain_certified"] = (
            None if dense is None else bool(dense["certified"]))
        summary["dense_polynomial_domain"] = dense
        summary["contact_nonnegativity_conditions_hold"] = (
            None if contact is None else bool(contact["holds"]))
        summary["contact_nonnegativity_conditions"] = contact
        summary["usable"] = acceptance_gate(summary)
    except Exception as error:
        import traceback
        (root / "error.txt").write_text(traceback.format_exc())
        summary.update(status="Timed_Out" if isinstance(error, TimeoutError) else "Rejected",
                       error=f"{type(error).__name__}: {error}")
    finally:
        if plant is not None:
            summary.update(last_rhs_time=plant.last_time, last_rhs_distance=plant.last_distance,
                           plant_evaluations=plant.evaluations,
                           maximum_load_residual=plant.maximum_load_residual,
                           maximum_power_residual=plant.maximum_power_residual,
                           maximum_load_iterations=plant.maximum_load_iterations,
                           load_method_counts=plant.load_method_counts,
                           minimum_load_jacobian_determinant=(
                               plant.minimum_load_jacobian_determinant
                               if np.isfinite(plant.minimum_load_jacobian_determinant) else None),
                           lifted_wheel_evaluations=plant.lifted_wheel_evaluations,
                           speed_regulated_evaluations=plant.speed_regulated_evaluations,
                           maximum_motor_speed_ratio=plant.maximum_motor_speed_ratio)
            if not spec.planar_dense_certificate:
                # Diagnostics over every plant evaluation, including rejected
                # trials. An attempt aborted before its first successful
                # evaluation still holds the ±inf sentinels, which
                # ``allow_nan=False`` rejects, so they map to None here.
                summary.update(
                    out_of_range_evaluations=plant.out_of_range_evaluations,
                    jounce_evaluation_minimum_m=finite_or_none(plant.minimum_jounce),
                    jounce_evaluation_maximum_m=finite_or_none(plant.maximum_jounce),
                    maximum_damper_velocity=finite_or_none(plant.maximum_damper_velocity))
        elapsed = time.perf_counter() - started
        peak = peak_memory_bytes()
        summary["elapsed_wall_seconds"] = elapsed
        summary["peak_memory_bytes"] = peak
        summary["peak_memory_source"] = (None if peak is None else
                                         "resource.getrusage(RUSAGE_SELF).ru_maxrss "
                                         "(bytes on macOS, kibibytes elsewhere, converted)")
        (root / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        print(json.dumps(summary, allow_nan=False), flush=True)
    return 0 if summary["status"] in ("Completed", "Duration_Limit") else 2


def main(argv=None):
    args = parse_args(argv)
    if args.child_attempt is not None:
        return child(args)
    attempt = args.output_root / f"attempt_{time.time_ns()}"
    attempt.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, "-u", str(Path(__file__).resolve()),
               *(sys.argv[1:] if argv is None else argv), "--child-attempt", str(attempt)]
    started = time.perf_counter()
    with (attempt / "run.log").open("w") as log:
        try:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=args.timeout_seconds)
            status = "Child_Completed" if result.returncode == 0 else "Child_Rejected"
            return_code = result.returncode
        except subprocess.TimeoutExpired:
            status, return_code = "Timed_Out", 124
    runtime = time.perf_counter() - started
    parent = {"status": status, "return_code": return_code,
              "total_wall_seconds": runtime, "within_wall_limit": runtime <= args.timeout_seconds,
              "timeout_seconds": args.timeout_seconds,
              "requested_wall_seconds": args.timeout_seconds,
              "elapsed_wall_seconds": runtime,
              "solver_wall_budget_seconds": solver_wall_budget(args.timeout_seconds, args.model),
              "model": args.model, "command": command,
              "attempt": str(attempt)}
    (attempt / "parent.json").write_text(json.dumps(parent, indent=2) + "\n")
    print(json.dumps(parent), flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
