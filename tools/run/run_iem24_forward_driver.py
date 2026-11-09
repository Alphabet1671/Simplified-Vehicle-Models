"""Run an isolated forward-driver attempt under a parent wall deadline."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, asdict
import hashlib
import importlib
import json
import math
from numbers import Real
from pathlib import Path
import re
import subprocess
import sys
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ElementTree

import numpy as np

from simplified_vehicle_models.solvers.hybrid_setup import (
    HYBRID_EXECUTION_ALLOCATOR_ID, HYBRID_HORIZON_S, HYBRID_MODE, HYBRID_SETUP_VERSION, HYBRID_SLABS,
    HYBRID_POLICY_MAX_EXTRA_FORECASTS, HYBRID_POLICY_BRACKET_RESOLUTION, HYBRID_POLICY_MAX_TOTAL_FORECASTS,
)
from simplified_vehicle_models.solvers.hybrid_driver import HYBRID_POLICY_VERSION
from simplified_vehicle_models.solvers.hybrid_speed_plan import HYBRID_PLAN_VERSION

#: Canonical ten-lap MIS endurance track: the active default for new runs.
DEFAULT_TRACK = Path("models/tracks/2024_michigan_endurance_multilap_closed.xml")
#: Fraction of the point-mass cornering grip the preview plan uses on an open
#: course when --lateral-margin is not given. At 1 the car is planned at the
#: steady-state grip limit and cannot change direction: the skidpad's
#: right-to-left change then leaves the path by about 2 m (7DoF). Measured on the
#: FSAE skidpad: both models track within 0.25 m at 0.92 and below, the 7DoF also
#: at 0.95, and neither at 0.97. 0.90 leaves headroom for the 14DoF. The value is
#: empirical, not derived.
OPEN_COURSE_LATERAL_MARGIN = 0.90

#: Closed-lap lateral margin of the 14DoF model when --lateral-margin is not
#: given. The planner's point-mass corner speed over-predicts the 14DoF plant's
#: steady lateral limit by up to about 8 % in tight corners (steady-corner solve
#: at the planned speed: 15.3 against 16.6 m/s^2 at kappa 0.12, 16.2 against
#: 16.9 at 0.08). At margin 1 the 14DoF car therefore runs wide at corner exit
#: (0.44 m MIS, 0.49 m FSC). Measured 2026-11-04: 0.92 tracks MIS in 0.202 m and
#: FSC in 0.251 m; 0.90 tracks FSC in 0.228 m. The value is empirical, not derived.
VERTICAL_CLOSED_LATERAL_MARGIN = 0.90

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
#: Established default wall budget for new planar-7DoF attempts. Explicit
#: user-selected timeout budgets are accepted for either model.
PLANAR_MAXIMUM_WALL_SECONDS = 300.
#: Maximum path spacing: a coarser plan is not a resolution the driver may run.
MAXIMUM_PATH_SPACING_METRES = .5
#: Track subtypes that carry a signed centerline. A closed one is a periodic lap;
#: an open one is a standing start with no periodicity.
SIGNED_TRACK_SUBTYPES = ("closed-xy-profile", "open-xy-profile")

#: Sources every model depends on. The powertrain closure moved to
#: ``forward_powertrain.py`` in Task 7 and both plants call it, so both hash it:
#: for the planar rung that changes recorded provenance, not any value.
_ENVELOPE_SOURCES = (
    Path("src/simplified_vehicle_models/solvers/braking_envelope.py"),
    Path("src/simplified_vehicle_models/solvers/envelope_driver.py"),
)
_SOLVER_SOURCES = (
    Path("src/simplified_vehicle_models/drivetrain/controller.py"),
    Path("src/simplified_vehicle_models/solvers/driver_maps.py"),
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
    Path("src/simplified_vehicle_models/solvers/vertical_dense_certificate.py"),
    Path("src/simplified_vehicle_models/solvers/dense_domain.py"),
    Path("src/simplified_vehicle_models/solvers/vertical_equilibrium.py"),
    Path("src/simplified_vehicle_models/formulations/vertical_14dof.py"),
    Path("src/simplified_vehicle_models/tires/unitire_transient.py"),
    Path("src/simplified_vehicle_models/tires/vertical.py"),
    Path("src/simplified_vehicle_models/tires/unitire_simple.py"),
    Path("src/simplified_vehicle_models/suspension/components.py"),
    Path("src/simplified_vehicle_models/suspension/corner.py"),
    Path("src/simplified_vehicle_models/suspension/damper_table.py"),
    Path("src/simplified_vehicle_models/suspension/elements.py"),
    Path("src/simplified_vehicle_models/suspension/kinematics.py"),
    Path("src/simplified_vehicle_models/suspension/layouts/independent_corner.py"),
    Path("src/simplified_vehicle_models/suspension/monotone_spline.py"),
    Path("src/simplified_vehicle_models/suspension/network.py"),
    Path("src/simplified_vehicle_models/suspension/physical.py"),
    Path("src/simplified_vehicle_models/suspension/physical_components.py"),
    Path("src/simplified_vehicle_models/suspension/layouts/parallel.py"),
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
    #: Default lateral margin on a closed lap when --lateral-margin is not given.
    closed_lateral_margin: float = 1.

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
        component_roles=("tire", "tire-transient", "suspension"),
        planar_dense_certificate=False,
        wall_margin_fraction=.02,
        closed_lateral_margin=VERTICAL_CLOSED_LATERAL_MARGIN,
    ),
}
#: Model names in CLI order.
MODEL_NAMES: tuple[str, ...] = tuple(MODELS)
#: Plant state-name patterns that expose the four wheel speeds, most specific
#: first: the planar plant names them ``omega_*``, the vertical plant
#: ``wheel_speed_*``.
_WHEEL_SPEED_PATTERNS = ("omega_{}", "wheel_speed_{}")
_WHEELS = ("fl", "fr", "rl", "rr")


def plant_class_for(spec: ModelSpec, vehicle):
    """Return the plant class for a compiled vehicle.

    A vertical vehicle whose suspension network carries its own states (the
    hydraulic layout) needs a plant class with those states appended; every
    other vehicle uses the spec's plant class unchanged.
    """
    module = importlib.import_module(spec.plant_module)
    factory = getattr(module, "vertical_plant_class", None)
    if factory is not None and hasattr(vehicle, "suspension"):
        network = getattr(vehicle.suspension, "network", None)
        if network is not None and int(getattr(network, "n_states", 0)) > 0:
            return factory(vehicle)
    return spec.plant


def network_state_ranges(vehicle, plant, accepted_state):
    """Return the summary entries for the suspension network's own states.

    The range covers every plant evaluation (rejected Newton trials included) and
    the accepted trajectory. A state outside the network's physical bounds, for
    example a gas volume beyond an accumulator stop, makes ``network_state_valid``
    false, which the ``table_domain_valid`` gate reads.
    """
    network = vehicle.suspension.network
    count = int(getattr(plant, "network_states", 0))
    if count == 0:
        return {}
    bounds = np.asarray(network.state_bounds, dtype=float)
    offset = plant.state_size - plant.network_states
    accepted = accepted_state[offset:]
    low = np.minimum(plant.network_state_minimum, accepted.min(axis=1))
    high = np.maximum(plant.network_state_maximum, accepted.max(axis=1))
    valid = bool(np.all(low >= bounds[:, 0]) and np.all(high <= bounds[:, 1]))
    return {"network_state_names": list(network.state_names),
            "network_state_minimum": low.tolist(), "network_state_maximum": high.tolist(),
            "network_state_bounds": bounds.tolist(), "network_state_valid": valid}


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
    """Resolve one component role of an assembly to its XML path.

    A role path such as ``suspension/damper-front`` descends into the children of
    a component that nests others.
    """
    vehicle_xml = Path(vehicle_xml)
    head, _, rest = role.partition("/")
    root = ElementTree.parse(vehicle_xml).getroot()
    for component in root.findall("./components/component"):
        if component.get("role") == head:
            path = canonical_path(vehicle_xml.parent / component.get("ref"))
            return component_xml(path, rest) if rest else path
    raise ValueError(f"{vehicle_xml}: no component with role {head!r}")


def component_data_paths(component_xml_path):
    """Return every external data file one component XML references."""
    component_xml_path = Path(component_xml_path)
    root = ElementTree.parse(component_xml_path).getroot()
    return tuple(canonical_path(component_xml_path.parent / data.get("path"))
                 for data in root.iter("data") if data.get("path"))


def model_inputs(model: str, track_xml):
    """Return named physical inputs, recursively including nested components."""
    spec = model_spec(model)
    track_xml, track_csv = track_files(track_xml)
    inputs = {"track_xml": track_xml, "track_csv": track_csv,
              "tire_xml": component_xml(spec.vehicle, "tire"),
              "formulation_xml": spec.formulation, "vehicle_xml": spec.vehicle}
    for index, data_path in enumerate(component_data_paths(spec.vehicle)):
        inputs["vehicle_data" if index == 0 else f"vehicle_data_{index}"] = data_path
    for role in spec.component_roles:
        if role == "tire":
            continue  # already recorded as ``tire_xml``
        _record_component(inputs, role.replace("-", "_"), component_xml(spec.vehicle, role))
    return inputs


def _record_component(inputs, key, xml):
    """Record one component XML, its data tables, and (recursively) its children."""
    inputs[f"{key}_xml"] = xml
    for index, data in enumerate(component_data_paths(xml)):
        inputs[f"{key}_data" if index == 0 else f"{key}_data_{index}"] = data
    root = ElementTree.parse(xml).getroot()
    for child in root.findall("./components/component"):
        _record_component(inputs, f"{key}_{child.get('role').replace('-', '_')}",
                          canonical_path(Path(xml).parent / child.get("ref")))


def source_files(model: str, track_xml, longitudinal_policy="braking-envelope", vertical_corner_samples=0, transient_corner_planning=False, physical_brake_planning=False, longitudinal_mode="legacy"):
    """Return every file hashed and snapshotted for one run, in order.

    The file names must stay unique because the attempt's ``source_snapshot``
    copies each source by base name.
    """
    spec = model_spec(model)
    sources = [Path(__file__), *spec.solver_sources,
               *((Path("src/simplified_vehicle_models/solvers/envelope_driver.py"),
                  Path("src/simplified_vehicle_models/solvers/brake_authority.py"),
                  Path("src/simplified_vehicle_models/controls/brake_inverse.py"),
                  Path("src/simplified_vehicle_models/solvers/forward_powertrain.py"))
                 if physical_brake_planning else ()),
               *(_ENVELOPE_SOURCES if longitudinal_policy == "braking-envelope" else ()),
               *((Path("src/simplified_vehicle_models/solvers/vertical_capability.py"),)
                 if vertical_corner_samples and model == "14dof-simple" else ()),
               *((Path("src/simplified_vehicle_models/solvers/yaw_transition.py"),)
                 if transient_corner_planning else ()),
               *model_inputs(model, track_xml).values()]
    if longitudinal_mode == "hybrid-drive-brake-v1":
        sources.extend(Path("src/simplified_vehicle_models/solvers") / name for name in
            ("hybrid_setup.py", "hybrid_driver.py", "hybrid_speed_plan.py", "longitudinal_capability.py"))
    unique = []
    for path in sources:
        if path not in unique:
            unique.append(path)
    names = [path.name for path in unique]
    if len(set(names)) != len(names):
        raise ValueError("source snapshot file names must be unique")
    return tuple(unique)


def apply_vertical_corner_restriction(vehicle, plan, envelope, *, max_queries, is_closed, artifact_path):
    """Apply opt-in tested-speed evidence to the envelope and preview plan."""
    from simplified_vehicle_models.solvers.vertical_capability import constrain_corner_speeds
    from simplified_vehicle_models.solvers.braking_envelope import restrict_braking_envelope
    evidence = constrain_corner_speeds(vehicle, plan.distance, plan.curvature, envelope.corner_speed,
                                       max_queries=max_queries, is_closed=is_closed)
    payload = asdict(evidence)
    payload["caps"] = evidence.caps.tolist()
    payload["samples"] = [asdict(sample) for sample in evidence.samples]
    payload["caveat"] = ("Sparse tested-speed restrictions are operational evidence, not a physical maximum "
                         "or feasibility guarantee; unsampled speeds and stations are not certified.")
    artifact_path = Path(artifact_path)
    artifact_path.write_text(json.dumps(payload, sort_keys=True, indent=2, allow_nan=False) + "\n")
    semantic_hash = hashlib.sha256(canonical_json(payload).encode()).hexdigest()
    artifact_hash = hashlib.sha256(artifact_path.read_bytes()).hexdigest()
    restricted = restrict_braking_envelope(envelope, evidence.caps,
        provenance={"query_budget": max_queries, "semantic_sha256": semantic_hash})
    speed = np.minimum(plan.speed, restricted.entry_speed)
    if not is_closed:
        speed[0] = 0.
    from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
    restricted_plan = SpeedPlan(plan.distance, speed, plan.curvature, is_closed=plan.is_closed)
    detail = {"file": artifact_path.name, "query_budget": max_queries,
              "query_count": evidence.query_count, "semantic_sha256": semantic_hash,
              "artifact_sha256": artifact_hash, "reasons": list(evidence.reasons),
              "caveat": payload["caveat"]}
    return restricted_plan, restricted, detail


def apply_transient_corner_restriction(control, plan, envelope, artifact_path):
    """Lower caps for sampled yaw-transition demand and bind their evidence."""
    from simplified_vehicle_models.solvers.yaw_transition import yaw_transition_speed_caps
    from simplified_vehicle_models.solvers.braking_envelope import restrict_braking_envelope
    from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
    evidence = yaw_transition_speed_caps(plan.distance, plan.curvature, control,
                                         envelope.corner_speed, is_closed=plan.is_closed)
    payload = {"caps": evidence.caps.tolist(),
               "curvature_gradient": evidence.curvature_gradient.tolist(),
               "provenance": dict(evidence.provenance),
               "steering_feedforward": "actuator-inverse"}
    semantic = hashlib.sha256(canonical_json(payload).encode()).hexdigest()
    path = Path(artifact_path)
    path.write_text(json.dumps(payload, sort_keys=True, indent=2, allow_nan=False) + "\n")
    restricted = restrict_braking_envelope(envelope, evidence.caps,
        provenance={"yaw_transition_semantic_sha256": semantic})
    speeds = np.minimum(plan.speed, restricted.entry_speed)
    if not plan.is_closed:
        speeds[0] = 0.
    updated = SpeedPlan(plan.distance, speeds, plan.curvature, is_closed=plan.is_closed)
    return updated, restricted, {"file": path.name, "semantic_sha256": semantic,
        "artifact_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "restricted_nodes": int(np.count_nonzero(evidence.caps < envelope.corner_speed)),
        "provenance": dict(evidence.provenance)}


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
    """Return finite jounce extrema over accepted states and dense bounds."""
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
    """Return legacy jounce design metadata, not an enforced travel bound."""
    limits = np.asarray(vehicle.suspension.kinematics.jounce_limits, dtype=float)
    return float(np.max(limits[:, 0])), float(np.min(limits[:, 1]))


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

    Only a signed ``closed-xy-profile`` or ``open-xy-profile`` track is
    accepted; a legacy distance-radius profile has no turn direction and cannot
    drive the 7DoF. An open track is run as a standing start.
    """
    track_xml = Path(track_xml)
    try:
        root = ElementTree.parse(track_xml).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read track XML {track_xml}: {error}") from error
    if root.get("type") != "track-model" or root.get("subtype") not in SIGNED_TRACK_SUBTYPES:
        raise ValueError(f"{track_xml}: track must be a signed closed-xy-profile "
                         "or open-xy-profile track-model")
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


def command_dict(command):
    if command is None:
        return None
    return {"throttle": float(command.throttle), "brake_force": float(command.brake_force),
            "steering_rate": float(command.steering_rate)}


def canonical_json(value):
    """Serialize JSON identity records without lossy coercion or NaN."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _hybrid_steering_plan(hybrid_plan):
    """Adapt the tuple-only steering preview to the canonical hybrid plan."""
    def sample(station):
        item=hybrid_plan.sample(station)
        return item.speed_mps,item.curvature_inv_m,item.acceleration_mps2
    return SimpleNamespace(distance=hybrid_plan.distance_m,speed=hybrid_plan.speed_mps,
        curvature=hybrid_plan.curvature_inv_m,is_closed=hybrid_plan.is_closed,sample=sample)


def policy_identity_settings(args):
    """Settings and deterministic envelope-builder inputs that define policy identity."""
    from dataclasses import asdict
    from simplified_vehicle_models.solvers.envelope_driver import EnvelopeDriverSettings
    mode=getattr(args,"longitudinal_mode", "physical-reachable-v1" if getattr(args,"physical_brake_planning",False) else "legacy")
    identity={"policy": args.longitudinal_policy,
            "longitudinal_mode": mode,
            "forecast_version": "operational-diagnostic-v1" if getattr(args,"physical_brake_planning",False) else None,
            "forecast_horizon_s": 3.0 if getattr(args,"physical_brake_planning",False) else None,
            "authority_provenance_version": "compiled-source-calibration-v1" if getattr(args,"physical_brake_planning",False) else None,
            "settings": asdict(EnvelopeDriverSettings(speed_gain=args.speed_gain,
                mode="physical-reachable-v1" if getattr(args,"physical_brake_planning",False) else "legacy",
                planning_transition_time_s=None, forecast_horizon_s=3.0)),
            "physical_actuator": {key: getattr(args, key) for key in ("pedal_travel_time", "brake_apply_rate", "brake_release_rate")} if getattr(args,"physical_brake_planning",False) else None,
            "source_closure_sha256": getattr(args, "physical_source_closure_sha256", None) if getattr(args,"physical_brake_planning",False) else None,
            "builder": {"deceleration_margin": args.envelope_deceleration_margin,
                        "speed_query_points": args.envelope_speed_points,
                        "maximum_speed": args.maximum_speed,
                        "lateral_margin": args.lateral_margin,
                        "storage_state_of_charge": getattr(args, "initial_storage_soc", None),
                        "is_closed": getattr(args, "track_is_closed", None),
                        "builder_parameters": getattr(args, "builder_parameters", None),
                        **({"vertical_corner_samples": args.vertical_corner_samples,
                            "vertical_capability_semantic_sha256": getattr(args, "vertical_capability_semantic_sha256", None)}
                           if getattr(args, "vertical_corner_samples", 0) else {}),
                        **({"transient_corner_planning": True,
                            "yaw_transition_semantic_sha256": getattr(args, "yaw_transition_semantic_sha256", None)}
                           if getattr(args, "transient_corner_planning", False) else {})}}

    if mode == "hybrid-drive-brake-v1":
        identity.update({"longitudinal_mode": mode,
            "policy": HYBRID_POLICY_VERSION,
            "hybrid_setup_version": HYBRID_SETUP_VERSION,
            "hybrid_plan_version": HYBRID_PLAN_VERSION,
            "hybrid_policy_version": HYBRID_POLICY_VERSION,
            "hybrid_horizon_s": HYBRID_HORIZON_S, "hybrid_slabs": HYBRID_SLABS,
            "hybrid_max_extra_forecasts": HYBRID_POLICY_MAX_EXTRA_FORECASTS,
            "hybrid_bracket_resolution": HYBRID_POLICY_BRACKET_RESOLUTION,
            "hybrid_max_total_forecasts": HYBRID_POLICY_MAX_TOTAL_FORECASTS,
            "hybrid_max_total_transitions": HYBRID_POLICY_MAX_TOTAL_FORECASTS*HYBRID_SLABS,
            "provider_identity": getattr(args, "hybrid_provider_identity", None),
            "allocator_identity": getattr(args, "hybrid_allocator_identity", None),
            "physical_actuator": {key: getattr(args, key) for key in ("pedal_travel_time", "throttle_apply_rate", "throttle_release_rate", "brake_apply_rate", "brake_release_rate")},
            "source_closure_sha256": getattr(args, "physical_source_closure_sha256", None)})
    return identity


def warm_start_matches(previous, source_hashes, args, pedal_map_sha256=None, policy_settings_identity=None):
    """True when a previous attempt may seed this run (same plan, track and sources).

    Source hashes include the track XML/CSV, tire XML and formulation, so a
    historical attempt without them cannot warm-start a track-aware run.
    """
    settings = previous.get("settings", {})
    defaults = {"preview_seconds": .15, "heading_gain": 1.5, "lateral_gain": 1.5, "speed_gain": 2.,
                "lateral_margin": None, "initial_speed": 0.}
    previous_policy = previous.get("longitudinal_policy", {})
    if not isinstance(previous_policy, dict):
        previous_policy = {}
    current_policy_settings = policy_settings_identity or policy_identity_settings(args)
    hybrid_mode = getattr(args, "longitudinal_mode", "legacy") == "hybrid-drive-brake-v1"
    expected_policy = "hybrid-drive-brake-v1" if hybrid_mode else "historical-preview-v1" if getattr(args, "longitudinal_mode", "legacy") == "historical-preview-v1" else "braking-envelope"
    policy_matches = (previous_policy.get("policy") == expected_policy
                      and settings.get("longitudinal_policy") == expected_policy
                      and (not hybrid_mode or previous_policy.get("settings_identity", {}).get("setup_identity") == current_policy_settings.get("setup_identity"))
                       and (getattr(args,"longitudinal_mode", "legacy") != "hybrid-drive-brake-v1" or settings.get("longitudinal_mode") == "hybrid-drive-brake-v1")
                       and settings.get("physical_brake_planning", False) == bool(getattr(args, "physical_brake_planning", False))
                       and settings.get("transient_corner_planning", False) == bool(getattr(args, "transient_corner_planning", False))
                      and settings.get("envelope_deceleration_margin", .6) == args.envelope_deceleration_margin
                      and settings.get("envelope_speed_points", 9) == args.envelope_speed_points
                      and (expected_policy == "historical-preview-v1" or previous_policy.get("deceleration_margin", .6) == args.envelope_deceleration_margin)
                      and (expected_policy == "historical-preview-v1" or previous_policy.get("speed_query_points", 9) == args.envelope_speed_points)
                      and previous_policy.get("settings_identity_sha256") ==
                           hashlib.sha256(canonical_json(current_policy_settings).encode()).hexdigest())
    return bool(policy_matches
                and previous.get("status") == "Completed" and previous.get("lap_completed")
                and previous.get("source_hashes") == source_hashes
                and (previous.get("pedal_map", {}).get("semantic_sha256", None) == pedal_map_sha256
                     or (pedal_map_sha256 == "no-pedal-map" and "pedal_map" not in previous))
                and all(settings.get(key, defaults.get(key)) == getattr(args, key)
                        for key in ("maximum_speed", "lateral_acceleration",
                                    "pedal_travel_time", "throttle_apply_rate",
                                    "throttle_release_rate", "brake_apply_rate",
                                    "brake_release_rate", "spacing", "preview_seconds",
                                    "heading_gain", "lateral_gain", "speed_gain", "lateral_margin", "initial_speed"))
                and canonical_path(settings.get("track") or LEGACY_TRACK)
                == canonical_path(args.track))


def _load_warm_start_arrays(warm_start, previous, source_hashes, args, pedal_map_sha256, policy_settings_identity, plant):
    """Validate a completed seed against the actual setup identity before loading arrays."""
    if not warm_start_matches(previous, source_hashes, args, pedal_map_sha256, policy_settings_identity):
        raise ValueError("warm start requires a completed lap with matching speed cap, track, source hashes and pedal-map identity")
    trace = np.loadtxt(Path(warm_start) / "trace.csv", delimiter=",", skiprows=1)
    pedals = np.loadtxt(Path(warm_start) / "pedal.csv", delimiter=",", skiprows=1)
    if (trace.ndim != 2 or trace.shape[1] != 1 + plant.state_size
            or pedals.shape != (len(trace), 2)
            or not np.array_equal(trace[:, 0], pedals[:, 0])):
        raise ValueError("warm-start trace and pedal samples must match")
    return trace, pedals


def _nonnegative_integer(value):
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("must be an integer >= 0") from None
    if str(parsed) != str(value).strip() or parsed < 0:
        raise argparse.ArgumentTypeError("must be an integer >= 0")
    return parsed


def _positive_integer(value):
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("must be an integer >= 2") from None
    if str(parsed) != str(value).strip() or parsed < 2:
        raise argparse.ArgumentTypeError("must be an integer >= 2")
    return parsed


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=MODEL_NAMES, default="planar-7dof",
                        help="Forward model: planar 7DoF or vertical 14DoF; --timeout-seconds "
                             "sets the user-selected wall budget for either model")
    parser.add_argument("--output-root", type=Path,
        default=Path("docs/validation/iem24-murphy/forward_driver_trials"))
    parser.add_argument("--timeout-seconds", type=float, default=300.)
    parser.add_argument("--duration", type=float, default=300.)
    parser.add_argument("--maximum-speed", type=float, default=8.)
    parser.add_argument("--initial-speed", type=float, default=0., help="actual initial speed for supported rolling starts; open-course legacy default remains standing start")
    parser.add_argument("--lateral-acceleration", type=float, default=None,
                        help="Optional cornering cap (m/s^2) on the point-mass preview plan")
    parser.add_argument("--pedal-travel-time", type=float, default=.175,
                        help="Seconds for a full pedal travel (0-100%% throttle or 0-max brake)")
    for name in ("throttle-apply-rate", "throttle-release-rate",
                 "brake-apply-rate", "brake-release-rate"):
        parser.add_argument(f"--{name}", dest=name.replace("-", "_"), type=float, default=None,
                            help="Directional normalized signed-pedal slew limit (1/s)")
    parser.add_argument("--preview-seconds", type=float, default=.15,
                         help="Preview horizon used by the shared steering controller")
    parser.add_argument("--lateral-margin", type=float, default=None,
                        help="Fraction (above 0, at most 1) of the point-mass cornering grip the live "
                             "preview plan uses. 1 plans at the grip limit; a lower value leaves grip for "
                             "steering changes. Default on a closed lap: 1 for planar-7dof, "
                             f"{VERTICAL_CLOSED_LATERAL_MARGIN} for 14dof-simple; "
                             f"{OPEN_COURSE_LATERAL_MARGIN} on an open course. Ignored by the "
                             "braking-envelope policy")
    parser.add_argument("--speed-gain", type=float, default=2.,
                        help="Proportional speed-error gain (1/s) of the shared speed controller")
    parser.add_argument("--heading-gain", type=float, default=1.5,
                         help="Heading-error gain used by the shared steering controller")
    parser.add_argument("--lateral-gain", type=float, default=1.5,
                         help="Lateral-error gain used by the shared steering controller")
    parser.add_argument("--maximum-step", type=float, default=.02)
    parser.add_argument("--rtol", type=float, default=1.e-6)
    parser.add_argument("--method", choices=("Radau", "BDF"), default="Radau")
    parser.add_argument("--spacing", type=float, default=.5)
    parser.add_argument("--envelope-deceleration-margin", type=float, default=.6)
    parser.add_argument("--envelope-speed-points", type=_positive_integer, default=9)
    parser.add_argument("--longitudinal-mode", choices=("historical-preview-v1", "legacy", "physical-reachable-v1", "hybrid-drive-brake-v1"), default=None)
    parser.add_argument("--physical-brake-planning", action="store_true",
                        help="Opt-in physical-reachable-v1 sampled operational brake forecast; not a feasibility certificate")
    parser.add_argument("--transient-corner-planning", action="store_true",
                        help="Opt-in sampled yaw-transition force screen and steering-actuator inverse; not a physical feasibility guarantee")
    parser.add_argument("--vertical-corner-samples", type=_nonnegative_integer, default=0,
                         help="Optional bounded vertical-model steady-corner query budget (14DoF only); "
                              "tested-speed restriction, not a physical maximum")
    parser.add_argument("--track", type=Path, default=DEFAULT_TRACK,
                        help="Signed closed-xy-profile or open-xy-profile track XML (default: the "
                             f"canonical ten-lap MIS track {DEFAULT_TRACK}). An open track is run "
                             "as a standing start from rest in legacy mode; hybrid mode requires --initial-speed > 0")
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
    parser.add_argument("--pedal-map-config", dest="pedal_map_config_path", type=Path, help="strict schema-versioned physical pedal calibration JSON")
    parser.add_argument("--expected-pedal-map-sha256", help=argparse.SUPPRESS)
    parser.add_argument("--expected-source-closure-sha256", help=argparse.SUPPRESS)
    parser.add_argument("--child-attempt", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    explicit_mode = args.longitudinal_mode
    if args.physical_brake_planning and explicit_mode not in (None, "physical-reachable-v1"):
        parser.error("--physical-brake-planning alias contradicts --longitudinal-mode")
    args.longitudinal_mode = ("physical-reachable-v1" if args.physical_brake_planning else "legacy" if args.transient_corner_planning or args.vertical_corner_samples else "historical-preview-v1") if explicit_mode is None else explicit_mode
    if args.longitudinal_mode == "historical-preview-v1" and (args.transient_corner_planning or args.vertical_corner_samples):
        parser.error("historical-preview-v1 cannot use envelope corner restrictions")
    args.longitudinal_policy = "historical-preview-v1" if args.longitudinal_mode == "historical-preview-v1" else "braking-envelope"
    args.physical_brake_planning = args.longitudinal_mode == "physical-reachable-v1"
    if args.longitudinal_mode == "hybrid-drive-brake-v1" and args.physical_brake_planning:
        parser.error("hybrid-drive-brake-v1 cannot be combined with --physical-brake-planning")
    if args.longitudinal_mode == "hybrid-drive-brake-v1" and (args.model != "14dof-simple" or args.transient_corner_planning or args.vertical_corner_samples):
        parser.error("hybrid-drive-brake-v1 requires --model 14dof-simple and forbids transient/yaw and separate vertical-corner planning")
    if args.vertical_corner_samples and args.model != "14dof-simple":
        parser.error("--vertical-corner-samples requires --model 14dof-simple")
    import math
    if not math.isfinite(args.initial_speed) or args.initial_speed < 0.:
        parser.error("initial-speed must be finite and nonnegative")
    if args.longitudinal_mode == "hybrid-drive-brake-v1" and args.initial_speed <= 0.:
        parser.error("hybrid-drive-brake-v1 supports rolling starts only; specify a positive --initial-speed (standing rest is not qualified)")
    for name in ("timeout_seconds", "duration", "maximum_speed", "lateral_acceleration",
                 "maximum_step", "rtol", "spacing", "pedal_travel_time",
                 "throttle_apply_rate", "throttle_release_rate", "brake_apply_rate",
                 "brake_release_rate", "preview_seconds",
                 "heading_gain", "lateral_gain", "speed_gain", "lateral_margin"):
        value = getattr(args, name)
        if value is None:
            continue
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")
    rates = (args.throttle_apply_rate, args.throttle_release_rate,
             args.brake_apply_rate, args.brake_release_rate)
    if any(rate is not None for rate in rates) and not all(rate is not None for rate in rates):
        parser.error("all four directional pedal rates must be supplied together")
    if args.lateral_margin is not None and args.lateral_margin > 1.:
        parser.error("lateral_margin must be at most 1")
    if not math.isfinite(args.envelope_deceleration_margin) or not 0. < args.envelope_deceleration_margin <= 1.:
        parser.error("envelope-deceleration-margin must be finite and in (0, 1]")
    if args.envelope_speed_points < 2:
        parser.error("envelope-speed-points must be an integer >= 2")
    if args.pedal_map_config_path is not None:
        try:
            _, pre_config, _ = load_runner_pedal_map(args.pedal_map_config_path)
        except (OSError, ValueError, TypeError) as error:
            parser.error(f"--pedal-map-config: {error}")
    try:
        track_files(args.track)
    except ValueError as error:
        parser.error(f"--track: {error}")
    args.track = canonical_path(args.track)
    args.pedal_map_config_path = (None if args.pedal_map_config_path is None else
                                  Path(args.pedal_map_config_path).expanduser().resolve())
    if args.warm_start is not None and args.pedal_travel_time is None and not all(
            rate is not None for rate in rates):
        parser.error("warm start requires pedal dynamics")
    if args.quasi_static_attitude and not args.record:
        parser.error("--quasi-static-attitude needs --record")
    if args.spacing > MAXIMUM_PATH_SPACING_METRES:
        parser.error(f"path spacing must be <= {MAXIMUM_PATH_SPACING_METRES} m")
    try:
        (args.pedal_map_config_payload, args.pedal_map_config,
         args.pedal_map_file_sha256) = load_runner_pedal_map(args.pedal_map_config_path)
    except (OSError, ValueError, TypeError) as error:
        parser.error(f"--pedal-map-config: {error}")
    if args.expected_pedal_map_sha256 is not None and args.child_attempt is None:
        parser.error("--expected-pedal-map-sha256 is reserved for runner children")
    if args.expected_pedal_map_sha256 is not None and args.expected_pedal_map_sha256 != (args.pedal_map_file_sha256 or ""):
        parser.error("--pedal-map-config changed after parent validation")
    if args.expected_source_closure_sha256 is not None and args.child_attempt is None:
        parser.error("--expected-source-closure-sha256 is reserved for runner children")
    return args


def load_runner_pedal_map(path):
    from simplified_vehicle_models.solvers.driver_maps import PedalMapConfiguration, parse_pedal_map_config
    if path is None:
        config = PedalMapConfiguration()
        return None, config, None
    raw = Path(path).read_bytes()

    def reject_constant(value):
        raise ValueError(f"nonfinite JSON constant {value} is not allowed")

    payload = json.loads(raw.decode("utf-8"), parse_constant=reject_constant)
    config = parse_pedal_map_config(payload)
    return payload, config, hashlib.sha256(raw).hexdigest()


def pedal_slew_provenance(args):
    """Return directional slew provenance only when explicitly configured."""
    if args.throttle_apply_rate is None:
        return None
    return {"schema_version": 1, "effective_normalized_rates": {
        "throttle_apply_rate": args.throttle_apply_rate,
        "throttle_release_rate": args.throttle_release_rate,
        "brake_apply_rate": args.brake_apply_rate,
        "brake_release_rate": args.brake_release_rate}}


def pedal_map_provenance(args, vehicle=None):
    config = args.pedal_map_config
    identity = config.sha256 if config is not None and config.physical_calibration else "no-pedal-map"
    bound = vehicle
    if vehicle is not None:
        from simplified_vehicle_models.solvers.driver_maps import bind_pedal_calibration
        bound = bind_pedal_calibration(vehicle, config.physical_calibration)
    return {"schema_version": 1, "source_path": None if args.pedal_map_config is None else str(args.pedal_map_config),
            "original_json": args.pedal_map_config_payload, "file_sha256": args.pedal_map_file_sha256,
            "normalized": config.to_dict() if config is not None else None,
            "semantic_sha256": identity,
            "physical_calibration": (config.to_dict()["physical_calibration"] if config else None),
            "has_physical_calibration": bool(config and config.physical_calibration is not None)}, bound


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
        "pedal_map": summary.get("pedal_map"),
        **({"longitudinal_policy": summary["longitudinal_policy"]} if "longitudinal_policy" in summary else {}),
        **({"pedal_slew": summary["pedal_slew"]} if "pedal_slew" in summary else {}),
    }, created_utc=datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"))
    summary["run_record_file"] = record_path.name
    summary["run_record_rows"] = int(sink.rows)


def event_times(track_xml, time, distance):
    """Return the elapsed time of every timing window the track XML declares.

    A window is a pair of parameters ``<name>_start_distance`` and
    ``<name>_end_distance``. Its time is the difference of the times at which the
    path coordinate ``distance`` reaches the two values. The path coordinate
    never decreases on an accepted run, so the time is read by interpolation.
    A window beyond the distance the run reached is reported as ``None``.
    """
    root = ElementTree.parse(track_xml).getroot()
    values = {}
    for parameter in root.iter("parameter"):
        try:
            values[parameter.get("name")] = float(parameter.get("value"))
        except (TypeError, ValueError):
            continue
    time = np.asarray(time, dtype=float)
    distance = np.maximum.accumulate(np.asarray(distance, dtype=float))
    result = {}
    for key, start in values.items():
        if not key.endswith("_start_distance"):
            continue
        name = key[:-len("_start_distance")]
        end = values.get(f"{name}_end_distance")
        if end is None:
            continue
        if end > distance[-1] + 1.e-9:
            result[name] = None
            continue
        at_start, at_end = np.interp([start, end], distance, time)
        result[name] = float(at_end - at_start)
    return result


def vertical_contact_valid(lap_completed, minimum_contact_load, *, network_state_valid=True):
    """Vertical contact gate independent of descriptive suspension-table extension counts."""
    return bool(lap_completed and math.isfinite(minimum_contact_load)
                and minimum_contact_load >= 0. and network_state_valid)


def acceptance_gate(summary):
    """Return true only when every required lap-validity gate is established.

    A closed lap must also close its periodic state. An open course
    (``track_is_closed`` false) must reach the end of the course instead.
    """
    delta = summary.get("periodic_state_delta")
    tolerances = summary.get("periodic_state_tolerances")
    def finite_real_vector(values):
        return (isinstance(values, (list, tuple)) and bool(values)
                and all(isinstance(value, Real) and math.isfinite(value)
                        for value in values))

    if summary.get("track_is_closed", True) is False:
        periodic = summary.get("lap_completed") is True
        periodicity_ok = True
    else:
        periodic = (summary.get("lap_completed") is True
                    and finite_real_vector(delta)
                    and finite_real_vector(tolerances)
                    and len(delta) == len(tolerances)
                    and all(tolerance >= 0 and abs(value) <= tolerance
                            for value, tolerance in zip(delta, tolerances)))
        periodicity_ok = summary.get("periodicity_valid") is True
    return bool(periodic
                and periodicity_ok
                and summary.get("integration_completed") is True
                and summary.get("path_tracking_valid") is True
                and summary.get("resolution_valid") is True
                and summary.get("checked_rhs_physics_valid") is True
                and summary.get("contact_valid") is True)


def child(args):
    started = time.perf_counter()
    root = args.child_attempt
    spec = model_spec(args.model)
    summary = {"status": "Starting", "lap_completed": False, "usable": False,
               "method": "transient_forward_driver", "minimum_time_optimum": False,
               "model": args.model, "requested_wall_seconds": args.timeout_seconds,
                              "settings": {k: str(v) if isinstance(v, Path) else v
                            for k, v in vars(args).items() if k not in {"pedal_map_config_payload", "pedal_map_config", "pedal_map_file_sha256"}},
                **({"longitudinal_policy": {"policy": args.longitudinal_policy,
                    "deceleration_margin": args.envelope_deceleration_margin,
                    "speed_query_points": args.envelope_speed_points,
                    "settings_identity_sha256": None}}
                   )}
    summary["settings"]["longitudinal_policy"] = ("hybrid-drive-brake-v1"
        if args.longitudinal_mode == "hybrid-drive-brake-v1" else args.longitudinal_policy)
    summary["settings"]["longitudinal_mode"] = args.longitudinal_mode
    summary["settings"]["initial_speed_mps"] = float(args.initial_speed)
    plant = None
    policy_decisions = []
    policy_solve_completed = False
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
        sources = source_files(args.model, args.track, args.longitudinal_policy,
                               args.vertical_corner_samples, args.transient_corner_planning,
                               args.physical_brake_planning, args.longitudinal_mode)
        physical_inputs = model_inputs(args.model, args.track)
        summary["source_hashes"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in sources}
        warm_start_previous = None
        if args.warm_start is not None and args.longitudinal_mode == "hybrid-drive-brake-v1":
            warm_start_previous = json.loads((args.warm_start / "summary.json").read_text())
            if warm_start_previous.get("source_hashes") != summary["source_hashes"]:
                raise ValueError("warm-start source/calibration identity changed before compile or prior arrays")
        if args.physical_brake_planning or args.longitudinal_mode == "hybrid-drive-brake-v1":
            args.physical_source_closure_sha256 = hashlib.sha256(
                canonical_json(summary["source_hashes"]).encode()).hexdigest()
            expected_closure = args.expected_source_closure_sha256
            if expected_closure is not None and expected_closure != args.physical_source_closure_sha256:
                raise ValueError("source/calibration closure changed between parent validation and child compile")
        summary["model_inputs"] = {key: str(path) for key, path in physical_inputs.items()}
        snapshot = root / "source_snapshot"
        snapshot.mkdir()
        source_names = {}
        for path in sources:
            name = path.name
            if name in source_names:
                name = str(path).replace("/", "__")
            source_names[name] = path
            (snapshot / name).write_bytes(path.read_bytes())
        summary["environment"] = {"python": sys.version, "numpy": np.__version__,
                                  "scipy": scipy.__version__, "platform": platform.platform()}
        vehicle = compile_formulation(str(spec.formulation)).model
        summary["pedal_map"], vehicle = pedal_map_provenance(args, vehicle)
        slew = pedal_slew_provenance(args)
        if slew is not None:
            summary["pedal_slew"] = slew
        summary["runtime_model_identity"] = {"formulation": str(spec.formulation),
                                             "model": args.model,
                                             "semantic_pedal_map_sha256": summary["pedal_map"]["semantic_sha256"]}
        # The vertical 14DoF model composes the planar parameters; the driver and
        # the preview plan reads the shared planar layer.
        control = getattr(vehicle, "planar", vehicle)
        if not spec.planar_dense_certificate:
            summary["model_provenance"] = vertical_model_provenance(
                vehicle, physical_inputs, summary["source_hashes"])
        track = load_track(args.track)
        count = int(np.ceil(track.lap_length / args.spacing))
        distance = np.linspace(0., track.lap_length, count + 1)
        _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
        live_limit = True
        lateral_cap = args.lateral_acceleration
        # An open course is a standing start: the plan begins at rest and does
        # not wrap, and the run has no periodicity to check.
        open_course = not track.is_closed
        lateral_margin = args.lateral_margin
        if lateral_margin is None:
            lateral_margin = (OPEN_COURSE_LATERAL_MARGIN if open_course
                              else spec.closed_lateral_margin)
        summary["lateral_margin"] = float(lateral_margin)
        summary["track_is_closed"] = bool(track.is_closed)
        from simplified_vehicle_models.powertrain.energy_storage import StorageState
        storage = StorageState(control.drivetrain.powertrain.energy_storage.initial_state_of_charge)
        hybrid_mode = args.longitudinal_mode == "hybrid-drive-brake-v1"
        historical_mode = args.longitudinal_mode == "historical-preview-v1"
        envelope = None
        if hybrid_mode:
            # Shared hybrid setup owns the only longitudinal reachability plan.
            # This pointwise geometry plan supplies initial speed only; it is
            # not an envelope and is never passed into the hybrid builder.
            plan = build_speed_plan(distance, curvature, args.maximum_speed,
                                    lateral_cap, 1., 2., vehicle=control,
                                    live_limit=live_limit, open_course=open_course,
                                    lateral_margin=lateral_margin)
        else:
            plan = build_speed_plan(distance, curvature, args.maximum_speed,
                                    lateral_cap, 1., 2., vehicle=control if live_limit else None,
                                    live_limit=live_limit, open_course=open_course,
                                    lateral_margin=lateral_margin)
            if not historical_mode:
                from simplified_vehicle_models.solvers.braking_envelope import build_braking_envelope
                envelope = build_braking_envelope(distance, curvature, control,
                    maximum_speed=args.maximum_speed, lateral_margin=lateral_margin,
                    deceleration_margin=args.envelope_deceleration_margin,
                    speed_query_points=args.envelope_speed_points, storage_state=storage,
                    is_closed=bool(track.is_closed))
        from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy, EnvelopeDriverSettings
        hybrid_setup = None
        if args.longitudinal_mode == "hybrid-drive-brake-v1":
            if args.model != "14dof-simple" or args.transient_corner_planning:
                raise ValueError("hybrid mode requires full 14DoF and forbids transient yaw/TV planning")
            from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext
            from simplified_vehicle_models.solvers.hybrid_setup import build_hybrid_setup
            from simplified_vehicle_models.tires.unitire_simple import loaded_radius
            from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar, normal_loads
            rated_loads=normal_loads(point_mass_view_of_planar(control),max(args.maximum_speed,1.),0.,0.)
            radii=loaded_radius(control.tire,rated_loads)
            pc=PedalPolicyContext(args.throttle_release_rate or 1./args.pedal_travel_time,
                args.brake_apply_rate or 1./args.pedal_travel_time,.02,control.maximum_pedal_force,.85,
                args.throttle_apply_rate or 1./args.pedal_travel_time,args.brake_release_rate or 1./args.pedal_travel_time)
            root_path=Path(__file__).resolve().parents[2]
            model_hashes={}
            for source in sources:
                path=Path(source).resolve()
                try: rel=path.relative_to(root_path).as_posix()
                except ValueError: continue
                model_hashes[rel]=hashlib.sha256(path.read_bytes()).hexdigest()
            args.hybrid_allocator_identity=HYBRID_EXECUTION_ALLOCATOR_ID
            hybrid_setup=build_hybrid_setup(control,distance_m=distance,curvature_inv_m=curvature,
                maximum_speed_mps=args.maximum_speed,initial_speed_mps=float(args.initial_speed),
                storage_soc=float(storage.state_of_charge),wheel_radii_m=tuple(float(v) for v in radii),
                model_source_hashes=model_hashes,maximum_pedal_force_n=control.maximum_pedal_force,
                pedal_context=pc,lateral_margin=lateral_margin,is_closed=bool(track.is_closed),
                execution_allocator_identity=HYBRID_EXECUTION_ALLOCATOR_ID)
            args.hybrid_provider_identity=hybrid_setup.provider.identity
            hybrid_plan=hybrid_setup.plan
            plan = _hybrid_steering_plan(hybrid_plan)
            plant=plant_class_for(spec,vehicle)(vehicle,hybrid_plan)
        vertical_evidence_hash = None
        if args.vertical_corner_samples:
            plan, envelope, detail = apply_vertical_corner_restriction(
                vehicle, plan, envelope, max_queries=args.vertical_corner_samples,
                is_closed=bool(track.is_closed), artifact_path=root / "vertical_corner_samples.json")
            vertical_evidence_hash = detail["semantic_sha256"]
            args.vertical_capability_semantic_sha256 = vertical_evidence_hash
            summary["vertical_corner_samples"] = detail
        if args.transient_corner_planning:
            plan, envelope, detail = apply_transient_corner_restriction(
                control, plan, envelope, root / "yaw_transition.json")
            args.yaw_transition_semantic_sha256 = detail["semantic_sha256"]
            summary["yaw_transition"] = detail
        if hybrid_setup is not None:
            hybrid_plan = hybrid_setup.plan
            def sample_for_driver(station):
                item = hybrid_plan.sample(station)
                return item.speed_mps, item.curvature_inv_m, item.acceleration_mps2
            plan = _hybrid_steering_plan(hybrid_plan)
            hybrid_steering_plan = plan
            summary["hybrid_setup"]={"identity":hybrid_setup.setup_identity,"plan_version":hybrid_setup.plan.algorithm_version,
                "policy_version":HYBRID_POLICY_VERSION,"setup_version":HYBRID_SETUP_VERSION,
                "horizon_s":hybrid_setup.policy.config.horizon_s,"slabs":hybrid_setup.policy.config.slabs,
                "provider_identity":hybrid_setup.provider.identity,"allocator_identity":HYBRID_EXECUTION_ALLOCATOR_ID,
                "plan_speed_mps":hybrid_setup.plan.speed_mps.tolist(),
                "corner_speed_cap_mps":hybrid_setup.plan.corner_speed_cap_mps.tolist(),
                "braking_speed_bound_mps":hybrid_setup.plan.braking_speed_bound_mps.tolist()}
        array_hashes = None
        if envelope is not None:
            array_names = ("distance", "curvature", "corner_speed", "entry_speed", "deceleration", "binding_index")
            arrays = {name: getattr(envelope, name) for name in array_names}
            arrays["segment_speed_domain_max"] = np.asarray(envelope.provenance["segment_speed_domain_max"], dtype=float)
            array_hashes = {name: hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()
                            for name, value in arrays.items()}
            np.savez_compressed(root / "envelope_arrays.npz", **arrays)
        envelope_settings = None if hybrid_setup is not None or historical_mode else EnvelopeDriverSettings(speed_gain=args.speed_gain,
            mode=("physical-reachable-v1" if args.physical_brake_planning else "legacy"),
            planning_transition_time_s=None, forecast_horizon_s=3.0)
        if args.physical_brake_planning:
            args.longitudinal_forecast_horizon_s = 3.0
        settings = asdict(envelope_settings) if envelope_settings is not None else {}
        if hybrid_setup is not None:
            settings = {"policy": HYBRID_POLICY_VERSION,
                "setup_version": HYBRID_SETUP_VERSION, "longitudinal_mode": HYBRID_MODE,
                "horizon_s": HYBRID_HORIZON_S, "slabs": HYBRID_SLABS,
                "max_extra_forecasts": hybrid_setup.policy.config.max_extra_forecasts,
                "bracket_resolution": hybrid_setup.policy.config.bracket_resolution,
                "max_total_forecasts": 3 + hybrid_setup.policy.config.max_extra_forecasts,
                "max_total_transitions": (3 + hybrid_setup.policy.config.max_extra_forecasts) * hybrid_setup.policy.config.slabs,
                "policy_version": HYBRID_POLICY_VERSION}
        summary["longitudinal_policy"] = {
            "policy": "hybrid-drive-brake-v1" if hybrid_setup is not None else args.longitudinal_policy, "settings": settings,
            "longitudinal_mode": args.longitudinal_mode,
            "forecast_version": "operational-diagnostic-v1" if args.physical_brake_planning else None,
            "forecast_horizon_s": 3.0 if args.physical_brake_planning else None,
            "transient_corner_planning": bool(args.transient_corner_planning),
            "maximum_speed": float(args.maximum_speed), "lateral_margin": float(lateral_margin),
            "deceleration_margin": float(args.envelope_deceleration_margin) if envelope is not None else None,
            "speed_query_points": int(args.envelope_speed_points) if envelope is not None else None,
            "storage_state": {"state_of_charge": float(storage.state_of_charge)},
            "is_closed": bool(track.is_closed),
            **({"builder": dict(envelope.provenance), "array_sha256": array_hashes,
                "arrays_file": "envelope_arrays.npz",
                "builder_source_sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in _ENVELOPE_SOURCES}}
                if envelope is not None else {"builder": "build_speed_plan", "live_limit": True}
                if historical_mode else {"builder": "build_hybrid_setup",
                    "setup_identity": hybrid_setup.setup_identity, "arrays_file": "hybrid_plan_arrays.npz"}),
            "caveat": "Sampled operational model-conditioned plan; not a feasibility certificate or tire-authority guarantee."}
        identity_settings = {"policy": "hybrid-drive-brake-v1" if hybrid_setup is not None else args.longitudinal_policy, "longitudinal_mode": args.longitudinal_mode,
            "forecast_version": "operational-diagnostic-v1" if args.physical_brake_planning else None,
            "forecast_horizon_s": 3.0 if args.physical_brake_planning else None,
            "authority_provenance_version": "compiled-source-calibration-v1" if args.physical_brake_planning else None,
            "physical_actuator": {key: getattr(args, key) for key in ("pedal_travel_time", "brake_apply_rate", "brake_release_rate")} if args.physical_brake_planning else None,
            "source_closure_sha256": getattr(args, "physical_source_closure_sha256", None) if args.physical_brake_planning else None,
            "transient_corner_planning": bool(args.transient_corner_planning), "settings": settings,
            "builder": {"deceleration_margin": float(args.envelope_deceleration_margin),
                "speed_query_points": int(args.envelope_speed_points),
                "maximum_speed": float(args.maximum_speed), "lateral_margin": float(lateral_margin),
                "storage_state_of_charge": float(storage.state_of_charge),
                "is_closed": bool(track.is_closed),
                **({"vertical_corner_samples": int(args.vertical_corner_samples),
                    "vertical_capability_semantic_sha256": vertical_evidence_hash}
                   if args.vertical_corner_samples else {}),
                **({"transient_corner_planning": True,
                    "yaw_transition_semantic_sha256": args.yaw_transition_semantic_sha256}
                   if args.transient_corner_planning else {})}}
        if hybrid_setup is not None:
            identity_settings = {"policy": "hybrid-drive-brake-v1", "longitudinal_mode": args.longitudinal_mode,
                "setup_identity": hybrid_setup.setup_identity,
                "setup_identity_payload": json.loads(hybrid_setup.setup_identity_payload_json),
                "setup_version": HYBRID_SETUP_VERSION,
                "policy_version": HYBRID_POLICY_VERSION,
                "horizon_s": hybrid_setup.policy.config.horizon_s,
                "slabs": hybrid_setup.policy.config.slabs,
                "max_extra_forecasts": hybrid_setup.policy.config.max_extra_forecasts,
                "bracket_resolution": hybrid_setup.policy.config.bracket_resolution,
                "max_total_forecasts": 3 + hybrid_setup.policy.config.max_extra_forecasts,
                "max_total_transitions": (3 + hybrid_setup.policy.config.max_extra_forecasts) * hybrid_setup.policy.config.slabs,
                "plan_version": hybrid_setup.plan.algorithm_version,
                "provider_identity": hybrid_setup.provider.identity,
                "allocator_identity": HYBRID_EXECUTION_ALLOCATOR_ID,
                "physical_actuator_rates": {key: getattr(args, key) for key in
                    ("throttle_apply_rate", "throttle_release_rate", "brake_apply_rate", "brake_release_rate")},
                "lateral_margin": float(lateral_margin), "brake_margin": float(args.envelope_deceleration_margin),
                "source_closure_sha256": getattr(args, "physical_source_closure_sha256", None)}
        args.builder_parameters = {"method": "sampled-quasi-static-v1",
            "deceleration_margin": float(args.envelope_deceleration_margin),
            "speed_query_points": int(args.envelope_speed_points),
            "maximum_speed": float(args.maximum_speed),
            "lateral_margin": float(lateral_margin),
            "storage_state_of_charge": float(storage.state_of_charge),
            "is_closed": bool(track.is_closed)}
        if hybrid_setup is None:
            identity_settings["builder"]["builder_parameters"] = args.builder_parameters
        summary["longitudinal_policy"]["settings_identity"] = identity_settings
        summary["longitudinal_policy"]["settings_identity_sha256"] = hashlib.sha256(
            canonical_json(identity_settings).encode()).hexdigest()
        summary["settings"].update(longitudinal_policy_settings_identity_sha256=
            summary["longitudinal_policy"]["settings_identity_sha256"],
            longitudinal_policy_settings_identity=identity_settings)
        artifact_payload = ({"builder": dict(envelope.provenance), "array_sha256": array_hashes}
                            if envelope is not None else {"builder": "build_speed_plan", "distance": plan.distance.tolist(), "speed": plan.speed.tolist(), "curvature": plan.curvature.tolist()}
                            if historical_mode else {"setup_identity": hybrid_setup.setup_identity,
                                "setup_identity_payload": json.loads(hybrid_setup.setup_identity_payload_json),
                                "plan_speed_mps": hybrid_setup.plan.speed_mps.tolist(),
                                "corner_speed_cap_mps": hybrid_setup.plan.corner_speed_cap_mps.tolist(),
                                "braking_speed_bound_mps": hybrid_setup.plan.braking_speed_bound_mps.tolist()})
        summary["longitudinal_policy"]["artifact_sha256"] = hashlib.sha256(
            canonical_json(artifact_payload).encode()).hexdigest()
        plant_class = plant_class_for(spec, vehicle)
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
        # The steering preview consumes the tuple sample contract, but hybrid
        # mode must sample the already-built canonical plan, not build a second.
        driver_plan = hybrid_steering_plan if hybrid_setup is not None else plan
        physical_predictor = None
        planning_factory = None
        current_factory = None
        if args.physical_brake_planning:
            from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
            physical_predictor = BrakeAuthorityPredictor(control,
                maximum_abs_curvature_inv_m=float(np.max(np.abs(curvature))),
                minimum_speed_mps=min(2.0, max(0.1, float(np.min(hybrid_setup.plan.speed_mps if hybrid_setup is not None else plan.speed)))),
                maximum_speed_mps=max(40.0, args.maximum_speed))
            planning_factory = lambda p, observation, pedal, state: p.brake_planning_context(
                state, pedal, predictor=physical_predictor)
            current_factory = lambda p, observation, pedal, state: p.brake_authority_context(
                state, predictor=physical_predictor)
        driver = PreviewDriver(driver_plan, control.wheelbase, control.steering_maximum_angle,
                               control.steering_maximum_rate, control.maximum_pedal_force,
                                preview_seconds=args.preview_seconds,
                                speed_gain=args.speed_gain,
                                heading_gain=args.heading_gain,
                                lateral_gain=args.lateral_gain,
                               pedal_travel_time=args.pedal_travel_time,
                                throttle_apply_rate=args.throttle_apply_rate,
                                throttle_release_rate=args.throttle_release_rate,
                                brake_apply_rate=args.brake_apply_rate,
                                brake_release_rate=args.brake_release_rate,
                                envelope_policy=None if hybrid_setup is not None or historical_mode else EnvelopePolicy(envelope, envelope_settings),
                                 hybrid_policy=hybrid_setup.policy if hybrid_setup is not None else None,
                                brake_planning_context_factory=planning_factory,
                                brake_current_context_factory=(lambda p, observation, pedal, state: p.brake_authority_context(state, predictor=hybrid_setup.brake_predictor)) if hybrid_setup is not None else current_factory,
                                brake_current_predictor=hybrid_setup.brake_predictor if hybrid_setup is not None else physical_predictor,
                                steering_feedforward=("actuator-inverse" if args.transient_corner_planning else "preview"))
        initial = plant.initial_state(float(hybrid_setup.plan.speed_mps[0] if hybrid_setup is not None else plan.speed[0]))
        summary["initial_condition"] = getattr(plant, "initial_condition", None)
        # A flying lap starts with the foot where the driver wants it. A standing
        # start begins with the foot off the pedal: the driver then presses the
        # throttle through the pedal lag instead of dropping full torque on a
        # wheel at rest.
        initial_pedal = 0. if open_course else None
        if args.warm_start is not None:
            previous = json.loads((args.warm_start / "summary.json").read_text())
            previous_settings = previous.get("settings", {})
            current_policy_identity = summary.get("longitudinal_policy", {}).get("settings_identity")
            if current_policy_identity is not None and "builder" in current_policy_identity:
                current_policy_identity["builder"]["builder_parameters"] = args.builder_parameters
            trace, pedals = _load_warm_start_arrays(args.warm_start, previous, summary["source_hashes"], args,
                summary["pedal_map"]["semantic_sha256"], current_policy_identity, plant)
            initial = np.array(trace[-1, 1:], copy=True)
            initial[state_index(plant, "s")] = 0.
            initial_pedal = float(pedals[-1, 1])
            if not np.all(np.isfinite(initial)):
                raise ValueError("warm-start final state must be finite")
            summary["warm_start_source"] = str(args.warm_start)
            summary["initial_condition"] = {"method": "warm-start",
                                            "source": str(args.warm_start)}
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
        policy_sidecar = root / "accepted_policy.json"
        decision_handle = None
        def persist_policy_prefix(status):
            payload = {"schema_version": 1, "policy": "hybrid-drive-brake-v1" if hybrid_setup is not None else args.longitudinal_policy, "status": status,
                       "accepted_decision_count": len(policy_decisions),
                       "decisions_file": "accepted_policy.jsonl"}
            temporary_path = policy_sidecar.with_suffix(".json.tmp")
            with temporary_path.open("w") as handle:
                json.dump(payload, handle, indent=2, allow_nan=False)
                handle.write("\n")
                handle.flush()
            temporary_path.replace(policy_sidecar)

        decision_path = root / "accepted_policy.jsonl"
        decision_handle = decision_path.open("w")
        persist_policy_prefix("in_progress")
        def accepted_policy_sink(sample):
            if sink is not None:
                sink(sample)
            if sample.policy_diagnostics is None:
                raise RuntimeError("accepted longitudinal-policy sample lacks cached decision")
            raw_decision = sample.policy_diagnostics.to_dict()
            decision = {"time": float(sample.time), "step_index": int(sample.index),
                "longitudinal_mode": args.longitudinal_mode,
                "decision_schema": "hybrid-telemetry-v1" if hybrid_setup is not None else "historical-preview-v1" if historical_mode else "task5-telemetry-v1",
                "decision": ({"schema": "hybrid-telemetry-v1", **raw_decision} if hybrid_setup is not None else raw_decision),
                "policy_kind": "hybrid" if hybrid_setup is not None else "historical-preview" if historical_mode else "envelope",
                "requested_command": command_dict(sample.requested_command),
                "mapped_command": command_dict(sample.mapped_command),
                "applied_command": command_dict(sample.command),
                "pedal": None if sample.pedal is None else float(sample.pedal)}
            decision_handle.write(json.dumps(decision, separators=(",", ":"), allow_nan=False) + "\n")
            decision_handle.flush()
            policy_decisions.append(decision)
        # Policy acceptance evidence is required for forward-policy validation;
        # the optional run-record step_sink remains opt-in and separate.
        if sink is not None:
            # One accepted-step observer owns both recording and policy evidence,
            # avoiding duplicate RunRecord writes when recording is enabled.
            recording["step_sink"] = accepted_policy_sink
            policy_recording = {}
        else:
            policy_recording = {"policy_sink": accepted_policy_sink}
        try:
            result = solve_forward(plant, driver, initial, duration=args.duration,
                                   wall_seconds=remaining,
                                   maximum_wall_seconds=args.timeout_seconds,
                                   maximum_step=args.maximum_step,
                                   rtol=args.rtol, atol=args.rtol * .01, method=args.method,
                                   initial_pedal=initial_pedal, **recording, **policy_recording)
            policy_solve_completed = True
        finally:
            if decision_handle is not None:
                decision_handle.close()
            persist_policy_prefix("complete" if policy_solve_completed else "accepted_prefix_incomplete")
        summary["solver_elapsed_seconds"] = float(result.wall_seconds)
        summary["accepted_policy_file"] = "accepted_policy.json"
        summary["accepted_policy_decisions"] = len(policy_decisions)
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
        if open_course:
            # An open course ends away from where it started, so closure is not
            # a validity test. The delta is still recorded, with no periodicity claim.
            periodic = None
            summary["event_times"] = event_times(
                args.track, result.time, result.state[state_index(plant, "s")])
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
                       steering_peak=float(np.max(np.abs(
                           result.state[state_index(plant, "steering")]))),
                       minimum_wheel_speed=float(np.min(
                           result.state[list(wheel_speed_indices(plant))])),
                       final_soc=float(result.state[state_index(plant, "soc"), -1]),
                       planned_lap_time=float(np.sum(np.diff(plan.distance)
                                                     / (.5 * (plan.speed[1:] + plan.speed[:-1])))),
                       planned_peak_lateral_acceleration=float(np.max(
                           plan.speed**2 * np.abs(plan.curvature))))
        summary["suspension_travel_policy"] = "unlimited"
        summary["suspension_travel_enforced"] = False
        # A completed transient is not usable unless every acceptance gate in
        # ``acceptance_gate`` passes (completion, periodicity, tracking, spacing,
        # checked physics and contact). Between-step and refinement checks were
        # removed by user decision on 2026-11-04.
        # Every accepted state has been checked by solve_forward. Zero contact
        # load is permitted (wheel lift); negative loads are rejected by plant.
        summary["minimum_contact_load_n"] = plant.minimum_contact_load
        if spec.planar_dense_certificate:
            summary["contact_valid"] = bool(result.lap_completed
                and math.isfinite(plant.minimum_contact_load)
                and plant.minimum_contact_load >= 0.)
        else:
            # Contact/usability is independent of unlimited travel. The range
            # remains descriptive design metadata; the compatibility flag means
            # accepted jounces are finite, not within that range.
            accepted_low, accepted_high, scope = accepted_jounce_extremes(result, plant)
            travel_valid = bool(math.isfinite(accepted_low) and math.isfinite(accepted_high))
            summary["suspension_travel_policy"] = "unlimited"
            summary["suspension_travel_enforced"] = False
            summary["jounce_travel_limits_m"] = None
            summary["jounce_design_range_m"] = vehicle.suspension.kinematics.jounce_limits.tolist()
            summary["jounce_gate_scope"] = scope
            summary["jounce_accepted_minimum_m"] = accepted_low
            summary["jounce_accepted_maximum_m"] = accepted_high
            summary["jounce_evaluation_minimum_m"] = finite_or_none(plant.minimum_jounce)
            summary["jounce_evaluation_maximum_m"] = finite_or_none(plant.maximum_jounce)
            summary["out_of_range_evaluations"] = plant.out_of_range_evaluations
            summary["jounce_travel_valid"] = travel_valid
            if getattr(plant, "network_states", 0):
                summary.update(network_state_ranges(vehicle, plant, result.state))
            summary["contact_valid"] = vertical_contact_valid(
                result.lap_completed, plant.minimum_contact_load,
                network_state_valid=summary.get("network_state_valid", True))
        # Diagnostic interval-wide evidence: it bounds the BDF dense interpolant
        # (not the true trajectory) for the explicit state domain, and checks
        # parameter-dependent contact conditions. It is recorded only; it is not
        # an acceptance gate.
        if spec.planar_dense_certificate:
            from simplified_vehicle_models.solvers.planar_dense_certificate import (
                certify_planar_dense_domain, contact_nonnegativity_conditions)
            if result.dense_bounds is not None:
                dense = certify_planar_dense_domain(vehicle, plan, result.dense_bounds, True)
            else:
                dense = {"certified": False, "reason": "no verifiable dense interpolant"}
            contact = contact_nonnegativity_conditions(vehicle)
        else:
            from simplified_vehicle_models.solvers.vertical_dense_certificate import (
                certify_vertical_dense_domain)
            if result.dense_bounds is not None:
                dense = certify_vertical_dense_domain(vehicle, plan, result.dense_bounds, summary,
                                                     pedal_dynamics=True)
            else:
                dense = {"certified": False, "reason": "no verifiable dense interpolant",
                         "scope": "vertical dense state-domain only (diagnostic)"}
            contact = None
        # Save the exact interpolant for post-run inspection.
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
        summary["accepted_policy_file"] = "accepted_policy.json"
        summary["accepted_policy_decisions"] = len(policy_decisions)
        summary["accepted_policy_status"] = ("complete" if policy_solve_completed else "accepted_prefix_incomplete")
        policy_record = summary.get("longitudinal_policy")
        if not isinstance(policy_record, dict):
            policy_record = {"policy": "braking-envelope", "settings_identity_status": "unavailable_before_policy_build"}
            summary["longitudinal_policy"] = policy_record
        if "settings_identity" in policy_record:
            policy_record["settings_identity_sha256"] = hashlib.sha256(
                canonical_json(policy_record["settings_identity"]).encode()).hexdigest()
            summary["settings"]["longitudinal_policy_settings_identity_sha256"] = policy_record["settings_identity_sha256"]
        else:
            policy_record["settings_identity_status"] = "unavailable_before_policy_build"
        if "builder" in policy_record or "array_sha256" in policy_record:
            policy_record["artifact_sha256"] = hashlib.sha256(
                canonical_json({"builder": policy_record.get("builder"),
                                "array_sha256": policy_record.get("array_sha256")}).encode()).hexdigest()
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
                summary.setdefault("suspension_travel_policy", "unlimited")
                summary.setdefault("suspension_travel_enforced", False)
                summary.setdefault("jounce_travel_limits_m", None)
                summary.setdefault("jounce_design_range_m", None)
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
    (attempt / "accepted_policy.json").write_text(json.dumps({"schema_version": 1,
        "policy": "hybrid-drive-brake-v1" if args.longitudinal_mode == "hybrid-drive-brake-v1" else args.longitudinal_policy, "status": "child_not_started",
        "accepted_decision_count": 0, "decisions_file": "accepted_policy.jsonl"}, indent=2) + "\n")
    (attempt / "accepted_policy.jsonl").write_text("")
    command = [sys.executable, "-u", str(Path(__file__).resolve()),
               *(sys.argv[1:] if argv is None else argv), "--child-attempt", str(attempt)]
    command.extend(("--expected-pedal-map-sha256", args.pedal_map_file_sha256 or ""))
    started = time.perf_counter()
    if args.physical_brake_planning or args.longitudinal_mode == "hybrid-drive-brake-v1":
        identity_sources = source_files(args.model, args.track, args.longitudinal_policy,
            args.vertical_corner_samples, args.transient_corner_planning, args.physical_brake_planning, args.longitudinal_mode)
        identity_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                           for path in identity_sources}
        args.physical_source_closure_sha256 = hashlib.sha256(
            canonical_json(identity_hashes).encode()).hexdigest()
        command.extend(("--expected-source-closure-sha256", args.physical_source_closure_sha256))
    child_settings_identity = policy_identity_settings(args)
    child_settings_identity_hash = (None if child_settings_identity is None else
        hashlib.sha256(canonical_json(child_settings_identity).encode()).hexdigest())
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
              **({"pedal_slew": pedal_slew_provenance(args)}
                 if pedal_slew_provenance(args) is not None else {}),
              "pedal_map": {"semantic_sha256": (args.pedal_map_config.sha256
                              if args.pedal_map_config.physical_calibration
                              else "no-pedal-map"),
                            "source_path": None if args.pedal_map_config_path is None else str(args.pedal_map_config_path),
                            "file_sha256": args.pedal_map_file_sha256},
              "total_wall_seconds": runtime, "within_wall_limit": runtime <= args.timeout_seconds,
              "timeout_seconds": args.timeout_seconds,
              "requested_wall_seconds": args.timeout_seconds,
              "elapsed_wall_seconds": runtime,
              "solver_wall_budget_seconds": solver_wall_budget(args.timeout_seconds, args.model),
              "model": args.model, "command": command,
              **({"longitudinal_policy": {"policy": args.longitudinal_policy,
                  "deceleration_margin": args.envelope_deceleration_margin,
                  "speed_query_points": args.envelope_speed_points,
                  "settings_identity_sha256": child_settings_identity_hash,
                   "preparation_settings_identity_sha256": child_settings_identity_hash,
                   "settings_identity_scope": "pre_child_preparation_config_not_actual_setup_identity"}}
                 ),
              "attempt": str(attempt)}
    (attempt / "parent.json").write_text(json.dumps(parent, indent=2) + "\n")
    sidecar_path = attempt / "accepted_policy.json"
    if sidecar_path.exists():
        sidecar = json.loads(sidecar_path.read_text())
        sidecar["parent_status"] = status
        sidecar["parent_return_code"] = return_code
        if status == "Timed_Out" and sidecar.get("status") in ("in_progress", "child_not_started"):
            sidecar["status"] = "timed_out_prefix_may_be_incomplete"
        sidecar_path.write_text(json.dumps(sidecar, indent=2) + "\n")
    print(json.dumps(parent), flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
