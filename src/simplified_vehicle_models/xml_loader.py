from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from simplified_vehicle_models.parameters import (
    AeroParameters,
    DrivetrainParameters,
    PointMassVehicleParameters,
    TireEnvelopeParameters,
)


_REGISTERED_MODELS = frozenset(
    {
        ("aero-model", "fixed-aero"),
        ("drivetrain-model", "four-wheel-electric"),
        ("tire-model", "load-sensitive-friction-ellipse"),
        ("vehicle-assembly", "point-mass"),
        ("vehicle-formulation", "point-mass"),
    }
)


@dataclass(frozen=True, slots=True)
class _Reference:
    tag: str
    role: str | None
    model: _Model


@dataclass(frozen=True, slots=True)
class _Model:
    path: Path
    model_id: str
    model_type: str
    subtype: str
    parameters: dict[str, float]
    units: dict[str, str]
    references: tuple[_Reference, ...]


def load_point_mass_vehicle(path: str | Path) -> PointMassVehicleParameters:
    """Load and compile one point-mass vehicle formulation."""

    source_path = Path(path).expanduser().resolve()
    model = _load_model(source_path, (), {}, {})
    _require_model_type(model, "vehicle-formulation", "point-mass")

    sources = [
        reference for reference in model.references if reference.tag == "source"
    ]
    if len(sources) != 1 or len(model.references) != 1:
        raise ValueError(
            f"{model.path}: the formulation must contain one source reference"
        )

    return _compile_vehicle(sources[0].model)


def _load_model(
    path: Path,
    chain: tuple[Path, ...],
    cache: dict[Path, _Model],
    model_ids: dict[str, Path],
) -> _Model:
    path = path.resolve()
    if path in chain:
        cycle_start = chain.index(path)
        cycle = chain[cycle_start:] + (path,)
        cycle_text = " -> ".join(str(item) for item in cycle)
        raise ValueError(f"component-reference cycle: {cycle_text}")
    if path in cache:
        return cache[path]

    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read XML model {path}: {error}") from error

    if root.tag != "model":
        raise ValueError(f"{path}: the root element must be model")
    if root.get("schema-version") != "1.0":
        raise ValueError(f"{path}: unsupported schema version")

    model_id = _required_attribute(root, "id", path)
    model_type = _required_attribute(root, "type", path)
    subtype = _required_attribute(root, "subtype", path)
    if (model_type, subtype) not in _REGISTERED_MODELS:
        raise ValueError(f"{path}: unknown model type {model_type}/{subtype}")

    prior_path = model_ids.get(model_id)
    if prior_path is not None and prior_path != path:
        raise ValueError(
            f"{path}: duplicate model id {model_id!r}; first used by {prior_path}"
        )
    model_ids[model_id] = path

    parameters, units = _parse_parameters(root, path)
    reference_models: list[_Reference] = []
    next_chain = chain + (path,)
    for element in root.iter():
        reference_path = element.get("ref")
        if reference_path is None:
            continue
        target_path = (path.parent / reference_path).resolve()
        target = _load_model(target_path, next_chain, cache, model_ids)
        reference_models.append(
            _Reference(tag=element.tag, role=element.get("role"), model=target)
        )

    model = _Model(
        path=path,
        model_id=model_id,
        model_type=model_type,
        subtype=subtype,
        parameters=parameters,
        units=units,
        references=tuple(reference_models),
    )
    cache[path] = model
    return model


def _required_attribute(
    element: ElementTree.Element,
    name: str,
    path: Path,
) -> str:
    value = element.get(name)
    if value is None or not value.strip():
        raise ValueError(f"{path}: missing required attribute {name!r}")
    return value


def _parse_parameters(
    root: ElementTree.Element,
    path: Path,
) -> tuple[dict[str, float], dict[str, str]]:
    parameters: dict[str, float] = {}
    units: dict[str, str] = {}
    for element in root.findall("./parameters/parameter"):
        name = _required_attribute(element, "name", path)
        if name in parameters:
            raise ValueError(f"{path}: duplicate parameter {name!r}")
        text = _required_attribute(element, "value", path)
        try:
            value = float(text)
        except ValueError as error:
            raise ValueError(
                f"{path}: parameter {name!r} must contain a valid float"
            ) from error
        if not math.isfinite(value):
            raise ValueError(f"{path}: parameter {name!r} must contain a finite float")
        parameters[name] = value
        units[name] = _required_attribute(element, "unit", path)
    return parameters, units


def _compile_vehicle(model: _Model) -> PointMassVehicleParameters:
    _require_model_type(model, "vehicle-assembly", "point-mass")
    values = _require_parameters(
        model,
        {
            "total_mass": "kg",
            "wheelbase": "m",
            "track_width": "m",
            "cg_height": "m",
            "front_mass_fraction": "1",
            "lateral_load_transfer_distribution": "1",
            "brake_bias": "1",
        },
    )

    components: dict[str, _Model] = {}
    for reference in model.references:
        if reference.tag != "component" or reference.role is None:
            raise ValueError(
                f"{model.path}: each vehicle reference must be a component"
            )
        if reference.role in components:
            raise ValueError(
                f"{model.path}: duplicate component role {reference.role!r}"
            )
        components[reference.role] = reference.model

    expected_roles = {"tire", "aero", "drivetrain"}
    if set(components) != expected_roles:
        missing = sorted(expected_roles - set(components))
        unknown = sorted(set(components) - expected_roles)
        message = (
            f"{model.path}: invalid component roles; "
            f"missing={missing}, unknown={unknown}"
        )
        raise ValueError(message)

    _require_positive(model, values, "total_mass", "wheelbase", "track_width")
    _require_nonnegative(model, values, "cg_height")
    _require_open_unit_interval(model, values, "front_mass_fraction", "brake_bias")
    _require_closed_unit_interval(
        model,
        values,
        "lateral_load_transfer_distribution",
    )
    return PointMassVehicleParameters(
        total_mass=values["total_mass"],
        wheelbase=values["wheelbase"],
        track_width=values["track_width"],
        cg_height=values["cg_height"],
        front_mass_fraction=values["front_mass_fraction"],
        lateral_load_transfer_distribution=values[
            "lateral_load_transfer_distribution"
        ],
        brake_bias=values["brake_bias"],
        tire=_compile_tire(components["tire"]),
        aero=_compile_aero(components["aero"]),
        drivetrain=_compile_drivetrain(components["drivetrain"]),
    )


def _compile_tire(model: _Model) -> TireEnvelopeParameters:
    _require_model_type(model, "tire-model", "load-sensitive-friction-ellipse")
    values = _require_parameters(
        model,
        {
            "longitudinal_mu_scale": "1",
            "lateral_mu_scale": "1",
            "load_sensitivity_slope": "1/N",
            "load_sensitivity_intercept": "1",
            "effective_radius": "m",
        },
    )
    _require_no_references(model)
    _require_positive(
        model,
        values,
        "longitudinal_mu_scale",
        "lateral_mu_scale",
        "load_sensitivity_intercept",
        "effective_radius",
    )
    if values["load_sensitivity_slope"] > 0.0:
        raise ValueError(
            f"{model.path}: load_sensitivity_slope must be nonpositive"
        )
    return TireEnvelopeParameters(**values)


def _compile_aero(model: _Model) -> AeroParameters:
    _require_model_type(model, "aero-model", "fixed-aero")
    values = _require_parameters(
        model,
        {
            "air_density": "kg/m^3",
            "downforce_area": "m^2",
            "drag_area": "m^2",
            "aero_balance": "1",
        },
    )
    _require_no_references(model)
    _require_positive(model, values, "air_density")
    _require_nonnegative(model, values, "downforce_area", "drag_area")
    _require_closed_unit_interval(model, values, "aero_balance")
    return AeroParameters(**values)


def _compile_drivetrain(model: _Model) -> DrivetrainParameters:
    _require_model_type(model, "drivetrain-model", "four-wheel-electric")
    values = _require_parameters(
        model,
        {
            "motor_count": "1",
            "gear_ratio": "1",
            "peak_motor_torque": "N*m",
            "power_limit": "W",
            "efficiency": "1",
            "maximum_motor_speed": "rpm",
        },
    )
    _require_no_references(model)
    motor_count = values.pop("motor_count")
    if not motor_count.is_integer() or motor_count < 1:
        raise ValueError(f"{model.path}: motor_count must be a positive integer")
    _require_positive(
        model,
        values,
        "gear_ratio",
        "peak_motor_torque",
        "power_limit",
        "maximum_motor_speed",
    )
    _require_open_closed_unit_interval(model, values, "efficiency")
    return DrivetrainParameters(motor_count=int(motor_count), **values)


def _require_positive(
    model: _Model,
    values: dict[str, float],
    *names: str,
) -> None:
    for name in names:
        if values[name] <= 0.0:
            raise ValueError(f"{model.path}: {name} must be positive")


def _require_nonnegative(
    model: _Model,
    values: dict[str, float],
    *names: str,
) -> None:
    for name in names:
        if values[name] < 0.0:
            raise ValueError(f"{model.path}: {name} must be nonnegative")


def _require_open_unit_interval(
    model: _Model,
    values: dict[str, float],
    *names: str,
) -> None:
    for name in names:
        if not 0.0 < values[name] < 1.0:
            raise ValueError(f"{model.path}: {name} must be between zero and one")


def _require_closed_unit_interval(
    model: _Model,
    values: dict[str, float],
    *names: str,
) -> None:
    for name in names:
        if not 0.0 <= values[name] <= 1.0:
            raise ValueError(f"{model.path}: {name} must be from zero through one")


def _require_open_closed_unit_interval(
    model: _Model,
    values: dict[str, float],
    *names: str,
) -> None:
    for name in names:
        if not 0.0 < values[name] <= 1.0:
            raise ValueError(f"{model.path}: {name} must be greater than zero")


def _require_model_type(model: _Model, model_type: str, subtype: str) -> None:
    if (model.model_type, model.subtype) != (model_type, subtype):
        raise ValueError(
            f"{model.path}: expected model type {model_type}/{subtype}, "
            f"received {model.model_type}/{model.subtype}"
        )


def _require_parameters(
    model: _Model,
    expected_units: dict[str, str],
) -> dict[str, float]:
    expected_names = set(expected_units)
    actual_names = set(model.parameters)
    missing = sorted(expected_names - actual_names)
    unknown = sorted(actual_names - expected_names)
    if missing or unknown:
        raise ValueError(
            f"{model.path}: invalid parameters; missing={missing}, unknown={unknown}"
        )
    for name, expected_unit in expected_units.items():
        unit = model.units[name]
        if unit != expected_unit:
            raise ValueError(
                f"{model.path}: parameter {name!r} must use unit {expected_unit!r}"
            )
    return dict(model.parameters)


def _require_no_references(model: _Model) -> None:
    if model.references:
        raise ValueError(f"{model.path}: component models cannot contain references")
