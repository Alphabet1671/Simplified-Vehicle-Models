"""Validate one component or one assembly against a parameter schema.

A component owns a closed schema. An unknown parameter in a component is an error.

An assembly owns an open schema. An unknown parameter in an assembly is allowed.
The compiler records each unknown assembly parameter as unconsumed.

A required parameter that is absent is always an error.
"""

from __future__ import annotations

from simplified_vehicle_models.xml.model import Model


def closed_parameters(
    model: Model,
    expected_units: dict[str, str],
) -> dict[str, float]:
    """Validate a closed parameter schema and return the values.

    Raise ValueError for a missing name, an unknown name, or a wrong unit.
    """

    missing, unknown = _collect_names(model, expected_units)
    if missing or unknown:
        raise ValueError(
            f"{model.path}: invalid parameters; "
            f"missing={list(missing)}, unknown={list(unknown)}"
        )
    _check_units(model, expected_units)
    return _values(model, expected_units)


def required_parameters(
    model: Model,
    expected_units: dict[str, str],
) -> tuple[dict[str, float], tuple[str, ...]]:
    """Validate the required parameters of an open schema.

    Return the values and the unknown parameter names.
    Raise ValueError for a missing name or a wrong unit.
    """

    missing, unknown = _collect_names(model, expected_units)
    if missing:
        raise ValueError(
            f"{model.path}: missing required parameters {list(missing)}; "
            f"unconsumed parameters {list(unknown)}"
        )
    _check_units(model, expected_units)
    return _values(model, expected_units), tuple(unknown)


def _collect_names(
    model: Model,
    expected_units: dict[str, str],
) -> tuple[list[str], list[str]]:
    expected_names = set(expected_units)
    actual_names = set(model.parameters)
    missing = sorted(expected_names - actual_names)
    unknown = sorted(actual_names - expected_names)
    return missing, unknown


def _values(model: Model, expected_units: dict[str, str]) -> dict[str, float]:
    return {name: model.parameters[name] for name in expected_units}


def _check_units(model: Model, expected_units: dict[str, str]) -> None:
    for name, expected_unit in expected_units.items():
        unit = model.units[name]
        if unit != expected_unit:
            raise ValueError(
                f"{model.path}: parameter {name!r} must use unit {expected_unit!r}"
            )


def require_model_type(model: Model, model_type: str, subtype: str) -> None:
    """Raise ValueError when the model type does not match."""

    if (model.model_type, model.subtype) != (model_type, subtype):
        raise ValueError(
            f"{model.path}: expected model type {model_type}/{subtype}, "
            f"received {model.model_type}/{model.subtype}"
        )


def require_no_references(model: Model) -> None:
    """Raise ValueError when a component contains a reference."""

    if model.references:
        raise ValueError(f"{model.path}: component models cannot contain references")


def require_positive(model: Model, values: dict[str, float], *names: str) -> None:
    """Raise ValueError when a value is not positive."""

    for name in names:
        if values[name] <= 0.0:
            raise ValueError(f"{model.path}: {name} must be positive")


def require_nonnegative(model: Model, values: dict[str, float], *names: str) -> None:
    """Raise ValueError when a value is negative."""

    for name in names:
        if values[name] < 0.0:
            raise ValueError(f"{model.path}: {name} must be nonnegative")


def require_open_unit_interval(
    model: Model,
    values: dict[str, float],
    *names: str,
) -> None:
    """Raise ValueError when a value is not between zero and one."""

    for name in names:
        if not 0.0 < values[name] < 1.0:
            raise ValueError(f"{model.path}: {name} must be between zero and one")


def require_closed_unit_interval(
    model: Model,
    values: dict[str, float],
    *names: str,
) -> None:
    """Raise ValueError when a value is outside the range from zero through one."""

    for name in names:
        if not 0.0 <= values[name] <= 1.0:
            raise ValueError(f"{model.path}: {name} must be from zero through one")


def require_open_closed_unit_interval(
    model: Model,
    values: dict[str, float],
    *names: str,
) -> None:
    """Raise ValueError when a value is not greater than zero and at most one."""

    for name in names:
        if not 0.0 < values[name] <= 1.0:
            raise ValueError(f"{model.path}: {name} must be greater than zero")
