"""Parse one XML model, resolve each reference, and validate it against its descriptor.

The loader keeps three duties:
- parse one file;
- resolve one component reference;
- validate the model type against the registry descriptor.

The loader parses a model in one of three modes.
- FULL parses the model and every reference.
- SHALLOW parses the model and the identity of every reference.
- IDENTITY parses the root attributes and the reference chain only.

The compiler uses SHALLOW for the formulation and the assembly.
The compiler uses FULL for each component that the formulation uses.
The compiler never parses the parameters of an unused component.

The component modules own the parameter schemas and the compile functions.
"""

from __future__ import annotations

from enum import Enum
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree
from typing import cast

from simplified_vehicle_models.compiler.registry import (
    component_descriptor,
    register_builtin_components,
)
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.xml.model import Model, ModelReference


class _LoadMode(Enum):
    """Select how much of one model the loader reads."""

    FULL = "full"
    SHALLOW = "shallow"
    IDENTITY = "identity"


def load_model(path: str | Path) -> Model:
    """Parse one XML model and every reference."""

    return _load(path, _LoadMode.FULL)


def load_model_shallow(path: str | Path) -> Model:
    """Parse one XML model and the identity of every reference.

    Read the parameters and the requirements of the named model.
    Read only the root attributes and the reference chain of each reference.
    """

    return _load(path, _LoadMode.SHALLOW)


def load_point_mass_vehicle(path: str | Path) -> PointMassVehicleParameters:
    """Load and compile one point-mass vehicle formulation."""

    from simplified_vehicle_models.compiler.compile import compile_formulation

    return cast(PointMassVehicleParameters, compile_formulation(path).model)


def _load(path: str | Path, mode: _LoadMode) -> Model:
    register_builtin_components()
    source_path = Path(path).expanduser().resolve()
    return _load_model(source_path, (), {}, {}, mode)


def _load_model(
    path: Path,
    chain: tuple[Path, ...],
    cache: dict[tuple[Path, _LoadMode], Model],
    model_ids: dict[str, Path],
    mode: _LoadMode,
) -> Model:
    path = path.resolve()
    if path in chain:
        cycle_start = chain.index(path)
        cycle = chain[cycle_start:] + (path,)
        cycle_text = " -> ".join(str(item) for item in cycle)
        raise ValueError(f"component-reference cycle: {cycle_text}")
    key = (path, mode)
    if key in cache:
        return cache[key]

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
    try:
        component_descriptor(model_type, subtype)
    except ValueError as error:
        raise ValueError(f"{path}: {error}") from error

    prior_path = model_ids.get(model_id)
    if prior_path is not None and prior_path != path:
        raise ValueError(
            f"{path}: duplicate model id {model_id!r}; first used by {prior_path}"
        )
    model_ids[model_id] = path

    if mode is _LoadMode.IDENTITY:
        parameters: dict[str, float] = {}
        units: dict[str, str] = {}
        requires: tuple[str, ...] = ()
    else:
        parameters, units = _parse_parameters(root, path)
        requires = _parse_requires(root, path)

    reference_mode = (
        _LoadMode.FULL if mode is _LoadMode.FULL else _LoadMode.IDENTITY
    )
    reference_models: list[ModelReference] = []
    next_chain = chain + (path,)
    for element in root.iter():
        reference_path = element.get("ref")
        if reference_path is None:
            continue
        target_path = (path.parent / reference_path).resolve()
        target = _load_model(
            target_path,
            next_chain,
            cache,
            model_ids,
            reference_mode,
        )
        reference_models.append(
            ModelReference(tag=element.tag, role=element.get("role"), model=target)
        )

    model = Model(
        path=path,
        model_id=model_id,
        model_type=model_type,
        subtype=subtype,
        parameters=parameters,
        units=units,
        references=tuple(reference_models),
        requires=requires,
    )
    cache[key] = model
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


def _parse_requires(
    root: ElementTree.Element,
    path: Path,
) -> tuple[str, ...]:
    requires: list[str] = []
    for element in root.findall("./requires"):
        requires.append(_required_attribute(element, "capability", path))
    return tuple(requires)
