"""Register the model types and look up one descriptor by type and subtype.

A descriptor states the capability of one model type and the function that compiles it.
The loader uses the registry to accept or to reject one model type.
The compiler uses the registry to compile one component.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from simplified_vehicle_models.xml.model import Model


CompileFunction = Callable[[Model], object]

COMPONENT_KIND = "component"
ASSEMBLY_KIND = "assembly"
FORMULATION_KIND = "formulation"

MODEL_KINDS = (COMPONENT_KIND, ASSEMBLY_KIND, FORMULATION_KIND)


@dataclass(frozen=True, slots=True)
class ComponentDescriptor:
    """Describe one registered model type and subtype."""

    model_type: str
    subtype: str
    kind: str
    capability: str | None
    compile_function: CompileFunction | None
    #: True when the model type holds child models through ``ref=`` elements.
    nests_components: bool = False
    #: Roles of the children. A nesting component needs every role exactly once.
    child_roles: tuple[str, ...] = ()

    @property
    def nests_assembly(self) -> bool:
        """Return true for an assembly whose children the assembly compiler resolves.

        The powertrain and drivetrain assemblies are of this kind. The vehicle
        assembly resolves their capabilities through a synthetic provider.
        """

        return self.nests_components and self.kind == ASSEMBLY_KIND

    @property
    def nests_component(self) -> bool:
        """Return true for a component that compiles its own children."""

        return self.nests_components and self.kind == COMPONENT_KIND

    @property
    def key(self) -> tuple[str, str]:
        """Return the type and the subtype of this descriptor."""

        return (self.model_type, self.subtype)

    @property
    def is_component(self) -> bool:
        """Return true when the descriptor names a compilable component."""

        return self.kind == COMPONENT_KIND


@dataclass(frozen=True, slots=True)
class ComponentCandidate:
    """Pair one referenced component with its descriptor, before compilation."""

    role: str
    model: Model
    descriptor: ComponentDescriptor

    @property
    def capability(self) -> str | None:
        """Return the capability that the component provides."""

        return self.descriptor.capability


@dataclass(frozen=True, slots=True)
class LoadedComponent:
    """Pair one compiled component with its candidate."""

    candidate: ComponentCandidate
    value: object
    #: Compiled children of a nesting component, in ``child_roles`` order.
    children: tuple[LoadedComponent, ...] = ()

    @property
    def role(self) -> str:
        """Return the role of the component in the assembly."""

        return self.candidate.role

    @property
    def model(self) -> Model:
        """Return the model of the component."""

        return self.candidate.model

    @property
    def descriptor(self) -> ComponentDescriptor:
        """Return the descriptor of the component."""

        return self.candidate.descriptor


_DESCRIPTORS: dict[tuple[str, str], ComponentDescriptor] = {}
_BOOTSTRAPPED = False


def register_component(
    model_type: str,
    subtype: str,
    *,
    kind: str = COMPONENT_KIND,
    capability: str | None = None,
    compile_function: CompileFunction | None = None,
    nests_components: bool = False,
    child_roles: tuple[str, ...] = (),
) -> ComponentDescriptor:
    """Register one model type and subtype.

    The kind is COMPONENT for a compilable component.
    The kind is ASSEMBLY or FORMULATION for a model that composes other models.
    The capability is None for an assembly or a formulation.
    The compile function is None for a model that composes other models.

    ``nests_components`` marks a model that holds child models. A nesting
    COMPONENT names its ``child_roles``; the compiler compiles each child and
    calls ``compile_function(model, children)`` with a mapping of role to
    compiled child value. A nesting ASSEMBLY keeps its own compile path.
    """

    if not model_type.strip() or not subtype.strip():
        raise ValueError("model_type and subtype must be nonempty")
    if kind not in MODEL_KINDS:
        raise ValueError(f"kind {kind!r} is not one of {list(MODEL_KINDS)}")
    child_roles = tuple(child_roles)
    if child_roles and not nests_components:
        raise ValueError("child_roles require nests_components=True")
    if len(set(child_roles)) != len(child_roles):
        raise ValueError(f"duplicate child role in {list(child_roles)}")
    if nests_components and kind == COMPONENT_KIND and not child_roles:
        raise ValueError("a nesting component must name its child roles")
    if nests_components and kind == FORMULATION_KIND:
        raise ValueError("a formulation cannot be registered as nesting components")
    key = (model_type, subtype)
    if key in _DESCRIPTORS:
        raise ValueError(f"model type {model_type}/{subtype} is already registered")
    descriptor = ComponentDescriptor(
        model_type=model_type,
        subtype=subtype,
        kind=kind,
        capability=capability,
        compile_function=compile_function,
        nests_components=nests_components,
        child_roles=child_roles,
    )
    _DESCRIPTORS[key] = descriptor
    return descriptor


def component_descriptor(model_type: str, subtype: str) -> ComponentDescriptor:
    """Return the descriptor of one model type and subtype."""

    register_builtin_components()
    descriptor = _DESCRIPTORS.get((model_type, subtype))
    if descriptor is None:
        raise ValueError(f"unknown model type {model_type}/{subtype}")
    return descriptor


def registered_descriptors() -> tuple[ComponentDescriptor, ...]:
    """Return every registered descriptor in registration order."""

    register_builtin_components()
    return tuple(_DESCRIPTORS.values())


def register_builtin_components() -> None:
    """Register the production model types.

    This function imports each production component module one time.
    Each component module registers itself on import.
    The bootstrap flag is set after the imports succeed.
    """

    global _BOOTSTRAPPED

    if _BOOTSTRAPPED:
        return
    from simplified_vehicle_models.aero import fixed_aero
    from simplified_vehicle_models.drivetrain import controller, lumped, transmission
    from simplified_vehicle_models.tires import (
        friction_ellipse,
        unitire_simple,
        unitire_transient,
    )
    from simplified_vehicle_models.powertrain import (
        assembly,
        energy_storage,
        motor_envelope,
        power_plant,
    )
    from simplified_vehicle_models.suspension import components

    _ = (
        fixed_aero,
        controller,
        lumped,
        transmission,
        friction_ellipse,
        unitire_simple,
        unitire_transient,
        energy_storage,
        assembly,
        motor_envelope,
        power_plant,
        components,
    )
    register_component(
        "vehicle-assembly",
        "four-wheel",
        kind=ASSEMBLY_KIND,
    )
    register_component(
        "vehicle-formulation",
        "point-mass",
        kind=FORMULATION_KIND,
    )
    register_component(
        "vehicle-formulation",
        "planar-7dof",
        kind=FORMULATION_KIND,
    )
    register_component(
        "vehicle-formulation",
        "14dof-simple",
        kind=FORMULATION_KIND,
    )
    _BOOTSTRAPPED = True
