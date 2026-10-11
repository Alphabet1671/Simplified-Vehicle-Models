"""Compile one formulation against one assembly.

The compiler loads the formulation, finds the assembly, resolves every requested
capability, and records the decision in a manifest.

Only the components that satisfy a request are compiled.
An unused assembly component is recorded in the manifest and is never compiled.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path

from simplified_vehicle_models.compiler.adapters import (
    CapabilityResolutionError,
    ResolutionResult,
    apply_adapter_chain,
    plan_capability,
)
from simplified_vehicle_models.compiler.manifest import (
    CapabilityRecord,
    CompileManifest,
    ComponentRecord,
    ModelIdentity,
    UnconsumedParameter,
)
from simplified_vehicle_models.compiler.registry import (
    ComponentCandidate,
    LoadedComponent,
    component_descriptor,
    register_builtin_components,
)
from simplified_vehicle_models.xml.loader import load_model, load_model_shallow
from simplified_vehicle_models.xml.model import Model


FormulationCompileFunction = Callable[
    [Model, Mapping[str, ResolutionResult]],
    object,
]


@dataclass(frozen=True, slots=True)
class CompiledFormulation:
    """Store one compiled formulation and its compile manifest."""

    model: object
    manifest: CompileManifest


@dataclass(frozen=True, slots=True)
class FormulationDefinition:
    """Describe how one formulation subtype compiles against an assembly."""

    assembly_type: str
    requested_capabilities: tuple[str, ...]
    consumed_parameters: tuple[str, ...]
    compile_function: FormulationCompileFunction


def compile_formulation(formulation_path: str | Path) -> CompiledFormulation:
    """Compile one formulation file and return the model and its manifest."""

    register_builtin_components()
    source_path = Path(formulation_path).expanduser().resolve()
    formulation = load_model_shallow(source_path)
    definition = _formulation_definition(formulation)

    sources = [
        reference
        for reference in formulation.references
        if reference.tag == "source"
    ]
    if len(sources) != 1 or len(formulation.references) != 1:
        raise ValueError(
            f"{formulation.path}: the formulation must contain one source reference"
        )
    source_identity = sources[0].model
    assembly = load_model_shallow(source_identity.path)
    descriptor = component_descriptor(assembly.model_type, assembly.subtype)
    if descriptor.model_type != definition.assembly_type:
        raise ValueError(
            f"{formulation.path}: the source must be a {definition.assembly_type}; "
            f"received {assembly.model_type}/{assembly.subtype}"
        )

    candidates = _assembly_candidates(assembly)
    requested = _requested_capabilities(formulation, definition)
    plans = []
    for capability in requested:
        try:
            plans.append(plan_capability(capability, candidates))
        except CapabilityResolutionError as error:
            raise CapabilityResolutionError(
                f"{assembly.path}: {error}"
            ) from error

    loaded: dict[str, LoadedComponent] = {}
    for plan in plans:
        role = plan.provider.role
        if role not in loaded:
            loaded[role] = compile_component(plan.provider)
    resolutions: dict[str, ResolutionResult] = {}
    for plan in plans:
        component = loaded[plan.provider.role]
        resolutions[plan.capability] = ResolutionResult(
            capability=plan.capability,
            component=component,
            chain=plan.chain,
            value=apply_adapter_chain(component.value, plan.chain),
        )

    model = definition.compile_function(assembly, resolutions)
    manifest = _build_manifest(
        formulation=formulation,
        assembly=assembly,
        definition=definition,
        candidates=candidates,
        loaded=loaded,
        resolutions=resolutions,
    )
    compiled = _attach_manifest(model, manifest)
    return CompiledFormulation(model=compiled, manifest=manifest)


def compile_component(candidate: ComponentCandidate) -> LoadedComponent:
    """Fully load and compile one candidate component.

    The candidate may hold an identity-only model.
    This function reads the model file again and parses every parameter.
    """

    model = load_model(candidate.model.path)
    descriptor = component_descriptor(model.model_type, model.subtype)
    if not descriptor.is_component:
        raise ValueError(
            f"{model.path}: model type {model.model_type}/{model.subtype} "
            "is not a component type"
        )
    compile_function = descriptor.compile_function
    if compile_function is None:
        raise ValueError(
            f"{model.path}: model type "
            f"{descriptor.model_type}/{descriptor.subtype} "
            "provides no compiled component"
        )
    full_candidate = ComponentCandidate(
        role=candidate.role,
        model=model,
        descriptor=descriptor,
    )
    return LoadedComponent(
        candidate=full_candidate,
        value=compile_function(model),
    )


def _assembly_candidates(assembly: Model) -> tuple[ComponentCandidate, ...]:
    candidates: list[ComponentCandidate] = []
    roles: set[str] = set()
    for reference in assembly.references:
        if reference.tag != "component" or reference.role is None:
            raise ValueError(
                f"{assembly.path}: each vehicle reference must be a component"
            )
        if reference.role in roles:
            raise ValueError(
                f"{assembly.path}: duplicate component role {reference.role!r}"
            )
        roles.add(reference.role)
        descriptor = component_descriptor(
            reference.model.model_type,
            reference.model.subtype,
        )
        if not descriptor.is_component:
            raise ValueError(
                f"{assembly.path}: component role {reference.role!r} references "
                f"{reference.model.model_type}/{reference.model.subtype}, "
                "which is not a component type"
            )
        candidates.append(
            ComponentCandidate(
                role=reference.role,
                model=reference.model,
                descriptor=descriptor,
            )
        )
    return tuple(candidates)


def _requested_capabilities(
    formulation: Model,
    definition: FormulationDefinition,
) -> tuple[str, ...]:
    requested = formulation.requires or definition.requested_capabilities
    ordered: list[str] = []
    for capability in requested:
        if capability not in ordered:
            ordered.append(capability)
    return tuple(ordered)


def _build_manifest(
    *,
    formulation: Model,
    assembly: Model,
    definition: FormulationDefinition,
    candidates: tuple[ComponentCandidate, ...],
    loaded: dict[str, LoadedComponent],
    resolutions: dict[str, ResolutionResult],
) -> CompileManifest:
    unused = tuple(
        ComponentRecord.of(candidate)
        for candidate in candidates
        if candidate.role not in loaded
    )
    consumed = set(definition.consumed_parameters)
    unconsumed = tuple(
        UnconsumedParameter(
            name=name,
            unit=assembly.units[name],
            value=assembly.parameters[name],
        )
        for name in sorted(set(assembly.parameters) - consumed)
    )
    capabilities = tuple(
        CapabilityRecord(
            capability=result.capability,
            source_capability=result.source_capability,
            component_role=result.component.role,
            component_model_id=result.component.model.model_id,
            adapter_chain=result.chain,
        )
        for result in resolutions.values()
    )
    return CompileManifest(
        formulation=ModelIdentity.of(formulation),
        assembly=ModelIdentity.of(assembly),
        loaded_components=tuple(
            ComponentRecord.of(component.candidate)
            for component in loaded.values()
        ),
        capabilities=capabilities,
        unused_components=unused,
        unconsumed_parameters=unconsumed,
    )


def _attach_manifest(model: object, manifest: CompileManifest) -> object:
    if not dataclasses.is_dataclass(model):
        raise TypeError(
            "the compiled model must be a dataclass to hold the manifest; "
            f"received {type(model).__name__}"
        )
    names = {field.name for field in dataclasses.fields(model)}
    if "manifest" not in names:
        raise ValueError(
            f"the compiled model {type(model).__name__} has no manifest field"
        )
    return dataclasses.replace(model, manifest=manifest)


def _formulation_definition(formulation: Model) -> FormulationDefinition:
    key = (formulation.model_type, formulation.subtype)
    if key == ("vehicle-formulation", "point-mass"):
        from simplified_vehicle_models.formulations.point_mass import (
            POINT_MASS_REQUESTED_CAPABILITIES,
            POINT_MASS_VEHICLE_PARAMETER_UNITS,
            compile_point_mass_vehicle,
        )

        return FormulationDefinition(
            assembly_type="vehicle-assembly",
            requested_capabilities=POINT_MASS_REQUESTED_CAPABILITIES,
            consumed_parameters=tuple(POINT_MASS_VEHICLE_PARAMETER_UNITS),
            compile_function=compile_point_mass_vehicle,
        )
    raise ValueError(
        f"{formulation.path}: unknown formulation type "
        f"{formulation.model_type}/{formulation.subtype}"
    )
