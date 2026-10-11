"""Record what the compiler loaded, used, reduced, and ignored.

One manifest reports the full compile decision:
- the formulation and the assembly;
- every component that the compiler compiled and used;
- every capability request and the way that the compiler satisfied it;
- every assembly component that the formulation did not use;
- every assembly parameter that the formulation did not consume.

The compiled model holds its manifest.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from simplified_vehicle_models.compiler.adapters import AdapterStep
from simplified_vehicle_models.compiler.registry import ComponentCandidate
from simplified_vehicle_models.xml.model import Model


@dataclass(frozen=True, slots=True)
class ModelIdentity:
    """Identify one XML model."""

    model_id: str
    model_type: str
    subtype: str
    path: Path

    @classmethod
    def of(cls, model: Model) -> ModelIdentity:
        """Build the identity of one loaded model."""

        return cls(
            model_id=model.model_id,
            model_type=model.model_type,
            subtype=model.subtype,
            path=model.path,
        )


@dataclass(frozen=True, slots=True)
class ComponentRecord:
    """Record one assembly component."""

    role: str
    model_id: str
    model_type: str
    subtype: str
    path: Path
    capability: str | None

    @classmethod
    def of(cls, candidate: ComponentCandidate) -> ComponentRecord:
        """Build the record of one assembly component."""

        return cls(
            role=candidate.role,
            model_id=candidate.model.model_id,
            model_type=candidate.model.model_type,
            subtype=candidate.model.subtype,
            path=candidate.model.path,
            capability=candidate.capability,
        )


@dataclass(frozen=True, slots=True)
class CapabilityRecord:
    """Record one capability request and the way that the compiler satisfied it."""

    capability: str
    source_capability: str | None
    component_role: str
    component_model_id: str
    adapter_chain: tuple[AdapterStep, ...]

    @property
    def is_direct(self) -> bool:
        """Return true when the provider satisfies the request without an adapter."""

        return not self.adapter_chain

    @property
    def is_reduced(self) -> bool:
        """Return true when the compiler applied at least one adapter."""

        return bool(self.adapter_chain)

    @property
    def adapter_names(self) -> tuple[str, ...]:
        """Return the names of the applied adapters."""

        return tuple(step.adapter_name for step in self.adapter_chain)


@dataclass(frozen=True, slots=True)
class UnconsumedParameter:
    """Record one assembly parameter that the formulation did not consume."""

    name: str
    unit: str
    value: float


@dataclass(frozen=True, slots=True)
class CompileManifest:
    """Report the full compile decision of one formulation."""

    formulation: ModelIdentity
    assembly: ModelIdentity
    loaded_components: tuple[ComponentRecord, ...]
    capabilities: tuple[CapabilityRecord, ...]
    unused_components: tuple[ComponentRecord, ...]
    unconsumed_parameters: tuple[UnconsumedParameter, ...]

    def capability(self, name: str) -> CapabilityRecord | None:
        """Return the record of one capability request, or None."""

        for record in self.capabilities:
            if record.capability == name:
                return record
        return None

    def component(self, role: str) -> ComponentRecord | None:
        """Return the record of one loaded component, or None."""

        for record in self.loaded_components:
            if record.role == role:
                return record
        return None

    def unused_roles(self) -> tuple[str, ...]:
        """Return the roles of every unused assembly component."""

        return tuple(record.role for record in self.unused_components)

    def unconsumed_names(self) -> tuple[str, ...]:
        """Return the names of every unconsumed assembly parameter."""

        return tuple(record.name for record in self.unconsumed_parameters)

    def as_text(self) -> str:
        """Return a report that a human can read. Do not print it."""

        lines = [
            f"formulation: {self.formulation.model_id} "
            f"({self.formulation.model_type}/{self.formulation.subtype}) "
            f"{self.formulation.path}",
            f"assembly: {self.assembly.model_id} "
            f"({self.assembly.model_type}/{self.assembly.subtype}) "
            f"{self.assembly.path}",
            "loaded components:",
        ]
        for record in self.loaded_components:
            lines.append(
                f"  {record.role}: {record.model_id} "
                f"({record.model_type}/{record.subtype}) "
                f"capability={record.capability} {record.path}"
            )
        lines.append("capabilities:")
        for record in self.capabilities:
            if record.is_direct:
                lines.append(
                    f"  {record.capability}: direct from {record.component_role}"
                )
            else:
                chain = " -> ".join(
                    f"{step.from_capability}:{step.adapter_name}"
                    for step in record.adapter_chain
                )
                chain = f"{chain} -> {record.capability}"
                lines.append(
                    f"  {record.capability}: reduced from "
                    f"{record.component_role} through {chain}"
                )
        lines.append("unused components:")
        for record in self.unused_components:
            lines.append(
                f"  {record.role}: {record.model_id} "
                f"({record.model_type}/{record.subtype}) "
                f"capability={record.capability} {record.path}"
            )
        lines.append("unconsumed parameters:")
        for record in self.unconsumed_parameters:
            lines.append(f"  {record.name}={record.value} {record.unit}")
        return "\n".join(lines)
