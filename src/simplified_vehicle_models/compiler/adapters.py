"""Register the reduction adapters and resolve one capability request.

An adapter converts a component that provides one capability into a value that
provides another capability. The compiler uses an adapter chain when no component
provides the requested capability directly.

The chain search is deterministic.
The search visits the adapters in registration order.
The search prefers the shortest chain.
The manifest records the chosen chain.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass

from simplified_vehicle_models.compiler.registry import (
    ComponentCandidate,
    LoadedComponent,
)


AdapterFunction = Callable[[object], object]


class CapabilityResolutionError(ValueError):
    """Report a capability that no component and no adapter chain satisfies."""


@dataclass(frozen=True, slots=True)
class AdapterStep:
    """Record one registered reduction adapter."""

    from_capability: str
    to_capability: str
    adapter_function: AdapterFunction

    @property
    def adapter_name(self) -> str:
        """Return the name of the adapter function."""

        return getattr(self.adapter_function, "__name__", repr(self.adapter_function))


@dataclass(frozen=True, slots=True)
class ResolutionPlan:
    """Record the provider and the adapter chain of one capability request."""

    capability: str
    provider: ComponentCandidate
    chain: tuple[AdapterStep, ...]

    @property
    def source_capability(self) -> str | None:
        """Return the capability of the provider component."""

        return self.provider.capability

    @property
    def is_direct(self) -> bool:
        """Return true when the provider satisfies the request without an adapter."""

        return not self.chain


@dataclass(frozen=True, slots=True)
class ResolutionResult:
    """Record the compiled value that satisfies one capability request."""

    capability: str
    component: LoadedComponent
    chain: tuple[AdapterStep, ...]
    value: object

    @property
    def source_capability(self) -> str | None:
        """Return the capability of the provider component."""

        return self.component.candidate.capability

    @property
    def is_direct(self) -> bool:
        """Return true when the provider satisfies the request without an adapter."""

        return not self.chain


_ADAPTERS: list[AdapterStep] = []


def register_adapter(
    from_capability: str,
    to_capability: str,
    adapter_function: AdapterFunction,
) -> AdapterStep:
    """Register one reduction adapter and reject a cycle.

    Reject a duplicate adapter for the same capability pair.
    Reject an adapter that closes a directed cycle.
    """

    if not from_capability.strip() or not to_capability.strip():
        raise ValueError("from_capability and to_capability must be nonempty")
    if not callable(adapter_function):
        raise ValueError("adapter_function must be callable")
    if any(
        step.from_capability == from_capability
        and step.to_capability == to_capability
        for step in _ADAPTERS
    ):
        raise ValueError(
            f"adapter {from_capability} -> {to_capability} is already registered"
        )
    if from_capability == to_capability or _reaches(to_capability, from_capability):
        raise ValueError(
            f"adapter {from_capability} -> {to_capability} creates an adapter cycle"
        )
    step = AdapterStep(
        from_capability=from_capability,
        to_capability=to_capability,
        adapter_function=adapter_function,
    )
    _ADAPTERS.append(step)
    return step


def unregister_adapter(step: AdapterStep) -> None:
    """Remove one registered adapter."""

    _ADAPTERS.remove(step)


def registered_adapters() -> tuple[AdapterStep, ...]:
    """Return every registered adapter in registration order."""

    return tuple(_ADAPTERS)


def plan_capability(
    capability: str,
    candidates: Iterable[ComponentCandidate],
) -> ResolutionPlan:
    """Plan the provider and the adapter chain for one capability request.

    Use the first provider in candidate order for a direct provision.
    Search the shortest adapter chain in registration order.
    Raise CapabilityResolutionError when no provider and no chain exists.
    """

    ordered = tuple(candidates)
    direct = [item for item in ordered if item.capability == capability]
    if direct:
        return ResolutionPlan(capability=capability, provider=direct[0], chain=())

    frontier: deque[tuple[str, tuple[AdapterStep, ...]]] = deque(
        [(capability, ())]
    )
    visited = {capability}
    while frontier:
        needed, steps = frontier.popleft()
        for adapter in _ADAPTERS:
            if adapter.to_capability != needed:
                continue
            following = adapter.from_capability
            if following in visited:
                continue
            extended = (adapter,) + steps
            provider = next(
                (item for item in ordered if item.capability == following),
                None,
            )
            if provider is not None:
                return ResolutionPlan(
                    capability=capability,
                    provider=provider,
                    chain=extended,
                )
            visited.add(following)
            frontier.append((following, extended))
    available = sorted(
        {item.capability for item in ordered if item.capability is not None}
    )
    reducible = sorted({step.from_capability for step in _ADAPTERS})
    raise CapabilityResolutionError(
        f"no component and no adapter chain provides capability {capability!r}; "
        f"components provide {available}, adapters accept {reducible}"
    )


def resolve_capability(
    capability: str,
    components: Iterable[LoadedComponent],
) -> ResolutionResult:
    """Resolve one capability request against compiled components.

    A direct provision has an empty chain.
    A reduction reports the adapter chain that the compiler applied.
    """

    ordered = tuple(components)
    plan = plan_capability(capability, (item.candidate for item in ordered))
    component = next(
        item for item in ordered if item.candidate == plan.provider
    )
    value = apply_adapter_chain(component.value, plan.chain)
    return ResolutionResult(
        capability=capability,
        component=component,
        chain=plan.chain,
        value=value,
    )


def apply_adapter_chain(value: object, chain: tuple[AdapterStep, ...]) -> object:
    """Apply each adapter of one chain to a value, in order."""

    for step in chain:
        value = step.adapter_function(value)
    return value


def _reaches(start: str, goal: str) -> bool:
    """Return true when a directed adapter path runs from start to goal."""

    if start == goal:
        return True
    visited = {start}
    frontier = [start]
    while frontier:
        current = frontier.pop()
        for step in _ADAPTERS:
            if step.from_capability != current:
                continue
            if step.to_capability == goal:
                return True
            if step.to_capability not in visited:
                visited.add(step.to_capability)
                frontier.append(step.to_capability)
    return False
