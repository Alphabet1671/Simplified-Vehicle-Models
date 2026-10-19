"""Quasi-static hydraulic circuit with gas accumulators at every junction.

The circuit has ``N`` junction nodes. Each node holds a polytropic gas volume
``g_k`` (the node's only state) and so has a pressure ``p_k = P_k(g_k)`` that
follows from the state alone. Wheel cylinders feed the nodes, and orifice lines
connect pairs of nodes. Fluid is incompressible and the lines carry no inertia,
so a line flow is an algebraic function of the pressure drop (Q4 option (a) of
the plan: accumulator and orifice states only).

Coupling to the wheels is one signed matrix ``areas`` (``N x 4``):

* ``areas[k, i]`` is the fluid volume that wheel ``i`` delivers into node ``k``
  per metre of compression, referred to the wheel jounce (so the motion ratio
  is already inside it). A rod-side chamber has a negative entry.
* the force on wheel ``i`` is ``F_i = sum_k areas[k, i] p_k`` (virtual work),
  positive in compression, with gauge pressures.

Conservation at node ``k``: the fluid volume ``V0_k - g_k`` changes by the wheel
delivery minus the line outflow,

    dg_k/dt = (sum of line flows leaving node k) - areas[k] . jounce_rate.

Static equilibrium. A closed circuit holds a fixed fluid charge, the charge at
zero jounce and the design pressures. With open lines all nodes of a connected
component share one pressure ``p``, which solves

    sum_j g_j(p) = sum_j g_j(p_design) - sum_k areas[k] . z,

summed over the nodes of the component. ``equilibrium_volumes`` solves it by
bisection.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.hydraulics.elements import (
    ATMOSPHERE,
    AccumulatorSet,
    CylinderSet,
    OrificeSet,
)
from simplified_vehicle_models.suspension.errors import DomainError

#: Relative tolerance of the compile-time check that every line joins nodes of
#: equal design pressure (otherwise the design state would not be at rest).
DESIGN_PRESSURE_RTOL = 1.0e-9
_BISECTIONS = 80


@dataclass(frozen=True)
class CircuitRates:
    """Result of one circuit evaluation."""

    pressure: np.ndarray  # gauge pressure of every node [Pa]
    gas_rate: np.ndarray  # dg/dt of every node [m^3/s]
    line_flow: np.ndarray  # flow of every line, from -> to [m^3/s]


@dataclass(frozen=True)
class HydraulicCircuit:
    """Junction nodes, wheel areas, orifice lines and gas accumulators."""

    node_names: tuple[str, ...]
    areas: np.ndarray  # (N, 4) signed [m^2]
    accumulators: AccumulatorSet
    design_pressure: np.ndarray  # gauge [Pa], one per node
    line_from: np.ndarray  # int indices
    line_to: np.ndarray
    orifices: OrificeSet

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.node_names)
        count = len(names)
        if count < 1 or len(set(names)) != count:
            raise ValueError("node names must be unique and not empty")
        areas = np.array(self.areas, dtype=float)
        if areas.shape != (count, 4) or not np.all(np.isfinite(areas)):
            raise ValueError(f"areas must be a finite ({count}, 4) matrix")
        if self.accumulators.size != count:
            raise ValueError("every node needs exactly one accumulator")
        design = np.array(self.design_pressure, dtype=float).reshape(-1)
        if design.shape != (count,) or not np.all(np.isfinite(design)):
            raise ValueError("design_pressure must hold one finite value per node")
        line_from = np.array(self.line_from, dtype=int).reshape(-1)
        line_to = np.array(self.line_to, dtype=int).reshape(-1)
        if line_from.shape != line_to.shape or line_from.size != self.orifices.size:
            raise ValueError("line_from, line_to and the orifices must have equal length")
        if line_from.size and (
            line_from.min() < 0 or line_to.min() < 0
            or line_from.max() >= count or line_to.max() >= count
        ):
            raise ValueError("a line refers to a node that does not exist")
        if np.any(line_from == line_to):
            raise ValueError("a line may not join a node to itself")
        # Design state must be at rest: gas volumes at the design pressure are in
        # range, and a line joins nodes of equal design pressure.
        self.accumulators.volume_at(design)  # raises DomainError out of range
        for a, b in zip(line_from, line_to):
            scale = max(abs(design[a]), abs(design[b]), 1.0)
            if abs(design[a] - design[b]) > DESIGN_PRESSURE_RTOL * scale:
                raise ValueError(
                    f"line {names[a]}-{names[b]} joins nodes with different design pressures")
        for array in (areas, design, line_from, line_to):
            array.setflags(write=False)
        object.__setattr__(self, "node_names", names)
        object.__setattr__(self, "areas", areas)
        object.__setattr__(self, "design_pressure", design)
        object.__setattr__(self, "line_from", line_from)
        object.__setattr__(self, "line_to", line_to)
        object.__setattr__(self, "_components", self._connected_components())

    # -- structure ----------------------------------------------------------------
    @property
    def size(self) -> int:
        return len(self.node_names)

    @property
    def state_names(self) -> tuple[str, ...]:
        return tuple(f"gas_volume_{name}" for name in self.node_names)

    def _connected_components(self) -> tuple[np.ndarray, ...]:
        parent = list(range(self.size))

        def find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for a, b in zip(self.line_from, self.line_to):
            parent[find(int(a))] = find(int(b))
        groups: dict[int, list[int]] = {}
        for node in range(self.size):
            groups.setdefault(find(node), []).append(node)
        return tuple(np.array(members, dtype=int) for members in groups.values())

    @property
    def components(self) -> tuple[np.ndarray, ...]:
        return self._components

    # -- static -------------------------------------------------------------------
    def design_volumes(self) -> np.ndarray:
        """Return the gas volumes at the design pressures (the zero-jounce state)."""

        return self.accumulators.volume_at(self.design_pressure)

    def design_forces(self) -> np.ndarray:
        """Return the corner forces at zero jounce and the design pressures."""

        return CylinderSet(self.areas).force(self.design_pressure)

    def forces_from_state(self, gas_volume) -> np.ndarray:
        """Return the corner forces ``areas^T p(g)`` of a gas-volume state."""

        return CylinderSet(self.areas).force(self.accumulators.pressure(gas_volume))

    def equilibrium_volumes(self, jounce) -> np.ndarray:
        """Return the gas volumes of the closed circuit at rest at ``jounce``.

        Raises ``DomainError`` when the fluid charge does not fit the
        accumulators (a membrane on its stop or the compressed-gas stop).
        """

        z = np.asarray(jounce, dtype=float)
        if z.shape != (4,):
            raise ValueError("jounce must hold four values")
        design = self.design_volumes()
        acc = self.accumulators
        result = np.empty(self.size)
        delivered = self.areas @ z  # fluid pushed into each node [m^3]
        for members in self._components:
            target = float(np.sum(design[members]) - np.sum(delivered[members]))
            low_limit = float(np.sum(acc.minimum_volume[members]))
            high_limit = float(np.sum(acc.precharge_volume[members]))
            if not (low_limit <= target <= high_limit):
                raise DomainError(
                    f"hydraulic charge of {[self.node_names[m] for m in members]} does not fit "
                    f"its accumulators at this jounce: gas {target:.6g} m^3 outside "
                    f"[{low_limit:.6g}, {high_limit:.6g}]")
            result[members] = _volumes_for_total(acc, members, target)
        return result

    # -- dynamics -----------------------------------------------------------------
    def evaluate(self, gas_volume, jounce_rate) -> CircuitRates:
        """Return pressures, state rates and line flows at a state and wheel rates."""

        zdot = np.asarray(jounce_rate, dtype=float)
        pressure = self.accumulators.pressure(gas_volume)  # raises DomainError
        outflow = np.zeros(self.size)
        flow = np.zeros(0)
        if self.orifices.size:
            flow, _ = self.orifices.flow(pressure[self.line_from] - pressure[self.line_to])
            np.add.at(outflow, self.line_from, flow)
            np.add.at(outflow, self.line_to, -flow)
        cylinder = CylinderSet(self.areas)
        return CircuitRates(pressure=pressure, gas_rate=outflow - cylinder.flow(zdot),
                            line_flow=flow)


def _volumes_for_total(acc: AccumulatorSet, members: np.ndarray, target: float) -> np.ndarray:
    """Return gas volumes of ``members`` at one common pressure with total ``target``."""

    p0 = acc.precharge_pressure[members] + ATMOSPHERE
    v0 = acc.precharge_volume[members]
    n = acc.exponent[members]
    gmin = acc.minimum_volume[members]

    def volumes(absolute: float) -> np.ndarray:
        return np.minimum(v0, v0 * (p0 / absolute) ** (1.0 / n))

    low = float(np.min(p0)) * 0.5
    high = float(np.max(p0 * (v0 / gmin) ** n)) * 1.0000001
    for _ in range(_BISECTIONS):
        middle = 0.5 * (low + high)
        if float(np.sum(volumes(middle))) > target:
            low = middle
        else:
            high = middle
    return volumes(0.5 * (low + high))
