"""Hydraulically interconnected layout: corner springs plus gas-orifice cylinder circuits.

Per wheel the force is

* the wheel-referred corner spring curve (it carries the sprung static load and
  the bump stop, exactly as in the independent-corner layout),
* the cylinder force change ``sum_k areas[k, i] (p_k - p_design_k)`` of a
  ``HydraulicCircuit`` (``hydraulics/circuit.py``). The design-pressure force is
  subtracted, so the circuit adds interconnection stiffness and no static load:
  the corner springs keep carrying the sprung static load at zero jounce, and
* the damper table at the damper velocity ``zdot / MR`` returned through ``/ MR``.

Why the springs stay. A gas spring that must carry a corner load ``F`` over a
travel ``z`` has a rate of at most about ``n F / z`` (the gas must not be
compressed to zero), far below the baseline wheel rate. The hydraulic part of
this layout therefore adds the interconnected stiffness (roll, pitch, warp
circuits) and leaves heave stiffness and the bump stop to the springs.

States (Q4 option (a)): the dimensionless gas fraction ``x_k = g_k / V0_k`` of
every junction node. Lines have no volume and no inertia. ``x`` lies in
``[g_min / V0, 1]``; a state outside raises ``DomainError`` and the plant turns it
into a rejected trial.

``state=None`` means "the circuit at rest at this jounce" (all lines open and
fully equalised). That is the static equilibrium of the static solve; it is *not*
the instantaneous state of a running plant.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from simplified_vehicle_models.hydraulics.circuit import HydraulicCircuit
from simplified_vehicle_models.hydraulics.elements import OrificeSet
from simplified_vehicle_models.suspension.elements import CornerSprings
from simplified_vehicle_models.suspension.errors import DomainError
from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics, _four
from simplified_vehicle_models.suspension.network import CompiledSuspension, NetworkForces

LAYOUT = "hydraulic"
STATIC_CHECK_RTOL = 1.0e-9
#: Periodic-closure tolerance of a gas fraction (dimensionless).
GAS_PERIODIC_TOLERANCE = 5.0e-3
_AXLES = ((0, (0, 1)), (1, (2, 3)))


@dataclass(frozen=True)
class HydraulicNetwork:
    """Corner springs, hydraulic circuit and dampers of the hydraulic layout."""

    kinematics: SuspensionKinematics
    springs: CornerSprings
    circuit: HydraulicCircuit
    dampers: tuple  # (front, rear), each with ``force(velocity)``

    has_inertia = False

    def __post_init__(self) -> None:
        if len(self.dampers) != 2:
            raise ValueError("dampers must be (front, rear)")

    # -- configuration ----------------------------------------------------------
    @property
    def n_states(self) -> int:
        return self.circuit.size

    @property
    def state_names(self) -> tuple[str, ...]:
        return tuple(f"gas_fraction_{name}" for name in self.circuit.node_names)

    @property
    def state_tolerances(self) -> np.ndarray:
        """Periodic-closure tolerance of every state."""

        return np.full(self.n_states, GAS_PERIODIC_TOLERANCE)

    @property
    def state_bounds(self) -> np.ndarray:
        """Physical ``(low, high)`` range of every state, shape ``(n_states, 2)``."""

        acc = self.circuit.accumulators
        return np.column_stack((acc.minimum_volume / acc.precharge_volume, np.ones(self.n_states)))

    def with_kinematics(self, kinematics: SuspensionKinematics) -> HydraulicNetwork:
        return replace(self, kinematics=kinematics)

    def with_circuit(self, circuit: HydraulicCircuit) -> HydraulicNetwork:
        return replace(self, circuit=circuit)

    def with_dampers(self, front, rear) -> HydraulicNetwork:
        return replace(self, dampers=(front, rear))

    def with_line_scale(self, factor: float) -> HydraulicNetwork:
        """Return a copy whose orifice resistances are multiplied by ``factor``."""

        if not (np.isfinite(factor) and factor > 0.0):
            raise ValueError("line resistance scale must be finite and positive")
        orifices = self.circuit.orifices
        scaled = OrificeSet(linear=orifices.linear * factor,
                            quadratic=orifices.quadratic * factor)
        return self.with_circuit(replace(self.circuit, orifices=scaled))

    # -- states -----------------------------------------------------------------
    def _volumes(self, state) -> np.ndarray:
        x = np.asarray(state, dtype=float)
        if x.shape != (self.n_states,):
            raise ValueError(f"network state must hold {self.n_states} gas fractions")
        return x * self.circuit.accumulators.precharge_volume

    def initial_state(self, z_static=None) -> np.ndarray:
        """Return the gas fractions of the circuit at rest at ``z_static``."""

        z = np.zeros(4) if z_static is None else _four("jounce", z_static)
        return (self.circuit.equilibrium_volumes(z)
                / self.circuit.accumulators.precharge_volume)

    # -- forces -----------------------------------------------------------------
    def forces(self, jounce, jounce_rate, state=None) -> NetworkForces:
        z = _four("jounce", jounce)
        zdot = _four("jounce_rate", jounce_rate)
        if state is None:
            gas = self.circuit.equilibrium_volumes(z)
            gas_rate = np.zeros(self.n_states)
        else:
            gas = self._volumes(state)
            gas_rate = (self.circuit.evaluate(gas, zdot).gas_rate
                        / self.circuit.accumulators.precharge_volume)
        hydraulic = self.circuit.forces_from_state(gas) - self.circuit.design_forces()
        spring = np.empty(4)
        damper = np.empty(4)
        velocity = np.empty(4)
        flag = np.zeros(4, dtype=bool)
        for axle, (left, right) in _AXLES:
            pair = [left, right]
            spring[pair] = self.springs.axle(axle)(z[pair])
            ratio = self.kinematics.axle(axle).motion_ratio(z[pair])
            v = zdot[pair] / ratio
            result = self.dampers[axle].force(v)
            velocity[pair] = v
            damper[pair] = np.asarray(result.force, dtype=float) / ratio
            flag[pair] = np.asarray(result.out_of_range, dtype=bool)
        return NetworkForces(
            total=spring + hydraulic + damper,
            elements={"spring": spring, "hydraulic": hydraulic, "damper": damper},
            damper_velocity=velocity,
            out_of_range=flag,
            ds_dt=gas_rate,
        )

    def static_forces(self, jounce, state=None) -> np.ndarray:
        """Return the spring and cylinder force of the circuit at rest (or at ``state``)."""

        z = _four("jounce", jounce)
        spring = np.empty(4)
        for axle, (left, right) in _AXLES:
            spring[[left, right]] = self.springs.axle(axle)(z[[left, right]])
        gas = self.circuit.equilibrium_volumes(z) if state is None else self._volumes(state)
        return spring + self.circuit.forces_from_state(gas) - self.circuit.design_forces()

    def static_check(self, jounce=None) -> np.ndarray:
        z = np.zeros(4) if jounce is None else jounce
        return self.static_forces(z)

    def pressures(self, state) -> np.ndarray:
        """Return the gauge pressure of every node for a state."""

        return self.circuit.accumulators.pressure(self._volumes(state))

    # -- inertia ----------------------------------------------------------------
    def inertia_block(self, jounce) -> np.ndarray:
        return np.zeros((4, 4))

    def inertial_forcing(self, jounce, jounce_rate) -> np.ndarray:
        return np.zeros(4)


def build_hydraulic(
    *,
    kinematics: SuspensionKinematics,
    springs: CornerSprings,
    circuit: HydraulicCircuit,
    damper_front,
    damper_rear,
) -> CompiledSuspension:
    """Assemble the hydraulic suspension and check statics and travel.

    * zero jounce must carry the sprung static corner loads (rtol 1e-9);
    * the closed circuit must fit its accumulators over the full travel of every
      wheel (each wheel moved alone to both limits).
    """

    network = HydraulicNetwork(kinematics=kinematics, springs=springs, circuit=circuit,
                               dampers=(damper_front, damper_rear))
    expected = kinematics.static_sprung_load
    actual = network.static_check()
    if not np.allclose(actual, expected, rtol=STATIC_CHECK_RTOL, atol=0.0):
        raise ValueError(
            "the springs and hydraulic circuit do not carry the sprung static corner loads at "
            f"zero jounce: got {actual.tolist()} N, expected {expected.tolist()} N"
        )
    limits = kinematics.jounce_limits
    for wheel in range(4):
        for bound in limits[wheel]:
            z = np.zeros(4)
            z[wheel] = bound
            try:
                circuit.equilibrium_volumes(z)
            except DomainError as error:
                raise ValueError(
                    f"the hydraulic circuit does not fit the travel of wheel {wheel} "
                    f"at jounce {bound:g} m: {error}") from error
    return CompiledSuspension(kinematics=kinematics, network=network, layout=LAYOUT)
