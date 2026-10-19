"""The force-network seam between a suspension layout and the vertical evaluator.

A force network maps the four wheel jounces, the four jounce rates and any
internal states to four wheel-referred corner forces. The evaluator builds the
generalised forces from those corner forces by virtual work, so every layout
(independent corners, axle or four-mode modal springs, a hydraulic circuit)
plugs in at this one place.

Sign convention: a corner force is positive in compression. It pushes the wheel
down and the body up.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Mapping, Protocol, runtime_checkable

import numpy as np

from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics


@dataclass(frozen=True)
class NetworkForces:
    """Wheel-referred corner forces of one network evaluation.

    ``total`` is the force on each corner (N, order FL, FR, RL, RR).
    ``elements`` names the per-corner contribution of each force element, for
    example ``spring``, ``arb`` and ``damper`` of the independent-corner layout;
    the keys depend on the layout. ``damper_velocity`` is the velocity (m/s) at
    the dampers, positive in compression. ``out_of_range`` flags corners whose
    damper velocity left its table. ``ds_dt`` is the rate of the network's own
    states, empty for a stateless layout.
    """

    total: np.ndarray
    elements: Mapping[str, np.ndarray]
    damper_velocity: np.ndarray
    out_of_range: np.ndarray
    ds_dt: np.ndarray = field(default_factory=lambda: np.zeros(0))


@runtime_checkable
class ForceNetwork(Protocol):
    """The corner-force law of one suspension layout."""

    #: Number of internal network states (0 for a stateless layout).
    n_states: int
    #: True when the network adds inertia (inerter) to the jounce coordinates.
    has_inertia: bool

    def initial_state(self, z_static) -> np.ndarray:
        """Return the internal state at the static jounces ``z_static``."""

    def forces(self, jounce, jounce_rate, state) -> NetworkForces:
        """Return the corner forces; raise ``DomainError`` outside a table."""

    def static_forces(self, jounce, state) -> np.ndarray:
        """Return the preload-bearing (rate-free) corner forces for the static solve."""

    def static_check(self, jounce=None) -> np.ndarray:
        """Return the rate-free corner forces at zero jounce."""

    def inertia_block(self, jounce) -> np.ndarray:
        """Return the 4x4 jounce-coordinate inertia the network adds (kg)."""

    def inertial_forcing(self, jounce, jounce_rate) -> np.ndarray:
        """Return the velocity-squared forcing of the jounce rows of a varying inertia."""

    def with_kinematics(self, kinematics: SuspensionKinematics) -> ForceNetwork:
        """Return the same network bound to other kinematics."""


@dataclass(frozen=True)
class CompiledSuspension:
    """A compiled suspension: kinematics plus the force network of one layout."""

    kinematics: SuspensionKinematics
    network: ForceNetwork
    layout: str

    def with_kinematics(self, kinematics: SuspensionKinematics) -> CompiledSuspension:
        """Return a copy with other kinematics, rebound in the network."""

        return replace(self, kinematics=kinematics,
                       network=self.network.with_kinematics(kinematics))

    def with_axle_geometry(self, axle: int, **changes) -> CompiledSuspension:
        """Return a copy with fields of one axle's geometry replaced."""

        return self.with_kinematics(self.kinematics.with_axle(axle, **changes))

    def with_network(self, network: ForceNetwork) -> CompiledSuspension:
        """Return a copy with another force network."""

        return replace(self, network=network)
