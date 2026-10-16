"""Static vertical equilibrium for the 14-DoF vehicle model."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.aero.fixed_aero import aerodynamic_forces
from simplified_vehicle_models.formulations.planar_7dof import wheel_positions
from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
from simplified_vehicle_models.tires.vertical import tire_deflection_load


@dataclass(frozen=True, slots=True)
class VerticalState:
    """Mechanical first-order state, in the fixed 21-value layout."""

    v_x: float = 0.0
    v_y: float = 0.0
    v_z: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    heave: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    wheel_speed_fl: float = 0.0
    wheel_speed_fr: float = 0.0
    wheel_speed_rl: float = 0.0
    wheel_speed_rr: float = 0.0
    jounce_fl: float = 0.0
    jounce_fr: float = 0.0
    jounce_rl: float = 0.0
    jounce_rr: float = 0.0
    jounce_rate_fl: float = 0.0
    jounce_rate_fr: float = 0.0
    jounce_rate_rl: float = 0.0
    jounce_rate_rr: float = 0.0

    def as_vector(self) -> np.ndarray:
        """Return values in the documented mechanical state order."""
        return np.asarray(tuple(getattr(self, field) for field in _STATE_FIELDS), dtype=float)

    @classmethod
    def from_vector(cls, vector) -> VerticalState:
        """Build a state from exactly 21 finite values."""
        values = np.asarray(vector, dtype=float)
        if values.shape != (21,):
            raise ValueError(f"vertical state vector must have shape (21,), got {values.shape}")
        if not np.all(np.isfinite(values)):
            raise ValueError("vertical state vector must be finite")
        return cls(*map(float, values))


_STATE_FIELDS = tuple(VerticalState.__dataclass_fields__)


def solve_static_equilibrium(
    vehicle: VerticalVehicleParameters,
    speed: float = 0.0,
) -> VerticalState:
    """Solve static ride equilibrium in heave, roll, pitch, and four jounces.

    Spring preload is defined at zero jounce; consequently the equilibrium
    coordinates are solved while each spring force remains referenced to that
    design position. Tire forces and aero load are included in the force/moment
    balance. This solve deliberately uses only NumPy.
    """
    speed = float(speed)
    if not np.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and nonnegative")

    planar = vehicle.planar
    downforce, _ = aerodynamic_forces(planar.aero, speed)
    positions = wheel_positions(planar)
    gravity = 9.80665

    def residual(q):
        heave, roll, pitch, *jounce = q
        z = np.asarray(jounce)
        # Positive-down heave; positive SAE roll lowers the left side. Pitch
        # is positive nose-up, hence front suspension compression decreases.
        wheel_vertical = heave + positions[:, 1] * roll - positions[:, 0] * pitch
        deflection = wheel_vertical + z
        tire = tire_deflection_load(planar.tire, deflection, np.zeros(4), 0.0)
        spring = _spring_forces(vehicle, z)
        wheel_load = tire.load
        body_vertical = np.sum(spring) - vehicle.sprung_mass * gravity - downforce
        roll_moment = spring[0] + spring[2] - spring[1] - spring[3]
        front_aero = downforce * planar.aero.aero_balance
        front_target = (vehicle.sprung_mass * gravity * vehicle.sprung_front_fraction
                        + front_aero)
        pitch_moment = np.sum(spring[:2]) - front_target
        unsprung = np.array([
            vehicle.unsprung_mass_front, vehicle.unsprung_mass_front,
            vehicle.unsprung_mass_rear, vehicle.unsprung_mass_rear,
        ])
        wheel_balance = wheel_load - spring - unsprung * gravity
        return np.r_[body_vertical, roll_moment, pitch_moment, wheel_balance]

    # Eight coordinates: body heave/roll/pitch and four jounces. Newton steps
    # use a scaled finite-difference Jacobian and backtracking for robustness.
    q = np.zeros(7)  # heave, roll, pitch, and four jounces
    for _ in range(80):
        r = residual(q)
        if np.linalg.norm(r, ord=np.inf) < 1e-8:
            break
        jac = np.empty((7, 7))
        for column in range(7):
            step = 1e-6 if column in (0, 3, 4, 5, 6) else 1e-7
            trial = q.copy()
            trial[column] += step
            jac[:, column] = (residual(trial) - r) / step
        delta, *_ = np.linalg.lstsq(jac, -r, rcond=None)
        scale = 1.0
        norm = np.linalg.norm(r)
        while scale > 1e-6 and np.linalg.norm(residual(q + scale * delta)) >= norm:
            scale *= 0.5
        q += scale * delta
    else:
        raise RuntimeError("static vertical equilibrium did not converge")
    if np.linalg.norm(residual(q), ord=np.inf) > 1e-6:
        raise RuntimeError("static vertical equilibrium residual exceeds tolerance")

    state = np.zeros(21)
    state[0] = speed
    state[6:9] = q[:3]
    state[9:13] = speed / planar.tire.r_1
    state[13:17] = q[3:]
    return VerticalState.from_vector(state)


def _spring_forces(vehicle: VerticalVehicleParameters, jounce: np.ndarray) -> np.ndarray:
    """Evaluate only the preload-bearing spring and ARB contribution."""
    result = np.empty(4)
    for axle, (left, right) in ((vehicle.suspension.front, (0, 1)),
                                 (vehicle.suspension.rear, (2, 3))):
        pair = jounce[[left, right]]
        result[[left, right]] = axle.wheel_rate_spring(pair)
        arb = 0.5 * axle.arb_wheel_rate * (pair[0] - pair[1])
        result[left] += arb
        result[right] -= arb
    return result
