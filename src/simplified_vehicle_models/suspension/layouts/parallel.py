"""Additive physical corner and axle-mode suspension force network.

Motion ratio definition. For every strut, ``motion_ratio`` is the travel of one
wheel divided by the end-to-end travel of the real spring/damper unit, in the
mode that the strut resists:

* corner strut: wheel travel ``z_i`` / shaft travel;
* heave strut: the travel of one wheel in pure heave (``z_L = z_R = z``) /
  shaft travel, so shaft travel is ``z / MR``;
* roll strut: the travel of one wheel in pure roll (``z_R = -z_L = z``) /
  shaft travel, so shaft travel is ``z / MR``.

``ROWS`` is therefore the mean ``(z_L + z_R) / 2`` for heave and the half
difference ``(z_R - z_L) / 2`` for roll. Shaft velocity uses the same Jacobian.
By virtual work, the wheel force is ``A.T @ shaft_force``. Shaft spring rates,
preloads, damper tables and inertances are physical shaft values, not wheel
rates. In pure heave or pure roll, one strut gives each wheel a rate of
``k / (2 * MR**2)``: both wheels share the one unit. A roll strut gives the axle a
roll stiffness of ``k * half_track**2 / MR**2``.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics, _four
from simplified_vehicle_models.suspension.network import CompiledSuspension, NetworkForces

LAYOUT = "parallel-corner-axle"
STATIC_CHECK_RTOL = 1.0e-9
STRUT_NAMES = ('corner-fl', 'corner-fr', 'corner-rl', 'corner-rr',
               'heave-front', 'roll-front', 'heave-rear', 'roll-rear')
ROWS = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1],
                 [.5,.5,0,0], [-.5,.5,0,0], [0,0,.5,.5], [0,0,-.5,.5]], dtype=float)


@dataclass(frozen=True)
class ParallelNetwork:
    kinematics: SuspensionKinematics
    struts: tuple
    n_states = 0

    def __post_init__(self):
        if len(self.struts) != 8:
            raise ValueError("struts must contain eight elements in STRUT_NAMES order")

    @property
    def jacobian(self):
        return ROWS * np.array([s.orientation / s.motion_ratio for s in self.struts])[:, None]

    @property
    def has_inertia(self):
        return any(s.inertance > 0. for s in self.struts)

    def with_kinematics(self, kinematics):
        return replace(self, kinematics=kinematics)

    def initial_state(self, z_static=None):
        return np.zeros(0)

    def _evaluate(self, z, zdot):
        z = _four('jounce', z); zdot = _four('jounce_rate', zdot)
        A = self.jacobian
        x, v = A @ z, A @ zdot
        total, damp, flags = np.zeros(8), np.zeros(8), np.zeros(8, dtype=bool)
        for i, s in enumerate(self.struts):
            if s.spring_rate == 0 and s.damper_scale == 0 and s.preload == 0:
                continue
            f, d, flag = s.force(x[i], v[i])
            total[i], damp[i], flags[i] = float(f), float(d), bool(flag)
        return z, zdot, A, x, v, total, damp, flags

    def forces(self, jounce, jounce_rate, state=None):
        z, zdot, A, x, v, shaft, damping, flags = self._evaluate(jounce, jounce_rate)
        contributions = {key: np.zeros(4) for key in ('preload','spring','damper')}
        for i, s in enumerate(self.struts):
            enabled = s.spring_rate > 0 or s.damper_scale > 0 or s.preload > 0
            if enabled:
                contributions['preload'] += A[i] * s.preload
                contributions['spring'] += A[i] * s.spring_rate * x[i]
                contributions['damper'] += A[i] * damping[i]
        wheel_v = np.zeros(4); wheel_flag = np.zeros(4, dtype=bool)
        for wheel in range(4):
            connected = [i for i in range(8) if A[i, wheel] != 0 and self.struts[i].damper_scale > 0]
            if connected:
                wheel_v[wheel] = max(abs(v[i]) for i in connected)
                wheel_flag[wheel] = any(flags[i] for i in connected)
        element_velocity = {name: float(v[i]) for i, name in enumerate(STRUT_NAMES)}
        element_force = {name: float(shaft[i]) for i, name in enumerate(STRUT_NAMES)}
        element_flags = {name: bool(flags[i]) for i, name in enumerate(STRUT_NAMES)}
        return NetworkForces(A.T @ shaft, contributions, wheel_v, wheel_flag,
                             element_velocity=element_velocity, element_force=element_force,
                             element_out_of_range=element_flags)

    def static_forces(self, jounce, state=None):
        z = _four('jounce', jounce)
        A = self.jacobian; x = A @ z
        shaft = np.array([s.preload + s.spring_rate*x[i] if s.spring_rate or s.preload else 0.
                          for i,s in enumerate(self.struts)])
        return A.T @ shaft

    def static_check(self, jounce=None):
        return self.static_forces(np.zeros(4) if jounce is None else jounce)

    def inertia_block(self, jounce):
        z = _four('jounce', jounce)
        A = self.jacobian
        b = np.array([s.inertance for s in self.struts])
        return A.T @ (b[:, None] * A)

    def inertial_forcing(self, jounce, jounce_rate):
        z = _four('jounce', jounce)
        _four('jounce_rate', jounce_rate)
        return np.zeros(4)


def build_parallel_suspension(*, kinematics, struts) -> CompiledSuspension:
    network = ParallelNetwork(kinematics, tuple(struts))
    expected, actual = kinematics.static_sprung_load, network.static_check()
    if not np.allclose(actual, expected, rtol=STATIC_CHECK_RTOL, atol=0.0):
        raise ValueError(f"physical strut zero-jounce forces {actual.tolist()} N do not match static sprung loads {expected.tolist()} N")
    return CompiledSuspension(kinematics, network, LAYOUT)
