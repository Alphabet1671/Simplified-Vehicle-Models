"""Express the planar vehicle physics for a nonlinear program.

The NumPy model in ``formulations/planar_7dof.py`` solves the coupled vertical
load and tire force with a fixed-point iteration. A nonlinear program must not
nest that loop. This module instead treats the four vertical loads as
**algebraic variables** and returns the consistency equations together with the
state derivatives, so the solver closes the loop exactly.

This module duplicates the tire and load equations in symbolic form. That
duplication is deliberate. The plan for this project allows a second
implementation when a cross-comparison is the verification strategy, and
``tests/test_planar_ocp.py`` compares this physics against the NumPy model at
matched states.

CasADi is an optional dependency. Import this module only when the optimal
control path is used.
"""

from __future__ import annotations

from dataclasses import dataclass

import casadi as ca
import numpy as np

from simplified_vehicle_models.core.conventions import GRAVITY
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters


#: Below this slip the source uses the linear stiffness branch.
_LINEAR_SLIP_FLOOR = 1.0e-4

#: Numerical guard inside the slip square roots. It keeps the Jacobian finite
#: at zero slip. It is far below every physical slip, so it changes no result.
_SLIP_GUARD = 1.0e-14


@dataclass(frozen=True, slots=True)
class SymbolicTire:
    """Store the symbolic tire outputs of one evaluation."""

    longitudinal_force: object
    lateral_force: object
    aligning_moment: object
    rolling_moment: object
    loaded_radius: object
    slip_ratio: object
    slip_angle: object
    phi: object
    f_bar: object
    longitudinal_friction: object


def _polyval(coefficients: tuple[float, ...], value):
    """Evaluate one polynomial in descending order with Horner's rule."""

    result = ca.SX(0.0)
    for coefficient in coefficients:
        result = result * value + coefficient
    return result


def _smooth_max(first, second, epsilon: float):
    """Return a smooth approximation of the larger of two values."""

    return 0.5 * (first + second + ca.sqrt((first - second) ** 2 + epsilon**2))


def symbolic_loaded_radius(parameters, vertical_load):
    """Return the loaded radius of one tire at a symbolic vertical load."""

    load_ratio = vertical_load / parameters.f_z_rated
    radius_from_load = (
        parameters.r_1
        + parameters.r_2 * load_ratio
        + parameters.r_3 * load_ratio**2
    )
    lateral_shift = parameters.f_y_shift_0 + parameters.f_y_shift_fz * load_ratio
    radius_from_lateral = (
        parameters.k_rl_0
        + parameters.k_rl_1 * load_ratio
        + parameters.k_rl_2 * load_ratio**2
    ) * lateral_shift**2
    return ca.fmax(radius_from_load + radius_from_lateral, 1.0e-4)


def symbolic_tire(
    parameters,
    slip_angle,
    slip_ratio,
    vertical_load,
    wheel_speed,
) -> SymbolicTire:
    """Return the tire forces and moments of one wheel.

    The equations mirror ``tires.unitire_simple.evaluate_unitire`` exactly,
    except that the source's hard branches become smooth or clamped forms:

    - the contact load is clamped to a compressive value;
    - the small-slip linear branch uses ``if_else`` on the same test;
    - the maximum of a force floor uses ``fmax``.
    """

    f_z = ca.fmax(vertical_load, 0.0)
    load_ratio = f_z / parameters.f_z_rated
    radius = symbolic_loaded_radius(parameters, f_z)

    k_x = _polyval(parameters.k_x, load_ratio)
    k_y = _polyval(parameters.k_y, load_ratio)
    k_cx = parameters.k_cx
    k_cy = parameters.k_cy

    s_x = slip_ratio
    s_y = ca.tan(slip_angle)

    v_roll = ca.fmax(ca.fabs(wheel_speed) * radius, parameters.vroll_min)
    v_sx = ca.fabs(s_x) * v_roll
    v_sy = ca.fabs(s_y) * v_roll

    def friction_law(slip_speed, mu_0, mu_s, shape, transition):
        argument = ca.fabs(slip_speed) / transition
        soft = ca.log(argument + ca.exp(-argument))
        return mu_s + (mu_0 - mu_s) * ca.exp(-(shape**2) * soft**2)

    mu_x = friction_law(
        v_sx, parameters.mu_0_x, parameters.mu_s_x, parameters.h_x, parameters.v_m_x
    )
    mu_y = friction_law(
        v_sy, parameters.mu_0_y, parameters.mu_s_y, parameters.h_y, parameters.v_m_y
    )

    phi_x = k_x * s_x / ca.fmax(mu_x * f_z, parameters.force_eps)
    phi_y = k_y * s_y / ca.fmax(mu_y * f_z, parameters.force_eps)
    # The guard keeps the derivative finite when both slips are zero. The
    # linear branch below handles that point, but the derivative of a square
    # root at zero is not finite, and a solver needs a finite Jacobian.
    phi = ca.sqrt(phi_x**2 + phi_y**2 + _SLIP_GUARD)

    f_bar = 1.0 - ca.exp(
        -phi
        - parameters.e_1 * phi**2
        - (parameters.e_1**2 + 1.0 / 12.0) * phi**3
    )
    lam = 1.0 + (k_y / ca.fmax(k_x, parameters.stiff_eps) - 1.0) * f_bar
    denominator = ca.sqrt((lam * phi_x) ** 2 + phi_y**2 + _SLIP_GUARD)

    # Use the linear branch below the slip floor, exactly as the source does.
    linear = denominator < parameters.slip_eps
    safe = ca.if_else(linear, 1.0, denominator)
    longitudinal = ca.if_else(
        linear,
        k_x * s_x,
        f_bar * (lam * phi_x / safe) * mu_x * f_z,
    )
    lateral = ca.if_else(
        linear,
        k_y * s_y,
        f_bar * (phi_y / safe) * mu_y * f_z,
    )

    d_x_0 = _polyval(parameters.d_x_0, load_ratio)
    d_e = _polyval(parameters.d_e, load_ratio)
    d_1 = _polyval(parameters.d_1, load_ratio)
    d_2 = _polyval(parameters.d_2, load_ratio)
    trail = (d_x_0 + d_e) * ca.exp(-d_1 * phi - d_2 * phi**2) - d_e

    carcass_x = longitudinal / ca.fmax(k_cx, parameters.stiff_eps)
    carcass_y = lateral / ca.fmax(k_cy, parameters.stiff_eps)
    aligning = -(lateral * (trail + carcass_x) - longitudinal * carcass_y)

    # The fitted coefficients make the overturning terms vanish, so the
    # symbolic model omits the overturning moment. The planar model does not
    # use it, and the NumPy test pins that it is zero for this fit.
    rolling_coefficient = (
        parameters.f_rr_0
        + parameters.f_rr_1 * load_ratio
        + parameters.f_rr_2 * load_ratio**2
    )
    rolling = (
        -f_z
        * rolling_coefficient
        * radius
        * (
            1.0
            + parameters.h_rr
            * ca.tan(
                (ca.pi / 2.0)
                * ca.fabs(wheel_speed)
                / ca.fmax(parameters.omega_cr, parameters.omega_eps)
            )
        )
    )

    return SymbolicTire(
        longitudinal_force=longitudinal,
        lateral_force=lateral,
        aligning_moment=aligning,
        rolling_moment=rolling,
        loaded_radius=radius,
        slip_ratio=slip_ratio,
        slip_angle=slip_angle,
        phi=phi,
        f_bar=f_bar,
        longitudinal_friction=mu_x,
    )
