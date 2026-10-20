"""Provisional transient layer around the unchanged steady UniTire function.

Reference: Ma et al., "The Unsteady-State Response of Tires to Slip Angle and
Vertical Load Variations", Machines 2022, 10(7), 527,
DOI 10.3390/machines10070527. The project plan cites equations 52-55, 60-61,
64 and 69-70. The paper text was not available while this module was written.
Neither the equation numbers nor the statements about the paper's content
(secant stiffness, trail pole and zero, tire-specific m and n) are verified
against the paper; they come from the project plan.

This module is a reduced specialization, not a fitted or validated transient
tire. It keeps three first-order states per wheel:

``lateral_deformation`` (m)
    The carcass lateral deformation. With
    ``sigma_y(Fz) = lateral_length_scale * k_y(Fz) / k_cy`` the effective slip
    angle is ``alpha_effective = lateral_deformation / sigma_y(Fz)``, and
    ``d(lateral_deformation)/dt = max(|Vx|, 0.1) * (alpha_command - alpha_effective)``.
    This is a reduced specialization motivated by the load-coupled relaxation
    of equation (61), with the steady ``k_y`` polynomial used for the length
    instead of a secant stiffness, so no ``dFz/dt`` or nested root is
    evaluated. It is not a verbatim implementation of the paper.
``trail_angle`` (rad)
    A delayed slip angle for the pneumatic trail:
    ``d(trail_angle)/dt = max(|Vx|, 0.1) * (alpha_effective - trail_angle) / sigma_trail``
    with ``sigma_trail = sigma_y / trail_pole_ratio``, and ``|Vx|`` floored at
    0.1 m/s in all three rates. The effective trail is
    ``t_eff = t(trail_angle) + trail_lead_ratio * (t(alpha_effective) - t(trail_angle))``,
    a bounded lead term intended to mimic the zero of equation (64) and the
    trail pole of equations (69)-(70). It is a project approximation, not the
    paper's expression. The lead correction is zero at steady state.
``longitudinal_slip`` (1)
    PROPOSED extension, not from the paper:
    ``d(longitudinal_slip)/dt = max(|Vx|, 0.1) * (kappa_command - longitudinal_slip) / sigma_x``
    with ``sigma_x = longitudinal_length_scale * k_x(Fz) / k_cx``.

The vertical load is never filtered. The forces come from the unchanged
``evaluate_unitire`` at the delayed slips. The sign conventions of the steady
model are reused, including the aligning moment formula.

Contact. One smoothstep window ``w(Fz)`` is 0 at ``Fz <= 0.5 N`` and 1 at
``Fz >= 2 N``. It multiplies Fx, Fy, Mz and the overturning moment (Mz uses the windowed
forces), and it blends the state rates: ``rate = w * slip_law_rate + (1 - w) * (-state / 0.01 s)``.
Forces, state rates and their first derivatives therefore have no jump at any
load. Below 0.5 N the forces are zero and the states relax toward zero with a
0.01 s time constant. The rolling-resistance moment is not windowed (it is
proportional to Fz). The window never changes the vertical force.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.compiler.capabilities import (
    TRANSIENT_COMBINED_SLIP,
)
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters
from simplified_vehicle_models.tires.unitire_simple import (
    TireForces,
    evaluate_unitire,
)
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_no_references,
    require_positive,
)


TIRE_MODEL_TYPE = "tire-model"
UNITIRE_TRANSIENT_SUBTYPE = "unitire-transient-simple"

#: Time constant of the state relaxation without contact (s).
CONTACT_LOSS_TIME_CONSTANT = 0.01
#: Contact window: zero at and below, full at and above (N). It scales the
#: tangential forces and blends the state rates.
WINDOW_ZERO_LOAD = 0.5
WINDOW_FULL_LOAD = 2.0
#: Speed floor for the relaxation rates (m/s). Below about 0.15 m/s a launch from
#: rest stalls the driven wheel: the slip state cannot follow a wheel that spins
#: up in milliseconds. 0.5 leaves margin and changes nothing above 0.5 m/s.
RATE_SPEED_FLOOR = 0.5

_EPSILON = float(np.finfo(np.float64).eps)
_MINIMUM_LENGTH = 1.0e-6

_UNITS = {
    "longitudinal_length_scale": "1",
    "lateral_length_scale": "1",
    "trail_pole_ratio": "1",
    "trail_lead_ratio": "1",
}


@dataclass(frozen=True, slots=True)
class TransientTireParameters:
    """Store the PROPOSED scalar coefficients of the transient layer."""

    longitudinal_length_scale: float
    lateral_length_scale: float
    trail_pole_ratio: float
    trail_lead_ratio: float


@dataclass(frozen=True, slots=True)
class TransientTireState:
    """Store the three internal values of one wheel, or their rates."""

    longitudinal_slip: float
    lateral_deformation: float
    trail_angle: float


def _polyval(coefficients: tuple[float, ...], value: float) -> float:
    """Evaluate a descending-order polynomial (same order as the steady tire)."""

    result = 0.0
    for coefficient in coefficients:
        result = result * value + coefficient
    return result


def relaxation_lengths(
    steady: UniTireSimpleParameters,
    transient: TransientTireParameters,
    vertical_load: float,
) -> tuple[float, float]:
    """Return ``(sigma_x, sigma_y)`` in metres at one vertical load."""

    load_ratio = max(float(vertical_load), 0.0) / max(steady.f_z_rated, _EPSILON)
    k_x = _polyval(steady.k_x, load_ratio)
    k_y = _polyval(steady.k_y, load_ratio)
    sigma_x = (
        transient.longitudinal_length_scale
        * k_x
        / max(steady.k_cx, steady.stiff_eps)
    )
    sigma_y = (
        transient.lateral_length_scale * k_y / max(steady.k_cy, steady.stiff_eps)
    )
    return sigma_x, sigma_y


def check_relaxation_lengths(
    steady: UniTireSimpleParameters,
    transient: TransientTireParameters,
    maximum_load: float,
    samples: int = 201,
) -> None:
    """Raise ValueError unless both lengths are positive from 0 to ``maximum_load``."""

    for load in np.linspace(0.0, float(maximum_load), samples):
        sigma_x, sigma_y = relaxation_lengths(steady, transient, float(load))
        if not (sigma_x > 0.0 and sigma_y > 0.0):
            raise ValueError(
                "relaxation lengths must be positive; "
                f"sigma_x={sigma_x}, sigma_y={sigma_y} at {load} N"
            )


def contact_force_window(vertical_load: float) -> float:
    """Return the smoothstep tangential force window of one vertical load."""

    span = WINDOW_FULL_LOAD - WINDOW_ZERO_LOAD
    s = min(max((float(vertical_load) - WINDOW_ZERO_LOAD) / span, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def steady_rolling_state(
    steady: UniTireSimpleParameters,
    transient: TransientTireParameters,
    slip_angle: float,
    slip_ratio: float,
    vertical_load: float,
) -> TransientTireState:
    """Return the state at steady rolling, or zero without contact."""

    if vertical_load <= WINDOW_ZERO_LOAD:
        return TransientTireState(0.0, 0.0, 0.0)
    _, sigma_y = relaxation_lengths(steady, transient, vertical_load)
    return TransientTireState(
        longitudinal_slip=float(slip_ratio),
        lateral_deformation=float(slip_angle) * sigma_y,
        trail_angle=float(slip_angle),
    )


def evaluate_transient_unitire(
    steady_parameters: UniTireSimpleParameters,
    transient_parameters: TransientTireParameters,
    state: TransientTireState,
    slip_angle: float,
    slip_ratio: float,
    vertical_load: float,
    wheel_speed: float,
    longitudinal_speed: float,
) -> tuple[TireForces, TransientTireState]:
    """Return the delayed tire forces and the rates of the three states.

    ``slip_angle`` and ``slip_ratio`` are the commands. ``wheel_speed`` is the
    hub angular speed (rad/s) and ``longitudinal_speed`` the contact patch
    speed (m/s); only their magnitudes are used.
    """

    steady = steady_parameters
    tire = transient_parameters
    fz = float(vertical_load)
    speed = abs(float(longitudinal_speed))
    omega = abs(float(wheel_speed))

    sigma_x, sigma_y = relaxation_lengths(steady, tire, fz)
    sigma_x = max(sigma_x, _MINIMUM_LENGTH)
    sigma_y = max(sigma_y, _MINIMUM_LENGTH)
    alpha_effective = state.lateral_deformation / sigma_y
    kappa = state.longitudinal_slip

    main = evaluate_unitire(steady, alpha_effective, kappa, fz, omega)
    trail = evaluate_unitire(steady, state.trail_angle, kappa, fz, omega)
    t_delayed = trail.pneumatic_trail
    t_effective = t_delayed + tire.trail_lead_ratio * (
        main.pneumatic_trail - t_delayed
    )

    # The steady aligning-moment formula with the effective trail, evaluated
    # with the windowed forces so that Mz vanishes smoothly with contact.
    window = contact_force_window(fz)
    fx = main.longitudinal_force * window
    fy = main.lateral_force * window
    offset_x = fx / max(steady.k_cx, steady.stiff_eps)
    offset_y = fy / max(steady.k_cy, steady.stiff_eps)
    aligning_moment = -(fy * (t_effective + offset_x) - fx * offset_y)

    forces = TireForces(
        longitudinal_force=fx + 0.0,
        lateral_force=fy + 0.0,
        vertical_force=main.vertical_force,
        overturning_moment=main.overturning_moment * window + 0.0,
        rolling_resistance_moment=main.rolling_resistance_moment,
        aligning_moment=aligning_moment + 0.0,
        effective_radius=main.effective_radius,
        loaded_radius=main.loaded_radius,
        mu_x=main.mu_x,
        mu_y=main.mu_y,
        pneumatic_trail=t_effective,
        relaxation_length=sigma_y,
        s_x=main.s_x,
        s_y=main.s_y,
        phi=main.phi,
        f_bar=main.f_bar,
        lambda_factor=main.lambda_factor,
    )

    relax = -1.0 / CONTACT_LOSS_TIME_CONSTANT
    sigma_trail = sigma_y / tire.trail_pole_ratio
    rate_speed = max(speed, RATE_SPEED_FLOOR)
    follow_longitudinal = (
        rate_speed
        * (float(slip_ratio) - state.longitudinal_slip)
        / sigma_x
    )
    follow_lateral = rate_speed * (float(slip_angle) - alpha_effective)
    follow_trail = rate_speed * (alpha_effective - state.trail_angle) / sigma_trail
    rates = TransientTireState(
        longitudinal_slip=window * follow_longitudinal
        + (1.0 - window) * relax * state.longitudinal_slip,
        lateral_deformation=window * follow_lateral
        + (1.0 - window) * relax * state.lateral_deformation,
        trail_angle=window * follow_trail
        + (1.0 - window) * relax * state.trail_angle,
    )
    return forces, rates


def compile_unitire_transient(model: Model) -> TransientTireParameters:
    """Compile the scalar coefficients of the transient layer."""

    require_model_type(model, TIRE_MODEL_TYPE, UNITIRE_TRANSIENT_SUBTYPE)
    require_no_references(model)
    values = closed_parameters(model, _UNITS)
    require_positive(
        model,
        values,
        "longitudinal_length_scale",
        "lateral_length_scale",
        "trail_pole_ratio",
    )
    if not 0.0 <= values["trail_lead_ratio"] <= 1.0:
        raise ValueError(
            f"{model.path}: trail_lead_ratio must be from zero through one"
        )
    return TransientTireParameters(**values)


register_component(
    TIRE_MODEL_TYPE,
    UNITIRE_TRANSIENT_SUBTYPE,
    capability=TRANSIENT_COMBINED_SLIP,
    compile_function=compile_unitire_transient,
)
