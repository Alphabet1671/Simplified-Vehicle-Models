"""Evaluate the fitted simplified steady-state UniTire model.

The model is a direct port of ``unitire_simple_solve.m`` from the
FSAE-VD-Personal-Scripts repository. The source assumes zero inclination
angle. It uses SAE tire axes: ``+x`` forward, ``+y`` right, ``+z`` down.

The vehicle frame is also SAE. The tire lateral and vertical senses therefore
match the vehicle senses. A positive slip angle gives a positive lateral force,
and that force acts to the right in the vehicle frame. No lateral sign
conversion is necessary. The tests pin this agreement.

The source reports the aligning moment in SAE tire axes. A positive slip angle
therefore gives a negative aligning moment.

This module imports no solver state and holds no memory.
Every polynomial coefficient tuple uses descending power order.
"""

from __future__ import annotations

import numpy as np

from simplified_vehicle_models.compiler.capabilities import STEADY_COMBINED_SLIP
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_nonnegative,
    require_positive,
)


TIRE_MODEL_TYPE = "tire-model"
UNITIRE_SIMPLE_SUBTYPE = "unitire-simple"

#: The source uses the MATLAB machine epsilon as a plain divisor floor.
_EPSILON = float(np.finfo(np.float64).eps)

#: The source divides the half circle by this value for the rolling moment.
_HALF_PI = float(np.pi) / 2.0

_UNITS = {
    "f_z_rated": "N",
    "k_cx": "N/m",
    "k_cy": "N/m",
    "k_x_2": "N",
    "k_x_1": "N",
    "k_x_0": "N",
    "k_y_2": "N",
    "k_y_1": "N",
    "k_y_0": "N",
    "e_1": "1",
    "d_x_0_1": "m",
    "d_x_0_0": "m",
    "d_e_1": "m",
    "d_e_0": "m",
    "d_1_1": "1",
    "d_1_0": "1",
    "d_2_1": "1",
    "d_2_0": "1",
    "mu_0_x": "1",
    "mu_s_x": "1",
    "h_x": "1",
    "v_m_x": "m/s",
    "mu_0_y": "1",
    "mu_s_y": "1",
    "h_y": "1",
    "v_m_y": "m/s",
    "k_11": "N*m",
    "k_12": "N*m",
    "k_13": "N*m",
    "k_21": "N*m",
    "k_22": "N*m",
    "k_23": "N*m",
    "m_x_r_1": "N*m",
    "m_x_r_2": "N*m",
    "m_x_r_3": "N*m",
    "r_1": "m",
    "r_2": "m",
    "r_3": "m",
    "f_y_shift_0": "N",
    "f_y_shift_fz": "N",
    "k_rl_0": "m/N^2",
    "k_rl_1": "m/N^2",
    "k_rl_2": "m/N^2",
    "f_rr_0": "1",
    "f_rr_1": "1",
    "f_rr_2": "1",
    "h_rr": "1",
    "omega_cr": "rad/s",
    "omega_eps": "rad/s",
    "vroll_min": "m/s",
    "force_eps": "N",
    "stiff_eps": "1",
    "slip_eps": "1",
}

#: Vector parameters, in descending coefficient order.
_VECTOR_PARAMETERS = {
    "k_x": ("k_x_2", "k_x_1", "k_x_0"),
    "k_y": ("k_y_2", "k_y_1", "k_y_0"),
    "d_x_0": ("d_x_0_1", "d_x_0_0"),
    "d_e": ("d_e_1", "d_e_0"),
    "d_1": ("d_1_1", "d_1_0"),
    "d_2": ("d_2_1", "d_2_0"),
}


class TireForces:
    """Store the tire forces, moments, and radii of one evaluation.

    Every field is a NumPy array with the broadcast input shape.
    A scalar input therefore gives a zero dimensional array.
    """

    __slots__ = (
        "longitudinal_force",
        "lateral_force",
        "vertical_force",
        "overturning_moment",
        "rolling_resistance_moment",
        "aligning_moment",
        "effective_radius",
        "loaded_radius",
        "mu_x",
        "mu_y",
        "pneumatic_trail",
        "relaxation_length",
        "s_x",
        "s_y",
        "phi",
        "f_bar",
        "lambda_factor",
    )

    def __init__(
        self,
        *,
        longitudinal_force: FloatArray,
        lateral_force: FloatArray,
        vertical_force: FloatArray,
        overturning_moment: FloatArray,
        rolling_resistance_moment: FloatArray,
        aligning_moment: FloatArray,
        effective_radius: FloatArray,
        loaded_radius: FloatArray,
        mu_x: FloatArray,
        mu_y: FloatArray,
        pneumatic_trail: FloatArray,
        relaxation_length: FloatArray,
        s_x: FloatArray,
        s_y: FloatArray,
        phi: FloatArray,
        f_bar: FloatArray,
        lambda_factor: FloatArray,
    ) -> None:
        self.longitudinal_force = longitudinal_force
        self.lateral_force = lateral_force
        self.vertical_force = vertical_force
        self.overturning_moment = overturning_moment
        self.rolling_resistance_moment = rolling_resistance_moment
        self.aligning_moment = aligning_moment
        self.effective_radius = effective_radius
        self.loaded_radius = loaded_radius
        self.mu_x = mu_x
        self.mu_y = mu_y
        self.pneumatic_trail = pneumatic_trail
        self.relaxation_length = relaxation_length
        # The source reports these intermediates in out.debug.
        self.s_x = s_x
        self.s_y = s_y
        self.phi = phi
        self.f_bar = f_bar
        self.lambda_factor = lambda_factor

    def __repr__(self) -> str:
        return (
            "TireForces("
            f"longitudinal_force={self.longitudinal_force!r}, "
            f"lateral_force={self.lateral_force!r}, "
            f"aligning_moment={self.aligning_moment!r})"
        )


def _polyval(coefficients: tuple[float, ...], value: FloatArray) -> FloatArray:
    """Evaluate one polynomial in descending coefficient order.

    The source uses the MATLAB ``polyval`` order.
    Element zero multiplies the highest power.
    """

    result = np.zeros_like(value, dtype=np.float64)
    for coefficient in coefficients:
        result = result * value + coefficient
    return result


def _friction_law(
    slip_speed: FloatArray,
    mu_0: float,
    mu_s: float,
    shape: float,
    transition_speed: float,
) -> FloatArray:
    """Evaluate the source friction law for one slip direction.

    The law moves from the peak value ``mu_0`` at zero speed to the sliding
    value ``mu_s`` at high speed.
    """

    argument = np.abs(slip_speed) / max(transition_speed, _EPSILON)
    soft = np.log(argument + np.exp(-argument))
    return mu_s + (mu_0 - mu_s) * np.exp(-(shape**2) * soft**2)


def loaded_radius(
    parameters: UniTireSimpleParameters,
    vertical_load: FloatArray,
) -> FloatArray:
    """Return the loaded radius of one tire at a vertical load.

    The radius depends on the normalized load only.
    It does not depend on the slip, so a caller can use it before it evaluates
    the tire forces.
    """

    load_ratio = vertical_load / max(parameters.f_z_rated, _EPSILON)
    radius_from_load = (
        parameters.r_1
        + parameters.r_2 * load_ratio
        + parameters.r_3 * load_ratio**2
    )
    lateral_shift = (
        parameters.f_y_shift_0 + parameters.f_y_shift_fz * load_ratio
    )
    radius_from_lateral = (
        parameters.k_rl_0
        + parameters.k_rl_1 * load_ratio
        + parameters.k_rl_2 * load_ratio**2
    ) * (0.0 - lateral_shift) ** 2
    return np.maximum(radius_from_load + radius_from_lateral, 1.0e-4)


def evaluate_unitire(
    parameters: UniTireSimpleParameters,
    slip_angle: FloatArray,
    slip_ratio: FloatArray,
    vertical_load: FloatArray,
    wheel_speed: FloatArray,
) -> TireForces:
    """Evaluate the simplified steady-state UniTire model.

    ``slip_angle`` uses radians and the SAE sense.
    A positive angle steers the wheel to the right.
    ``slip_ratio`` is dimensionless and is zero at free rolling.
    ``vertical_load`` is the compressive load in newtons.
    ``wheel_speed`` is the hub angular speed in radians per second.
    Pass the magnitude of the hub angular speed.

    The inputs broadcast against each other.
    The outputs use the broadcast shape.
    """

    alpha, kappa, f_z, omega = np.broadcast_arrays(
        np.asarray(slip_angle, dtype=np.float64),
        np.asarray(slip_ratio, dtype=np.float64),
        np.asarray(vertical_load, dtype=np.float64),
        np.asarray(wheel_speed, dtype=np.float64),
    )

    # A tire cannot pull on the road. Clamp the load to a compressive value, so
    # a lifted wheel carries no force. The clamp also keeps the load-dependent
    # coefficients inside their physical domain. Without it, a large negative
    # load turns the pneumatic-trail decay into a growth and overflows.
    f_z = np.maximum(f_z, 0.0)

    # Normalized load and the load dependent radius.
    f_zn = f_z / max(parameters.f_z_rated, _EPSILON)
    radius = loaded_radius(parameters, f_z)
    effective_radius = radius

    # Slip stiffness at the current load.
    k_x = _polyval(parameters.k_x, f_zn)
    k_y = _polyval(parameters.k_y, f_zn)
    k_cx = parameters.k_cx
    k_cy = parameters.k_cy

    s_x = kappa
    s_y = np.tan(alpha)

    # The source recovers the rolling speed from the hub speed in magnitude.
    v_roll = np.maximum(np.abs(omega) * effective_radius, parameters.vroll_min)
    v_sx = np.abs(s_x) * v_roll
    v_sy = np.abs(s_y) * v_roll

    mu_x = _friction_law(
        v_sx,
        parameters.mu_0_x,
        parameters.mu_s_x,
        parameters.h_x,
        parameters.v_m_x,
    )
    mu_y = _friction_law(
        v_sy,
        parameters.mu_0_y,
        parameters.mu_s_y,
        parameters.h_y,
        parameters.v_m_y,
    )

    # The normal load gives each pure slip direction its force scale.
    force_scale_x = mu_x * f_z
    force_scale_y = mu_y * f_z

    phi_x = k_x * s_x / np.maximum(force_scale_x, parameters.force_eps)
    phi_y = k_y * s_y / np.maximum(force_scale_y, parameters.force_eps)
    phi = np.hypot(phi_x, phi_y)

    f_bar = 1.0 - np.exp(
        -phi
        - parameters.e_1 * phi**2
        - (parameters.e_1**2 + 1.0 / 12.0) * phi**3
    )
    lam = 1.0 + (k_y / np.maximum(k_x, parameters.stiff_eps) - 1.0) * f_bar
    denominator = np.hypot(lam * phi_x, phi_y)

    # The source uses the linear stiffness branch below a small slip.
    linear = denominator < parameters.slip_eps
    safe_denominator = np.where(linear, 1.0, denominator)
    longitudinal_force = np.where(
        linear,
        k_x * s_x,
        f_bar * (lam * phi_x / safe_denominator) * mu_x * f_z,
    )
    lateral_force = np.where(
        linear,
        k_y * s_y,
        f_bar * (phi_y / safe_denominator) * mu_y * f_z,
    )

    # Pneumatic trail and the aligning moment.
    d_x_0 = _polyval(parameters.d_x_0, f_zn)
    d_e = _polyval(parameters.d_e, f_zn)
    d_1 = _polyval(parameters.d_1, f_zn)
    d_2 = _polyval(parameters.d_2, f_zn)
    pneumatic_trail = (d_x_0 + d_e) * np.exp(
        -d_1 * phi - d_2 * phi**2
    ) - d_e

    carcass_offset_x = longitudinal_force / max(k_cx, parameters.stiff_eps)
    carcass_offset_y = lateral_force / max(k_cy, parameters.stiff_eps)
    aligning_moment = -(
        lateral_force * (pneumatic_trail + carcass_offset_x)
        - longitudinal_force * carcass_offset_y
    )

    # Overturning moment. The source cubic term is zero in the fitted set.
    gamma_e = np.arctan2(carcass_offset_y, radius)
    k_1 = (
        parameters.k_11 + parameters.k_12 * f_zn + parameters.k_13 * f_zn**2
    )
    k_2 = (
        parameters.k_21 + parameters.k_22 * f_zn + parameters.k_23 * f_zn**2
    )
    overturning_from_lateral = f_z * carcass_offset_y
    overturning_from_camber = -k_1 * gamma_e - (k_2 * gamma_e) ** 3
    overturning_from_load = (
        parameters.m_x_r_1
        + parameters.m_x_r_2 * f_zn
        + parameters.m_x_r_3 * f_zn**2
    )
    overturning_moment = (
        overturning_from_lateral
        + overturning_from_camber
        + overturning_from_load
    )

    # Rolling resistance moment.
    rolling_coefficient = (
        parameters.f_rr_0
        + parameters.f_rr_1 * f_zn
        + parameters.f_rr_2 * f_zn**2
    )
    rolling_moment = (
        -f_z
        * rolling_coefficient
        * radius
        * (
            1.0
            + parameters.h_rr
            * np.tan(
                _HALF_PI
                * np.abs(omega)
                / max(parameters.omega_cr, parameters.omega_eps)
            )
        )
    )

    relaxation_length = k_y / max(k_cy, parameters.stiff_eps)

    return TireForces(
        longitudinal_force=longitudinal_force,
        lateral_force=lateral_force,
        vertical_force=f_z,
        overturning_moment=overturning_moment,
        rolling_resistance_moment=rolling_moment,
        aligning_moment=aligning_moment,
        effective_radius=effective_radius,
        loaded_radius=radius,
        mu_x=mu_x,
        mu_y=mu_y,
        pneumatic_trail=pneumatic_trail,
        relaxation_length=relaxation_length,
        s_x=s_x,
        s_y=s_y,
        phi=phi,
        f_bar=f_bar,
        lambda_factor=lam,
    )


def compile_unitire_simple(model: Model) -> UniTireSimpleParameters:
    """Compile one fitted simplified steady-state UniTire model."""

    if model.subtype == "unitire-transient-simple":
        # Lazy import avoids a module cycle while allowing the complete XML to
        # serve the unchanged steady capability as well as the transient one.
        from simplified_vehicle_models.tires.unitire_transient import (
            compile_unitire_transient,
        )

        return compile_unitire_transient(model).steady
    require_model_type(model, TIRE_MODEL_TYPE, UNITIRE_SIMPLE_SUBTYPE)
    values = closed_parameters(model, _UNITS)
    require_positive(
        model,
        values,
        "f_z_rated",
        "k_cx",
        "k_cy",
        "v_m_x",
        "v_m_y",
        "omega_cr",
        "omega_eps",
        "vroll_min",
        "force_eps",
        "stiff_eps",
    )
    require_nonnegative(model, values, "slip_eps")

    vectors = {
        name: tuple(values[element] for element in elements)
        for name, elements in _VECTOR_PARAMETERS.items()
    }
    scalars = {
        name: value
        for name, value in values.items()
        if name not in {
            element
            for elements in _VECTOR_PARAMETERS.values()
            for element in elements
        }
    }
    # Compute effective radius from loaded radius at rated load
    # For point-mass compatibility, use loaded_radius at f_z_rated
    from simplified_vehicle_models.tires.unitire_simple import loaded_radius
    rated_load = scalars.get("f_z_rated", 1000.0)
    # Create a temporary parameters object to compute loaded radius
    temp_params = UniTireSimpleParameters(**scalars, **vectors, effective_radius=0.0)
    effective_radius = float(loaded_radius(temp_params, rated_load))
    scalars["effective_radius"] = effective_radius
    return UniTireSimpleParameters(**scalars, **vectors)


register_component(
    TIRE_MODEL_TYPE,
    UNITIRE_SIMPLE_SUBTYPE,
    capability=STEADY_COMBINED_SLIP,
    compile_function=compile_unitire_simple,
)
