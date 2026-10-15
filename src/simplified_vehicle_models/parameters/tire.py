from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class TireEnvelopeParameters:
    """Store the parameters for a tire force envelope."""

    longitudinal_mu_scale: float
    lateral_mu_scale: float
    load_sensitivity_slope: float
    load_sensitivity_intercept: float
    effective_radius: float


@dataclass(frozen=True, slots=True)
class UniTirePeakEnvelope:
    """Tabulated pure-slip grip peaks reduced from simplified UniTire.

    The force capacities preserve load dependence and are evaluated by linear
    interpolation between the sampled normal loads. ``effective_radius`` is
    the loaded radius at the tire's rated load and is used by point-mass motor
    speed limits.
    """

    vertical_loads: FloatArray
    longitudinal_peak_force: FloatArray
    lateral_peak_force: FloatArray
    effective_radius: float
    reference_speed: float

    def force_capacities(
        self, loads: FloatArray
    ) -> tuple[FloatArray, FloatArray]:
        """Return pure longitudinal and lateral peak capacities at loads."""

        load_values = np.maximum(np.asarray(loads, dtype=np.float64), 0.0)
        longitudinal = np.interp(
            load_values,
            np.asarray(self.vertical_loads, dtype=np.float64),
            np.asarray(self.longitudinal_peak_force, dtype=np.float64),
            left=0.0,
        )
        lateral = np.interp(
            load_values,
            np.asarray(self.vertical_loads, dtype=np.float64),
            np.asarray(self.lateral_peak_force, dtype=np.float64),
            left=0.0,
        )
        return longitudinal, lateral


@dataclass(frozen=True, slots=True)
class UniTireSimpleParameters:
    """Store one fitted simplified steady-state UniTire parameter set.

    The model assumes zero inclination angle.
    Every polynomial coefficient tuple uses descending power order.
    Element zero multiplies the highest power of the normalized load.
    The name of each field matches the MATLAB source field.
    """

    f_z_rated: float
    k_cx: float
    k_cy: float
    k_x: tuple[float, ...]
    k_y: tuple[float, ...]
    e_1: float
    d_x_0: tuple[float, ...]
    d_e: tuple[float, ...]
    d_1: tuple[float, ...]
    d_2: tuple[float, ...]
    mu_0_x: float
    mu_s_x: float
    h_x: float
    v_m_x: float
    mu_0_y: float
    mu_s_y: float
    h_y: float
    v_m_y: float
    k_11: float
    k_12: float
    k_13: float
    k_21: float
    k_22: float
    k_23: float
    m_x_r_1: float
    m_x_r_2: float
    m_x_r_3: float
    r_1: float
    r_2: float
    r_3: float
    f_y_shift_0: float
    f_y_shift_fz: float
    k_rl_0: float
    k_rl_1: float
    k_rl_2: float
    f_rr_0: float
    f_rr_1: float
    f_rr_2: float
    h_rr: float
    omega_cr: float
    omega_eps: float
    vroll_min: float
    force_eps: float
    stiff_eps: float
    slip_eps: float
    effective_radius: float
