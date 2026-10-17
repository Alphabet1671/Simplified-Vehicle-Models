"""Between-step evaluation-domain certificate for the vertical 14-DoF plant.

``VerticalForwardPlant.evaluate`` has no inner implicit load loop: given a state
and a driver command it is an explicit composition of table lookups, one
constant-mass-block solve, the tire vertical law and the shared powertrain
closure.  An evaluation can therefore only go wrong between accepted nodes if
one of the following *obligations* fails at some state of the dense
interpolant, so this module proves each of them from the per-interval state
enclosures ``bounds`` (shape ``(intervals, 38 or 39, 2)``) by interval
arithmetic:

1. every jounce stays inside the spring / motion-ratio / jounce-steer spline
   domain (the physical jounce limits);
2. every damper velocity ``jounce_rate / motion_ratio`` stays inside the damper
   table axes (``out_of_range`` can never be raised and no linear extension is
   used);
3. the tire deflection stays where the closed-form loaded-radius inverse is
   defined, and the tire normal load (elastic part times ``1 + psi`` plus the
   damping term) stays below ``SUPPORTED_MAXIMUM_LOAD`` where the inverse is
   proved valid;
4. the 7x7 vertical mass block is constant (zero inertance, constant motion
   ratio) and positive definite, so the acceleration solve never fails;
5. the body speed stays above the plant's guard, wheel speeds stay
   nonnegative, SOC stays in ``[0, 1]`` and steering stays inside its bound
   (the plant's explicit guards), and the path denominator / progress stay
   positive (delegated to ``certify_vertical_dense_domain``);
6. every state enclosure is finite (so no transient-tire state leaves the real
   line).

Assumptions that are *not* proved here, and are stated in the certificate:

* the enclosure bounds the integrator's dense interpolant, not the true ODE
  trajectory (the refinement tube is the separate empirical support for that);
* the monotone cubic spline stays inside the hull of its knot values on each
  segment (Fritsch-Carlson), used only for the motion-ratio table;
* the shared powertrain closure's bracketing / monotonicity argument, covered by
  ``tests/test_powertrain_closure_audit.py`` (an exhaustive grid audit);
* directed rounding: the enclosure padding covers rounding, as in the planar
  certificate.

Any obligation that cannot be evaluated fails closed.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from simplified_vehicle_models.formulations.vertical_14dof import (
    VerticalState, sprung_wheel_positions, vertical_mass_block,
)
from simplified_vehicle_models.tires.vertical import (
    SUPPORTED_MAXIMUM_LOAD, DAMPING_LOAD_FLOOR, tire_deflection_load,
)

from .vertical_dense_certificate import certify_vertical_dense_domain

__all__ = ["certify_vertical_between_step", "SCOPE"]

#: Global index of a VerticalState field: path coordinates (s, n, psi) come first.
_OFFSET = 3
_NAMES = [field.name for field in dataclasses.fields(VerticalState)]


def _index(name: str) -> int:
    return _OFFSET + _NAMES.index(name)


_HEAVE, _ROLL, _PITCH = _index("heave"), _index("roll"), _index("pitch")
_VZ, _ROLL_RATE, _PITCH_RATE = _index("v_z"), _index("roll_rate"), _index("pitch_rate")
_JOUNCE = [_index(f"jounce_{c}") for c in ("fl", "fr", "rl", "rr")]
_JOUNCE_RATE = [_index(f"jounce_rate_{c}") for c in ("fl", "fr", "rl", "rr")]
_VX = _index("v_x")
_WHEELS = [_index(f"wheel_speed_{c}") for c in ("fl", "fr", "rl", "rr")]
_SOC, _STEER = 24, 25

SCOPE = (
    "evaluation-domain obligations of the explicit vertical 14-DoF right-hand side "
    "over the BDF dense interpolant (spline/damper/tire/mass-block/powertrain "
    "guards); not the true ODE trajectory, not directed rounding, and the "
    "powertrain closure monotonicity rests on the shared grid audit"
)


def _scale(lo, hi, factor):
    """Interval of ``factor * [lo, hi]`` for a constant ``factor``."""
    a, b = factor * lo, factor * hi
    return np.minimum(a, b), np.maximum(a, b)


def _fail(reason, checks=None, **extra):
    result = {"certified": False, "reason": reason, "scope": SCOPE,
              "checks": dict(checks or {})}
    result.update(extra)
    return result


def _axle_hull(axle):
    """Return the motion-ratio hull and verify it is a positive constant-hull table."""
    values = np.asarray(axle.motion_ratio_damper.y, dtype=float)
    return float(values.min()), float(values.max())


def certify_vertical_between_step(vehicle, plan, bounds, summary=None, pedal_dynamics=True):
    """Prove the evaluation-domain obligations on every dense interval.

    Returns a dictionary with ``certified`` (true only if every obligation holds
    on every interval), per-check booleans in ``checks`` and the worst margins in
    ``margins``.  ``plan`` is the speed plan used for the path-denominator part.
    """
    bounds = np.asarray(bounds, dtype=float)
    expected = 39 if pedal_dynamics else 38
    if bounds.ndim != 3 or bounds.shape[2] != 2 or bounds.shape[1] != expected:
        return _fail("enclosure shape does not match the 14DoF plant")
    if bounds.shape[0] == 0 or not np.all(np.isfinite(bounds)):
        return _fail("enclosure is empty or non-finite")
    lo, hi = bounds[:, :, 0], bounds[:, :, 1]
    if np.any(lo > hi):
        return _fail("enclosure lower bound exceeds upper bound")

    dense = certify_vertical_dense_domain(vehicle, plan, bounds, summary,
                                          pedal_dynamics=pedal_dynamics)
    checks = {"state_domain_and_path": bool(dense.get("certified"))}
    margins = {}

    # -- 1/2. spline and damper domains ------------------------------------
    try:
        axles = (vehicle.suspension.front, vehicle.suspension.rear)
        sides = ((0, 1, axles[0]), (2, 3, axles[1]))
        jounce_margin = np.inf
        damper_margin = np.inf
        ratio_positive = True
        for dampers_index, (left, right, axle) in enumerate(sides):
            limit_lo, limit_hi = (float(x) for x in axle.jounce_limits)
            ratio_lo, ratio_hi = _axle_hull(axle)
            ratio_positive &= ratio_lo > 0.0
            table = vehicle.dampers[dampers_index].table
            axis_lo = float(table.velocity_axis_rebound[0])
            axis_hi = float(table.velocity_axis_compression[-1])
            for corner in (left, right):
                z_lo, z_hi = lo[:, _JOUNCE[corner]], hi[:, _JOUNCE[corner]]
                jounce_margin = min(jounce_margin, float(np.min(z_lo - limit_lo)),
                                    float(np.min(limit_hi - z_hi)))
                rate_lo, rate_hi = lo[:, _JOUNCE_RATE[corner]], hi[:, _JOUNCE_RATE[corner]]
                if ratio_lo > 0.0:
                    # v = rate / ratio with ratio in [ratio_lo, ratio_hi] > 0.
                    v_lo = np.minimum(rate_lo / ratio_lo, rate_lo / ratio_hi)
                    v_hi = np.maximum(rate_hi / ratio_lo, rate_hi / ratio_hi)
                    damper_margin = min(damper_margin, float(np.min(v_lo - axis_lo)),
                                        float(np.min(axis_hi - v_hi)))
        margins["jounce_spline_domain"] = float(jounce_margin)
        margins["damper_velocity_axis"] = float(damper_margin)
        checks["spring_mr_jounce_steer_domain"] = bool(jounce_margin >= 0.0)
        checks["damper_velocity_inside_table"] = bool(ratio_positive and damper_margin >= 0.0)
    except (AttributeError, IndexError, TypeError, ValueError) as error:
        return _fail(f"suspension or damper tables could not be read: {error}", checks)

    # -- 3. tire deflection / load ----------------------------------------
    try:
        pos = sprung_wheel_positions(vehicle)
        tire = vehicle.planar.tire
        damping = float(vehicle.tire_vertical_damping)
        r2, r3 = float(tire.r_2), float(tire.r_3)
        delta_cap = r2 * r2 / (4.0 * r3) if r3 > 0.0 else np.inf
        load_cap = SUPPORTED_MAXIMUM_LOAD
        worst_load = 0.0
        deflection_margin = np.inf
        wheel_lift_possible = False
        for corner in range(4):
            x_arm, y_arm = pos[corner]
            # z_body = heave + roll*y - pitch*x ;  hub_down = z_body - jounce
            roll_lo, roll_hi = _scale(lo[:, _ROLL], hi[:, _ROLL], y_arm)
            pitch_lo, pitch_hi = _scale(lo[:, _PITCH], hi[:, _PITCH], -x_arm)
            down_lo = lo[:, _HEAVE] + roll_lo + pitch_lo - hi[:, _JOUNCE[corner]]
            down_hi = hi[:, _HEAVE] + roll_hi + pitch_hi - lo[:, _JOUNCE[corner]]
            r_roll_lo, r_roll_hi = _scale(lo[:, _ROLL_RATE], hi[:, _ROLL_RATE], y_arm)
            r_pitch_lo, r_pitch_hi = _scale(lo[:, _PITCH_RATE], hi[:, _PITCH_RATE], -x_arm)
            rate_hi = (hi[:, _VZ] + r_roll_hi + r_pitch_hi - lo[:, _JOUNCE_RATE[corner]])
            deflection_margin = min(deflection_margin,
                                    float(np.min(0.99 * delta_cap - down_hi)))
            wheel_lift_possible |= bool(np.any(down_lo <= 0.0))
            # The elastic load is nondecreasing in deflection, and
            # load <= elastic + damping * max(rate, 0) (psi <= u, u <= damping*rate/(elastic+floor)).
            clipped = np.clip(down_hi, 0.0, 0.99 * delta_cap)
            elastic_hi = np.asarray(tire_deflection_load(
                tire, clipped, np.zeros_like(clipped), 0.0).load, dtype=float)
            upper = elastic_hi + damping * np.maximum(rate_hi, 0.0) * (
                elastic_hi / (elastic_hi + DAMPING_LOAD_FLOOR))
            worst_load = max(worst_load, float(np.max(upper)))
        margins["tire_deflection_inverse_domain"] = float(deflection_margin)
        margins["tire_load_upper_bound_n"] = float(worst_load)
        margins["tire_load_cap_n"] = float(load_cap)
        checks["tire_deflection_in_inverse_domain"] = bool(deflection_margin > 0.0)
        checks["tire_load_below_supported_maximum"] = bool(
            np.isfinite(worst_load) and worst_load <= load_cap)
        # Diagnostic only: loads are clamped at zero, so lift is not a violation.
        margins["wheel_lift_possible"] = bool(wheel_lift_possible)
    except (AttributeError, IndexError, TypeError, ValueError, FloatingPointError) as error:
        return _fail(f"tire vertical law could not be bounded: {error}", checks)

    # -- 4. constant, positive-definite mass block -------------------------
    try:
        inertia_free = all(float(a.inertance) == 0.0 for a in axles)
        constant_ratio = all(float(np.ptp(a.motion_ratio_damper.y)) == 0.0 for a in axles)
        block = vertical_mass_block(vehicle, np.zeros(4))
        eigenvalue = float(np.min(np.linalg.eigvalsh(0.5 * (block + block.T))))
        margins["mass_block_minimum_eigenvalue"] = eigenvalue
        checks["mass_block_constant"] = bool(inertia_free and constant_ratio)
        checks["mass_block_positive_definite"] = bool(np.isfinite(eigenvalue) and eigenvalue > 0.0)
    except (AttributeError, IndexError, TypeError, ValueError, np.linalg.LinAlgError) as error:
        return _fail(f"mass block could not be checked: {error}", checks)

    # -- 5. explicit plant guards -----------------------------------------
    margins["body_speed_over_guard"] = float(np.min(lo[:, _VX] - 0.25))
    margins["wheel_speed_floor"] = float(np.min(lo[:, _WHEELS]))
    margins["soc_floor"] = float(np.min(lo[:, _SOC]))
    margins["soc_ceiling"] = float(np.min(1.0 - hi[:, _SOC]))
    margins["steering_headroom"] = float(np.min(
        float(vehicle.planar.steering_maximum_angle) + 1.0e-7
        - np.maximum(np.abs(lo[:, _STEER]), np.abs(hi[:, _STEER]))))
    checks["plant_guards"] = bool(
        margins["body_speed_over_guard"] > 0.0 and margins["wheel_speed_floor"] >= 0.0
        and margins["soc_floor"] >= 0.0 and margins["soc_ceiling"] >= 0.0
        and margins["steering_headroom"] >= 0.0)
    checks["input_enclosure_finite"] = True  # required on entry; not a claim about tire forces

    certified = all(checks.values())
    return {
        "certified": bool(certified),
        "checks": checks,
        "margins": margins,
        "intervals": int(bounds.shape[0]),
        "state_domain": {key: dense.get(key) for key in ("certified", "failed", "margins")},
        "scope": SCOPE,
        "assumptions": [
            "enclosure bounds the BDF dense interpolant, not the true ODE trajectory",
            "motion-ratio spline stays in the hull of its knot values (Fritsch-Carlson)",
            "powertrain closure monotonicity: tests.test_powertrain_closure_audit",
            "slip, transient-tire and aero evaluations are total on finite inputs (every "
            "division is floored by a positive constant); argued by reading the code, "
            "NOT enclosed by interval arithmetic, and the tire forces are not bounded here",
            "rounding covered by enclosure padding, not directed rounding",
        ],
        "model": "14dof-simple",
    }
