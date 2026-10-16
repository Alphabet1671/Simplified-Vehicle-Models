"""Certify the planar plant's explicit state domain over BDF dense intervals.

Input is the per-interval component enclosure from
``dense_domain.dense_component_bounds``. Every check here is an inequality in
the state components themselves, or a monotone/interval-arithmetic composition
of them, so the enclosure decides it for the whole interval, not only at
sampled points.

What this does NOT certify (see the validation note):

* the true ODE trajectory, only the integrator's reported interpolant;
* convergence of the implicit load fixed point at interior states;
* the powertrain closure at interior states.

Floating-point rounding is covered by the enclosure padding, not by directed
rounding.
"""

from __future__ import annotations

import numpy as np

#: State layout of ``PlanarForwardPlant``.
_S, _N, _PSI, _VX, _VY, _YAW = 0, 1, 2, 3, 4, 5
_OMEGA = slice(6, 10)
_SOC, _STEER, _PEDAL = 10, 11, 12

MIN_SPEED = 0.25
MIN_DENOMINATOR = 0.5
MIN_PROGRESS = 0.25
STEERING_TOLERANCE = 1.e-7


def _interval_product(a_lo, a_hi, b_lo, b_hi):
    corners = np.stack([a_lo * b_lo, a_lo * b_hi, a_hi * b_lo, a_hi * b_hi])
    return corners.min(axis=0), corners.max(axis=0)


def _cos_range(lo, hi):
    """Return the exact range of cos over [lo, hi] (radians, any width)."""
    width_ok = (hi - lo) < 2. * np.pi
    end_lo, end_hi = np.cos(lo), np.cos(hi)
    low = np.minimum(end_lo, end_hi)
    high = np.maximum(end_lo, end_hi)
    # Interior maxima of cos sit at 2*pi*k, minima at pi + 2*pi*k.
    has_max = np.floor(hi / (2. * np.pi)) >= np.ceil(lo / (2. * np.pi))
    has_min = np.floor((hi - np.pi) / (2. * np.pi)) >= np.ceil((lo - np.pi) / (2. * np.pi))
    high = np.where(has_max | ~width_ok, 1., high)
    low = np.where(has_min | ~width_ok, -1., low)
    return low, high


def _sin_range(lo, hi):
    low, high = _cos_range(lo - np.pi / 2., hi - np.pi / 2.)
    return low, high


def curvature_enclosure(distance, curvature, lower, upper):
    """Return min/max of the periodic piecewise-linear curvature on [lower, upper].

    ``distance[-1]`` is the lap length, matching ``SpeedPlan.sample``'s
    ``np.interp`` of ``position % lap``. Piecewise-linear extrema sit at the
    interval ends or at table nodes, so the enclosure is exact.
    """
    distance = np.asarray(distance, dtype=float)
    curvature = np.asarray(curvature, dtype=float)
    lap = float(distance[-1])
    count = np.asarray(lower).size
    lows = np.empty(count)
    highs = np.empty(count)
    # Three laps of nodes cover any interval shorter than one lap.
    nodes = np.concatenate([distance[:-1] + k * lap for k in range(3)] + [[3. * lap]])
    values = np.concatenate([curvature[:-1]] * 3 + [curvature[-1:]])
    for index in range(count):
        a, b = float(lower[index]), float(upper[index])
        if b - a >= lap:
            lows[index], highs[index] = curvature.min(), curvature.max()
            continue
        a_wrapped = a % lap + lap
        b_wrapped = a_wrapped + (b - a)
        inside = (nodes > a_wrapped) & (nodes < b_wrapped)
        ends = np.interp([a_wrapped, b_wrapped], nodes, values)
        sample = np.concatenate([ends, values[inside]])
        lows[index], highs[index] = sample.min(), sample.max()
    return lows, highs


def certify_planar_dense_domain(vehicle, plan, bounds, pedal_dynamics):
    """Check every plant domain inequality on every dense interval.

    ``bounds`` has shape ``(intervals, states, 2)`` holding ``[min, max]``. The
    result is a dictionary. ``certified`` is true only if every inequality
    holds on every interval with a finite enclosure. Margins are reported as
    the worst (smallest) distance to each limit.
    """
    bounds = np.asarray(bounds, dtype=float)
    expected = 13 if pedal_dynamics else 12
    if bounds.ndim != 3 or bounds.shape[2] != 2 or bounds.shape[1] != expected:
        return {"certified": False, "reason": "enclosure shape does not match the plant"}
    if bounds.shape[0] == 0 or not np.all(np.isfinite(bounds)):
        return {"certified": False, "reason": "enclosure is empty or non-finite"}
    lo, hi = bounds[:, :, 0], bounds[:, :, 1]

    margins = {}
    margins["soc_floor"] = float(np.min(lo[:, _SOC]))
    margins["soc_ceiling"] = float(np.min(1. - hi[:, _SOC]))
    margins["speed_over_minimum"] = float(np.min(lo[:, _VX] - MIN_SPEED))
    margins["wheel_speed_floor"] = float(np.min(lo[:, _OMEGA]))
    margins["steering_headroom"] = float(np.min(
        vehicle.steering_maximum_angle + STEERING_TOLERANCE
        - np.maximum(np.abs(lo[:, _STEER]), np.abs(hi[:, _STEER]))))
    omega_cr = float(getattr(vehicle.tire, "omega_cr", np.inf))
    margins["wheel_speed_below_rolling_pole"] = float(np.min(omega_cr - hi[:, _OMEGA]))

    kappa_lo, kappa_hi = curvature_enclosure(plan.distance, plan.curvature,
                                             lo[:, _S], hi[:, _S])
    product_lo, product_hi = _interval_product(kappa_lo, kappa_hi, lo[:, _N], hi[:, _N])
    den_lo, den_hi = 1. - product_hi, 1. - product_lo
    margins["path_denominator_over_minimum"] = float(np.min(den_lo - MIN_DENOMINATOR))

    cos_lo, cos_hi = _cos_range(lo[:, _PSI], hi[:, _PSI])
    sin_lo, sin_hi = _sin_range(lo[:, _PSI], hi[:, _PSI])
    a_lo, a_hi = _interval_product(lo[:, _VX], hi[:, _VX], cos_lo, cos_hi)
    b_lo, b_hi = _interval_product(lo[:, _VY], hi[:, _VY], sin_lo, sin_hi)
    numerator_lo = a_lo - b_hi
    # Dividing by a positive denominator range: the worst case divides a
    # positive numerator by the largest denominator, a negative one by the
    # smallest.
    worst = np.where(numerator_lo >= 0., numerator_lo / den_hi, numerator_lo / den_lo)
    margins["path_progress_over_minimum"] = float(np.min(worst - MIN_PROGRESS))

    if pedal_dynamics:
        margins["pedal_overshoot_beyond_travel"] = float(np.max(
            np.maximum(np.abs(lo[:, 12]), np.abs(hi[:, 12])) - 1.))

    # Pedal overshoot is reported, not rejected: realized_command clips it.
    limits = {name: value for name, value in margins.items()
              if name != "pedal_overshoot_beyond_travel"}
    # A zero margin passes only for ">=" limits; strict ones need > 0.
    strict = {"speed_over_minimum", "path_progress_over_minimum"}
    failed = sorted(name for name, value in limits.items()
                    if (value <= 0. if name in strict else value < 0.))
    return {
        "certified": not failed,
        "failed": failed,
        "intervals": int(bounds.shape[0]),
        "margins": margins,
        "scope": "BDF dense interpolant of the reported solution; not the true ODE "
                 "trajectory; no load-closure or powertrain closure certificate",
    }


#: Largest linear-branch slip threshold and force floor for which the contact
#: conditions below ignore the tire's small-slip linear branch.
MAX_SLIP_EPS = 1.e-6
MAX_FORCE_EPS = 10.
#: Relative slack demanded of every contact condition.
CONTACT_SLACK = 1.e-3


def contact_nonnegativity_conditions(vehicle):
    """Check sufficient, parameter-dependent conditions for nonnegative loads.

    The quasi-static load model returns tire loads from the longitudinal and
    lateral acceleration guesses. If every implied acceleration lies in the
    box where both axle load totals are nonnegative, every load-fixed-point
    iterate (damped or not, converged or not) stays in that box, for any
    finite state. The conditions secure the axle totals only. Inside that box
    a single inside wheel can still be pushed below zero by the unequal front
    and rear roll distribution; the planar model lifts such a wheel at zero
    load (``wheel_lift``), so tire loads stay nonnegative by construction.
    ``tests/test_contact_condition_grid.py`` audits both statements on a grid. Conditions, with ``M`` the largest friction
    coefficient, ``k = h / L``, ``W = m g + q CLA`` and ``D = q CdA`` for every
    dynamic pressure ``q >= 0`` (affine in q, so the q = 0 value and the slope
    suffice):

    * (c) lateral lift: ``2 M h / t <= 1``
    * (a) front axle: ``k (M W - D) <= m g phi + beta q CLA``
    * (b) rear axle: ``k (M W + D) <= m g (1 - phi) + (1 - beta) q CLA``

    The tire force bound ``|F| <= M Fz`` holds on the nonlinear branch. The
    linear branch has no load factor, so the checker also requires a small
    ``slip_eps`` and ``force_eps``. The argument is an analytic sketch tested
    against sampled states, not a machine-checked proof.
    """
    tire, aero = vehicle.tire, vehicle.aero
    mu = max(tire.mu_0_x, tire.mu_s_x, tire.mu_0_y, tire.mu_s_y)
    mass, wheelbase = vehicle.total_mass, vehicle.wheelbase
    track, height = vehicle.track_width, vehicle.cg_height
    phi, beta = vehicle.front_mass_fraction, aero.aero_balance
    lift, drag_area = aero.downforce_area, aero.drag_area
    k = height / wheelbase
    slack = 1. - CONTACT_SLACK
    checks = {
        "lateral_lift": 2. * mu * height / track,
        "front_static": k * mu / phi,
        "rear_static": k * mu / (1. - phi),
    }
    allowed = {name: slack for name in checks}
    # The slope conditions divide out the area, so skip them for zero-lift cars.
    if lift > 0.:
        checks["front_aero"] = k * (mu * lift - drag_area) / (beta * lift) if beta > 0. else np.inf
        checks["rear_aero"] = (k * (mu * lift + drag_area) / ((1. - beta) * lift)
                               if beta < 1. else np.inf)
        allowed["front_aero"] = allowed["rear_aero"] = slack
    elif drag_area > 0.:
        # Without downforce, drag grows unopposed and no finite q is safe.
        checks["drag_without_downforce"] = np.inf
        allowed["drag_without_downforce"] = slack
    checks["linear_branch_slip_eps"] = tire.slip_eps / MAX_SLIP_EPS
    checks["linear_branch_force_eps"] = tire.force_eps / MAX_FORCE_EPS
    allowed["linear_branch_slip_eps"] = allowed["linear_branch_force_eps"] = 1.
    failed = sorted(name for name, ratio in checks.items() if not ratio <= allowed[name])
    return {
        "holds": not failed,
        "failed": failed,
        "ratios": {name: float(value) for name, value in checks.items()},
        "friction_bound": float(mu),
        "scope": "sufficient analytic conditions for every finite state; "
                 "unmachine-checked argument, rounding at exact lift saturation not covered",
    }
