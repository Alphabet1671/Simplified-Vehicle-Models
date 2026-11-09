"""Lightweight quasi-steady, hydraulic-limited brake authority predictor.

Predicts positive deceleration as ``-d(v_x)/dt`` (m/s²), not body-axis
acceleration in a corner. It uses the shared hydraulic inverse and a bounded
rolling-force balance; this is deliberately not a transient tire solver.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from simplified_vehicle_models.controls.brake_inverse import invert_brake_effort
from simplified_vehicle_models.controls.brakes import brake_torques
from simplified_vehicle_models.solvers.forward_powertrain import shape_brake_command


@dataclass(frozen=True, slots=True)
class BrakeAuthorityPrediction:
    """Prediction: deceleration in m/s², pedal force N, wheel torque N·m."""
    deceleration_mps2: float
    pedal_force_n: float
    wheel_torques_nm: np.ndarray
    in_domain: bool
    limiting_constraint: str
    shortfall_mps2: float = 0.0
    controlled_quantity: str = "-d(v_x)/dt"
    assumption: str = "quasi_steady_rolling_fixed_bias_contact_prefix; corner_projection_not_certified"

    def __post_init__(self):
        a = np.array(self.wheel_torques_nm, dtype=float, copy=True)
        if a.shape != (4,) or not np.all(np.isfinite(a)):
            raise ValueError("wheel_torques_nm must be a finite FL,FR,RL,RR vector")
        a.setflags(write=False)
        object.__setattr__(self, "wheel_torques_nm", a)


@dataclass(frozen=True, slots=True)
class BrakeAuthorityContext:
    """Immutable operating point and applicability bounds.

    ``speed_mps`` is body longitudinal speed, ``curvature_inv_m`` signed path
    curvature, ``wheel_radii_m`` loaded radii, ``normal_loads_n`` contact loads.
    Loads are estimates from a compatible full model, not a planar truth.
    Optional per-wheel longitudinal capacities bound combined-slip authority.
    """
    speed_mps: float
    curvature_inv_m: float = 0.0
    wheel_radii_m: tuple[float, float, float, float] | None = None
    normal_loads_n: tuple[float, float, float, float] | None = None
    longitudinal_capacity_n: tuple[float, float, float, float] | None = None
    soc: float = 1.0
    regen_enabled: bool = False
    v_y_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    steering_rad: float = 0.0

    def __post_init__(self):
        scalars = (self.speed_mps, self.curvature_inv_m, self.soc, self.v_y_mps,
                   self.yaw_rate_rps, self.steering_rad)
        if not all(math.isfinite(float(x)) for x in scalars):
            raise ValueError("context scalars must be finite")
        for name in ("wheel_radii_m", "normal_loads_n", "longitudinal_capacity_n"):
            value = getattr(self, name)
            if value is not None:
                value = tuple(float(x) for x in value)
                if len(value) != 4 or not all(math.isfinite(x) for x in value):
                    raise ValueError(f"{name} must be a finite four-tuple")
                if name == "wheel_radii_m" and any(x <= 0 for x in value):
                    raise ValueError("wheel radii must be positive")
                if name == "normal_loads_n" and any(x < 0 for x in value):
                    raise ValueError("normal loads must be nonnegative")
                if name == "longitudinal_capacity_n" and any(x < 0 for x in value):
                    raise ValueError("contact capacities must be nonnegative")
                object.__setattr__(self, name, value)
        if not 0.0 <= self.soc <= 1.0:
            raise ValueError("SOC must be in [0, 1]")
        if self.speed_mps <= 0.0:
            raise ValueError("speed must be positive")


class BrakeAuthorityPredictor:
    """Cached parameter-conditioned predictor; evaluate is bounded and stateless."""
    def __init__(self, vehicle, *, maximum_abs_curvature_inv_m=0.0,
                 minimum_speed_mps=2.0, maximum_speed_mps=40.0):
        if vehicle.controller.regen_enabled:
            raise ValueError("regenerative braking is unsupported")
        self.vehicle = vehicle
        from simplified_vehicle_models.solvers.vertical_forward import _drivetrain_context
        self._driven,self._gearing,self._gear_efficiency=_drivetrain_context(vehicle.planar if hasattr(vehicle,"planar") else vehicle)
        self.domain = (float(minimum_speed_mps), float(maximum_speed_mps),
                       float(maximum_abs_curvature_inv_m))
        if (not all(math.isfinite(x) for x in self.domain) or self.domain[0] <= 0
                or self.domain[1] <= self.domain[0] or self.domain[2] < 0):
            raise ValueError("invalid predictor domain")
        if not math.isfinite(vehicle.maximum_pedal_force) or vehicle.maximum_pedal_force <= 0:
            raise ValueError("maximum pedal force must be positive and finite")
        # Exact fast path is limited to unshaped constant-ratio circuits with
        # inactive pressure limit; all other chains use the exact forward map/inverse.
        from simplified_vehicle_models.controls.brake_inverse import _validate_monotonic
        _validate_monotonic(vehicle)
        from simplified_vehicle_models.solvers.forward_powertrain import shape_brake_command
        _, at_max = shape_brake_command(vehicle, vehicle.maximum_pedal_force)
        self._exact_map_cache_safe = (vehicle.pedal_curve.is_constant
            and vehicle.controller.hydraulic_deadband_n == 0.)
        self._linear_fast_path = (self._exact_map_cache_safe and vehicle.pedal_curve.is_constant
            and getattr(vehicle.controller, "actuator_calibration", None) is None
            and not brake_torques(at_max, vehicle.maximum_pedal_force, vehicle.pedal_curve,
                vehicle.hydraulic_ratio_front, vehicle.hydraulic_ratio_rear,
                vehicle.maximum_brake_pressure, vehicle.caliper_area, vehicle.brake_bias,
                vehicle.pad_mu_front, vehicle.pad_mu_rear, vehicle.brake_effective_radius_front,
                vehicle.brake_effective_radius_rear).is_pressure_limited)
        self._hydraulic_effort_total = None
        # Cache the forward-model endpoint once. Its plateau is a real actuator limit.
        self.maximum_torques_nm = np.array(brake_torques(
            vehicle.maximum_pedal_force, vehicle.maximum_pedal_force, vehicle.pedal_curve,
            vehicle.hydraulic_ratio_front, vehicle.hydraulic_ratio_rear,
            vehicle.maximum_brake_pressure, vehicle.caliper_area, vehicle.brake_bias,
            vehicle.pad_mu_front, vehicle.pad_mu_rear, vehicle.brake_effective_radius_front,
            vehicle.brake_effective_radius_rear).wheel_torques, copy=True)
        self.maximum_torques_nm.setflags(write=False)
        if self._linear_fast_path:
            self._hydraulic_effort_total = float(np.sum(np.abs(self.maximum_torques_nm)))

    def _exact_hydraulic_map(self, force):
        _, hydraulic_force = shape_brake_command(self.vehicle, force)
        return brake_torques(hydraulic_force, self.vehicle.maximum_pedal_force,
            self.vehicle.pedal_curve, self.vehicle.hydraulic_ratio_front,
            self.vehicle.hydraulic_ratio_rear, self.vehicle.maximum_brake_pressure,
            self.vehicle.caliper_area, self.vehicle.brake_bias, self.vehicle.pad_mu_front,
            self.vehicle.pad_mu_rear, self.vehicle.brake_effective_radius_front,
            self.vehicle.brake_effective_radius_rear)

    def _linear_effort_inverse(self, effort):
        total = float(np.sum(np.abs(self.maximum_torques_nm)))
        force = min(self.vehicle.maximum_pedal_force,
                    self.vehicle.maximum_pedal_force * effort / total) if total else 0.0
        from types import SimpleNamespace
        return SimpleNamespace(pedal_force_n=force, feasible=effort <= total)

    def _rolling_force(self, loads, radii, speed):
        from simplified_vehicle_models.tires.unitire_simple import evaluate_unitire
        result = evaluate_unitire(self.vehicle.tire, np.zeros(4), np.zeros(4), loads,
                                  max(speed, 0.1) / radii)
        return float(np.sum(-np.asarray(result.rolling_resistance_moment, dtype=float) / radii))

    @staticmethod
    def _validate_context(context):
        if not isinstance(context, BrakeAuthorityContext):
            raise TypeError("context must be BrakeAuthorityContext")
        if context.wheel_radii_m is None or context.normal_loads_n is None:
            raise ValueError("explicit loaded wheel radii and normal loads are required")
        if context.regen_enabled:
            raise ValueError("regenerative braking is unsupported")

    def predict(self, context: BrakeAuthorityContext, pedal_force_n: float,
                target_deceleration_mps2: float | None = None) -> BrakeAuthorityPrediction:
        self._validate_context(context)
        if not math.isfinite(pedal_force_n) or pedal_force_n < 0:
            raise ValueError("pedal_force_n must be finite and nonnegative")
        v = context.speed_mps
        vmin, vmax, kmax = self.domain
        in_domain = (vmin <= v <= vmax and abs(context.curvature_inv_m) <= kmax
                     and all(n > 0 for n in context.normal_loads_n))
        force = min(float(pedal_force_n), float(self.vehicle.maximum_pedal_force))
        if self._linear_fast_path:
            torques = np.asarray(self.maximum_torques_nm * (force / self.vehicle.maximum_pedal_force))
            mapped_pressure_limited = False
        else:
            mapped = self._exact_hydraulic_map(force)
            torques = np.asarray(mapped.wheel_torques, dtype=float)
            mapped_pressure_limited = mapped.is_pressure_limited
        capacities = context.longitudinal_capacity_n
        constraint = "none"
        prefix = 1.0
        if capacities is not None:
            # Fixed front/rear bias keeps one common-pedal vector. The scale is
            # the longest prefix satisfying every per-wheel contact bound.
            limits = np.asarray(capacities) * np.asarray(context.wheel_radii_m)
            demand = -torques
            ratios = np.divide(limits, demand, out=np.full(4, np.inf), where=demand > 0)
            prefix = float(min(1.0, np.min(ratios)))
            if prefix < 1.0:
                torques = torques * prefix
                # Return the common pedal command whose cached total hydraulic
                # effort corresponds to this capped torque prefix.
                target_effort = float(np.sum(np.abs(torques)))
                if self._linear_fast_path:
                    force = target_effort / self._hydraulic_effort_total * self.vehicle.maximum_pedal_force
                elif self._exact_map_cache_safe:
                    inverse = invert_brake_effort(self.vehicle, target_effort)
                    force = inverse.pedal_force_n
                else:
                    raise ValueError("contact-prefix replay unsupported for shaped/nonlinear hydraulic chain")
                replay = self._exact_hydraulic_map(force)
                torques = np.asarray(replay.wheel_torques, dtype=float)
                constraint = "tire_contact_capacity"
        radii = np.asarray(context.wheel_radii_m)
        # Wheel rotational inertia couples wheel torque to vehicle deceleration.
        effective_mass = float(self.vehicle.total_mass + self.vehicle.wheel_inertia * np.sum(1. / radii**2))
        aero = 0.5 * self.vehicle.aero.air_density * self.vehicle.aero.drag_area * v * v
        # Rolling resistance is queried from the model tire law at each explicit
        # normal load; negative tire rolling moment resists forward motion.
        loads = np.asarray(context.normal_loads_n, dtype=float)
        rolling_force = self._rolling_force(loads, radii, v)
        # Rolling/inertia balance uses a fixed torque bias; tires are only a
        # conservative scalar prefix cap, not a four-wheel allocator. Corner
        # force projection is diagnostic only, not an admission certificate.
        steering_projection = np.array([math.cos(context.steering_rad), math.cos(context.steering_rad), 1., 1.])
        road_force = float(np.sum((-torques / radii) * steering_projection)) + max(0.0, float(aero)) + rolling_force
        # Frenet/body kinematics: dv_x/dt = a_body_x + r*v_y. This is a
        # first-order force projection, not an exact combined-slip corner model.
        decel = max(0.0, road_force / effective_mass - context.yaw_rate_rps * context.v_y_mps)
        if not in_domain and constraint != "tire_contact_capacity":
            constraint = "outside_domain"
        elif force >= self.vehicle.maximum_pedal_force and pedal_force_n > force:
            constraint = "maximum_pedal_force"
        elif mapped_pressure_limited:
            constraint = "maximum_brake_pressure"
        elif constraint == "none" and pedal_force_n > 0:
            constraint = "hydraulic_torque"
        if target_deceleration_mps2 is not None and (not math.isfinite(target_deceleration_mps2) or target_deceleration_mps2 < 0):
            raise ValueError("target deceleration must be finite and nonnegative")
        if target_deceleration_mps2 is not None and decel + 1e-12 < target_deceleration_mps2:
            if constraint == "none": constraint = "contact_prefix_shortfall" if prefix < 1.0 else "authority_shortfall"
        shortfall = max(0., float(target_deceleration_mps2)-decel) if target_deceleration_mps2 is not None else 0.
        return BrakeAuthorityPrediction(decel, force, torques, in_domain, constraint, shortfall)

    def request_current(self, context: BrakeAuthorityContext, target_deceleration_mps2: float) -> BrakeAuthorityPrediction:
        """Pure O(1) inverse for a freshly-derived current state context.

        Supports only the validated unshaped, constant-ratio hydraulic map. Unlike
        :meth:`request`, this performs no tire query, bisection, or second prediction;
        its returned force is a feedforward command, not a feasibility certificate.
        """
        if not self._linear_fast_path:
            raise NotImplementedError("current-state fast inverse requires a constant linear hydraulic map")
        self._validate_context(context)
        target=float(target_deceleration_mps2)
        if not math.isfinite(target) or target < 0.:
            raise ValueError("target deceleration must be finite and nonnegative")
        if not self.domain[0] <= context.speed_mps <= self.domain[1]:
            raise ValueError("current speed is outside predictor inverse domain")
        if abs(context.curvature_inv_m) > self.domain[2]:
            raise ValueError("current curvature is outside predictor inverse domain")
        radii=np.asarray(context.wheel_radii_m,dtype=float)
        loads=np.asarray(context.normal_loads_n,dtype=float)
        v=context.speed_mps
        effective_mass=float(self.vehicle.total_mass+self.vehicle.wheel_inertia*np.sum(1./radii**2))
        aero=.5*self.vehicle.aero.air_density*self.vehicle.aero.drag_area*v*v
        # UniTire's pure-rolling moment is algebraic in load, radius, speed and
        # cached tire coefficients. Match _rolling_force without invoking its full kernel.
        tire=self.vehicle.tire
        fzn=loads/tire.f_z_rated
        coefficient=tire.f_rr_0+tire.f_rr_1*fzn+tire.f_rr_2*fzn**2
        omega=np.maximum(v,.1)/radii
        rolling_moment=-loads*coefficient*radii*(1.+tire.h_rr*np.tan(
            .5*math.pi*np.abs(omega)/max(tire.omega_cr,tire.omega_eps)))
        rolling=float(np.sum(-rolling_moment/radii))
        projection=np.array([math.cos(context.steering_rad),math.cos(context.steering_rad),1.,1.])
        weights=(-self.maximum_torques_nm/radii)*projection
        total=float(np.sum(-self.maximum_torques_nm))
        force_per_effort=float(np.sum(weights)/total)
        if not math.isfinite(force_per_effort) or force_per_effort<=0.:
            raise ValueError("current context has no positive common-bias longitudinal projection")
        requested=max(0.,(target+context.yaw_rate_rps*context.v_y_mps)*effective_mass-aero-rolling)
        requested_effort=requested/force_per_effort
        pedal=float(np.clip(requested_effort*self.vehicle.maximum_pedal_force/
                            self._hydraulic_effort_total,0.,self.vehicle.maximum_pedal_force))
        torques=self.maximum_torques_nm*(pedal/self.vehicle.maximum_pedal_force)
        if context.longitudinal_capacity_n is not None:
            ratios=np.divide(np.asarray(context.longitudinal_capacity_n)*radii,-torques,
                             out=np.full(4,np.inf),where=(-torques)>0.)
            prefix=float(np.clip(np.min(ratios),0.,1.))
            torques=torques*prefix
            pedal=pedal*prefix
        achieved=max(0.,(float(np.sum((-torques/radii)*projection))+aero+rolling)/effective_mass-
                     context.yaw_rate_rps*context.v_y_mps)
        in_domain=(self.domain[0]<=v<=self.domain[1] and
                   abs(context.curvature_inv_m)<=self.domain[2] and all(n>0 for n in loads))
        capacity_limited=(context.longitudinal_capacity_n is not None and prefix < 1.-1.e-12)
        constraint=("tire_contact_capacity" if capacity_limited else
                    ("maximum_pedal_force" if requested_effort>self._hydraulic_effort_total
                     else "hydraulic_torque"))
        return BrakeAuthorityPrediction(achieved,pedal,torques,in_domain,constraint,max(0.,target-achieved))

    def request(self, context: BrakeAuthorityContext, target_deceleration_mps2: float) -> BrakeAuthorityPrediction:
        """Invert requested decel by bounded bisection; report force-limit shortfall."""
        if not math.isfinite(target_deceleration_mps2) or target_deceleration_mps2 < 0:
            raise ValueError("target deceleration must be finite and nonnegative")
        self._validate_context(context)
        # Convert requested decel to common-pedal torque effort using the
        # current balance-bar vector. One scalar inverse plus one prediction;
        # no nested tire map or per-wheel allocation occurs on request.
        radii = np.asarray(context.wheel_radii_m, dtype=float)
        eff_mass = self.vehicle.total_mass + self.vehicle.wheel_inertia*np.sum(1./radii**2)
        aero = .5*self.vehicle.aero.air_density*self.vehicle.aero.drag_area*context.speed_mps**2
        loads = np.asarray(context.normal_loads_n, dtype=float)
        rolling_force = self._rolling_force(loads, radii, context.speed_mps)
        force_request = max(0., target_deceleration_mps2*eff_mass-aero-rolling_force)
        weights = -self.maximum_torques_nm/radii
        weighted_force_per_torque = float(np.sum(weights)/np.sum(-self.maximum_torques_nm))
        effort_request = force_request/weighted_force_per_torque
        inverse = (self._linear_effort_inverse(effort_request) if self._linear_fast_path
                   else invert_brake_effort(self.vehicle, effort_request))
        result = self.predict(context, inverse.pedal_force_n, target_deceleration_mps2)
        max_result = self.predict(context, self.vehicle.maximum_pedal_force, target_deceleration_mps2)
        if result.limiting_constraint == "tire_contact_capacity":
            return BrakeAuthorityPrediction(result.deceleration_mps2, result.pedal_force_n,
                result.wheel_torques_nm, result.in_domain, "tire_contact_capacity",
                max(0., target_deceleration_mps2-result.deceleration_mps2))
        if not inverse.feasible or result.deceleration_mps2 + .01 < target_deceleration_mps2:
            return BrakeAuthorityPrediction(max_result.deceleration_mps2, max_result.pedal_force_n,
                max_result.wheel_torques_nm, max_result.in_domain, "maximum_pedal_force",
                max(0., target_deceleration_mps2-max_result.deceleration_mps2))
        if not result.in_domain:
            return BrakeAuthorityPrediction(result.deceleration_mps2, result.pedal_force_n,
                result.wheel_torques_nm, False, "outside_domain", result.shortfall_mps2)
        return result
