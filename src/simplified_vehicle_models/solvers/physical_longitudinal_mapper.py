"""Opt-in quasi-steady force-to-actuator command mapping.

This maps point-mass equilibrium forces plus an optional wheel-inertia proxy
through the existing actuator inverse. It is not a transient tire/body inverse
or a physical feasibility certificate.
"""
from __future__ import annotations

from dataclasses import dataclass
from numbers import Real
import math

import numpy as np

from simplified_vehicle_models.solvers.forward_driver import DriverCommand
from simplified_vehicle_models.solvers.longitudinal_actuator import (
    ActuatorInversionSettings, invert_longitudinal_actuators,
)


@dataclass(frozen=True)
class PhysicalMapperSettings:
    speed_gain: float = 2.
    torque_tolerance_nm: float = 1.e-4
    root_method: str = "safeguarded_interpolation"
    include_wheel_inertia: bool = True

    def __post_init__(self):
        _finite(self.speed_gain, "speed_gain", positive=True)
        _finite(self.torque_tolerance_nm, "torque_tolerance_nm", positive=True)
        if self.root_method not in ("bisection", "safeguarded_interpolation"):
            raise ValueError("root_method must select a supported actuator root method")
        if type(self.include_wheel_inertia) is not bool:
            raise ValueError("include_wheel_inertia must be boolean")


def _finite(value, name, *, positive=False, nonnegative=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (Real, np.number)):
        raise ValueError(f"{name} must be a finite real number")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.) or (nonnegative and result < 0.):
        raise ValueError(f"{name} must be finite and in range")
    return result


class PhysicalLongitudinalMapper:
    """Callable opt-in mapper suitable for ``solve_forward(command_mapper=...)``."""

    _required = ("s", "n", "heading_error", "v_x", "v_y", "yaw_rate",
                 "omega_fl", "omega_fr", "omega_rl", "omega_rr", "soc", "steering")
    _wheel_names = ("omega_fl", "omega_fr", "omega_rl", "omega_rr")

    def __init__(self, vehicle, reference, *, state_names, settings=None,
                 brake_governor_margin=.85, equilibrium_evaluator=None,
                 actuator_inverter=None):
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar

        if bool(vehicle.controller.regen_enabled):
            raise NotImplementedError("physical mapper does not support regen-enabled controllers")
        self.vehicle = vehicle
        self.reference = reference
        self.settings = PhysicalMapperSettings() if settings is None else settings
        if not isinstance(self.settings, PhysicalMapperSettings):
            raise ValueError("settings must be PhysicalMapperSettings")
        self.brake_governor_margin = _finite(brake_governor_margin, "brake_governor_margin", positive=True)
        names = tuple(state_names)
        if len(set(names)) != len(names) or any(name not in names for name in self._required):
            raise ValueError("state_names must uniquely publish all required state fields")
        self._indices = {name: names.index(name) for name in self._required}
        self._state_size = len(names)
        self.point_mass_vehicle = point_mass_view_of_planar(vehicle)
        inertia = _finite(vehicle.wheel_inertia, "wheel_inertia", nonnegative=True)
        self._wheel_inertia = inertia
        if equilibrium_evaluator is None:
            from simplified_vehicle_models.formulations.point_mass import evaluate_equilibrium
            equilibrium_evaluator = evaluate_equilibrium
        if not callable(equilibrium_evaluator) or not callable(actuator_inverter or invert_longitudinal_actuators):
            raise ValueError("evaluators must be callable")
        self._equilibrium = equilibrium_evaluator
        self._inverter = actuator_inverter or invert_longitudinal_actuators
        self._counts = dict(callback_queries=0, actuator_saturations=0,
                             infeasible_quasi_steady_demands=0,
                             max_abs_actuator_torque_error_nm=0.,
                             max_abs_requested_torque_nm=0.,
                             actuator_evaluations_total=0,
                             brake_governor_limited_queries=0, failed_queries=0)

    def snapshot_diagnostics(self):
        return {**self._counts, "accepted": False, "U3_qualified": False,
                "physically_validated": False,
                "model": "quasi_steady_force_plus_optional_wheel_inertia_proxy",
                "no_tv": True}

    def __call__(self, time, plant_state, observation, requested):
        try:
            time = _finite(time, "time", nonnegative=True)
            if not isinstance(requested, DriverCommand):
                raise ValueError("requested must be DriverCommand")
            req = tuple(_finite(v, "requested command") for v in
                        (requested.throttle, requested.brake_force, requested.steering_rate))
            if req[2] != requested.steering_rate:
                raise ValueError("invalid steering rate")
            if abs(req[2]) > _finite(self.vehicle.steering_maximum_rate, "maximum steering rate", positive=True):
                raise ValueError("requested steering rate exceeds vehicle limit")
            state = np.array(plant_state, dtype=float, copy=True)
            state.setflags(write=False)
            if state.shape != (self._state_size,) or not np.all(np.isfinite(state)):
                raise ValueError("plant_state must match state_names and be finite")
            values = {name: float(state[index]) for name, index in self._indices.items()}
            wheels = np.asarray([values[name] for name in self._wheel_names])
            speed = values["v_x"]
            if np.any(wheels < 0.) or not 0. <= values["soc"] <= 1. or speed < 0.:
                raise ValueError("wheel speeds or SOC outside physical domain")
            observation_fields = ("distance", "lateral_error", "heading_error", "speed",
                                  "yaw_rate", "steering", "lateral_speed")
            if not all(math.isfinite(float(getattr(observation, field))) for field in observation_fields):
                raise ValueError("observation fields must be finite")
            if observation.speed < 0.:
                raise ValueError("observation speed must be nonnegative")
            coherence = (("distance", "s"), ("lateral_error", "n"),
                         ("heading_error", "heading_error"), ("speed", "v_x"),
                         ("lateral_speed", "v_y"), ("yaw_rate", "yaw_rate"),
                         ("steering", "steering"))
            if any(not math.isclose(float(getattr(observation, field)), values[name],
                                    rel_tol=0., abs_tol=1.e-8)
                   for field, name in coherence):
                raise ValueError("observation does not correspond to plant_state")
            target, curvature, feedforward = self.reference.sample(observation.distance)
            desired = _finite(feedforward + self.settings.speed_gain * (target - observation.speed), "desired acceleration")
            eq = self._equilibrium(self.point_mass_vehicle, observation.speed, curvature, desired)
            loads = np.asarray(eq.normal_loads, dtype=float)
            forces = np.asarray(eq.longitudinal_forces, dtype=float)
            if loads.shape != (4,) or forces.shape != (4,) or not np.all(np.isfinite(loads)) or not np.all(np.isfinite(forces)) or np.any(loads < 0.):
                raise ValueError("equilibrium loads/forces must be finite four-vectors with nonnegative loads")
            from simplified_vehicle_models.tires.unitire_simple import loaded_radius
            radii = np.asarray(loaded_radius(self.vehicle.tire, loads), dtype=float)
            if radii.shape != (4,) or not np.all(np.isfinite(radii)) or np.any(radii <= 0.):
                raise ValueError("loaded radii must be finite and positive")
            torque = float(np.dot(forces, radii))
            if self.settings.include_wheel_inertia:
                torque += float(np.sum(self._wheel_inertia * desired / radii))
            torque = _finite(torque, "requested wheel torque")
            # Keep callbacks isolated from caller-owned mutables.
            radii.setflags(write=False)
            result = self._inverter(self.vehicle, wheels.copy(), values["soc"], torque,
                                    settings=ActuatorInversionSettings(
                                        torque_tolerance_nm=self.settings.torque_tolerance_nm,
                                        root_method=self.settings.root_method))
            self._counts["callback_queries"] += 1
            if not bool(eq.is_feasible):
                self._counts["infeasible_quasi_steady_demands"] += 1
            self._counts["actuator_saturations"] += int(bool(result.saturated))
            self._counts["max_abs_actuator_torque_error_nm"] = max(
                self._counts["max_abs_actuator_torque_error_nm"], abs(float(result.torque_error_nm)))
            self._counts["max_abs_requested_torque_nm"] = max(
                self._counts["max_abs_requested_torque_nm"], abs(torque))
            self._counts["actuator_evaluations_total"] += int(result.evaluations)
            margin = float(observation.wheel_speed_margin)
            if math.isnan(margin) or margin == -math.inf:
                raise ValueError("wheel_speed_margin must not be NaN or negative infinity")
            governor = min(max(margin / self.brake_governor_margin, 0.), 1.)
            brake = float(result.brake_force) * governor
            self._counts["brake_governor_limited_queries"] += int(governor < 1. and brake > 0.)
            return DriverCommand(float(result.throttle) if brake == 0. else 0., brake, req[2])
        except Exception:
            self._counts["failed_queries"] += 1
            raise
