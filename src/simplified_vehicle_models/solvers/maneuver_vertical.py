"""Cartesian open-loop maneuver adapter for the vertical 14-DoF plant."""
from __future__ import annotations

import numpy as np

from simplified_vehicle_models.controls.maneuvers.schema import CapabilityError
from simplified_vehicle_models.controls.maneuvers.types import Observation, RobotCommand
from simplified_vehicle_models.solvers.forward_driver import DriverCommand, RateSystem, SpeedPlan
from simplified_vehicle_models.solvers.maneuver_planar import ActuatorCalibration
from simplified_vehicle_models.solvers.vertical_forward import (
    MECHANICAL_INDEX, STEERING_INDEX, TIRE_OFFSET, WHEELS, body_accelerations,
    vertical_plant_class,
)


class VerticalManeuverPlant:
    """Expose the vertical forward evaluator through canonical maneuver I/O.

    A zero-curvature open plan makes the forward plant's path coordinates
    coincide with world x/y/heading: s=x, n=y, heading_error=heading.
    """

    def __init__(self, vehicle, calibration: ActuatorCalibration,
                 reference_length_m: float = 1_000_000.0):
        if not isinstance(calibration, ActuatorCalibration):
            raise CapabilityError("an explicit ActuatorCalibration artifact is required")
        if not np.isfinite(reference_length_m) or reference_length_m <= 0:
            raise ValueError("reference_length_m must be positive and finite")
        expected_force = float(vehicle.planar.controller.maximum_drive_motor_torque_nm)
        if not np.isclose(calibration.accelerator_force_n[-1], expected_force,
                          rtol=1e-9, atol=1e-9):
            raise CapabilityError("accelerator force map endpoint must equal vehicle maximum drive motor torque")
        self.vehicle, self.calibration = vehicle, calibration
        self.reference_length_m = float(reference_length_m)
        self._plant_class = vertical_plant_class(vehicle)
        self.state_size = self._plant_class.state_size
        self.base = None

    def _new_base(self, speed_mps):
        plan = SpeedPlan(np.array([0., self.reference_length_m]),
                         np.array([speed_mps, speed_mps]), np.zeros(2), is_closed=False)
        self.base = self._plant_class(self.vehicle, plan)
        return self.base

    def initial_state(self, speed_mps: float, x_m=0., y_m=0., heading_rad=0.):
        values = np.asarray([speed_mps, x_m, y_m, heading_rad], dtype=float)
        if not np.all(np.isfinite(values)) or speed_mps <= 0:
            raise ValueError("initial speed must be positive and pose values finite")
        base = self._new_base(float(speed_mps))
        state = base.initial_state(float(speed_mps))
        state = np.array(state, dtype=float, copy=True)
        state[0], state[1], state[2] = float(x_m), float(y_m), float(heading_rad)
        return state

    def _ensure_base(self, state):
        if self.base is None:
            values = np.asarray(state, dtype=float)
            if values.shape != (self.state_size,):
                raise ValueError("invalid vertical maneuver state")
            speed = float(values[3])
            self._new_base(max(speed, 1e-9))
        return self.base

    def _evaluate(self, time_s, state, command):
        if not isinstance(command, RobotCommand):
            raise TypeError("command must be a RobotCommand")
        values = np.asarray(state, dtype=float)
        base = self._ensure_base(values)
        if values.shape != (self.state_size,) or not np.all(np.isfinite(values)):
            raise ValueError("invalid vertical maneuver state")
        c = self.calibration
        target = float(np.clip(command.swa_rad, -c.maximum_swa_rad, c.maximum_swa_rad))
        rate = float(np.clip((target - values[STEERING_INDEX]) / 0.08,
                             -c.maximum_swa_rate_rad_s, c.maximum_swa_rate_rad_s))
        accelerator = c.accelerator_force(command.accelerator)
        maximum = c.accelerator_force(1.)
        throttle = float(np.clip(accelerator / maximum, 0., 1.))
        brake = c.brake_force(command.brake)
        return base.evaluate(time_s, values, DriverCommand(throttle, brake, rate)), (target, rate, throttle, brake)

    def evaluate(self, time_s, state, command):
        return self._evaluate(time_s, state, command)[0]

    def observe(self, time_s, state=None, command=None):
        if state is None:
            state, time_s = time_s, 0.
        if command is None:
            command = RobotCommand(0., 0., 0.)
        values = np.asarray(state, dtype=float)
        system, _ = self._evaluate(time_s, values, command)
        rates = system.solve()
        ax, ay = body_accelerations(values, rates)
        return Observation(float(time_s), float(values[0]), float(values[1]),
                           float(values[2]), float(np.hypot(values[3], values[4])),
                           float(values[8]), float(values[STEERING_INDEX]), float(ay))

    def accepted_outputs(self, time_s, state, command) -> dict:
        values = np.asarray(state, dtype=float)
        system, (target, rate, throttle, brake) = self._evaluate(time_s, values, command)
        rates = system.solve()
        ax, ay = body_accelerations(values, rates)
        mechanical = values[list(MECHANICAL_INDEX)]
        outputs = {
            "time_s": float(time_s), "requested_swa_rad": target,
            "swa_rad": float(values[STEERING_INDEX]), "swa_rate_rad_s": rate,
            "accelerator_force_n": self.calibration.accelerator_force(command.accelerator),
            "throttle_fraction": throttle, "brake_force_n": brake,
            "speed_mps": float(np.hypot(values[3], values[4])),
            "x_m": float(values[0]), "y_m": float(values[1]),
            "heading_rad": float(values[2]), "body_ax_mps2": float(ax),
            "body_ay_mps2": float(ay), "ay_mps2": float(ay),
            "roll_rad": float(mechanical[7]), "pitch_rad": float(mechanical[8]),
            "heave_m": float(mechanical[6]),
            "wheel_speeds_rad_s": np.array(mechanical[9:13], copy=True),
            "jounce_m": np.array(mechanical[13:17], copy=True),
            "jounce_rates_mps": np.array(mechanical[17:21], copy=True),
        }
        for i, wheel in enumerate(WHEELS):
            outputs[f"wheel_speed_{wheel}_rad_s"] = float(mechanical[9+i])
            outputs[f"jounce_{wheel}_m"] = float(mechanical[13+i])
        outputs["tire_states"] = np.array(values[TIRE_OFFSET:TIRE_OFFSET+12], copy=True)
        return outputs


__all__ = ["VerticalManeuverPlant"]
