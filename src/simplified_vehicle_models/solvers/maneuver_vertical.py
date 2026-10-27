"""Cartesian open-loop maneuver adapter for the vertical 14-DoF plant."""
from __future__ import annotations

import numpy as np

from simplified_vehicle_models.controls.maneuvers.schema import CapabilityError
from simplified_vehicle_models.controls.maneuvers.types import Observation, RobotCommand
from simplified_vehicle_models.solvers.forward_driver import DriverCommand, RateSystem, SpeedPlan
from simplified_vehicle_models.solvers.maneuver_planar import ActuatorCalibration
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState, contact_velocities, rotate_tire_forces_to_body,
    slip_quantities, wheel_positions, wheel_steering_angles,
)
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
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
        """Record catalogue-compatible algebraic outputs at an accepted sample.

        All values come from the same vertical evaluator invocation used for this
        sample.  This mapping performs no later export-time plant replay.
        """
        values = np.asarray(state, dtype=float)
        system, (target, rate, throttle, brake) = self._evaluate(time_s, values, command)
        rates = system.solve()
        ax, ay = body_accelerations(values, rates)
        mechanical = values[list(MECHANICAL_INDEX)]
        evaluation, powertrain = self.base.last_evaluation, self.base.last_powertrain
        if evaluation is None or powertrain is None:
            raise RuntimeError("vertical accepted outputs require evaluated plant evidence")
        planar = self.vehicle.planar
        loads = np.asarray(evaluation.normal_loads, dtype=float)
        tire_forces = np.asarray(evaluation.tire_forces, dtype=float)
        fx, fy, mz = (tire_forces[:, column] for column in range(3))
        radii = np.asarray(loaded_radius(planar.tire, loads), dtype=float)
        angles = (np.asarray(wheel_steering_angles(float(values[STEERING_INDEX]), len(WHEELS)), dtype=float)
                  + np.asarray(self.vehicle.suspension.kinematics.toe(mechanical[13:17]), dtype=float))
        longitudinal, lateral = contact_velocities(
            PlanarState(float(mechanical[0]), float(mechanical[1]), float(mechanical[5]),
                        np.asarray(mechanical[9:13], dtype=float)),
            wheel_positions(planar), angles)
        slip_ratio, slip_angle = slip_quantities(np.asarray(mechanical[9:13], dtype=float), radii,
                                                  longitudinal, lateral, 1.0)
        body_x, body_y = rotate_tire_forces_to_body(fx, fy, angles)
        positions = np.asarray(wheel_positions(planar), dtype=float)
        cosine, sine = np.cos(values[2]), np.sin(values[2])
        rotation = np.array(((cosine, -sine), (sine, cosine)))
        world_wheels = np.array((values[0], values[1])) + positions @ rotation.T
        outputs = {
            # Canonical RunRecord/animator evidence.
            "speed": float(np.hypot(values[3], values[4])),
            "body-slip-angle": float(np.arctan2(values[4], values[3])),
            "position-x": float(values[0]), "position-y": float(values[1]),
            "heading-world": float(values[2]),
            "wheel-position-x": world_wheels[:, 0], "wheel-position-y": world_wheels[:, 1],
            "wheel-steering-angle": angles, "wheel-loaded-radius": radii,
            "acceleration-longitudinal": float(ax), "acceleration-lateral": float(ay),
            "yaw-acceleration": float(rates[8]), "acceleration-vertical": float(rates[5]),
            "roll": float(mechanical[7]), "pitch": float(mechanical[8]),
            "roll-rate": float(mechanical[3]), "pitch-rate": float(mechanical[4]),
            "heave": float(mechanical[6]),
            "tire-longitudinal-force": fx, "tire-lateral-force": fy,
            "tire-aligning-moment": mz, "tire-vertical-force": loads,
            "tire-slip-ratio": slip_ratio, "tire-slip-angle": slip_angle,
            "tire-force-in-body-x": body_x, "tire-force-in-body-y": body_y,
            "wheel-drive-torque": np.asarray(powertrain.wheel_torques, dtype=float),
            "throttle": float(throttle), "brake-force": float(brake),
            "brake-fraction": float(command.brake), "steering": float(values[STEERING_INDEX]),
            "steering-rate": float(rate), "bus-power": float(powertrain.bus_power),
            "bus-voltage": float(powertrain.voltage), "bus-current": float(powertrain.current),
            "soc": float(values[24]),
            # Maneuver-native compatibility channels retained for existing clients.
            "time_s": float(time_s), "requested_swa_rad": target,
            "swa_rad": float(values[STEERING_INDEX]), "swa_rate_rad_s": rate,
            "accelerator_force_n": self.calibration.accelerator_force(command.accelerator),
            "throttle_fraction": throttle, "brake_force_n": brake,
            "speed_mps": float(np.hypot(values[3], values[4])),
            "x_m": float(values[0]), "y_m": float(values[1]), "heading_rad": float(values[2]),
            "body_ax_mps2": float(ax), "body_ay_mps2": float(ay), "ay_mps2": float(ay),
            "roll_rad": float(mechanical[7]), "pitch_rad": float(mechanical[8]),
            "heave_m": float(mechanical[6]), "wheel_speeds_rad_s": np.array(mechanical[9:13], copy=True),
            "jounce_m": np.array(mechanical[13:17], copy=True),
            "jounce_rates_mps": np.array(mechanical[17:21], copy=True),
        }
        for i, wheel in enumerate(WHEELS):
            outputs[f"wheel_speed_{wheel}_rad_s"] = float(mechanical[9+i])
            outputs[f"jounce_{wheel}_m"] = float(mechanical[13+i])
        outputs["tire_states"] = np.array(values[TIRE_OFFSET:TIRE_OFFSET+12], copy=True)
        return outputs


__all__ = ["VerticalManeuverPlant"]
