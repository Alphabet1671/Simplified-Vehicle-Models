"""Cartesian open-loop maneuver adapter for the transient planar vehicle."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from simplified_vehicle_models.controls.maneuvers.schema import CapabilityError
from simplified_vehicle_models.controls.maneuvers.types import Observation, RobotCommand
from simplified_vehicle_models.controls.steering import steering_angles
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState, evaluate_planar_dynamics, wheel_steering_angles,
)
from simplified_vehicle_models.solvers.forward_driver import RateSystem, TrialStateError
from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
from simplified_vehicle_models.tires.unitire_simple import loaded_radius


@dataclass(frozen=True)
class ActuatorCalibration:
    maximum_swa_rad: float
    maximum_swa_rate_rad_s: float
    rack_travel_per_revolution_m: float
    steer_arm_length_m: float
    maximum_road_wheel_angle_rad: float
    ackermann_inner_angle_rad: np.ndarray
    ackermann_outer_offset_rad: np.ndarray
    accelerator_travel: np.ndarray
    accelerator_force_n: np.ndarray
    brake_travel: np.ndarray
    brake_force_n: np.ndarray

    @classmethod
    def from_dict(cls, payload: dict) -> "ActuatorCalibration":
        if not isinstance(payload, dict):
            raise CapabilityError("explicit actuator calibration is required")
        required = cls.__dataclass_fields__.keys()
        if any(key not in payload for key in required):
            raise CapabilityError("calibration must provide steering geometry, Ackermann, and both pedal maps")
        scalars = [float(payload[k]) for k in list(required)[:5]]
        if not np.all(np.isfinite(scalars)) or np.any(np.asarray(scalars) <= 0):
            raise ValueError("steering calibration limits and geometry must be positive and finite")
        def table(x, y, name):
            x, y = np.asarray(x, float), np.asarray(y, float)
            if x.ndim != 1 or y.shape != x.shape or x.size < 2 or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)) or np.any(np.diff(x) <= 0) or np.any(np.diff(y) < 0):
                raise ValueError(f"{name} calibration must be finite and monotonic")
            return x, y
        ai, ao = table(payload['ackermann_inner_angle_rad'], payload['ackermann_outer_offset_rad'], 'Ackermann')
        at, af = table(payload['accelerator_travel'], payload['accelerator_force_n'], 'accelerator')
        bt, bf = table(payload['brake_travel'], payload['brake_force_n'], 'brake')
        if at[0] > 0 or at[-1] < 1 or bt[0] > 0 or bt[-1] < 1 or af[0] < 0 or bf[0] < 0:
            raise ValueError("pedal maps must cover travel [0,1] with nonnegative force")
        return cls(*scalars, ai, ao, at, af, bt, bf)

    def _map(self, travel, x, y):
        if not np.isfinite(travel) or not 0 <= travel <= 1:
            raise ValueError("pedal travel must lie in [0,1]")
        return float(np.interp(travel, x, y))

    def accelerator_force(self, travel): return self._map(travel, self.accelerator_travel, self.accelerator_force_n)
    def brake_force(self, travel): return self._map(travel, self.brake_travel, self.brake_force_n)

    def wheel_angles(self, vehicle, swa):
        from simplified_vehicle_models.controls.steering import AckermannTable
        result = steering_angles(float(np.clip(swa, -self.maximum_swa_rad, self.maximum_swa_rad)),
            self.rack_travel_per_revolution_m, self.steer_arm_length_m,
            min(self.maximum_road_wheel_angle_rad, vehicle.steering_maximum_angle),
            AckermannTable(self.ackermann_inner_angle_rad, self.ackermann_outer_offset_rad))
        # Keep the calibration's left/right wheel channels stable across signs;
        # the measured Ackermann offset is applied symmetrically to the pair.
        angles = np.array(result.wheel_angles, copy=True)
        offset = float(np.interp(abs(result.reference_angle), self.ackermann_inner_angle_rad, self.ackermann_outer_offset_rad))
        magnitude = min(abs(result.reference_angle), self.maximum_road_wheel_angle_rad, vehicle.steering_maximum_angle)
        inner, outer = magnitude, max(magnitude - offset, 0.)
        return np.asarray([np.sign(swa) * outer, np.sign(swa) * inner, 0., 0.])


class ManeuverPlanarPlant:
    state_size = 12
    state_names = ("x", "y", "heading", "v_x", "v_y", "yaw_rate", "omega_fl", "omega_fr", "omega_rl", "omega_rr", "soc", "swa")

    def __init__(self, vehicle, calibration: ActuatorCalibration):
        if not isinstance(calibration, ActuatorCalibration):
            raise CapabilityError("a separate explicit ActuatorCalibration artifact is required")
        self.vehicle, self.calibration = vehicle, calibration
        transmission = vehicle.drivetrain.drivetrain.transmission
        idx = {n:i for i,n in enumerate(("fl","fr","rl","rr"))}
        self.driven = np.array([idx[n] for n in transmission.driven_wheels])
        if len(self.driven) != vehicle.motor.unit_count:
            raise CapabilityError("motor layout does not match driven wheels")
        self.gear_efficiency = transmission.gear_efficiency
        self.gearing = vehicle.motor.gear_ratio * self.gear_efficiency

    def initial_state(self, speed_mps: float, x_m=0., y_m=0., heading_rad=0.):
        vals = np.asarray([speed_mps,x_m,y_m,heading_rad], float)
        if not np.all(np.isfinite(vals)) or speed_mps <= 0:
            raise ValueError("initial speed must be positive and all pose values finite")
        s = np.zeros(12); s[:3] = [x_m,y_m,heading_rad]; s[3] = speed_mps
        s[10] = self.vehicle.storage.initial_state_of_charge
        loads = __import__('simplified_vehicle_models.formulations.planar_7dof', fromlist=['evaluate_normal_loads']).evaluate_normal_loads(self.vehicle, speed_mps, 0., 0.)
        radius = loaded_radius(self.vehicle.tire, loads)
        s[6:10] = speed_mps / radius
        return s

    def observe(self, time_s, state=None):
        if state is None: state, time_s = time_s, 0.
        rate = getattr(self, '_accepted_rate', np.zeros(12))
        ax, ay = rate[3] - state[5]*state[4], rate[4] + state[5]*state[3]
        return Observation(float(time_s), *map(float, (state[0],state[1],state[2],np.hypot(state[3],state[4]),state[5],state[11],ay)))

    def _evaluate(self, time_s, state, command):
        s=np.asarray(state,float)
        if s.shape != (12,) or not np.all(np.isfinite(s)): raise ValueError("invalid Cartesian planar state")
        if not 0 <= s[10] <= 1: raise TrialStateError("state of charge outside bounds")
        if s[3] <= 0 or np.any(s[6:10] < 0): raise TrialStateError("planar contact model requires forward motion and nonnegative wheel speeds")
        c=self.calibration
        target=float(np.clip(command.swa_rad,-c.maximum_swa_rad,c.maximum_swa_rad))
        swa_rate=float(np.clip((target-s[11])/0.08,-c.maximum_swa_rate_rad_s,c.maximum_swa_rate_rad_s))
        wheels=c.wheel_angles(self.vehicle,s[11])
        accelerator=c.accelerator_force(command.accelerator)
        maxdrive=max(c.accelerator_force(1.),1.e-12)
        throttle=float(np.clip(accelerator/maxdrive,0.,1.))
        brake=c.brake_force(command.brake)
        from simplified_vehicle_models.solvers.forward_driver import DriverCommand
        dc=DriverCommand(throttle,brake,swa_rate)
        p=evaluate_forward_powertrain(self.vehicle,self.driven,self.gearing,self.gear_efficiency,s[6:10],s[10],dc)
        mech=PlanarState(float(s[3]),float(s[4]),float(s[5]),s[6:10],float(s[10]))
        d=evaluate_planar_dynamics(self.vehicle,mech,p.wheel_torques,wheels)
        if not d.load_converged: raise RuntimeError(f"load closure failed: residual={d.load_residual:g}")
        if np.any(d.normal_loads < 0): raise TrialStateError("planar contact load is negative")
        r=np.zeros(12); h=s[2]
        r[0]=s[3]*np.cos(h)-s[4]*np.sin(h); r[1]=s[3]*np.sin(h)+s[4]*np.cos(h); r[2]=s[5]
        r[3:6]=[d.longitudinal_acceleration,d.lateral_acceleration,d.yaw_acceleration]; r[6:10]=d.wheel_accelerations
        r[10]=-p.current/self.vehicle.storage.pack_capacity; r[11]=swa_rate
        self._accepted_rate=r.copy(); self._last=(d,p,wheels,target,swa_rate,throttle,brake)
        return RateSystem(r)

    def evaluate(self,time_s,state,command): return self._evaluate(time_s,state,command)

    def accepted_outputs(self,time_s,state,command):
        self._evaluate(time_s,state,command)
        d,p,wheels,target,rate,throttle,brake=self._last
        return {'time_s':float(time_s),'swa_rad':float(state[11]),'requested_swa_rad':target,'swa_rate_rad_s':rate,'road_wheel_angles_rad':np.array(wheels,copy=True),'accelerator_force_n':self.calibration.accelerator_force(command.accelerator),'brake_force_n':brake,'throttle_fraction':throttle,'wheel_torques_nm':np.array(p.wheel_torques,copy=True)}
