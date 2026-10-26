"""Model-aware feedforward tracking for the yaw lap simulation suite.

This mode is explicit; historic PreviewDriver behavior is unchanged. Static
actuator inversion is feedforward, not a claim of transient tyre capability.
"""
from dataclasses import dataclass
import math
import numpy as np
from simplified_vehicle_models.solvers.forward_driver import PreviewDriver,DriverCommand


@dataclass(frozen=True)
class ModelAwareYawDriver(PreviewDriver):
    mass: float = 1.
    force_speed: tuple[float,...] = (0.,100.)
    drive_force: tuple[float,...] = (6.,6.)
    drag_force: tuple[float,...] = (0.,0.)
    brake_pedal: tuple[float,...] = (0.,1.)
    brake_wheel_force: tuple[float,...] = (0.,6.)
    yaw_damping: float = .12
    minimum_preview_m: float = 1.

    def __post_init__(self):
        super().__post_init__()
        if not math.isfinite(self.mass) or self.mass<=0:raise ValueError('mass must be positive')
        for name in ('yaw_damping','minimum_preview_m'):
            if not math.isfinite(getattr(self,name)) or getattr(self,name)<0:raise ValueError(f'{name} must be nonnegative')
        for axis,values in ((self.force_speed,self.drive_force),(self.force_speed,self.drag_force),
                            (self.brake_pedal,self.brake_wheel_force)):
            a=np.asarray(axis,float);v=np.asarray(values,float)
            if (a.ndim!=1 or len(a)<2 or v.shape!=a.shape or not np.all(np.isfinite(a))
                or not np.all(np.isfinite(v)) or np.any(np.diff(a)<=0) or np.any(v<0)):
                raise ValueError('actuator tables must be finite nonnegative with increasing axes')
        if np.any(np.diff(self.brake_wheel_force)<0):raise ValueError('brake force table must be monotone')

    @classmethod
    def for_vehicle(cls,plan,vehicle,point_mass,**overrides):
        from simplified_vehicle_models.formulations.point_mass import aerodynamic_forces,_default_motor_shares
        from simplified_vehicle_models.powertrain.assembly import maximum_drive_force
        from simplified_vehicle_models.powertrain.energy_storage import StorageState
        from simplified_vehicle_models.controls.brakes import brake_torques
        runtime=point_mass.drivetrain
        speeds=np.linspace(0.,max(1.,float(max(plan.speed))*1.1),129)
        shares=_default_motor_shares(runtime.powertrain.power_plant,runtime.drivetrain.transmission)
        drive=[maximum_drive_force(runtime.powertrain,runtime.drivetrain,float(v),
            StorageState(runtime.powertrain.energy_storage.initial_state_of_charge),
            effective_radius=point_mass.tire.effective_radius,wheel_shares=shares).maximum_drive_force for v in speeds]
        drag=[aerodynamic_forces(point_mass,float(v))[1] for v in speeds]
        pedal=np.linspace(0.,vehicle.maximum_pedal_force,257);brake=[]
        for value in pedal:
            hydraulic=max(float(value)-vehicle.controller.hydraulic_deadband_n,0.)
            result=brake_torques(hydraulic/vehicle.maximum_pedal_force,vehicle.maximum_pedal_force,
                vehicle.pedal_curve,vehicle.hydraulic_ratio,vehicle.maximum_brake_pressure,vehicle.caliper_area,
                vehicle.brake_bias,vehicle.pad_mu_front,vehicle.pad_mu_rear,
                vehicle.brake_effective_radius_front,vehicle.brake_effective_radius_rear)
            brake.append(float(np.sum(result.wheel_torque_magnitudes))/point_mass.tire.effective_radius)
        values=dict(plan=plan,wheelbase=vehicle.wheelbase,maximum_steering=vehicle.steering_maximum_angle,
            maximum_steering_rate=vehicle.steering_maximum_rate,maximum_brake_force=vehicle.maximum_pedal_force,
            mass=point_mass.total_mass,force_speed=tuple(speeds),drive_force=tuple(drive),drag_force=tuple(drag),
            brake_pedal=tuple(pedal),brake_wheel_force=tuple(brake),preview_seconds=.10,
            heading_gain=1.8,lateral_gain=2.,steering_time_constant=.08,
            pedal_travel_time=.18,pedal_time_constant=.03)
        values.update(overrides);return cls(**values)

    def command(self,observation):
        # Reuse observation/domain checks and existing pedal/brake safeguards.
        super().command(observation)
        speed=max(observation.speed,0.)
        target,curvature,feedforward=self.plan.sample(observation.distance)
        lookahead=max(self.minimum_preview_m,speed*self.preview_seconds)
        _,preview_curvature,_=self.plan.sample(observation.distance+lookahead)
        course_error=observation.heading_error+math.atan2(observation.lateral_speed,max(speed,.25))
        desired=(math.atan(self.wheelbase*preview_curvature)-self.heading_gain*course_error
            -math.atan2(self.lateral_gain*observation.lateral_error,max(speed,1.))
            -self.yaw_damping*(observation.yaw_rate-speed*curvature))
        desired=float(np.clip(desired,-self.maximum_steering,self.maximum_steering))
        steering=float(np.clip((desired-observation.steering)/self.steering_time_constant,
            -self.maximum_steering_rate,self.maximum_steering_rate))
        # Keep a small numerical reserve at the physical stop; trial states at
        # the bound must always be driven inward rather than held against it.
        stop = self.maximum_steering * (1. - 1e-6)
        steering = min(steering, (stop-observation.steering)/self.steering_time_constant)
        steering = max(steering, (-stop-observation.steering)/self.steering_time_constant)
        steering = float(np.clip(steering,-self.maximum_steering_rate,self.maximum_steering_rate))
        acceleration=feedforward+self.speed_gain*(target-observation.speed)
        force=self.mass*acceleration+float(np.interp(speed,self.force_speed,self.drag_force))
        if force>=0:
            available=float(np.interp(speed,self.force_speed,self.drive_force))
            throttle=float(np.clip(force/max(available,1e-12),0.,1.));brake=0.
        else:
            throttle=0.
            brake=float(np.interp(-force,self.brake_wheel_force,self.brake_pedal))
            governor=float(np.clip(observation.wheel_speed_margin/self.brake_governor_margin,0.,1.))
            brake*=governor
        return DriverCommand(throttle,brake,steering)
