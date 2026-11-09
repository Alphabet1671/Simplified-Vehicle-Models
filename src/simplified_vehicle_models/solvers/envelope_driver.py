"""Pure actual-speed and pedal-aware braking-envelope longitudinal policy."""
from __future__ import annotations
from dataclasses import dataclass
import math
from numbers import Real
import inspect
import numpy as np


# Bump when physical-mode command semantics change. Runner identity wiring is
# owned by the later orchestration task; execution-source hashing already sees this file.
PHYSICAL_POLICY_VERSION = "physical-policy-semantic-v2"


def _finite(value, name, *, positive=True):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    value = float(value)
    if not math.isfinite(value) or (value <= 0 if positive else value < 0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return value

@dataclass(frozen=True, slots=True)
class EnvelopeDriverSettings:
    speed_gain: float = 2.
    throttle_acceleration_scale: float = 2.5
    brake_acceleration_scale: float = 6.
    response_allowance_s: float = .06
    disabled_transition_time_s: float = .4
    positive_acceleration_reserve_mps2: float = 4.
    feedforward_distance_m: float = 2.
    planning_transition_time_s: float | None = None
    mode: str = "legacy"
    forecast_horizon_s: float = 3.0
    def __post_init__(self):
        if self.mode not in ("legacy", "physical-reachable-v1"):
            raise ValueError("mode must be legacy or physical-reachable-v1")
        # Public semantic version is independent of the compatibility mode label.
        if self.mode == "physical-reachable-v1":
            _ = PHYSICAL_POLICY_VERSION
        for name in ('speed_gain','throttle_acceleration_scale','brake_acceleration_scale','disabled_transition_time_s'):
            _finite(getattr(self,name),name)
        _finite(self.response_allowance_s,'response_allowance_s',positive=False)
        _finite(self.positive_acceleration_reserve_mps2,'positive_acceleration_reserve_mps2',positive=False)
        _finite(self.feedforward_distance_m,'feedforward_distance_m')
        if self.planning_transition_time_s is not None:
            _finite(self.planning_transition_time_s,'planning_transition_time_s')
        _finite(self.forecast_horizon_s,'forecast_horizon_s')
        if self.forecast_horizon_s > 3.:
            raise ValueError('forecast_horizon_s must be <= 3 seconds')

@dataclass(frozen=True, slots=True)
class PedalPolicyContext:
    throttle_release_rate: float
    brake_apply_rate: float
    pedal_time_constant: float
    maximum_brake_force: float
    governor_margin: float
    throttle_apply_rate: float | None = None
    brake_release_rate: float | None = None
    def __post_init__(self):
        for name in ("throttle_release_rate","brake_apply_rate","pedal_time_constant","maximum_brake_force","governor_margin"):
            _finite(getattr(self,name),name)
        for name in ("throttle_apply_rate","brake_release_rate"):
            value=getattr(self,name)
            if value is not None: _finite(value,name)

def advance_signed_pedal(position, target, duration, context):
    """Exact piecewise integration of the driver's signed pedal ODE.

    Splits at p=0, follows the plant's four direction/sign-specific rate
    limits, then uses the exact exponential branch once unsaturated.
    """
    p=float(np.clip(position,-1.,1.)); target=float(np.clip(target,-1.,1.)); left=float(duration)
    if left<0 or not math.isfinite(left): raise ValueError("duration must be finite nonnegative")
    tau=context.pedal_time_constant
    for _ in range(4):
        delta=target-p
        if abs(delta)<1e-14 or left<=1e-14: break
        if p>1e-12: rate=context.throttle_release_rate if delta<0 else (context.throttle_apply_rate or context.throttle_release_rate)
        elif p< -1e-12: rate=(context.brake_release_rate or context.brake_apply_rate) if delta>0 else context.brake_apply_rate
        else: rate=(context.throttle_apply_rate or context.throttle_release_rate) if delta>0 else context.brake_apply_rate
        rate=float(rate)
        direction=1. if delta>0 else -1.
        boundary=0. if p*target<0 else target
        distance=abs(boundary-p)
        threshold=tau*rate
        if abs(delta)>threshold+1e-14:
            linear_distance=max(0.,abs(delta)-threshold)
            linear_time=linear_distance/rate
            if p*target<0 and distance<=linear_distance+1e-14:
                crossing_time=distance/rate
                if crossing_time>=left:
                    p+=direction*rate*left; left=0.; break
                p=0.; left-=crossing_time; continue
            if linear_time>=left:
                p+=direction*rate*left; left=0.; break
            p+=direction*linear_distance; left-=linear_time
        else:
            p=target-delta*math.exp(-left/tau); left=0.
    return float(np.clip(p,-1.,1.))

@dataclass(frozen=True, slots=True)
class PhysicalBrakePlanningContext:
    """Immutable low-cost longitudinal forecast table for one operating condition.

    ``deceleration_by_force_mps2`` is built outside policy evaluation from a
    current-state BrakeAuthorityContext. Interpolation is deliberately scalar;
    no tire or hydraulic model is called during candidate rollout.
    """
    pedal: PedalPolicyContext
    force_knots_n: tuple[float, ...]
    deceleration_by_force_mps2: tuple[float, ...]
    speed_mps: float
    curvature_inv_m: float = 0.0
    drive_acceleration_table: tuple[tuple[float, ...], ...] = ()
    speed_knots_mps: tuple[float, ...] = ()
    maximum_brake_force_n: float | None = None
    authority_shortfall_mps2: float = 0.0
    speed_dependent_deceleration: tuple[tuple[float, ...], ...] = ()
    curvature_domain_inv_m: float | None = None
    maximum_drive_acceleration_mps2: float = 0.0
    planned_horizon_s: float = 0.0
    maximum_braking_margin_mps2: float = 0.0
    governor_margin: float = 0.85
    predictor: object | None = None
    safety_deceleration_by_force_mps2: tuple[float, ...] = ()
    current_authority_context: object | None = None
    current_predictor: object | None = None

    def __post_init__(self):
        if not isinstance(self.pedal, PedalPolicyContext):
            raise TypeError("pedal must be PedalPolicyContext")
        f=tuple(float(x) for x in self.force_knots_n)
        a=tuple(float(x) for x in self.deceleration_by_force_mps2)
        if len(f)<2 or len(f)!=len(a) or any(not math.isfinite(x) for x in f+a):
            raise ValueError("planning table needs matching finite force/deceleration knots")
        if f[0]!=0. or any(y<=x for x,y in zip(f,f[1:])) or any(y<x for x,y in zip(a,a[1:])):
            raise ValueError("planning table knots must be increasing from zero")
        if f[-1]>self.pedal.maximum_brake_force or not math.isfinite(self.speed_mps) or self.speed_mps<=0:
            raise ValueError("planning table force/speed domain is invalid")
        object.__setattr__(self,"force_knots_n",f)
        object.__setattr__(self,"deceleration_by_force_mps2",a)
        speeds=tuple(float(x) for x in self.speed_knots_mps)
        drives=tuple(tuple(float(y) for y in row) for row in self.drive_acceleration_table)
        if speeds and (len(speeds)!=len(drives) or any(y<=x for x,y in zip(speeds,speeds[1:]))):
            raise ValueError("drive map speed axis must be increasing and match rows")
        if drives and any(len(row)!=2 or not all(math.isfinite(y) and y>=0 for y in row) for row in drives):
            raise ValueError("drive map rows must contain zero/full-throttle acceleration")
        if not math.isfinite(self.maximum_drive_acceleration_mps2) or self.maximum_drive_acceleration_mps2<0:
            raise ValueError("maximum drive acceleration must be finite nonnegative")
        if not math.isfinite(self.planned_horizon_s) or self.planned_horizon_s<0 or self.planned_horizon_s>3.:
            raise ValueError("planned horizon must be in [0,3] seconds")
        if not math.isfinite(self.maximum_braking_margin_mps2) or self.maximum_braking_margin_mps2<0:
            raise ValueError("maximum braking margin must be nonnegative")
        object.__setattr__(self,"speed_knots_mps",speeds)
        object.__setattr__(self,"drive_acceleration_table",drives)
        speeddec=tuple(tuple(float(y) for y in row) for row in self.speed_dependent_deceleration)
        if speeddec and (len(speeddec)!=len(speeds) or any(len(row)!=len(f) for row in speeddec)
                         or any(not math.isfinite(y) or y<0 for row in speeddec for y in row)
                         or any(any(y<x for x,y in zip(row,row[1:])) for row in speeddec)):
            raise ValueError("speed-dependent brake table dimensions/values invalid")
        if not math.isfinite(self.governor_margin) or self.governor_margin<=0:
            raise ValueError("governor margin must be positive")
        object.__setattr__(self,"speed_dependent_deceleration",speeddec)
        safety=tuple(float(x) for x in self.safety_deceleration_by_force_mps2)
        if safety and (len(safety)!=len(f) or any(not math.isfinite(x) or x<0 for x in safety)
                       or any(y<x for x,y in zip(safety,safety[1:]))):
            raise ValueError("safety deceleration table must be finite, nonnegative and monotone")
        object.__setattr__(self,"safety_deceleration_by_force_mps2",safety)

    @classmethod
    def from_predictor(cls, predictor, authority_context, pedal_context, *, knot_count=8):
        """Build a bounded table once from explicit current-state authority data.

        This factory is intentionally outside ``decide``: hydraulic/tire queries
        happen during context creation, never in candidate or horizon loops.
        """
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
        if not isinstance(authority_context, BrakeAuthorityContext):
            raise TypeError("authority_context must be BrakeAuthorityContext")
        if isinstance(knot_count,bool) or not isinstance(knot_count,int) or not 2<=knot_count<=8:
            raise ValueError("knot_count must be in [2, 8]")
        forces=tuple(float(x) for x in np.linspace(0.,pedal_context.maximum_brake_force,knot_count))
        decels=[]
        safety_decels=[]
        for force in forces:
            result=predictor.predict(authority_context,force)
            if not result.in_domain:
                raise ValueError("authority predictor context is outside its declared domain")
            decels.append(float(result.deceleration_mps2))
            safety_decels.append(float(result.deceleration_mps2)*.60)
        forecast_min=min(predictor.domain[0], float(authority_context.speed_mps))
        forecast_max=max(predictor.domain[1], float(authority_context.speed_mps))
        speeds=tuple(np.linspace(forecast_min,forecast_max,8))
        # Every speed row is queried once during explicit context construction;
        # no tire/hydraulic work occurs in decide/candidate loops.
        speed_rows=[]
        base=authority_context
        for speed in speeds:
            row=[]
            from dataclasses import replace
            operating=replace(base,speed_mps=float(speed))
            # Sample under nominal vertical loads for a reusable table. The
            # .60 planning reduction is operational margin, not a physical bound.
            for force in forces:
                result=predictor.predict(operating,force)
                if not result.in_domain:
                    raise ValueError("speed-conditioned authority context outside domain")
                row.append(max(0.,float(result.deceleration_mps2)))
            speed_rows.append(tuple(row))
        # Physical table is distinct from the margin-reduced schedule forecast.
        # Conservative drive forecast uses the full-torque, minimum-SOC voltage
        # bound at each speed as an upper acceleration bound. Exact closure is
        # evaluated during context setup only, never per RHS candidate.
        from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
        vehicle=predictor.vehicle.planar if hasattr(predictor.vehicle,"planar") else predictor.vehicle
        driven,gearing,efficiency=(predictor._driven,predictor._gearing,predictor._gear_efficiency)
        drive_rows=[]
        for speed in speeds:
            wheel_speed=np.full(4,float(speed)/max(float(np.mean(authority_context.wheel_radii_m)),.1))
            command=__import__('simplified_vehicle_models.solvers.forward_driver',fromlist=['DriverCommand']).DriverCommand(1.,0.,0.)
            point=evaluate_forward_powertrain(vehicle,driven,gearing,efficiency,
                                              wheel_speed,0.0,command)
            mass=vehicle.total_mass+vehicle.wheel_inertia*np.sum(1./np.asarray(authority_context.wheel_radii_m)**2)
            positive_force=float(np.sum(np.maximum(point.wheel_torques,0.)/np.asarray(authority_context.wheel_radii_m)))
            upper=positive_force/max(mass,1.)
            drive_rows.append((0.,upper))
        drives=tuple(drive_rows)
        # Physical target mapping and safety planning are kept as separate tables.
        return cls(pedal_context,forces,tuple(max(0.,value) for value in decels),authority_context.speed_mps,
                   authority_context.curvature_inv_m,drives,speeds,pedal_context.maximum_brake_force,
                   max((max(decels)-min(decels))*0.05,0.),tuple(speed_rows),predictor.domain[2],
                   max((row[1] for row in drives),default=0.),governor_margin=pedal_context.governor_margin,
                   predictor=predictor,current_authority_context=authority_context)

    def force_for_deceleration(self, target, speed=None):
        """Continuous inverse of the un-margined physical deceleration map.

        Targets beyond the validated table saturate explicitly at its force cap.
        This is an actuator target, not a feasibility guarantee; the safety
        forecast remains separately available through ``safety_deceleration``.
        """
        target=max(0.,float(target))
        forces=self.force_knots_n
        values=self.deceleration_by_force_mps2
        if speed is not None and self.speed_dependent_deceleration:
            row_index=min(len(self.speed_knots_mps)-1,max(0,int(np.searchsorted(self.speed_knots_mps,speed))))
            if row_index and row_index<len(self.speed_knots_mps):
                lo,hi=self.speed_knots_mps[row_index-1:row_index+1]
                alpha=(speed-lo)/(hi-lo)
                values=tuple(a+alpha*(b-a) for a,b in zip(self.speed_dependent_deceleration[row_index-1],self.speed_dependent_deceleration[row_index]))
            else:
                values=self.speed_dependent_deceleration[row_index]
        return float(np.interp(target,values,forces,left=forces[0],right=forces[-1]))

    def safety_deceleration(self, force, speed=None):
        """Operational safety forecast, retaining the documented 0.60 margin."""
        physical=self.deceleration(force,speed)
        return None if physical is None else .60*physical

    def drive_acceleration(self, speed, throttle, station=None):
        if not self.speed_knots_mps or not self.drive_acceleration_table:
            return self.maximum_drive_acceleration_mps2
        if speed < self.speed_knots_mps[0] or speed > self.speed_knots_mps[-1]:
            return None
        i=min(len(self.speed_knots_mps)-2,max(0,int(np.searchsorted(self.speed_knots_mps,speed)-1)))
        lo,hi=self.speed_knots_mps[i:i+2]
        alpha=(speed-lo)/(hi-lo)
        a0=self.drive_acceleration_table[i][0]+alpha*(self.drive_acceleration_table[i+1][0]-self.drive_acceleration_table[i][0])
        a1=self.drive_acceleration_table[i][1]+alpha*(self.drive_acceleration_table[i+1][1]-self.drive_acceleration_table[i][1])
        return max(0.,a0+float(np.clip(throttle,0.,1.))*(a1-a0))

    def deceleration(self, force, speed=None, station=None):
        if force<self.force_knots_n[0] or force>self.force_knots_n[-1]: return None
        if self.curvature_domain_inv_m is not None and abs(self.curvature_inv_m)>self.curvature_domain_inv_m+1e-12: return None
        if speed is not None and self.speed_dependent_deceleration:
            speeds=self.speed_knots_mps
            if speed<speeds[0] or speed>speeds[-1]: return None
            if station is not None and self.curvature_domain_inv_m is not None:
                # Context is valid only at its explicit curvature; the caller
                # separately rejects trajectories entering unknown curvature.
                if abs(self.curvature_inv_m)>self.curvature_domain_inv_m+1e-12: return None
            si=min(len(speeds)-2,max(0,int(np.searchsorted(speeds,speed)-1)))
            alpha=(speed-speeds[si])/(speeds[si+1]-speeds[si])
            def interpolate(row):
                for i in range(len(self.force_knots_n)-1):
                    if force<=self.force_knots_n[i+1]:
                        beta=(force-self.force_knots_n[i])/(self.force_knots_n[i+1]-self.force_knots_n[i])
                        return row[i]+beta*(row[i+1]-row[i])
                return row[-1]
            a=interpolate(self.speed_dependent_deceleration[si])
            b=interpolate(self.speed_dependent_deceleration[si+1])
            return a+alpha*(b-a)
        if speed is not None and abs(speed-self.speed_mps)>max(1.,.15*self.speed_mps): return None
        for i in range(len(self.force_knots_n)-1):
            lo,hi=self.force_knots_n[i:i+2]
            if force<=hi:
                u=(force-lo)/(hi-lo)
                return self.deceleration_by_force_mps2[i]+u*(self.deceleration_by_force_mps2[i+1]-self.deceleration_by_force_mps2[i])
        return self.deceleration_by_force_mps2[-1]

@dataclass(frozen=True, slots=True)
class FinalPhysicalCommand:
    """Coherent final force request and its single current-state inverse result."""
    target_deceleration_mps2: float
    urgency_deceleration_mps2: float
    requested_deceleration_mps2: float
    force_n: float
    prediction: object | None


def final_physical_command(target_deceleration_mps2, urgency_deceleration_mps2,
                           *, predictor=None, current_context=None, planning_context=None,
                           speed=None):
    """Choose max(target, urgency), then map once through the current inverse.

    The frozen table is a compatibility fallback only when current predictor
    context is unavailable; its prediction must not be labeled current-state.
    """
    target=max(0.,float(target_deceleration_mps2))
    urgency=max(0.,float(urgency_deceleration_mps2))
    desired=max(target,urgency)
    prediction=None
    if predictor is not None and current_context is not None:
        from dataclasses import replace
        context=replace(current_context,speed_mps=float(speed)) if speed is not None else current_context
        prediction=predictor.request_current(context,desired)
        force=float(prediction.pedal_force_n)
    elif planning_context is not None:
        force=float(planning_context.force_for_deceleration(desired,speed))
    else:
        force=0.
    return FinalPhysicalCommand(target,urgency,desired,force,prediction)


@dataclass(frozen=True, slots=True)
class CapEventForecast:
    outcome: bool | None
    reason: str
    horizon_s: float
    samples: int
    event_station_m: float | None
    minimum_cap_slack_mps2_squared: float | None
    crossing_time_s: float | None = None


def forecast_cap_event(envelope, planning_context, pedal_position, final_force_n,
                       *, start_station_m, speed_mps, horizon_s=3., samples=60):
    """Cheap sampled rollout. Slack is cap_speed**2-speed**2 in m²/s²."""
    if not 0. < horizon_s <= 3. or not 1 <= samples <= 120:
        raise ValueError("forecast budget requires horizon in (0,3] and samples in [1,120]")
    event_distance=float(envelope.sample(start_station_m).binding_distance)
    event_station=float(start_station_m+max(0.,event_distance))
    if event_distance <= 0.:
        return CapEventForecast(None,"no_binding_event",horizon_s,0,event_station,None)
    speeds=planning_context.speed_knots_mps
    if speeds and (speed_mps < speeds[0] or speed_mps > speeds[-1]):
        return CapEventForecast(None,"speed_outside_forecast_domain",horizon_s,0,event_station,None)
    context=planning_context.pedal; dt=horizon_s/samples
    station=float(start_station_m); speed=float(speed_mps); pedal=float(pedal_position)
    minimum_slack=float("inf")
    target=-min(1.,max(0.,final_force_n/context.maximum_brake_force))
    for i in range(1,samples+1):
        pedal=advance_signed_pedal(pedal,target,dt,context)
        brake_force=max(-pedal,0.)*context.maximum_brake_force
        # The .60 safety forecast is an explicit operational margin; physical
        # target inversion and the actual command remain unscaled.
        decel=planning_context.safety_deceleration(brake_force,speed)
        drive=planning_context.drive_acceleration(speed,max(pedal,0.),station)
        if decel is None or drive is None:
            return CapEventForecast(None,"forecast_domain_exit",horizon_s,i,event_station,
                                    None if minimum_slack==float("inf") else minimum_slack)
        acceleration=drive-decel
        next_speed=max(0.,speed+acceleration*dt)
        next_station=station+.5*(max(0.,speed)+next_speed)*dt
        if next_station>=event_station:
            ds=max(0.,event_station-station)
            crossing_dt=min(dt,ds/max(speed,1e-12)) if acceleration==0. else min(dt,max(0.,(-speed+math.sqrt(max(0.,speed*speed+2.*acceleration*ds)))/acceleration))
            crossing_speed=max(0.,speed+acceleration*crossing_dt)
            event_sample=envelope.sample(event_station)
            cap=min(event_sample.corner_speed,event_sample.entry_speed)
            slack=cap*cap-crossing_speed*crossing_speed
            minimum_slack=min(minimum_slack,slack)
            return CapEventForecast(slack>=-0.5,"event_crossed",horizon_s,i,event_station,minimum_slack,
                                    (i-1)*dt+crossing_dt)
        station=next_station; speed=next_speed
        sample=envelope.sample(station)
        minimum_slack=min(minimum_slack,min(sample.corner_speed,sample.entry_speed)**2-speed**2)
    return CapEventForecast(None,"event_beyond_horizon",horizon_s,samples,event_station,
                            None if minimum_slack==float("inf") else minimum_slack)


@dataclass(frozen=True, slots=True)
class EnvelopeDecision:
    distance: float
    actual_speed: float
    pedal_position: float | None
    current_corner_cap: float
    preview_station: float
    preview_entry_speed: float
    reference_speed: float
    squared_speed: float
    squared_speed_slope: float
    envelope_deceleration: float
    binding_index: int
    binding_station: float
    binding_distance: float
    speed_domain_max: float
    transition_time_s: float
    reserve_distance_m: float
    speed_squared_slack: float
    acceleration_request: float
    requested_throttle: float
    ungoverned_brake_force: float
    governor_factor: float
    governed_brake_force: float
    authority_domain_exceeded: bool
    governor_shortfall: bool
    planning_transition_time_s: float | None = None
    actual_transition_time_s: float | None = None
    planner_mode: str = "legacy"
    candidate_count: int = 0
    rollout_horizon_s: float = 0.0
    feasible_schedule: bool | None = None
    fallback_reason: str | None = None
    physical_brake_force_n: float | None = None
    predicted_deceleration_mps2: float | None = None
    authority_shortfall_mps2: float | None = None
    tracking_error_mps2: float | None = None
    rollout_pre_governor_feasible: bool | None = None
    final_inverse_deceleration_mps2: float | None = None
    final_inverse_force_n: float | None = None
    final_inverse_shortfall_mps2: float | None = None
    raw_target_inverse_deceleration_mps2: float | None = None
    raw_target_inverse_force_n: float | None = None
    raw_target_inverse_shortfall_mps2: float | None = None
    urgency_force_n: float | None = None
    urgency_force_deceleration_mps2: float | None = None
    forecast_reason: str | None = None
    forecast_event_station_m: float | None = None
    forecast_crossing_time_s: float | None = None
    forecast_samples: int | None = None
    forecast_cap_slack_mps2_squared: float | None = None
    forecast_slack_units: str = "m^2/s^2"
    target_pedal_position: float | None = None
    transition_forecast_horizon_s: float = 0.0
    competing_corner_cap_current_mps: float | None = None
    competing_corner_cap_preview_mps: float | None = None
    def to_dict(self):
        return {name:getattr(self,name) for name in self.__dataclass_fields__}

@dataclass(frozen=True, slots=True)
class EnvelopePolicy:
    envelope: object
    settings: EnvelopeDriverSettings = EnvelopeDriverSettings()
    def __post_init__(self):
        if not callable(getattr(self.envelope,'sample',None)):
            raise TypeError('envelope must provide sample(station)')
        if not isinstance(self.settings,EnvelopeDriverSettings):
            raise TypeError('settings must be EnvelopeDriverSettings')
    @staticmethod
    def _derived(value, name):
        value=float(value)
        if not math.isfinite(value):
            raise ValueError(f"nonfinite derived {name} in envelope policy")
        return value

    def decide(self, observation, pedal_position=None, context=None):
        raw_station=observation.distance
        if isinstance(raw_station,bool) or not isinstance(raw_station,Real) or not math.isfinite(float(raw_station)):
            raise ValueError('observation.distance must be finite numeric')
        s=float(raw_station)
        # Open envelopes hold their endpoint outside the sampled interval. Solver
        # Jacobian/Newton trials may briefly probe a negative station at launch.
        v=_finite(observation.speed,'observation.speed',positive=False)
        for name in ('lateral_error','heading_error','yaw_rate','steering','lateral_speed'):
            value=getattr(observation,name)
            if isinstance(value,bool) or not isinstance(value,Real):
                raise TypeError(f'observation.{name} must be a real number')
            if not math.isfinite(float(value)):
                raise ValueError(f'observation.{name} must be finite')
        margin=getattr(observation,'wheel_speed_margin',float('inf'))
        if isinstance(margin,bool) or not isinstance(margin,Real) or math.isnan(float(margin)):
            raise ValueError('observation.wheel_speed_margin must be numeric and not NaN')
        if margin < 0:
            raise ValueError('observation.wheel_speed_margin must be nonnegative')
        if pedal_position is not None:
            if isinstance(pedal_position,bool) or not isinstance(pedal_position,Real): raise TypeError('pedal_position must be real')
            p=float(pedal_position)
            if not math.isfinite(p) or not -1.<=p<=1.: raise ValueError('pedal_position must be finite and in [-1, 1]')
        else: p=None
        planning_context = None
        if isinstance(context, PhysicalBrakePlanningContext):
            planning_context = context
            context = context.pedal
        if context is not None and not isinstance(context,PedalPolicyContext): raise TypeError('context must be PedalPolicyContext or None')
        if context is None:
            actual_T=self.settings.disabled_transition_time_s
            maxbrake=1.
            governor_margin=.85
        else:
            p_for_time=1. if p is None else p
            actual_T=max(p_for_time,0.)/context.throttle_release_rate+(1.-max(-p_for_time,0.))/context.brake_apply_rate+self.settings.response_allowance_s+3.*context.pedal_time_constant
            maxbrake=context.maximum_brake_force; governor_margin=context.governor_margin
        actual_T=self._derived(actual_T,'actual transition time')
        planning_T=self.settings.planning_transition_time_s
        T=actual_T if planning_T is None else self._derived(planning_T,'planning transition time')
        reserve=self._derived(v*T+.5*self.settings.positive_acceleration_reserve_mps2*T*T,'reserve distance')
        preview_station=self._derived(s+reserve,'preview station')
        current=self.envelope.sample(s); preview=self.envelope.sample(preview_station)
        qglobal=self._derived(self.envelope.maximum_speed**2,'global cap squared')
        h=self.settings.feedforward_distance_m
        def effective_q(station):
            entry=self.envelope.sample(self._derived(station+reserve,'effective entry station'))
            corner=self.envelope.sample(station)
            return self._derived(min(entry.squared_speed,corner.corner_speed**2,qglobal),'effective squared speed')
        qref=effective_q(s)
        q_ahead=effective_q(self._derived(s+h,'feedforward station'))
        if self.settings.mode == "physical-reachable-v1":
            # The spatial reference is stationary: pedal position affects only
            # forward reachability, never the cap station being tracked.
            def stationary_q(station):
                sample=self.envelope.sample(station)
                return self._derived(min(sample.squared_speed,sample.corner_speed**2,qglobal),'stationary cap')
            qref=stationary_q(s)
            q_ahead=stationary_q(self._derived(s+h,'feedforward station'))
        slope=self._derived((q_ahead-qref)/h,'finite-chord squared-speed slope')
        reference=math.sqrt(max(0.,qref))
        reference=self._derived(reference,'reference speed')
        slope=self._derived(slope,'selected squared-speed slope')
        raw=self._derived(.5*slope+self.settings.speed_gain*(reference-v),'acceleration request')
        global_acceleration_limit=self.settings.speed_gain*(self.envelope.maximum_speed-v)
        raw=self._derived(min(raw,global_acceleration_limit),'globally bounded acceleration request')
        throttle=0. if raw<=0 else min(1.,raw/self.settings.throttle_acceleration_scale)
        brake=self._derived(min(1.,max(0.,-raw/self.settings.brake_acceleration_scale))*maxbrake,'ungoverned brake force')
        candidate_count=0
        feasible_schedule=None
        fallback_reason=None
        physical_force=None
        predicted_decel=None
        actual_rollout_horizon=0.0
        shortfall=None
        tracking_error=None
        target_inverse_decel=None
        target_inverse_force=None
        target_inverse_shortfall=None
        raw_target_inverse_decel=None
        raw_target_inverse_force=None
        raw_target_inverse_shortfall=None
        urgency_force=None
        urgency_force_decel=None
        binding_distance=0.
        forecast_metadata=None
        if self.settings.mode == "physical-reachable-v1":
            if planning_context is None or p is None:
                fallback_reason="missing_explicit_physical_context_or_pedal"
                throttle=0.0; brake=0.0; feasible_schedule=None
            else:
                # Physical force inversion is independent of planning authority.
                # .60 is retained only as a separately reported operational
                # safety forecast; it never scales the actuation target.
                target_decel=max(0.,-raw)
                authority=planning_context.current_authority_context
                current_predictor=planning_context.current_predictor or planning_context.predictor
                current_result=None
                binding_distance=max(0.,float(current.binding_distance))
                required=0.
                if binding_distance>0.:
                    binding_station=s+binding_distance
                    cap_speed=min(self.envelope.sample(binding_station).corner_speed,
                                  self.envelope.sample(binding_station).entry_speed)
                    required=max(0.,(v*v-cap_speed*cap_speed)/(2.*max(binding_distance,1e-6)))
                urgency_force_decel=required
                # Never ask an inverse to manufacture a zero-deceleration brake
                # force; ordinary neutral/positive-demand operation coasts or drives.
                if target_decel <= 0. and required <= 0.:
                    command=FinalPhysicalCommand(target_decel,required,target_decel,0.,None)
                else:
                    command=final_physical_command(target_decel,required,
                        predictor=current_predictor,current_context=authority,
                        planning_context=planning_context,speed=v)
                physical_force=command.force_n
                current_result=command.prediction
                if current_result is not None:
                    target_inverse_decel=current_result.deceleration_mps2
                    target_inverse_force=physical_force
                    target_inverse_shortfall=current_result.shortfall_mps2
                else:
                    target_inverse_force=planning_context.force_for_deceleration(target_decel,v)
                    target_inverse_decel=planning_context.deceleration(target_inverse_force,v)
                    target_inverse_shortfall=(max(0.,target_decel-target_inverse_decel)
                                              if target_inverse_decel is not None else None)
                urgency_force=(planning_context.force_for_deceleration(required,v)
                               if required>0 else 0.)
                raw_target_inverse_force=planning_context.force_for_deceleration(target_decel,v)
                raw_target_inverse_decel=planning_context.deceleration(raw_target_inverse_force,v)
                raw_target_inverse_shortfall=(max(0.,target_decel-raw_target_inverse_decel)
                    if raw_target_inverse_decel is not None else None)
                brake=float(np.clip(physical_force,0.,context.maximum_brake_force))
                throttle=0. if brake>0. or raw<=0. else min(1.,raw/self.settings.throttle_acceleration_scale)
                # Telemetry keeps present-state prediction distinct from the
                # frozen initial-context forecast used only for event diagnosis.
                predicted_decel=(current_result.deceleration_mps2 if current_result is not None
                                 else planning_context.deceleration(brake))
                shortfall=(max(0.,target_decel-(predicted_decel or 0.))
                           if predicted_decel is not None else None)
                tracking_error=((predicted_decel or 0.)-target_decel
                                if predicted_decel is not None else None)
                horizon=getattr(self.settings,'forecast_horizon_s',3.0) if binding_distance>0. else 0.
                actual_rollout_horizon=horizon
                feasible_schedule=None
                if binding_distance<=0. and v>min(current.corner_speed,current.entry_speed)+1.e-8:
                    fallback_reason="current_cap_already_violated"
                candidate_count=0
        margin=getattr(observation,'wheel_speed_margin',float('inf'))
        governor=min(1.,max(0.,margin/governor_margin))
        governed=self._derived(brake*governor,'governed brake force')
        if self.settings.mode=="physical-reachable-v1" and planning_context is not None and p is not None and binding_distance>0.:
            forecast=forecast_cap_event(self.envelope,planning_context,p,governed,
                start_station_m=s,speed_mps=v,horizon_s=horizon,samples=60)
            forecast_metadata=forecast
            feasible_schedule=forecast.outcome
            fallback_reason=None if forecast.outcome is not None else forecast.reason
            if governor<1.0 and brake>0.0 and feasible_schedule is None:
                fallback_reason="governed_"+str(forecast.reason)
        slack=self._derived(qref-v*v,'squared-speed slack')
        diagnostics=(s,v,p,current.corner_speed,preview_station,preview.entry_speed,reference,qref,slope,
                     preview.deceleration,preview.binding_index,preview.binding_station,preview.binding_distance,
                     preview.speed_domain_max,T,reserve,slack,raw,throttle,brake,governor,governed)
        if any(isinstance(value,float) and not math.isfinite(value) for value in diagnostics):
            raise ValueError('nonfinite envelope policy diagnostic')
        return EnvelopeDecision(
            distance=s, actual_speed=v, pedal_position=p, current_corner_cap=current.corner_speed,
            preview_station=preview_station, preview_entry_speed=preview.entry_speed,
            reference_speed=reference, squared_speed=qref, squared_speed_slope=slope,
            envelope_deceleration=preview.deceleration, binding_index=preview.binding_index,
            binding_station=preview.binding_station, binding_distance=preview.binding_distance,
            speed_domain_max=preview.speed_domain_max, transition_time_s=T,
            reserve_distance_m=reserve, speed_squared_slack=slack, acceleration_request=raw,
            requested_throttle=throttle, ungoverned_brake_force=brake, governor_factor=governor,
            governed_brake_force=governed,
            authority_domain_exceeded=(v>current.speed_domain_max or v>preview.speed_domain_max),
            governor_shortfall=brake>governed+1e-12,
            planning_transition_time_s=planning_T, actual_transition_time_s=actual_T,
            planner_mode=self.settings.mode, candidate_count=candidate_count,
            rollout_horizon_s=actual_rollout_horizon, feasible_schedule=feasible_schedule,
            fallback_reason=fallback_reason, physical_brake_force_n=physical_force,
            predicted_deceleration_mps2=predicted_decel, authority_shortfall_mps2=shortfall,
            tracking_error_mps2=tracking_error, rollout_pre_governor_feasible=None,
            final_inverse_deceleration_mps2=target_inverse_decel,
            final_inverse_force_n=target_inverse_force, final_inverse_shortfall_mps2=target_inverse_shortfall,
            raw_target_inverse_deceleration_mps2=raw_target_inverse_decel,
            raw_target_inverse_force_n=raw_target_inverse_force,
            raw_target_inverse_shortfall_mps2=raw_target_inverse_shortfall,
            urgency_force_n=urgency_force, urgency_force_deceleration_mps2=urgency_force_decel,
            forecast_reason=forecast_metadata.reason if forecast_metadata else fallback_reason,
            forecast_event_station_m=forecast_metadata.event_station_m if forecast_metadata else None,
            forecast_crossing_time_s=forecast_metadata.crossing_time_s if forecast_metadata else None,
            forecast_samples=forecast_metadata.samples if forecast_metadata else None,
            forecast_cap_slack_mps2_squared=(forecast_metadata.minimum_cap_slack_mps2_squared if forecast_metadata else None),
            forecast_slack_units="m^2/s^2",
            target_pedal_position=float(throttle-brake/maxbrake) if context is not None else None,
            transition_forecast_horizon_s=(float(horizon) if self.settings.mode == "physical-reachable-v1" and planning_context is not None and p is not None else 0.),
            competing_corner_cap_current_mps=float(current.corner_speed),
            competing_corner_cap_preview_mps=float(preview.corner_speed))
