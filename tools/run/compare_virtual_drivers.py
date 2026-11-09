"""Preparation and pure-analysis utilities for controlled virtual-driver comparisons.

No truth simulation is performed by preparation. Runtime adapters preserve the
production solve_forward API and require one actual shared compiled setup.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from hashlib import sha256
import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
from typing import Mapping

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
_LAST_PREPARED_FACTORY=None  # populated only by preparation CLI for local contract tests
HISTORICAL_SOURCE_SHA256="d184e21202be8f1683fc4110527764b94e5bce894596e61a374657578679d971"
HISTORICAL_COMMIT="b303889dd6a483d76a987320167a6586436ff69b"
VARIANT_IDS=("original-forward-law/current-physical-map","current-physical","hybrid")
ALGORITHM_IDS=("historical-preview-forward-backward-current-limits-v1","physical-reachable-v1","hybrid-drive-brake-v1")
REFERENCE_LAWS=("historical-speed-squared-forward-backward-plus-periodic-4m-smoothing","stationary-physical-reachable-envelope-v1","hybrid-shared-capability-speed-plan-v1")


def sha256_bytes(data): return sha256(data).hexdigest()


@dataclass(frozen=True)
class RunConfigIdentity:
    model_semantic_sha256:str
    source_hashes:tuple[tuple[str,str],...]
    track_sha256:str
    calibration_hashes:tuple[tuple[str,str],...]
    initial_full_state_sha256:str
    initial_soc:float
    pedal_and_steering_state:tuple[float,...]
    solver:tuple[tuple[str,object],...]
    lateral_margin:float
    brake_margin:float
    provider_identity:str
    allocator_identity:str
    foot_rates:tuple[float,float,float,float]
    pedal_lag_s:float
    maximum_pedal_force_n:float
    governor_margin:float
    corner_caps_sha256:str


@dataclass(frozen=True)
class ComparisonIdentity:
    variant_id:str
    algorithm_id:str
    reference_law:str
    mapping_id:str
    controller_source_sha256:str
    current_source_closure_sha256:str
    run_config:RunConfigIdentity
    runtime_binding_sha256:str=""


@dataclass(frozen=True)
class ComparisonCommand:
    variant_id:str
    reference_speed_mps:float
    target_progress_acceleration_mps2:float
    body_vx_dot_target_mps2:float|None
    body_conversion_status:str
    throttle_request:float
    requested_brake_force_n:float
    mapping_id:str
    mapped_command:object
    policy_diagnostics:object


def validate_primary_identities(items):
    if len(items)!=3 or {i.variant_id for i in items}!=set(VARIANT_IDS): raise ValueError("comparison requires exactly three declared primary variants")
    if any(i.mapping_id!="current-physical-inverse-v1" for i in items): raise ValueError("primary variants must use same current physical inverse")
    if len({i.current_source_closure_sha256 for i in items})!=1: raise ValueError("primary variants must share current source closure")
    if any(i.run_config!=items[0].run_config for i in items[1:]): raise ValueError("primary variants differ in frozen run configuration")
    for i in items:
        n=VARIANT_IDS.index(i.variant_id)
        if (i.algorithm_id,i.reference_law)!=(ALGORITHM_IDS[n],REFERENCE_LAWS[n]): raise ValueError("undeclared algorithm/reference identity")
        if n==0 and i.controller_source_sha256!=HISTORICAL_SOURCE_SHA256: raise ValueError("historical source identity mismatch")
        if n>0 and (not i.controller_source_sha256 or i.controller_source_sha256==HISTORICAL_SOURCE_SHA256): raise ValueError("current controller source identity missing")


@dataclass(frozen=True)
class HistoricalSpeedPlan:
    distance:np.ndarray
    speed:np.ndarray
    curvature:np.ndarray
    acceleration:np.ndarray
    is_closed:bool=True
    def __post_init__(self):
        for name in ("distance","speed","curvature","acceleration"):
            values=np.asarray(getattr(self,name),dtype=np.float64)
            if values.ndim!=1 or len(values)<2 or not np.all(np.isfinite(values)):
                raise ValueError(f"historical plan {name} must be finite one-dimensional data")
            # Back with immutable bytes: setflags(write=True) cannot undo this.
            frozen=np.frombuffer(np.ascontiguousarray(values).tobytes(),dtype=np.float64)
            object.__setattr__(self,name,frozen)
    def sample(self,station):
        s=float(station)%float(self.distance[-1]) if self.is_closed else float(station)
        return tuple(float(np.interp(s,self.distance,a)) for a in (self.speed,self.curvature,self.acceleration))


def _periodic_smooth_acceleration(distance_m,acceleration_mps2,half_window_m=4.):
    x=np.asarray(distance_m,dtype=float);y=np.asarray(acceleration_mps2,dtype=float)
    if x.ndim!=1 or y.shape!=x.shape or len(x)<2 or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)) or np.any(np.diff(x)<=0): raise ValueError("historical smoother inputs invalid")
    half=float(half_window_m);lap=float(x[-1])
    if not math.isfinite(half) or half<=0: raise ValueError("smoothing half-window must be positive")
    if lap<=4.*half:return y.copy()
    out=np.empty(len(x)-1)
    for i,s in enumerate(x[:-1]):
        d=(x[:-1]-s+lap/2)%lap-lap/2;out[i]=np.mean(y[:-1][np.abs(d)<=half])
    return np.r_[out,out[0]]


def build_historical_speed_plan(distance_m,curvature_inv_m,corner_speed_cap_mps,*,drive_acceleration_mps2,brake_deceleration_mps2,maximum_speed_mps,lateral_acceleration_limit_mps2,is_closed=True,smoothing_half_window_m=4.):
    x=np.asarray(distance_m,dtype=float);k=np.asarray(curvature_inv_m,dtype=float);cap=np.asarray(corner_speed_cap_mps,dtype=float)
    vals=(drive_acceleration_mps2,brake_deceleration_mps2,maximum_speed_mps,lateral_acceleration_limit_mps2)
    if x.ndim!=1 or len(x)<3 or k.shape!=x.shape or cap.shape!=x.shape or not all(np.all(np.isfinite(a)) for a in (x,k,cap)) or np.any(np.diff(x)<=0) or x[0]!=0 or np.any(cap<=0):raise ValueError("historical planner inputs invalid")
    if not all(math.isfinite(float(v)) and float(v)>0 for v in vals):raise ValueError("matched planner limits must be positive finite")
    if not isinstance(is_closed,bool):raise TypeError("is_closed must be bool")
    qcap=np.minimum(cap,np.minimum(float(maximum_speed_mps),np.sqrt(float(lateral_acceleration_limit_mps2)/np.maximum(np.abs(k),1e-12))))**2
    if is_closed:
        if len(x)<4 or not math.isclose(float(k[0]),float(k[-1]),abs_tol=1e-10) or not math.isclose(float(qcap[0]),float(qcap[-1]),abs_tol=1e-8):raise ValueError("closed plan requires duplicated seam and at least three unique nodes")
        n=len(x)-1;q=qcap[:-1].copy()
        for _ in range(2*n+2):
            before=q.copy()
            for i in range(n):
                j=(i+1)%n;q[j]=min(q[j],q[i]+2*float(drive_acceleration_mps2)*(x[i+1]-x[i]))
            for i in range(n-1,-1,-1):
                j=(i+1)%n;q[i]=min(q[i],q[j]+2*float(brake_deceleration_mps2)*(x[i+1]-x[i]))
            if np.max(np.abs(before-q))<1e-10:break
        else:raise RuntimeError("historical forward/backward planner did not converge")
        speed=np.sqrt(np.r_[q,q[0]])
    else:
        q=qcap.copy()
        for i in range(len(q)-1):q[i+1]=min(q[i+1],q[i]+2*float(drive_acceleration_mps2)*(x[i+1]-x[i]))
        for i in range(len(q)-2,-1,-1):q[i]=min(q[i],q[i+1]+2*float(brake_deceleration_mps2)*(x[i+1]-x[i]))
        speed=np.sqrt(q)
    accel=np.gradient(speed**2,x)/2.
    if is_closed:accel=_periodic_smooth_acceleration(x,accel,smoothing_half_window_m)
    if not np.all(np.isfinite(speed)) or not np.all(np.isfinite(accel)):raise ValueError("historical planner produced nonfinite output")
    arrays=[]
    for a in (x,speed,k,accel):
        b=np.array(a,copy=True);b.setflags(write=False);arrays.append(b)
    return HistoricalSpeedPlan(*arrays,is_closed)


def historical_forward_law(target_speed_mps,observed_speed_mps,feedforward_mps2,wheel_speed_margin=float("inf"),governor_margin=.85):
    vals=(target_speed_mps,observed_speed_mps,feedforward_mps2,governor_margin)
    if not all(math.isfinite(float(v)) for v in vals) or math.isnan(float(wheel_speed_margin)) or float(wheel_speed_margin)<0 or governor_margin<=0:raise ValueError("historical law inputs invalid")
    a=float(feedforward_mps2)+2*(float(target_speed_mps)-float(observed_speed_mps));g=min(1.,max(0.,float(wheel_speed_margin)/float(governor_margin)))
    return a,min(1.,max(0.,a/6.)),min(1.,max(0.,-a/6.))*g


def periodic_smooth_acceleration(distance_m,acceleration_mps2,half_window_m=4.):
    return _periodic_smooth_acceleration(distance_m,acceleration_mps2,half_window_m)


def predeclare_first_braking_zone(distance_m,curvature_inv_m,corner_cap_mps,*,straight_reference_mps=None,minimum_entry_station_m=0.):
    """Choose the first strict positive-station cap drop, independent of truth."""
    x=np.asarray(distance_m,dtype=float);k=np.asarray(curvature_inv_m,dtype=float);cap=np.asarray(corner_cap_mps,dtype=float)
    if x.ndim!=1 or len(x)<4 or k.shape!=x.shape or cap.shape!=x.shape or not all(np.all(np.isfinite(a)) for a in (x,k,cap)) or np.any(np.diff(x)<=0) or np.any(cap<=0) or np.any(x<0):raise ValueError("zone geometry invalid")
    if not math.isfinite(float(minimum_entry_station_m)) or minimum_entry_station_m<0:raise ValueError("minimum entry station must be nonnegative")
    ref=float(np.max(cap) if straight_reference_mps is None else straight_reference_mps)
    if not math.isfinite(ref) or ref<=0:raise ValueError("straight reference must be positive finite")
    lower=ref-max(1e-9,ref*1e-8)
    # Require a preceding geometry-defined straight section; node zero is never
    # an event. A closed path is unwrapped in setup before this function is called.
    candidates=[i for i in range(1,len(x)-1) if x[i]>minimum_entry_station_m and cap[i]<lower and cap[i+1]<lower and cap[i-1]>=lower]
    if not candidates:raise ValueError("no strict positive-station sustained geometry-defined braking event after straight reference")
    i=int(candidates[0]);start=max(0,i-2);end=min(len(x)-1,i+2)
    return {"schema":"geometry-first-braking-zone-v2","path_direction":"increasing-unwrapped-station","event_start_index":i,"event_end_index":i+1,"entry_station_m":float(x[i]),"segment_start_station_m":float(x[start]),"segment_end_station_m":float(x[end]),"indices":list(range(start,end+1)),"reference_speed_mps":ref,"entry_target_speed_mps":float(cap[i]),"curvature_inv_m_at_entry":float(k[i]),"corner_cap_mps_at_entry":float(cap[i])}


def analyze_trace(trace:Mapping[str,object],zone:Mapping[str,object])->dict:
    required=("time_s","station_m","progress_speed_mps","pedal_signed","brake_force_applied_n")
    missing=[x for x in required if x not in trace]
    if missing:raise ValueError("trace missing required accepted channels: "+", ".join(missing))
    a={x:np.asarray(trace[x],dtype=float) for x in required};n=len(a["time_s"])
    if n<2 or any(v.ndim!=1 or len(v)!=n or not np.all(np.isfinite(v)) for v in a.values()):raise ValueError("accepted trace row-parity/nonfinite")
    t,s,v=a["time_s"],a["station_m"],a["progress_speed_mps"]
    if np.any(np.diff(t)<=0) or np.any(np.diff(s)<=0):raise ValueError("accepted timestamps and progress station must increase strictly")
    entry=float(zone["entry_station_m"])
    if entry<s[0] or entry>s[-1]:raise ValueError("predeclared entry station was not crossed; extrapolation forbidden")
    matches=np.flatnonzero(np.isclose(s,entry,rtol=0.,atol=1e-10))
    if len(matches): idx=int(matches[0]);entry_speed=float(v[idx]);entry_time=float(t[idx]);bracket=(idx,idx);fraction=0.
    else:
        hi=int(np.searchsorted(s,entry,side="right"));lo=hi-1
        if lo<0 or hi>=n or not s[lo]<entry<s[hi]:raise ValueError("predeclared entry is not bracketed by accepted samples")
        fraction=float((entry-s[lo])/(s[hi]-s[lo]));entry_speed=float(v[lo]+fraction*(v[hi]-v[lo]));entry_time=float(t[lo]+fraction*(t[hi]-t[lo]));bracket=(lo,hi)
    target=float(zone["entry_target_speed_mps"]);br=a["brake_force_applied_n"];ped=a["pedal_signed"]
    start=float(zone.get("segment_start_station_m",s[0]));end=float(zone.get("segment_end_station_m",entry))
    if start<s[0] or end>s[-1] or start>=end:
        if "segment_start_station_m" not in zone and "segment_end_station_m" not in zone:start=float(s[0]);end=float(entry)
        if start<s[0] or end>s[-1] or start>=end:raise ValueError("predeclared KPI segment is outside accepted trace or invalid")
    # Clip accepted samples to the predeclared segment with boundary interpolation;
    # raw accepted rows remain untouched and row-parity validation is unchanged.
    interior=np.flatnonzero((s>start)&(s<end));stations=np.r_[start,s[interior],end]
    times=np.interp(stations,s,t);speeds=np.interp(stations,s,v);pedals=np.interp(stations,s,ped);brakes=np.interp(stations,s,br)
    decel=-np.gradient(speeds,times)
    onset=np.flatnonzero((brakes[1:]>0)&(brakes[:-1]<=0));release=np.flatnonzero((stations[1:]>=start)&(pedals[:-1]>0)&(pedals[1:]<=0))
    result={"units":{"station":"m","time":"s","speed":"m/s","deceleration":"m/s^2","brake_force":"N"},"entry_station_m":entry,"entry_target_speed_mps":target,"entry_speed_mps":entry_speed,"entry_time_s":entry_time,"entry_speed_error_mps":float(entry_speed-target),"entry_bracket_accepted_row_indices":list(bracket),"entry_bracket_fraction":fraction,"kpi_segment_start_station_m":start,"kpi_segment_end_station_m":end,"brake_onset_station_m":float(stations[int(onset[0]+1)]) if len(onset) else None,"brake_onset_time_s":float(times[int(onset[0]+1)]) if len(onset) else None,"throttle_release_station_m":float(stations[int(release[0]+1)]) if len(release) else None,"actual_signed_deceleration_min_mps2":float(np.min(decel)),"actual_signed_deceleration_max_mps2":float(np.max(decel)),"tire_capacity":{"available":False,"reason":"accepted physical capacity channel not supplied"}}
    slip=trace.get("slip_ratio")
    if slip is not None and np.asarray(slip).ndim==2 and np.asarray(slip).shape[0]==n and np.all(np.isfinite(slip)):
        slip=np.asarray(slip,dtype=float);selected=slip[(s>=start)&(s<=end)];result["slip"]={"max_abs_by_wheel":np.max(np.abs(selected),axis=0).tolist(),"unit":"dimensionless","station_window_m":[start,end]}
    else:result["slip"]={"available":False,"reason":"accepted tire slip channel unavailable or malformed"}
    power=trace.get("pack_power_w")
    if power is None:result.update(pack_energy_j=None,pack_energy_unavailable_reason="accepted plant pack power channel not supplied")
    else:
        power=np.asarray(power,dtype=float)
        if power.shape!=t.shape or not np.all(np.isfinite(power)):raise ValueError("pack power row parity invalid")
        sample_power=np.interp(stations,s,power);result["pack_energy_j"]=float(np.trapezoid(sample_power,times));result["pack_energy_station_window_m"]=[start,end]
    result["brake_force_impulse_ns"]=float(np.trapezoid(brakes,times));result["brake_force_impulse_station_window_m"]=[start,end];return result


class HistoricalForwardBackwardAdapter:
    variant_id=VARIANT_IDS[0];algorithm_id=ALGORITHM_IDS[0];controller_source_sha256=HISTORICAL_SOURCE_SHA256;reference_law=REFERENCE_LAWS[0]
    def __init__(self,plan,*,steering_driver,physical_predictor,shared_envelope,current_governor_margin=.85):
        from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy
        if not isinstance(shared_envelope,EnvelopePolicy):raise TypeError("historical adapter needs actual shared EnvelopePolicy")
        self.plan=plan;self.steering_driver=steering_driver;self.predictor=physical_predictor;self.envelope_policy=shared_envelope;self.hybrid_policy=None;self.has_pedal_dynamics=True
        self.maximum_brake_force=float(steering_driver.maximum_brake_force)
        self.brake_governor_margin=float(current_governor_margin)
        self.pedal_time_constant=float(steering_driver.pedal_time_constant)
        self.maximum_steering=float(steering_driver.maximum_steering)
        self.has_pedal_dynamics=True
        for n in ("wheelbase","maximum_steering","maximum_steering_rate","maximum_brake_force","preview_seconds","speed_gain","heading_gain","lateral_gain","steering_time_constant","pedal_travel_time","pedal_time_constant","throttle_apply_rate","throttle_release_rate","brake_apply_rate","brake_release_rate","steering_feedforward"):
            setattr(self,n,getattr(steering_driver,n))
        self.brake_governor_margin=float(current_governor_margin);self.current_governor_margin=float(current_governor_margin);self.brake_planning_context_factory=getattr(steering_driver,"brake_planning_context_factory",None);self.brake_planning_context=None;self.brake_planning_context_cache=None
        self.brake_current_context_factory=lambda p,o,pedal,state:p.brake_authority_context(state,predictor=physical_predictor);self.brake_current_predictor=physical_predictor
    def command_with_context(self,observation,pedal_position=None,plant_state=None,plant=None,planning_context_override=None,use_resolved_context=False):
        from simplified_vehicle_models.solvers.forward_driver import DriverCommand
        from simplified_vehicle_models.solvers.longitudinal_capability import path_progress_to_body_vx_dot
        if plant is None or plant_state is None:raise ValueError("historical command needs full current plant/state")
        ref,_,ff=self.plan.sample(observation.distance);kin=plant.hybrid_kinematic_context(plant_state)
        denom=1.-kin.curvature_inv_m*kin.lateral_error_m
        if not math.isfinite(denom) or denom<=0:raise ValueError("invalid Frenet denominator")
        progress=(kin.vx_mps*math.cos(kin.heading_error_rad)-kin.vy_mps*math.sin(kin.heading_error_rad))/denom
        raw=float(ff)+2.*(float(ref)-progress);target=float(np.clip(raw,-12.,12.));body=None;status="not-needed-positive-demand"
        throttle=float(np.clip(raw/6.,0.,1.));brake=0.
        if raw<0:
            try:body=float(path_progress_to_body_vx_dot(target,kin));status="valid"
            except ValueError:status="invalid"
            if body is None:raise ValueError("invalid Frenet body acceleration conversion")
            ctx=plant.brake_authority_context(plant_state,predictor=self.predictor)
            inverse=self.predictor.request_current(ctx,max(0.,-body))
            governor=float(np.clip(observation.wheel_speed_margin/self.current_governor_margin,0.,1.))
            brake=min(float(self.predictor.vehicle.maximum_pedal_force),float(inverse.pedal_force_n))*governor
        command=DriverCommand(throttle,brake,self.steering_driver._steering_command(observation))
        decision={"target_progress_acceleration_mps2":target,"reference_speed_mps":ref,"feedforward_mps2":ff,"body_vx_dot_target_mps2":body,"body_conversion_status":status,"requested_brake_force_n":brake}
        self._last_comparison_command=ComparisonCommand(self.variant_id,ref,target,body,status,throttle,brake,"current-physical-inverse-v1",command,decision)
        return command,decision
    def pedal_target(self,command):return float(np.clip(command.throttle-command.brake_force/self.maximum_brake_force,-1.,1.))
    def pedal_rate(self,position,target):
        delta=float(target)-float(position)
        if position>0.:limit=self.throttle_release_rate if delta<0. else self.throttle_apply_rate
        elif position<0.:limit=self.brake_release_rate if delta>0. else self.brake_apply_rate
        else:limit=self.throttle_apply_rate if delta>=0. else self.brake_apply_rate
        return float(np.clip(delta/self.pedal_time_constant,-limit,limit))
    def realized_command(self,position,requested):
        from simplified_vehicle_models.solvers.forward_driver import DriverCommand
        p=float(np.clip(position,-1.,1.))
        return DriverCommand(max(p,0.),max(-p,0.)*self.maximum_brake_force,requested.steering_rate)
    def command(self,observation):raise ValueError("historical comparison adapter needs plant/state context")


class CurrentPolicyComparisonAdapter:
    comparison_variant_id=VARIANT_IDS[1]
    def __init__(self,driver):
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
        if not isinstance(driver,PreviewDriver) or driver.envelope_policy is None or driver.hybrid_policy is not None:raise TypeError("current adapter requires production PreviewDriver with EnvelopePolicy")
        self.driver=driver
    def __getattr__(self,n):return getattr(self.driver,n)
    def command_with_context(self,*args,**kwargs):return self.driver.command_with_context(*args,**kwargs)


def build_current_physical_comparison_adapter(*,envelope,settings,steering_driver,physical_predictor,planning_context_factory,current_context_factory):
    from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
    from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy
    policy=EnvelopePolicy(envelope,settings)
    return PreviewDriver(plan=steering_driver.plan,wheelbase=steering_driver.wheelbase,maximum_steering=steering_driver.maximum_steering,maximum_steering_rate=steering_driver.maximum_steering_rate,maximum_brake_force=steering_driver.maximum_brake_force,preview_seconds=steering_driver.preview_seconds,speed_gain=steering_driver.speed_gain,heading_gain=steering_driver.heading_gain,lateral_gain=steering_driver.lateral_gain,steering_time_constant=steering_driver.steering_time_constant,pedal_travel_time=steering_driver.pedal_travel_time,pedal_time_constant=steering_driver.pedal_time_constant,throttle_apply_rate=steering_driver.throttle_apply_rate,throttle_release_rate=steering_driver.throttle_release_rate,brake_apply_rate=steering_driver.brake_apply_rate,brake_release_rate=steering_driver.brake_release_rate,envelope_policy=policy,brake_planning_context_factory=planning_context_factory,brake_current_context_factory=current_context_factory,brake_current_predictor=physical_predictor)


def build_hybrid_comparison_adapter(*,hybrid_policy,steering_driver):
    from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
    from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy
    if not isinstance(hybrid_policy,HybridLongitudinalPolicy):raise TypeError("hybrid adapter requires production HybridLongitudinalPolicy")
    return PreviewDriver(plan=steering_driver.plan,wheelbase=steering_driver.wheelbase,maximum_steering=steering_driver.maximum_steering,maximum_steering_rate=steering_driver.maximum_steering_rate,maximum_brake_force=steering_driver.maximum_brake_force,preview_seconds=steering_driver.preview_seconds,speed_gain=steering_driver.speed_gain,heading_gain=steering_driver.heading_gain,lateral_gain=steering_driver.lateral_gain,steering_time_constant=steering_driver.steering_time_constant,pedal_travel_time=steering_driver.pedal_travel_time,pedal_time_constant=steering_driver.pedal_time_constant,throttle_apply_rate=steering_driver.throttle_apply_rate,throttle_release_rate=steering_driver.throttle_release_rate,brake_apply_rate=steering_driver.brake_apply_rate,brake_release_rate=steering_driver.brake_release_rate,hybrid_policy=hybrid_policy,brake_current_predictor=hybrid_policy.predictor)


def _runtime_source_digest(driver,plan):
    objects=[driver.driver if isinstance(driver,CurrentPolicyComparisonAdapter) else driver]
    objects += [getattr(objects[0],"envelope_policy",None),getattr(objects[0],"hybrid_policy",None)]
    hashes={}
    for obj in objects:
        if obj is not None:
            try:
                p=Path(inspect.getsourcefile(type(obj))).resolve();hashes[str(p)]=sha256_bytes(p.read_bytes())
            except (TypeError,OSError):pass
    p={"sources":sorted(hashes.items()),"distance":np.asarray(plan.distance).tolist(),"speed":np.asarray(getattr(plan,"speed",getattr(plan,"speed_mps",[]))).tolist(),"curvature":np.asarray(getattr(plan,"curvature",getattr(plan,"curvature_inv_m",[]))).tolist()}
    return sha256_bytes(json.dumps(p,sort_keys=True,separators=(",",":"),allow_nan=False).encode())


def _plan_digest(plan):
    payload={"distance":np.asarray(getattr(plan,"distance",getattr(plan,"distance_m",[]))).tolist(),"speed":np.asarray(getattr(plan,"speed",getattr(plan,"speed_mps",[]))).tolist(),"curvature":np.asarray(getattr(plan,"curvature",getattr(plan,"curvature_inv_m",[]))).tolist(),"acceleration":np.asarray(getattr(plan,"acceleration",getattr(plan,"acceleration_mps2",[]))).tolist(),"is_closed":bool(getattr(plan,"is_closed",False))}
    return sha256_bytes(json.dumps(payload,sort_keys=True,separators=(",",":"),allow_nan=False).encode())


def runtime_driver_binding(driver,variant_id):
    """Canonical live configuration binding used to reject forged/mutated identities."""
    base,_,_,_=_binding(driver,variant_id)
    policy=getattr(base,"hybrid_policy",None) or getattr(base,"envelope_policy",None)
    provider=getattr(policy,"provider",None) or getattr(getattr(policy,"envelope",None),"provenance",None)
    predictor=getattr(base,"brake_current_predictor",None) or getattr(base,"predictor",None)
    payload={"variant_id":variant_id,"driver_type":type(base).__module__+"."+type(base).__qualname__,"driver_source_sha256":_runtime_source_digest(driver,base.plan),"plan_sha256":_plan_digest(base.plan),"policy_type":None if policy is None else type(policy).__module__+"."+type(policy).__qualname__,"policy_settings":None if getattr(policy,"settings",None) is None else __import__('dataclasses').asdict(policy.settings),"hybrid_config":None if getattr(policy,"config",None) is None else __import__('dataclasses').asdict(policy.config),"policy_envelope_sha256":None if getattr(policy,"envelope",None) is None or not all(hasattr(policy.envelope,n) for n in ("distance","curvature","corner_speed","entry_speed","deceleration","binding_index")) else sha256_bytes(b"".join(np.ascontiguousarray(getattr(policy.envelope,n)).tobytes() for n in ("distance","curvature","corner_speed","entry_speed","deceleration","binding_index"))),"provider_identity":getattr(provider,"identity",None),"policy_provider_identity":getattr(policy,"provider_identity",None),"predictor_type":None if predictor is None else type(predictor).__module__+"."+type(predictor).__qualname__,"predictor_source_hashes":getattr(predictor,"model_source_hashes",None),"actuator_rates":list(_binding(driver,variant_id)[1]),"maximum_force":_binding(driver,variant_id)[2],"governor_margin":_binding(driver,variant_id)[3],"hybrid_plan_identity":getattr(getattr(policy,"plan",None),"identity",None)}
    try:encoded=json.dumps(payload,sort_keys=True,separators=(",",":"),allow_nan=False,default=lambda x:dict(x) if isinstance(x,Mapping) else str(x))
    except (TypeError,ValueError) as exc:raise ValueError(f"driver binding is not canonicalizable: {exc}") from exc
    return sha256_bytes(encoded.encode())


def _items(x):return tuple(sorted(dict(x).items())) if isinstance(x,Mapping) else tuple(x)


def _binding(driver,variant):
    if variant==VARIANT_IDS[0]:
        if not isinstance(driver,HistoricalForwardBackwardAdapter):raise ValueError("original adapter type mismatch")
        base=driver
    elif variant==VARIANT_IDS[1]:
        if not isinstance(driver,CurrentPolicyComparisonAdapter):raise ValueError("current adapter type mismatch")
        base=driver.driver
    else:
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
        from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy
        if not isinstance(driver,PreviewDriver) or not isinstance(driver.hybrid_policy,HybridLongitudinalPolicy):raise ValueError("hybrid adapter type mismatch")
        base=driver
    rates=tuple(getattr(base,n,None) for n in ("throttle_apply_rate","throttle_release_rate","brake_apply_rate","brake_release_rate"))
    return base,rates,float(base.maximum_brake_force),float(base.brake_governor_margin)


def shared_comparison_run(solve_forward,*,plant,initial_state,initial_pedal,driver,run_config_identity,variant_identity,solver_settings,policy_sink_factory=None):
    state=np.asarray(initial_state,dtype=float)
    if state.shape!=(int(plant.state_size),) or not np.all(np.isfinite(state)) or sha256_bytes(state.tobytes())!=run_config_identity.initial_full_state_sha256:raise ValueError("initial complete state differs from frozen identity")
    if variant_identity.run_config!=run_config_identity:raise ValueError("variant run config mismatch")
    i=VARIANT_IDS.index(variant_identity.variant_id)
    base,rates,maxforce,governor=_binding(driver,variant_identity.variant_id)
    if (variant_identity.algorithm_id,variant_identity.reference_law)!=(ALGORITHM_IDS[i],REFERENCE_LAWS[i]):raise ValueError("algorithm/reference identity mismatch")
    source=HISTORICAL_SOURCE_SHA256 if i==0 else _runtime_source_digest(driver,base.plan)
    if variant_identity.controller_source_sha256!=source:raise ValueError("controller source/plan digest mismatch")
    if not variant_identity.runtime_binding_sha256 or variant_identity.runtime_binding_sha256!=runtime_driver_binding(driver,variant_identity.variant_id):raise ValueError("actual driver/policy/provider/plan binding differs from frozen identity")
    expected_provider=run_config_identity.provider_identity
    live_policy=getattr(base,"hybrid_policy",None) or getattr(base,"envelope_policy",None)
    live_envelope=getattr(live_policy,"envelope",None)
    if variant_identity.variant_id==VARIANT_IDS[2] and getattr(live_policy,"provider_identity",None)!=expected_provider:raise ValueError("hybrid policy provider differs from frozen provider identity")
    if variant_identity.variant_id==VARIANT_IDS[1] and live_envelope is None:raise ValueError("current physical policy has no actual envelope")
    if variant_identity.mapping_id!="current-physical-inverse-v1" or variant_identity.current_source_closure_sha256!=source_manifest_digest(dict(run_config_identity.source_hashes)):raise ValueError("mapping/source closure identity mismatch")
    if rates!=run_config_identity.foot_rates or maxforce!=run_config_identity.maximum_pedal_force_n or governor!=run_config_identity.governor_margin:raise ValueError("driver actuator rates/force/governor differ from frozen identity")
    expected_solver=dict(run_config_identity.solver);actual_solver=dict(solver_settings);expected_pedal=float(expected_solver.pop("initial_pedal",run_config_identity.pedal_and_steering_state[0]));actual_pedal=float(actual_solver.pop("initial_pedal",initial_pedal))
    if _items(actual_solver)!=_items(expected_solver):raise ValueError("solver settings differ from frozen identity")
    if initial_pedal!=expected_pedal or actual_pedal!=expected_pedal:raise ValueError("initial pedal differs from frozen identity")
    kwargs=dict(solver_settings);kwargs["initial_pedal"]=initial_pedal
    if policy_sink_factory:kwargs["policy_sink"]=policy_sink_factory(variant_identity.variant_id)
    elif "policy_sink" not in kwargs:kwargs["policy_sink"]=lambda sample:None
    result=solve_forward(plant,driver,state.copy(),**kwargs)
    return {"identity":variant_identity,"result":result,"integration_performed":True}


def run_primary_comparison(*,solve_forward,plant,initial_state,initial_pedal,run_config_identity,variants,solver_settings,policy_sink_factory=None):
    ids=[v["identity"] for v in variants];validate_primary_identities(ids)
    if {v["identity"].variant_id for v in variants}!=set(VARIANT_IDS):raise ValueError("exactly one driver per primary required")
    # Validate all bindings before first integration.
    for v in variants:
        identity=v["identity"];base,rates,mx,gov=_binding(v["driver"],identity.variant_id);i=VARIANT_IDS.index(identity.variant_id)
        source=HISTORICAL_SOURCE_SHA256 if i==0 else _runtime_source_digest(v["driver"],base.plan)
        if (identity.algorithm_id,identity.reference_law,identity.controller_source_sha256)!=(ALGORITHM_IDS[i],REFERENCE_LAWS[i],source):raise ValueError("driver differs from declared frozen algorithm/source identity")
        if not identity.runtime_binding_sha256 or identity.runtime_binding_sha256!=runtime_driver_binding(v["driver"],identity.variant_id):raise ValueError("actual driver/policy/provider/plan binding differs from frozen identity")
        if rates!=run_config_identity.foot_rates or mx!=run_config_identity.maximum_pedal_force_n or gov!=run_config_identity.governor_margin:raise ValueError("driver config differs from shared identity")
    return [shared_comparison_run(solve_forward,plant=plant,initial_state=initial_state,initial_pedal=initial_pedal,driver=v["driver"],run_config_identity=run_config_identity,variant_identity=v["identity"],solver_settings=solver_settings,policy_sink_factory=policy_sink_factory) for v in variants]


def source_manifest_digest(hashes):return sha256_bytes(json.dumps(sorted(hashes.items()),separators=(",",":"),ensure_ascii=False).encode())


def source_closure(track,*,root=ROOT):
    import sys
    for p in (str(root/"src"),str(root)):
        if p not in sys.path:sys.path.insert(0,p)
    from tools.run.run_iem24_forward_driver import model_inputs,source_files
    named=model_inputs("14dof-simple",track);runner=source_files("14dof-simple",track,longitudinal_mode="hybrid-drive-brake-v1")
    paths=list(runner)+list(named.values())+list((root/"src/simplified_vehicle_models").rglob("*.py"))+[root/"tools/run/compare_virtual_drivers.py"]
    return freeze_files(paths,root=root)


def freeze_files(paths,*,root=ROOT):
    base=root.resolve();out={}
    for path in sorted(set(Path(p) for p in paths),key=lambda p:p.as_posix()):
        full=(base/path).resolve() if not path.is_absolute() else path.resolve()
        if base not in full.parents or not full.is_file():raise ValueError(f"source not repository-rooted: {path}")
        out[full.relative_to(base).as_posix()]=sha256_bytes(full.read_bytes())
    if not out:raise ValueError("empty source freeze")
    return out


def analyze_record(record,zone,*,plant):
    """Analyze accepted StepSample rows using real plant Frenet/body kinematics."""
    samples=list(record)
    if not samples:raise ValueError("accepted record is empty")
    if any(sample.pedal is None for sample in samples):raise ValueError("accepted foot/pedal telemetry unavailable; comparison KPI is gated")
    times=[];station=[];speed=[];body_speed=[];pedal=[];brake=[];power=[]
    for sample in samples:
        state=np.asarray(sample.state,dtype=float)
        observation=plant.observe(state);kin=plant.hybrid_kinematic_context(state)
        denom=1.-kin.curvature_inv_m*kin.lateral_error_m
        # Retain both channels: this body-coordinate speed observation is useful
        # for provenance, but progress-speed KPI is reconstructed independently.
        # Progress speed is a kinematic rate, not a body-acceleration conversion.
        # Keep vy_dot_valid available to other callers for acceleration transforms,
        # but do not gate this exact Frenet speed mapping on that assumption.
        kinematics=(kin.vx_mps,kin.vy_mps,kin.heading_error_rad,kin.curvature_inv_m,kin.lateral_error_m)
        if not all(math.isfinite(float(value)) for value in kinematics) or not math.isfinite(denom) or denom<.5:
            raise ValueError("accepted Frenet kinematic context invalid for progress-speed KPI")
        calculated_speed=float((kin.vx_mps*math.cos(kin.heading_error_rad)-kin.vy_mps*math.sin(kin.heading_error_rad))/denom)
        rates=np.asarray(sample.rates,dtype=float)
        if rates.ndim!=1 or len(rates)<1 or not math.isfinite(float(rates[0])) or rates[0]<=0:
            raise ValueError("accepted observed progress rate invalid for progress-speed KPI")
        if not math.isclose(calculated_speed,float(rates[0]),rel_tol=1e-8,abs_tol=1e-8):
            raise ValueError("Frenet progress-speed formula disagrees with accepted observed progress rate")
        times.append(float(sample.time));station.append(float(observation.distance))
        body_speed.append(float(state[3]));speed.append(calculated_speed)
        pedal.append(float(sample.pedal));brake.append(float(sample.command.brake_force))
        power.append(float(getattr(getattr(sample,"applied_powertrain_diagnostics",None),"pack_power_w",np.nan)))
    trace={"time_s":times,"station_m":station,"progress_speed_mps":speed,"body_vx_mps":body_speed,"pedal_signed":pedal,"brake_force_applied_n":brake}
    if np.all(np.isfinite(power)):trace["pack_power_w"]=power
    return analyze_trace(trace,zone)


def _jsonable(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if hasattr(value,"to_dict"):return _jsonable(value.to_dict())
    if hasattr(value,"__dataclass_fields__"):return {k:_jsonable(getattr(value,k)) for k in value.__dataclass_fields__}
    if isinstance(value,Mapping):return {str(k):_jsonable(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [_jsonable(v) for v in value]
    if value is None or isinstance(value,(str,int,float,bool)):return value
    return repr(value)


def _atomic_json(path,payload):
    path=Path(path);tmp=path.with_name(path.name+".tmp")
    if tmp.exists():raise FileExistsError(tmp)
    tmp.write_text(json.dumps(_jsonable(payload),sort_keys=True,indent=2,allow_nan=False)+"\n")
    tmp.replace(path)


def _sample_payload(sample):
    return {"index":int(sample.index),"time_s":float(sample.time),"state":np.asarray(sample.state).tolist(),"rates":np.asarray(sample.rates).tolist(),"requested_command":_jsonable(sample.requested_command),"mapped_command":_jsonable(sample.mapped_command),"applied_command":_jsonable(sample.command),"pedal":sample.pedal,"policy_diagnostics":_jsonable(sample.policy_diagnostics),"applied_powertrain_diagnostics":_jsonable(sample.applied_powertrain_diagnostics)}


def persist_return_before_analysis(directory,*,identity,config,solver_settings,result,samples,plant,zone,analyzer=analyze_record):
    """Durably persist raw solver/accepted samples before invoking KPI analysis."""
    directory=Path(directory);rows=list(samples)
    if not rows:raise ValueError("solver returned zero accepted observer rows")
    _atomic_json(directory/"raw-return.json",{"identity":identity,"config":config,"solver_settings":solver_settings,"variant_result":result,"accepted_samples":[_sample_payload(x) for x in rows]})
    try:
        kpis=analyzer(rows,zone,plant=plant)
        _atomic_json(directory/"analysis.json",{"identity":identity,"kpis":kpis})
        return kpis
    except BaseException as exc:
        _atomic_json(directory/"analysis-failure.json",{"identity":identity,"exception_type":type(exc).__name__,"exception":str(exc),"raw_checkpoint":"raw-return.json"})
        raise


def _canonical_packet_bytes(packet):
    return json.dumps(packet,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")


def validate_rebuilt_packet(packet,rebuilt):
    """Require exact equality with a freshly prepared execution manifest."""
    if not isinstance(packet,dict) or not isinstance(rebuilt,dict):raise ValueError("prepared packet and rebuilt manifest must be objects")
    missing=sorted(set(rebuilt)-set(packet));extra=sorted(set(packet)-set(rebuilt))
    if missing or extra:raise ValueError(f"prepared packet fields differ from rebuilt manifest (missing={missing}, extra={extra})")
    if _canonical_packet_bytes(packet)!=_canonical_packet_bytes(rebuilt):raise ValueError("prepared execution-authoritative manifest differs from rebuilt preparation")
    return True


def execute_prepared_screen(args,track,out,*,solver_override=None):
    """Execute only a byte-bound prepared packet; never called in preparation tests."""
    manifest_path=args.prepared_manifest if args.prepared_manifest.is_absolute() else ROOT/args.prepared_manifest
    artifact_dir=args.artifact_dir if args.artifact_dir.is_absolute() else ROOT/args.artifact_dir
    if not manifest_path.is_file(): raise SystemExit("prepared manifest does not exist")
    if artifact_dir.exists(): raise SystemExit("artifact directory already exists; refusing overwrite")
    packet=json.loads(manifest_path.read_text())
    if packet.get("schema")!="task7-comparison-source-freeze-v3" or packet.get("status")!="prepared-no-integration": raise SystemExit("manifest is not a prepared v3 packet")
    if packet.get("track_sha256")!=sha256_bytes(track.read_bytes()): raise SystemExit("prepared track bytes differ")
    current=source_closure(track)
    if current!=packet.get("source_hashes") or source_manifest_digest(current)!=packet.get("current_source_closure_sha256"): raise SystemExit("prepared source closure is stale")
    # Rebuild the actual factory from identical source/configuration bytes, then
    # verify every frozen identity before calling the production solver.
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        main(["--prepare-only","--track",str(track),"--manifest-dir",td])
        rebuilt=json.loads((Path(td)/"source-freeze.json").read_text())
    try:validate_rebuilt_packet(packet,rebuilt)
    except ValueError as exc:raise SystemExit(str(exc)) from exc
    prepared=_LAST_PREPARED_FACTORY
    if prepared is None: raise SystemExit("prepared factory unavailable")
    zone=rebuilt["predeclared_zone"]
    settings=dict(prepared["solver_settings"])
    settings.pop("initial_pedal",None)
    artifact_dir.mkdir(parents=True,exist_ok=False)
    from simplified_vehicle_models.solvers.forward_driver import solve_forward
    solver_fn=solve_forward if solver_override is None else solver_override
    results=[]
    for index,(driver,identity) in enumerate(zip(prepared["drivers"],prepared["identities"])):
        rows=[];variant_dir=artifact_dir/f"variant-{index}";variant_dir.mkdir(parents=False,exist_ok=False)
        def sink(sample): rows.append(sample)
        plant_type=type(prepared["plant"]);plant=plant_type(prepared["model"],prepared["plan"])
        _atomic_json(variant_dir/"checkpoint.json",{"identity":identity,"config":prepared["config"],"solver_settings":settings,"status":"running"})
        try:
            run=shared_comparison_run(solver_fn,plant=plant,initial_state=prepared["initial"],initial_pedal=prepared["initial_pedal"],driver=driver,run_config_identity=prepared["config"],variant_identity=identity,solver_settings=settings,policy_sink_factory=lambda _v:sink)
            if not rows:raise ValueError("solver returned zero accepted observer rows")
        except BaseException as exc:
            _atomic_json(variant_dir/"failure.json",{"identity":identity,"config":prepared["config"],"solver_settings":settings,"exception_type":type(exc).__name__,"exception":str(exc),"accepted_samples":rows})
            results.append({"variant_id":identity.variant_id,"accepted_rows":len(rows),"status":"failed","failure_type":type(exc).__name__})
            _atomic_json(artifact_dir/"screen-result.json",{"schema":"task7-comparison-screen-result-v1","zone":zone,"variants":results})
            raise
        try:
            persist_return_before_analysis(variant_dir,identity=identity,config=prepared["config"],solver_settings=settings,result=run["result"],samples=rows,plant=plant,zone=zone)
            results.append({"variant_id":identity.variant_id,"accepted_rows":len(rows),"status":"complete"})
        except BaseException as exc:
            results.append({"variant_id":identity.variant_id,"accepted_rows":len(rows),"status":"analysis-failed","failure_type":type(exc).__name__})
            _atomic_json(artifact_dir/"screen-result.json",{"schema":"task7-comparison-screen-result-v1","zone":zone,"variants":results})
            raise
        _atomic_json(artifact_dir/"screen-result.json",{"schema":"task7-comparison-screen-result-v1","zone":zone,"variants":results})
    return 0


def build_parser():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--track",type=Path,default=Path("models/tracks/fsc_may_2026_centerline.xml"));p.add_argument("--manifest-dir",type=Path,default=Path(".superpowers/task7comparison/frozen"));p.add_argument("--prepare-only",action="store_true",default=True);p.add_argument("--screen",action="store_true");p.add_argument("--prepared-manifest",type=Path);p.add_argument("--artifact-dir",type=Path);return p


def main(argv=None):
    args=build_parser().parse_args(argv);track=args.track if args.track.is_absolute() else ROOT/args.track;out=args.manifest_dir if args.manifest_dir.is_absolute() else ROOT/args.manifest_dir
    if args.screen:
        if args.prepared_manifest is None or args.artifact_dir is None:
            raise SystemExit("--screen requires --prepared-manifest and --artifact-dir; no implicit execution packet")
        return execute_prepared_screen(args,track,out)
    if args.prepared_manifest is not None or args.artifact_dir is not None:
        raise SystemExit("execution-only options require --screen")
    # Real setup-only preparation compiles the same model/track/hybrid builder,
    # but performs no solve_forward call and writes actual geometry/caps zone.
    import sys
    for p in (str(ROOT/"src"),str(ROOT)):
        if p not in sys.path:sys.path.insert(0,p)
    from tools.run import run_iem24_forward_driver as runner
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.tracks.profile import load_track
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar,normal_loads
    from simplified_vehicle_models.tires.unitire_simple import loaded_radius
    from simplified_vehicle_models.powertrain.energy_storage import StorageState
    from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext
    from simplified_vehicle_models.solvers.hybrid_setup import build_hybrid_setup, HYBRID_EXECUTION_ALLOCATOR_ID
    from simplified_vehicle_models.solvers.braking_envelope import build_braking_envelope
    from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
    from simplified_vehicle_models.solvers.forward_driver import PreviewDriver,SpeedPlan
    from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy,EnvelopeDriverSettings
    from types import SimpleNamespace
    model=compile_formulation(str(runner.VERTICAL_FORMULATION)).model;control=getattr(model,"planar",model);track_obj=load_track(track)
    # Bounded comparison preparation: 48 geometry nodes and a common rolling
    # approach speed selected below the actual start cap but above the first
    # predeclared entry target. This is setup-only, never a production default.
    node_count=48;distance=np.linspace(0.,track_obj.lap_length,node_count);_,_,_,curv=evaluate_path(track_obj.path_coordinates,distance);soc=float(control.drivetrain.powertrain.energy_storage.initial_state_of_charge);state=StorageState(soc);initial_speed=29.0;rate=1./.175
    pc=PedalPolicyContext(rate,rate,.02,control.maximum_pedal_force,.85,rate,rate);rloads=normal_loads(point_mass_view_of_planar(control),initial_speed,0.,0.);radii=loaded_radius(control.tire,rloads)
    sources=runner.source_files("14dof-simple",track,longitudinal_mode="hybrid-drive-brake-v1");source_hashes={(Path(p) if Path(p).is_absolute() else ROOT/Path(p)).relative_to(ROOT).as_posix():sha256_bytes((Path(p) if Path(p).is_absolute() else ROOT/Path(p)).read_bytes()) for p in sources}
    setup=build_hybrid_setup(control,distance_m=distance,curvature_inv_m=curv,maximum_speed_mps=40.,initial_speed_mps=initial_speed,storage_soc=soc,wheel_radii_m=tuple(map(float,radii)),model_source_hashes=source_hashes,maximum_pedal_force_n=control.maximum_pedal_force,pedal_context=pc,lateral_margin=.9,is_closed=track_obj.is_closed,execution_allocator_identity=HYBRID_EXECUTION_ALLOCATOR_ID)
    zone=predeclare_first_braking_zone(setup.plan.distance_m,setup.plan.curvature_inv_m,setup.corner_speed_caps_mps,straight_reference_mps=float(np.max(setup.corner_speed_caps_mps)))
    closure=source_closure(track);manifest={"schema":"task7-comparison-source-freeze-v3","source_hashes":closure,"current_source_closure_sha256":source_manifest_digest(closure),"historical_commit":HISTORICAL_COMMIT,"historical_source_sha256":HISTORICAL_SOURCE_SHA256,"historical_source_actual_sha256":sha256_bytes((ROOT/".superpowers/sdd/2026-11-05-original-driver-comparison/history/b303889_forward_driver.py").read_bytes()),"variant_ids":list(VARIANT_IDS),"status":"prepared-no-integration","preparation_scope":"factory-only-no-execution-or-telemetry-acceptance","screen_contract":{"requires_explicit_prepared_manifest_and_unique_artifact_dir":True,"duration_s":8.0,"wall_seconds":240.0,"maximum_wall_seconds":240.0,"accepted_samples":"StepSample observer after successful solve_forward return only; timeout does not claim partial accepted-row preservation","per_variant_fresh_plant":True,"integration_performed":False},"setup_identity":setup.setup_identity,"provider_identity":setup.provider.identity,"track_sha256":sha256_bytes(track.read_bytes()),"distance_m":setup.plan.distance_m.tolist(),"curvature_inv_m":setup.plan.curvature_inv_m.tolist(),"corner_speed_caps_mps":setup.plan.corner_speed_cap_mps.tolist(),"physical_corner_speed_caps_mps":setup.physical_corner_speed_caps_mps.tolist(),"capability_domain_speed_caps_mps":setup.capability_domain_speed_caps_mps.tolist(),"braking_speed_bound_mps":setup.plan.braking_speed_bound_mps.tolist(),"predeclared_zone":zone,"zone_sha256":sha256_bytes(json.dumps(zone,sort_keys=True,separators=(",",":"),allow_nan=False).encode()),"zone_kpi_contract":{"entry_target_speed_mps":zone["entry_target_speed_mps"],"entry_target_speed_source":"geometry-first shared corner cap at positive-station event"},"initial_speed_mps":initial_speed,"initial_soc":soc,"initial_state_layout_size":38,"foot_rates":[rate]*4,"pedal_time_constant_s":.02,"maximum_pedal_force_n":control.maximum_pedal_force,"governor_margin":.85,"relay_max_extra_forecasts":setup.policy.config.max_extra_forecasts,"relay_bracket_resolution":setup.policy.config.bracket_resolution,"relay_max_total_transitions":(3+setup.policy.config.max_extra_forecasts)*setup.policy.config.slabs,"relay_horizon_s":setup.policy.config.horizon_s,"relay_slabs":setup.policy.config.slabs,"model_semantic_sha256":setup.setup_identity,"same_driver_factories_constructed":False}
    hist_plan=build_historical_speed_plan(setup.plan.distance_m,setup.plan.curvature_inv_m,setup.corner_speed_caps_mps,drive_acceleration_mps2=2.5,brake_deceleration_mps2=3.,maximum_speed_mps=40.,lateral_acceleration_limit_mps2=9.,is_closed=track_obj.is_closed)
    from tools.run.run_iem24_forward_driver import model_spec,plant_class_for
    from types import SimpleNamespace
    def sample_for_plant(station):
        sample=setup.plan.sample(station)
        return sample.speed_mps,sample.curvature_inv_m,sample.acceleration_mps2
    driver_plan=SimpleNamespace(distance=setup.plan.distance_m,speed=setup.plan.speed_mps,curvature=setup.plan.curvature_inv_m,is_closed=setup.plan.is_closed,sample=sample_for_plant)
    historical_driver_plan=SpeedPlan(hist_plan.distance,hist_plan.speed,hist_plan.curvature,is_closed=hist_plan.is_closed)
    plant=plant_class_for(model_spec("14dof-simple"),model)(model,driver_plan)
    initial=plant.initial_state(initial_speed)
    pedal_context=pc
    predictor=setup.brake_predictor
    current_context_factory=lambda p,o,pedal,state:p.brake_authority_context(state,predictor=predictor)
    planning_context_factory=lambda p,o,pedal,state:p.brake_planning_context(state,pedal,predictor=predictor)
    authority=plant.brake_authority_context(initial,predictor=predictor)
    physical_plan=SpeedPlan(setup.plan.distance_m,setup.plan.speed_mps,setup.plan.curvature_inv_m,is_closed=setup.plan.is_closed)
    from simplified_vehicle_models.solvers.envelope_driver import PhysicalBrakePlanningContext
    physical_context=PhysicalBrakePlanningContext.from_predictor(predictor,authority,pedal_context,knot_count=8)
    physical_context=replace(physical_context,current_authority_context=authority,current_predictor=predictor)
    pointmass=point_mass_view_of_planar(control)
    envelope=build_braking_envelope(setup.plan.distance_m,setup.plan.curvature_inv_m,pointmass,maximum_speed=40.,lateral_margin=.9,deceleration_margin=.60,speed_query_points=9,storage_state=state,is_closed=track_obj.is_closed)
    settings=EnvelopeDriverSettings(speed_gain=2.,mode="physical-reachable-v1",planning_transition_time_s=None,forecast_horizon_s=3.)
    steer=PreviewDriver(plan=physical_plan,wheelbase=control.wheelbase,maximum_steering=control.steering_maximum_angle,maximum_steering_rate=control.steering_maximum_rate,maximum_brake_force=control.maximum_pedal_force,preview_seconds=.15,speed_gain=2.,heading_gain=1.5,lateral_gain=1.5,pedal_travel_time=.175,pedal_time_constant=.02,throttle_apply_rate=rate,throttle_release_rate=rate,brake_apply_rate=rate,brake_release_rate=rate,brake_governor_margin=.85)
    current_driver=build_current_physical_comparison_adapter(envelope=envelope,settings=settings,steering_driver=steer,physical_predictor=predictor,planning_context_factory=planning_context_factory,current_context_factory=current_context_factory)
    historical_driver=HistoricalForwardBackwardAdapter(hist_plan,steering_driver=steer,physical_predictor=predictor,shared_envelope=EnvelopePolicy(envelope,settings))
    hybrid_steer=replace(steer,plan=physical_plan)
    hybrid_driver=build_hybrid_comparison_adapter(hybrid_policy=setup.policy,steering_driver=hybrid_steer)
    hybrid_driver.hybrid_policy.provider_identity=setup.provider.identity
    variants=[{"identity":None,"driver":historical_driver},{"identity":None,"driver":CurrentPolicyComparisonAdapter(current_driver)},{"identity":None,"driver":hybrid_driver}]
    start_cap=float(setup.corner_speed_caps_mps[0])
    if np.asarray(initial).shape!=(38,) or initial_speed>start_cap+1e-10:
        raise ValueError(f"prepared rolling start invalid: requested={initial_speed:.9g} m/s start_cap={start_cap:.9g} m/s state_size={np.asarray(initial).size}")
    observed=plant.observe(initial)
    command_checks=[]
    for station in (0.,float(zone["entry_station_m"])):
        state_at=plant.initial_state(initial_speed)
        state_at=np.array(state_at,copy=True);state_at[0]=station
        observation=plant.observe(state_at);force_context=plant.brake_authority_context(state_at,predictor=predictor)
        planning=plant.brake_planning_context(state_at,pedal_context,predictor=predictor)
        for variant in variants:
            driver=variant["driver"]
            kwargs={"observation":observation,"pedal_position":0.,"plant_state":state_at,"plant":plant}
            if isinstance(driver,CurrentPolicyComparisonAdapter):
                cmd,diag=driver.command_with_context(**kwargs,planning_context_override=replace(planning,current_authority_context=force_context,current_predictor=predictor))
            elif isinstance(driver,HistoricalForwardBackwardAdapter):
                cmd,diag=driver.command_with_context(**kwargs)
            else:
                cmd,diag=driver.command_with_context(**kwargs)
            if not all(math.isfinite(float(x)) for x in (cmd.throttle,cmd.brake_force,cmd.steering_rate)):raise ValueError(f"{variant['driver'].variant_id if hasattr(variant['driver'],'variant_id') else variant['driver'].comparison_variant_id} returned nonfinite preparation command at station {station}")
            command_checks.append({"variant_id":getattr(driver,"variant_id",getattr(driver,"comparison_variant_id",VARIANT_IDS[2])),"station_m":station,"state_size":int(state_at.size),"command":{"throttle":float(cmd.throttle),"brake_force_n":float(cmd.brake_force),"steering_rate_rps":float(cmd.steering_rate)},"context_provider_identity":setup.provider.identity})
    solver_identity=(("method","Radau"),("duration",8.),("maximum_step",.02),("rtol",1.e-6),("atol",1.e-8),("wall_seconds",240.),("maximum_wall_seconds",240.),("initial_pedal",0.))
    config=RunConfigIdentity(model_semantic_sha256=setup.setup_identity,source_hashes=tuple(sorted(closure.items())),track_sha256=sha256_bytes(track.read_bytes()),calibration_hashes=tuple(sorted((k,v) for k,v in source_hashes.items() if "calibration" in k or k.endswith(".csv"))),initial_full_state_sha256=sha256_bytes(np.asarray(initial,dtype=float).tobytes()),initial_soc=soc,pedal_and_steering_state=(0.,float(initial[25])),solver=solver_identity,lateral_margin=.9,brake_margin=.60,provider_identity=setup.provider.identity,allocator_identity=HYBRID_EXECUTION_ALLOCATOR_ID,foot_rates=(rate,rate,rate,rate),pedal_lag_s=.02,maximum_pedal_force_n=float(control.maximum_pedal_force),governor_margin=.85,corner_caps_sha256=sha256_bytes(np.ascontiguousarray(setup.corner_speed_caps_mps).tobytes()))
    identities=[]
    for i,item in enumerate(variants):
        driver=item["driver"];base,_,_,_=_binding(driver,VARIANT_IDS[i]);controller=HISTORICAL_SOURCE_SHA256 if i==0 else _runtime_source_digest(driver,base.plan)
        ident=ComparisonIdentity(VARIANT_IDS[i],ALGORITHM_IDS[i],REFERENCE_LAWS[i],"current-physical-inverse-v1",controller,source_manifest_digest(dict(config.source_hashes)),config,runtime_driver_binding(driver,VARIANT_IDS[i]))
        item["identity"]=ident;identities.append(ident)
    validate_primary_identities(identities)
    global _LAST_PREPARED_FACTORY
    _LAST_PREPARED_FACTORY={"model":model,"plan":driver_plan,"plant":plant,"initial":np.array(initial,copy=True),"initial_pedal":0.,"solver_settings":dict(solver_identity),"config":config,"drivers":[x["driver"] for x in variants],"identities":identities,"variants":variants}
    manifest.update({"initial_state_sha256":sha256_bytes(np.asarray(initial,dtype=float).tobytes()),"initial_state_size":int(np.asarray(initial).size),"initial_pedal":0.0,"zone_station_offset_m":0.0,"geometry_node_count":int(len(distance)),"same_driver_factories_constructed":len(variants)==3 and len(identities)==3,"driver_command_preparation_checks":command_checks,"run_config_identity":{field:getattr(config,field) for field in config.__dataclass_fields__ if field!="source_hashes"},"run_config_source_hashes":list(config.source_hashes),"variant_runtime_binding_sha256":[v.runtime_binding_sha256 for v in identities],"solver_identity":dict(solver_identity),"execution_telemetry_status":"not-run-awaiting-parent-review"})
    manifest["historical_plan_speed_mps"]=hist_plan.speed.tolist()
    manifest["execution_variants"]=[{"variant_id":i.variant_id,"algorithm_id":i.algorithm_id,"reference_law":i.reference_law,"mapping_id":i.mapping_id,"controller_source_sha256":i.controller_source_sha256,"current_source_closure_sha256":i.current_source_closure_sha256,"runtime_binding_sha256":i.runtime_binding_sha256} for i in identities]
    manifest["solver_identity_canonical"]={k:v for k,v in solver_identity}
    manifest["plan_identity"]={"distance_sha256":sha256_bytes(np.ascontiguousarray(setup.plan.distance_m).tobytes()),"speed_sha256":sha256_bytes(np.ascontiguousarray(setup.plan.speed_mps).tobytes()),"curvature_sha256":sha256_bytes(np.ascontiguousarray(setup.plan.curvature_inv_m).tobytes()),"acceleration_sha256":sha256_bytes(np.ascontiguousarray(setup.plan.acceleration_mps2).tobytes()),"is_closed":bool(setup.plan.is_closed)}
    out.mkdir(parents=True,exist_ok=True);(out/"source-freeze.json").write_text(json.dumps(manifest,sort_keys=True,indent=2)+"\n")
    print(json.dumps({"manifest":str(out/"source-freeze.json"),"source_count":len(closure),"source_closure_sha256":manifest["current_source_closure_sha256"],"setup_identity":setup.setup_identity,"provider_identity":setup.provider.identity,"zone":zone,"integration_performed":False},sort_keys=True));return 0


if __name__=="__main__":raise SystemExit(main())
