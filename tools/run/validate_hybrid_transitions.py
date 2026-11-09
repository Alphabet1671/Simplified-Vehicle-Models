"""Bounded actual 38-state hybrid transition runner.

Runs selected prospective transitions through the real solve_forward -> PreviewDriver
-> HybridLongitudinalPolicy -> DriverCommand -> signed pedal -> plant path. Evidence
is bounded and not a feasibility certificate.
"""
from __future__ import annotations
import argparse, hashlib, json, time, os, tempfile
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext
from simplified_vehicle_models.solvers.forward_driver import DriverObservation, ForwardIntegrationError, ForwardResult, StepSample, PreviewDriver, SpeedPlan, solve_forward
from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy, HybridPolicyConfig
from simplified_vehicle_models.solvers.hybrid_speed_plan import HybridSpeedPlan, build_hybrid_speed_plan
from simplified_vehicle_models.solvers.longitudinal_capability import validate_execution_allocator
from simplified_vehicle_models.solvers.longitudinal_capability import CurrentVehicleCapability, CapabilityQuery
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant, SOC_INDEX
from tools.run.validate_hybrid_driver import FORMULATION, ROOT, CORE_SOURCE_NAMES

VALIDATION_RESOURCE_CONTRACT_VERSION='task5-validator-resource-v2-240s'
MAX_PERMITTED_WALL_SECONDS=240.
CASE_IDS=("20-to-10-p-plus1","20-to-10-p-zero","20-to-10-p-minus-half","15-to-5-p-plus1","15-to-5-p-zero","15-to-5-p-minus-half","left-curvature","right-curvature","prospective-drive-brake","stop-neutral","stop-plus-point3","periodic-seam","tv-yaw-reject")
def _sha(paths): return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}
def _finite_positive(name,value):
    value=float(value)
    if not np.isfinite(value) or value<=0: raise ValueError(f"{name} must be finite and positive")
    return value
def validate_runner_config(*,duration,wall_seconds):
    duration=_finite_positive("duration",duration); wall_seconds=_finite_positive("wall_seconds",wall_seconds)
    if duration>12 or wall_seconds>MAX_PERMITTED_WALL_SECONDS: raise ValueError(f"duration/wall bound exceeded (12 s, {MAX_PERMITTED_WALL_SECONDS:g} wall s)")
    return duration,wall_seconds
def trace_metrics(*,time_s,state,station_m,speed_mps,cap_mps,contact_min_n,event_station_m,target_speed_mps,candidate_count,rollout_samples,solver_status,source_hashes_match=True,require_rollout=False,contact_metric_available=True):
    t=np.asarray(time_s,float); x=np.asarray(state,float); s=np.asarray(station_m,float); v=np.asarray(speed_mps,float); cap=np.asarray(cap_mps,float); c=np.asarray(candidate_count,int); slabs=np.asarray(rollout_samples,int); n=len(t)
    if n<2 or x.ndim!=2 or x.shape[0]!=n or any(a.shape!=(n,) for a in (s,v,cap,c,slabs)): raise ValueError("trace arrays have inconsistent dimensions or no arrival samples")
    if any(not np.all(np.isfinite(a)) for a in (t,x,s,v,cap)): raise ValueError("trace contains nonfinite time/state/path/speed/cap")
    if contact_metric_available and (not np.isfinite(contact_min_n) or contact_min_n<0): raise ValueError("invalid contact metric")
    if np.any(c<0) or np.any(slabs<0): raise ValueError("negative rollout telemetry")
    if solver_status not in ("finished","terminal_event"): raise ValueError(f"solver did not reach an accepted end: {solver_status}")
    post=s>=event_station_m; arrival=np.flatnonzero(post&(v<=target_speed_mps+.5))
    if not np.any(post) or not len(arrival): raise ValueError("event arrival not observed")
    overshoot=np.maximum(0.,v[post]-cap[post])
    if not np.all(np.isfinite(overshoot)): raise ValueError("nonfinite post-event overspeed")
    mc=int(c.max(initial=0)); ms=int(slabs.max(initial=0)); mo=float(np.max(overshoot))
    return {"passed":bool(mo<=.5 and mc<=3 and ms<=180 and (not require_rollout or (mc>0 and ms>0)) and source_hashes_match),"arrival_index":int(arrival[0]),"max_post_event_overshoot_mps":mo,"candidate_count_max":mc,"rollout_samples_max":ms,"source_hashes_match":bool(source_hashes_match)}
def save_failure_trace(path,*,samples,plant,exception,diagnostics,source_before,source_after,identity,stage="solver"):
    identity={**identity,"actual_requested_wall_seconds":identity.get("actual_requested_wall_seconds"),"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION}
    ts=np.asarray([x.time for x in samples],float); x=np.stack([x.state for x in samples]) if samples else np.empty((0,plant.state_size)); rates=np.stack([x.rates for x in samples]) if samples else np.empty_like(x); speed=np.asarray([plant.observe(row).speed for row in x]) if len(x) else np.empty(0); d=[q.policy_diagnostics for q in samples]
    unavailable=isinstance(exception,TimeoutError) and len(samples)==0
    req=np.asarray([[np.nan,np.nan] if q.requested_command is None else [q.requested_command.throttle,q.requested_command.brake_force] for q in samples]).reshape((-1,2))
    applied=np.asarray([[q.command.throttle,q.command.brake_force] for q in samples]).reshape((-1,2)); mapped=np.asarray([[np.nan,np.nan] if getattr(q,"mapped_command",None) is None else [q.mapped_command.throttle,q.mapped_command.brake_force] for q in samples]).reshape((-1,2))
    arrays=_accepted_record_arrays(samples)
    arrays.update(time=ts,state=x,rates=rates,station_m=x[:,0] if len(x) else np.empty(0),speed_mps=speed,body_vx_mps=x[:,3] if x.shape[1]>3 else np.empty(0),requested_command=req,requested_command_available=np.asarray([q.requested_command is not None for q in samples]),applied_command=applied,mapped_command=mapped,phase=np.asarray([getattr(q,"phase","") for q in d],dtype="U32"),status=np.asarray([getattr(q,"status","") for q in d],dtype="U96"),candidate_count=np.asarray([getattr(q,"candidate_count",0) for q in d],int),rollout_samples=np.asarray([getattr(q,"rollout_samples",0) for q in d],int),event_crossed=np.asarray([str(getattr(q,"event_crossed",None)) for q in d],dtype="U8"),failure_type=np.asarray(type(exception).__name__),failure_message=np.asarray(str(exception)),failure_diagnostics=np.asarray(json.dumps(diagnostics,default=str)),accepted_samples=np.asarray(len(samples)),trajectory_availability=np.asarray("unavailable_before_observer_replay" if unavailable else ("observer_rows_delivered_before_failure" if samples else "no_observer_rows_delivered")),source_sha256_before=np.asarray(json.dumps(source_before,sort_keys=True)),source_sha256_after=np.asarray(json.dumps(source_after,sort_keys=True)),source_hashes_match=np.asarray(source_before==source_after),identity=np.asarray(json.dumps(identity,sort_keys=True)),failure_stage=np.asarray(stage),solver_status=np.asarray("FAILED_NO_OBSERVER_TRACE" if unavailable else "FAILED_OBSERVER_ROWS_ONLY"))
    return _atomic_npz(path,**arrays)

def _accepted_record_arrays(samples):
    n=len(samples); diagnostics=[q.policy_diagnostics for q in samples]
    cmd=lambda c: [np.nan,np.nan] if c is None else [c.throttle,c.brake_force]
    fields={"sample_index":np.asarray([getattr(q,"index",i) for i,q in enumerate(samples)],dtype=int),"accepted_time":np.asarray([q.time for q in samples],float),"accepted_state":np.stack([q.state for q in samples]) if n else np.empty((0,0)),"accepted_rates":np.stack([q.rates for q in samples]) if n else np.empty((0,0)),"pedal":np.asarray([np.nan if q.pedal is None else q.pedal for q in samples]),"pedal_available":np.asarray([q.pedal is not None for q in samples]),"requested_command":np.asarray([cmd(q.requested_command) for q in samples],float).reshape((-1,2)),"requested_command_available":np.asarray([q.requested_command is not None for q in samples]),"applied_command":np.asarray([cmd(q.command) for q in samples],float).reshape((-1,2)),"applied_command_available":np.ones(n,dtype=bool),"mapped_command":np.asarray([cmd(getattr(q,"mapped_command",None)) for q in samples],float).reshape((-1,2)),"mapped_command_available":np.asarray([getattr(q,"mapped_command",None) is not None for q in samples]),"policy_diagnostics_available":np.asarray([d is not None for d in diagnostics]),"policy_phase":np.asarray([getattr(d,"phase","") if d is not None else "" for d in diagnostics],dtype="U32"),"policy_status":np.asarray([getattr(d,"status","") if d is not None else "" for d in diagnostics],dtype="U96"),"candidate_count":np.asarray([getattr(d,"candidate_count",-1) if d is not None else -1 for d in diagnostics],int),"rollout_samples":np.asarray([getattr(d,"rollout_samples",-1) if d is not None else -1 for d in diagnostics],int),"event_crossed":np.asarray([str(getattr(d,"event_crossed",None)) if d is not None else "unavailable" for d in diagnostics],dtype="U16"),"event_crossing_speed_mps":np.asarray([np.nan if d is None or getattr(d,"event_crossing_speed_mps",None) is None else d.event_crossing_speed_mps for d in diagnostics]),"event_speed_cap_mps":np.asarray([np.nan if d is None or getattr(d,"event_speed_cap_mps",None) is None else d.event_speed_cap_mps for d in diagnostics]),"target_progress_acceleration_mps2":np.asarray([np.nan if d is None or getattr(d,"target_progress_acceleration_mps2",None) is None else d.target_progress_acceleration_mps2 for d in diagnostics]),"applied_powertrain_diagnostics_available":np.asarray([getattr(q,"applied_powertrain_diagnostics",None) is not None for q in samples]),"applied_powertrain_diagnostics_json":np.asarray([json.dumps(q.applied_powertrain_diagnostics.to_dict(),sort_keys=True,allow_nan=False) if getattr(q,"applied_powertrain_diagnostics",None) is not None else "" for q in samples],dtype="U2048")}
    powertrain=[getattr(q,"applied_powertrain_diagnostics",None) for q in samples]
    fields["applied_powertrain_diagnostics_available"]=np.asarray([d is not None for d in powertrain],dtype=bool)
    fields["applied_powertrain_diagnostics_json"]=np.asarray([json.dumps(d.to_dict(),sort_keys=True,allow_nan=False) if d is not None else "" for d in powertrain],dtype=str)
    return fields
def _atomic_npz(path,**arrays):
    path=Path(path)
    if path.exists(): raise FileExistsError(path)
    fd,tmp=tempfile.mkstemp(prefix="."+path.name+".",suffix=".tmp",dir=path.parent)
    try:
        with os.fdopen(fd,"wb") as f: np.savez_compressed(f,**arrays); f.flush(); os.fsync(f.fileno())
        if path.exists(): raise FileExistsError(path)
        os.link(tmp,path); os.unlink(tmp)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
    return path
def _atomic_json(path,value):
    """Write strict RFC-compatible JSON; invalid nonfinite evidence fails closed."""
    path=Path(path)
    if path.exists(): raise FileExistsError(path)
    fd,tmp=tempfile.mkstemp(prefix="."+path.name+".",suffix=".tmp",dir=path.parent)
    try:
        with os.fdopen(fd,"w",encoding="utf-8") as f: f.write(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+"\n"); f.flush(); os.fsync(f.fileno())
        if path.exists(): raise FileExistsError(path)
        os.link(tmp,path); os.unlink(tmp)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
    return path
def future_event_preflight(policy,*,observation,pedal_position,kinematics,soc,brake_context,expected_event_cap_mps=10.):
    command=policy.decide(observation=observation,pedal_position=pedal_position,kinematics=kinematics,soc=soc,brake_context=brake_context); d=command.decision
    if d.candidate_count<=0 or d.rollout_samples<=0: raise RuntimeError("prospective preflight did not exercise future rollout")
    if d.event_speed_cap_mps is None or abs(float(d.event_speed_cap_mps)-expected_event_cap_mps)>.01: raise RuntimeError("prospective preflight cap mismatch")
    if d.event_crossed is None and d.status!="event_unknown:event_beyond_horizon": raise RuntimeError("preflight unknown for unexpected reason")
    if d.event_crossed is False: raise RuntimeError("prospective preflight predicted no crossing")
    if d.event_crossed is True and (d.event_crossing_speed_mps is None or d.event_crossing_speed_mps>expected_event_cap_mps+.5): raise RuntimeError("prospective preflight overspeed")
    return command
def _make_case(case_id,duration):
    if case_id.startswith("20-to-10"): return 20.,10.,{"20-to-10-p-plus1":1.,"20-to-10-p-zero":0.,"20-to-10-p-minus-half":-.5}[case_id],0.,duration
    if case_id.startswith("15-to-5"): return 15.,5.,{"15-to-5-p-plus1":1.,"15-to-5-p-zero":0.,"15-to-5-p-minus-half":-.5}[case_id],0.,duration
    if case_id.startswith("stop-"): return 3.,.01,.3 if case_id.endswith("point3") else 0.,0.,duration
    if case_id=="left-curvature": return 10.,5.,0.,1/30,duration
    if case_id=="right-curvature": return 10.,5.,0.,-1/30,duration
    if case_id=="prospective-drive-brake": return 20.,10.,1.,0.,duration
    if case_id=="periodic-seam": return 20.,20.,0.,0.,duration
    if case_id=="tv-yaw-reject": return 10.,5.,0.,0.,duration
    raise ValueError(f"unknown case {case_id}")
def _prospective_stop_distance(plan,provider,model,predictor,*,initial_speed_mps,soc,initial_pedal=0.,maximum_time_s=6.,dt=.01,verify_dense=True):
    """Integrate reduced (station,speed,pedal) feedback with real policy and foot ODE."""
    from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy,HybridPolicyConfig
    from simplified_vehicle_models.solvers.longitudinal_capability import PathKinematicContext
    from simplified_vehicle_models.solvers.forward_driver import DriverObservation
    pc=PedalPolicyContext(1/.175,1/.175,.02,model.planar.maximum_pedal_force,.85,1/.175,1/.175)
    policy=HybridLongitudinalPolicy(plan,provider,predictor,HybridPolicyConfig(maximum_brake_force_n=model.planar.maximum_pedal_force,pedal_context=pc,slabs=30))
    s=0.;v=float(initial_speed_mps);p=float(initial_pedal);t=0.; rows=[]; radius=.29; mass=model.planar.total_mass+model.planar.wheel_inertia*4/radius**2
    # Initial normal load and zero curvature are explicit fixed quasi-steady assumptions.
    predictor=BrakeAuthorityPredictor(model.planar,maximum_abs_curvature_inv_m=.001,minimum_speed_mps=.005,maximum_speed_mps=40.)
    explicit_physical_oracle_queries=[]
    target_speed=.01
    event_bracket=None
    while v>target_speed and t<maximum_time_s:
        query_speed=max(v,.005)
        q=provider.evaluate(CapabilityQuery(query_speed,0.,float(soc),0.))
        explicit_physical_oracle_queries.append({"speed_mps":float(query_speed),"curvature_inv_m":0.,"soc":float(soc),"yaw_moment_request_nm":0.,"in_domain":bool(q.in_domain),"drive_net_progress_acceleration_mps2":float(q.drive_net_progress_acceleration_mps2),"coast_net_progress_acceleration_mps2":float(q.coast_net_progress_acceleration_mps2),"brake_net_progress_acceleration_mps2":float(q.brake_net_progress_acceleration_mps2)})
        if not q.in_domain: raise ValueError("stop oracle provider query outside domain")
        kin=PathKinematicContext(v,0.,0.,0.,0.,0.,0.,0.,True)
        # Fixed straight-path static normal-load approximation (not transient certification).
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
        bc=BrakeAuthorityContext(float(v),0.,(radius,)*4,tuple([model.planar.total_mass*9.80665/4]*4),soc=float(soc))
        obs=DriverObservation(s,0.,0.,v,0.,0.)
        cmd=policy.decide(observation=obs,pedal_position=p,kinematics=kin,soc=soc,brake_context=bc)
        target=cmd.first_pedal_target
        if target is None: target=-cmd.brake_force_n/model.planar.maximum_pedal_force if cmd.brake_force_n else cmd.throttle
        delta=float(target)-p
        if p>0: rate=pc.throttle_release_rate if delta<0 else pc.throttle_apply_rate
        elif p<0: rate=pc.brake_release_rate if delta>0 else pc.brake_apply_rate
        else: rate=pc.throttle_apply_rate if delta>=0 else pc.brake_apply_rate
        pdot=float(np.clip(delta/pc.pedal_time_constant,-rate,rate)); pn=float(np.clip(p+pdot*dt,-1.,1.))
        pedal_mid=.5*(p+pn)
        predictor_query=None
        if pedal_mid<0:
            prediction=predictor.predict(bc,-pedal_mid*model.planar.maximum_pedal_force); acc=-prediction.deceleration_mps2
            predictor_query={"speed_mps":float(v),"curvature_inv_m":0.,"soc":float(soc),"wheel_radii_m":list(bc.wheel_radii_m),"normal_loads_n":list(bc.normal_loads_n),"pedal_force_n":float(-pedal_mid*model.planar.maximum_pedal_force),"deceleration_mps2":float(prediction.deceleration_mps2),"in_domain":bool(prediction.in_domain),"assumption":prediction.assumption}
        else:
            from tools.run.validate_hybrid_driver import _physical_pedal_acceleration
            acc=_physical_pedal_acceleration(model,predictor,bc,v,pedal_mid,soc)
            # Predictor minimum is .005 m/s; shared powertrain scalar uses .01 m/s.
        step=dt; crossing=None
        trial_v=v+acc*dt
        if acc<0. and trial_v<=target_speed:
            step=(target_speed-v)/acc
            if not 0.<step<=dt: raise ValueError("invalid prospective terminal-event bracket")
            crossing={"speed_before_mps":float(v),"speed_after_full_step_mps":float(trial_v),"fraction_of_step":float(step/dt),"step_start_time_s":float(t),"step_start_station_m":float(s)}
        vn=v+acc*step
        sn=s+.5*(v+vn)*step
        pn_step=p+(pn-p)*(step/dt)
        t+=step; s=sn; p=pn_step; v=vn
        rows.append({"time_s":float(t),"station_m":float(s),"speed_mps":float(v),"pedal":float(p),"policy_status":cmd.decision.status,"policy_decision":cmd.decision.to_dict(),"policy_command":{"first_pedal_target":cmd.first_pedal_target,"throttle":cmd.throttle,"brake_force_n":cmd.brake_force_n},"foot_rate_per_s":float(pdot),"applied_midpoint_pedal":float(pedal_mid),"acceleration_mps2":float(acc),"explicit_physical_oracle_query":explicit_physical_oracle_queries[-1],"policy_internal_query_recording":"not intercepted; policy/provider internals excluded","predictor_query":predictor_query})
        if crossing is not None:
            event_bracket={**crossing,"event_time_s":float(t),"event_station_m":float(s),"event_pedal":float(p),"event_speed_mps":float(v),"event_fraction_of_full_step":crossing["fraction_of_step"]}
            # Verify the endpoint is within the same provider and predictor authority
            # domains without integrating beyond the positive target event.
            endpoint_query=provider.evaluate(CapabilityQuery(target_speed,0.,float(soc),0.))
            if not endpoint_query.in_domain: raise ValueError("terminal event endpoint is outside provider domain")
            try:
                predictor.predict(BrakeAuthorityContext(target_speed,0.,(radius,)*4,tuple([model.planar.total_mass*9.80665/4]*4),soc=float(soc)),0.)
            except ValueError as exc: raise ValueError(f"terminal event endpoint outside predictor domain: {exc}") from exc
            explicit_physical_oracle_queries.append({"speed_mps":target_speed,"curvature_inv_m":0.,"soc":float(soc),"yaw_moment_request_nm":0.,"in_domain":bool(endpoint_query.in_domain),"drive_net_progress_acceleration_mps2":float(endpoint_query.drive_net_progress_acceleration_mps2),"coast_net_progress_acceleration_mps2":float(endpoint_query.coast_net_progress_acceleration_mps2),"brake_net_progress_acceleration_mps2":float(endpoint_query.brake_net_progress_acceleration_mps2),"query_role":"terminal_endpoint_authority_check"})
            break
    if v>target_speed+1e-12: raise ValueError("prospective stop oracle exceeded six-second bound")
    query_speeds=[q["speed_mps"] for q in explicit_physical_oracle_queries]
    convergence=None
    if verify_dense and dt>.001:
        reference=_prospective_stop_distance(plan,provider,model,predictor,initial_speed_mps=initial_speed_mps,soc=soc,initial_pedal=initial_pedal,maximum_time_s=maximum_time_s,dt=.001,verify_dense=False)
        difference=100.*abs(float(s)-reference["distance_m"])/max(reference["distance_m"],1e-9)
        if difference>1.: raise ValueError(f"stop oracle dense-step convergence exceeds 1%: {difference:.6g}%")
        convergence={"reference_dt_s":.001,"reference_distance_m":reference["distance_m"],"coarse_vs_reference_error_percent":difference,"tolerance_percent":1.}
    return {"distance_m":float(s),"time_s":float(t),"samples":len(rows),"terminal_speed_mps":float(v),"target_speed_mps":target_speed,"terminal_event_bracket":event_bracket,"dt_s":dt,"policy":"HybridLongitudinalPolicy.decide feedback; 30 policy slabs/query","initial_plan_sample":{"station_m":0.,"speed_mps":float(plan.sample(0.).speed_mps),"cap_mps":float(plan.sample(0.).corner_speed_cap_mps),"acceleration_mps2":float(plan.sample(0.).acceleration_mps2),"braking_speed_bound_mps":float(plan.sample(0.).braking_speed_bound_mps)},"foot_model":"signed pedal rates 1/.175, pedal time constant .02; integrated ODE","explicit_physical_oracle_queries":explicit_physical_oracle_queries,"explicit_physical_oracle_query_count":len(explicit_physical_oracle_queries),"explicit_physical_oracle_query_scope":"integration-state authority queries plus terminal endpoint check only; excludes internal HybridLongitudinalPolicy/provider capability queries","explicit_physical_oracle_query_speed_mps_range":[float(min(query_speeds)),float(max(query_speeds))] if query_speeds else None,"dense_convergence":convergence,"integrated_samples":rows,"fixed_assumptions":"straight progress curvature=0; fixed SOC; initial normal loads fixed at static quarter-load; compiled provider/powertrain; no actual-trace inputs","query_scope":"explicit physical oracle probes only; policy-internal capability queries are not enumerated","inputs":{"initial_speed_mps":float(initial_speed_mps),"initial_pedal":float(initial_pedal),"soc":float(soc),"target_speed_mps":target_speed,"maximum_time_s":maximum_time_s,"integrator_step_s":dt,"policy_slabs":30,"pedal_time_constant_s":.02,"pedal_apply_release_rate_per_s":1/.175,"predictor_minimum_speed_mps":.005,"straight_curvature_inv_m":0.,"vehicle_mass_kg":float(model.planar.total_mass),"wheel_inertia_kg_m2":float(model.planar.wheel_inertia),"effective_mass_kg":float(mass),"wheel_radii_m":[radius]*4,"static_normal_loads_n":[float(model.planar.total_mass*9.80665/4)]*4,"maximum_pedal_force_n":float(model.planar.maximum_pedal_force),"brake_margin":float(plan.brake_margin),"provider_identity":provider.identity,"provider_model_hash":provider.model_hash,"provider_source_closure_hash":provider.source_closure_hash,"plan_identity":plan.identity,"predictor_assumption":"BrakeAuthorityPredictor fixed straight static-load context","powertrain_acceleration_model":"shared _physical_pedal_acceleration, including coast and effective-mass correction"}}

def prospective_cap_event_station(provider,*,initial_speed_mps,peak_speed_mps,target_speed_mps,curvature_inv_m,soc,brake_margin=.60,throttle_release_s=.35,actuator_lag_s=.10,fixed_buffer_m=20.):
    vals=np.asarray([initial_speed_mps,peak_speed_mps,target_speed_mps,curvature_inv_m,soc,brake_margin,throttle_release_s,actuator_lag_s,fixed_buffer_m],float)
    if not np.all(np.isfinite(vals)) or initial_speed_mps<=0 or peak_speed_mps<=initial_speed_mps or target_speed_mps<=0 or target_speed_mps>=peak_speed_mps or not 0<brake_margin<=1 or min(throttle_release_s,actuator_lag_s,fixed_buffer_m)<0: raise ValueError("invalid prospective event inputs")
    q=lambda v:provider.evaluate(CapabilityQuery(float(v),float(curvature_inv_m),float(soc),0.))
    dr=[q(v) for v in np.linspace(initial_speed_mps,peak_speed_mps,5)]; br=[q(v) for v in np.linspace(target_speed_mps,peak_speed_mps,9)]
    if not all(x.in_domain for x in dr+br): raise ValueError("prospective authority outside provider domain")
    drive=min(x.drive_net_progress_acceleration_mps2 for x in dr); brake=min(max(0.,-x.brake_net_progress_acceleration_mps2) for x in br)
    if drive<=0 or brake<=0: raise ValueError("prospective event lacks authority")
    dd=(peak_speed_mps**2-initial_speed_mps**2)/(2*drive); bd=(peak_speed_mps**2-target_speed_mps**2)/(2*brake*brake_margin); lead=peak_speed_mps*(throttle_release_s+actuator_lag_s)
    return {"event_station_m":float(dd+bd+lead+fixed_buffer_m),"drive_acceleration_min_mps2":drive,"brake_deceleration_min_mps2":brake,"brake_margin":brake_margin,"drive_distance_m":dd,"brake_distance_m":bd,"foot_release_lead_distance_m":lead,"fixed_buffer_m":fixed_buffer_m,"authority_speed_samples_mps":np.linspace(target_speed_mps,peak_speed_mps,9).tolist(),"basis":"physical provider sampled before actual truth"}
def transition_gate(*,reached_end,finite_state,contact_min,event_arrived,post_event_overshoots,candidate_counts,rollout_samples,source_hashes_match=True,require_rollout=False):
    o=np.asarray(post_event_overshoots,float); c=np.asarray(candidate_counts,int); s=np.asarray(rollout_samples,int); workload=bool(c.size and s.size and c.max()<=3 and s.max()<=180 and (not require_rollout or (c.max()>0 and s.max()>0)))
    return bool(reached_end and finite_state and np.isfinite(contact_min) and contact_min>=0 and event_arrived and o.size and np.all(np.isfinite(o)) and np.all(o<=.5) and workload and source_hashes_match)

def terminal_stop_gate(*,reached_end,finite_state,contact_min,arrival_index,speed_mps,cap_mps,requested_available,source_hashes_match,candidate_counts,rollout_samples,distance_error_percent):
    v=np.asarray(speed_mps,float); cap=np.asarray(cap_mps,float); c=np.asarray(candidate_counts,int); slabs=np.asarray(rollout_samples,int)
    good=(reached_end and finite_state and np.isfinite(contact_min) and contact_min>=0 and arrival_index is not None and 0<=arrival_index<len(v) and len(v)>1 and np.all(np.isfinite(v)) and np.all(np.isfinite(cap)) and np.all(v<=cap+.5+1e-9) and np.all(requested_available) and source_hashes_match and c.size and slabs.size and c.max()<=3 and slabs.max()<=180 and distance_error_percent is not None and np.isfinite(distance_error_percent) and abs(distance_error_percent)<=5.)
    return bool(good)
def _build_terminal_stop_fixture(provider,*,soc,initial_speed_mps=3.,target_speed_mps=.01,brake_margin=.60,distance_m=2.):
    """Runner-only immutable terminal prescription; not builder-certified physics."""
    vals=np.asarray([soc,initial_speed_mps,target_speed_mps,brake_margin,distance_m],float)
    if not np.all(np.isfinite(vals)) or initial_speed_mps<=target_speed_mps or target_speed_mps<=0 or distance_m<=0 or not 0<brake_margin<=1: raise ValueError("invalid terminal-stop fixture inputs")
    # Curvature is fixed to the actual straight fixture. Prove current provider domain
    # and positive signed braking authority across the prescribed speed interval.
    queries=[]
    for speed in np.linspace(target_speed_mps,initial_speed_mps,9):
        q=provider.evaluate(CapabilityQuery(float(speed),0.,float(soc),0.))
        if not q.in_domain or not np.isfinite(q.brake_net_progress_acceleration_mps2) or q.brake_net_progress_acceleration_mps2>=0:
            raise ValueError("terminal prescription lacks in-domain positive braking authority")
        queries.append(q)
    s=np.linspace(0.,float(distance_m),101); qtraj=np.linspace(initial_speed_mps**2,target_speed_mps**2,len(s)); v=np.sqrt(qtraj); accel=np.full_like(v,(target_speed_mps**2-initial_speed_mps**2)/(2*distance_m))
    # Cap samples follow the same q-linear trajectory; dense knots make the public
    # linear cap interpolation converge to its exact square-root speed profile.
    plan=HybridSpeedPlan(s,v,accel,v.copy(),v.copy(),np.full(len(s),len(s)-1,dtype=int),np.zeros_like(s),False,float(soc),float(brake_margin),provider.identity,provider.source_closure_hash,"runner-terminal-stop-v1","TERMINAL_PRESCRIPTION_NOT_BUILDER_CERTIFIED",0,tuple(sorted({c for q in queries for c in q.limiting_constraints})),"task5-terminal-fixture-v1",1,1e-8)
    evidence={"status":"TERMINAL_PRESCRIPTION_NOT_BUILDER_CERTIFIED","basis":"prospective exact straight-path terminal cap; positive endpoint .01 m/s; fixed curvature=0 and SOC","provider_identity":provider.identity,"source_closure_hash":provider.source_closure_hash,"soc":float(soc),"brake_margin":float(brake_margin),"speed_queries_mps":[float(x) for x in np.linspace(target_speed_mps,initial_speed_mps,9)],"authority_mps2":[float(q.brake_net_progress_acceleration_mps2) for q in queries],"distance_m":float(distance_m)}
    return plan,evidence

def _assemble(case_id,duration,*,forecast_slabs=60):
    if isinstance(forecast_slabs,bool) or not isinstance(forecast_slabs,int) or not 1<=forecast_slabs<=60: raise ValueError("forecast_slabs must be an integer in [1, 60]")
    v0,target,pedal,curvature,_=_make_case(case_id,duration); model=compile_formulation(str(FORMULATION)).model; vehicle=model.planar; stations=np.linspace(0.,600.,601); caps=np.full_like(stations,max(30.,v0+2.)); k=np.full_like(stations,curvature)
    if abs(curvature)>.001: caps[:]=v0
    if abs(curvature)>.001: speed_nodes=(.01,2.5,5.,7.5,10.,10.1); kgrid=(curvature-.001,curvature,curvature+.001)
    else: speed_nodes=(.005,.01,5.,10.,20.,22.,25.,40.); kgrid=(-.001,0.,.001)
    if case_id.startswith("stop-"): caps[:]=3.; caps[stations>=5.]=.01 # steering plan only; terminal hybrid plan is separately prescribed
    elif case_id=="prospective-drive-brake": caps[:]=22.
    elif case_id=="periodic-seam": caps[:]=20.; stations=np.linspace(0.,1000.,1001); caps=np.full_like(stations,20.); k=np.zeros_like(stations)
    else:
        event0=75. if v0>=15 else 45.; caps[stations>=event0]=target
    steering=SpeedPlan(stations,caps,k,False); plant=VerticalForwardPlant(model,steering); initial=plant.initial_state(v0); soc=float(initial[SOC_INDEX])
    if not .8<=soc<=.9: raise RuntimeError("initial SOC outside provider domain")
    provider=CurrentVehicleCapability(vehicle,wheel_radii_m=(.29,)*4,speed_grid_mps=speed_nodes,curvature_grid_inv_m=kgrid,soc_grid=(.8,.9),source_label="task5-transition-actual-38state-v4",calibration_label="compiled-IEM-current")
    event_meta=None; event=5. if case_id.startswith("stop-") else (75. if v0>=15 else 45.)
    stop_plan_meta=None
    if case_id.startswith("stop-"):
        plan,stop_plan_meta=_build_terminal_stop_fixture(provider,soc=soc)
        # This plan is only the immutable terminal prescription. A separate prospective
        # reduced rollout is recorded as an approximation; no actual samples enter it.
    if case_id=="prospective-drive-brake":
        event_meta=prospective_cap_event_station(provider,initial_speed_mps=20.,peak_speed_mps=22.,target_speed_mps=10.,curvature_inv_m=0.,soc=soc); event=float(np.ceil(event_meta["event_station_m"])); caps[:]=22.; caps[stations>=event]=10.
    if case_id=="periodic-seam":
        initial[0]=495.; pd=np.linspace(0.,500.,501); plan=build_hybrid_speed_plan(pd,np.zeros_like(pd),np.full_like(pd,20.),provider,initial_speed_mps=v0,storage_soc=soc,is_closed=True)
    elif not case_id.startswith("stop-"): plan=build_hybrid_speed_plan(stations,k,caps,provider,initial_speed_mps=v0,storage_soc=soc,is_closed=False)
    if case_id=="prospective-drive-brake":
        event_meta["snapped_event_station_m"]=event; event_meta["plan_event_cap_mps"]=plan.sample(0.).future_event_speed_cap_mps
        if abs(event_meta["plan_event_cap_mps"]-10.)>.01: raise RuntimeError("prospective plan does not carry future cap10")
    if case_id=="tv-yaw-reject":
        allocator=getattr(provider,"allocator_identity",provider.identity)
        execution_identity=None
        try: validate_execution_allocator(allocator,execution_identity,100.,provider)
        except ValueError as exc: return {"case_id":case_id,"status":"EXPECTED_UNSUPPORTED","error":str(exc),"capability_identity":allocator,"execution_identity":execution_identity,"provider_identity":provider.identity,"request_yaw_moment_nm":100.,"source_sha256":_sha(CORE_SOURCE_NAMES)}
        raise RuntimeError("nonzero yaw request unexpectedly accepted")
    pc=PedalPolicyContext(1/.175,1/.175,.02,vehicle.maximum_pedal_force,.85,1/.175,1/.175); predictor=BrakeAuthorityPredictor(vehicle,maximum_abs_curvature_inv_m=max(map(abs,kgrid)),minimum_speed_mps=.005,maximum_speed_mps=max(speed_nodes)); policy=HybridLongitudinalPolicy(plan,provider,predictor,HybridPolicyConfig(maximum_brake_force_n=vehicle.maximum_pedal_force,pedal_context=pc,slabs=forecast_slabs)); driver=PreviewDriver(steering,vehicle.wheelbase,vehicle.steering_maximum_angle,vehicle.steering_maximum_rate,vehicle.maximum_pedal_force,pedal_travel_time=.175,hybrid_policy=policy); driver=replace(driver,throttle_apply_rate=1/.175,throttle_release_rate=1/.175,brake_apply_rate=1/.175,brake_release_rate=1/.175)
    if case_id=="prospective-drive-brake":
        probe=initial.copy(); probe[0]=40.; probe[3]=float(np.sqrt(20.**2+2.*.9845*40.)); obs=plant.observe(probe); kin=plant.hybrid_kinematic_context(probe); bc=plant.brake_authority_context(probe,predictor=predictor); preflight=future_event_preflight(policy,observation=obs,pedal_position=1.,kinematics=kin,soc=soc,brake_context=bc); ps=plan.sample(obs.distance)
        if ps.future_event_speed_cap_mps is None or abs(ps.future_event_speed_cap_mps-10.)>.01 or ps.future_event_station_m is None: raise RuntimeError("preflight plan sample missing future event cap10")
    return locals()
def seam_crossing_evidence(station_m,*,seam_m=500.,wrapped=False):
    s=np.asarray(station_m,float)
    if s.ndim!=1 or len(s)<2 or not np.all(np.isfinite(s)): raise ValueError("finite station samples required")
    if wrapped:
        pairs=np.flatnonzero(np.diff(s)<-seam_m/2)
        if not len(pairs): raise ValueError("accepted path has no wrapped seam transition")
        i=int(pairs[0]); frac=float((seam_m-s[i])/(s[i+1]+seam_m-s[i]))
    else:
        pairs=np.flatnonzero((s[:-1]<seam_m)&(s[1:]>=seam_m))
        if not len(pairs): raise ValueError("accepted path has no seam crossing")
        i=int(pairs[0]); frac=float((seam_m-s[i])/(s[i+1]-s[i]))
    return {"index_before":i,"index_after":i+1,"station_before_m":float(s[i]),"station_after_m":float(s[i+1]),"path_fraction":frac,"seam_station_m":float(seam_m)}

def seam_continuity_evidence(*,station_m,time_s,state,requested_command,applied_command,pedal,requested_available,pedal_available,plan,driver,indices=None,station_tolerance_m=.05,speed_tolerance_mps=.5):
    station=np.asarray(station_m,float); times=np.asarray(time_s,float); states=np.asarray(state,float); req=np.asarray(requested_command,float); app=np.asarray(applied_command,float); pedals=np.asarray(pedal,float); reqok=np.asarray(requested_available,bool); pok=np.asarray(pedal_available,bool)
    ev=seam_crossing_evidence(station,seam_m=500.) if indices is None else {"index_before":int(indices[0]),"index_after":int(indices[1]),"station_before_m":float(station[indices[0]]),"station_after_m":float(station[indices[1]]),"seam_station_m":500.}
    i,j=ev["index_before"],ev["index_after"]
    if not (0<=i<j<len(station) and j==i+1): raise ValueError("seam bracket indices are not adjacent accepted samples")
    if not np.all(reqok[[i,j]]) or not np.all(pok[[i,j]]) or req.ndim!=2 or req.shape[1]<3 or app.ndim!=2 or app.shape[1]<3: raise ValueError("seam bracket requested command or signed pedal unavailable")
    if not np.all(np.isfinite(req[[i,j]])) or not np.all(np.isfinite(app[[i,j]])) or not np.all(np.isfinite(pedals[[i,j]])) or not np.all(np.isfinite(states[[i,j]])) or not np.all(np.isfinite(times[[i,j]])): raise ValueError("nonfinite seam bracket evidence")
    dt=float(times[j]-times[i]); ds=float(station[j]-station[i])
    if dt<=0 or ds<=0: raise ValueError("seam bracket time/station must advance")
    v=max(abs(float(states[i,3])),abs(float(states[j,3])),1.)
    max_ds=v*dt+station_tolerance_m
    if ds>max_ds: raise ValueError("seam station jump exceeds bounded progression")
    vx_delta=abs(float(states[j,3]-states[i,3]))
    vx_bound=30.*dt+speed_tolerance_mps
    if vx_delta>vx_bound: raise ValueError("body longitudinal speed reset/discontinuity across seam")
    from simplified_vehicle_models.solvers.vertical_forward import STEERING_INDEX
    steering_index=STEERING_INDEX if states.shape[1]>STEERING_INDEX else None
    steering_delta=0. if steering_index is None else abs(float(states[j,steering_index]-states[i,steering_index]))
    try:
        steering_rate=float(driver.maximum_steering_rate)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("seam driver maximum_steering_rate unavailable or invalid") from exc
    if not np.isfinite(steering_rate) or steering_rate<0.:
        raise ValueError("seam driver maximum_steering_rate must be finite and nonnegative")
    steering_bound=steering_rate*dt+1e-6
    if steering_delta>steering_bound: raise ValueError("steering state reset/rate discontinuity across seam")
    rates=driver
    limits={"throttle":max(rates.throttle_apply_rate,rates.throttle_release_rate),"brake_force":max(rates.brake_apply_rate,rates.brake_release_rate),"pedal":max(rates.throttle_apply_rate,rates.throttle_release_rate,rates.brake_apply_rate,rates.brake_release_rate)}
    changes={"requested_command_throttle":abs(float(req[j,0]-req[i,0])),"requested_command_brake_force_n":abs(float(req[j,1]-req[i,1])),"applied_command_throttle":abs(float(app[j,0]-app[i,0])),"applied_command_brake_force_n":abs(float(app[j,1]-app[i,1])),"signed_pedal":abs(float(pedals[j]-pedals[i]))}
    # Feedback commands may legitimately differ; these are diagnostic deltas, not
    # exact-equality tests. No-reset continuity is enforced on actual pedal slew.
    if changes["signed_pedal"]>limits["pedal"]*dt+1e-6: raise ValueError("signed pedal reset/rate discontinuity across seam")
    expected_applied_i=[max(float(pedals[i]),0.),max(-float(pedals[i]),0.)*driver.maximum_brake_force]
    expected_applied_j=[max(float(pedals[j]),0.),max(-float(pedals[j]),0.)*driver.maximum_brake_force]
    if not np.allclose(app[i,:2],expected_applied_i,rtol=0.,atol=1e-6) or not np.allclose(app[j,:2],expected_applied_j,rtol=0.,atol=1e-6): raise ValueError("applied pedal commands do not match signed pedal across seam")
    # Steering-rate requests and applied commands are retained as diagnostics;
    # finite lagged actuator response is expected, so no request-equality gate.
    if np.max(np.abs(req[[i,j]]))>1e6 or np.max(np.abs(app[[i,j]]))>1e6: raise ValueError("seam command outside finite continuity bounds")
    ps0=plan.sample(float(station[i])).speed_mps; ps1=plan.sample(float(station[j])).speed_mps
    if abs(ps1-ps0)>.5: raise ValueError("periodic plan reference discontinuity across seam")
    ev.update({"time_before_s":float(times[i]),"time_after_s":float(times[j]),"dt_s":dt,"station_progress_m":ds,"maximum_progress_bound_m":max_ds,"requested_command_before":req[i].tolist(),"requested_command_after":req[j].tolist(),"requested_command_available":True,"requested_steering_rate_before":float(req[i,2]),"requested_steering_rate_after":float(req[j,2]),"applied_command_before":app[i].tolist(),"applied_command_after":app[j].tolist(),"applied_steering_rate_before":float(app[i,2]),"applied_steering_rate_after":float(app[j,2]),"steering_rate_lagged_response_diagnostic":True,"signed_pedal_before":float(pedals[i]),"signed_pedal_after":float(pedals[j]),"pedal_available":True,"command_delta_diagnostics":changes,"pedal_rate_limit_per_s":limits["pedal"],"pedal_rate_bound":limits["pedal"]*dt+1e-6,"body_vx_before_mps":float(states[i,3]),"body_vx_after_mps":float(states[j,3]),"plan_reference_speed_before_mps":float(ps0),"plan_reference_speed_after_mps":float(ps1),"plan_reference_continuity_tolerance_mps":.5})
    return ev

def _postprocess_forward_result(result,samples,plant,plan,*,case_id,v0,target,pedal,event,duration,forecast_slabs,source_before,source_after,identity,output_dir,source_hashes_match_required=True,driver=None):
    if not samples: raise ValueError("postprocessing requires accepted StepSample rows")
    state=np.stack([x.state for x in samples]); ts=np.asarray([x.time for x in samples],float); rates=np.stack([x.rates for x in samples]); expected=np.asarray(result.state,float).T
    if expected.shape==(state.shape[1],state.shape[0]) and not np.allclose(state,expected.T,rtol=1e-8,atol=1e-10): raise ValueError("accepted StepSample states do not align with ForwardResult")
    if any(x.policy_diagnostics is None for x in samples): raise ValueError("policy diagnostics unavailable")
    request_available=np.asarray([x.requested_command is not None for x in samples],bool); requested=np.asarray([[np.nan,np.nan,np.nan] if x.requested_command is None else [x.requested_command.throttle,x.requested_command.brake_force,x.requested_command.steering_rate] for x in samples]); applied=np.asarray([[x.command.throttle,x.command.brake_force,x.command.steering_rate] for x in samples]); pedals=np.asarray([np.nan if x.pedal is None else x.pedal for x in samples]); d=[x.policy_diagnostics for x in samples]
    candidates=np.asarray([x.candidate_count for x in d]); slabs=np.asarray([x.rollout_samples for x in d]); phases=np.asarray([x.phase for x in d],dtype="U32"); statuses=np.asarray([x.status for x in d],dtype="U96"); speed=np.asarray([plant.observe(x.state).speed for x in samples]); station=state[:,0]; cap=np.interp(station,plan.distance_m,plan.corner_speed_cap_mps); overshoot=speed-cap
    stop=case_id.startswith("stop-"); arrival=np.flatnonzero(speed<=.0100001) if stop else np.flatnonzero((station>=event)&(speed<=target+.5)); arrival_i=int(arrival[0]) if len(arrival) else None
    seam=None
    if case_id=="periodic-seam":
        if driver is None: raise ValueError("periodic-seam requires actual driver rate configuration")
        seam=seam_continuity_evidence(station_m=station,time_s=ts,state=state,requested_command=requested,applied_command=applied,pedal=pedals,requested_available=request_available,pedal_available=np.asarray([x.pedal is not None for x in samples]),plan=plan,driver=driver)
        seam.update({"speed_before_mps":float(speed[seam["index_before"]]),"speed_after_mps":float(speed[seam["index_after"]]),"curvature_before_inv_m":float(plant.hybrid_kinematic_context(state[seam["index_before"]]).curvature_inv_m),"curvature_after_inv_m":float(plant.hybrid_kinematic_context(state[seam["index_after"]]).curvature_inv_m)})
    stop_oracle=identity.get("stop_oracle") if stop else None
    from simplified_vehicle_models.solvers.vertical_forward import JOUNCE_INDEX,STEERING_INDEX,WHEEL_SPEED_INDEX
    kinctx=[plant.hybrid_kinematic_context(row) for row in state]; kin=[q.vy_dot_valid for q in kinctx]; context_valid=np.asarray(kin,bool); vy_dot=np.asarray([q.vy_dot_estimate_mps2 for q in kinctx]); actual_curvature=np.asarray([q.curvature_inv_m for q in kinctx]); jounce=state[:,list(JOUNCE_INDEX)]; wheel_speeds=state[:,list(WHEEL_SPEED_INDEX)]; steering=state[:,STEERING_INDEX]; body_vx=state[:,3]; post=station>=event; err=float(max(0.,np.max(overshoot[post]))) if np.any(post) else None; source_match=(source_before==source_after) and source_hashes_match_required
    try:
        if stop:
            if len(ts)<2 or not np.all(np.isfinite(state)) or not np.isfinite(plant.minimum_contact_load) or plant.minimum_contact_load<0: raise ValueError("invalid terminal-stop trace/contact")
            if not np.all(request_available): raise ValueError("requested command unavailable on terminal-stop trace")
            if not source_match: raise ValueError("source hashes changed")
            if candidates.max(initial=0)>3 or slabs.max(initial=0)>180: raise ValueError("workload bound exceeded")
            if arrival_i is None: raise ValueError("terminal .01 m/s arrival not observed")
            if not np.all(np.isfinite(overshoot)) or np.max(overshoot)>.5+1e-9: raise ValueError("terminal-stop cap overshoot exceeds 0.5 m/s tolerance")
            if ts[-1]<duration-1e-7 and speed[-1]>.0100001: raise ValueError("solver did not reach stop event or requested duration")
            actual=float(station[arrival_i]-station[0]); error=None if stop_oracle is None else 100.*abs(stop_oracle["distance_m"]-actual)/max(actual,1e-9)
            metrics={"passed":terminal_stop_gate(reached_end=(ts[-1]>=duration-1e-7 or arrival_i is not None),finite_state=bool(np.all(np.isfinite(state))),contact_min=plant.minimum_contact_load,arrival_index=arrival_i,speed_mps=speed,cap_mps=cap,requested_available=request_available,source_hashes_match=source_match,candidate_counts=candidates,rollout_samples=slabs,distance_error_percent=error),"arrival_index":arrival_i,"terminal_stop":True,"arrival_semantics":"first accepted speed <= .0100001 m/s; unrelated event station is context only","actual_distance_m":actual,"forecast_distance_m":None if stop_oracle is None else stop_oracle["distance_m"],"distance_error_percent":error,"source_hashes_match":source_match,"candidate_count_max":int(candidates.max(initial=0)),"rollout_samples_max":int(slabs.max(initial=0))}
        else:
            metrics=trace_metrics(time_s=ts,state=state,station_m=station,speed_mps=speed,cap_mps=cap,contact_min_n=plant.minimum_contact_load,event_station_m=event,target_speed_mps=target,candidate_count=candidates,rollout_samples=slabs,solver_status="finished" if ts[-1]>=duration-1e-7 else "failed",source_hashes_match=source_match,require_rollout=case_id=="prospective-drive-brake",contact_metric_available=False)
    except ValueError as exc: metrics={"passed":False,"reason":str(exc)}
    post_overshoot=overshoot if stop else overshoot[post]
    if stop: passed=bool(metrics.get("passed",False))
    else: passed=bool(np.all(request_available)) and transition_gate(reached_end=(len(ts)>1 and ts[-1]>=duration-1e-7),finite_state=bool(np.all(np.isfinite(state))),contact_min=plant.minimum_contact_load if np.isfinite(plant.minimum_contact_load) else -1.,event_arrived=arrival_i is not None,post_event_overshoots=post_overshoot,candidate_counts=candidates,rollout_samples=slabs,source_hashes_match=source_match,require_rollout=case_id=="prospective-drive-brake") and metrics.get("passed",False) and err is not None and err<=.5
    from simplified_vehicle_models.solvers.hybrid_driver import HYBRID_POLICY_VERSION
    row={"case_id":case_id,"status":"PASS" if passed else "FAIL","initial_speed_mps":v0,"target_cap_mps":target,"initial_signed_pedal":pedal,"event_station_m":event,"duration_s":duration,"solver_end_s":float(ts[-1]),"arrival_time_s":None if arrival_i is None else float(ts[arrival_i]),"max_post_event_overshoot_mps":err,"max_post_event_overshoot_status":"NOT_APPLICABLE" if stop and not np.any(post) else "AVAILABLE","max_stop_cap_overshoot_mps":float(max(0.,np.max(overshoot))) if stop else None,"contact_min_n_summary_only":float(plant.minimum_contact_load),"contact_load_trace_available":False,"all_states_finite":bool(np.all(np.isfinite(state))),"candidate_count_max":int(candidates.max(initial=0)),"rollout_samples_max":int(slabs.max(initial=0)),"rollout_event_samples":int(np.count_nonzero((candidates>0)&(slabs>0))),"trace_metrics":metrics,"source_sha256_before":source_before,"source_sha256_after":source_after,"source_hashes_match":source_match,"provider_identity":identity["provider_identity"],"provider_model_hash":identity["provider_model_hash"],"provider_source_closure_hash":identity["provider_source_closure_hash"],"plan_identity":identity["plan_identity"],"policy_version":HYBRID_POLICY_VERSION,"solver_message":result.message,"lap_completed":result.lap_completed,"distance_oracle_status":"PROSPECTIVE_POLICY_FEEDBACK" if stop_oracle else "NOT_APPLICABLE","distance_oracle_tolerance_percent":5.0 if stop else None,"stop_distance_oracle":stop_oracle,"forecast_vs_actual_distance_error_percent":None if not stop_oracle or arrival_i is None else 100.*abs(stop_oracle["distance_m"]-float(station[arrival_i]-station[0]))/max(float(station[arrival_i]-station[0]),1e-9),"seam_crossing":seam,"terminal_fixture":identity.get("terminal_fixture"),"forecast_slabs":forecast_slabs,"production_default_slabs":HybridPolicyConfig().slabs}
    arrays={"stop_oracle_json":np.asarray(json.dumps(stop_oracle,sort_keys=True) if stop_oracle is not None else ""),"stop_actual_distance_m":np.asarray(float(station[arrival_i]-station[0]) if stop and arrival_i is not None else np.nan),"stop_forecast_actual_error_percent":np.asarray(float(metrics.get("distance_error_percent")) if stop and metrics.get("distance_error_percent") is not None else np.nan),"stop_evidence_identity_json":np.asarray(json.dumps({"terminal_fixture":identity.get("terminal_fixture"),"provider_identity":identity.get("provider_identity"),"provider_model_hash":identity.get("provider_model_hash"),"provider_source_closure_hash":identity.get("provider_source_closure_hash"),"plan_identity":identity.get("plan_identity"),"source_sha256_before":source_before,"source_sha256_after":source_after},sort_keys=True) if stop else ""),"time":ts,"state":state,"rates":rates,"station_m":station,"speed_mps":speed,"body_vx_mps":body_vx,"wheel_speed_rad_s":wheel_speeds,"curvature_inv_m":actual_curvature,"context_valid":context_valid,"vy_dot_mps2":vy_dot,"steering_rad":steering,"jounce_m":jounce,"contact_load_trace_available":np.asarray(False),"pedal":pedals,"requested_command":requested,"requested_command_available":request_available,"applied_command":applied,"phase":phases,"status":statuses,"candidate_count":candidates,"rollout_samples":slabs,"cap_mps":cap,"cap_overshoot_mps":overshoot,"event_station_m":np.asarray(event),"event_cap_mps":np.asarray(target),"event_crossed":np.asarray([str(x.event_crossed) for x in d],dtype="U8"),"event_crossing_speed_mps":np.asarray([np.nan if x.event_crossing_speed_mps is None else x.event_crossing_speed_mps for x in d]),"event_crossing_time_s":np.asarray([np.nan if x.event_crossing_time_s is None else x.event_crossing_time_s for x in d]),"future_event_speed_cap_mps":np.asarray([np.nan if x.event_speed_cap_mps is None else x.event_speed_cap_mps for x in d]),"seam_crossing_json":np.asarray(json.dumps(seam,sort_keys=True) if seam is not None else ""),"seam_indices":np.asarray([-1,-1] if seam is None else [seam["index_before"],seam["index_after"]],dtype=int),"seam_requested_before":np.asarray([np.nan,np.nan] if seam is None else seam["requested_command_before"],dtype=float),"seam_requested_after":np.asarray([np.nan,np.nan] if seam is None else seam["requested_command_after"],dtype=float),"seam_applied_before":np.asarray([np.nan,np.nan] if seam is None else seam["applied_command_before"],dtype=float),"seam_applied_after":np.asarray([np.nan,np.nan] if seam is None else seam["applied_command_after"],dtype=float),"seam_signed_pedal":np.asarray([np.nan,np.nan] if seam is None else [seam["signed_pedal_before"],seam["signed_pedal_after"]],dtype=float),"seam_command_available":np.asarray(False if seam is None else seam["requested_command_available"]),"seam_pedal_available":np.asarray(False if seam is None else seam["pedal_available"]),"source_sha256_before":np.asarray(json.dumps(source_before,sort_keys=True)),"source_sha256_after":np.asarray(json.dumps(source_after,sort_keys=True)),"identity":np.asarray(json.dumps(identity,sort_keys=True))}
    raw_path=output_dir/(case_id+"-raw.npz"); row["trace_file"]=raw_path.name; _atomic_npz(raw_path,**arrays); _atomic_json(output_dir/(case_id+".json"),row)
    return row
def run_case(case_id,output_dir,*,duration=8.,wall_seconds=90.,forecast_slabs=60):
    duration,wall_seconds=validate_runner_config(duration=duration,wall_seconds=wall_seconds)
    if isinstance(forecast_slabs,bool) or not isinstance(forecast_slabs,int) or not 1<=forecast_slabs<=60: raise ValueError("forecast_slabs must be an integer in [1, 60]")
    output_dir=Path(output_dir)
    # Own the case destination before assembly or any solver work.  Existing
    # directories may contain prior evidence and are never reused in place.
    output_dir.mkdir(parents=True,exist_ok=False)
    before=_sha(CORE_SOURCE_NAMES); samples=[]
    try: z=_assemble(case_id,duration,forecast_slabs=forecast_slabs)
    except Exception as exc:
        after=_sha(CORE_SOURCE_NAMES)
        _atomic_json(output_dir/(case_id+"-assembly-failure.json"),{"case_id":case_id,"actual_requested_wall_seconds":wall_seconds,"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION,"status":"FAILED_ASSEMBLY","stage":"assembly","error":f"{type(exc).__name__}: {exc}","source_sha256_before":before,"source_sha256_after":after,"source_hashes_match":before==after,"forecast_slabs":forecast_slabs,"production_default_slabs":HybridPolicyConfig().slabs})
        raise
    after_assembly=_sha(CORE_SOURCE_NAMES)
    if "status" in z:return z
    if case_id=="prospective-drive-brake" and z["preflight"].decision.candidate_count<=0: raise RuntimeError("missing preflight rollout")
    samples=[]; kwargs=dict(duration=duration,wall_seconds=wall_seconds,maximum_wall_seconds=wall_seconds,maximum_step=.01,rtol=2e-5,atol=1e-7,method="BDF",initial_pedal=z["pedal"],step_sink=samples.append)
    if case_id.startswith("stop-"):kwargs["stop_speed_mps"]=.01
    try: result=solve_forward(z["plant"],z["driver"],z["initial"],**kwargs)
    except Exception as exc:
        after=_sha(CORE_SOURCE_NAMES); identity={"actual_requested_wall_seconds":wall_seconds,"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION,"forecast_slabs":forecast_slabs,"production_default_slabs":HybridPolicyConfig().slabs,"provider_identity":z["provider"].identity,"plan_identity":z["plan"].identity,"source_sha256_before":before,"source_sha256_after_assembly":after_assembly,"source_sha256_after":after}; artifact_error=None
        try: save_failure_trace(output_dir/(case_id+"-first-failure.npz"),samples=samples,plant=z["plant"],exception=exc,diagnostics=getattr(exc,"diagnostics",{}) or {},source_before=before,source_after=after,identity=identity,stage="solver")
        except Exception as write_exc: artifact_error=f"{type(write_exc).__name__}: {write_exc}"
        if isinstance(exc,ForwardIntegrationError):
            last_s=float(samples[-1].state[0]) if samples else None; last_v=float(z["plant"].observe(samples[-1].state).speed) if samples else None
            suffix=f"; failure_artifact_error={artifact_error}" if artifact_error else ""
            raise RuntimeError(f"{type(exc).__name__}: {exc}; diagnostics={json.dumps(getattr(exc,'diagnostics',{}),default=str)}; accepted_observer_samples={len(samples)}; last_s={last_s}; last_v={last_v}{suffix}") from exc
        if artifact_error: raise RuntimeError(f"{type(exc).__name__}: {exc}; failure_artifact_error={artifact_error}") from exc
        raise
    after=_sha(CORE_SOURCE_NAMES)
    source_match=before==after_assembly==after
    stop_oracle=None
    if case_id.startswith("stop-"):
        stop_oracle=_prospective_stop_distance(z["plan"],z["provider"],z["model"],z["driver"].hybrid_policy.predictor,initial_speed_mps=z["v0"],soc=float(z["initial"][SOC_INDEX]),initial_pedal=z["pedal"])
    stop_actual_distance=None; stop_error_percent=None
    if stop_oracle is not None:
        accepted_speed=np.asarray([z["plant"].observe(sample.state).speed for sample in samples],float)
        arrivals=np.flatnonzero(accepted_speed<=.0100001)
        if len(arrivals):
            stop_actual_distance=float(samples[int(arrivals[0])].state[0]-samples[0].state[0])
            stop_error_percent=100.*abs(stop_oracle["distance_m"]-stop_actual_distance)/max(stop_actual_distance,1e-9)
    identity={"actual_requested_wall_seconds":wall_seconds,"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION,"stop_oracle":stop_oracle,"stop_actual_distance_m":stop_actual_distance,"stop_forecast_actual_error_percent":stop_error_percent,"oracle_basis":None if stop_oracle is None else stop_oracle["fixed_assumptions"],"oracle_explicit_physical_queries":None if stop_oracle is None else stop_oracle["explicit_physical_oracle_queries"],"terminal_fixture":z.get("stop_plan_meta"),"provider_identity":z["provider"].identity,"provider_model_hash":z["provider"].model_hash,"provider_source_closure_hash":z["provider"].source_closure_hash,"plan_identity":z["plan"].identity,"policy_version":__import__("simplified_vehicle_models.solvers.hybrid_driver",fromlist=["HYBRID_POLICY_VERSION"]).HYBRID_POLICY_VERSION,"event_station_m":z["event"],"target_cap_mps":z["target"],"initial_speed_mps":z["v0"],"initial_pedal":z["pedal"],"duration_s":duration,"actual_requested_wall_seconds":wall_seconds,"wall_budget_s":wall_seconds,"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION,"forecast_slabs":forecast_slabs,"production_default_slabs":HybridPolicyConfig().slabs,"source_sha256_before":before,"source_sha256_after_assembly":after_assembly,"source_sha256_after":after,"source_hashes_match":source_match,"assembly_solve_post_hash_match":source_match}
    checkpoint=output_dir/(case_id+"-solver-return.npz")
    try:
        _atomic_npz(checkpoint,**{**_accepted_record_arrays(samples),"forward_time":np.asarray(result.time),"forward_state":np.asarray(result.state),"forward_lap_completed":np.asarray(result.lap_completed),"forward_wall_seconds":np.asarray(result.wall_seconds),"forward_evaluations":np.asarray(result.evaluations),"forward_message":np.asarray(result.message),"identity":np.asarray(json.dumps(identity,sort_keys=True)),"source_sha256_before":np.asarray(json.dumps(before,sort_keys=True)),"source_sha256_after_assembly":np.asarray(json.dumps(after_assembly,sort_keys=True)),"source_sha256_after":np.asarray(json.dumps(after,sort_keys=True)),"source_hashes_match":np.asarray(source_match),"forecast_slabs":np.asarray(forecast_slabs),"actual_requested_wall_seconds":np.asarray(wall_seconds),"validation_resource_contract_version":np.asarray(VALIDATION_RESOURCE_CONTRACT_VERSION),"production_default_slabs":np.asarray(HybridPolicyConfig().slabs)})
        row=_postprocess_forward_result(result,samples,z["plant"],z["plan"],case_id=case_id,v0=z["v0"],target=z["target"],pedal=z["pedal"],event=z["event"],duration=duration,forecast_slabs=forecast_slabs,source_before=before,source_after=after,identity=identity,output_dir=output_dir,source_hashes_match_required=source_match,driver=z["driver"]); row["solver_checkpoint"]=checkpoint.name; return row
    except Exception as exc:
        _atomic_json(output_dir/(case_id+"-postprocessing-failure.json"),{"case_id":case_id,"actual_requested_wall_seconds":wall_seconds,"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION,"status":"FAILED_POSTPROCESSING","stage":"postprocessing","error":f"{type(exc).__name__}: {exc}","solver_checkpoint":checkpoint.name if checkpoint.exists() else None,"identity":identity,"source_sha256_after":_sha(CORE_SOURCE_NAMES)})
        raise RuntimeError(f"postprocessing failed; raw checkpoint {checkpoint}: {type(exc).__name__}: {exc}") from exc
def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument("--case",choices=CASE_IDS,action="append",required=True); parser.add_argument("--output",type=Path,required=True); parser.add_argument("--duration",type=float,default=8.); parser.add_argument("--wall-seconds",type=float,default=90.); parser.add_argument("--forecast-slabs",type=int,default=60,help="experimental per-run slab count; production default remains 60"); a=parser.parse_args(argv)
    if len(set(a.case))!=len(a.case):parser.error("duplicate cases")
    if not 1<=a.forecast_slabs<=60:parser.error("forecast-slabs must be in [1,60]")
    try:validate_runner_config(duration=a.duration,wall_seconds=a.wall_seconds)
    except ValueError as exc:parser.error(str(exc))
    a.output.mkdir(parents=True,exist_ok=False); rows=[]
    for case in a.case:
        d=a.output/case
        try:row=run_case(case,d,duration=a.duration,wall_seconds=a.wall_seconds,forecast_slabs=a.forecast_slabs)
        except Exception as exc:
            row={"case_id":case,"status":"FAIL","error":f"{type(exc).__name__}: {exc}","actual_requested_wall_seconds":a.wall_seconds,"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION}
            try:
                if d.resolve().parent!=a.output.resolve():raise RuntimeError("case output escaped root")
                d.mkdir(parents=True,exist_ok=True); p=d/(case+"-cli-failure.json")
                with p.open("x",encoding="utf-8") as f:f.write(json.dumps(row,indent=2,sort_keys=True)+"\n")
            except FileExistsError:row["failure_artifact_error"]="existing artifact preserved"
            except Exception as artifact_exc:row["failure_artifact_error"]=f"{type(artifact_exc).__name__}: {artifact_exc}"
        rows.append(row);print(json.dumps({"case":case,"status":row["status"],"error":row.get("error")}),flush=True)
        if row["status"] not in ("PASS","EXPECTED_UNSUPPORTED"): break
    payload={"generated_utc":datetime.now(timezone.utc).isoformat(),"actual_requested_wall_seconds":a.wall_seconds,"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION,"max_permitted_wall_seconds":MAX_PERMITTED_WALL_SECONDS,"status":"PASS_SELECTED_CASES" if len(rows)==len(a.case) and all(r["status"] in ("PASS","EXPECTED_UNSUPPORTED") for r in rows) else "FAIL_STOPPED_FIRST","scope":"selected bounded actual cases; no omitted case claimed","rows":rows}; p=a.output/"task-5-transitions-report.json"
    with p.open("x",encoding="utf-8") as f:f.write(json.dumps(payload,indent=2,sort_keys=True)+"\n")
    return 0 if payload["status"]=="PASS_SELECTED_CASES" else 1
if __name__=="__main__":raise SystemExit(main())
