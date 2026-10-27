"""Compare saved yaw plans and validated exploratory forward replays."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def sha256(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def _array_digest(path):
    try:
        with np.load(path, allow_pickle=False) as z:
            raw = json.loads(str(z['metadata'].item()))
        return raw.get('array_digest')
    except Exception:
        return None


def plan_info(path):
    path=Path(path); inputs=read_json(path.parent/'inputs.json'); summary=read_json(path.parent/'summary.json')
    args=(inputs or {}).get('args') or {}
    meta={}
    try:
        with np.load(path,allow_pickle=False) as z: meta=json.loads(str(z['metadata'].item()))
    except Exception: pass
    plan_prov=meta.get('provenance') or {}
    # Older historical artifacts lack source revision in both NPZ metadata and
    # adjacent inputs.json; newer writers may preserve it in either location.
    plan_inputs_prov=((inputs or {}).get('study_contract') or {}).get('provenance') or {}
    plan_source=(plan_prov.get('source_revision') or plan_prov.get('git_revision') or
                 plan_inputs_prov.get('source_revision') or (inputs or {}).get('source_revision'))
    plan_dirty=(plan_prov.get('source_dirty_hashes') if 'source_dirty_hashes' in plan_prov else
                plan_inputs_prov.get('source_dirty_hashes', (inputs or {}).get('source_dirty_hashes')))
    return {'path':str(path),'exists':path.is_file(),'file_sha256':sha256(path),'array_digest':_array_digest(path),
        'source_revision':plan_source,'source_revision_status':'known' if plan_source not in (None,'','unknown') else 'unknown_or_unavailable',
        'source_dirty_hashes':plan_dirty,
        'lap_time_s':(summary or {}).get('lap_time_s'),'solver_validation':(summary or {}).get('profile_validation',{}).get('valid'),
        'plan_status':(summary or {}).get('status'),'track':args.get('track'),'map_identity':(inputs or {}).get('map_identity'),
        'track_identity':(inputs or {}).get('track_identity'),'inputs':inputs}


def _stats_intervals(time, mask, *, min_duration=0.1):
    mask=np.asarray(mask,bool); time=np.asarray(time,float); intervals=[]
    ix=np.flatnonzero(mask)
    if not len(ix): return intervals
    groups=np.split(ix,np.flatnonzero(np.diff(ix)>1)+1)
    for group in groups:
        start,end=float(time[group[0]]),float(time[group[-1]])
        if end-start>=min_duration: intervals.append({'start_s':start,'end_s':end,'duration_s':end-start})
    return intervals


def _yaw_audit(path):
    path=Path(path); record=path/'run_record.npz'
    if not record.is_file(): return {'status':'unknown','reason':'run_record.npz missing'}
    try:
        from simplified_vehicle_models.planning.yaw_replay_metrics import yaw_approximation_diagnostics,tangential_acceleration_from_body
        from simplified_vehicle_models.results.run_record import load_run_record
        r=load_run_record(record); names=set(r.manifest.channel_names)
        required=('body-slip-angle','speed','yaw-acceleration','acceleration-longitudinal','acceleration-lateral')
        absent=[n for n in required if n not in names]
        if absent: return {'status':'unknown','reason':f'required channels absent: {absent}'}
        inp=read_json(path/'inputs.json') or {}; replay_args=inp.get('args') or {}; plan_path=replay_args.get('plan')
        if not plan_path or not Path(plan_path).is_file(): return {'status':'unknown','reason':'bound plan path unavailable'}
        with np.load(plan_path,allow_pickle=False) as z:
            pd=np.asarray(z['distance'],float); curvature=np.asarray(z['curvature'],float)
        if 's' not in r.manifest.state_names or 'yaw_rate' not in r.manifest.state_names or not all(n in r.manifest.state_names for n in ('v_x','v_y')):
            return {'status':'unknown','reason':'physical station/body velocity/yaw-rate state unavailable'}
        t=np.asarray(r.time,float); sraw=r.state_column('s').astype(float); period=float(r.manifest.track.get('lap_length_m',pd[-1]-pd[0]))
        # Unwrap the recorded station by lifting any periodic decrease into the next lap.
        station=np.unwrap(2*np.pi*(sraw-pd[0])/period)*period/(2*np.pi)+pd[0]
        # If unwrapping starts with a whole-lap phase shift, retain monotone physical coordinate.
        station += round((sraw[0]-station[0])/period)*period
        uniform_t=np.arange(float(t[0]),float(t[-1]),.01)
        if len(uniform_t)<3: return {'status':'unknown','reason':'record too short'}
        def sample(v): return np.interp(uniform_t,t,np.asarray(v,float))
        su=sample(station); vx=sample(r.state_column('v_x')); vy=sample(r.state_column('v_y'))
        rr=sample(r.state_column('yaw_rate')); beta=sample(r.channel_values('body-slip-angle'))
        yaw_acc=sample(r.channel_values('yaw-acceleration')); ax=sample(r.channel_values('acceleration-longitudinal')); ay=sample(r.channel_values('acceleration-lateral'))
        at=tangential_acceleration_from_body(vx,vy,ax,ay)
        wrapped=np.mod(su-pd[0],period)+pd[0]; k=np.interp(wrapped,pd,curvature)
        diag=yaw_approximation_diagnostics(uniform_t,su,vx,vy,rr,yaw_acc,at,k,filter_width=11,lap_length=period)
        for key in ('beta_rad','beta_dot_rad_s','course_rate_rad_s','speed_mps','curvature_prime_per_m2','yaw_acceleration_demand_rad_s2'): diag.pop(key,None)
        # Plant-limit and tracking attribution uses channels/summary already saved.
        channel_names=r.manifest.channel_names
        def channel(name): return np.asarray(r.channel_values(name),float).reshape(len(t),-1)
        limits={}
        if 'throttle' in names: limits['throttle_saturation']=channel('throttle')[:,0]>=1-1e-6
        if 'brake-fraction' in names: limits['brake_saturation']=channel('brake-fraction')[:,0]>=1-1e-6
        # Steering limit value is not a recorded channel nor a replay-input attestation;
        # do not derive a saturation interval from the observed maximum itself.
        plan_feasible=True # strict replay artifact admission binds only validated plan; interval feasibility is sampled plan data.
        # Curvature sign reversals are events, not a continuous interval.
        kval=np.interp(np.mod(station-pd[0],period)+pd[0],pd,curvature)
        reversals=np.flatnonzero(np.sign(kval[1:])!=np.sign(kval[:-1]))+1
        at_native=tangential_acceleration_from_body(r.state_column('v_x'),r.state_column('v_y'),
                r.channel_values('acceleration-longitudinal'),r.channel_values('acceleration-lateral'))
        # Body-frame tire force and aligning-moment channels exist. Sum moments about CG;
        # no yaw inertia is recorded, so equation closure remains unavailable.
        tire_moment=None; moment_reason=[]
        if all(n in names for n in ('tire-force-in-body-x','tire-force-in-body-y','tire-aligning-moment')):
            fx=channel('tire-force-in-body-x'); fy=channel('tire-force-in-body-y'); mz=channel('tire-aligning-moment')
            positions=np.asarray(r.manifest.vehicle_geometry.get('wheel_positions_body_m',[]),float)
            if positions.shape==(4,2): tire_moment=np.sum(positions[:,0]*fy-positions[:,1]*fx+mz,axis=1)
            else: moment_reason.append('vehicle_geometry.wheel_positions_body_m absent or not four 2D points')
        else: moment_reason.append('one or more tire force/alignment-moment channels absent')
        moment={'status':'tire_moment_sum_computed_not_dynamics_closure' if tire_moment is not None else 'unknown',
                'available_channels':['tire-force-in-body-x','tire-force-in-body-y','tire-aligning-moment'],
                'missing_or_limitations':moment_reason + ['yaw inertia I_z is not recorded in run-record channels/vehicle_geometry; cannot compare sum moment to I_z*r_dot'],
                'net_tire_yaw_moment_rms_Nm':float(np.sqrt(np.mean(tire_moment**2))) if tire_moment is not None else None}
        # Large sideslip-rate residual uses an explicit diagnostic threshold (0.05 rad/s),
        # reported as exploratory marker, not a physical acceptance threshold.
        beta_dot=np.gradient(beta,np.diff(uniform_t).mean()); course=rr+beta_dot; vtot=np.hypot(vx,vy)
        large=np.abs(course-vtot*k)>0.05
        attribution={'thresholds':{'absolute_course_rate_residual_rad_s':0.05,'minimum_interval_s':0.1},
          'feasible_plan_assumption':'plan passed strict saved-plan validation; continuous feasibility is not certified',
          'actuator_limit_intervals':{name:_stats_intervals(t,mask) for name,mask in limits.items()},
          'large_sideslip_rate_residual_intervals':_stats_intervals(uniform_t,large),
          'curvature_reversal_events_s':[float(t[i]) for i in reversals],
          'tracking_failure_intervals':'unavailable: replay record does not persist pointwise reference speed/lateral error; aggregate tracking metrics are in summary.json',
          'unavailable_actuator_limit_intervals':{'steering_rate':'steering maximum-rate limit absent from replay inputs/recorded channels'},
          'correlation_with_curvature_reversals':'events and intervals are separately listed; no causal/correlation claim because pointwise tracking error is not persisted',
          'correlation_computed':False}
        diag.update(status='computed',resampling={'kind':'uniform_time_for_beta_and_unwrapped_station','dt_s':.01,
             'curvature_derivative':'nonuniform periodic three-point physical-distance formula','filter_width_samples':11,
             'filter_duration_s':.11,'filter_kind':'periodic_boxcar'}, tangential_acceleration_source='projection of body acceleration [ax,ay] onto [vx,vy] divided by total speed',
             force_moment_consistency=moment,interval_attribution=attribution)
        return diag
    except Exception as e:
        return {'status':'unknown','reason':f'{type(e).__name__}: {e}'}


def replay_info(path, plan, *, expected_track=None, expected_map=None, plan_digest=None,
                plan_source_revision=None, plan_dirty_hashes=None):
    path=Path(path); summary=read_json(path/'summary.json'); inputs=read_json(path/'inputs.json')
    if summary is None: return {'path':str(path),'plan':str(plan),'status':'missing','exploratory_completion':False,'comparison_status':'inconclusive','identity_checks':{'status':'unknown','reason':'summary/inputs missing'},'yaw_approximation_audit':_yaw_audit(path)}
    args=(inputs or {}).get('args') or {}; provenance=((inputs or {}).get('study_contract') or {}).get('provenance') or {}
    replay_revision=provenance.get('source_revision')
    plan_revision_known=plan_source_revision not in (None,'','unknown')
    replay_revision_known=replay_revision not in (None,'','unknown')
    dirty_match=(plan_dirty_hashes == provenance.get('source_dirty_hashes')) if plan_dirty_hashes is not None else None
    checks={'track_match':args.get('track')==expected_track,'map_identity_match':(inputs or {}).get('map_identity')==expected_map,
            'plan_path_match':Path(args.get('plan','')).resolve()==Path(plan).resolve(),
            'plan_array_digest_match':provenance.get('plan_array_digest')==plan_digest if plan_digest else False,
            'plan_file_digest_match':provenance.get('plan_file_digest')==sha256(plan) if sha256(plan) else False,
            'plan_source_revision':plan_source_revision,'replay_source_revision':replay_revision,
            'plan_source_revision_known':plan_revision_known,'replay_source_revision_known':replay_revision_known,
            'source_revision_match':bool(plan_revision_known and replay_revision_known and plan_source_revision==replay_revision),
            'plan_source_dirty_hashes':plan_dirty_hashes,'replay_source_dirty_hashes':provenance.get('source_dirty_hashes'),
            'source_dirty_identity_match':dirty_match}
    required_checks=[v for k,v in checks.items() if k not in ('plan_source_revision','replay_source_revision','plan_source_dirty_hashes','replay_source_dirty_hashes','source_dirty_identity_match')]
    if dirty_match is not None: required_checks.append(dirty_match)
    checks['status']='pass' if all(required_checks) else 'inconclusive_or_mismatch'
    fv=summary.get('footprint_validation') or {}
    row={k:summary.get(k) for k in ('driven_time_s','speed_error_rms_mps','speed_error_max_abs_mps','lateral_error_rms_m','lateral_error_max_m','throttle_saturation_time_fraction_realized','brake_saturation_time_fraction_realized','steering_rate_saturation_time_fraction_realized','solver_validation_complete','sampled_center_contained','between_step_containment_certified')}
    row.update(path=str(path),plan=str(plan),status=summary.get('status'),lap_completed=summary.get('lap_completed'),
       solver_validation_status='complete' if summary.get('solver_validation_complete') else 'incomplete_or_unknown',center_containment=summary.get('sampled_center_contained'),
       footprint_containment=fv.get('status'),wheel_contact_containment=(fv.get('wheel_contact') or {}).get('status'),between_step_containment=summary.get('between_step_containment_certified'),
       replay_inputs=inputs,identity_checks=checks,exploratory_completion=bool(summary.get('lap_completed') and summary.get('solver_validation_complete')),
       comparison_status='completed_exploratory' if summary.get('lap_completed') and summary.get('solver_validation_complete') and checks['status']=='pass' else 'inconclusive',yaw_approximation_audit=_yaw_audit(path))
    return row


def build_case(spec):
    parts=spec.split(':',5)
    if len(parts)!=6: raise ValueError('case NAME:TRACK:OPT_PLAN:FIXED_PLAN:OPT_REPLAY:FIXED_REPLAY required')
    name,track,opt,fixed,opt_run,fixed_run=parts
    op=plan_info(opt); fp=plan_info(fixed)
    expected_map=op['map_identity'] if op['map_identity']==fp['map_identity'] else None
    expected_track=track
    optrow=replay_info(opt_run,opt,expected_track=expected_track,expected_map=expected_map,plan_digest=op['array_digest'],
                       plan_source_revision=op['source_revision'],plan_dirty_hashes=op['source_dirty_hashes'])
    fixedrow=replay_info(fixed_run,fixed,expected_track=expected_track,expected_map=expected_map,plan_digest=fp['array_digest'],
                         plan_source_revision=fp['source_revision'],plan_dirty_hashes=fp['source_dirty_hashes'])
    pair_ok=bool(op['exists'] and fp['exists'] and op['solver_validation'] is True and fp['solver_validation'] is True and
        op['track']==track and fp['track']==track and expected_map and op['track_identity']==fp['track_identity'] and
        optrow.get('identity_checks',{}).get('status')=='pass' and fixedrow.get('identity_checks',{}).get('status')=='pass')
    return {'name':name,'track':track,'optimized_plan':op,'fixed_plan':fp,'optimized_replay':optrow,'fixed_replay':fixedrow,
       'pairing':{'fixed_grid':'matched historical 2 m fixed exploratory plan','track_match':track,'identity_status':'pass' if pair_ok else 'inconclusive_or_mismatch',
                  'unknown_provenance_is_inconclusive':True}}


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',action='append',default=[]);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);data={'schema_version':2,'scope':'historical exploratory replays only; no geometry selection or optimizer convergence claim','cases':[build_case(c) for c in a.case]}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');return 0
if __name__=='__main__':raise SystemExit(main())
