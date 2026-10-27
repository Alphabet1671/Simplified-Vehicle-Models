"""Compare saved yaw plans and validated exploratory forward replays.

Examples are documented in Task 9 report. Missing artifacts remain unknown.
"""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np


def read_json(path):
    try: return json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError): return None


def plan_info(path):
    path=Path(path); inputs=read_json(path.parent/'inputs.json'); summary=read_json(path.parent/'summary.json')
    return {'path':str(path),'exists':path.is_file(),'lap_time_s':(summary or {}).get('lap_time_s'),
        'solver_validation':(summary or {}).get('profile_validation',{}).get('valid'),
        'plan_status':(summary or {}).get('status'),'inputs':inputs}


def _yaw_audit(path):
    path=Path(path); record=path/'run_record.npz'
    if not record.is_file(): return {'status':'unknown','reason':'run_record.npz missing'}
    try:
        from simplified_vehicle_models.planning.yaw_replay_metrics import yaw_approximation_diagnostics
        from simplified_vehicle_models.results.run_record import load_run_record
        r=load_run_record(record); names=r.manifest.channel_names
        # physical distance and body-frame beta from accepted states/plant channels
        idx={name:i for i,name in enumerate(names)}
        needed=('body-slip-angle','speed','yaw-acceleration','acceleration-longitudinal')
        if any(k not in idx for k in needed): return {'status':'unknown','reason':'required yaw audit channels absent'}
        inp=read_json(path/'inputs.json') or {}; plan_path=(inp.get('args') or {}).get('plan')
        if not plan_path or not Path(plan_path).is_file(): return {'status':'unknown','reason':'bound plan path unavailable'}
        with np.load(plan_path,allow_pickle=False) as z:
            pd=np.asarray(z['distance'],float); curvature=np.asarray(z['curvature'],float)
        state_names=r.manifest.state_names
        if 's' not in state_names: return {'status':'unknown','reason':'physical station state unavailable'}
        station=r.state_column('s').astype(float)
        beta=r.channel_values('body-slip-angle').astype(float); speed=r.channel_values('speed').astype(float)
        yaw_acc=r.channel_values('yaw-acceleration').astype(float); at=r.channel_values('acceleration-longitudinal').astype(float)
        period=float(pd[-1]-pd[0]); station=np.mod(station-pd[0],period)+pd[0]
        uniform_t=np.arange(float(r.time[0]),float(r.time[-1]),.01)
        if len(uniform_t)<3: return {'status':'unknown','reason':'record too short'}
        s=np.interp(uniform_t,r.time,station); v=np.interp(uniform_t,r.time,speed)
        b=np.interp(uniform_t,r.time,beta); rr=np.interp(uniform_t,r.time,r.state_column('yaw_rate'))
        ra=np.interp(uniform_t,r.time,yaw_acc); aa=np.interp(uniform_t,r.time,at)
        k=np.interp(s,pd,curvature)
        diag=yaw_approximation_diagnostics(uniform_t,s,v*np.cos(b),v*np.sin(b),rr,ra,aa,k,filter_width=11)
        diag.pop('beta_rad',None);diag.pop('beta_dot_rad_s',None);diag.pop('course_rate_rad_s',None);diag.pop('speed_mps',None)
        diag.update(status='computed',resampling={'kind':'uniform_time_then_physical_station_curvature','dt_s':.01,'filter_width_samples':11,'filter_kind':'periodic_boxcar'},
                    force_moment_consistency='unknown: total body-force/moment residual accounting not yet implemented')
        return diag
    except Exception as e:
        return {'status':'unknown','reason':str(e)}


def replay_info(path, plan):
    path=Path(path); summary=read_json(path/'summary.json'); inputs=read_json(path/'inputs.json')
    if summary is None: return {'path':str(path),'plan':str(plan),'status':'missing','comparison_status':'inconclusive','yaw_approximation_audit':_yaw_audit(path)}
    fv=summary.get('footprint_validation') or {}
    row={k:summary.get(k) for k in ('driven_time_s','speed_error_rms_mps','speed_error_max_abs_mps','lateral_error_rms_m','lateral_error_max_m',
      'throttle_saturation_time_fraction_realized','brake_saturation_time_fraction_realized','steering_rate_saturation_time_fraction_realized','solver_validation_complete','sampled_center_contained','between_step_containment_certified')}
    row.update(path=str(path),plan=str(plan),status=summary.get('status'),lap_completed=summary.get('lap_completed'),
      solver_validation_status='complete' if summary.get('solver_validation_complete') else 'incomplete_or_unknown',
      center_containment=summary.get('sampled_center_contained'),footprint_containment=fv.get('status'),
      wheel_contact_containment=(fv.get('wheel_contact') or {}).get('status'),between_step_containment=summary.get('between_step_containment_certified'),
      replay_inputs=inputs,comparison_status='completed' if summary.get('lap_completed') and summary.get('solver_validation_complete') else 'inconclusive',
      yaw_approximation_audit=_yaw_audit(path))
    return row


def build_case(spec):
    parts=spec.split(':',5)
    if len(parts)!=6: raise ValueError('case NAME:TRACK:OPT_PLAN:FIXED_PLAN:OPT_REPLAY:FIXED_REPLAY required')
    name,track,opt,fixed,opt_run,fixed_run=parts
    return {'name':name,'track':track,'optimized_plan':plan_info(opt),'fixed_plan':plan_info(fixed),
      'optimized_replay':replay_info(opt_run,opt),'fixed_replay':replay_info(fixed_run,fixed),
      'pairing':{'fixed_grid':'matched historical 2 m fixed exploratory plan','track_match':track,'unknown_provenance_is_inconclusive':True}}


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',action='append',default=[]);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);data={'schema_version':1,'scope':'historical exploratory replays only; no geometry selection or optimizer convergence claim','cases':[build_case(c) for c in a.case]}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');return 0
if __name__=='__main__':raise SystemExit(main())
