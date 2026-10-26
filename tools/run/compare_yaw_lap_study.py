"""Compare saved lap-suite runs without repeating planning or replay."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from simplified_vehicle_models.planning.yaw_convergence import ConvergenceSample,convergence_study
from tools.run.yaw_study_contract import comparison_identity, validate_comparison_pair, study_contract


def _load_sample(directory, dimension):
    from simplified_vehicle_models.planning.yaw_lap_artifact import load_lap_plan
    from tools.run import run_yaw_aware_forward_driver as runner
    from tools.run.yaw_map_options import resolve_yaw_map
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.tracks.profile import load_track
    from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
    d=Path(directory); summary=json.loads((d/'summary.json').read_text()); inputs=json.loads((d/'inputs.json').read_text())
    if summary.get('status')!='planned': raise ValueError('run_not_planned:'+str(summary.get('cause') or summary.get('status')))
    contract=inputs.get('study_contract')
    if not isinstance(contract,dict): raise ValueError('missing_study_contract')
    provenance=contract.get('provenance',{})
    if not provenance.get('map_file_sha256') or not provenance.get('plan_array_digest') or not provenance.get('plan_file_digest'):
        raise ValueError('unbound_plan_summary')
    if summary.get('plan_array_digest')!=provenance['plan_array_digest']:
        raise ValueError('unbound_plan_summary')
    plan_path=d/'plan.npz'
    from simplified_vehicle_models.planning.yaw_lap_artifact import _digest, ARRAYS, SCHEMA
    with np.load(plan_path,allow_pickle=False) as artifact:
        metadata=json.loads(str(artifact['metadata'].item()))
        raw={key:np.array(artifact[key],float,copy=True) for key in ARRAYS}
    if metadata.get('schema')!=SCHEMA or _digest(raw)!=metadata.get('array_digest'):
        raise ValueError('plan_array_integrity_mismatch')
    station=raw['reference_distance']
    if not np.all(np.isfinite(station)) or np.any(np.diff(station)<=0): raise ValueError('duplicate_or_invalid_reference_spacing')
    if any(not np.all(np.isfinite(value)) for value in raw.values()): raise ValueError('nonfinite_plan_samples')
    if hashlib.sha256(plan_path.read_bytes()).hexdigest()!=provenance['plan_file_digest']:
        raise ValueError('plan_source_digest_mismatch')
    if summary.get('plan_array_digest')!=provenance['plan_array_digest']:
        raise ValueError('unbound_plan_summary')
    expected=contract.get('canonical_json_sha256')
    rebuilt=study_contract(contract.get('arguments',{}),contract.get('resolved_settings',{}),provenance)
    if not expected or rebuilt['canonical_json_sha256']!=expected: raise ValueError('study_contract_digest_mismatch')
    profile=summary.get('profile_validation')
    if not isinstance(profile,dict): raise ValueError('missing_profile_evidence')
    if not isinstance(profile.get('validation'),dict) and not isinstance(profile.get('valid'),bool):
        raise ValueError('unbound_profile_evidence')
    contract_digest=contract.get('canonical_json_sha256')
    artifact_evidence=metadata.get('provenance',{})
    if artifact_evidence.get('study_contract_sha256')!=contract_digest:
        raise ValueError('unbound_profile_evidence')
    args=inputs['args']; track_path=Path(args['track'])
    contract_args=contract.get('arguments',{})
    for key in ('track','spacing','control_spacing'):
        if key in args and key in contract_args and args[key]!=contract_args[key]:
            raise ValueError('study_contract_argument_mismatch:'+key)
    vehicle=compile_formulation(str(runner.PLANAR_FORMULATION)).model; pm=point_mass_view_of_planar(vehicle); vmax=runner.physical_map_speed_limit(pm)
    class A: pass
    map_args=A()
    for key,value in args.items(): setattr(map_args,key,value)
    map_args.yaw_map=Path(args['yaw_map'])
    if hashlib.sha256(map_args.yaw_map.read_bytes()).hexdigest()!=provenance.get('map_file_sha256'):
        raise ValueError('map_source_digest_mismatch')
    capability,_=resolve_yaw_map(pm,vehicle.yaw_inertia,np.linspace(0.,vmax,12),runner.anchored_axis(args.get('yaw_map_ax_min',-6.),args.get('yaw_map_ax_max',3.),9),map_args)
    track=load_track(track_path)
    from simplified_vehicle_models.formulations.yaw_map_artifact import source_provenance
    track_identity=hashlib.sha256(json.dumps(source_provenance(str(track_path)),sort_keys=True).encode()).hexdigest()
    if capability.source_identity!=inputs.get('map_identity') or track_identity!=inputs.get('track_identity'):
        raise ValueError('run_map_or_track_identity_mismatch')
    try:
        _,_,arrays,metadata,validation=load_lap_plan(plan_path,capability,expected_track_identity=track_identity)
    except ValueError as error:
        if 'strict profile validation' in str(error): raise ValueError('profile_invalid') from error
        raise
    if metadata['array_digest']!=provenance['plan_array_digest']: raise ValueError('plan_digest_binding_mismatch')
    if metadata['map_identity']!=provenance.get('map_identity') or metadata['track_identity']!=provenance.get('track_identity'):
        raise ValueError('plan_source_digest_mismatch')
    diagnostic=summary.get('diagnostics',{}); kind=diagnostic.get('kind')
    optimizer=bool(diagnostic.get('diagnostics',{}).get('converged')) if kind=='optimized_line' else None
    reported=float(summary['lap_time_s']); recomputed=float(np.trapezoid(1./arrays['speed'],arrays['distance']))
    if not math.isfinite(reported) or not math.isclose(reported,recomputed,rel_tol=2e-5,abs_tol=2e-5):
        raise ValueError('reported_lap_time_mismatch')
    reference=ReferenceCurve(track); ref=reference.evaluate(arrays['reference_distance'])
    offset=(arrays['north']-ref.north)*(-np.sin(ref.heading))+(arrays['east']-ref.east)*np.cos(ref.heading)
    sample=ConvergenceSample(d.name,float(args['spacing']),float(args['control_spacing']),reported,
        arrays['reference_distance'],arrays['speed'],offset,bool(validation.valid),
        bool(summary.get('profile_resolution_converged',False)),optimizer,
        diagnostic.get('diagnostics',{}).get('search_stop_reason') or profile.get('budget_reason'))
    evidence={'run':d.name,'plan_array_digest':metadata['array_digest'],'plan_file_digest':provenance['plan_file_digest'],
        'map_identity':metadata['map_identity'],'track_identity':metadata['track_identity'],
        'study_contract_sha256':expected,'strict_profile_validation':'valid' if validation.valid else 'invalid'}
    return sample,evidence


def compare_runs(directories,*,dimension):
    paths=list(map(Path,directories)); rows=[];failures=[];contracts=[];identities=set();evidence=[]
    for directory in paths:
        try:
            inputs=json.loads((directory/'inputs.json').read_text()); identities.add((inputs.get('map_identity'),inputs.get('track_identity')))
            contract=inputs.get('study_contract')
            if isinstance(contract,dict): contracts.append((directory,contract))
            sample,record=_load_sample(directory,dimension);rows.append(sample);evidence.append(record)
        except Exception as error:
            failures.append({'run':str(directory),'status':'inconclusive','cause':str(error)})
    pair_errors=[]
    if len(identities)>1:
        pair_errors.append('study runs must have identical track and map identities')
        failures.append({'status':'inconclusive','cause':'mismatched_map_or_track_identity'})
    if len(contracts)!=len(paths):
        bound={p for p,_ in contracts}
        failures.extend({'run':str(p),'status':'inconclusive','cause':'missing_study_contract'} for p in paths if p not in bound)
    for i,(p,a) in enumerate(contracts):
        for q,b in contracts[i+1:]:
            try: validate_comparison_pair(a,b,dimension)
            except ValueError as error:
                pair_errors.append(str(error)); failures.append({'runs':[str(p),str(q)],'status':'inconclusive','cause':str(error)})
    result=convergence_study(rows,dimension=dimension)
    if contracts and len(contracts)==len(paths):
        try: result['comparison_identity']=comparison_identity(contracts[0][1],dimension)
        except ValueError as error: failures.append({'status':'inconclusive','cause':str(error)})
    result['validation_evidence']=evidence
    result['failed_runs']=failures
    if failures or pair_errors: result['status']='inconclusive'
    result['scope']='saved discrete plans; fresh strict profile validation against exact map and track; no continuous feasibility certification'
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs',nargs='+',type=Path);p.add_argument('--dimension',choices=('geometry','control'),required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args(argv)
    if args.output.exists():p.error('output already exists')
    result=compare_runs(args.runs,dimension=args.dimension)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as out:json.dump(result,out,indent=2,allow_nan=False);out.write('\n')
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
