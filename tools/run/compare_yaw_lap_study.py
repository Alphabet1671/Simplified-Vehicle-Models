"""Compare saved lap-suite runs without repeating planning or replay."""
import argparse
import json
from pathlib import Path
import numpy as np
from simplified_vehicle_models.planning.yaw_convergence import ConvergenceSample,convergence_study
from tools.run.yaw_study_contract import comparison_identity, validate_comparison_pair


def compare_runs(directories,*,dimension):
    rows=[];failures=[];identities=set();contracts=[]
    for directory in map(Path,directories):
        summary=json.loads((directory/'summary.json').read_text())
        inputs=json.loads((directory/'inputs.json').read_text())
        identities.add((inputs['map_identity'],inputs['track_identity']))
        contract=inputs.get('study_contract')
        if contract is None:
            failures.append({'run':str(directory),'status':'unbound_historical_contract',
                             'cause':'missing_study_contract'})
        else:
            contracts.append(contract)
        if summary['status']!='planned':
            failures.append({'run':str(directory),'status':summary['status'],'cause':summary.get('cause'),
                             'message':summary.get('message')});continue
        with np.load(directory/'plan.npz',allow_pickle=False) as artifact:
            metadata=json.loads(str(artifact['metadata'].item()))
            from simplified_vehicle_models.planning.yaw_lap_artifact import _digest,ARRAYS,SCHEMA
            if metadata.get('schema')!=SCHEMA or _digest({key:artifact[key] for key in ARRAYS})!=metadata.get('array_digest'):
                raise ValueError('lap plan array integrity mismatch')
            if 'plan_array_digest' not in summary:
                failures.append({'run':str(directory),'status':'unbound_legacy_plan',
                                 'cause':'missing_plan_array_digest'})
                continue
            if summary['plan_array_digest']!=metadata['array_digest']:
                raise ValueError('summary and plan artifact digest disagree')
            if (metadata['map_identity'],metadata['track_identity'])!=(inputs['map_identity'],inputs['track_identity']):
                raise ValueError('plan and input provenance disagree')
            if summary['diagnostics']['kind']=='optimized_line':
                diag=summary['diagnostics']['diagnostics'];optimizer_converged=bool(diag['converged'])
                budget=diag.get('search_stop_reason');profile=diag['profile_diagnostics']
            else:
                optimizer_converged=True;profile=summary['diagnostics']['profile'];budget=None
            # Physical line displacement relative to the reference is not a
            # control coefficient: use saved geometry and source track reference.
            from simplified_vehicle_models.tracks.profile import load_track
            from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
            reference=ReferenceCurve(load_track(inputs['args']['track']))
            ref=reference.evaluate(artifact['reference_distance'])
            offset=(artifact['north']-ref.north)*(-np.sin(ref.heading))+(artifact['east']-ref.east)*np.cos(ref.heading)
            rows.append(ConvergenceSample(directory.name,inputs['args']['spacing'],inputs['args']['control_spacing'],
                summary['lap_time_s'],artifact['reference_distance'],artifact['speed'],offset,
                True,bool(profile.get('resolution_converged',False)),optimizer_converged,
                budget or profile.get('budget_reason')))
    if len(identities)>1:raise ValueError('study runs must have identical track and map identities')
    if len(contracts) != len(directories):
        failures.append({'status':'unbound_historical_contract','cause':'mixed_or_missing_study_contract'})
    for index, first in enumerate(contracts):
        for second in contracts[index+1:]:
            validate_comparison_pair(first,second,dimension)
    result=convergence_study(rows,dimension=dimension)
    if contracts and len(contracts) == len(directories):
        result['comparison_identity']=comparison_identity(contracts[0],dimension)
    result['failed_runs']=failures
    if failures:result['status']='inconclusive'
    result['scope']='saved discrete plans; no new capability/geometry validation performed by comparison'
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
