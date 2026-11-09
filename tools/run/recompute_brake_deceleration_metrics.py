"""Recompute settled gates from saved state-aware NPZ accepted-state traces."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np
from validate_brake_deceleration import settled_metrics

ROOT=Path(__file__).resolve().parents[2]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trace-dir',type=Path,required=True); ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args(); a.trace_dir=(ROOT/a.trace_dir).resolve(); prior=ROOT/'.superpowers/sdd/implementation-plan/steady-deceleration-stateaware.json'
    base=json.loads(prior.read_text()); rows=[]
    for old in base['rows']:
        target=old['target_deceleration_mps2']; pedal=old['initial_pedal']
        trace=a.trace_dir/f'brake-decel-{target:g}-{pedal:g}.npz'
        with np.load(trace) as d:
            t=d['time']; force_req=d['requested_brake_force_N']; force_app=d['applied_brake_force_N']; decel=d['measured_deceleration_mps2']
        try:
            met=settled_metrics(t,force_req,force_app,decel,target)
            row={**old,**met,'pass':bool(met['tracking_pass'] and old['all_states_finite'] and old['terminal_speed_mps']>0.),
                 'metrics_recomputed_from_trace':str(trace.relative_to(ROOT))}
        except ValueError as error:
            row={**old,'pass':False,'tracking_pass':False,'settling_failure':str(error),
                 'metrics_recomputed_from_trace':str(trace.relative_to(ROOT))}
        rows.append(row)
    base['rows']=rows; base['pass']=all(r.get('pass',False) for r in rows)
    base['settling_rule'].update({'contiguous_eligible_window':True,'max_absolute_deceleration_error_gate':True,
                                  'recomputed_without_plant_rerun':True})
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(base,indent=2)+'\n')
    print(json.dumps(base,indent=2))
if __name__=='__main__': main()
