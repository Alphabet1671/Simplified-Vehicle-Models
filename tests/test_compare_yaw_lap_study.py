import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from tools.run.compare_yaw_lap_study import compare_runs
from unittest.mock import patch
from simplified_vehicle_models.planning.yaw_convergence import ConvergenceSample
from simplified_vehicle_models.planning.yaw_lap_artifact import ARRAYS,_digest,SCHEMA
from tools.run.yaw_study_contract import study_contract


def saved_run(root,name,*,spacing,control,map_identity='map',profile_evidence=True,contract=True,report_time=.2):
    path=root/name; path.mkdir()
    arrays={key:np.array([0.,1.,2.]) for key in ARRAYS}; arrays['speed']=np.array([10.,10.,10.])
    metadata=dict(schema=SCHEMA,array_digest=_digest(arrays),map_identity=map_identity,track_identity='track')
    args={'track':'synthetic','spacing':spacing,'control_spacing':control}
    # plan artifact captures the pre-finalized contract digest, as the runner does.
    preliminary=study_contract(args,{'geometry_spacing_m':spacing,'control_spacing_m':control},{'map_identity':map_identity,'track_identity':'track','map_file_sha256':'maphash','map_source_identity':map_identity,
        'track_data_hashes':{'track':'hash'},'vehicle_configuration_hashes':{'vehicle':'hash'},'boundary_type':'flying','maximum_speed_mps':70.,
        'warm_start_policy':'cold','warm_start_digest':'none','yaw_moment_tolerance':1e-7,'interval_q_tolerance':1e-8,'acceleration_tolerance':1e-7,
        'source_revision':'src','source_dirty_hashes':{'source':'digest'}})
    metadata['provenance']={'study_contract_sha256':preliminary['canonical_json_sha256'],'study_contract_version':'yaw-study-contract-v1'}
    np.savez(path/'plan.npz',**arrays,metadata=np.array(json.dumps(metadata)))
    provenance={'map_identity':map_identity,'track_identity':'track','map_file_sha256':'maphash','map_source_identity':map_identity,
        'track_data_hashes':{'track':'hash'},'vehicle_configuration_hashes':{'vehicle':'hash'},
        'boundary_type':'flying','maximum_speed_mps':70.,'warm_start_policy':'cold','warm_start_digest':'none',
        'yaw_moment_tolerance':1e-7,'interval_q_tolerance':1e-8,'acceleration_tolerance':1e-7,
        'source_revision':'src','source_dirty_hashes':{'source':'digest'},'plan_array_digest':metadata['array_digest'],
        'plan_file_digest':__import__('hashlib').sha256((path/'plan.npz').read_bytes()).hexdigest()}
    manifest={'map_identity':map_identity,'track_identity':'track','args':args}
    if contract: manifest['study_contract']=study_contract(args,{'geometry_spacing_m':spacing,'control_spacing_m':control},provenance)
    (path/'inputs.json').write_text(json.dumps(manifest))
    summary={'status':'planned','lap_time_s':report_time,'plan_array_digest':metadata['array_digest'],
        'profile_resolution_converged':True,'diagnostics':{'kind':'fixed_geometry','profile':{'resolution_converged':True}}}
    if profile_evidence: summary['profile_validation']={'valid':True}
    (path/'summary.json').write_text(json.dumps(summary))
    return path


def loaded_sample(path, *, valid=True, time=.2, spacing=None, control=None, optimizer=None):
    inputs=json.loads((path/'inputs.json').read_text()); a=inputs['args']
    return (ConvergenceSample(path.name,float(spacing or a['spacing']),float(control or a['control_spacing']),time,
        np.array([0.,1.,2.]),np.array([10.,10.,10.]),np.zeros(3),valid,True,optimizer),
        {'run':path.name,'strict_profile_validation':'valid' if valid else 'invalid',
         'plan_array_digest':'array','plan_file_digest':'file','map_identity':'map','track_identity':'track','study_contract_sha256':'contract'})


class SavedStudyTests(unittest.TestCase):
    def test_digest_consistent_physically_invalid_plan_reported_inconclusive(self):
        with tempfile.TemporaryDirectory() as td:
            run=saved_run(Path(td),'bad',spacing=1.,control=10.)
            with patch('tools.run.compare_yaw_lap_study._load_sample',return_value=loaded_sample(run,valid=False)):
                result=compare_runs([run],dimension='geometry')
            self.assertEqual(result['validation_evidence'][0]['strict_profile_validation'],'invalid')
            self.assertEqual(result['status'],'inconclusive')
            # This artifact's array digest is internally consistent; validation is separate evidence.
            with np.load(run/'plan.npz',allow_pickle=False) as artifact:
                self.assertEqual(_digest({key:artifact[key] for key in ARRAYS}),json.loads(str(artifact['metadata'].item()))['array_digest'])

    def test_missing_profile_evidence_is_explicit(self):
        with tempfile.TemporaryDirectory() as td:
            run=saved_run(Path(td),'old',spacing=1.,control=10.,profile_evidence=False)
            with patch('tools.run.compare_yaw_lap_study._load_sample',side_effect=ValueError('missing_profile_evidence')):
                result=compare_runs([run],dimension='geometry')
            self.assertIn('missing_profile_evidence',{r['cause'] for r in result['failed_runs']})

    def test_historical_missing_contract_is_explicit(self):
        with tempfile.TemporaryDirectory() as td:
            run=saved_run(Path(td),'old',spacing=1.,control=10.,contract=False)
            with patch('tools.run.compare_yaw_lap_study._load_sample',side_effect=ValueError('missing_study_contract')):
                result=compare_runs([run],dimension='geometry')
            self.assertIn('missing_study_contract',{r['cause'] for r in result['failed_runs']})

    def test_digest_summary_mismatch_is_explicit(self):
        with tempfile.TemporaryDirectory() as td:
            run=saved_run(Path(td),'old',spacing=1.,control=10.)
            with patch('tools.run.compare_yaw_lap_study._load_sample',side_effect=ValueError('unbound_plan_summary')):
                result=compare_runs([run],dimension='geometry')
            self.assertIn('unbound_plan_summary',{r['cause'] for r in result['failed_runs']})

    def test_cross_map_is_explicit_inconclusive(self):
        with tempfile.TemporaryDirectory() as td:
            one=saved_run(Path(td),'one',spacing=2.,control=10.); two=saved_run(Path(td),'two',spacing=1.,control=10.,map_identity='other')
            with patch('tools.run.compare_yaw_lap_study._load_sample',side_effect=lambda p,d:loaded_sample(p)):
                result=compare_runs([one,two],dimension='geometry')
            self.assertIn('mismatched_map_or_track_identity',{r['cause'] for r in result['failed_runs']})
            self.assertEqual(result['status'],'inconclusive')

    def test_model_exposes_thresholds_and_preserves_pair_failures(self):
        rows=[ConvergenceSample('coarse',4.,10.,10.,np.array([0.,1.]),np.array([5.,5.]),np.zeros(2),True,True,None),
              ConvergenceSample('middle',2.,10.,10.,np.array([0.,1.]),np.array([6.,6.]),np.zeros(2),True,True,None),
              ConvergenceSample('fine',1.,10.,10.,np.array([0.,1.]),np.array([6.,6.]),np.zeros(2),True,True,None)]
        from simplified_vehicle_models.planning.yaw_convergence import convergence_study
        result=convergence_study(rows,dimension='geometry')
        self.assertEqual([c['status'] for c in result['comparisons']],['not_converged','converged'])
        self.assertEqual(result['finest_pair_status'],'converged'); self.assertEqual(result['status'],'not_converged')
        self.assertEqual(result['comparisons'][0]['thresholds']['profile_candidate_time_relative'],.001)
        self.assertIn('piecewise-linear',result['comparisons'][0]['interpolation_rule'])


if __name__=='__main__': unittest.main()
