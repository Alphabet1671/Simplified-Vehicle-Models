import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from tools.run.compare_yaw_lap_study import compare_runs
from unittest.mock import patch
from simplified_vehicle_models.planning.yaw_lap_artifact import ARRAYS,_digest,SCHEMA


def saved_run(root,name,*,spacing,control,map_identity='map',contract=True):
    path=root/name; path.mkdir()
    arrays={key:np.array([0.,1.,2.]) for key in ARRAYS}
    arrays['speed']=np.array([10.,10.,10.])
    metadata=dict(schema=SCHEMA,array_digest=_digest(arrays),map_identity=map_identity,track_identity='track')
    np.savez(path/'plan.npz',**arrays,metadata=np.array(json.dumps(metadata)))
    inputs={'map_identity':map_identity,'track_identity':'track',
        'args':{'track':'synthetic','spacing':spacing,'control_spacing':control}}
    if contract:
        inputs['study_contract']={'arguments':dict(inputs['args']), 'resolved_settings':{},
            'provenance':{'map_identity':map_identity, 'track_identity':'track',
                           'warm_start_policy':'cold', 'source_revision':'unknown',
                           'source_dirty_hashes':'unknown'}, 'contract_version':'yaw-study-contract-v1'}
    (path/'inputs.json').write_text(json.dumps(inputs))
    (path/'summary.json').write_text(json.dumps({'status':'planned','lap_time_s':100.,
        'plan_array_digest':metadata['array_digest'],'diagnostics':{'kind':'fixed_line',
        'profile':{'resolution_converged':True}}}))
    return path


class SavedStudyTests(unittest.TestCase):
    def test_bound_same_map_refinement_converges(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); coarse=saved_run(root,'coarse',spacing=2.,control=10.)
            fine=saved_run(root,'fine',spacing=1.,control=10.)
            fake_reference=type('Reference',(),{'evaluate':lambda self,s:type('E',(),{
                'north':np.zeros_like(s),'east':np.zeros_like(s),'heading':np.zeros_like(s)})()})()
            with patch('simplified_vehicle_models.tracks.profile.load_track',return_value=object()), \
                 patch('simplified_vehicle_models.tracks.line_trajectory.ReferenceCurve',return_value=fake_reference):
                result=compare_runs([coarse,fine],dimension='geometry')
            self.assertEqual(result['status'],'converged')
            self.assertFalse(result['failed_runs'])

    def test_cross_map_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); first=saved_run(root,'one',spacing=2.,control=20.)
            second=saved_run(root,'two',spacing=1.,control=10.,map_identity='other')
            with patch('simplified_vehicle_models.tracks.profile.load_track',return_value=object()), \
                 patch('simplified_vehicle_models.tracks.line_trajectory.ReferenceCurve',return_value=type('R',(),{'evaluate':lambda self,s:type('E',(),{'north':np.zeros_like(s),'east':np.zeros_like(s),'heading':np.zeros_like(s)})()})()):
                with self.assertRaisesRegex(ValueError,'identical track and map'):
                    compare_runs([first,second],dimension='geometry')

    def test_same_grid_is_inconclusive(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); first=saved_run(root,'one',spacing=1.,control=10.)
            second=saved_run(root,'two',spacing=1.,control=10.)
            fake_reference=type('Reference',(),{'evaluate':lambda self,s:type('E',(),{
                'north':np.zeros_like(s),'east':np.zeros_like(s),'heading':np.zeros_like(s)})()})()
            with patch('simplified_vehicle_models.tracks.profile.load_track',return_value=object()), \
                 patch('simplified_vehicle_models.tracks.line_trajectory.ReferenceCurve',return_value=fake_reference):
                result=compare_runs([first,second],dimension='geometry')
            self.assertEqual(result['status'],'inconclusive')

    def test_altered_plan_arrays_rejected_before_geometry_comparison(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'inputs.json').write_text(json.dumps(dict(map_identity='map',track_identity='track')))
            (root/'summary.json').write_text(json.dumps(dict(status='planned')))
            arrays={key:np.array([0.,1.,2.]) for key in ARRAYS}
            metadata=dict(schema=SCHEMA,array_digest=_digest(arrays),map_identity='map',track_identity='track')
            arrays['speed'][1]=10.
            np.savez(root/'plan.npz',**arrays,metadata=np.array(json.dumps(metadata)))
            with self.assertRaisesRegex(ValueError,'integrity'):
                compare_runs([root],dimension='geometry')

    def test_missing_contract_is_inconclusive(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'inputs.json').write_text(json.dumps(dict(map_identity='map',track_identity='track')))
            (root/'summary.json').write_text(json.dumps(dict(status='planned')))
            arrays={key:np.array([0.,1.,2.]) for key in ARRAYS}
            metadata=dict(schema=SCHEMA,array_digest=_digest(arrays),map_identity='map',track_identity='track')
            np.savez(root/'plan.npz',**arrays,metadata=np.array(json.dumps(metadata)))
            result=compare_runs([root],dimension='geometry')
            self.assertEqual(result['status'],'inconclusive')
            self.assertEqual(result['failed_runs'][0]['cause'],'missing_study_contract')

    def test_report_bound_digest_rejects_substituted_complete_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'inputs.json').write_text(json.dumps(dict(map_identity='map',track_identity='track')))
            (root/'summary.json').write_text(json.dumps(dict(status='planned',plan_array_digest='different')))
            arrays={key:np.array([0.,1.,2.]) for key in ARRAYS}
            metadata=dict(schema=SCHEMA,array_digest=_digest(arrays),map_identity='map',track_identity='track')
            np.savez(root/'plan.npz',**arrays,metadata=np.array(json.dumps(metadata)))
            with self.assertRaisesRegex(ValueError,'digest disagree'):
                compare_runs([root],dimension='geometry')
