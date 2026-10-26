import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from tools.run.compare_yaw_lap_study import compare_runs
from simplified_vehicle_models.planning.yaw_lap_artifact import ARRAYS,_digest,SCHEMA


class SavedStudyTests(unittest.TestCase):
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

    def test_legacy_unbound_report_is_inconclusive(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'inputs.json').write_text(json.dumps(dict(map_identity='map',track_identity='track')))
            (root/'summary.json').write_text(json.dumps(dict(status='planned')))
            arrays={key:np.array([0.,1.,2.]) for key in ARRAYS}
            metadata=dict(schema=SCHEMA,array_digest=_digest(arrays),map_identity='map',track_identity='track')
            np.savez(root/'plan.npz',**arrays,metadata=np.array(json.dumps(metadata)))
            result=compare_runs([root],dimension='geometry')
            self.assertEqual(result['status'],'inconclusive')
            self.assertEqual(result['failed_runs'][0]['cause'],'missing_plan_array_digest')

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
