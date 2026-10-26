import tempfile
import unittest
from pathlib import Path
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.formulations.yaw_map_artifact import save_yaw_map, load_yaw_map


class ArtifactTests(unittest.TestCase):
    def test_integrity_identity_and_completion(self):
        cap = YawCapabilityMap([0., 10.], [-1., 0., 1.], [-20., 0., 20.],
            np.ones((2,3,3)), np.ones((2,3,3)), np.ones((2,3,3), bool), 1., 'physical_no_tv_v1', 'fixture')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.npz'
            save_yaw_map(path, cap, report={'complete': True}, identity='current')
            loaded, metadata = load_yaw_map(path, expected_identity='current')
            np.testing.assert_equal(loaded.lateral_acceleration, [-20.,0.,20.])
            with self.assertRaisesRegex(ValueError, 'stale'):
                load_yaw_map(path, expected_identity='old')
            with np.load(path, allow_pickle=False) as a: arrays = {k:a[k].copy() for k in a.files}
            arrays['yaw_moment_into_turn'][0,0,0] = 4.
            with path.open('wb') as stream: np.savez(stream, **arrays)
            with self.assertRaisesRegex(ValueError, 'integrity'):
                load_yaw_map(path, expected_identity='current')

    def test_incomplete_maps_are_not_accepted_as_cache(self):
        cap = YawCapabilityMap([0., 10.], [-1.,1.], [-20.,20.],
            np.ones((2,2,2)), np.ones((2,2,2)), np.ones((2,2,2), bool), 1., 'physical_no_tv_v1', 'fixture')
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'map.npz'
            save_yaw_map(path, cap, report={'complete':False}, identity='current')
            with self.assertRaisesRegex(ValueError, 'incomplete'):
                load_yaw_map(path, expected_identity='current')
            load_yaw_map(path, expected_identity='current', allow_incomplete=True)
            with self.assertRaises(FileExistsError):
                save_yaw_map(path, cap, report={}, identity='current')
