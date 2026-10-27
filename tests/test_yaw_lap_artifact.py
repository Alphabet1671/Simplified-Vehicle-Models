import tempfile
from pathlib import Path
import unittest
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
from simplified_vehicle_models.planning.yaw_lap_artifact import save_lap_plan,load_lap_plan


class LapArtifactTests(unittest.TestCase):
    def fixture(self):
        shape=(2,3,3)
        cap=YawCapabilityMap([0.,10.],[-2.,0.,2.],[-2.,0.,2.],np.full(shape,1e6),np.full(shape,1e6),
            np.ones(shape,bool),1.,'physical_no_tv_v1','map-fixture')
        d=np.array([0.,1.,2.]);p=YawPath(d,np.zeros(3),np.zeros(3),True)
        g=dict(reference_distance=d,north=d,east=np.zeros(3),heading=np.zeros(3))
        return cap,p,g

    def test_save_load_revalidates_exact_profile_and_identities(self):
        cap,p,g=self.fixture()
        with tempfile.TemporaryDirectory() as d:
            target=Path(d)/'plan.npz'
            save_lap_plan(target,yaw_path=p,speed=np.full(3,5.),geometry=g,map_identity=cap.source_identity,
                track_identity='track',maximum_speed=10.,profile_settings=CoupledProfileSettings())
            path,plan,arrays,meta,validation=load_lap_plan(target,cap,expected_track_identity='track')
            self.assertTrue(validation.valid);np.testing.assert_equal(plan.speed,[5.,5.,5.])
            self.assertFalse(arrays['north'].flags.writeable)
            with self.assertRaisesRegex(ValueError,'track identity'):
                load_lap_plan(target,cap,expected_track_identity='other')

    def test_saved_artifact_revalidation_uses_full_profile_contract(self):
        cap,p,g=self.fixture()
        with tempfile.TemporaryDirectory() as d:
            target=Path(d)/'plan.npz'
            save_lap_plan(target,yaw_path=p,speed=np.full(3,5.),geometry=g,map_identity=cap.source_identity,
                track_identity='track',maximum_speed=10.,profile_settings=CoupledProfileSettings())
            path,plan,arrays,meta,validation=load_lap_plan(target,cap,expected_track_identity='track')
            self.assertTrue(validation.valid)
            self.assertEqual(validation.dense_sample_valid, None)

    def test_nonzero_offset_periodic_geometry_is_saved_exactly(self):
        cap,_,_=self.fixture()
        d=np.array([0.,1.,2.]); ref=np.array([10.,11.5,14.])
        path=YawPath(d,np.array([.02,.03,.02]),np.array([.001,.002,.001]),True)
        geom=dict(reference_distance=ref,north=np.array([5.,6.,5.]),east=np.array([2.,3.,2.]),heading=np.array([0.,1.,2*np.pi]))
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)/'offset.npz'
            save_lap_plan(target,yaw_path=path,speed=np.full(3,5.),geometry=geom,
                map_identity=cap.source_identity,track_identity='track',maximum_speed=10.,
                profile_settings=CoupledProfileSettings())
            with np.load(target,allow_pickle=False) as stored:
                np.testing.assert_array_equal(stored['reference_distance'],ref)
                np.testing.assert_array_equal(stored['north'],geom['north'])
                np.testing.assert_array_equal(stored['east'],geom['east'])

    def test_tampered_arrays_rejected_before_replay(self):
        cap,p,g=self.fixture()
        with tempfile.TemporaryDirectory() as d:
            target=Path(d)/'plan.npz'
            save_lap_plan(target,yaw_path=p,speed=np.full(3,5.),geometry=g,map_identity=cap.source_identity,
                track_identity='track',maximum_speed=10.,profile_settings=CoupledProfileSettings())
            with np.load(target) as a:values={k:a[k].copy() for k in a.files}
            values['speed'][1]=6.
            with target.open('wb') as stream:np.savez(stream,**values)
            with self.assertRaisesRegex(ValueError,'integrity'):
                load_lap_plan(target,cap,expected_track_identity='track')
