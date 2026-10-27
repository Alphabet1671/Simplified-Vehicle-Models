import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from tools.run import run_yaw_lap_suite as suite

class ReplayBindingTests(unittest.TestCase):
    def test_saved_nonzero_offset_geometry_and_speed_bind_without_replanning(self):
        distance=np.array([0.,2.,5.,9.]); reference=np.array([0.,2.2,5.4,9.6])
        speed=np.array([4.,5.,6.,4.]); k=np.array([.02,.03,.04,.02]); kr=np.array([.001,.002,.003,.001])
        path=SimpleNamespace(distance=distance,curvature=k,curvature_rate=kr,is_closed=True)
        speed_plan=SimpleNamespace(distance=distance,speed=speed,curvature=k,is_closed=True)
        arrays=dict(north=np.array([2.,2.4,3.,2.]),east=np.array([0.,2.,5.,0.]),
                    heading=np.array([0.,.4,1.,2*np.pi]),reference_distance=reference)
        captured={}
        class Plant:
            def __init__(self,vehicle,plan):
                self.vehicle=vehicle; self.state_names=('v_x',); captured['speed']=plan.speed.copy(); captured['plant_plan']=plan
            def initial_state(self,speed): return np.array([speed])
        class Sink:
            def __init__(self,plant,geometry,*a,**kw): captured['geometry']=geometry
            def close(self,*a,**kw): pass
        def stop(*a,**kw):
            captured['sink_called']=kw['step_sink']
            kw['diagnostics_sink']({'counts':{'accepted_states_validated':1,'dense_probes_validated':1}})
            raise RuntimeError('stub stop')
        vehicle=SimpleNamespace(wheelbase=2.5,steering_maximum_angle=.5,steering_maximum_rate=.5,maximum_pedal_force=1000.)
        args=SimpleNamespace(controller='preview',duration=1.,replay_seconds=1.,maximum_step=.05,
             preview_seconds=None,heading_gain=None,lateral_gain=None,yaw_damping=None)
        with tempfile.TemporaryDirectory() as td, patch.object(suite,'__name__',suite.__name__), \
             patch('simplified_vehicle_models.solvers.planar_forward.PlanarForwardPlant',Plant), \
             patch('simplified_vehicle_models.results.plant_channels.RunRecordSink',Sink), \
             patch('simplified_vehicle_models.solvers.forward_driver.solve_forward',stop), \
             patch('tools.run.yaw_study_contract.update_replay_initial_state'), \
             patch('simplified_vehicle_models.planning.yaw_aware_line.optimize_yaw_aware_line',side_effect=AssertionError('optimizer called')), \
             patch('simplified_vehicle_models.solvers.yaw_aware_profile.yaw_aware_profile',side_effect=AssertionError('fixed builder called')):
            with self.assertRaisesRegex(RuntimeError,'stub stop'):
                suite.replay(args,Path(td),vehicle,object(),path,speed_plan,arrays,{'array_digest':'abc'})
        np.testing.assert_array_equal(captured['speed'],speed)
        np.testing.assert_array_equal(captured['geometry'].distance,distance)
        np.testing.assert_array_equal(captured['geometry'].x_north,arrays['north'])
        np.testing.assert_array_equal(captured['geometry'].y_east,arrays['east'])
        np.testing.assert_array_equal(captured['geometry'].heading,arrays['heading'])
        np.testing.assert_array_equal(captured['geometry'].reference_distance,reference)
        np.testing.assert_array_equal(captured['geometry'].curvature,k)

if __name__=='__main__': unittest.main()
