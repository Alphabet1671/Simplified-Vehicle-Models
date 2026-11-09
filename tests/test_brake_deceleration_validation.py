import unittest
import numpy as np

from tools.run.validate_brake_deceleration import settled_metrics, state_aware_force
from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext, BrakeAuthorityPredictor
from simplified_vehicle_models.compiler.compile import compile_formulation


class BrakeDecelerationValidationTests(unittest.TestCase):
    def test_metrics_use_same_predeclared_settled_window(self):
        t=np.arange(0.,1.01,.05)
        req=np.full(len(t),200.)
        applied=req.copy()
        decel=np.full(len(t),3.1)
        result=settled_metrics(t,req,applied,decel,3.)
        self.assertTrue(result['tracking_pass'])
        self.assertAlmostEqual(result['measured_deceleration_mean_mps2'],3.1)
        self.assertGreaterEqual(result['settled_samples'],2)
        self.assertGreaterEqual(result['settled_duration_s'],.2)

    def test_isolated_force_gap_cannot_be_bridged(self):
        t=np.arange(0.,.81,.1); req=np.full(len(t),100.); app=req.copy(); app[5]=80.
        with self.assertRaisesRegex(ValueError,'contiguous settled window'):
            settled_metrics(t,req,app,np.full(len(t),3.),3.)

    def test_contiguous_eligible_run_passes_after_short_gap(self):
        t=np.arange(0.,.91,.1); req=np.full(len(t),100.); app=req.copy(); app[4]=80.
        out=settled_metrics(t,req,app,np.full(len(t),3.),3.)
        self.assertTrue(out['tracking_pass'])
        self.assertAlmostEqual(out['settled_duration_s'],.4)

    def test_max_error_gate_rejects_outlier_when_mean_passes(self):
        t=np.arange(0.,.81,.1); req=np.full(len(t),100.); app=req.copy()
        decel=np.full(len(t),3.); decel[5]=4.0
        out=settled_metrics(t,req,app,decel,3.)
        self.assertTrue(out['mean_tracking_pass'])
        self.assertFalse(out['maximum_error_pass'])
        self.assertFalse(out['tracking_pass'])

    def test_empty_window_is_hard_failure(self):
        t=np.array([0.,.35,.45]); req=np.full(3,100.)
        with self.assertRaisesRegex(ValueError,'contiguous settled window'):
            settled_metrics(t,req,req,np.full(3,3.),3.)

    @classmethod
    def setUpClass(cls):
        cls.vehicle=compile_formulation('models/formulations/iem24_murphy_real_planar_7dof.xml').model
        cls.predictor=BrakeAuthorityPredictor(cls.vehicle)

    def test_state_aware_inverse_matches_physical_inverse_at_context(self):
        v=self.vehicle; context=BrakeAuthorityContext(20.,0.,(.19,)*4,(2500.,)*4)
        for target in (3.,6.):
            fast=state_aware_force(self.predictor,context,target)
            exact=self.predictor.request(context,target).pedal_force_n
            self.assertAlmostEqual(fast,exact,delta=.02)
        self.assertEqual(state_aware_force(self.predictor,context,0.),0.)

    def test_unit_and_error_gate_at_six(self):
        t=np.array([0.,.35,.45,.55,.65])
        result=settled_metrics(t,np.full(5,50.),np.full(5,50.),np.full(5,6.29),6.)
        self.assertTrue(result['tracking_pass'])
        self.assertAlmostEqual(result['error_limit_mps2'],.3)
        self.assertAlmostEqual(result['requested_force_mean_N'],50.)


if __name__=='__main__':
    unittest.main()
