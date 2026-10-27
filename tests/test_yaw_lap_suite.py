import unittest
from pathlib import Path
from tools.run.run_yaw_lap_suite import parse_args, _failure_summary
from simplified_vehicle_models.solvers.forward_driver import ForwardIntegrationError
from simplified_vehicle_models.planning.yaw_replay_metrics import time_weighted_rms, time_weighted_fraction


class SuiteParserTests(unittest.TestCase):
    def test_irregular_time_weighted_metrics_use_trapezoidal_integrals(self):
        import numpy as np
        time=np.array([0.,1.,3.]);error=np.array([0.,2.,0.])
        expected=np.sqrt((1.*(0.+4.)/2.+2.*(4.+0.)/2.)/3.)
        self.assertAlmostEqual(time_weighted_rms(time,error),expected)
        self.assertAlmostEqual(time_weighted_fraction(time,np.array([False,True,False])),.5)
    def test_saved_replay_requires_plan_and_map(self):
        args=parse_args(['--track','track.xml','--output-dir','run','--yaw-map','map.npz',
                         '--mode','replay','--plan','plan.npz'])
        self.assertEqual(args.plan,Path('plan.npz'))
        self.assertEqual(args.controller,'preview')

    def test_fixed_plan_and_replay_are_distinct_modes(self):
        args=parse_args(['--track','track.xml','--output-dir','run','--yaw-map','map.npz'])
        self.assertEqual(args.mode,'fixed')
        self.assertIsNone(args.plan)

    def test_validation_failure_summary_exposes_diagnostics_and_unvalidated_snapshot(self):
        error=ForwardIntegrationError('accepted-state validation exceeded budget',{
            'failure_kind':'validation_timeout','last_state_kind':'solver_accepted_unvalidated',
            'last_accepted_time':12.5,'accepted_step_count':4,'rhs_evaluations':87,
            'rejected_domain_trials':2,'elapsed_wall_seconds':3.1,'wall_seconds':3.,
            'validation_timings':{'accepted_state_validation_seconds':1.2},
            'validation_counts':{'accepted_states_validated':3}},[1.,2.],12.5)
        summary=_failure_summary(error)
        self.assertEqual(summary['status'],'failed')
        self.assertEqual(summary['last_state_snapshot'],'integration_failure.json')
        self.assertFalse(summary['snapshot_validated'])
        diag=summary['replay_validation_diagnostics']
        self.assertEqual(diag['failure_kind'],'validation_timeout')
        self.assertEqual(diag['validation_counts']['accepted_states_validated'],3)
        self.assertEqual(diag['validation_timings']['accepted_state_validation_seconds'],1.2)
