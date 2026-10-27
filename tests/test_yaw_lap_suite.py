import unittest
from pathlib import Path
from tools.run.run_yaw_lap_suite import parse_args, _failure_summary
from simplified_vehicle_models.solvers.forward_driver import ForwardIntegrationError


class SuiteParserTests(unittest.TestCase):
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
