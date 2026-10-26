import unittest
from pathlib import Path
from tools.run.run_yaw_lap_suite import parse_args


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
