"""Portable contract tests for the stop validation runner."""
import json
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.run import validate_brake_stop as runner


class StopValidationContractTests(unittest.TestCase):
    def test_metrics_parser_accepts_rows(self):
        self.assertEqual(runner.parse_metrics('{"rows": []}')['rows'], [])

    def test_metrics_parser_rejects_missing_rows(self):
        with self.assertRaises(ValueError):
            runner.parse_metrics('{"result": []}')

    def test_event_is_positive_operational_speed_not_rest(self):
        self.assertEqual(runner.EVENT_SPEED_MPS, .01)
        self.assertNotEqual(runner.EVENT_SPEED_MPS, 0.)

    def test_target_and_plan_domains_are_separate(self):
        self.assertEqual(runner.TARGETS, (.7, 1.))
        self.assertEqual(runner.MAX_TIME_S, 10.)

    def test_distance_metrics_compare_to_actual_denominator(self):
        metrics = runner.stop_distance_metrics(10.4, 10.)
        self.assertAlmostEqual(metrics['relative_error_fraction_actual_denominator'], .04)
        self.assertTrue(metrics['pass'])

    def test_distance_metrics_reject_zero_negative_or_nonfinite(self):
        for predicted, actual in ((0., 1.), (-1., 1.), (1., 0.), (1., -1.), (float('nan'), 1.)):
            with self.subTest(predicted=predicted, actual=actual):
                with self.assertRaises(ValueError):
                    runner.stop_distance_metrics(predicted, actual)


if __name__ == '__main__':
    unittest.main()
