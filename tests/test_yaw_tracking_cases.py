import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from tools.run.run_yaw_tracking_case import (apply_state_overrides, synthetic_path_fixtures,
    validate_case, CASE_FIELDS, CONTROLLER_FIELDS)
from tools.run.run_yaw_lap_suite import apply_initial_state_overrides

ROOT = Path(__file__).resolve().parents[1]


class YawTrackingCaseTests(unittest.TestCase):
    def test_versioned_matrix_contains_required_factors_and_physical_limits(self):
        matrix = json.loads((ROOT / 'docs/dev/yaw-lap-suite-closure/yaw-tracking-cases-v1.json').read_text())
        self.assertEqual(matrix['schema_version'], 1)
        self.assertEqual(len(matrix['state_perturbations']), 10)
        self.assertEqual(matrix['controller_sweep']['minimum_preview_m'], 1.0)
        self.assertEqual(matrix['actuator_sensitivity']['physical_limits'], 'unchanged')
        self.assertEqual(matrix['physical_validation']['status'], 'inconclusive_not_run')

    def valid_case(self):
        return {'plan': 'saved-plan.npz', 'plan_sha256': 'a' * 64,
                'initial_state_override': {},
                'controller': {'preview_seconds': 0.10, 'yaw_damping': 0.12,
                    'heading_gain': 1.8, 'lateral_gain': 2.0, 'minimum_preview_m': 1.0},
                'plant_variation': {}}

    def test_case_requires_every_tuple_field_and_rejects_unknown_top_level(self):
        complete = self.valid_case()
        validate_case(complete)
        for field in CASE_FIELDS:
            bad = dict(complete)
            del bad[field]
            with self.subTest(missing=field), self.assertRaisesRegex(ValueError, 'missing|required'):
                validate_case(bad)
        bad = dict(complete, extra='ignored?')
        with self.assertRaisesRegex(ValueError, 'unknown top-level'):
            validate_case(bad)

    def test_case_rejects_nonfinite_and_unknown_state_fields(self):
        for override in ({'speed': float('nan')}, {'steering': 0.2}):
            bad = self.valid_case()
            bad['initial_state_override'] = override
            with self.assertRaises(ValueError):
                validate_case(bad)

    def test_controller_requires_complete_resolved_fields_and_sane_domain(self):
        for field in CONTROLLER_FIELDS:
            bad = self.valid_case()
            del bad['controller'][field]
            with self.subTest(missing=field), self.assertRaises(ValueError):
                validate_case(bad)
        for field, value in (('preview_seconds', -0.01), ('yaw_damping', -0.01),
                             ('heading_gain', float('nan')), ('lateral_gain', 0.0),
                             ('minimum_preview_m', -1.0), ('heading_gain', 2.0),
                             ('lateral_gain', 1.5), ('minimum_preview_m', 0.0)):
            bad = self.valid_case()
            bad['controller'][field] = value
            with self.subTest(controller={field: value}), self.assertRaises(ValueError):
                validate_case(bad)

    def test_analytic_s_bend_seam_and_pinch_fixture(self):
        fixture = synthetic_path_fixtures()
        self.assertEqual(fixture['seam_index'], abs(fixture['curvature_rate']).argmax())
        self.assertGreater(fixture['curvature'].max(), 0)
        self.assertLess(fixture['curvature'].min(), 0)
        self.assertLess(fixture['half_width'].min(), fixture['half_width'].max())

    def test_suite_override_helper_rejects_inadmissible_state(self):
        import numpy as np
        state = np.zeros(12)
        state[3] = 0.3
        with self.assertRaisesRegex(ValueError, 'inadmissible'):
            apply_initial_state_overrides(state, {'speed': -0.5}, closed=True)

    def test_inadmissible_perturbed_state_rejected_without_clipping(self):
        import numpy as np
        state = np.zeros(12)
        state[3] = 0.3
        with self.assertRaisesRegex(ValueError, 'inadmissible'):
            apply_state_overrides(state, {'speed': -0.5}, closed=True)
        self.assertEqual(state[3], 0.3)

    def test_case_rejects_invalid_actuator_multipliers(self):
        with self.assertRaises(ValueError):
            validate_case({'plant_variation': {'steering': 0}})

    def test_single_physical_case_fails_closed_without_identity_binding_under_watchdog(self):
        with tempfile.TemporaryDirectory() as directory:
            case = Path(directory) / 'case.json'
            case.write_text(json.dumps({**self.valid_case(), 'plan': '', 'plan_sha256': ''}))
            command = [sys.executable, 'tools/run/run_with_deadline.py', '--seconds', '20', '--',
                       sys.executable, 'tools/run/run_yaw_tracking_case.py', '--case', str(case)]
            env = dict(os.environ, PYTHONPATH='src:.')
            result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, timeout=25)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('nonempty immutable saved-plan path', result.stderr)


if __name__ == '__main__':
    unittest.main()
