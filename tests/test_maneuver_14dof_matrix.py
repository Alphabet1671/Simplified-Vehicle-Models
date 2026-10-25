import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.maneuver_gui.runner import run_definition
from tools.maneuver_gui.artifacts import load_manifest


class Maneuver14DofMatrixTests(unittest.TestCase):
    def _definition(self, model, calibration, *, duration=2.0):
        return {
            'schema': 'maneuver-v1', 'name': 'bounded 14dof step steer',
            'preset': 'step_steer', 'backend': 'simulation',
            'vehicle': {'model': model},
            'initial': {'speed_mps': 16.6666666667, 'x_m': 0., 'y_m': 0., 'heading_rad': 0.},
            'clock': {'controller_tick_s': .02},
            'steering': {'mode': 'peak_swa', 'amplitude_rad': .025},
            'longitudinal': {'mode': 'constant_accelerator', 'accelerator': .12},
            'profile': {'duration_s': duration}, 'limits': {},
            'termination': {'duration_s': duration}, 'calibration': calibration, 'csv': None,
        }

    def _calibration(self, model_path):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        vehicle = compile_formulation(model_path).model
        p = vehicle.planar if hasattr(vehicle, 'planar') else vehicle
        return {
            'maximum_swa_rad': 8., 'maximum_swa_rate_rad_s': 2.,
            'rack_travel_per_revolution_m': .04, 'steer_arm_length_m': .10,
            'maximum_road_wheel_angle_rad': p.steering_maximum_angle,
            'ackermann_inner_angle_rad': [.0, .6], 'ackermann_outer_offset_rad': [.0, .1],
            'accelerator_travel': [0., 1.],
            'accelerator_force_n': [0., p.controller.maximum_drive_motor_torque_nm],
            'brake_travel': [0., .5, 1.], 'brake_force_n': [0., 137., 420.],
        }

    def _run(self, definition, inspect_result=None):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            source = root / 'definition.json'
            source.write_text(json.dumps(definition))
            attempt = run_definition(source, root / 'out', timeout_s=180)
            summary = json.loads((attempt / 'summary.json').read_text())
            self.assertEqual(summary['execution_outcome'], 'completed', summary)
            result = attempt / 'result'
            manifest = json.loads((result / 'manifest.json').read_text())
            self.assertEqual(load_manifest(result), manifest)
            with np.load(result / 'run_record.npz', allow_pickle=False) as evidence:
                arrays = {key: evidence[key] for key in evidence.files}
            with (result / 'commands.csv').open(newline='') as stream:
                commands = list(csv.DictReader(stream))
            if inspect_result is not None:
                inspect_result(result)
            return manifest, arrays, commands

    def test_model_kind_manifest_validation(self):
        vertical = 'models/formulations/iem24_murphy_real_14dof_simple.xml'
        vertical_definition = self._definition(vertical, self._calibration(vertical), duration=.1)
        def verify_rejections(result):
            manifest_path = result / 'manifest.json'
            original = json.loads(manifest_path.read_text())
            tampered = dict(original)
            tampered['model_kind'] = 'planar-7dof'
            for changed in (
                tampered,
                {**original, 'model_kind': 'unsupported-model'},
            ):
                manifest_path.write_text(json.dumps(changed))
                with self.assertRaises(ValueError):
                    load_manifest(result)
        self._run(vertical_definition, inspect_result=verify_rejections)

    def test_iem24_vertical_and_legacy_planar_runs(self):
        vertical = 'models/formulations/iem24_murphy_real_14dof_simple.xml'
        manifest, evidence, commands = self._run(self._definition(vertical, self._calibration(vertical)))
        self.assertEqual(manifest['model_kind'], '14dof-simple')
        self.assertGreaterEqual(evidence['state'].shape[1], 38)
        self.assertEqual(evidence['state'].shape, evidence['state_rate'].shape)
        self.assertGreater(len(commands), 0)
        self.assertTrue(any(float(row['swa_rad']) != 0. for row in commands))

        planar = 'models/formulations/iem24_murphy_real_planar_7dof.xml'
        manifest, evidence, commands = self._run(self._definition(planar, self._calibration(planar), duration=.1))
        self.assertEqual(manifest['model_kind'], 'planar-7dof')
        self.assertEqual(evidence['state'].shape[1], 12)
        self.assertEqual(evidence['state'].shape, evidence['state_rate'].shape)
        self.assertGreater(len(commands), 0)


if __name__ == '__main__':
    unittest.main()
