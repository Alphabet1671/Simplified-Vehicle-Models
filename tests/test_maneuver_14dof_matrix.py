import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.maneuver_gui.runner import run_definition
from tools.maneuver_gui.artifacts import load_manifest
from simplified_vehicle_models.controls.maneuvers.schema import load_definition


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
            channels = json.loads((result / 'channels.json').read_text())
            if inspect_result is not None:
                inspect_result(result)
            return manifest, arrays, commands, channels

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

    def test_vertical_csv_playback_preserves_source_digest(self):
        vertical = 'models/formulations/iem24_murphy_real_14dof_simple.xml'
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            csv_path = root / 'commands.csv'
            csv_path.write_text('time_s,swa,accelerator,brake\n0,0,0.1,0\n0.1,0.01,0.1,0\n')
            definition = self._definition(vertical, self._calibration(vertical), duration=.1)
            definition.update({'preset': 'csv_playback', 'steering': {'mode': 'csv'},
                               'longitudinal': {'mode': 'csv'},
                               'csv': {'source': 'commands.csv',
                                       'mapping': {'time': 'time_s', 'swa': 'swa',
                                                   'accelerator': 'accelerator', 'brake': 'brake'},
                                       'interpolation': 'hold', 'time_unit': 's', 'swa_unit': 'rad',
                                       'accelerator_unit': 'fraction', 'brake_unit': 'fraction'}})
            source = root / 'definition.json'; source.write_text(json.dumps(definition))
            attempt = run_definition(source, root / 'out', timeout_s=180)
            summary = json.loads((attempt / 'summary.json').read_text())
            self.assertEqual(summary['execution_outcome'], 'completed', summary)
            self.assertIn('csv_sha256', summary)
            channels = json.loads((attempt / 'result' / 'channels.json').read_text())
            self.assertTrue({'position-x', 'position-y', 'heading-world'}.issubset(channels[0]))

    def test_iem24_vertical_and_legacy_planar_runs(self):
        vertical = 'models/formulations/iem24_murphy_real_14dof_simple.xml'
        manifest, evidence, commands, channels = self._run(self._definition(vertical, self._calibration(vertical)))
        self.assertEqual(manifest['model_kind'], '14dof-simple')
        self.assertGreaterEqual(evidence['state'].shape[1], 38)
        self.assertEqual(evidence['state'].shape, evidence['state_rate'].shape)
        self.assertGreater(len(commands), 0)
        self.assertTrue(any(float(row['swa_rad']) != 0. for row in commands))
        self.assertTrue({'position-x', 'position-y', 'heading-world'}.issubset(channels[0]))

        planar = 'models/formulations/iem24_murphy_real_planar_7dof.xml'
        manifest, evidence, commands, _channels = self._run(self._definition(planar, self._calibration(planar), duration=.1))
        self.assertEqual(manifest['model_kind'], 'planar-7dof')
        self.assertEqual(evidence['state'].shape[1], 12)
        self.assertEqual(evidence['state'].shape, evidence['state_rate'].shape)
        self.assertGreater(len(commands), 0)


class RequestedMatrixInputTests(unittest.TestCase):
    root = Path(__file__).resolve().parents[1] / 'tools/maneuver_gui/examples/iem24_14dof'
    formulation = 'models/formulations/iem24_murphy_real_14dof_simple.xml'

    def test_requested_inputs_are_strict_simulation_definitions(self):
        names = ('step', 'ramp', 'sine', 'acceleration', 'braking', 'frequency-sweep')
        for name in names:
            with self.subTest(name=name):
                data = load_definition(self.root / f'{name}.maneuver.json').to_dict()
                self.assertEqual(data['backend'], 'simulation')
                self.assertEqual(data['vehicle']['model'], self.formulation)
                self.assertEqual(data['calibration']['accelerator_travel'], [0., 1.])
                self.assertEqual(data['calibration']['accelerator_force_n'], [0., 10.5])
                self.assertEqual(data['calibration']['brake_travel'], [0., .5, 1.])
                self.assertEqual(data['calibration']['maximum_road_wheel_angle_rad'], 0.5235987755982988)
        for name in ('step', 'ramp', 'sine'):
            data = load_definition(self.root / f'{name}.maneuver.json').to_dict()
            self.assertEqual(data['initial']['speed_mps'], 16.6666666667)
            self.assertEqual(data['steering']['mode'], 'peak_ay')
            self.assertEqual(data['steering']['amplitude_mps2'], 9.80665)
            self.assertEqual(data['longitudinal']['mode'], 'cruise_pid')
            self.assertEqual(data['longitudinal']['target_speed_mps'], 16.6666666667)
            self.assertIn('FROZEN PEAK-AY CALIBRATION REQUIRED', data['name'])
            self.assertNotIn('pending', json.dumps(data['calibration']).lower())
        acceleration = load_definition(self.root / 'acceleration.maneuver.json').to_dict()
        self.assertEqual(acceleration['steering']['mode'], 'locked_swa')
        self.assertEqual(acceleration['longitudinal']['accelerator'], .5)
        braking = load_definition(self.root / 'braking.maneuver.json').to_dict()
        self.assertEqual(braking['steering']['mode'], 'locked_swa')
        self.assertEqual(braking['longitudinal']['start_brake'], .5)
        self.assertEqual(braking['longitudinal']['start_accelerator'], 0.)

    def test_frequency_sweep_csv_matches_requested_law(self):
        with (self.root / 'frequency-sweep.csv').open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1001)
        time = np.array([float(row['time_s']) for row in rows])
        swa = np.array([float(row['swa']) for row in rows])
        self.assertTrue(np.allclose(np.diff(time), .01, atol=1e-12))
        self.assertEqual((time[0], time[-1]), (0., 10.))
        phase = 2*np.pi*.1*(10**(time/5)-1)/(np.log(10)/5)
        self.assertTrue(np.allclose(swa, .025*np.sin(phase), atol=1e-12))
        self.assertTrue(all(float(row['accelerator']) == .1 for row in rows))
        self.assertTrue(all(float(row['brake']) == 0. for row in rows))
        sweep = load_definition(self.root / 'frequency-sweep.maneuver.json').to_dict()
        self.assertEqual(sweep['steering']['mode'], 'csv')
        self.assertEqual(sweep['longitudinal']['mode'], 'csv')
        self.assertEqual(sweep['csv']['mapping'], {'time': 'time_s', 'swa': 'swa',
                                                     'accelerator': 'accelerator', 'brake': 'brake'})


if __name__ == '__main__':
    unittest.main()
