import copy
import math
import tempfile
import unittest
from pathlib import Path

from simplified_vehicle_models.controls.maneuvers.types import RobotCommand
from simplified_vehicle_models.controls.maneuvers.schema import (
    CapabilityError, convert_unit, load_definition, save_definition,
)
from simplified_vehicle_models.controls.maneuvers.types import ManeuverDefinition


def fixture(preset="step_steer"):
    return {
        "schema": "maneuver-v1", "name": "contract", "preset": preset,
        "backend": "simulation", "vehicle": {"model": "planar-7dof"},
        "initial": {"speed_mps": 10.0, "x_m": 0.0, "y_m": 0.0, "heading_rad": 0.0},
        "clock": {"controller_tick_s": 0.01},
        "steering": {"mode": "peak_swa", "amplitude_rad": 0.2, "direction": 1},
        "longitudinal": {"mode": "constant_accelerator", "accelerator": 0.1},
        "profile": {"lead_in_s": 1.0, "rise_s": 0.2, "hold_s": 1.0, "release_s": 0.2, "recovery_s": 1.0, "duration_s": 3.4},
        "limits": {"max_swa_rad": 1.0, "max_swa_rate_rad_s": 2.0},
        "termination": {"duration_s": 3.4}, "calibration": None, "csv": None,
    }


class CommandTests(unittest.TestCase):
    def test_command_validates_physical_units(self):
        self.assertAlmostEqual(convert_unit(180., 'deg', 'rad'), math.pi)
        self.assertEqual(RobotCommand(0.2, 0.1, 0.).accelerator, 0.1)
        for values in ((float('nan'), 0., 0.), (0., 1.1, 0.), (0., .1, .1), (True, 0., 0.)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                RobotCommand(*values)

    def test_unit_conversions_and_bad_inputs(self):
        self.assertAlmostEqual(convert_unit(1., 'g', 'm/s^2'), 9.80665)
        self.assertEqual(convert_unit(50., 'percent', 'fraction'), .5)
        self.assertEqual(convert_unit(500., 'ms', 's'), .5)
        for pair in [('kg', 'm'), ('deg', 'km/h')]:
            with self.assertRaises(ValueError): convert_unit(1., *pair)
        with self.assertRaises(ValueError): convert_unit(True, 'deg', 'rad')


class DefinitionTests(unittest.TestCase):
    def test_all_presets_round_trip_and_defensive_copy(self):
        for preset in ('step_steer','ramp_steer','sine_steer','straight_maintain','straight_accel_brake','circular_skidpad','csv_playback'):
            payload = fixture(preset)
            if preset == 'csv_playback':
                payload['csv'] = {'mapping': {'swa': 's', 'accelerator': 'a', 'brake': 'b'}, 'interpolation': 'hold'}
                payload['steering']['mode'] = 'csv'
                payload['longitudinal']['mode'] = 'csv'
            definition = ManeuverDefinition.from_dict(payload)
            self.assertEqual(ManeuverDefinition.from_dict(definition.to_dict()), definition)
            payload['vehicle']['model'] = 'changed'
            self.assertEqual(definition.to_dict()['vehicle']['model'], 'planar-7dof')

    def test_strict_keys_types_and_modes(self):
        bad = fixture(); bad['surprise'] = 1
        with self.assertRaises(ValueError): ManeuverDefinition.from_dict(bad)
        bad = fixture(); bad['vehicle']['surprise'] = 1
        with self.assertRaises(ValueError): ManeuverDefinition.from_dict(bad)
        bad = fixture(); bad['csv'] = {'mapping': {'swa': 's', 'accelerator': 'a', 'brake': 'b', 'surprise': 'x'}, 'interpolation': 'hold'}
        with self.assertRaises(ValueError): ManeuverDefinition.from_dict(bad)
        bad = fixture(); bad['calibration'] = ['unexpected']
        with self.assertRaises(ValueError): ManeuverDefinition.from_dict(bad)
        bad = fixture(); bad['steering']['mode'] = 'peak_ay'; bad['steering']['amplitude_mps2'] = 4.; bad['steering']['amplitude_rad'] = None
        with self.assertRaises(CapabilityError): ManeuverDefinition.from_dict(bad)
        bad = fixture(); bad['profile']['hold_s'] = True
        with self.assertRaises(ValueError): ManeuverDefinition.from_dict(bad)

    def test_timing_cycles_direction_and_exclusive_modes(self):
        for key, value in [('lead_in_s', -1), ('rise_s', 0), ('duration_s', float('inf'))]:
            bad = fixture(); bad['profile'][key] = value
            with self.assertRaises(ValueError): ManeuverDefinition.from_dict(bad)
        bad = fixture('sine_steer'); bad['profile']['cycles'] = 1.5
        with self.assertRaises(ValueError): ManeuverDefinition.from_dict(bad)
        bad = fixture(); bad['steering']['direction'] = 0
        with self.assertRaises(ValueError): ManeuverDefinition.from_dict(bad)
        bad = fixture(); bad['steering']['mode'] = 'locked_swa'; bad['steering']['amplitude_rad'] = 0.2
        with self.assertRaisesRegex(ValueError, 'mode|exclusive'):
            ManeuverDefinition.from_dict(bad)

    def test_csv_owns_every_channel(self):
        bad = fixture('csv_playback'); bad['csv'] = {'mapping': {'swa': 's', 'accelerator': 'a', 'brake': 'b'}, 'interpolation': 'hold'}
        bad['steering']['mode'] = 'peak_swa'
        with self.assertRaisesRegex(ValueError, 'CSV|csv|overlay|owner'):
            ManeuverDefinition.from_dict(bad)

    def test_calibration_backend_gate_and_json_io(self):
        payload = fixture()
        self.assertIsNotNone(ManeuverDefinition.from_dict(payload))
        payload['backend'] = 'physical'
        with self.assertRaises(CapabilityError): ManeuverDefinition.from_dict(payload)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'maneuver.json'
            d = ManeuverDefinition.from_dict(fixture())
            save_definition(path, d)
            self.assertEqual(load_definition(path), d)
            path.write_text('{', encoding='utf-8')
            with self.assertRaises(ValueError): load_definition(path)


if __name__ == '__main__': unittest.main()
