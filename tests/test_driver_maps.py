import json
import math
import tempfile
import unittest
from pathlib import Path

from simplified_vehicle_models.solvers.driver_maps import (
    LongitudinalDemandMap,
    MonotoneTable1D,
    PedalActuatorCalibration,
    PedalMapConfiguration,
    load_pedal_map_config,
    parse_pedal_map_config,
)


class MonotoneTableTests(unittest.TestCase):
    def table(self, axis=(0, .2, .4, 1), value=(0, .5, .5, 1)):
        return MonotoneTable1D(axis, value, '1', '1')

    def test_interpolation_clamp_and_plateau_inverse(self):
        t = self.table()
        self.assertEqual(t.evaluate(.2), .5)
        self.assertEqual(t.evaluate(.3), .5)
        self.assertEqual(t.evaluate(-4), 0)
        self.assertEqual(t.evaluate(8), 1)
        self.assertEqual(t.inverse(.5), .2)
        self.assertAlmostEqual(t.inverse(.75), .7)
        self.assertEqual(t.inverse(0), 0)
        self.assertEqual(t.inverse(2), 1)

    def test_inverse_plateaus_at_both_ends_and_constant_output(self):
        terminal = MonotoneTable1D([0, .5, .75, 1], [0, 1, 1, 1], '1', '1')
        self.assertEqual(terminal.inverse(1), .5)
        self.assertEqual(terminal.inverse(2), 1)  # strictly out of range clamps
        initial = MonotoneTable1D([0, .25, .5, 1], [0, 0, 0, 1], '1', '1')
        self.assertEqual(initial.inverse(0), 0)
        constant = MonotoneTable1D([0, .5, 1], [2, 2, 2], '1', '1')
        self.assertEqual(constant.inverse(2), 0)
        self.assertEqual(constant.inverse(-1), 0)
        self.assertEqual(constant.inverse(3), 1)

    def test_copies_input_and_rejects_bad_inputs_and_queries(self):
        x, y = [0, 1], [0, 1]
        t = MonotoneTable1D(x, y, '1', '1')
        x[1] = 3
        self.assertEqual(t.axis, (0., 1.))
        for args in [([0], [0]), ([0, 1], [0]), ([0, 0], [0, 1]),
                     ([0, 1], [1, 0]), ([0, True], [0, 1]),
                     ([0, float('inf')], [0, 1]), ([0, 1], [0, float('nan')])]:
            with self.subTest(args=args), self.assertRaises((TypeError, ValueError)):
                MonotoneTable1D(*args, '1', '1')
        for q in (True, float('nan'), float('inf')):
            with self.subTest(q=q), self.assertRaises((TypeError, ValueError)):
                t.evaluate(q)
            with self.subTest(q=q), self.assertRaises((TypeError, ValueError)):
                t.inverse(q)


class LayerTests(unittest.TestCase):
    def test_demand_sign_zero_and_exclusive_outputs(self):
        drive = MonotoneTable1D([0, 2, 6], [0, .35, 1], 'm/s^2', '1')
        brake = MonotoneTable1D([0, 2, 8], [0, .25, 1], 'm/s^2', '1')
        m = LongitudinalDemandMap(drive, brake)
        self.assertEqual(m.command(0), (0, 0))
        self.assertEqual(m.command(1), (.175, 0))
        self.assertEqual(m.command(-4), (0, .5))
        self.assertEqual(m.command(100), (1, 0))
        self.assertEqual(m.command(-100), (0, 1))
        with self.assertRaises((TypeError, ValueError)):
            m.command(True)

    def test_demand_ownership_units_and_endpoints(self):
        good = MonotoneTable1D([0, 1], [0, 1], 'm/s^2', '1')
        bad_unit = MonotoneTable1D([0, 1], [0, 1], 'km/h', '1')
        for d, b in ((good, bad_unit), (bad_unit, good)):
            with self.assertRaises(ValueError):
                LongitudinalDemandMap(d, b)
        with self.assertRaises(ValueError):
            LongitudinalDemandMap(MonotoneTable1D([0, 1], [.1, 1], 'm/s^2', '1'), good)

    def calibration(self):
        return PedalActuatorCalibration(
            MonotoneTable1D([0, .2, 1], [0, .08, 1], '1', '1'),
            MonotoneTable1D([0, .3, 1], [0, .1, 1], '1', '1'))

    def test_physical_shapes_force_normalization_bounds_and_units(self):
        c = self.calibration()
        self.assertEqual(c.shape_throttle(.2), .08)
        self.assertEqual(c.shape_throttle(1), 1)
        self.assertAlmostEqual(c.shape_brake_force(30, 100), 10.0) # normalized input .3 maps to .1
        for x in (-.01, 1.01, float('nan'), True):
            with self.assertRaises((TypeError, ValueError)):
                c.shape_throttle(x)
        for maximum in (0, -1, float('inf'), True):
            with self.assertRaises((TypeError, ValueError)):
                c.shape_brake_force(0, maximum)
        with self.assertRaises(ValueError):
            PedalActuatorCalibration(MonotoneTable1D([0, 1], [0, 1], 'm/s^2', '1'), self.calibration().brake_force_fraction)


class ConfigurationTests(unittest.TestCase):
    def payload(self):
        return {'schema_version': 1, 'demand_map': {
            'units': {'acceleration_axis': 'm/s^2', 'deceleration_axis': 'm/s^2', 'pedal_output': '1'},
            'acceleration_to_throttle': {'axis': [0, 2, 6], 'value': [0, .35, 1]},
            'deceleration_to_brake': {'axis': [0, 2, 8], 'value': [0, .25, 1]}},
            'physical_calibration': {
            'units': {'throttle_input': '1', 'throttle_effective_request': '1', 'brake_input': '1', 'brake_effective_request': '1'},
            'throttle_fraction': {'axis': [0, .2, 1], 'value': [0, .08, 1]},
            'brake_force_fraction': {'axis': [0, .3, 1], 'value': [0, .1, 1]}}}

    def test_both_each_or_neither_round_trip_and_digest(self):
        full = self.payload()
        for payload in (full, {**full, 'demand_map': None}, {**full, 'physical_calibration': None},
                        {'schema_version': 1}):
            c = parse_pedal_map_config(payload)
            self.assertIsInstance(c, PedalMapConfiguration)
            restored = parse_pedal_map_config(c.to_dict())
            self.assertEqual(c.to_dict(), restored.to_dict())
            self.assertEqual(c.sha256, restored.sha256)
        reversed_keys = json.loads(json.dumps(full, sort_keys=True))
        self.assertEqual(parse_pedal_map_config(full).sha256, parse_pedal_map_config(reversed_keys).sha256)
        self.assertEqual(parse_pedal_map_config(full).to_dict()['schema_version'], 1)

    def test_strict_schema_units_and_json_nonfinite(self):
        base = self.payload()
        cases = []
        for version in (True, 1.0, 2, 0): cases.append({**base, 'schema_version': version})
        cases.append({**base, 'extra': 1})
        cases.append({k: v for k, v in base.items() if k != 'schema_version'})
        cases.append({**base, 'demand_map': {**base['demand_map'], 'mystery': 1}})
        cases.append({**base, 'demand_map': {k: v for k, v in base['demand_map'].items() if k != 'acceleration_to_throttle'}})
        cases.append({**base, 'demand_map': {**base['demand_map'], 'units': {**base['demand_map']['units'], 'pedal_output': 'percent'}}})
        for p in cases:
            with self.subTest(payload=p), self.assertRaises(ValueError): parse_pedal_map_config(p)
        with self.assertRaises(ValueError): parse_pedal_map_config(json.loads('{"schema_version":1,"x":NaN}'))
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'map.json'
            path.write_text(json.dumps(base))
            self.assertEqual(load_pedal_map_config(path).sha256, parse_pedal_map_config(base).sha256)

    def test_integer_float_normalization(self):
        p = self.payload()
        q = json.loads(json.dumps(p))
        self.assertEqual(parse_pedal_map_config(p).sha256, parse_pedal_map_config(q).sha256)


if __name__ == '__main__':
    unittest.main()
