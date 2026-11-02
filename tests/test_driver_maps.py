import json
import math
import tempfile
import unittest
from pathlib import Path

from simplified_vehicle_models.solvers.driver_maps import (
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


class CalibrationTests(unittest.TestCase):
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
        return {"schema_version": 1, "physical_calibration": {
            "units": {"throttle_input": "1", "throttle_effective_request": "1",
                      "brake_input": "1", "brake_effective_request": "1"},
            "throttle_fraction": {"axis": [0, .2, 1], "value": [0, .08, 1]},
            "brake_force_fraction": {"axis": [0, .3, 1], "value": [0, .1, 1]}}}

    def test_physical_calibration_round_trip_digest_and_empty(self):
        for payload in (self.payload(), {"schema_version": 1}):
            config = parse_pedal_map_config(payload)
            restored = parse_pedal_map_config(config.to_dict())
            self.assertEqual(config.to_dict(), restored.to_dict())
            self.assertEqual(config.sha256, restored.sha256)
        self.assertEqual(parse_pedal_map_config({"schema_version": 1}).physical_calibration, None)

    def test_demand_policy_schema_is_actionably_retired(self):
        with self.assertRaisesRegex(ValueError, "demand_map is retired"):
            parse_pedal_map_config({"schema_version": 1, "demand_map": None})

    def test_strict_physical_schema_and_json_nonfinite(self):
        base = self.payload()
        cases = [{**base, "schema_version": version} for version in (True, 1.0, 2, 0)]
        cases.extend(({**base, "extra": 1}, {"physical_calibration": base["physical_calibration"]},
                      {**base, "physical_calibration": {**base["physical_calibration"], "extra": 1}}))
        for payload in cases:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                parse_pedal_map_config(payload)
        with self.assertRaises(ValueError):
            parse_pedal_map_config(json.loads('{"schema_version":1,"x":NaN}'))
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "map.json"
            path.write_text(json.dumps(base))
            self.assertEqual(load_pedal_map_config(path).sha256, parse_pedal_map_config(base).sha256)


if __name__ == '__main__':
    unittest.main()
