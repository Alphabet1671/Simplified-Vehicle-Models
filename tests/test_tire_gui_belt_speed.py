import unittest

import numpy as np

from tools.tire_gui.belt_speed import resolve_belt_speed
from tools.tire_gui.contracts import RunData


class BeltSpeedTests(unittest.TestCase):
    def run_data(self, **channels):
        n = len(next(iter(channels.values())))
        return RunData(7, 'synthetic tire', channels, {}, np.arange(n))

    def test_auto_retains_measured_speed_and_real_stops_with_provenance(self):
        measured = np.array([0.0, 0.01, 2.0])
        original = self.run_data(speed=measured, omega=np.zeros(3), effective_radius=np.full(3, .2), kappa=np.zeros(3))
        resolved = resolve_belt_speed(original)
        np.testing.assert_array_equal(resolved.channels['speed'], measured)
        np.testing.assert_array_equal(resolved.channels['speed_measured'], measured)
        self.assertEqual(resolved.metadata['belt_speed_source'], 'measured')
        self.assertIn('near zero', resolved.metadata['belt_speed_warning'])
        np.testing.assert_array_equal(original.channels['speed'], measured)
        self.assertNotIn('speed_measured', original.channels)

    def test_placeholder_auto_recovers_using_effective_radius_and_slip(self):
        original = self.run_data(speed=np.full(3, .01), omega=np.array([10., 20., 30.]),
                                 effective_radius=np.full(3, .2), kappa=np.full(3, .1))
        resolved = resolve_belt_speed(original)
        np.testing.assert_allclose(resolved.channels['speed'], np.array([10., 20., 30.])*.2/1.1)
        np.testing.assert_array_equal(resolved.channels['speed_measured'], np.full(3, .01))
        self.assertEqual(resolved.metadata['belt_speed_source'], 'wheel')
        self.assertIn('placeholder', resolved.metadata['belt_speed_warning'])
        self.assertFalse(original.channels['speed'].flags.writeable)

    def test_missing_or_invalid_wheel_inputs_do_not_silently_fallback_in_wheel_mode(self):
        cases = [
            self.run_data(speed=np.full(2, .01), omega=np.zeros(2), effective_radius=np.full(2, .2), kappa=np.zeros(2)),
            self.run_data(speed=np.full(2, .01), omega=np.full(2, 20.), effective_radius=np.full(2, 2.), kappa=np.zeros(2)),
            self.run_data(speed=np.full(2, .01), omega=np.full(2, 20.), effective_radius=np.full(2, .2), kappa=np.full(2, -1.)),
            self.run_data(speed=np.full(2, .01), omega=np.full(2, 20.), effective_radius=np.full(2, .2)),
        ]
        for run in cases:
            with self.subTest(channels=tuple(run.channels)), self.assertRaisesRegex(ValueError, 'Wheel-speed recovery'):
                resolve_belt_speed(run, source='wheel')

    def test_auto_invalid_fallback_retains_placeholder_and_records_warning(self):
        original = self.run_data(speed=np.full(2, .01), omega=np.zeros(2), effective_radius=np.full(2, .2), kappa=np.zeros(2))
        resolved = resolve_belt_speed(original)
        np.testing.assert_array_equal(resolved.channels['speed'], [.01, .01])
        self.assertEqual(resolved.metadata['belt_speed_source'], 'measured')
        self.assertIn('Placeholder speed retained', resolved.metadata['belt_speed_warning'])

    def test_override_and_measured_restoration(self):
        original = self.run_data(speed=np.array([.01, .01]), omega=np.array([10., 20.]),
                                 effective_radius=np.full(2, .2), kappa=np.zeros(2))
        overridden = resolve_belt_speed(original, source='override', override=3.5)
        np.testing.assert_array_equal(overridden.channels['speed'], [3.5, 3.5])
        self.assertEqual(overridden.metadata['belt_speed_source'], 'override')
        restored = resolve_belt_speed(overridden, source='measured')
        np.testing.assert_array_equal(restored.channels['speed'], [.01, .01])
        self.assertEqual(restored.metadata['belt_speed_source'], 'measured')
        for bad in (None, 0, -1, np.inf, np.nan):
            with self.subTest(override=bad), self.assertRaisesRegex(ValueError, 'finite positive'):
                resolve_belt_speed(original, source='override', override=bad)

    def test_auto_without_speed_uses_qualified_wheel_data(self):
        original = self.run_data(omega=np.full(2, 10.), effective_radius=np.full(2, .2), kappa=np.zeros(2))
        resolved = resolve_belt_speed(original)
        np.testing.assert_array_equal(resolved.channels['speed'], [2., 2.])
        self.assertEqual(resolved.metadata['belt_speed_source'], 'wheel')

    def test_rejects_malformed_speed_and_wheel_channel_alignment(self):
        malformed = RunData(7, 'synthetic', {
            'speed': np.array([1., 2.]), 'omega': np.array([10., 10.]),
            'effective_radius': np.array([.2, .2]), 'kappa': np.zeros(2),
        }, {}, np.arange(3))
        with self.assertRaisesRegex(ValueError, 'speed length must match raw_indices'):
            resolve_belt_speed(malformed)

        misaligned_wheel = self.run_data(speed=np.full(2, .01), omega=np.array([10.]),
                                         effective_radius=np.full(2, .2), kappa=np.zeros(2))
        auto = resolve_belt_speed(misaligned_wheel)
        np.testing.assert_array_equal(auto.channels['speed'], [.01, .01])
        self.assertIn('omega length must match raw_indices', auto.metadata['belt_speed_warning'])
        with self.assertRaisesRegex(ValueError, 'omega length must match raw_indices'):
            resolve_belt_speed(misaligned_wheel, source='wheel')

        malformed_override = RunData(7, 'synthetic', {'speed': np.ones((2, 1))}, {}, np.arange(2))
        with self.assertRaisesRegex(ValueError, 'speed must be one-dimensional'):
            resolve_belt_speed(malformed_override, source='override', override=2.)


if __name__ == '__main__':
    unittest.main()
