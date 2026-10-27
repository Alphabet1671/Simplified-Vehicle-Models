import unittest

import numpy as np

from tools.tire_gui.data import contiguous_slices, normalize_channels
from tools.tire_gui.segments import compatibility_selection, matlab_segment_indices
from tools.tire_gui.contracts import RunData


class DataTests(unittest.TestCase):
    def test_matlab_indices(self):
        np.testing.assert_array_equal(matlab_segment_indices(1, 6, 3), [0, 3, 5])

    def test_gap_is_not_interpolated(self):
        blocks = contiguous_slices(np.array([0., .01, .02, 1.13, 1.14]))
        self.assertEqual([(s.start, s.stop) for s in blocks], [(0, 3), (3, 5)])

    def test_signs_and_units(self):
        raw = {'SA': np.array([3.]), 'FZ': np.array([-660.]), 'V': np.array([36.]),
               'N': np.array([60.]), 'SL': np.array([.1])}
        units = {'SA': 'deg', 'FZ': 'N', 'V': 'kph', 'N': 'rpm', 'SL': 'dimensionless'}
        out = normalize_channels(raw, units)
        self.assertAlmostEqual(out['alpha'][0], -np.pi / 60)
        self.assertEqual(out['fz'][0], 660.)
        self.assertEqual(out['speed'][0], 10.)
        self.assertAlmostEqual(out['omega'][0], 2*np.pi)
        self.assertEqual(out['kappa'][0], .1)

    def test_compatibility_exact_counts_and_provenance(self):
        # Two synthetic runs, long enough to cover all exact source table rows.
        def run(run_id, tire, length):
            n = length
            a = np.zeros(n)
            return RunData(run_id, tire, {'alpha': a, 'kappa': a, 'fz': a + 1000,
                'fx': a, 'fy': a, 'mz': a, 'speed': a + 10, 'omega': a + 10,
                'ia': a, 'pressure': a}, {}, np.arange(n))
        selected = compatibility_selection(run(8, '43075', 26000), run(72, '43100', 26000))
        self.assertEqual(len(selected.lateral.target), 1080)
        self.assertEqual(len(selected.mz.target), 1080)
        self.assertEqual(len(selected.longitudinal.target), 720)
        self.assertEqual(len(selected.combined_fx.target), 1440)
        self.assertEqual(len(selected.combined_fy.target), 1440)
        self.assertEqual(selected.lateral.run_id, 8)
        self.assertEqual(selected.longitudinal.tire_id, '43100')
        with self.assertRaises(ValueError):
            compatibility_selection(run(8, '43075', 1000), run(72, '43100', 26000))


if __name__ == '__main__':
    unittest.main()
