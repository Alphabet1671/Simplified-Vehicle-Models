import unittest
from pathlib import Path
import tempfile

import numpy as np

try:
    from scipy.io import savemat
except ImportError as exc:
    raise unittest.SkipTest(f'tire GUI data tests require optional scipy: {exc}')

from tools.tire_gui.data import contiguous_slices, normalize_channels, load_run
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

    def test_ambiguous_aliases_are_rejected_but_sl_precedes_sr(self):
        with self.assertRaisesRegex(ValueError, 'Ambiguous.*alpha'):
            normalize_channels({'SA': np.array([1.]), 'SLA': np.array([1.])}, {'SA': 'deg', 'SLA': 'deg'})
        out = normalize_channels({'SL': np.array([.2]), 'SR': np.array([.7])},
                                 {'SL': 'dimensionless', 'SR': 'dimensionless'})
        self.assertEqual(out['kappa'][0], .2)

    def test_invalid_gap_threshold(self):
        for gap in (0., -1., np.nan, np.inf):
            with self.subTest(gap=gap), self.assertRaisesRegex(ValueError, 'maximum_gap'):
                contiguous_slices(np.array([0., .01]), gap)

    def test_synthetic_mat_loader_happy_and_descriptor_error(self):
        channel = {'name': np.array(['ET', 'SA', 'FZ', 'FX', 'FY', 'MZ', 'V', 'N', 'SL', 'IA', 'P']),
                   'units': np.array(['sec', 'deg', 'N', 'N', 'N', 'N-m', 'kph', 'rpm', 'none', 'deg', 'kPa'])}
        arrays = {'ET': np.array([0., .01]), 'SA': np.array([1., 2.]), 'FZ': np.array([-10., -20.]),
                  'FX': np.array([1., 2.]), 'FY': np.array([3., 4.]), 'MZ': np.array([.1, .2]),
                  'V': np.array([36., 36.]), 'N': np.array([60., 60.]), 'SL': np.array([.1, .2]),
                  'IA': np.array([0., 0.]), 'P': np.array([100., 100.]), 'RUN': np.array([8, 8]),
                  'tireid': 'Hoosier 43075 R20'}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'synthetic.mat'
            savemat(path, {**arrays, 'channel': channel})
            loaded = load_run(path)
            self.assertEqual((loaded.run_id, loaded.tire_id), (8, 'Hoosier 43075 R20'))
            np.testing.assert_array_equal(loaded.channels['fz'], [10., 20.])
            bad = Path(tmp) / 'bad.mat'
            savemat(bad, arrays)
            with self.assertRaisesRegex(ValueError, 'descriptor'):
                load_run(bad)

    def test_nonmonotonic_time_strict_by_default_and_retained_for_steady_mode(self):
        channel = {'name': np.array(['ET', 'SA', 'FZ', 'FX', 'FY', 'MZ', 'SL']),
                   'units': np.array(['sec', 'deg', 'N', 'N', 'N', 'N-m', 'none'])}
        arrays = {'ET': np.array([0., .01, .01, .03]), 'SA': np.arange(4.),
                  'FZ': -np.full(4, 100.), 'FX': np.arange(4.), 'FY': np.arange(4.),
                  'MZ': np.arange(4.), 'SL': np.arange(4.)/10, 'RUN': np.full(4, 72),
                  'tireid': 'Hoosier 43100 R20'}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'duplicate-time.mat'
            savemat(path, {**arrays, 'channel': channel})
            with self.assertRaisesRegex(ValueError, 'nonmonotonic'):
                load_run(path)
            steady = load_run(path, require_monotonic_time=False)
            np.testing.assert_array_equal(steady.channels['time'], arrays['ET'])
            np.testing.assert_array_equal(steady.raw_indices, np.arange(4))
            self.assertIn('nonmonotonic', steady.metadata['time_warning'])

    def test_compatibility_rejects_wrong_tire_identity(self):
        def run(run_id, tire):
            a = np.zeros(26000)
            return RunData(run_id, tire, {'alpha': a, 'kappa': a, 'fz': a+1, 'fx': a,
                'fy': a, 'mz': a, 'speed': a, 'omega': a, 'ia': a, 'pressure': a}, {}, np.arange(len(a)))
        with self.assertRaisesRegex(ValueError, '43075'):
            compatibility_selection(run(8, '43100 R20'), run(72, '43100 R20'))
        with self.assertRaisesRegex(ValueError, '43100'):
            compatibility_selection(run(8, '43075 R20'), run(72, '43075 R20'))

    def test_compatibility_exact_counts_and_provenance(self):
        # Two synthetic runs, long enough to cover all exact source table rows.
        def run(run_id, tire, length):
            n = length
            a = np.zeros(n)
            return RunData(run_id, tire, {'alpha': a, 'kappa': a, 'fz': a + 1000,
                'fx': a, 'fy': a, 'mz': a, 'speed': a + 10, 'omega': a + 10,
                'ia': a, 'pressure': a}, {}, np.arange(n))
        selected = compatibility_selection(run(8, '43075 R20', 26000), run(72, '43100 R20', 26000))
        self.assertEqual(len(selected.lateral.target), 1080)
        self.assertEqual(len(selected.mz.target), 1080)
        self.assertEqual(len(selected.longitudinal.target), 720)
        self.assertEqual(len(selected.combined_fx.target), 1440)
        self.assertEqual(len(selected.combined_fy.target), 1440)
        self.assertEqual(selected.lateral.run_id, 8)
        self.assertEqual(selected.longitudinal.tire_id, '43100 R20')
        with self.assertRaises(ValueError):
            compatibility_selection(run(8, '43075 R20', 1000), run(72, '43100 R20', 26000))


if __name__ == '__main__':
    unittest.main()
