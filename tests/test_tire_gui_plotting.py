"""Model display sweeps must not connect different experimental conditions."""
import unittest
import numpy as np
from tools.tire_gui.contracts import SampleSet
from tools.tire_gui.plotting import model_sweeps


class ModelSweepTests(unittest.TestCase):
    def samples(self):
        return SampleSet(np.array([-.1, -.1, .1, .1, -.1, .1]),
                         np.array([-.2, -.2, .2, .2, -.2, .2]),
                         np.array([400., 800., 400., 800., 400., 400.]),
                         np.arange(6.), np.array([0, 0, 0, 0, 2, 2]))

    def test_interleaved_loads_and_repeated_segments_are_separate(self):
        samples = self.samples()
        sweeps = model_sweeps(samples, 'alpha')
        self.assertEqual(len(sweeps), 3)
        for sweep in sweeps:
            self.assertEqual(len(sweep.alpha), 121)
            self.assertEqual(len(np.unique(sweep.fz)), 1)
            self.assertEqual(len(np.unique(sweep.segment_ids)), 1)
            self.assertTrue(np.all(np.diff(sweep.alpha) > 0))
            self.assertEqual(len(np.unique(sweep.kappa)), 1)
        np.testing.assert_array_equal(samples.target, np.arange(6.))
        self.assertFalse(samples.alpha.flags.writeable)

    def test_combined_sweeps_hold_alpha_and_load_constant(self):
        sweeps = model_sweeps(self.samples(), 'kappa', count=41)
        for sweep in sweeps:
            self.assertEqual(len(sweep.kappa), 41)
            self.assertTrue(np.all(np.diff(sweep.kappa) > 0))
            self.assertEqual(len(np.unique(sweep.alpha)), 1)
            self.assertEqual(len(np.unique(sweep.fz)), 1)

    def test_empty_and_invalid_variable(self):
        empty = SampleSet(*(np.array([]) for _ in range(5)))
        self.assertEqual(model_sweeps(empty, 'alpha'), ())
        with self.assertRaises(ValueError):
            model_sweeps(self.samples(), 'fz')
