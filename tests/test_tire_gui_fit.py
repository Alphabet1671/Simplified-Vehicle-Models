import unittest
from pathlib import Path
import numpy as np

from tools.tire_gui.fit import force_cost, fit_steady, FORCE_NAMES, TRAIL_NAMES
from tools.tire_gui.contracts import SampleSet, SteadySelection, FitCancelled
from tools.tire_gui.model import load_snapshot, predict_steady, with_parameters

STEADY = Path('models/tires/hoosier-43075-r20-simple.xml')


class ObjectiveTests(unittest.TestCase):
    def test_separate_group_rms(self):
        self.assertEqual(force_cost(tuple(np.array([v]) for v in (1., 2., 3., 4.))), 8.25)

    def test_empty_group_is_explicit(self):
        with self.assertRaises(ValueError):
            force_cost((np.array([]),) * 4)


def _samples(alpha, kappa, load):
    n = len(alpha)
    return SampleSet(np.asarray(alpha), np.asarray(kappa), np.full(n, load), np.zeros(n), np.zeros(n, dtype=int))


class FitTests(unittest.TestCase):
    def setUp(self):
        self.seed = load_snapshot(STEADY)
        self.speed = 40.193 / 3.6
        self.lat = _samples(np.linspace(-.12,.12,13), np.zeros(13), 889.)
        self.lon = _samples(np.zeros(13), np.linspace(-.2,.2,13), 889.)
        self.cfx = _samples(np.linspace(-.1,.1,13), np.linspace(-.15,.15,13), 660.)
        self.cfy = _samples(np.linspace(-.1,.1,13), np.linspace(-.15,.15,13), 660.)
        self.mz = self.lat
        self.data = SteadySelection(self.lat,self.lon,self.cfx,self.cfy,self.mz,self.speed)

    def synthesized(self, model):
        def target(samples, axis):
            pred = predict_steady(model,samples,self.speed)
            return getattr(pred,axis).copy()
        from dataclasses import replace
        return SteadySelection(*(replace(s,target=target(s,axis)) for s,axis in [
            (self.lat,'lateral_force'),(self.lon,'longitudinal_force'),(self.cfx,'longitudinal_force'),
            (self.cfy,'lateral_force'),(self.mz,'aligning_moment')]),self.speed)

    def test_truth_gives_zero_residual_objective(self):
        import tools.tire_gui.fit as module
        data = self.synthesized(self.seed)
        self.assertEqual(module._safe_cost(self.seed,data,lambda:False),0.0)

    def test_fit_improves_one_unlocked_mu_and_preserves_locks(self):
        truth = with_parameters(self.seed, {'mu_0_y': self.seed.values['mu_0_y'] * .85})
        data = self.synthesized(truth)
        locked = frozenset(set(FORCE_NAMES+TRAIL_NAMES)-{'mu_0_y'})
        before = self.seed.values['mu_0_y']
        result = fit_steady(self.seed,data,locked,progress=lambda _:None)
        self.assertLess(result.metrics['force_objective'], 1e-5)
        self.assertAlmostEqual(result.candidate.values['mu_0_y'], truth.values['mu_0_y'], places=5)
        for name in locked: self.assertEqual(result.candidate.values[name], self.seed.values[name])
        self.assertEqual(self.seed.values['mu_0_y'],before)

    def test_cancel_before_prediction(self):
        with self.assertRaises(FitCancelled):
            fit_steady(self.seed,self.synthesized(self.seed),frozenset(set(FORCE_NAMES+TRAIL_NAMES)-{'mu_0_y'}),cancel=lambda:True)


if __name__ == '__main__':
    unittest.main()
