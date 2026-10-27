import unittest
import tempfile
from pathlib import Path
import numpy as np

try:
    import scipy.optimize  # noqa: F401
except ImportError as exc:
    raise unittest.SkipTest(f'tire GUI fit tests require optional scipy: {exc}')

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
        metadata = {'selections': {'load_n': 660, 'groups': ['run8:3'], 'chosen': ['run8:3', 'run72:2']}, 'events': [{'id': 'synthetic:1', 'start': 10, 'stop': 20, 'approved': False, 'reason': 'synthetic'}], 'diagnostics': {'summary': 'synthetic-nested-1-2', 'source': 'synthetic'}}
        from dataclasses import replace
        source = replace(self.seed, session_metadata=metadata)
        result = fit_steady(source,data,locked,progress=lambda _:None)
        self.assertEqual(result.candidate.session_metadata, source.session_metadata)
        from tools.tire_gui.save import save_session, load_session
        with tempfile.TemporaryDirectory() as folder:
            save_session(result.candidate, Path(folder), 'nested-meta', include_transient=True)
            restored = load_session(Path(folder) / 'nested-meta')
            from tools.tire_gui.save import _thaw
            restored_meta, source_meta = _thaw(restored.session_metadata), _thaw(source.session_metadata)
            for key, value in source_meta.items():
                self.assertEqual(restored_meta[key], value)
        self.assertLess(result.metrics['force_objective'], 1e-5)
        self.assertAlmostEqual(result.candidate.values['mu_0_y'], truth.values['mu_0_y'], places=5)
        for name in locked: self.assertEqual(result.candidate.values[name], self.seed.values[name])
        self.assertEqual(self.seed.values['mu_0_y'],before)

    def test_cancel_before_prediction(self):
        with self.assertRaises(FitCancelled):
            fit_steady(self.seed,self.synthesized(self.seed),frozenset(set(FORCE_NAMES+TRAIL_NAMES)-{'mu_0_y'}),cancel=lambda:True)

    def test_nan_prediction_fails_cleanly(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        bad = SimpleNamespace(longitudinal_force=np.full(13,np.nan), lateral_force=np.zeros(13), aligning_moment=np.zeros(13))
        with patch('tools.tire_gui.fit.predict_steady', return_value=bad):
            with self.assertRaisesRegex(ValueError,'nonfinite steady prediction'):
                import tools.tire_gui.fit as module
                module._safe_cost(self.seed,self.synthesized(self.seed),lambda:False)

    def test_bound_saturation_is_reported(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from scipy.special import logit
        def fake_minimize(fun,x0,**kwargs):
            terminal = np.array([logit(1-1e-12)])
            fun(terminal)
            return SimpleNamespace(x=terminal, success=True, nit=2)
        locked=frozenset(set(FORCE_NAMES+TRAIL_NAMES)-{'mu_0_y'})
        truth=with_parameters(self.seed,{'mu_0_y':4.0})
        with patch('tools.tire_gui.fit.minimize',side_effect=fake_minimize):
            result=fit_steady(self.seed,self.synthesized(truth),locked)
        self.assertEqual(result.status_by_parameter['mu_0_y'],'bound-saturated')
        self.assertTrue(any('mu_0_y' in warning and 'bound' in warning for warning in result.warnings))

    def test_nonconvergence_retains_finite_candidate_and_status(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        def fake_minimize(fun,x0,**kwargs):
            fun(x0)
            return SimpleNamespace(x=x0, success=False, nit=3)
        locked=frozenset(set(FORCE_NAMES+TRAIL_NAMES)-{'mu_0_y'})
        with patch('tools.tire_gui.fit.minimize',side_effect=fake_minimize):
            result=fit_steady(self.seed,self.synthesized(self.seed),locked)
        self.assertFalse(result.converged)
        self.assertEqual(result.status_by_parameter['mu_0_y'],'not converged')
        self.assertTrue(np.isfinite(result.metrics['force_objective']))
        self.assertTrue(any('not converge' in warning for warning in result.warnings))
        self.assertEqual(self.seed.values['mu_0_y'],result.candidate.values['mu_0_y'])

    def test_nonconverged_status_uses_retained_best_not_terminal_point(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from scipy.special import expit, logit
        def fake_minimize(fun,x0,**kwargs):
            fun(x0)  # Make the seed the best evaluated candidate.
            terminal = np.array([logit(.999999)])
            return SimpleNamespace(x=terminal, success=False, nit=4)
        locked=frozenset(set(FORCE_NAMES+TRAIL_NAMES)-{'mu_0_y'})
        with patch('tools.tire_gui.fit.minimize',side_effect=fake_minimize):
            result=fit_steady(self.seed,self.synthesized(self.seed),locked)
        self.assertEqual(result.candidate.values['mu_0_y'],self.seed.values['mu_0_y'])
        self.assertEqual(result.status_by_parameter['mu_0_y'],'not converged')
        self.assertFalse(any('mu_0_y' in warning and 'bound' in warning for warning in result.warnings))

    def test_cancel_during_final_metrics_raises(self):
        from unittest.mock import patch
        locked=frozenset(set(FORCE_NAMES+TRAIL_NAMES)-{'mu_0_y'})
        calls=[0]
        def cancellation():
            calls[0]+=1
            # During final metrics the mock predictions are countable and no
            # optimizer objective prediction should occur.
            return calls[0] > 8
        with patch('tools.tire_gui.fit.minimize', side_effect=lambda fun,x0,**kw: (fun(x0), type('Result',(),{'x':x0,'success':True,'nit':1})())[1]):
            with self.assertRaises(FitCancelled):
                fit_steady(self.seed,self.synthesized(self.seed),locked,cancel=cancellation)


if __name__ == '__main__':
    unittest.main()
