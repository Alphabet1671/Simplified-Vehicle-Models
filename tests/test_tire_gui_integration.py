"""Synthetic end-to-end coverage for the tire correlation workflow."""
from pathlib import Path
import tempfile
import unittest

try:
    import scipy  # noqa: F401
except ImportError as exc:
    raise unittest.SkipTest(f'tire GUI integration requires optional scipy: {exc}')

import numpy as np

from simplified_vehicle_models.tires.unitire_simple import compile_unitire_simple
from simplified_vehicle_models.tires.unitire_transient import compile_unitire_transient
from simplified_vehicle_models.xml.loader import load_model
from tools.tire_gui.contracts import RunData
from tools.tire_gui.events import TransientEvent
from tools.tire_gui.fit import FORCE_NAMES, TRAIL_NAMES, fit_steady
from tools.tire_gui.model import load_snapshot, predict_steady, with_parameters
from tools.tire_gui.save import load_session, save_session
from tools.tire_gui.segments import compatibility_selection
from tools.tire_gui.transient import fit_transient, simulate_event

SEED = Path('models/tires/hoosier-43075-r20-simple.xml')


def _fake_run(run_id, tire_id, count):
    rows = np.arange(count)
    return RunData(run_id, tire_id, {
        'alpha': np.zeros(count), 'kappa': np.zeros(count), 'fz': np.full(count, 660.),
        'fy': np.sin(rows / 37) * 500., 'fx': np.cos(rows / 31) * 300.,
        'mz': np.sin(rows / 43) * 30., 'time': rows * .01,
    }, {}, rows)


def _transient_run(snapshot):
    n = 100
    t = np.arange(n) * .01
    alpha = np.where(t < .3, 0., .04)
    z = np.zeros(n)
    channels = {'time': t, 'alpha': alpha, 'kappa': z, 'fz': z + 660.,
                'speed': z + 11., 'omega': z + 60., 'ia': z,
                'pressure': z + 82700., 'fx': z, 'fy': z, 'mz': z}
    source = RunData(7, 'synthetic-run7', channels, {'source': 'synthetic'}, np.arange(n))
    event = TransientEvent(source, 0, n, 30, True, 'lateral', ())
    replay = simulate_event(snapshot, event, .005)
    observed = dict(channels)
    observed['fy'] = replay[:, 1]
    observed['mz'] = replay[:, 2]
    run = RunData(7, 'synthetic-run7', observed, {'source': 'synthetic'}, np.arange(n))
    return run, TransientEvent(run, 0, n, 30, True, 'lateral', ())


class TireGuiIntegrationTests(unittest.TestCase):
    def test_synthetic_load_fit_replay_manual_save_and_compiler_roundtrip(self):
        seed = load_snapshot(SEED)
        # Exercise the same explicit run-8/run-72 sample generation boundary as
        # compatibility mode, without reading licensed/private MAT files.
        selection = compatibility_selection(_fake_run(8, 'Hoosier 43075 R20', 21168),
                                            _fake_run(72, 'Hoosier 43100 R20 7-inch rim', 25992))
        self.assertEqual(len(selection.lateral.target), 6 * 180)
        self.assertEqual(len(selection.longitudinal.target), 4 * 180)
        self.assertEqual(len(selection.combined_fx.target), 8 * 180)
        self.assertEqual(selection.longitudinal.tire_id, 'Hoosier 43100 R20 7-inch rim')

        # Steady force and trail stages execute through the repository evaluator.
        speed = selection.belt_speed
        groups = (selection.lateral, selection.longitudinal, selection.combined_fx,
                  selection.combined_fy, selection.mz)
        targets = []
        for sample, component in zip(groups, ('lateral_force', 'longitudinal_force',
                                               'longitudinal_force', 'lateral_force', 'aligning_moment')):
            predicted = predict_steady(seed, sample, speed)
            from dataclasses import replace
            targets.append(replace(sample, target=np.array(getattr(predicted, component), copy=True)))
        from tools.tire_gui.contracts import SteadySelection
        exact = SteadySelection(*targets, speed)
        all_steady = frozenset((*FORCE_NAMES, *TRAIL_NAMES))
        steady_result = fit_steady(seed, exact, all_steady - {'mu_0_y', 'd_x_0_1'}, progress=lambda _: None)
        self.assertTrue(np.isfinite(steady_result.metrics['force_objective']))
        self.assertTrue(np.isfinite(steady_result.metrics['trail_objective']))
        self.assertIn('fy_rmse', steady_result.metrics)
        self.assertIn('mz_rmse', steady_result.metrics)
        self.assertEqual(steady_result.metrics['lateral_count'], 1080.)
        self.assertEqual(steady_result.metrics['combined_fy_count'], 1440.)

        # Generate a known synthetic response, then exercise actual replay and a
        # one-parameter transient fit while leaving other axes locked.
        run, event = _transient_run(steady_result.candidate)
        replay = simulate_event(steady_result.candidate, event, .005)
        self.assertEqual(replay.shape, (100, 3))
        self.assertTrue(np.all(np.isfinite(replay)))
        truth = with_parameters(steady_result.candidate, {'lateral_length_scale': 1.4})
        truth_replay = simulate_event(truth, event, .005)
        channels = dict(run.channels)
        channels['fy'] = truth_replay[:, 1]
        channels['mz'] = truth_replay[:, 2]
        observed = RunData(7, 'synthetic-run7', channels, {'source': 'synthetic'}, np.arange(100))
        event = TransientEvent(observed, 0, 100, 30, True, 'lateral', ())
        locked = frozenset({'longitudinal_length_scale', 'trail_pole_ratio', 'trail_lead_ratio'})
        transient_result = fit_transient(steady_result.candidate, (event,), locked, progress=lambda _: None)
        self.assertEqual(transient_result.status_by_parameter['longitudinal_length_scale'], 'unidentified / provisional')
        self.assertTrue(np.isfinite(transient_result.metrics['event_count']))
        self.assertIn(transient_result.status_by_parameter['lateral_length_scale'],
                      ('fitted', 'unidentified / provisional'))

        # A steady manual edit must stale transient evidence. Save the paired XML
        # and session metadata, then compile the emitted files using real loaders.
        edited = with_parameters(transient_result.candidate, {'mu_0_y': transient_result.candidate.values['mu_0_y'] * .99})
        self.assertEqual(edited.provenance['transient_fit_status'], 'stale')
        self.assertTrue(all(edited.parameter_status[n] == 'stale' for n in ('lateral_length_scale', 'trail_pole_ratio', 'trail_lead_ratio')))
        metadata = {'selections': {'run': 8, 'holdout_ids': ['run72_combined:0']},
                    'locks': {'longitudinal_length_scale': True},
                    'bounds': {'mu_0_y': [.5, 4.]},
                    'events': [{'id': 'synthetic-event', 'start': 0., 'stop': .99,
                                'baseline_stop': .3, 'run_id': 7, 'axis': 'lateral', 'approved': True}],
                    'source_files': ['synthetic://run7']}
        with tempfile.TemporaryDirectory() as folder:
            paths = save_session(edited, Path(folder), 'roundtrip', include_transient=True, metadata=metadata)
            self.assertEqual(len(paths), 3)
            steady_model = compile_unitire_simple(load_model(paths[0]))
            transient_model = compile_unitire_transient(load_model(paths[1]))
            self.assertEqual(steady_model.mu_0_y, edited.values['mu_0_y'])
            self.assertEqual(transient_model.lateral_length_scale, edited.values['lateral_length_scale'])
            restored = load_session(Path(folder) / 'roundtrip')
            self.assertEqual(restored.steady_fingerprint, edited.steady_fingerprint)
            self.assertEqual(restored.parameter_status['longitudinal_length_scale'], 'provisional')
            self.assertEqual(restored.session_metadata['events'][0]['baseline_stop'], .3)
            self.assertEqual(restored.provenance['transient_fit_status'], 'stale')


if __name__ == '__main__':
    unittest.main()
