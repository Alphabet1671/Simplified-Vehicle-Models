from pathlib import Path
import unittest

import numpy as np

from tools.tire_gui.contracts import RunData
from tools.tire_gui.events import TransientEvent, candidate_events
from tools.tire_gui.model import load_snapshot
from tools.tire_gui.transient import fit_transient, simulate_event

STEADY = Path('models/tires/hoosier-43075-r20-simple.xml')


def make_step_run(*, gap=False, mz=True, longitudinal=False):
    t = np.arange(200) * .01
    z = np.zeros_like(t)
    alpha = np.where(t < .2, 0., .03)
    kappa = np.where(t < .2, 0., .05) if longitudinal else z.copy()
    if gap:
        t[100:] += .1
    channels = dict(time=t, alpha=alpha, kappa=kappa, fz=z+660, speed=z+11,
                    omega=z+60, ia=z.copy(), pressure=z+82700,
                    fx=z.copy(), fy=z.copy(), mz=z.copy())
    if not mz:
        channels.pop('mz')
    return RunData(7, 'synthetic', channels, {'source':'synthetic'}, np.arange(t.size))


class EventTests(unittest.TestCase):
    def test_unapproved_event_cannot_fit(self):
        run = make_step_run()
        event = TransientEvent(run, 0, len(run.raw_indices), 20, False, 'lateral', ())
        with self.assertRaisesRegex(ValueError, 'approved'):
            fit_transient(load_snapshot(STEADY), (event,), frozenset(), lambda: False, lambda p: None)

    def test_candidate_uses_exclusive_baseline_and_quality_reasons(self):
        events = candidate_events(make_step_run())
        self.assertTrue(events)
        self.assertEqual(events[0].baseline_stop, 20)
        self.assertFalse(events[0].approved)
        self.assertTrue(any('approval' in s.lower() for s in events[0].exclusions))

    def test_gap_cannot_be_replayed_across_even_if_manually_approved(self):
        run = make_step_run(gap=True)
        event = TransientEvent(run, 0, len(run.raw_indices), 20, True, 'lateral', ())
        with self.assertRaisesRegex(ValueError, 'gap|contiguous'):
            simulate_event(load_snapshot(STEADY), event, .002)

    def test_missing_mz_blocks_trail_fit(self):
        run = make_step_run(mz=False)
        event = TransientEvent(run, 0, len(run.raw_indices), 20, True, 'lateral', ())
        with self.assertRaisesRegex(ValueError, 'Mz|moment'):
            fit_transient(load_snapshot(STEADY), (event,), frozenset(), lambda: False, lambda p: None)

    def test_unexcited_x_stays_unchanged_and_reported_provisional(self):
        run = make_step_run()
        event = TransientEvent(run, 0, len(run.raw_indices), 20, True, 'lateral', ())
        snap = load_snapshot(STEADY)
        result = fit_transient(snap, (event,), frozenset(), lambda: False, lambda p: None)
        self.assertEqual(result.candidate.values['longitudinal_length_scale'], snap.values['longitudinal_length_scale'])
        self.assertIn('provisional', result.status_by_parameter['longitudinal_length_scale'])

    def test_simulation_outputs_finite_for_contiguous_event(self):
        run = make_step_run()
        event = TransientEvent(run, 0, len(run.raw_indices), 20, True, 'lateral', ())
        pred = simulate_event(load_snapshot(STEADY), event, .002)
        self.assertEqual(pred.shape, (200, 3))
        self.assertTrue(np.all(np.isfinite(pred)))


if __name__ == '__main__':
    unittest.main()
