from pathlib import Path
import unittest

import numpy as np

try:
    import scipy.integrate  # noqa: F401
    import scipy.optimize  # noqa: F401
except ImportError as exc:
    raise unittest.SkipTest(f'tire GUI transient tests require optional scipy: {exc}')

from tools.tire_gui.contracts import RunData, FitCancelled
from tools.tire_gui.events import TransientEvent, candidate_events
from tools.tire_gui.model import load_snapshot
from tools.tire_gui.transient import fit_transient, simulate_event
from tools.tire_gui.model import with_parameters

STEADY = Path('models/tires/hoosier-43075-r20-simple.xml')


def make_step_run(*, gap=False, mz=True, longitudinal=False, variable_speed=False, variable_load=False, dt=.01, n=200):
    t = np.arange(n) * dt
    z = np.zeros_like(t)
    alpha = np.where(t < .2, 0., .03)
    kappa = np.where(t < .2, 0., .05) if longitudinal else z.copy()
    if gap:
        t[100:] += .1
    speed = z+11 if not variable_speed else 8. + 6. * t / t[-1]
    load = z+660 if not variable_load else 500. + 300. * t / t[-1]
    channels = dict(time=t, alpha=alpha, kappa=kappa, fz=load, speed=speed,
                    omega=speed / .3, ia=z.copy(), pressure=z+82700,
                    fx=z.copy(), fy=z.copy(), mz=z.copy())
    if not mz:
        channels.pop('mz')
    return RunData(7, 'synthetic', channels, {'source':'synthetic'}, np.arange(t.size))


def approved(run, axis='lateral'):
    return TransientEvent(run, 0, len(run.raw_indices), 20, True, axis, ())


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
        self.assertEqual(events[0].start, 0)
        self.assertFalse(events[0].approved)
        self.assertTrue(any('approval' in s.lower() for s in events[0].exclusions))

    def test_candidate_records_baseline_load_excursion(self):
        run=make_step_run()
        channels=dict(run.channels)
        channels['fz']=channels['fz'].copy()
        channels['fz'][16]=500.
        changed=RunData(run.run_id,run.tire_id,channels,run.metadata,run.raw_indices)
        event=candidate_events(changed)[0]
        self.assertTrue(any('load' in reason for reason in event.exclusions))

    def test_fit_rejects_late_event_condition_excursions_but_replay_allows_them(self):
        snap=load_snapshot(STEADY)
        for channel,value,reason in (
                ('fz',800.,'load'),('ia',np.deg2rad(.4),'inclination'),
                ('pressure',87000.,'pressure'),('speed',13.,'speed')):
            with self.subTest(channel=channel):
                run=make_step_run()
                channels=dict(run.channels)
                channels[channel]=channels[channel].copy()
                channels[channel][120:]=value
                changed=RunData(run.run_id,run.tire_id,channels,run.metadata,run.raw_indices)
                event=approved(changed)
                replay=simulate_event(snap,event,.005)
                self.assertTrue(np.all(np.isfinite(replay)))
                with self.assertRaisesRegex(ValueError,reason):
                    fit_transient(snap,(event,),frozenset(),lambda:False,lambda p:None)

    def test_candidate_flags_late_condition_excursion(self):
        run=make_step_run()
        channels=dict(run.channels)
        channels['pressure']=channels['pressure'].copy()
        channels['pressure'][80:]+=4000.
        changed=RunData(run.run_id,run.tire_id,channels,run.metadata,run.raw_indices)
        event=candidate_events(changed)[0]
        self.assertTrue(any('pressure' in reason for reason in event.exclusions))

    def test_gap_cannot_be_replayed_across_even_if_manually_approved(self):
        run = make_step_run(gap=True)
        event = TransientEvent(run, 0, len(run.raw_indices), 20, True, 'lateral', ())
        with self.assertRaisesRegex(ValueError, 'gap|contiguous'):
            simulate_event(load_snapshot(STEADY), event, .002)

    def test_missing_mz_blocks_trail_fit(self):
        run = make_step_run(mz=False)
        event = approved(run)
        result=fit_transient(load_snapshot(STEADY),(event,),frozenset({'lateral_length_scale','longitudinal_length_scale'}),lambda:False,lambda p:None)
        self.assertIn('Mz unavailable',result.status_by_parameter['trail_pole_ratio'])

    def test_unexcited_x_stays_unchanged_and_reported_provisional(self):
        run = make_step_run()
        event = TransientEvent(run, 0, len(run.raw_indices), 20, True, 'lateral', ())
        snap = load_snapshot(STEADY)
        result = fit_transient(snap, (event,), frozenset(), lambda: False, lambda p: None)
        self.assertEqual(result.candidate.values['longitudinal_length_scale'], snap.values['longitudinal_length_scale'])
        self.assertIn('provisional', result.status_by_parameter['longitudinal_length_scale'])

    def test_simulation_outputs_finite_for_contiguous_event(self):
        run = make_step_run()
        event = approved(run)
        pred = simulate_event(load_snapshot(STEADY), event, .002)
        self.assertEqual(pred.shape, (200, 3))
        self.assertTrue(np.all(np.isfinite(pred)))

    def test_measured_speed_changes_transient_replay(self):
        snap = load_snapshot(STEADY)
        constant = make_step_run()
        varied = make_step_run(variable_speed=True)
        a = simulate_event(snap, approved(constant), .005)
        b = simulate_event(snap, approved(varied), .005)
        self.assertGreater(float(np.max(np.abs(a[:,1]-b[:,1]))), 1e-5)

    def test_variable_load_replay_remains_finite_and_changes_response(self):
        snap = load_snapshot(STEADY)
        constant = make_step_run()
        varied = make_step_run(variable_load=True)
        a = simulate_event(snap, approved(constant), .005)
        b = simulate_event(snap, approved(varied), .005)
        self.assertTrue(np.all(np.isfinite(b)))
        self.assertGreater(float(np.max(np.abs(a[:,1]-b[:,1]))), 1e-4)

    def test_halved_maximum_step_converges(self):
        snap = load_snapshot(STEADY)
        event = approved(make_step_run(variable_load=True, variable_speed=True))
        coarse=simulate_event(snap,event,.01)
        fine=simulate_event(snap,event,.005)
        finer=simulate_event(snap,event,.0025)
        self.assertLess(np.max(np.abs(fine-finer)), np.max(np.abs(coarse-fine)) + 1e-10)
        np.testing.assert_allclose(fine,finer,rtol=2e-4,atol=2e-3)

    def test_steady_limit_matches_steady_evaluator(self):
        from simplified_vehicle_models.tires.unitire_transient import steady_rolling_state, evaluate_transient_unitire
        from simplified_vehicle_models.tires.unitire_simple import evaluate_unitire
        snap=load_snapshot(STEADY)
        alpha=.03; fz=660.; speed=11.
        state=steady_rolling_state(snap.steady,snap.transient,alpha,0.,fz)
        force,rates=evaluate_transient_unitire(snap.steady,snap.transient,state,alpha,0.,fz,60.,speed)
        steady=evaluate_unitire(snap.steady,alpha,0.,fz,60.)
        self.assertAlmostEqual(force.lateral_force,steady.lateral_force,places=7)
        self.assertAlmostEqual(force.aligning_moment,steady.aligning_moment,places=7)
        self.assertAlmostEqual(rates.lateral_deformation,0.,places=12)

    def test_cancellation_during_replay_raises(self):
        run=make_step_run()
        calls=[0]
        def cancel():
            calls[0]+=1
            return calls[0]>3
        with self.assertRaises(FitCancelled):
            simulate_event(load_snapshot(STEADY),approved(run),.002,_cancel_check=cancel)

    def test_one_trail_lock_does_not_prevent_other_from_fitting(self):
        snap=load_snapshot(STEADY)
        event=approved(make_step_run())
        # Fit with only trail_lead_ratio unlocked; the locked pole ratio is fixed.
        result=fit_transient(snap,(event,),frozenset({'lateral_length_scale','trail_pole_ratio'}),lambda:False,lambda p:None)
        self.assertEqual(result.candidate.values['trail_pole_ratio'],snap.values['trail_pole_ratio'])
        self.assertEqual(result.status_by_parameter['trail_pole_ratio'],'locked')
        self.assertNotEqual(result.status_by_parameter['trail_lead_ratio'],'unchanged (fit group locked)')

    def test_missing_mz_does_not_invalidate_successful_fy_scale(self):
        snap=load_snapshot(STEADY)
        event=approved(make_step_run(mz=False))
        target=with_parameters(snap,{'lateral_length_scale':1.5})
        channels=dict(event.run.channels)
        pred=simulate_event(target,approved(make_step_run()),.005)
        channels['fy']=pred[:,1]
        run=RunData(event.run.run_id,event.run.tire_id,channels,event.run.metadata,event.run.raw_indices)
        result=fit_transient(snap,(approved(run),),frozenset({'trail_pole_ratio','trail_lead_ratio','longitudinal_length_scale'}),lambda:False,lambda p:None)
        self.assertIn(result.status_by_parameter['lateral_length_scale'],('fitted','unidentified / provisional'))
        self.assertEqual(result.status_by_parameter['trail_pole_ratio'],'locked')

    def test_cancel_before_fit(self):
        event=approved(make_step_run())
        with self.assertRaises(FitCancelled):
            fit_transient(load_snapshot(STEADY),(event,),frozenset(),lambda:True,lambda p:None)

    def test_single_active_trail_parameter_recovers_known_coefficient(self):
        snap=load_snapshot(STEADY)
        source=make_step_run(dt=.001,n=2000)
        event=approved(source)
        truth=with_parameters(snap,{'trail_pole_ratio':1.6})
        lateral=simulate_event(truth,event,.001)
        # Prevent the force stage from consuming this Mz-identification case;
        # hold lateral length at its trusted seed and retain measured Fy.
        target=simulate_event(truth,event,.001)
        channels=dict(source.channels); channels['mz']=target[:,2]; channels['fy']=lateral[:,1]
        run=RunData(source.run_id,source.tire_id,channels,source.metadata,source.raw_indices)
        locks=frozenset({'lateral_length_scale','trail_lead_ratio','longitudinal_length_scale'})
        result=fit_transient(snap,(approved(run),),locks,lambda:False,lambda p:None)
        self.assertEqual(result.status_by_parameter['lateral_length_scale'],'locked')
        self.assertEqual(result.status_by_parameter['trail_pole_ratio'],'fitted')
        self.assertAlmostEqual(result.candidate.values['trail_pole_ratio'],1.6,places=2)

    def test_single_active_longitudinal_parameter_recovers_known_coefficient(self):
        snap=load_snapshot(STEADY)
        run=make_step_run(dt=.005,n=400,longitudinal=True)
        event=approved(run,'longitudinal')
        truth=with_parameters(snap,{'longitudinal_length_scale':1.7})
        target=simulate_event(truth,event,.001)
        channels=dict(run.channels); channels['fx']=target[:,0]
        run=RunData(run.run_id,run.tire_id,channels,run.metadata,run.raw_indices)
        result=fit_transient(snap,(approved(run,'longitudinal'),),frozenset({'lateral_length_scale','trail_pole_ratio','trail_lead_ratio'}),lambda:False,lambda p:None)
        self.assertEqual(result.status_by_parameter['longitudinal_length_scale'],'fitted')
        self.assertAlmostEqual(result.candidate.values['longitudinal_length_scale'],1.7,places=3)

    def test_candidate_bound_hit_is_not_reported_as_fitted(self):
        snap=load_snapshot(STEADY)
        source=make_step_run(dt=.005,n=400)
        from types import SimpleNamespace
        from unittest.mock import patch
        event=approved(source)
        def hit_bound(fun,x0,bounds,**kwargs):
            return SimpleNamespace(x=np.array([bounds[1][0]]),jac=np.ones((1,1)),success=True,cost=0.)
        with patch('tools.tire_gui.transient.least_squares',side_effect=hit_bound):
            result=fit_transient(snap,(event,),frozenset({'trail_pole_ratio','trail_lead_ratio','longitudinal_length_scale'}),lambda:False,lambda p:None)
        self.assertTrue(result.metrics['lateral_length_scale_bound_hit'])
        self.assertNotEqual(result.status_by_parameter['lateral_length_scale'],'fitted')

    def test_single_active_lateral_parameter_recovers_known_coefficient(self):
        snap=load_snapshot(STEADY)
        source=make_step_run(dt=.005,n=400)
        truth=with_parameters(snap,{'lateral_length_scale':1.8})
        target=simulate_event(truth,approved(source),.001)
        channels=dict(source.channels); channels['fy']=target[:,1]
        run=RunData(source.run_id,source.tire_id,channels,source.metadata,source.raw_indices)
        result=fit_transient(snap,(approved(run),),frozenset({'trail_pole_ratio','trail_lead_ratio','longitudinal_length_scale'}),lambda:False,lambda p:None)
        self.assertEqual(result.status_by_parameter['lateral_length_scale'],'fitted')
        self.assertAlmostEqual(result.candidate.values['lateral_length_scale'],1.8,places=3)
        self.assertAlmostEqual(result.candidate.values['trail_pole_ratio'],snap.values['trail_pole_ratio'])
        self.assertIn('lateral_nyquist_hz',result.metrics)
        self.assertIn('lateral_excited_input_bandwidth_95_hz',result.metrics)

    def test_candidate_response_time_gate_reverts_scale(self):
        snap=load_snapshot(STEADY)
        source=make_step_run(dt=.019,n=210)
        truth=with_parameters(snap,{'lateral_length_scale':.2})
        target=simulate_event(truth,approved(source),.005)
        channels=dict(source.channels); channels['fy']=target[:,1]
        run=RunData(source.run_id,source.tire_id,channels,source.metadata,source.raw_indices)
        result=fit_transient(snap,(approved(run),),frozenset({'trail_pole_ratio','trail_lead_ratio','longitudinal_length_scale'}),lambda:False,lambda p:None)
        self.assertNotEqual(result.status_by_parameter['lateral_length_scale'],'fitted')
        self.assertEqual(result.candidate.values['lateral_length_scale'],snap.values['lateral_length_scale'])


if __name__ == '__main__':
    unittest.main()
