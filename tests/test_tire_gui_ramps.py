import unittest

import numpy as np

from tools.tire_gui.contracts import RunData
from tools.tire_gui.events import TransientEvent
from tools.tire_gui.transient_plotting import ramp_events, isolated_step_events, grouped_ramps, aligned_time


def make_run(*, speed=24/3.6, load=500., dt=.01, jitter=0., gap=False, kappa=False):
    # Long zero prehistory, two triangular cycles, and recovery after each leg.
    time = np.arange(240, dtype=float) * dt
    base = np.zeros_like(time)
    source = np.array([0., 0., .04, .08, .04, 0., -.04, -.08, -.04, 0., 0.])
    knots = np.linspace(0, len(time)-1, len(source), dtype=int)
    command = np.interp(np.arange(len(time)), knots, source)
    if jitter:
        command = command + jitter*np.sin(np.arange(len(time))*2.1)
    if kappa:
        alpha, slip = base, command
    else:
        alpha, slip = command, base
    if gap:
        time[120:] += .1
    channels = dict(time=time, alpha=alpha, kappa=slip, fz=np.full(len(time), load),
                    speed=np.full(len(time), speed), omega=np.full(len(time), speed/.3),
                    ia=base, pressure=base+82000, fx=base, fy=base)
    return RunData(9, 'synthetic', channels, {'source': 'synthetic'}, np.arange(len(time)))


class RampTests(unittest.TestCase):
    def test_detects_all_triangular_steering_legs_and_aligns_command(self):
        run = make_run(jitter=.00015)
        events = ramp_events(run)
        self.assertGreaterEqual(len(events), 3)
        self.assertTrue(all(e.axis == 'lateral' for e in events))
        groups = grouped_ramps(events)
        self.assertTrue(groups)
        for group in groups:
            for event in group.events:
                aligned = group.aligned_time(event)
                self.assertEqual(len(aligned), event.stop-event.start)
                self.assertLessEqual(float(np.min(np.abs(aligned))), .02)

    def test_kappa_only_yields_longitudinal_ramps(self):
        events = ramp_events(make_run(kappa=True))
        self.assertTrue(events)
        self.assertTrue(all(event.axis == 'longitudinal' for event in events))

    def test_does_not_cross_gap_or_emit_short_prehistory_events(self):
        events = ramp_events(make_run(gap=True))
        time = events[0].run.channels['time']
        for event in events:
            self.assertLessEqual(float(np.max(np.diff(time[event.start:event.stop]))), .02)
            self.assertGreaterEqual(event.baseline_stop-event.start, 20)

    def test_groups_load_levels_but_separates_speed_and_rate(self):
        def event(run, onset, stop, baseline_stop, axis='lateral'):
            return TransientEvent(run, onset, stop, baseline_stop, False, axis, ())
        def ramp(speed, rate, load):
            n=100; t=np.arange(n)*.01; x=np.zeros(n); x[25:65]=np.linspace(0,rate*.39,40); x[65:]=x[64]
            z=np.zeros(n); c=dict(time=t,alpha=x,kappa=z,fz=z+load,speed=z+speed,omega=z+speed/.3,ia=z,pressure=z+82000,fx=z,fy=z)
            return RunData(1,'synthetic',c,{},np.arange(n))
        runs=[ramp(8.,.05,400.),ramp(8.02,.051,700.),ramp(20.,.05,500.),ramp(8.,.15,500.)]
        ev=[event(r,5,75,25) for r in runs]
        groups=grouped_ramps(ev)
        self.assertEqual(sorted(len(g.events) for g in groups),[1,1,2])
        self.assertTrue(all(g.axis=='lateral' for g in groups))
        self.assertAlmostEqual(groups[0].steer_rate, np.rad2deg(.05), delta=.01)
        self.assertEqual(groups[0].secondary_slip, 0.)

    def test_holds_and_slow_jitter_do_not_fragment_ramps(self):
        n=300; t=np.arange(n)*.01; x=np.zeros(n)
        x[40:160]=np.linspace(0,.08,120); x[160:200]=.08
        x[200:260]=np.linspace(.08,0,60); x[260:]=0
        x += .0001*np.sin(np.arange(n)*2.7)
        z=np.zeros(n); c=dict(time=t,alpha=x,kappa=z,fz=z+500,speed=z+10,omega=z+30,ia=z,pressure=z+82000,fx=z,fy=z)
        run=RunData(3,'synthetic',c,{},np.arange(n))
        events=ramp_events(run)
        self.assertEqual(len(events),2)

    def test_time_reset_is_a_hard_boundary_and_standalone_aligner(self):
        run=make_run()
        channels=dict(run.channels)
        channels['time']=channels['time'].copy(); channels['time'][120:] -= 1.2
        reset=RunData(run.run_id,run.tire_id,channels,run.metadata,run.raw_indices)
        events=ramp_events(reset)
        self.assertGreaterEqual(len(events),3)
        for event in events:
            self.assertTrue(np.all(np.diff(channels['time'][event.start:event.stop])>0))
            self.assertEqual(len(aligned_time(event)),event.stop-event.start)

    def test_midpoint_alignment_matches_analytic_linear_command_crossing(self):
        dt=.01; t=np.arange(120)*dt; alpha=np.zeros(120)
        alpha[30:71]=np.linspace(0.,.08,41); alpha[71:]=.08
        zero=np.zeros(120); c=dict(time=t,alpha=alpha,kappa=zero,fz=zero+500,speed=zero+10,omega=zero+30,ia=zero,pressure=zero+82000,fx=zero,fy=zero)
        run=RunData(4,'synthetic',c,{},np.arange(120))
        event=TransientEvent(run,10,76,30,False,'lateral',())
        aligned=aligned_time(event,direction=1)
        # The measured command rises 0 to .08 rad in .40 s; midpoint at t=.50.
        self.assertAlmostEqual(t[event.start]-aligned[0], .50, places=6)

    def test_conditions_and_tire_are_not_merged(self):
        # Reuse construction from the grouping test with a different tire identity.
        n=100; t=np.arange(n)*.01; x=np.zeros(n); x[25:65]=np.linspace(0,.02,40); x[65:]=x[64]
        z=np.zeros(n)
        def run(tire, ia=0., pressure=82000., kappa=0.):
            c=dict(time=t,alpha=x,kappa=z+kappa,fz=z+500,speed=z+10,omega=z+30,ia=z+ia,pressure=z+pressure,fx=z,fy=z)
            return RunData(9,tire,c,{},np.arange(n))
        events=[]
        for r in (run('tireA'),run('tireB'),run('tireA',ia=np.deg2rad(.5)),run('tireA',pressure=86000.),run('tireA',kappa=.03)):
            events.append(TransientEvent(r,5,75,25,False,'lateral',()))
        groups=grouped_ramps(events)
        self.assertEqual(len(groups),4)
        self.assertEqual(len(grouped_ramps(reversed(events))),4)

    def test_isolated_step_requires_zero_ia_and_sustained_holds(self):
        n=140; t=np.arange(n)*.01; alpha=np.zeros(n)
        alpha[40:45]=np.linspace(0,np.deg2rad(1),5); alpha[45:]=np.deg2rad(1)
        z=np.zeros(n); ia=z+np.deg2rad(.02)*np.sin(np.arange(n)*.7)
        c=dict(time=t,alpha=alpha,kappa=z,fz=z+500,speed=z,omega=z,ia=ia,pressure=z+82000,fx=z,fy=z)
        run=RunData(7,'synthetic',c,{},np.arange(n))
        events=isolated_step_events(run)
        self.assertEqual(len(events),1)
        event=events[0]
        self.assertGreaterEqual(event.baseline_stop-event.start,20)
        self.assertGreaterEqual(t[event.stop-1]-t[event.baseline_stop],.3)
        self.assertTrue(grouped_ramps(events, require_moving=False))
        self.assertFalse(grouped_ramps(events))

    def test_isolated_step_rejects_sweeps_and_nonzero_or_missing_ia(self):
        n=180; t=np.arange(n)*.01; x=np.zeros(n)
        x[30:80]=np.linspace(0,np.deg2rad(2),50); x[80:130]=np.linspace(np.deg2rad(2),0,50); x[130:]=0
        z=np.zeros(n)
        def make(ia=True, value=0.):
            c=dict(time=t,alpha=x,kappa=z,fz=z+500,speed=z,omega=z,pressure=z+82000,fx=z,fy=z)
            if ia: c['ia']=z+value
            return RunData(8,'synthetic',c,{},np.arange(n))
        self.assertFalse(isolated_step_events(make()))
        self.assertFalse(isolated_step_events(make(value=np.deg2rad(.3))))
        self.assertFalse(isolated_step_events(make(ia=False)))

    def test_isolated_detector_accepts_single_sample_command_step(self):
        n=120; t=np.arange(n)*.01; x=np.zeros(n); x[35:39]=np.linspace(0,np.deg2rad(1.),4); x[39:]=np.deg2rad(1.)
        z=np.zeros(n); c=dict(time=t,alpha=x,kappa=z,fz=z+500,speed=z,omega=z,ia=z,pressure=z+82000,fx=z,fy=z)
        run=RunData(7,'synthetic',c,{},np.arange(n))
        events=isolated_step_events(run)
        self.assertEqual(len(events),1)
        self.assertTrue(grouped_ramps(events, require_moving=False))

    def test_isolated_detector_rejects_sine_and_triangle_sweeps(self):
        n=500; t=np.arange(n)*.01; x=np.zeros(n)
        phase=np.linspace(0,np.pi,201)
        x[50:251]=np.deg2rad(2)*np.sin(phase)
        x[300:500]=np.deg2rad(2)*np.r_[np.linspace(0,1,100),np.linspace(1,0,100)]
        z=np.zeros(n); c=dict(time=t,alpha=x,kappa=z,fz=z+500,speed=z,omega=z,ia=z,pressure=z+82000,fx=z,fy=z)
        run=RunData(7,'synthetic',c,{},np.arange(n))
        self.assertFalse(isolated_step_events(run))

    def test_isolated_step_does_not_join_later_command_after_hold(self):
        n=240; t=np.arange(n)*.01; x=np.zeros(n)
        x[30:35]=np.linspace(0,np.deg2rad(1),5); x[35:110]=np.deg2rad(1)
        x[110:115]=np.linspace(np.deg2rad(1),0,5); x[115:175]=0
        x[175:180]=np.linspace(0,np.deg2rad(6),5); x[180:]=np.deg2rad(6)
        z=np.zeros(n); c=dict(time=t,alpha=x,kappa=z,fz=z+500,speed=z,omega=z,ia=z,pressure=z+82000,fx=z,fy=z)
        run=RunData(7,'synthetic',c,{},np.arange(n))
        events=isolated_step_events(run)
        self.assertEqual(len(events),2)
        self.assertLess(np.rad2deg(run.channels['alpha'][events[0].stop-1]),2.)

    def test_preserves_original_sampled_command_knots(self):
        run=make_run()
        event=ramp_events(run)[0]
        np.testing.assert_array_equal(run.channels['alpha'][event.start:event.stop],
                                      event.run.channels['alpha'][event.start:event.stop])


if __name__ == '__main__':
    unittest.main()
