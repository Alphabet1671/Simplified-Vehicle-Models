"""Portable hybrid maneuver metric and production-path gate tests."""
import unittest
import numpy as np

from tools.run.validate_hybrid_driver import (hold_gate_metrics, time_weighted_deceleration,
    _pedal_rate, _physical_pedal_acceleration, _generator_pedal_command)


class HybridManeuverMetricTests(unittest.TestCase):
    def test_fixture_hold_phase_is_distinct_from_measurement_window(self):
        from tools.run.validate_hybrid_driver import HOLD_TIME_S, SETTLE_AFTER_HOLD_S
        self.assertEqual(HOLD_TIME_S, .60)
        self.assertEqual(SETTLE_AFTER_HOLD_S, .35)
        self.assertEqual(HOLD_TIME_S + SETTLE_AFTER_HOLD_S, .95)

    def test_shared_physical_oracle_inverse_and_pedal_rate(self):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
        from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant, SOC_INDEX
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext
        from tools.run.validate_hybrid_driver import FORMULATION
        model=compile_formulation(str(FORMULATION)).model
        old=SpeedPlan(np.array([0.,100.]),np.array([40.,40.]),np.zeros(2),False)
        plant=VerticalForwardPlant(model,old);state=plant.initial_state(20.);soc=float(state[SOC_INDEX])
        predictor=BrakeAuthorityPredictor(model.planar,maximum_abs_curvature_inv_m=.001,minimum_speed_mps=.01,maximum_speed_mps=40.)
        context=plant.brake_authority_context(state,predictor=predictor)
        for pedal in (-.35,-.15,.0,.2,.65):
            accel=_physical_pedal_acceleration(model,predictor,context,20.,pedal,soc)
            if pedal<0.:
                inverse=predictor.request_current(context,-accel)
                recovered=-inverse.pedal_force_n/model.planar.maximum_pedal_force
                self.assertAlmostEqual(recovered,pedal,delta=1e-10)
            else:
                self.assertTrue(np.isfinite(accel))
        command=_generator_pedal_command(-3.,context,predictor,model.planar.maximum_pedal_force)
        self.assertLess(command,0.)
        self.assertAlmostEqual(_physical_pedal_acceleration(model,predictor,context,20.,command,soc),-3.,delta=1e-10)
        self.assertEqual(_generator_pedal_command(0.,context,predictor,model.planar.maximum_pedal_force),0.)
        self.assertGreater(_generator_pedal_command(1.,context,predictor,model.planar.maximum_pedal_force),0.)
        pc=PedalPolicyContext(1/.175,1/.175,.02,model.planar.maximum_pedal_force,.85,1/.175,1/.175)
        self.assertAlmostEqual(_pedal_rate(.2,.0,pc),-1/.175)
        self.assertAlmostEqual(_pedal_rate(-.2,.0,pc),1/.175)
        self.assertAlmostEqual(_pedal_rate(0.,-.3,pc),-1/.175)

    def test_time_weighted_metrics_are_not_row_weighted(self):
        # Piecewise constant -2 m/s² over 0.1s then -4 m/s² over 0.9s.
        t=np.array([0.,.1,1.]);v=np.array([10.,9.8,6.2])
        result=time_weighted_deceleration(t,v,start_s=0.,end_s=1.)
        self.assertAlmostEqual(result['mean_deceleration_mps2'],3.8,places=12)
        gate=hold_gate_metrics(t,v,3.8,start_s=0.,end_s=1.)
        self.assertFalse(gate['pass'])  # Max-error gate catches the short transient.
        self.assertAlmostEqual(gate['maximum_absolute_deceleration_error_mps2'],1.8,places=12)

    def test_empty_invalid_and_noncontiguous_measurement_rejected(self):
        t=np.array([0.,.1,.2]);v=np.array([2.,1.8,1.6])
        with self.assertRaisesRegex(ValueError,'empty'):
            hold_gate_metrics(t,v,2.,start_s=.3,end_s=.4)
        with self.assertRaisesRegex(ValueError,'contiguous'):
            hold_gate_metrics(t,v,2.,start_s=0.,end_s=.2,eligible=np.array([True,False,True]))
        with self.assertRaisesRegex(ValueError,'strict time'):
            time_weighted_deceleration([0.,0.],[2.,1.],start_s=0.,end_s=1.)

    def test_gate_checks_both_mean_and_maximum_error(self):
        t=np.array([0.,.1,.2,.3]);v=np.array([10.,9.7,9.4,9.1])
        pass_row=hold_gate_metrics(t,v,3.,start_s=0.,end_s=.3)
        self.assertTrue(pass_row['pass'])
        outlier=hold_gate_metrics([0.,.1,.2,.3],[10.,9.7,9.0,8.7],3.,start_s=0.,end_s=.3)
        self.assertFalse(outlier['pass'])
        self.assertGreater(outlier['maximum_absolute_deceleration_error_mps2'],outlier['threshold_mps2'])


if __name__=='__main__': unittest.main()
