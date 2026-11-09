"""First-order steering inverse contracts; defaults preserve legacy preview."""
import unittest
from dataclasses import replace
import numpy as np
from simplified_vehicle_models.solvers.forward_driver import PreviewDriver, SpeedPlan, DriverObservation

class SteeringActuatorInverseTests(unittest.TestCase):
    def driver(self, curvature):
        plan=SpeedPlan(np.array([0.,1.,2.]),np.full(3,5.),np.asarray(curvature),is_closed=False)
        return PreviewDriver(plan,1.5,.6,10.,1000.)

    def test_inverse_matches_analytic_first_order_target(self):
        d=replace(self.driver([0.,.1,.2]),steering_feedforward='actuator-inverse')
        o=DriverObservation(1.,0.,0.,5.,.5,0.)
        grad=(np.arctan(1.5*.11)-np.arctan(1.5*.09))/.2
        expected=(np.arctan(.15)+.08*5*grad)/.08
        self.assertAlmostEqual(d._steering_command(o),expected,places=10)

    def test_constant_curvature_same_for_both_modes(self):
        d=self.driver([.1,.1,.1]);o=DriverObservation(1.,.02,.01,5.,.5,.12,.01)
        self.assertEqual(d._steering_command(o),replace(d,steering_feedforward='actuator-inverse')._steering_command(o))

    def test_physical_slew_and_angle_bounds_are_not_bypassed(self):
        d=replace(self.driver([-1.,0.,1.]),steering_feedforward='actuator-inverse',maximum_steering_rate=.2)
        o=DriverObservation(1.,0.,0.,5.,0.,.6)
        self.assertLessEqual(abs(d._steering_command(o)),.2)
        self.assertLessEqual(d._steering_command(o),0.)

    def test_open_endpoint_uses_one_sided_not_halved_derivative(self):
        d=replace(self.driver([0.,.1,.2]),steering_feedforward='actuator-inverse')
        o=DriverObservation(0.,0.,0.,5.,0.,0.)
        expected=5.*np.arctan(1.5*.01)/.1
        self.assertAlmostEqual(d._steering_command(o),expected,places=10)

    def test_closed_seam_inverse_is_periodic(self):
        plan=SpeedPlan(np.array([0.,1.,3.,4.]),np.full(4,5.),np.array([0.,.1,-.1,0.]))
        d=PreviewDriver(plan,1.5,.6,10.,1000.,steering_feedforward='actuator-inverse')
        left=DriverObservation(0.,0.,0.,5.,0.,0.)
        right=replace(left,distance=4.)
        self.assertAlmostEqual(d._steering_command(left),d._steering_command(right),places=10)

    def test_bad_mode_rejected(self):
        with self.assertRaisesRegex(ValueError,'steering_feedforward'):
            replace(self.driver([0.,0.,0.]),steering_feedforward='unknown')
