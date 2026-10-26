import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from simplified_vehicle_models.solvers.forward_driver import (
    DriverObservation, ForwardIntegrationError, PreviewDriver, RateSystem,
    SpeedPlan, solve_forward,
)


class Plant:
    state_size=1
    def observe(self,state): return DriverObservation(float(state[0]),0.,0.,5.,0.,0.)
    def evaluate(self,time,state,command): return RateSystem(np.array([1.]))


class ForwardFailureDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        plan=SpeedPlan(np.array([0.,10.]),np.array([5.,5.]),np.zeros(2))
        self.driver=PreviewDriver(plan,1.,1.,1.,100.)

    def test_unsuccessful_solver_has_typed_readonly_accepted_snapshot(self):
        fake=SimpleNamespace(success=False,message="step size failure",t=np.array([0.,.3]),
                             y=np.array([[2.,3.]]))
        with patch("scipy.integrate.solve_ivp",return_value=fake):
            with self.assertRaises(ForwardIntegrationError) as caught:
                solve_forward(Plant(),self.driver,np.array([2.]),duration=1.,wall_seconds=1.)
        error=caught.exception
        self.assertIsInstance(error,RuntimeError)
        self.assertEqual(error.last_time,.3)
        np.testing.assert_array_equal(error.last_state,[3.])
        self.assertFalse(error.last_state.flags.writeable)
        self.assertEqual(error.diagnostics["failure_kind"],"solver_unsuccessful")
        self.assertEqual(error.diagnostics["accepted_step_count"],1)

    def test_empty_solver_history_uses_initial_snapshot(self):
        fake=SimpleNamespace(success=False,message="failed",t=np.array([]),y=np.empty((1,0)))
        with patch("scipy.integrate.solve_ivp",return_value=fake):
            with self.assertRaises(ForwardIntegrationError) as caught:
                solve_forward(Plant(),self.driver,np.array([2.]),duration=1.,wall_seconds=1.)
        np.testing.assert_array_equal(caught.exception.last_state,[2.])
        self.assertEqual(caught.exception.diagnostics["accepted_step_count"],0)
        self.assertEqual(caught.exception.diagnostics["last_state_kind"],"initial")

    def test_domain_rejection_message_and_time_are_retained(self):
        def fake(rhs, *_args, **_kwargs):
            rhs(.25,np.array([3.]))
            return SimpleNamespace(success=False,message="failed",t=np.array([0.]),y=np.array([[2.]]))
        class RejectingPlant(Plant):
            def evaluate(self,time,state,command):
                from simplified_vehicle_models.solvers.forward_driver import TrialStateError
                if time > 0.: raise TrialStateError("wheel speed below domain")
                return RateSystem(np.array([1.]))
        with patch("scipy.integrate.solve_ivp",side_effect=fake):
            with self.assertRaises(ForwardIntegrationError) as caught:
                solve_forward(RejectingPlant(),self.driver,np.array([2.]),duration=1.,wall_seconds=1.)
        rejection=caught.exception.diagnostics["last_domain_rejection"]
        self.assertEqual(rejection,{"time":.25,"message":"wheel speed below domain"})
        self.assertEqual(caught.exception.diagnostics["last_state_kind"],"solver_accepted_unvalidated")

    def test_diagnostic_counts_and_limits_are_reported(self):
        fake=SimpleNamespace(success=False,message="failed",t=np.array([0.]),y=np.array([[2.]]))
        with patch("scipy.integrate.solve_ivp",return_value=fake):
            with self.assertRaises(ForwardIntegrationError) as caught:
                solve_forward(Plant(),self.driver,np.array([2.]),duration=7.,wall_seconds=2.,method="BDF")
        diag=caught.exception.diagnostics
        self.assertEqual((diag["duration"],diag["wall_seconds"],diag["method"]),(7.,2.,"BDF"))
        self.assertIn("rhs_evaluations",diag); self.assertIn("rejected_domain_trials",diag)


if __name__ == "__main__": unittest.main()
