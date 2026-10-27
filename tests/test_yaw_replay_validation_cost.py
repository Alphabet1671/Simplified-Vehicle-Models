"""Validation-coverage and accounting regressions for bounded replay."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from simplified_vehicle_models.solvers.forward_driver import (
    DriverObservation, ForwardIntegrationError, PreviewDriver, RateSystem,
    SpeedPlan, TrialStateError, solve_forward,
)


class Plant:
    state_size = 1
    def __init__(self, reject=None): self.reject = reject
    def observe(self, state): return DriverObservation(float(state[0]), 0., 0., 5., 0., 0.)
    def evaluate(self, time, state, command):
        if self.reject and self.reject(time, state): raise TrialStateError("injected invalid replay state")
        return RateSystem(np.array([1.]))


class ReplayValidationCostTests(unittest.TestCase):
    def setUp(self):
        self.driver = PreviewDriver(SpeedPlan(np.array([0., 10.]), np.array([5., 5.]), np.zeros(2)), 1., 1., 1., 100.)

    def fake_result(self, states=None, dense=None):
        states = np.array([[0., 1., 2.]]) if states is None else np.asarray(states, dtype=float)
        times = np.array([0., 1., 2.])
        dense = (lambda t: np.array([t])) if dense is None else dense
        return SimpleNamespace(success=True, message="ok", t=times, y=states,
                               t_events=[np.array([])], sol=dense)

    def test_each_accepted_state_and_each_between_step_midpoint_is_validated(self):
        result = self.fake_result()
        counts = {}
        sink = []
        with patch("scipy.integrate.solve_ivp", return_value=result):
            out = solve_forward(Plant(), self.driver, np.array([0.]), duration=2., wall_seconds=10.,
                                method="Radau", step_sink=sink.append, diagnostics_sink=counts.update)
        self.assertEqual(len(sink), len(result.t))
        self.assertEqual(out.dense_samples, len(result.t)-1)
        self.assertEqual(counts["counts"]["accepted_states_validated"], len(result.t))
        self.assertEqual(counts["counts"]["dense_probes_validated"], len(result.t)-1)
        self.assertEqual(counts["counts"]["defect_checks"], len(result.t)-1)
        self.assertEqual(counts["counts"]["records_written"], len(result.t))
        for key in ("integration_seconds", "accepted_state_validation_seconds", "dense_state_domain_validation_seconds",
                    "defect_checking_seconds", "record_writing_seconds", "summary_construction_seconds"):
            self.assertGreaterEqual(counts["timings"][key], 0.)

    def test_invalid_accepted_state_fails_revalidation(self):
        result = self.fake_result()
        plant = Plant(lambda time, state: time == 1.)
        with patch("scipy.integrate.solve_ivp", return_value=result):
            with self.assertRaises(TrialStateError):
                solve_forward(plant, self.driver, np.array([0.]), duration=2., wall_seconds=10., method="Radau")

    def test_invalid_dense_midpoint_fails_revalidation(self):
        result = self.fake_result(dense=lambda time: np.array([time + (1. if time == .5 else 0.)]))
        plant = Plant(lambda time, state: time == .5)
        with patch("scipy.integrate.solve_ivp", return_value=result):
            with self.assertRaises(TrialStateError):
                solve_forward(plant, self.driver, np.array([0.]), duration=2., wall_seconds=10., method="Radau")

    def test_timeout_during_post_integration_validation_retains_unvalidated_snapshot(self):
        result = self.fake_result()
        # Integration observes the first timestamps; validation crosses the budget before accepted state 0.
        ticks = iter([0., 0., 0., 0., .2, .2, .2, .2])
        with patch("scipy.integrate.solve_ivp", return_value=result), patch(
                "simplified_vehicle_models.solvers.forward_driver.perf_counter", side_effect=lambda: next(ticks, .2)):
            with self.assertRaises(ForwardIntegrationError) as caught:
                solve_forward(Plant(), self.driver, np.array([0.]), duration=2., wall_seconds=.1, method="Radau")
        self.assertEqual(caught.exception.diagnostics["failure_kind"], "validation_timeout")
        self.assertEqual(caught.exception.diagnostics["last_state_kind"], "solver_accepted_unvalidated")
        self.assertFalse(caught.exception.last_state.flags.writeable)


if __name__ == "__main__": unittest.main()
