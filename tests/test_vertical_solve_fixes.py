"""Regression tests for the 14DoF solve-chain fixes of 2026-11-04.

Each test pins one defect found in the chain audit:

* the wheel-spin rolling-resistance sign;
* path-coordinate and nonfinite trial states must be retryable rejections;
* the brake governor reads kinematic wheel slip, not the lagged tire state;
* the anti-geometry drive/brake switch and the roll-centre |roll| are smooth.
"""
import dataclasses
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations import vertical_14dof as v14
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState, contact_velocities, slip_quantities, wheel_positions, wheel_steering_angles,
)
from simplified_vehicle_models.solvers.forward_driver import (
    DriverCommand, SpeedPlan, TrialStateError,
)
from simplified_vehicle_models.solvers import vertical_forward as vf
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.tires.unitire_transient import steady_rolling_state

FORMULATION = "models/formulations/iem24_murphy_real_14dof_simple.xml"
_VEHICLE = []


def vehicle():
    if not _VEHICLE:
        _VEHICLE.append(compile_formulation(FORMULATION).model)
    return _VEHICLE[0]


def plant(speed=11.0, curvature=-0.1226):
    plan = SpeedPlan(np.array([0.0, 200.0]), np.full(2, speed), np.full(2, curvature))
    return vf.VerticalForwardPlant(vehicle(), plan)


def wheel_rates(model, state, torques=np.zeros(4)):
    """Return the four wheel-spin accelerations of the evaluator at a state."""
    p = plant()
    tires = p.tire_states(state)
    evaluation = v14.evaluate_vertical_dynamics(
        model, p.mechanical_state(state), torques, float(state[vf.STEERING_INDEX]), tires)
    return np.asarray(evaluation.rates)[9:13], evaluation


class RollingResistanceSignTests(unittest.TestCase):
    def test_rolling_resistance_retards_a_forward_rolling_wheel(self):
        """With rolling resistance on, a forward wheel decelerates more, never less.

        The tire returns a negative moment for a forward roll, so the wheel
        equation must add it. The former code subtracted it and spun the wheel up.
        """
        base = vehicle()
        tire = dataclasses.replace(base.planar.tire, f_rr_0=0.015)
        resisting = dataclasses.replace(base, planar=dataclasses.replace(base.planar, tire=tire))
        state = plant().initial_state(11.0)
        free, evaluation = wheel_rates(base, state)
        loaded, loaded_evaluation = wheel_rates(resisting, state)
        moments = np.array([f for f in loaded_evaluation.tire_forces[:, 2]])
        # The rolling moment itself is not in tire_forces; check the effect.
        self.assertTrue(np.all(loaded < free), f"free {free}, with resistance {loaded}")
        expected = -0.015 * loaded_evaluation.normal_loads * loaded_radius(
            tire, loaded_evaluation.normal_loads) / base.planar.wheel_inertia
        np.testing.assert_allclose(loaded - free, expected, rtol=0.05)


class RetryableTrialStateTests(unittest.TestCase):
    def setUp(self):
        self.plant = plant()
        self.state = self.plant.initial_state(11.0)

    def test_path_denominator_violation_is_a_trial_rejection(self):
        state = self.state.copy()
        state[1] = 0.6 / -0.1226  # 1 - kappa*n = 0.4 < 0.5
        with self.assertRaises(TrialStateError):
            self.plant.evaluate(0.0, state, DriverCommand(0.0, 0.0, 0.0))

    def test_negative_progress_is_a_trial_rejection(self):
        state = self.state.copy()
        state[2] = np.pi  # heading reversed: progress is negative
        with self.assertRaises(TrialStateError):
            self.plant.evaluate(0.0, state, DriverCommand(0.0, 0.0, 0.0))

    def test_steering_beyond_the_bound_is_a_trial_rejection(self):
        state = self.state.copy()
        state[vf.STEERING_INDEX] = vehicle().planar.steering_maximum_angle + 1e-3
        with self.assertRaises(TrialStateError):
            self.plant.evaluate(0.0, state, DriverCommand(0.0, 0.0, 0.0))

    def test_trial_rejection_is_the_type_the_integrator_retries(self):
        self.assertTrue(issubclass(TrialStateError, ValueError))


class GovernorObservationTests(unittest.TestCase):
    def test_margin_is_the_kinematic_slip_of_the_most_locked_wheel(self):
        p = plant()
        state = p.initial_state(11.0)
        state[list(vf.WHEEL_SPEED_INDEX)] *= np.array([0.97, 0.99, 0.95, 1.0])
        p.evaluate(0.0, state, DriverCommand(0.0, 0.0, 0.0))
        loads = p.last_evaluation.normal_loads
        m = p.mechanical_state(state)
        angles = (wheel_steering_angles(float(state[vf.STEERING_INDEX]), 4)
                  + vehicle().suspension.kinematics.toe(state[list(vf.JOUNCE_INDEX)]))
        longitudinal, lateral = contact_velocities(
            PlanarState(m.v_x, m.v_y, m.yaw_rate, state[list(vf.WHEEL_SPEED_INDEX)]),
            wheel_positions(vehicle().planar), angles)
        slip, _ = slip_quantities(state[list(vf.WHEEL_SPEED_INDEX)],
                                  loaded_radius(vehicle().planar.tire, loads),
                                  longitudinal, lateral, 1.0)
        self.assertAlmostEqual(p.observe(state).wheel_speed_margin, float(np.min(slip)) + 1.0,
                               places=9)

    def test_margin_does_not_wait_for_the_tire_slip_state(self):
        """A locking wheel lowers the margin at once, before the tire state follows."""
        p = plant()
        state = p.initial_state(11.0)
        before = p.observe(state).wheel_speed_margin
        state[vf.WHEEL_SPEED_INDEX[2]] *= 0.5  # rear-left wheel half locked
        self.assertLess(p.observe(state).wheel_speed_margin, before - 0.4)


class SmoothnessTests(unittest.TestCase):
    def _jounce_forces(self, slip):
        """Return the four jounce generalized forces at one rear slip ratio.

        Rear wheel speed sets the rear tire slip, so the rear contact force F_x
        passes through zero as ``slip`` does. Tire states are held at zero, so
        the transient force is a smooth function of the commanded slip.
        """
        p = plant(curvature=0.0)
        state = p.initial_state(11.0)
        mechanical = p.mechanical_state(state).as_vector()
        radius = float(loaded_radius(vehicle().planar.tire, np.array([700.0]))[0])
        mechanical[11:13] = mechanical[0] * (1.0 + slip) / radius
        tires = tuple(steady_rolling_state(vehicle().planar.tire, vehicle().transient_tire,
                                               0.0, float(slip if i >= 2 else 0.0), 700.0)
                      for i in range(4))
        evaluation = v14.evaluate_vertical_dynamics(
            vehicle(), v14.VerticalState.from_vector(mechanical), np.zeros(4), 0.0, tires)
        return np.asarray(evaluation.generalized_forces)[3:], evaluation

    def test_anti_force_has_no_slope_step_where_rear_fx_crosses_zero(self):
        """The anti geometry blends drive and brake, so the force is smooth at F_x = 0.

        Rear anti-squat (0.55) and anti-lift (0.574) give different gains, so a
        hard switch puts a slope step in the rear jounce forces at F_x = 0. The
        one-sided slopes just left and right of the zero crossing must agree.
        """
        lo, hi = -2e-3, 2e-3
        for _ in range(60):  # find the rear slip where the summed rear F_x is zero
            mid = 0.5 * (lo + hi)
            fx = self._jounce_forces(mid)[1].tire_forces[2:, 0].sum()
            lo, hi = (mid, hi) if fx < 0.0 else (lo, mid)
        zero = 0.5 * (lo + hi)
        h = 2e-5
        left = (self._jounce_forces(zero)[0] - self._jounce_forces(zero - h)[0]) / h
        right = (self._jounce_forces(zero + h)[0] - self._jounce_forces(zero)[0]) / h
        np.testing.assert_allclose(right[2:], left[2:], rtol=5e-2, atol=1.0)

    def test_roll_centre_height_is_smooth_through_zero_roll(self):
        k = vehicle().suspension.kinematics
        smoothing = v14.ROLL_CENTRE_SMOOTHING_ROLL
        query = lambda roll: float(np.hypot(roll, smoothing) - smoothing)
        heights = [k.roll_centre_height(0, query(r)) for r in (-1e-5, 0.0, 1e-5)]
        left = (heights[1] - heights[0]) / 1e-5
        right = (heights[2] - heights[1]) / 1e-5
        self.assertLess(abs(right - (-left)), 1e-3)  # even function with zero slope at 0
        self.assertLess(abs(right), 0.02)
        # Away from zero the smoothing changes the query by under 0.1 mm of height.
        self.assertLess(abs(k.roll_centre_height(0, query(np.radians(1.0)))
                            - k.roll_centre_height(0, np.radians(1.0))), 1e-4)


if __name__ == "__main__":
    unittest.main()
