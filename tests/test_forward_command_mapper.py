"""Focused contract tests for state-aware forward command mapping."""
import unittest

import numpy as np

from simplified_vehicle_models.solvers.forward_driver import (
    DriverCommand, DriverObservation, PreviewDriver, RateSystem,
    build_speed_plan, solve_forward,
)


class LinearPlant:
    state_size = 2

    def __init__(self):
        self.observations = []
        self.commands = []

    def observe(self, state):
        observation = DriverObservation(float(state[0]), 0., 0., float(state[1]), 0., 0.)
        self.observations.append(observation)
        return observation

    def evaluate(self, time, state, command):
        self.commands.append(command)
        return RateSystem(np.array([state[1], command.throttle - command.brake_force / 1000.]))


class ForwardCommandMapperTests(unittest.TestCase):
    def setUp(self):
        plan = build_speed_plan(np.array([0., 100.]), np.zeros(2), 10., 3., 2., 2.)
        self.driver = PreviewDriver(plan, 1.5, .5, 2., 1000.)
        self.initial = np.array([0., 10.])

    def run_forward(self, plant, **kwargs):
        return solve_forward(plant, self.driver, self.initial, duration=.02,
                             maximum_step=.005, method="BDF", **kwargs)

    def test_omitted_and_none_mapper_are_bitwise_equal(self):
        left, right = LinearPlant(), LinearPlant()
        a = self.run_forward(left)
        b = self.run_forward(right, command_mapper=None)
        np.testing.assert_array_equal(a.time, b.time)
        np.testing.assert_array_equal(a.state, b.state)
        self.assertEqual(a.evaluations, b.evaluations)
        self.assertEqual(left.commands, right.commands)

    def test_callback_gets_readonly_copy_and_same_observation_command(self):
        plant = LinearPlant()
        received = []
        def mapper(time, state, observation, requested):
            received.append((time, state.copy(), observation, requested))
            self.assertFalse(state.flags.writeable)
            self.assertEqual(state.shape, (plant.state_size,))
            return DriverCommand(.25, 0., requested.steering_rate)
        result = self.run_forward(plant, command_mapper=mapper)
        self.assertGreater(len(received), 0)
        self.assertEqual(received[0][0], 0.)
        self.assertIs(received[0][2], plant.observations[0])
        self.assertEqual(received[0][3], self.driver.command(received[0][2]))
        self.assertTrue(np.all(np.isfinite(result.state)))

    def test_noncallable_mapper_rejected(self):
        with self.assertRaises(ValueError):
            self.run_forward(LinearPlant(), command_mapper=3)

    def test_invalid_mapped_commands_fail_without_fallback(self):
        invalid = [DriverCommand(float("nan"), 0., 0.),
                   DriverCommand(.5, 1., 0.),
                   DriverCommand(-.1, 0., 0.),
                   DriverCommand(0., 1001., 0.),
                   DriverCommand(0., 0., 2.1),
                   (0., 0., 0.)]
        for command in invalid:
            with self.subTest(command=command):
                calls = []
                def mapper(*args):
                    calls.append(args)
                    return command
                with self.assertRaises(ValueError):
                    self.run_forward(LinearPlant(), command_mapper=mapper)
                self.assertTrue(calls)

    def test_initial_pedal_target_uses_mapped_command(self):
        plant = LinearPlant()
        driver = PreviewDriver(self.driver.plan, 1.5, .5, 2., 1000.,
                               pedal_travel_time=.2)
        calls = []
        def mapper(time, state, observation, requested):
            calls.append((time, state.copy()))
            return DriverCommand(.4, 0., requested.steering_rate)
        result = solve_forward(plant, driver, self.initial, duration=.01,
                               maximum_step=.005, command_mapper=mapper)
        self.assertEqual(calls[0][0], 0.)
        self.assertAlmostEqual(result.pedal[0], .4)

    def test_callback_state_mutation_is_blocked_and_input_unchanged(self):
        plant = LinearPlant()
        initial = self.initial.copy()
        def mapper(time, state, observation, requested):
            with self.assertRaises(ValueError):
                state[0] = 99.
            return requested
        self.run_forward(plant, command_mapper=mapper)
        np.testing.assert_array_equal(initial, self.initial)

    def test_callback_error_propagates(self):
        error = RuntimeError("mapper failed")
        def mapper(*args):
            raise error
        with self.assertRaisesRegex(RuntimeError, "mapper failed") as caught:
            self.run_forward(LinearPlant(), command_mapper=mapper)
        self.assertIs(caught.exception, error)


if __name__ == "__main__":
    unittest.main()
