import unittest
import numpy as np

from simplified_vehicle_models.solvers.maneuver_simulation import (
    SimulationSettings, simulate_maneuver,
)


class _Solved:
    def __init__(self, rates):
        self.rates = rates

    def solve(self):
        return self.rates


class _Plant:
    state_size = 1

    def __init__(self):
        self.evaluate_calls = 0

    def evaluate(self, time, state, command):
        self.evaluate_calls += 1
        return _Solved(np.array([1.0]))

    def observe(self, state):
        return float(state[0])


class _Program:
    def __init__(self):
        self.ticks = []

    def tick(self, observation, dt):
        self.ticks.append((observation, dt))
        return (len(self.ticks),)

    def breakpoints(self):
        return (0.0, 0.035, 0.07, 0.1)


class ManeuverSimulationTests(unittest.TestCase):
    def run_sim(self, plant):
        program = _Program()
        result = simulate_maneuver(
            plant, program, [0.0],
            SimulationSettings(duration_s=0.1, controller_tick_s=0.01, max_step_s=0.01),
        )
        return result, program

    def test_ticks_at_zero_and_each_tick_strictly_before_duration(self):
        result, program = self.run_sim(_Plant())
        self.assertEqual(result.status, "completed")
        self.assertEqual(len(program.ticks), 10)
        np.testing.assert_allclose([item[1] for item in program.ticks], [0.01] * 10)
        self.assertAlmostEqual(program.ticks[0][0], 0.0)
        self.assertEqual(result.requested, tuple((i,) for i in range(1, 11)))
        self.assertEqual(len(result.time_s), len(result.state))

    def test_history_independent_of_evaluate_call_counter(self):
        a, _ = self.run_sim(_Plant())
        other = _Plant()
        self.run_sim(other)
        other.evaluate_calls += 999
        b, _ = self.run_sim(_Plant())
        np.testing.assert_array_equal(a.time_s, b.time_s)
        np.testing.assert_array_equal(a.state, b.state)
        self.assertFalse(a.time_s.flags.writeable)
        self.assertFalse(a.state.flags.writeable)


if __name__ == "__main__":
    unittest.main()
