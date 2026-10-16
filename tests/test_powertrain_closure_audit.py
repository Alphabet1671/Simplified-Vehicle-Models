"""Audit the monotonicity argument behind the powertrain voltage closure.

``PlanarForwardPlant.powertrain`` solves ``V - terminal(V) = 0`` with a bracketed
root finder. A unique root follows when the residual is strictly increasing:

1. The speed-regulated torque envelope never decreases with bus voltage.
2. With wheel speeds >= 0 (a plant guard) and a voltage-independent regeneration
   request, the bus power demand is therefore nondecreasing in voltage, and it is
   then capped at the battery power limit, which keeps it nondecreasing.
3. Battery current is increasing in demanded power.
4. Terminal voltage ``V_oc - R I`` is therefore nonincreasing in ``V``.
5. So ``V - terminal(V)`` is strictly increasing and has at most one root.

This test checks steps 1 and 3 on the real parameter set over a dense grid and
steps 2, 4 and 5 on the very residual function the solver hands to ``brentq``.
It is an exhaustive grid audit, not a proof for every real input.
"""
import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.powertrain.energy_storage import StorageState, solve_current
from simplified_vehicle_models.powertrain.motor_envelope import speed_regulated_torque_limit
from simplified_vehicle_models.solvers import planar_forward
from simplified_vehicle_models.solvers.forward_driver import DriverCommand, SpeedPlan
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant


class PowertrainClosureAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(
            "models/formulations/iem24_murphy_real_planar_7dof.xml"
        ).model
        cls.plan = SpeedPlan(np.array([0., 100.]), np.array([8., 8.]), np.zeros(2))

    def test_torque_envelope_never_decreases_with_voltage(self):
        motor = self.vehicle.motor
        voltages = np.linspace(0.5 * motor.nominal_bus_voltage, motor.maximum_bus_voltage, 400)
        speeds = np.linspace(0., 1.3 * motor.maximum_motor_speed, 400)
        table = np.array([speed_regulated_torque_limit(motor, speeds, voltage)
                          for voltage in voltages])
        self.assertGreaterEqual(float(np.min(np.diff(table, axis=0))), -1.e-9)

    def test_current_increases_with_power(self):
        storage = self.vehicle.storage
        for charge in np.linspace(0.05, 1., 20):
            state = StorageState(float(charge))
            powers = np.linspace(-5.e3, 60.e3, 400)
            currents = np.array([solve_current(storage, state, power)[0] for power in powers])
            self.assertGreaterEqual(float(np.min(np.diff(currents))), -1.e-9)

    def test_residual_the_solver_brackets_is_increasing(self):
        captured = []
        real_brentq = planar_forward.brentq

        def spy(function, lower, upper, *args, **kwargs):
            captured.append((function, lower, upper))
            return real_brentq(function, lower, upper, *args, **kwargs)

        plant = PlanarForwardPlant(self.vehicle, self.plan)
        base = plant.initial_state(8.)
        motor = self.vehicle.motor
        top_wheel_speed = 1.2 * motor.maximum_motor_speed / motor.gear_ratio
        worst = np.inf
        brackets = 0
        with patch.object(planar_forward, "brentq", spy):
            for charge in (0.1, 0.5, 0.95):
                for wheel_speed in (0.5, 30., 0.8 * top_wheel_speed, 0.99 * top_wheel_speed,
                                    top_wheel_speed):
                    for throttle, brake in ((1., 0.), (0.4, 0.), (0., 0.), (0., 200.)):
                        state = base.copy()
                        state[10] = charge
                        state[6:10] = wheel_speed
                        captured.clear()
                        plant.powertrain(state, DriverCommand(throttle, brake, 0.))
                        for function, lower, upper in captured:
                            brackets += 1
                            grid = np.linspace(lower, upper, 200)
                            values = np.array([function(v) for v in grid])
                            worst = min(worst, float(np.min(np.diff(values))))
        self.assertGreater(brackets, 0, "no operating point reached the bracketed solve")
        # Allowed slack: the 1e-12 relative power-cap factor in the solver.
        self.assertGreaterEqual(worst, -1.e-6)


if __name__ == "__main__":
    unittest.main()
