"""Oracle tests for static vertical equilibrium."""

from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
from simplified_vehicle_models.tires.vertical import tire_deflection_load
from simplified_vehicle_models.formulations.planar_7dof import wheel_positions


ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"


class VerticalEquilibriumOracleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(str(FORMULATION)).model

    def test_static_equilibrium_load_oracle(self):
        vehicle = self.vehicle
        state = solve_static_equilibrium(vehicle)
        self.assertAlmostEqual(state.jounce_fl, 0.0, delta=1e-6)
        self.assertAlmostEqual(state.jounce_fr, 0.0, delta=1e-6)
        self.assertAlmostEqual(state.jounce_rl, 0.0, delta=1e-6)
        self.assertAlmostEqual(state.jounce_rr, 0.0, delta=1e-6)
        q = state.as_vector()
        positions = wheel_positions(vehicle.planar)
        jounce = q[13:17]
        tire_deflection = q[6] + positions[:, 1] * q[7] - positions[:, 0] * q[8] + jounce
        loads = tire_deflection_load(vehicle.planar.tire, tire_deflection, np.zeros(4), 0.0).load
        self.assertAlmostEqual(float(np.sum(loads)), vehicle.planar.total_mass * 9.80665, delta=1e-6)
        self.assertAlmostEqual(float(np.sum(loads[:2])), 0.45 * vehicle.planar.total_mass * 9.80665, delta=1e-6)

    def test_speed_equilibrium_includes_static_aero_load(self):
        vehicle = self.vehicle
        speed = 12.0
        state = solve_static_equilibrium(vehicle, speed=speed)
        q = state.as_vector()
        positions = wheel_positions(vehicle.planar)
        deflection = q[6] + positions[:, 1] * q[7] - positions[:, 0] * q[8] + q[13:17]
        loads = tire_deflection_load(vehicle.planar.tire, deflection, np.zeros(4), 0.0).load
        downforce = 0.5 * vehicle.planar.aero.air_density * speed**2 * vehicle.planar.aero.downforce_area
        self.assertAlmostEqual(float(np.sum(loads)), vehicle.planar.total_mass * 9.80665 + downforce, delta=1e-6)
        self.assertAlmostEqual(
            float(np.sum(loads[:2])),
            0.45 * vehicle.planar.total_mass * 9.80665 + vehicle.planar.aero.aero_balance * downforce,
            delta=1e-6,
        )


if __name__ == "__main__":
    unittest.main()
