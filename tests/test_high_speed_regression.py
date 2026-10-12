from pathlib import Path
import unittest

from simplified_vehicle_models.formulations.point_mass import (
    evaluate_equilibrium,
    maximum_vehicle_speed,
)
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


ROOT = Path(__file__).resolve().parents[1]


class HighSpeedRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = load_point_mass_vehicle(ROOT / "models/formulations/murphy_point_mass.xml")

    def test_motor_speed_is_active_above_analytic_limit(self):
        limit = maximum_vehicle_speed(self.vehicle)
        state = evaluate_equilibrium(self.vehicle, limit * 1.01, 0.0, 0.0)
        self.assertEqual(state.active_longitudinal_constraint, "motor_speed")
        self.assertEqual(state.drive_force_limit, 0.0)
        self.assertFalse(state.longitudinal_feasible)

    def test_below_limit_is_not_motor_speed_limited(self):
        limit = maximum_vehicle_speed(self.vehicle)
        state = evaluate_equilibrium(self.vehicle, limit * 0.98, 0.0, 0.0)
        self.assertNotEqual(state.active_longitudinal_constraint, "motor_speed")
        self.assertGreater(state.drive_force_limit, 0.0)


if __name__ == "__main__":
    unittest.main()
