from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.core.conventions import GRAVITY
from simplified_vehicle_models.formulations.point_mass import (
    acceleration_limits,
    aerodynamic_forces,
    evaluate_equilibrium,
    maximum_corner_speed,
    maximum_vehicle_speed,
    normal_loads,
    tire_force_capacities,
)
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml"


class PointMassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = load_point_mass_vehicle(FORMULATION_PATH)

    def test_static_loads_sum_to_vehicle_weight(self) -> None:
        loads = normal_loads(self.vehicle, 0.0, 0.0, 0.0)

        self.assertAlmostEqual(float(np.sum(loads)), self.vehicle.total_mass * GRAVITY)
        self.assertAlmostEqual(float(loads[0]), float(loads[1]))
        self.assertAlmostEqual(float(loads[2]), float(loads[3]))

    def test_aero_loads_scale_with_speed_squared(self) -> None:
        downforce_10, drag_10 = aerodynamic_forces(self.vehicle, 10.0)
        downforce_20, drag_20 = aerodynamic_forces(self.vehicle, 20.0)

        self.assertAlmostEqual(downforce_20, 4.0 * downforce_10)
        self.assertAlmostEqual(drag_20, 4.0 * drag_10)

    def test_longitudinal_load_transfer_has_correct_direction(self) -> None:
        static = normal_loads(self.vehicle, 0.0, 0.0, 0.0)
        accelerating = normal_loads(self.vehicle, 0.0, 5.0, 0.0)
        braking = normal_loads(self.vehicle, 0.0, -5.0, 0.0)

        self.assertLess(float(np.sum(accelerating[:2])), float(np.sum(static[:2])))
        self.assertGreater(float(np.sum(braking[:2])), float(np.sum(static[:2])))

    def test_lateral_transfer_conserves_total_load(self) -> None:
        straight = normal_loads(self.vehicle, 12.0, 0.0, 0.0)
        turning = normal_loads(self.vehicle, 12.0, 0.0, 6.0)

        self.assertAlmostEqual(float(np.sum(straight)), float(np.sum(turning)))
        self.assertLess(float(turning[0]), float(turning[1]))
        self.assertLess(float(turning[2]), float(turning[3]))

    def test_load_sensitivity_reduces_effective_friction(self) -> None:
        loads = np.asarray([300.0, 600.0, 900.0, 1200.0])
        longitudinal, lateral = tire_force_capacities(self.vehicle, loads)

        self.assertTrue(np.all(np.diff(longitudinal / loads) < 0.0))
        self.assertTrue(np.all(np.diff(lateral / loads) < 0.0))

    def test_measured_friction_is_physically_plausible(self) -> None:
        loads = np.asarray([700.0, 850.0, 1000.0, 1200.0])
        longitudinal, lateral = tire_force_capacities(self.vehicle, loads)
        lateral_friction = lateral / loads
        longitudinal_friction = longitudinal / loads

        self.assertTrue(np.all(lateral_friction >= 1.0))
        self.assertTrue(np.all(lateral_friction <= 1.4))
        self.assertTrue(np.all(longitudinal_friction >= 0.5))
        self.assertTrue(np.all(longitudinal_friction <= 0.8))

    def test_unloaded_tire_has_zero_capacity(self) -> None:
        longitudinal, lateral = tire_force_capacities(
            self.vehicle,
            np.asarray([0.0, 0.0, 0.0, 0.0]),
        )

        np.testing.assert_array_equal(longitudinal, 0.0)
        np.testing.assert_array_equal(lateral, 0.0)

    def test_live_limits_are_finite(self) -> None:
        launch = acceleration_limits(self.vehicle, 0.0, 0.0)
        moving = acceleration_limits(self.vehicle, 20.0, 0.0)

        self.assertGreater(launch.maximum_acceleration, 0.0)
        self.assertGreater(launch.maximum_deceleration, 0.0)
        self.assertGreater(moving.maximum_acceleration, 0.0)
        self.assertGreater(moving.maximum_deceleration, 0.0)
        self.assertTrue(np.isfinite(moving.maximum_acceleration))

    def test_drive_force_does_not_exceed_power_limit(self) -> None:
        speed = 30.0
        limits = acceleration_limits(self.vehicle, speed, 0.0)
        power_force = (
            self.vehicle.drivetrain.power_limit
            * self.vehicle.drivetrain.efficiency
            / speed
        )

        self.assertLessEqual(
            limits.acceleration_state.drive_force_limit,
            power_force,
        )

    def test_lateral_forces_close_yaw_moment(self) -> None:
        state = evaluate_equilibrium(self.vehicle, 10.0, 1.0 / 20.0, -2.0)

        self.assertTrue(state.is_feasible)
        self.assertAlmostEqual(state.yaw_moment_residual, 0.0, places=10)

    def test_wheel_lift_is_infeasible_without_added_load(self) -> None:
        state = evaluate_equilibrium(self.vehicle, 25.0, 1.0 / 3.0, 0.0)
        expected_load = (
            self.vehicle.total_mass * GRAVITY
            + aerodynamic_forces(self.vehicle, 25.0)[0]
        )

        self.assertFalse(state.contact_feasible)
        self.assertAlmostEqual(float(np.sum(state.normal_loads)), expected_load)

    def test_limit_diagnostics_match_solved_accelerations(self) -> None:
        limits = acceleration_limits(self.vehicle, 15.0, 1.0 / 30.0)

        self.assertAlmostEqual(
            limits.acceleration_state.longitudinal_acceleration,
            limits.maximum_acceleration,
        )
        self.assertAlmostEqual(
            limits.braking_state.longitudinal_acceleration,
            -limits.maximum_deceleration,
        )

    def test_tighter_corner_has_lower_speed(self) -> None:
        tight = maximum_corner_speed(self.vehicle, 8.0)
        open_corner = maximum_corner_speed(self.vehicle, 40.0)

        self.assertGreater(tight, 0.0)
        self.assertLess(tight, open_corner)
        self.assertLessEqual(open_corner, maximum_vehicle_speed(self.vehicle))


if __name__ == "__main__":
    unittest.main()
