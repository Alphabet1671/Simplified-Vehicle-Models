from pathlib import Path
import math
import unittest

import numpy as np

from simplified_vehicle_models.core.conventions import GRAVITY
from simplified_vehicle_models.formulations.point_mass import maximum_vehicle_speed
from simplified_vehicle_models.results.traces import LapMode, LapResult
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml"
TRACK_PATH = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_v11.xml"

# The golden values come from docs/validation/mis-point-mass.md.
# They pin the validated MIS lap result.
# A physics change must fail these tests. A human then decides if the change is correct.
# The relative tolerance is 1e-9. A tighter bound is brittle across platforms.
# A looser bound lets a real physics change through.
GOLDEN_FLYING_LAP_TIME = 69.22653728547469
GOLDEN_STANDING_LAP_TIME = 70.38955050759907
GOLDEN_PEAK_SPEED = 29.582305037708668
GOLDEN_NODE_COUNT = 1407
GOLDEN_RELATIVE_TOLERANCE = 1.0e-9


class MisRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = load_point_mass_vehicle(FORMULATION_PATH)
        cls.track = load_track(TRACK_PATH)
        cls.flying = solve_lap(
            cls.vehicle,
            cls.track,
            LapMode.FLYING,
            tolerance=1.0e-5,
        )
        cls.standing = solve_lap(
            cls.vehicle,
            cls.track,
            LapMode.STANDING_START,
            tolerance=1.0e-5,
        )

    def test_lap_times_are_reasonable(self) -> None:
        for result in (self.flying, self.standing):
            self.assertTrue(result.converged)
            self.assertTrue(np.isfinite(result.lap_time))
            self.assertGreater(result.lap_time, 50.0)
            self.assertLess(result.lap_time, 100.0)
        self.assertGreater(self.standing.lap_time, self.flying.lap_time)
        self.assertLess(self.standing.lap_time - self.flying.lap_time, 15.0)

    def test_boundary_conditions_are_satisfied(self) -> None:
        self.assertAlmostEqual(self.flying.speed[0], self.flying.speed[-1])
        self.assertEqual(self.standing.speed[0], 0.0)

    def test_golden_flying_lap_time(self) -> None:
        self._assert_golden(self.flying.lap_time, GOLDEN_FLYING_LAP_TIME)

    def test_golden_standing_lap_time(self) -> None:
        self._assert_golden(self.standing.lap_time, GOLDEN_STANDING_LAP_TIME)

    def test_golden_peak_speed(self) -> None:
        self._assert_golden(
            float(np.max(self.flying.speed)),
            GOLDEN_PEAK_SPEED,
        )
        self._assert_golden(
            float(np.max(self.standing.speed)),
            GOLDEN_PEAK_SPEED,
        )

    def test_golden_node_count(self) -> None:
        self.assertEqual(len(self.track.distance), GOLDEN_NODE_COUNT)
        self.assertEqual(len(self.flying.speed), GOLDEN_NODE_COUNT)
        self.assertEqual(len(self.standing.speed), GOLDEN_NODE_COUNT)

    def _assert_golden(self, actual: float, expected: float) -> None:
        self.assertTrue(
            math.isclose(
                actual,
                expected,
                rel_tol=GOLDEN_RELATIVE_TOLERANCE,
                abs_tol=0.0,
            ),
            f"expected {expected!r} within {GOLDEN_RELATIVE_TOLERANCE}, "
            f"received {actual!r}",
        )

    def test_independent_equilibrium_closure(self) -> None:
        speed_limit = maximum_vehicle_speed(self.vehicle)
        for result in (self.flying, self.standing):
            self.assertLessEqual(float(np.max(result.speed)), speed_limit + 1.0e-8)
            self._assert_equilibrium(result)

    def _assert_equilibrium(self, result: LapResult) -> None:
        vehicle = self.vehicle
        midpoint_speed = 0.5 * (result.speed[:-1] + result.speed[1:])
        dynamic_pressure = 0.5 * vehicle.aero.air_density * midpoint_speed**2
        drag = dynamic_pressure * vehicle.aero.drag_area
        downforce = dynamic_pressure * vehicle.aero.downforce_area
        force_residual = (
            np.sum(result.longitudinal_forces, axis=1)
            - drag
            - vehicle.total_mass * result.longitudinal_acceleration
        )
        np.testing.assert_allclose(force_residual, 0.0, atol=1.0e-8)
        np.testing.assert_allclose(
            np.sum(result.normal_loads, axis=1),
            vehicle.total_mass * GRAVITY + downforce,
            atol=1.0e-8,
        )
        self.assertTrue(np.all(result.normal_loads >= 0.0))

        distance_to_front = vehicle.wheelbase * (
            1.0 - vehicle.front_mass_fraction
        )
        distance_to_rear = vehicle.wheelbase * vehicle.front_mass_fraction
        lateral_moment = (
            np.sum(result.lateral_forces[:, :2], axis=1) * distance_to_front
            - np.sum(result.lateral_forces[:, 2:], axis=1) * distance_to_rear
        )
        left_force = (
            result.longitudinal_forces[:, 0]
            + result.longitudinal_forces[:, 2]
        )
        right_force = (
            result.longitudinal_forces[:, 1]
            + result.longitudinal_forces[:, 3]
        )
        longitudinal_moment = (
            right_force - left_force
        ) * vehicle.track_width * 0.5
        yaw_moment = lateral_moment + longitudinal_moment
        np.testing.assert_allclose(yaw_moment, 0.0, atol=1.0e-8)

        tire = vehicle.tire
        load_factor = (
            tire.load_sensitivity_slope * result.normal_loads
            + tire.load_sensitivity_intercept
        )
        longitudinal_capacity = (
            tire.longitudinal_mu_scale * load_factor * result.normal_loads
        )
        lateral_capacity = (
            tire.lateral_mu_scale * load_factor * result.normal_loads
        )
        utilization = np.sqrt(
            (result.longitudinal_forces / longitudinal_capacity) ** 2
            + (result.lateral_forces / lateral_capacity) ** 2
        )
        self.assertLessEqual(float(np.max(utilization)), 1.0 + 1.0e-7)

        commanded_force = (
            result.throttle * result.drive_force_limit
            - result.brake * result.brake_force_limit
        )
        np.testing.assert_allclose(
            commanded_force,
            np.sum(result.longitudinal_forces, axis=1),
            atol=1.0e-8,
        )
        self.assertFalse(
            np.any((result.throttle > 0.0) & (result.brake > 0.0))
        )
        mild_deceleration = result.longitudinal_acceleration < 0.0
        self.assertTrue(np.any(mild_deceleration & (result.throttle > 0.0)))


if __name__ == "__main__":
    unittest.main()
