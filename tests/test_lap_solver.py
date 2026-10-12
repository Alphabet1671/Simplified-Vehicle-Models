from pathlib import Path
from unittest.mock import patch
import unittest

import numpy as np

from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.tracks.types import TrackProfile
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle
from simplified_vehicle_models.formulations import point_mass


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml"


def make_track(radii: list[float], segment_length: float) -> TrackProfile:
    distance = np.arange(len(radii), dtype=np.float64) * segment_length
    segments = np.diff(distance)
    radius = np.asarray(radii, dtype=np.float64)
    return TrackProfile(
        distance=distance,
        segment_length=segments,
        radius=radius,
        curvature=1.0 / radius,
        is_closed=True,
        title="Synthetic track",
    )


class LapSolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = load_point_mass_vehicle(FORMULATION_PATH)

    def test_flying_lap_is_periodic(self) -> None:
        track = make_track([20.0] * 12, 8.0)

        result = solve_lap(self.vehicle, track, LapMode.FLYING)

        self.assertTrue(result.converged)
        self.assertAlmostEqual(result.speed[0], result.speed[-1])
        self.assertGreater(result.lap_time, 0.0)

    def test_standing_start_begins_at_zero(self) -> None:
        track = make_track([100000.0] * 8, 20.0)

        result = solve_lap(self.vehicle, track, LapMode.STANDING_START)

        self.assertTrue(result.converged)
        self.assertEqual(result.speed[0], 0.0)
        self.assertGreater(result.speed[1], 0.0)
        self.assertTrue(np.all(np.diff(result.elapsed_time) > 0.0))

    def test_solver_brakes_for_a_tight_corner(self) -> None:
        track = make_track(
            [100000.0, 100000.0, 100000.0, 8.0, 8.0, 100000.0, 100000.0],
            20.0,
        )

        result = solve_lap(self.vehicle, track, LapMode.FLYING)

        self.assertTrue(result.converged)
        self.assertLess(result.speed[3], result.speed[1])
        self.assertTrue(np.any(result.brake > 0.0))
        self.assertTrue(np.any(result.throttle > 0.0))

    def test_throttle_and_brake_do_not_overlap(self) -> None:
        track = make_track([30.0, 15.0, 8.0, 15.0, 30.0], 10.0)

        result = solve_lap(self.vehicle, track, LapMode.FLYING)

        self.assertFalse(np.any((result.throttle > 0.0) & (result.brake > 0.0)))
        self.assertTrue(np.all(np.isfinite(result.speed)))
        self.assertTrue(np.all(result.speed >= 0.0))

    def test_warm_start_matches_cold_lap_without_changing_evaluations(self) -> None:
        track = make_track([30.0, 15.0, 8.0, 15.0, 30.0], 10.0)
        with patch(
            "simplified_vehicle_models.solvers.forward_backward.maximum_acceleration_limit",
            wraps=point_mass.maximum_acceleration_limit,
        ) as cold_accel, patch(
            "simplified_vehicle_models.solvers.forward_backward.maximum_deceleration_limit",
            wraps=point_mass.maximum_deceleration_limit,
        ) as cold_brake:
            cold = solve_lap(self.vehicle, track, LapMode.FLYING)
        cold_evaluations = cold_accel.call_count + cold_brake.call_count

        with patch(
            "simplified_vehicle_models.solvers.forward_backward.maximum_acceleration_limit",
            wraps=point_mass.maximum_acceleration_limit,
        ) as warm_accel, patch(
            "simplified_vehicle_models.solvers.forward_backward.maximum_deceleration_limit",
            wraps=point_mass.maximum_deceleration_limit,
        ) as warm_brake:
            warm = solve_lap(
                self.vehicle,
                track,
                LapMode.FLYING,
                initial_speed=cold.speed,
                corner_speed_hint=cold.speed,
            )
        warm_evaluations = warm_accel.call_count + warm_brake.call_count

        np.testing.assert_allclose(warm.speed, cold.speed, atol=2.0e-8)
        self.assertAlmostEqual(warm.lap_time, cold.lap_time, delta=2.0e-8)
        # Transition/profile acceleration is intentionally unsupported until
        # a non-monotone assembled capability bracket is proven safe.
        self.assertEqual(warm_evaluations, cold_evaluations)

    def test_reports_nonconvergence_for_one_iteration(self) -> None:
        track = make_track(
            [100000.0, 100000.0, 100000.0, 6.0, 6.0, 100000.0, 100000.0],
            40.0,
        )

        result = solve_lap(
            self.vehicle,
            track,
            LapMode.FLYING,
            tolerance=1.0e-12,
            maximum_iterations=1,
        )

        self.assertFalse(result.converged)
        self.assertEqual(result.iterations, 1)


if __name__ == "__main__":
    unittest.main()
