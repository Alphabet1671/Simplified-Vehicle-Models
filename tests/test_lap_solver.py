from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.tracks.types import TrackProfile
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


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
