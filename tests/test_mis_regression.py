from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.lap_solver import LapMode, LapResult, solve_lap
from simplified_vehicle_models.point_mass import acceleration_limits, maximum_vehicle_speed
from simplified_vehicle_models.tracks import TrackProfile, load_track
from simplified_vehicle_models.xml_loader import load_point_mass_vehicle


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml"
TRACK_PATH = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_v11.xml"


class MisRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = load_point_mass_vehicle(FORMULATION_PATH)
        cls.track = load_track(TRACK_PATH)
        cls.flying = solve_lap(cls.vehicle, cls.track, LapMode.FLYING, tolerance=1.0e-5)
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

    def test_speed_and_force_limits_are_satisfied(self) -> None:
        speed_limit = maximum_vehicle_speed(self.vehicle)
        for result in (self.flying, self.standing):
            self.assertLessEqual(float(np.max(result.speed)), speed_limit + 1.0e-8)
            self._assert_transition_limits(result, self.track)

    def _assert_transition_limits(
        self,
        result: LapResult,
        track: TrackProfile,
    ) -> None:
        for index, required_acceleration in enumerate(
            result.longitudinal_acceleration
        ):
            evaluation_speed = 0.5 * (result.speed[index] + result.speed[index + 1])
            segment_curvature = max(
                abs(float(track.curvature[index])),
                abs(float(track.curvature[index + 1])),
            )
            limits = acceleration_limits(
                self.vehicle,
                float(evaluation_speed),
                segment_curvature,
            )
            self.assertTrue(limits.is_feasible)
            if required_acceleration >= 0.0:
                self.assertLessEqual(
                    float(required_acceleration),
                    limits.maximum_acceleration + 1.0e-8,
                )
            else:
                self.assertLessEqual(
                    float(-required_acceleration),
                    limits.maximum_deceleration + 1.0e-8,
                )


if __name__ == "__main__":
    unittest.main()
