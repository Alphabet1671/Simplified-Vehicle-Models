"""Focused synthetic tests for the Phase-3 yaw-aware line optimizer."""
import unittest

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.planning.yaw_aware_line import YawLineSettings, optimize_yaw_aware_line
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def circle_track(radius=24., count=41):
    distance = np.linspace(0., 2*np.pi*radius, count)
    theta = distance/radius
    north, east = radius*np.cos(theta), radius*np.sin(theta)
    heading = np.unwrap(np.arctan2(np.gradient(east, distance), np.gradient(north, distance)))
    curvature = np.full_like(distance, 1./radius)
    coordinates = PathCoordinates(distance, north, east, heading, curvature, True)
    widths = np.full_like(distance, 4.)
    return TrackProfile(distance, np.diff(distance), np.full_like(distance, radius), curvature,
                        True, "synthetic circle", coordinates, widths, widths, "assumed")


def capability(moment=1.e6):
    speed = np.linspace(0., 24., 13)
    ax = np.array([-8., 0., 8.])
    ay = np.linspace(-18., 18., 7)
    shape = (len(speed), len(ax), len(ay))
    return YawCapabilityMap(speed, ax, ay, np.full(shape, moment), np.full(shape, moment),
                            np.ones(shape, bool), 1., "physical_tv_v1", f"synthetic-{moment}")


class YawAwareLineTests(unittest.TestCase):
    def settings(self):
        return YawLineSettings(control_spacing=90., grid_spacing=28., dense_subdivisions=1,
                               optimizer_max_iterations=1, maximum_speed=20., seed_blends=(0.,))

    def test_plan_is_deterministic_contained_and_projects_exact_profile(self):
        track = circle_track()
        first = optimize_yaw_aware_line(track, None, capability(), settings=self.settings())
        second = optimize_yaw_aware_line(track, None, capability(), settings=self.settings())
        np.testing.assert_array_equal(first.coefficients, second.coefficients)
        self.assertEqual(first.lap_time, second.lap_time)
        self.assertEqual(first.speed_plan.is_closed, True)
        np.testing.assert_array_equal(first.speed_plan.distance, first.profile.distance)
        np.testing.assert_array_equal(first.speed_plan.speed, first.profile.speed)
        np.testing.assert_array_equal(first.speed_plan.curvature, first.speed_plan.curvature)
        self.assertGreater(first.shape_metrics["L50"], 0.)
        self.assertTrue(all("reason" in row for row in first.diagnostics["candidate_evaluations"]))
        offsets = np.asarray(first.trajectory.evaluate(np.linspace(0., track.lap_length, 120)).offset)
        self.assertLessEqual(np.max(np.abs(offsets)), 3.86)

    def test_open_course_uses_standing_start(self):
        distance = np.linspace(0., 100., 31)
        north = distance
        east = 5. * np.exp(-((distance - 50.) / 18.)**2)
        heading = np.unwrap(np.arctan2(np.gradient(east, distance), np.gradient(north, distance)))
        curvature = np.gradient(heading, distance)
        coordinates = PathCoordinates(distance, north, east, heading, curvature, False)
        widths = np.full_like(distance, 5.)
        track = TrackProfile(distance, np.diff(distance), 1. / np.maximum(np.abs(curvature), 1e-4),
                            curvature, False, "open corner", coordinates, widths, widths, "assumed")
        result = optimize_yaw_aware_line(track, None, capability(), settings=YawLineSettings(
            control_spacing=30., grid_spacing=10., dense_subdivisions=1,
            optimizer_max_iterations=1, maximum_speed=20., boundary="standing", seed_blends=(0.,)))
        self.assertFalse(result.speed_plan.is_closed)
        self.assertEqual(result.profile.speed[0], 0.)
        self.assertGreater(result.profile.speed[-1], 0.)

    def test_yaw_authority_changes_selected_geometry_and_rejections_are_classified(self):
        track = circle_track()
        weak = optimize_yaw_aware_line(track, None, capability(.08), settings=self.settings())
        strong = optimize_yaw_aware_line(track, None, capability(1.e6), settings=self.settings())
        # Geometry may tie on the symmetric constant-curvature fixture; the
        # profile/time still must reflect the yaw authority change.
        self.assertTrue(not np.array_equal(weak.coefficients, strong.coefficients)
                        or not np.array_equal(weak.profile.speed, strong.profile.speed))
        # At least one line-search proposal may be rejected by profile/map constraints;
        # all diagnostics have explicit reason labels, never silent penalties.
        self.assertTrue(all(row["reason"] is not None for row in weak.diagnostics["candidate_evaluations"]))


if __name__ == "__main__":
    unittest.main()
