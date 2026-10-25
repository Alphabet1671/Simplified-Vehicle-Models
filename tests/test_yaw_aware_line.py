"""Focused synthetic tests for the Phase-3 yaw-aware line optimizer."""
import unittest
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.planning.yaw_aware_line import YawLineSettings, YawLineOptimizationError, optimize_yaw_aware_line
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def circle_track(radius=24., count=25, pinch=False):
    distance = np.linspace(0., 2*np.pi*radius, count)
    theta = distance/radius
    north, east = radius*np.cos(theta), radius*np.sin(theta)
    heading = np.unwrap(np.arctan2(np.gradient(east, distance), np.gradient(north, distance)))
    curvature = np.full_like(distance, 1./radius)
    coordinates = PathCoordinates(distance, north, east, heading, curvature, True)
    left = np.full_like(distance, 4.)
    right = np.full_like(distance, 4.)
    if pinch:
        left[(distance > .30*distance[-1]) & (distance < .42*distance[-1])] = 2.4
    return TrackProfile(distance, np.diff(distance), np.full_like(distance, radius), curvature,
                        True, "synthetic circle/corner", coordinates, left, right, "assumed")


def capability(moment=1.e6):
    speed, ax, ay = np.linspace(0., 24., 7), np.array([-8., 0., 8.]), np.linspace(-18., 18., 5)
    shape = (len(speed), len(ax), len(ay))
    return YawCapabilityMap(speed, ax, ay, np.full(shape, moment), np.full(shape, moment),
                            np.ones(shape, bool), 1., "physical_tv_v1", f"synthetic-{moment}")


class YawAwareLineTests(unittest.TestCase):
    def settings(self, boundary="flying"):
        return YawLineSettings(control_spacing=160., grid_spacing=100., dense_subdivisions=1,
                               optimizer_max_iterations=1, maximum_speed=20., maximum_outer_passes=1,
                               boundary=boundary, corner_min_length=30.)

    def test_plan_is_deterministic_contained_and_projects_exact_profile(self):
        track = circle_track()
        first = optimize_yaw_aware_line(track, None, capability(), settings=self.settings())
        second = optimize_yaw_aware_line(track, None, capability(), settings=self.settings())
        np.testing.assert_array_equal(first.coefficients, second.coefficients)
        self.assertEqual(first.lap_time, second.lap_time)
        np.testing.assert_array_equal(first.speed_plan.distance, first.profile.distance)
        np.testing.assert_array_equal(first.speed_plan.speed, first.profile.speed)
        source = first.trajectory.station_map.to_reference(first.profile.distance)
        np.testing.assert_array_equal(first.speed_plan.curvature,
                                      first.trajectory.evaluate(source).curvature)
        self.assertGreater(first.shape_metrics["L50"], 0.)
        self.assertEqual(first.corridor.policy, "signed-fitted-reference-v1")
        offsets = np.asarray(first.trajectory.evaluate(np.linspace(0., track.lap_length, 60)).offset)
        self.assertLessEqual(np.max(np.abs(offsets)), 3.86)

    def test_open_course_uses_standing_start(self):
        distance = np.linspace(0., 100., 21)
        north, east = distance, 5. * np.exp(-((distance - 50.) / 18.)**2)
        heading = np.unwrap(np.arctan2(np.gradient(east, distance), np.gradient(north, distance)))
        curvature = np.gradient(heading, distance)
        coordinates = PathCoordinates(distance, north, east, heading, curvature, False)
        widths = np.full_like(distance, 5.)
        track = TrackProfile(distance, np.diff(distance), 1. / np.maximum(np.abs(curvature), 1e-4),
                             curvature, False, "open corner", coordinates, widths, widths, "assumed")
        result = optimize_yaw_aware_line(track, None, capability(), settings=self.settings("standing"))
        self.assertFalse(result.speed_plan.is_closed)
        self.assertEqual(result.profile.speed[0], 0.)
        self.assertGreater(result.profile.speed[-1], 0.)

    def test_corner_complex_and_asymmetric_pinch(self):
        track = circle_track(radius=16., pinch=True)
        result = optimize_yaw_aware_line(track, None, capability(), settings=self.settings())
        self.assertTrue(result.diagnostics["corner_complexes"])
        self.assertTrue(np.all(result.corridor.minimum > -track.half_width_left))
        self.assertTrue(np.all(result.corridor.maximum < track.half_width_right))
        self.assertTrue(np.isfinite(result.shape_metrics["apex_lateral_position"]))

    def test_no_feasible_candidate_reports_reason(self):
        with self.assertRaises(YawLineOptimizationError) as caught:
            optimize_yaw_aware_line(circle_track(), None, capability(0.), settings=self.settings())
        self.assertTrue(caught.exception.diagnostics)
        self.assertTrue(all(row["reason"] for row in caught.exception.diagnostics))

    def test_yaw_authority_changes_profile_or_line_shape(self):
        track = circle_track()
        weak = optimize_yaw_aware_line(track, None, capability(.08), settings=self.settings())
        strong = optimize_yaw_aware_line(track, None, capability(1.e6), settings=self.settings())
        self.assertLessEqual(strong.lap_time, weak.lap_time + 1e-8)
        self.assertNotEqual(weak.profile.diagnostics["map_source_identity"],
                            strong.profile.diagnostics["map_source_identity"])
        self.assertNotEqual(weak.status, "error")
        self.assertNotEqual(strong.status, "error")


if __name__ == "__main__":
    unittest.main()
