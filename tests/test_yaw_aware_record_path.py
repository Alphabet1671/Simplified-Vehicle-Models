"""Tests for exporting yaw-aware optimized lines to run-record geometry."""
import unittest

import numpy as np

from simplified_vehicle_models.planning.yaw_aware_line import optimize_yaw_aware_line
from simplified_vehicle_models.planning.yaw_aware_record_path import path_coordinates_from_yaw_line
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.planning.yaw_aware_line import YawLineSettings


def circle_track(radius=24., count=25):
    distance = np.linspace(0., 2*np.pi*radius, count)
    theta = distance/radius
    north, east = radius*np.cos(theta), radius*np.sin(theta)
    heading = np.unwrap(np.arctan2(np.gradient(east, distance), np.gradient(north, distance)))
    curvature = np.full_like(distance, 1./radius)
    coordinates = PathCoordinates(distance, north, east, heading, curvature, True)
    widths = np.full_like(distance, 4.)
    return TrackProfile(distance, np.diff(distance), np.full_like(distance, radius), curvature,
                        True, "synthetic circle", coordinates, widths, widths, "assumed")


def capability():
    speed, ax, ay = np.linspace(0., 24., 7), np.array([-8., 0., 8.]), np.linspace(-18., 18., 5)
    shape = (len(speed), len(ax), len(ay))
    moment = np.full(shape, 1.e6)
    return YawCapabilityMap(speed, ax, ay, moment, moment, np.ones(shape, bool), 1.,
                            "physical_tv_v1", "synthetic-record-path")


def settings(boundary="flying"):
    return YawLineSettings(control_spacing=160., grid_spacing=100., dense_subdivisions=1,
                           optimizer_max_iterations=1, maximum_speed=20., maximum_outer_passes=1,
                           boundary=boundary, corner_min_length=30.)


class YawAwareRecordPathTests(unittest.TestCase):
    def _circle_plan(self):
        return optimize_yaw_aware_line(circle_track(), None, capability(), settings=settings())

    def test_closed_plan_preserves_physical_stations_and_exact_geometry(self):
        plan = self._circle_plan()
        path = path_coordinates_from_yaw_line(plan)
        np.testing.assert_allclose(path.distance, plan.speed_plan.distance)
        np.testing.assert_allclose(path.curvature, plan.speed_plan.curvature)
        source = plan.trajectory.station_map.to_reference(plan.speed_plan.distance)
        evaluated = plan.trajectory.evaluate(source)
        np.testing.assert_allclose(path.x_north, evaluated.north)
        np.testing.assert_allclose(path.y_east, evaluated.east)
        np.testing.assert_allclose(path.heading, np.unwrap(evaluated.heading))
        np.testing.assert_allclose(path.reference_distance, source)
        self.assertTrue(path.is_closed)
        np.testing.assert_allclose([path.x_north[0], path.y_east[0]],
                                   [path.x_north[-1], path.y_east[-1]], atol=1e-8)

    def test_open_plan_keeps_endpoint_geometry_nonperiodic(self):
        distance = np.linspace(0., 100., 21)
        north, east = distance, 5. * np.exp(-((distance - 50.) / 18.)**2)
        heading = np.unwrap(np.arctan2(np.gradient(east, distance), np.gradient(north, distance)))
        curvature = np.gradient(heading, distance)
        coords = PathCoordinates(distance, north, east, heading, curvature, False)
        widths = np.full_like(distance, 5.)
        track = TrackProfile(distance, np.diff(distance), 1. / np.maximum(np.abs(curvature), 1e-4),
                             curvature, False, "open corner", coords, widths, widths, "assumed")
        plan = optimize_yaw_aware_line(track, None, capability(), settings=settings("standing"))
        path = path_coordinates_from_yaw_line(plan)
        self.assertFalse(path.is_closed)
        self.assertEqual(path.distance[0], 0.)
        self.assertAlmostEqual(path.distance[-1], plan.speed_plan.distance[-1])
        self.assertGreater(np.hypot(path.x_north[-1] - path.x_north[0],
                                    path.y_east[-1] - path.y_east[0]), 1.)
        self.assertNotAlmostEqual(path.heading[-1] - path.heading[0], 2*np.pi, places=3)

    def test_optimized_line_curvature_and_rate_are_its_own_periodic_seam_values(self):
        path=self._circle_plan().speed_plan
        self.assertAlmostEqual(path.curvature[0],path.curvature[-1],places=10)
        from simplified_vehicle_models.solvers.yaw_aware_profile import curvature_rate_from_samples
        rate=curvature_rate_from_samples(path.distance,path.curvature,is_closed=True)
        self.assertAlmostEqual(rate[0],rate[-1],places=8)

    def test_output_arrays_are_immutable(self):
        path = path_coordinates_from_yaw_line(self._circle_plan())
        for values in (path.distance, path.x_north, path.y_east, path.heading,
                       path.curvature, path.reference_distance):
            self.assertFalse(values.flags.writeable)


if __name__ == "__main__":
    unittest.main()
