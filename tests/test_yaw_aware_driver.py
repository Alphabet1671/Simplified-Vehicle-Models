"""No-TV closed-loop and geometric containment checks for Phase 4a."""
import unittest

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.planning.yaw_aware_line import YawLineSettings, optimize_yaw_aware_line
from simplified_vehicle_models.solvers.forward_driver import (
    DriverObservation, PreviewDriver, RateSystem,
)
from simplified_vehicle_models.solvers.yaw_aware_driver import (
    drive_yaw_aware_plan, trajectory_corridor_containment,
)
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def make_plan():
    radius, count = 24., 13
    distance = np.linspace(0., 2*np.pi*radius, count)
    theta = distance / radius
    north, east = radius*np.cos(theta), radius*np.sin(theta)
    heading = np.unwrap(np.arctan2(np.gradient(east, distance), np.gradient(north, distance)))
    curvature = np.full_like(distance, 1./radius)
    coordinates = PathCoordinates(distance, north, east, heading, curvature, True)
    widths = np.full_like(distance, 5.)
    track = TrackProfile(distance, np.diff(distance), np.full_like(distance, radius), curvature,
                         True, "synthetic circle", coordinates, widths, widths, "assumed")
    speed, ax, ay = np.linspace(0., 24., 13), np.array([-8., 0., 8.]), np.linspace(-18., 18., 7)
    shape = (len(speed), len(ax), len(ay))
    capability = YawCapabilityMap(speed, ax, ay, np.full(shape, 1.e6), np.full(shape, 1.e6),
                                  np.ones(shape, bool), 1., "physical_no_tv_v1", "fixture")
    settings = YawLineSettings(control_spacing=90., grid_spacing=28., dense_subdivisions=1,
                               optimizer_max_iterations=1, maximum_outer_passes=1, maximum_speed=20.)
    return optimize_yaw_aware_line(track, None, capability, settings=settings)


class FakeForwardPlant:
    """Kinematic lap plant, with exact centerline tracking."""
    state_size = 2

    def __init__(self, length, speed):
        self.length, self.speed = length, speed

    def observe(self, state):
        return DriverObservation(float(state[0]), float(state[1]), 0., self.speed,
                                 self.speed / 24., 0.)

    def evaluate(self, time, state, command):
        return RateSystem(np.array([self.speed, 0.]))


class YawAwareDriverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = make_plan()

    def test_geometry_only_containment_and_outside_offset(self):
        trajectory = self.plan.trajectory
        path_s = np.linspace(0., trajectory.station_map.path_end, 31)
        contained = trajectory_corridor_containment(self.plan, path_s, np.zeros_like(path_s))
        self.assertTrue(contained[2])
        outside = trajectory_corridor_containment(self.plan, path_s, np.full_like(path_s, 100.))
        self.assertFalse(outside[2])
        self.assertGreater(outside[0], 90.)

    def test_closed_path_forward_tracking(self):
        plan = self.plan
        speed = 10.
        driver = PreviewDriver(plan.speed_plan, 2.5, .5, 2., 1000., speed_gain=.01)
        plant = FakeForwardPlant(plan.speed_plan.distance[-1], speed)
        result = drive_yaw_aware_plan(plan, plant, driver, np.array([0., 0.]),
                                      duration=20., wall_seconds=10., maximum_step=.2)
        self.assertTrue(result.forward_result.lap_completed)
        self.assertTrue(result.completed)
        self.assertTrue(result.sampled_center_contained)
        self.assertLessEqual(result.max_corridor_violation, 1.e-12)
        self.assertGreater(result.driven_time, 0.)
        self.assertAlmostEqual(result.predicted_vs_driven_time,
                               result.driven_time - result.predicted_time)

    def test_incomplete_run_does_not_report_lap_time_comparison(self):
        plan = self.plan
        driver = PreviewDriver(plan.speed_plan, 2.5, .5, 2., 1000., speed_gain=.01)
        plant = FakeForwardPlant(plan.speed_plan.distance[-1], 10.)
        result = drive_yaw_aware_plan(plan, plant, driver, np.array([0., 0.]),
                                      duration=.1, wall_seconds=10., maximum_step=.02)
        self.assertFalse(result.completed)
        self.assertIsNone(result.driven_time)
        self.assertIsNone(result.predicted_vs_driven_time)


if __name__ == "__main__":
    unittest.main()
