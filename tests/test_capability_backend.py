import unittest
from unittest.mock import ANY, patch
import numpy as np

from simplified_vehicle_models.formulations.capability_backend import (
    CapabilityGrid, IdealCapabilityBackend,
)
from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface


class IdealBackendTest(unittest.TestCase):
    def test_backend_wraps_existing_builder_and_hash(self):
        vehicle = object()
        surface = CapabilitySurface(
            np.array([1.0, 2.0, 3.0]), np.array([-1.0, 0.0, 1.0]),
            np.ones((3, 3)), -np.ones((3, 3)), np.full(3, 5.0), 'vehicle-hash')
        with patch('simplified_vehicle_models.formulations.capability_backend.'
                   'build_capability_surface', return_value=surface) as build:
            sampled = IdealCapabilityBackend(vehicle).sample(
                CapabilityGrid(surface.speed, surface.curvature), curvature_limit=1.0)
        build.assert_called_once()
        self.assertEqual(sampled.mode, 'ideal')
        self.assertEqual(sampled.vehicle_hash, 'vehicle-hash')
        self.assertEqual(sampled.controller_hash, '')
        np.testing.assert_array_equal(sampled.surface.speed, surface.speed)
        np.testing.assert_array_equal(sampled.surface.curvature, surface.curvature)

    def test_grid_requires_increasing_finite_axes(self):
        with self.assertRaises(ValueError):
            CapabilityGrid(np.array([1.0, 1.0]), np.array([-1.0, 1.0]))

    def test_surface_is_regridded_onto_the_requested_axes(self):
        speed = np.array([1.0, 2.0, 3.0])
        curvature = np.array([-1.0, 0.0, 1.0])
        acceleration_max = 2.0 * speed[:, None] + 3.0 * curvature[None, :]
        raw = CapabilitySurface(
            speed, curvature, acceleration_max, -acceleration_max,
            np.array([10.0, 20.0, 30.0]), 'vehicle-hash')
        grid = CapabilityGrid(np.array([1.5, 2.5, 3.0]),
                              np.array([-0.5, 0.5, 1.0]))
        with patch('simplified_vehicle_models.formulations.capability_backend.'
                   'build_capability_surface', return_value=raw) as build:
            sampled = IdealCapabilityBackend(object()).sample(
                grid, curvature_limit=1.0)
        build.assert_called_once_with(
            ANY, curvature_limit=1.0, speed_points=3,
            curvature_points=3)
        np.testing.assert_array_equal(sampled.surface.speed, grid.speed)
        np.testing.assert_array_equal(sampled.surface.curvature, grid.curvature)
        expected_max = 2.0 * grid.speed[:, None] + 3.0 * grid.curvature[None, :]
        np.testing.assert_allclose(sampled.surface.acceleration_max, expected_max)
        np.testing.assert_allclose(sampled.surface.acceleration_min, -expected_max)
        np.testing.assert_allclose(sampled.surface.speed_ceiling,
                                   np.interp(grid.curvature, curvature,
                                             raw.speed_ceiling))
        self.assertEqual(sampled.surface.provenance_hash, 'vehicle-hash')
        self.assertTrue(np.all(sampled.feasible_mask))
        self.assertEqual(sampled.feasible_mask.shape, expected_max.shape)
        self.assertEqual(sampled.version, '1')
        self.assertEqual(sampled.metadata, {})
