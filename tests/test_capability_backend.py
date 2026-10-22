import unittest
from unittest.mock import ANY, patch
import numpy as np

from simplified_vehicle_models.controls.planning import ControlResponse
from simplified_vehicle_models.formulations.capability_backend import (
    CapabilityGrid, IdealCapabilityBackend,
)
from simplified_vehicle_models.formulations.controller_capability import (
    AllocationCapabilityBackend,
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

    def test_out_of_domain_axes_raise_a_clear_value_error(self):
        raw = CapabilitySurface(
            np.array([0.5, 1.75, 3.0]), np.array([-1.0, 0.0, 1.0]),
            np.ones((3, 3)), -np.ones((3, 3)), np.full(3, 5.0), 'vehicle-hash')
        cases = (
            ('speed', (0.4, 2.0, 3.0), (-1.0, 0.0, 1.0)),
            ('curvature', (0.5, 2.0, 3.0), (-2.0, -1.0, 0.0)),
        )
        for name, speed, curvature in cases:
            with self.subTest(axis=name):
                grid = CapabilityGrid(np.array(speed), np.array(curvature))
                with patch('simplified_vehicle_models.formulations.capability_backend.'
                           'build_capability_surface', return_value=raw):
                    with self.assertRaisesRegex(
                            ValueError,
                            f'{name} grid .* outside the builder domain'):
                        IdealCapabilityBackend(object()).sample(
                            grid, curvature_limit=1.0)


def _ideal_surface(grid):
    shape = (grid.speed.size, grid.curvature.size)
    return CapabilitySurface(
        grid.speed, grid.curvature, np.full(shape, 5.0), np.full(shape, -8.0),
        np.full(grid.curvature.size, 20.0), 'vehicle-hash')


def _plane_surface():
    speed = np.array([0.5, 5.5, 10.5])
    curvature = np.array([-0.2, 0.0, 0.2])
    amax = 4.0 + 0.1 * speed[:, None] + 0.2 * curvature[None, :]
    return CapabilitySurface(speed, curvature, amax, -amax,
                             np.full(curvature.size, 20.0), 'vehicle-hash')


def _plane_expected(grid):
    return 4.0 + 0.1 * grid.speed[:, None] + 0.2 * grid.curvature[None, :]


class _CapPolicy:
    name = 'cap-at-two'
    provenance_hash = 'cap-at-two-v1'
    is_stateful = False

    def evaluate(self, demand):
        feasible = demand.longitudinal_acceleration <= 2.0
        return ControlResponse(feasible, min(demand.longitudinal_acceleration, 2.0), 0.0)


class _AlwaysFeasible:
    name = 'always-feasible'
    provenance_hash = 'always-feasible-v1'
    is_stateful = False

    def evaluate(self, demand):
        return ControlResponse(True, demand.longitudinal_acceleration, 0.0)


class _BrakeOnlySaturation:
    name = 'brake-only-saturation'
    provenance_hash = 'brake-only-saturation-v1'
    is_stateful = False

    def evaluate(self, demand):
        tags = ('braking',) if demand.longitudinal_acceleration < 0.0 else ()
        return ControlResponse(True, demand.longitudinal_acceleration, 0.0, tags)


class AllocationBackendTest(unittest.TestCase):
    def test_known_cap_appears_in_the_sampled_surface(self):
        vehicle = object()
        backend = AllocationCapabilityBackend(vehicle, _CapPolicy())
        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]), np.array([-0.1, 0.0, 0.1]))
        with patch.object(AllocationCapabilityBackend, '_ideal',
                          return_value=_ideal_surface(grid)):
            sampled = backend.sample(grid, curvature_limit=0.1)
        self.assertEqual(sampled.mode, 'allocation')
        self.assertNotEqual(sampled.controller_hash, '')
        self.assertTrue(np.all(sampled.surface.acceleration_max <= 2.0 + 1e-9))
        self.assertTrue(np.any(np.isclose(sampled.surface.acceleration_max, 2.0,
                                          atol=1e-2)))
        self.assertTrue(np.all(sampled.feasible_mask))
        self.assertEqual(sampled.metadata['curvature_limit'], 0.1)
        self.assertEqual(sampled.metadata['scan_points'], 17)
        self.assertEqual(sampled.metadata['policy_name'], 'cap-at-two')

    def test_stateful_policy_is_rejected_in_allocation_mode(self):
        class StatefulPolicy(_CapPolicy):
            is_stateful = True
        with self.assertRaisesRegex(ValueError, 'stateful'):
            AllocationCapabilityBackend(object(), StatefulPolicy())

    def test_non_monotone_policy_is_flagged(self):
        class BumpPolicy:
            name = 'bump'
            provenance_hash = 'bump-v1'
            is_stateful = False
            def evaluate(self, demand):
                ok = demand.longitudinal_acceleration <= 1.0 or demand.longitudinal_acceleration >= 3.0
                return ControlResponse(ok, demand.longitudinal_acceleration, 0.0)
        backend = AllocationCapabilityBackend(object(), BumpPolicy())
        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]), np.array([-0.1, 0.0, 0.1]))
        sampled = backend.sample(grid, curvature_limit=0.1,
                                 ideal_surface=_ideal_surface(grid))
        self.assertTrue(sampled.metadata['non_monotone'])

    def test_infeasible_cells_are_masked_and_zeroed(self):
        class NeverPolicy:
            name = 'never'
            provenance_hash = 'never-v1'
            is_stateful = False

            def evaluate(self, demand):
                return ControlResponse(False, 0.0, 0.0)

        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]), np.array([-0.1, 0.0, 0.1]))
        sampled = AllocationCapabilityBackend(object(), NeverPolicy()).sample(
            grid, curvature_limit=0.1, ideal_surface=_ideal_surface(grid))
        self.assertFalse(np.any(sampled.feasible_mask))
        self.assertEqual(sampled.metadata['infeasible_cells'], 9)
        np.testing.assert_array_equal(sampled.surface.acceleration_max, 0.0)
        np.testing.assert_array_equal(sampled.surface.acceleration_min, 0.0)
        np.testing.assert_array_equal(sampled.surface.speed_ceiling, 0.0)

    def test_saturation_tags_are_recorded(self):
        class SaturatedPolicy(_CapPolicy):
            name = 'saturated-at-two'
            provenance_hash = 'saturated-at-two-v1'

            def evaluate(self, demand):
                feasible = demand.longitudinal_acceleration <= 2.0
                return ControlResponse(feasible, min(demand.longitudinal_acceleration, 2.0),
                                       0.0, ('acceleration',))

        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]), np.array([-0.1, 0.0, 0.1]))
        sampled = AllocationCapabilityBackend(object(), SaturatedPolicy()).sample(
            grid, curvature_limit=0.1, ideal_surface=_ideal_surface(grid))
        self.assertEqual(sampled.metadata['saturation_tags'], ['acceleration'])
        self.assertFalse(sampled.metadata['non_monotone'])

    def test_default_ideal_is_regridded_onto_the_grid(self):
        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]),
                              np.array([-0.1, 0.0, 0.1]))
        with patch('simplified_vehicle_models.formulations.controller_capability.'
                   'build_capability_surface', return_value=_plane_surface()) as build:
            sampled = AllocationCapabilityBackend(
                object(), _AlwaysFeasible()).sample(grid, curvature_limit=0.2)
        build.assert_called_once_with(
            ANY, curvature_limit=0.2, speed_points=3, curvature_points=3)
        expected = _plane_expected(grid)
        np.testing.assert_array_equal(sampled.surface.speed, grid.speed)
        np.testing.assert_array_equal(sampled.surface.curvature, grid.curvature)
        np.testing.assert_allclose(sampled.surface.acceleration_max, expected)
        np.testing.assert_allclose(sampled.surface.acceleration_min, -expected)
        self.assertTrue(np.all(sampled.feasible_mask))
        self.assertTrue(np.all(sampled.surface.acceleration_max
                               <= expected + 1e-9))
        self.assertTrue(np.all(sampled.surface.acceleration_min
                               >= -expected - 1e-9))

    def test_explicit_ideal_surface_is_regridded_onto_the_grid(self):
        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]),
                              np.array([-0.1, 0.0, 0.1]))
        with patch('simplified_vehicle_models.formulations.controller_capability.'
                   'build_capability_surface') as build:
            sampled = AllocationCapabilityBackend(
                object(), _AlwaysFeasible()).sample(
                    grid, curvature_limit=0.2, ideal_surface=_plane_surface())
        build.assert_not_called()
        expected = _plane_expected(grid)
        np.testing.assert_array_equal(sampled.surface.speed, grid.speed)
        np.testing.assert_array_equal(sampled.surface.curvature, grid.curvature)
        np.testing.assert_allclose(sampled.surface.acceleration_max, expected)
        np.testing.assert_allclose(sampled.surface.acceleration_min, -expected)
        self.assertTrue(np.all(sampled.surface.acceleration_max
                               <= expected + 1e-9))

    def test_braking_side_saturation_is_recorded(self):
        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]),
                              np.array([-0.1, 0.0, 0.1]))
        sampled = AllocationCapabilityBackend(
            object(), _BrakeOnlySaturation()).sample(
                grid, curvature_limit=0.1, ideal_surface=_ideal_surface(grid))
        self.assertEqual(sampled.metadata['saturation_tags'], ['braking'])

    def test_non_monotone_uses_widest_feasible_run(self):
        class TwoIslandPolicy:
            name = 'two-island'
            provenance_hash = 'two-island-v1'
            is_stateful = False

            def evaluate(self, demand):
                ax = demand.longitudinal_acceleration
                feasible = ax <= -6.0 or (-4.0 <= ax <= -3.0)
                return ControlResponse(feasible, ax, 0.0)

        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]),
                              np.array([-0.1, 0.0, 0.1]))
        sampled = AllocationCapabilityBackend(
            object(), TwoIslandPolicy()).sample(
                grid, curvature_limit=0.1, ideal_surface=_ideal_surface(grid))
        self.assertTrue(sampled.metadata['non_monotone'])
        scan = np.linspace(-8.0, 5.0, 17)
        self.assertTrue(np.all(sampled.surface.acceleration_max
                               > sampled.surface.acceleration_min))
        np.testing.assert_allclose(sampled.surface.acceleration_min, scan[0])
        np.testing.assert_allclose(sampled.surface.acceleration_max, scan[2])

    def test_invalid_sampling_parameters_are_rejected(self):
        cases = (
            {'scan_points': 2},
            {'bisection_steps': 0},
            {'scan_points': 4.5},
        )
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                with self.assertRaisesRegex(ValueError, next(iter(kwargs))):
                    AllocationCapabilityBackend(object(), _CapPolicy(), **kwargs)

    def test_mode_must_stay_allocation(self):
        with self.assertRaisesRegex(ValueError, 'mode'):
            AllocationCapabilityBackend(object(), _CapPolicy(), mode='closed_loop')
        self.assertEqual(
            AllocationCapabilityBackend(object(), _CapPolicy()).mode, 'allocation')
