from pathlib import Path
import tempfile
import unittest
from dataclasses import dataclass, replace

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import (
    acceleration_limits, point_mass_view_of_planar,
)
from simplified_vehicle_models.formulations.point_mass_capability import (
    build_capability_surface, capability_hash, load_capability_surface,
    save_capability_surface,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"


class CapabilitySurfaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = point_mass_view_of_planar(compile_formulation(str(FORMULATION)).model)
        cls.surface = build_capability_surface(cls.vehicle, curvature_limit=0.25,
                                                speed_points=41, curvature_points=21)

    def test_axes_are_sorted_and_symmetric(self):
        self.assertTrue(np.all(np.diff(self.surface.speed) > 0))
        self.assertTrue(np.all(np.diff(self.surface.curvature) > 0))
        self.assertAlmostEqual(float(self.surface.curvature[10]), 0.0)

    def test_limits_reproduce_numpy_physics_at_grid_nodes(self):
        for row in (0, 13, 40):
            for column in (0, 10, 20):
                limits = acceleration_limits(self.vehicle, float(self.surface.speed[row]),
                                             float(self.surface.curvature[column]))
                self.assertAlmostEqual(self.surface.acceleration_max[row,column], limits.maximum_acceleration,
                                       delta=1e-9 * max(1, abs(limits.maximum_acceleration)))
                self.assertAlmostEqual(self.surface.acceleration_min[row,column], -limits.maximum_deceleration,
                                       delta=1e-9 * max(1, abs(limits.maximum_deceleration)))

    def test_speed_ceiling_is_feasible_and_acceleration_boundary(self):
        for column in (0, 13, 20):
            ceiling = float(self.surface.speed_ceiling[column]); curvature = float(self.surface.curvature[column])
            below = acceleration_limits(self.vehicle, ceiling - 0.05, curvature)
            self.assertGreaterEqual(below.maximum_acceleration, -1e-3)
            self.assertTrue(below.coasting_state.contact_feasible)
            self.assertTrue(below.coasting_state.lateral_feasible)

    def test_ceiling_rejects_laterally_infeasible_high_curvature_top_speed(self):
        surface = build_capability_surface(self.vehicle, curvature_limit=10.0,
                                           speed_points=3, curvature_points=5)
        self.assertAlmostEqual(float(surface.speed_ceiling[2]), float(surface.speed[-1]))
        for j in (0, 1, 3, 4):
            self.assertLess(float(surface.speed_ceiling[j]), float(surface.speed[-1]))

    def test_acceleration_zero_root_precedes_lateral_feasibility_cap(self):
        ceiling = float(self.surface.speed_ceiling[13])
        curvature = float(self.surface.curvature[13])
        at = acceleration_limits(self.vehicle, ceiling, curvature)
        self.assertGreater(ceiling, 0.5)
        self.assertLessEqual(at.maximum_acceleration, 1.0e-3)
        self.assertTrue(at.coasting_state.contact_feasible and at.coasting_state.lateral_feasible)

    def test_lateral_feasibility_cap_is_backed_into_feasible_interval(self):
        surface = build_capability_surface(self.vehicle, curvature_limit=10.0,
                                           speed_points=3, curvature_points=5)
        for j in (0, 1, 3, 4):
            ceiling = float(surface.speed_ceiling[j])
            self.assertGreater(ceiling, 0.0)
            state = acceleration_limits(self.vehicle, ceiling, float(surface.curvature[j])).coasting_state
            self.assertTrue(state.contact_feasible and state.lateral_feasible)

    def test_hash_stable_hex_and_nested_physics_sensitive(self):
        first = capability_hash(self.vehicle)
        self.assertEqual(first, capability_hash(self.vehicle)); self.assertEqual(len(first), 64); int(first, 16)
        modified_tire = replace(self.vehicle.tire, effective_radius=self.vehicle.tire.effective_radius * 1.001)
        self.assertNotEqual(first, capability_hash(replace(self.vehicle, tire=modified_tire)))

    def test_hash_distinguishes_mapping_key_types(self):
        @dataclass(frozen=True)
        class Container:
            payload: object
        self.assertNotEqual(capability_hash(Container({1: "same"})),
                            capability_hash(Container({"1": "same"})))

    def test_hash_distinguishes_same_fields_different_types(self):
        @dataclass(frozen=True)
        class PhysicsOne:
            coefficient: float
        @dataclass(frozen=True)
        class PhysicsTwo:
            coefficient: float
        self.assertNotEqual(capability_hash(PhysicsOne(2.0)), capability_hash(PhysicsTwo(2.0)))

    def test_hash_distinguishes_public_attribute_classes(self):
        class PhysicsOne:
            def __init__(self): self.coefficient = 2.0
        class PhysicsTwo:
            def __init__(self): self.coefficient = 2.0
        self.assertNotEqual(capability_hash(PhysicsOne()), capability_hash(PhysicsTwo()))

    def test_round_trip_preserves_every_array(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "surface.npz"; save_capability_surface(self.surface, path)
            restored = load_capability_surface(path)
        for field in ("speed", "curvature", "acceleration_max", "acceleration_min", "speed_ceiling"):
            np.testing.assert_allclose(getattr(restored, field), getattr(self.surface, field))
        self.assertEqual(restored.provenance_hash, self.surface.provenance_hash)

    def test_rejects_bad_curvature_limit(self):
        with self.assertRaises(ValueError):
            build_capability_surface(self.vehicle, curvature_limit=0.0)
