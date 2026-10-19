"""The kinematics / force-network seam of the vertical suspension (Task 3).

The suspension compiles to ``CompiledSuspension(kinematics, network, layout)``.
The independent-corner layout is a pure refactor of the former
``corner_forces``; ``tests/test_suspension_baseline.py`` pins its numbers. This
module checks the seam itself: accessors, static check, inertia, and the
compile-time errors.
"""

from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from simplified_vehicle_models.compiler.compile import compile_formulation  # noqa: E402
from simplified_vehicle_models.suspension.corner import load_corner_arrays  # noqa: E402
from simplified_vehicle_models.suspension.elements import AxleArbRates, CornerSprings  # noqa: E402
from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics  # noqa: E402
from simplified_vehicle_models.suspension.layouts.independent_corner import (  # noqa: E402
    IndependentCornerNetwork,
    build_independent_corner,
)
from simplified_vehicle_models.suspension.network import (  # noqa: E402
    CompiledSuspension,
    ForceNetwork,
    NetworkForces,
)

FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
NPZ = ROOT / "models/data/iem24_murphy_real_corners.npz"
NPZ_SHA = "9fbee5d0f237d999e79024e60542a711320a523f73e30577682bbe1bd8dc9904"

_CACHE: dict = {}


def compiled():
    if "c" not in _CACHE:
        _CACHE["c"] = compile_formulation(FORMULATION)
    return _CACHE["c"]


def suspension() -> CompiledSuspension:
    return compiled().model.suspension


class CompiledShapeTests(unittest.TestCase):
    def test_compiles_to_kinematics_plus_network(self):
        s = suspension()
        self.assertIsInstance(s, CompiledSuspension)
        self.assertEqual(s.layout, "independent-corner")
        self.assertIsInstance(s.kinematics, SuspensionKinematics)
        self.assertIsInstance(s.network, IndependentCornerNetwork)
        self.assertIsInstance(s.network, ForceNetwork)
        self.assertEqual(s.network.n_states, 0)

    def test_manifest_lists_children_under_role_paths(self):
        roles = {record.role for record in compiled().manifest.loaded_components}
        self.assertTrue({"suspension", "suspension/kinematics", "suspension/spring",
                         "suspension/arb", "suspension/damper-front",
                         "suspension/damper-rear"} <= roles)
        self.assertNotIn("damper-front", roles)
        self.assertEqual(compiled().manifest.unused_components, ())

    def test_vehicle_has_no_front_rear_or_dampers_reach(self):
        vehicle = compiled().model
        self.assertFalse(hasattr(vehicle, "dampers"))
        self.assertFalse(hasattr(vehicle.suspension, "front"))


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k = suspension().kinematics
        cls.arrays = load_corner_arrays(NPZ, NPZ_SHA)

    def test_motion_ratio_matches_table(self):
        for wheel, axle in ((0, "front"), (1, "front"), (2, "rear"), (3, "rear")):
            expected = np.interp(0.01, self.arrays[f"motion_ratio_jounce_{axle}"],
                                 self.arrays[f"motion_ratio_{axle}"])
            self.assertAlmostEqual(self.k.motion_ratio(wheel, 0.01), float(expected), places=3)
        np.testing.assert_array_equal(
            self.k.motion_ratios([0.0, 0.01, -0.01, 0.0]),
            [self.k.motion_ratio(0, 0.0), self.k.motion_ratio(1, 0.01),
             self.k.motion_ratio(2, -0.01), self.k.motion_ratio(3, 0.0)])

    def test_axle_accessors_match_the_npz(self):
        for axle, name in ((0, "front"), (1, "rear")):
            self.assertEqual(self.k.anti_brake(axle), float(self.arrays[f"anti_brake_{name}"]))
            self.assertEqual(self.k.anti_drive(axle), float(self.arrays[f"anti_drive_{name}"]))
            self.assertEqual(self.k.camber_gain(axle), float(self.arrays[f"camber_gain_{name}"]))
            self.assertEqual(self.k.roll_centre_height(axle, 0.0),
                             float(self.arrays[f"rc_height_{name}"][0]))
        with self.assertRaises(ValueError):
            self.k.anti_brake(2)

    def test_jounce_limits_are_per_wheel(self):
        limits = self.k.jounce_limits
        self.assertEqual(limits.shape, (4, 2))
        np.testing.assert_array_equal(limits[0], limits[1])
        np.testing.assert_array_equal(limits[2], limits[3])
        np.testing.assert_array_equal(limits[0], self.arrays["jounce_limits_front"])
        np.testing.assert_array_equal(limits[2], self.arrays["jounce_limits_rear"])

    def test_toe_signs_follow_the_positive_right_convention(self):
        toed = self.k.with_axle(0, static_toe=0.01).with_axle(1, static_toe=0.02)
        np.testing.assert_allclose(toed.toe(np.zeros(4)), [0.01, -0.01, 0.02, -0.02])
        with self.assertRaises(ValueError):
            self.k.toe(np.zeros(3))

    def test_static_sprung_load_matches_npz(self):
        np.testing.assert_array_equal(
            self.k.static_sprung_load,
            [float(self.arrays["static_sprung_load_front"])] * 2
            + [float(self.arrays["static_sprung_load_rear"])] * 2)


class NetworkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s = suspension()
        cls.n = cls.s.network

    def test_static_check_equals_the_sprung_static_loads(self):
        np.testing.assert_allclose(self.n.static_check(), [505.77797375] * 2 + [646.50340125] * 2,
                                   rtol=1e-12)
        np.testing.assert_allclose(self.n.static_check(), self.s.kinematics.static_sprung_load,
                                   rtol=1e-9)

    def test_forces_at_rest_are_the_spring_preload(self):
        forces = self.n.forces(np.zeros(4), np.zeros(4))
        self.assertIsInstance(forces, NetworkForces)
        np.testing.assert_allclose(forces.total, self.n.static_check())
        np.testing.assert_array_equal(forces.elements["damper"], np.zeros(4))
        np.testing.assert_array_equal(forces.elements["arb"], np.zeros(4))
        self.assertEqual(forces.ds_dt.size, 0)
        self.assertFalse(forces.out_of_range.any())

    def test_total_is_the_sum_of_named_elements(self):
        z = np.array([0.01, -0.012, 0.004, 0.008])
        zdot = np.array([0.1, -0.2, 0.05, 0.3])
        forces = self.n.forces(z, zdot)
        np.testing.assert_array_equal(
            forces.total, forces.elements["spring"] + forces.elements["arb"]
            + forces.elements["damper"])
        # The ARB is antisymmetric on each axle and the rear axle carries it.
        np.testing.assert_array_equal(forces.elements["arb"][0], -forces.elements["arb"][1])
        self.assertEqual(forces.elements["arb"][0], 0.0)
        self.assertGreater(abs(forces.elements["arb"][2]), 0.0)

    def test_static_forces_drop_the_damper_only(self):
        z = np.array([0.01, -0.012, 0.004, 0.008])
        full = self.n.forces(z, np.zeros(4))
        np.testing.assert_allclose(self.n.static_forces(z), full.total, rtol=1e-12)

    def test_outside_the_travel_raises_not_extrapolates(self):
        with self.assertRaises(ValueError):
            self.n.forces(np.array([0.2, 0, 0, 0]), np.zeros(4))

    def test_shape_validation(self):
        with self.assertRaises(ValueError):
            self.n.forces(np.zeros(3), np.zeros(4))
        with self.assertRaises(ValueError):
            self.n.forces(np.zeros(4), np.full(4, np.nan))

    def test_zero_inertance_adds_no_inertia(self):
        self.assertFalse(self.n.has_inertia)
        np.testing.assert_array_equal(self.n.inertia_block(np.zeros(4)), np.zeros((4, 4)))
        np.testing.assert_array_equal(self.n.inertial_forcing(np.zeros(4), np.ones(4)), np.zeros(4))

    def test_inertia_block_is_b_over_motion_ratio_squared(self):
        net = self.n.with_inertance(2.0, 3.0)
        self.assertTrue(net.has_inertia)
        z = np.array([0.0, 0.005, -0.004, 0.002])
        block = net.inertia_block(z)
        expected = np.zeros((4, 4))
        for i in range(4):
            expected[i, i] = (2.0, 2.0, 3.0, 3.0)[i] / self.s.kinematics.motion_ratio(i, z[i]) ** 2
        np.testing.assert_array_equal(block, expected)
        np.testing.assert_array_equal(np.diag(np.diag(block)), block)

    def test_inertial_forcing_matches_the_lagrange_term(self):
        net = self.n.with_inertance(2.0, 0.0)
        z = np.array([0.006, -0.003, 0.0, 0.0])
        zdot = np.array([0.4, -0.2, 0.9, 0.9])
        forcing = net.inertial_forcing(z, zdot)
        for i in (0, 1):
            ratio = self.s.kinematics.motion_ratio(i, z[i])
            slope = self.s.kinematics.motion_ratio_slope(i, z[i])
            b_slope = -2.0 * 2.0 * slope / ratio**3
            self.assertEqual(forcing[i], -0.5 * b_slope * zdot[i] ** 2)
        self.assertEqual(forcing[2], 0.0)
        self.assertEqual(forcing[3], 0.0)

    def test_replaced_pieces_flow_through_the_forces(self):
        z = np.array([0.01, -0.01, 0.01, -0.01])
        base = self.n.forces(z, np.zeros(4)).elements["arb"]
        stiff = self.n.with_arb(5000.0, 21015.220229577164).forces(z, np.zeros(4)).elements["arb"]
        self.assertEqual(base[0], 0.0)
        self.assertEqual(stiff[0], 0.5 * 5000.0 * 0.02)
        self.assertEqual(stiff[2], base[2])


class CompiledSuspensionTests(unittest.TestCase):
    def test_with_axle_geometry_rebinds_network_kinematics(self):
        s = suspension().with_axle_geometry(0, static_toe=0.003)
        self.assertEqual(s.kinematics.front.static_toe, 0.003)
        self.assertIs(s.network.kinematics, s.kinematics)
        self.assertEqual(suspension().kinematics.front.static_toe, 0.0)

    def test_static_check_failure_stops_the_compile(self):
        s = suspension()
        n = s.network
        weaker = replace(
            s.kinematics,
            front=replace(s.kinematics.front, static_sprung_load=400.0),
        )
        with self.assertRaisesRegex(ValueError, "static corner loads"):
            build_independent_corner(
                kinematics=weaker, springs=n.springs, arb=n.arb,
                damper_front=n.dampers[0], damper_rear=n.dampers[1],
                inertance_front=0.0, inertance_rear=0.0)

    def test_negative_inertance_is_rejected(self):
        s = suspension()
        n = s.network
        with self.assertRaises(ValueError):
            build_independent_corner(
                kinematics=s.kinematics, springs=n.springs, arb=n.arb,
                damper_front=n.dampers[0], damper_rear=n.dampers[1],
                inertance_front=-1.0, inertance_rear=0.0)

    def test_element_value_validation(self):
        with self.assertRaises(ValueError):
            AxleArbRates(-1.0, 0.0)
        springs = suspension().network.springs
        with self.assertRaises(ValueError):
            springs.axle(2)
        self.assertIsInstance(springs, CornerSprings)


if __name__ == "__main__":
    unittest.main()
