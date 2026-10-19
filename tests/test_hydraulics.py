"""Hydraulic elements and circuit (Task 6): orifices, accumulators and conservation.

The checks follow the plan:

* an orifice is monotone and its flow inverts the pressure drop;
* accumulator pressure follows the polytropic law and is bounded by its stops;
* volume is conserved around the circuit;
* the static equilibrium of a closed circuit equalises the pressure of joined nodes.
"""

from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from simplified_vehicle_models.hydraulics.circuit import HydraulicCircuit  # noqa: E402
from simplified_vehicle_models.hydraulics.elements import (  # noqa: E402
    ATMOSPHERE,
    AccumulatorSet,
    OrificeSet,
)
from simplified_vehicle_models.suspension.errors import DomainError  # noqa: E402


def accumulators(count=4, precharge=5.0e5, volume=2.5e-4, exponent=1.4, minimum=4.0e-5):
    return AccumulatorSet(
        precharge_pressure=np.full(count, precharge), precharge_volume=np.full(count, volume),
        exponent=np.full(count, exponent), minimum_volume=np.full(count, minimum))


def example_circuit(area=1.5e-3, design=1.2e6, linear=3.0e8, quadratic=5.0e11):
    return HydraulicCircuit(
        node_names=("fl", "fr", "rl", "rr"), areas=area * np.eye(4),
        accumulators=accumulators(), design_pressure=np.full(4, design),
        line_from=[0, 2], line_to=[1, 3],
        orifices=OrificeSet(linear=np.full(2, linear), quadratic=np.full(2, quadratic)))


class OrificeTests(unittest.TestCase):
    def setUp(self):
        self.orifice = OrificeSet(linear=[2.0e8, 1.0e9], quadratic=[5.0e11, 0.0])

    def test_flow_inverts_the_pressure_drop(self):
        q = np.array([[-3e-4, -1e-6, 0.0, 2e-6, 5e-4]]).T * np.ones((1, 2))
        for row in q:
            drop = self.orifice.pressure_drop(row)
            back, _ = self.orifice.flow(drop)
            np.testing.assert_allclose(back, row, rtol=1e-9, atol=1e-18)

    def test_flow_is_monotone_odd_and_zero_at_zero(self):
        drop = np.linspace(-5e6, 5e6, 401)[:, None] * np.ones((1, 2))
        q, slope = self.orifice.flow(drop)
        self.assertTrue(np.all(np.diff(q, axis=0) > 0.0))
        self.assertTrue(np.all(slope > 0.0))
        np.testing.assert_allclose(q, -q[::-1], atol=1e-15)
        self.assertEqual(float(q[200, 0]), 0.0)

    def test_slope_matches_a_finite_difference(self):
        drop = np.array([3.0e5, -8.0e5])
        h = 1.0
        up, _ = self.orifice.flow(drop + h)
        down, _ = self.orifice.flow(drop - h)
        _, slope = self.orifice.flow(drop)
        np.testing.assert_allclose(slope, (up - down) / (2 * h), rtol=1e-6)

    def test_a_pure_quadratic_free_orifice_is_linear(self):
        q, _ = self.orifice.flow(np.array([1.0e6, 1.0e6]))
        self.assertAlmostEqual(float(q[1]), 1.0e6 / 1.0e9, places=15)

    def test_counted_orifice_range_flag(self):
        valve = OrificeSet(linear=[2.0e8], quadratic=[0.0], maximum_pressure_drop=[1.0e5])
        _, _, flag = valve.flow_with_flag(np.array([5.0e4]))
        self.assertFalse(bool(flag[0]))
        _, _, flag = valve.flow_with_flag(np.array([2.0e5]))
        self.assertTrue(bool(flag[0]))

    def test_invalid_orifices_are_rejected(self):
        with self.assertRaises(ValueError):
            OrificeSet(linear=[0.0], quadratic=[0.0])
        with self.assertRaises(ValueError):
            OrificeSet(linear=[1.0], quadratic=[-1.0])
        with self.assertRaises(ValueError):
            OrificeSet(linear=[1.0, 2.0], quadratic=[1.0])


class CylinderTests(unittest.TestCase):
    def test_cylinder_flow_has_the_expected_area_and_sign(self):
        cylinders = __import__(
            "simplified_vehicle_models.hydraulics.elements", fromlist=["CylinderSet"]
        ).CylinderSet(np.array([[2.0e-3, 0.0, 0.0, 0.0], [-1.0e-3, 0.0, 0.0, 0.0]]))
        np.testing.assert_allclose(cylinders.flow(np.array([0.2, 0.0, 0.0, 0.0])), [4.0e-4, -2.0e-4])
        np.testing.assert_allclose(cylinders.force(np.array([1.0e6, 2.0e6])), [0.0, 0.0, 0.0, 0.0], atol=1e-12)


class AccumulatorTests(unittest.TestCase):
    def setUp(self):
        self.gas = accumulators(count=2)

    def test_pressure_follows_the_polytropic_law(self):
        v0 = 2.5e-4
        for ratio in (1.0, 0.8, 0.5, 0.3):
            g = np.full(2, ratio * v0)
            expected = (5.0e5 + ATMOSPHERE) * ratio ** -1.4 - ATMOSPHERE
            np.testing.assert_allclose(self.gas.pressure(g), expected, rtol=1e-12)

    def test_precharge_pressure_at_the_precharge_volume(self):
        np.testing.assert_allclose(self.gas.pressure(np.full(2, 2.5e-4)), 5.0e5, rtol=1e-12)

    def test_stiffness_is_minus_dp_dg(self):
        g = np.array([1.5e-4, 2.0e-4])
        h = 1.0e-9
        slope = (self.gas.pressure(g + h) - self.gas.pressure(g - h)) / (2 * h)
        np.testing.assert_allclose(self.gas.stiffness(g), -slope, rtol=1e-6)

    def test_pressure_is_monotone_decreasing_in_volume(self):
        g = np.linspace(4.0e-5, 2.5e-4, 50)
        p = np.array([self.gas.pressure(np.array([v, v]))[0] for v in g])
        self.assertTrue(np.all(np.diff(p) < 0.0))

    def test_volume_at_inverts_pressure(self):
        for pressure in (6.0e5, 1.2e6, 3.0e6):
            g = self.gas.volume_at(np.full(2, pressure))
            np.testing.assert_allclose(self.gas.pressure(g), pressure, rtol=1e-10)

    def test_volume_clips_at_the_precharge_volume_below_the_precharge(self):
        np.testing.assert_allclose(self.gas.volume_at(np.full(2, 1.0e5)), 2.5e-4)

    def test_a_volume_outside_the_stops_raises_a_domain_error(self):
        with self.assertRaises(DomainError):
            self.gas.pressure(np.array([3.0e-5, 2.0e-4]))
        with self.assertRaises(DomainError):
            self.gas.pressure(np.array([2.0e-4, 2.6e-4]))
        with self.assertRaises(DomainError):
            self.gas.volume_at(np.full(2, 1.0e8))

    def test_invalid_accumulators_are_rejected(self):
        with self.assertRaises(ValueError):
            accumulators(exponent=0.9)
        with self.assertRaises(ValueError):
            accumulators(minimum=3.0e-4)
        with self.assertRaises(ValueError):
            accumulators(precharge=-2.0e5)


class CircuitTests(unittest.TestCase):
    def setUp(self):
        self.circuit = example_circuit()

    def test_the_design_state_is_at_rest_and_carries_the_design_forces(self):
        x = self.circuit.design_volumes()
        rates = self.circuit.evaluate(x, np.zeros(4))
        np.testing.assert_allclose(rates.gas_rate, 0.0, atol=1e-18)
        np.testing.assert_allclose(rates.pressure, 1.2e6, rtol=1e-10)
        np.testing.assert_allclose(self.circuit.forces_from_state(x),
                                   1.5e-3 * 1.2e6, rtol=1e-10)
        np.testing.assert_allclose(self.circuit.equilibrium_volumes(np.zeros(4)), x, rtol=1e-9)

    def test_the_cylinder_converts_jounce_rate_to_gas_rate_with_its_sign(self):
        x = self.circuit.design_volumes()
        rates = self.circuit.evaluate(x, np.array([0.1, 0.0, 0.0, 0.0]))
        # Compression pushes fluid into the node, so the gas is squeezed: dg/dt < 0.
        self.assertAlmostEqual(float(rates.gas_rate[0] + 1.5e-3 * 0.1), 0.0, places=15)
        # At the design state both nodes share one pressure, so no line flow exists yet.
        self.assertEqual(rates.gas_rate[1], 0.0)
        self.assertEqual(rates.gas_rate[2], 0.0)
        # One instant later the squeezed node is at higher pressure and feeds its neighbour.
        squeezed = self.circuit.design_volumes() * np.array([0.9, 1.0, 1.0, 1.0])
        later = self.circuit.evaluate(squeezed, np.zeros(4))
        self.assertGreater(later.gas_rate[0], 0.0)
        self.assertLess(later.gas_rate[1], 0.0)

    def test_rod_side_area_has_the_opposite_sign(self):
        circuit = replace(self.circuit, areas=-self.circuit.areas)
        rates = circuit.evaluate(circuit.design_volumes(), np.array([0.1, 0.0, 0.0, 0.0]))
        self.assertAlmostEqual(float(rates.gas_rate[0] - 1.5e-3 * 0.1), 0.0, places=15)

    def test_fluid_volume_is_conserved_around_the_circuit(self):
        # With no wheel motion the lines only move fluid between nodes.
        x = self.circuit.design_volumes() * np.array([0.9, 1.0, 0.8, 0.95])
        x = np.minimum(x, 2.5e-4)
        rates = self.circuit.evaluate(x, np.zeros(4))
        np.testing.assert_allclose(rates.gas_rate[0] + rates.gas_rate[1], 0.0, atol=1e-18)
        np.testing.assert_allclose(rates.gas_rate[2] + rates.gas_rate[3], 0.0, atol=1e-18)
        # With wheel motion the total gas rate equals minus the delivered flow.
        zdot = np.array([0.3, -0.1, 0.2, 0.05])
        rates = self.circuit.evaluate(x, zdot)
        np.testing.assert_allclose(np.sum(rates.gas_rate), -1.5e-3 * np.sum(zdot), atol=1e-15)

    def test_flow_goes_from_the_high_pressure_node_to_the_low_one(self):
        x = np.array([1.5e-4, 2.0e-4, 2.0e-4, 2.0e-4])  # node 0 is squeezed: high pressure
        rates = self.circuit.evaluate(x, np.zeros(4))
        self.assertGreater(rates.line_flow[0], 0.0)
        self.assertGreater(rates.gas_rate[0], 0.0)  # the squeezed node loses fluid
        self.assertLess(rates.gas_rate[1], 0.0)

    def test_equilibrium_equalises_the_pressure_of_joined_nodes(self):
        z = np.array([0.02, -0.01, -0.015, 0.0])
        x = self.circuit.equilibrium_volumes(z)
        p = self.circuit.accumulators.pressure(x)
        self.assertAlmostEqual(float(p[0]), float(p[1]), delta=1e-3 * p[0])
        self.assertAlmostEqual(float(p[2]), float(p[3]), delta=1e-3 * p[2])
        # Fluid charge of each joined pair is the design charge less what the cylinders pushed in.
        design = self.circuit.design_volumes()
        np.testing.assert_allclose(x[0] + x[1], design[0] + design[1] - 1.5e-3 * (z[0] + z[1]),
                                   rtol=1e-9)
        np.testing.assert_allclose(x[2] + x[3], design[2] + design[3] - 1.5e-3 * (z[2] + z[3]),
                                   rtol=1e-9)

    def test_antisymmetric_jounce_leaves_a_joined_pair_unchanged(self):
        x = self.circuit.equilibrium_volumes(np.array([0.02, -0.02, 0.0, 0.0]))
        np.testing.assert_allclose(x, self.circuit.design_volumes(), rtol=1e-8)

    def test_a_charge_that_does_not_fit_raises_a_domain_error(self):
        with self.assertRaises(DomainError):
            self.circuit.equilibrium_volumes(np.array([0.5, 0.5, 0.0, 0.0]))
        with self.assertRaises(DomainError):
            self.circuit.equilibrium_volumes(np.array([-0.5, -0.5, 0.0, 0.0]))

    def test_an_out_of_range_state_raises_a_domain_error(self):
        x = self.circuit.design_volumes().copy()
        x[1] = 1.0e-5
        with self.assertRaises(DomainError):
            self.circuit.evaluate(x, np.zeros(4))

    def test_lines_between_unequal_design_pressures_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "different design pressures"):
            replace(self.circuit, design_pressure=np.array([1.2e6, 1.3e6, 1.2e6, 1.2e6]))

    def test_invalid_graphs_are_rejected(self):
        with self.assertRaises(ValueError):
            replace(self.circuit, line_from=[0, 2], line_to=[0, 3])  # a self loop
        with self.assertRaises(ValueError):
            replace(self.circuit, line_from=[0, 2], line_to=[1, 9])  # a missing node
        with self.assertRaises(ValueError):
            replace(self.circuit, node_names=("a", "a", "b", "c"))
        with self.assertRaises(ValueError):
            replace(self.circuit, areas=np.ones((3, 4)))

    def test_a_design_pressure_beyond_the_gas_stop_is_rejected(self):
        with self.assertRaises(DomainError):
            replace(self.circuit, design_pressure=np.full(4, 5.0e7))

    def test_components_group_the_joined_nodes(self):
        groups = sorted(sorted(int(i) for i in group) for group in self.circuit.components)
        self.assertEqual(groups, [[0, 1], [2, 3]])

    def test_the_arrays_are_read_only(self):
        with self.assertRaises(ValueError):
            self.circuit.areas[0, 0] = 1.0


if __name__ == "__main__":
    unittest.main()
