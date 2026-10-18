"""Hydraulic suspension assembly, reduction oracles and forward smoke run."""

from __future__ import annotations

import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

from tests.test_suspension_layouts import compile_with  # noqa: E402
from simplified_vehicle_models.hydraulics.circuit import HydraulicCircuit  # noqa: E402
from simplified_vehicle_models.hydraulics.elements import AccumulatorSet, OrificeSet  # noqa: E402
from simplified_vehicle_models.suspension.layouts.four_mode import FOUR_MODE_MATRIX  # noqa: E402
from simplified_vehicle_models.suspension.layouts.hydraulic import HydraulicNetwork  # noqa: E402
from simplified_vehicle_models.solvers.vertical_forward import vertical_plant_class  # noqa: E402

HYDRAULIC_XML = ROOT / "models/suspensions/iem24_roll_decoupled_hydraulic.xml"


class ZeroDamper:
    def force(self, velocity):
        v = np.asarray(velocity, dtype=float)
        return SimpleNamespace(force=np.zeros_like(v), slope=np.zeros_like(v),
                               out_of_range=np.zeros(v.shape, dtype=bool))


def mode_circuit(areas, lines=()):
    count = len(areas)
    return HydraulicCircuit(
        node_names=tuple(f"node_{i}" for i in range(count)), areas=np.asarray(areas),
        accumulators=AccumulatorSet(
            precharge_pressure=np.full(count, 5.0e5),
            precharge_volume=np.full(count, 2.5e-4),
            exponent=np.full(count, 1.4),
            minimum_volume=np.full(count, 4.0e-5)),
        design_pressure=np.full(count, 1.2e6),
        line_from=np.asarray([pair[0] for pair in lines], dtype=int),
        line_to=np.asarray([pair[1] for pair in lines], dtype=int),
        orifices=OrificeSet(linear=np.full(len(lines), 1.0e-3) if lines else [],
                            quadratic=np.full(len(lines), 0.0) if lines else []),
    )


class HydraulicAssemblyTests(unittest.TestCase):
    def test_example_compiles_and_has_four_gas_states(self):
        compiled = compile_with(HYDRAULIC_XML)
        suspension = compiled.model.suspension
        self.assertEqual(suspension.layout, "hydraulic")
        self.assertEqual(suspension.network.n_states, 4)
        self.assertEqual(suspension.network.state_names,
                         ("gas_fraction_fl", "gas_fraction_fr", "gas_fraction_rl", "gas_fraction_rr"))
        np.testing.assert_allclose(suspension.network.static_check(),
                                   suspension.kinematics.static_sprung_load, rtol=1e-12)

    def test_initial_state_is_inside_each_accumulator_range(self):
        suspension = compile_with(HYDRAULIC_XML).model.suspension
        network = suspension.network
        x = network.initial_state()
        bounds = network.state_bounds
        self.assertTrue(np.all(x >= bounds[:, 0]))
        self.assertTrue(np.all(x <= bounds[:, 1]))
        np.testing.assert_allclose(network.pressures(x), 1.2e6, rtol=1e-10)

    def test_invalid_network_state_is_a_domain_error(self):
        suspension = compile_with(HYDRAULIC_XML).model.suspension
        x = suspension.network.initial_state()
        x[0] = 0.01
        with self.assertRaises(ValueError):
            suspension.network.forces(np.zeros(4), np.zeros(4), x)

    def test_stiffness_limit_matches_a_modal_gas_spring_oracle(self):
        suspension = compile_with(HYDRAULIC_XML).model.suspension
        # Four mode nodes, no lines: each gas node is one mode spring. The
        # circuit is compared against the modal transform, not against itself.
        circuit = mode_circuit(FOUR_MODE_MATRIX * 1.0e-3)
        network = HydraulicNetwork(suspension.kinematics, suspension.network.springs,
                                   circuit, (ZeroDamper(), ZeroDamper()))
        g = circuit.design_volumes()
        rates = circuit.accumulators.stiffness(g) * (1.0e-3 ** 2)
        # dP/dz = pressure stiffness times area squared.
        z = np.array([2e-5, -1e-5, 1.5e-5, -0.5e-5])
        actual = network.circuit.forces_from_state(network.circuit.equilibrium_volumes(z)) - network.circuit.design_forces()
        mode = FOUR_MODE_MATRIX @ z
        expected = FOUR_MODE_MATRIX.T @ (rates * mode)
        np.testing.assert_allclose(actual, expected, rtol=1e-3, atol=1e-5)

    def test_closed_lines_match_four_independent_gas_springs(self):
        suspension = compile_with(HYDRAULIC_XML).model.suspension
        circuit = mode_circuit(np.eye(4) * 1.0e-3)
        network = HydraulicNetwork(suspension.kinematics, suspension.network.springs,
                                   circuit, (ZeroDamper(), ZeroDamper()))
        z = np.array([1e-5, -2e-5, 1.5e-5, -0.5e-5])
        x = network.initial_state(z)
        x = x * circuit.accumulators.precharge_volume
        hydraulic = network.circuit.forces_from_state(x) - network.circuit.design_forces()
        pressure_delta = circuit.accumulators.pressure(x) - circuit.design_pressure
        expected = 1.0e-3 * pressure_delta
        np.testing.assert_allclose(hydraulic, expected, rtol=1e-10, atol=1e-12)

    def test_60_m_forward_run_has_no_domain_or_contact_failures(self):
        from tests.test_vertical_forward import ConstantThrottleDriver, TRIAL_SPEED, TRIAL_THROTTLE, straight_plan
        from simplified_vehicle_models.solvers.forward_driver import solve_forward

        vehicle = compile_with(HYDRAULIC_XML).model
        plant_class = vertical_plant_class(vehicle)
        plan = straight_plan(distance=60.0)
        plant = plant_class(vehicle, plan)
        start = time.perf_counter()
        result = solve_forward(
            plant, ConstantThrottleDriver(plan, TRIAL_THROTTLE), plant.initial_state(TRIAL_SPEED),
            duration=60.0, wall_seconds=250.0, maximum_step=0.005, rtol=1e-6,
            atol=1e-8, method="BDF")
        self.assertTrue(result.lap_completed)
        self.assertEqual(plant.out_of_range_evaluations, 0)
        self.assertEqual(plant.lifted_wheel_evaluations, 0)
        self.assertGreaterEqual(plant.minimum_contact_load, 0.0)
        self.assertLess(time.perf_counter() - start, 30.0)


if __name__ == "__main__":
    unittest.main()
