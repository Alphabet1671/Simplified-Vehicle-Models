"""Regression gate: the IEM24 14DoF suspension numbers must not move in a refactor.

The suspension-layout work (docs/superpowers/plans/2026-10-17-suspension-layouts.md)
reshapes how the vertical model reaches its suspension. The independent-corner
layout must reproduce the numbers recorded here. The fixture
``tests/fixtures/suspension_baseline.json`` was recorded from the code before the
refactor with::

    PYTHONPATH=src python3 tests/test_suspension_baseline.py --record

This module reads the suspension only through ``vehicle.suspension.network.forces``
(the force-network seam of Task 3), the evaluator, the solvers and the plant. A difference here is a bug in the refactor,
not a tolerance to widen.
"""

from __future__ import annotations

import json
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from simplified_vehicle_models.compiler.compile import compile_formulation  # noqa: E402
from simplified_vehicle_models.formulations.vertical_14dof import (  # noqa: E402
    VerticalState,
    evaluate_vertical_dynamics,
)
from simplified_vehicle_models.solvers.forward_driver import (  # noqa: E402
    DriverCommand,
    SpeedPlan,
    solve_forward,
)
from simplified_vehicle_models.solvers.vertical_equilibrium import (  # noqa: E402
    solve_static_equilibrium,
    solve_steady_corner,
)
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant  # noqa: E402
from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline  # noqa: E402

FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
FIXTURE = ROOT / "tests/fixtures/suspension_baseline.json"

#: Exact-equality tolerance for pure refactors: a few ulps of double precision.
RTOL = 1.0e-12
ATOL = 1.0e-12

_CACHE: dict = {}


def vehicle():
    if "vehicle" not in _CACHE:
        compiled = compile_formulation(FORMULATION).model
        # The baseline pins refactor invariance, so it runs with steering lift and
        # bump steer switched off; the IEM24 model itself carries the derived curves.
        kinematics = compiled.suspension.kinematics
        for axle in (0, 1):
            limits = kinematics.axle(axle).jounce_limits
            flat = MonotoneSpline([limits[0], 0.0, limits[1]], [0.0, 0.0, 0.0])
            kinematics = kinematics.with_axle(axle, jounce_steer=flat)
        kinematics = kinematics.with_axle(
            0, steering_lift_sine_left=0.0, steering_lift_cosine_left=0.0,
            steering_lift_sine_right=0.0, steering_lift_cosine_right=0.0)
        suspension = replace(compiled.suspension, kinematics=kinematics,
                             network=compiled.suspension.network.with_kinematics(kinematics))
        _CACHE["vehicle"] = replace(compiled, suspension=suspension)
    return _CACHE["vehicle"]


#: Mechanical states (21 values) that exercise roll, pitch, both ARB directions,
#: damper rebound and compression, and a roll-centre / anti-geometry load.
_MECHANICAL_CASES = {
    "rest": np.zeros(21),
    "bump_heave_pitch": VerticalState(
        v_x=18.0, heave=0.004, pitch=0.004, jounce_fl=0.006, jounce_fr=0.004,
        jounce_rl=-0.003, jounce_rr=-0.002, jounce_rate_fl=0.15, jounce_rate_fr=-0.1,
        jounce_rate_rl=0.05, jounce_rate_rr=-0.2, wheel_speed_fl=55.0, wheel_speed_fr=55.0,
        wheel_speed_rl=55.0, wheel_speed_rr=55.0).as_vector(),
    "left_roll": VerticalState(
        v_x=15.0, v_y=0.3, yaw_rate=0.4, roll=-0.03, heave=0.002, jounce_fl=-0.009,
        jounce_fr=0.011, jounce_rl=-0.007, jounce_rr=0.009, jounce_rate_fl=0.2,
        jounce_rate_fr=-0.25, jounce_rate_rl=0.1, jounce_rate_rr=-0.12, wheel_speed_fl=46.0,
        wheel_speed_fr=46.0, wheel_speed_rl=46.0, wheel_speed_rr=46.0).as_vector(),
}
_SAMPLES = {
    "rest": (np.zeros(4), np.zeros(4)),
    "heave_bump": (np.array([0.01, 0.01, 0.008, 0.008]), np.array([0.1, 0.1, 0.1, 0.1])),
    "roll_right": (np.array([-0.012, 0.012, -0.01, 0.01]), np.array([-0.2, 0.2, -0.15, 0.15])),
    "warp_rebound": (np.array([0.01, -0.01, -0.01, 0.01]), np.array([-0.3, 0.1, 0.2, -0.4])),
}


def _plant(speed: float, distance: float = 40.0):
    plan = SpeedPlan(np.array([0.0, distance]), np.full(2, float(speed)), np.zeros(2))
    return VerticalForwardPlant(vehicle(), plan), plan


def record() -> dict:
    """Compute every pinned quantity from the current code."""

    v = vehicle()
    out: dict = {"samples": {}, "evaluations": {}, "equilibrium": {}, "plant": {}}
    for name, (z, zdot) in _SAMPLES.items():
        forces = v.suspension.network.forces(z, zdot)
        out["samples"][name] = {
            "jounce": z.tolist(), "jounce_rate": zdot.tolist(),
            "spring": forces.elements["spring"].tolist(),
            "arb": forces.elements["arb"].tolist(),
            "damper": forces.elements["damper"].tolist(), "total": forces.total.tolist(),
            "damper_velocity": forces.damper_velocity.tolist(),
            "out_of_range": forces.out_of_range.astype(int).tolist(),
        }
    plant, _ = _plant(15.0)
    initial = plant.initial_state(15.0)
    tires = plant.tire_states(initial)
    for name, vector in _MECHANICAL_CASES.items():
        state = VerticalState.from_vector(vector)
        evaluation = evaluate_vertical_dynamics(
            v, state, np.array([20.0, 20.0, 40.0, 40.0]), 0.02, tires)
        out["evaluations"][name] = {
            "state": vector.tolist(),
            "rates": evaluation.rates.tolist(),
            "generalized_forces": evaluation.generalized_forces.tolist(),
            "normal_loads": evaluation.normal_loads.tolist(),
            "corner_total": evaluation.corner_forces.total.tolist(),
            "out_of_range_count": int(evaluation.out_of_range_count),
        }
    static = solve_static_equilibrium(v)
    out["equilibrium"]["static"] = static.as_vector().tolist()
    steady = solve_steady_corner(v, 15.0, 6.0)
    out["equilibrium"]["steady_corner_15ms_6ms2"] = steady.as_vector().tolist()
    system = plant.evaluate(0.0, initial, DriverCommand(0.0, 0.0, 0.0))
    out["plant"]["initial_state"] = initial.tolist()
    out["plant"]["initial_rates"] = np.asarray(system.forcing, dtype=float).tolist()
    out["plant"]["initial_mass_matrix_is_none"] = system.mass_matrix is None
    out["plant"]["jounce_minimum"] = plant.jounce_minimum.tolist()
    out["plant"]["jounce_maximum"] = plant.jounce_maximum.tolist()
    result = _short_run()
    out["plant"]["run_final_state"] = result.state[:, -1].tolist()
    out["plant"]["run_final_time"] = float(result.time[-1])
    return out


class _Throttle:
    def __init__(self, plan, throttle):
        self.plan = plan
        self.throttle = throttle

    def command(self, observation):
        return DriverCommand(self.throttle, 0.0, 0.0)


def _short_run():
    plant, plan = _plant(20.0, 100.0)
    initial = plant.initial_state(20.0)
    return solve_forward(plant, _Throttle(plan, 0.10), initial, duration=0.6, wall_seconds=120.0,
                         maximum_step=0.005, rtol=1e-6, atol=1e-8, method="BDF")


def _close(test: unittest.TestCase, actual, expected, label: str) -> None:
    np.testing.assert_allclose(
        np.asarray(actual, dtype=float), np.asarray(expected, dtype=float),
        rtol=RTOL, atol=ATOL, err_msg=label)


class SuspensionBaselineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.expected = json.loads(FIXTURE.read_text())

    def test_corner_force_breakdown_is_unchanged(self):
        v = vehicle()
        for name, row in self.expected["samples"].items():
            with self.subTest(sample=name):
                forces = v.suspension.network.forces(np.array(row["jounce"]),
                                                     np.array(row["jounce_rate"]))
                for key in ("spring", "arb", "damper"):
                    _close(self, forces.elements[key], row[key], f"{name}.{key}")
                for key in ("total", "damper_velocity"):
                    _close(self, getattr(forces, key), row[key], f"{name}.{key}")
                self.assertEqual(forces.out_of_range.astype(int).tolist(), row["out_of_range"])

    def test_evaluator_outputs_are_unchanged(self):
        v = vehicle()
        plant, _ = _plant(15.0)
        tires = plant.tire_states(plant.initial_state(15.0))
        for name, row in self.expected["evaluations"].items():
            with self.subTest(case=name):
                evaluation = evaluate_vertical_dynamics(
                    v, VerticalState.from_vector(row["state"]),
                    np.array([20.0, 20.0, 40.0, 40.0]), 0.02, tires)
                _close(self, evaluation.rates, row["rates"], f"{name}.rates")
                _close(self, evaluation.generalized_forces, row["generalized_forces"],
                       f"{name}.generalized_forces")
                _close(self, evaluation.normal_loads, row["normal_loads"], f"{name}.loads")
                _close(self, evaluation.corner_forces.total, row["corner_total"],
                       f"{name}.corner_total")
                self.assertEqual(int(evaluation.out_of_range_count), row["out_of_range_count"])

    def test_static_and_steady_corner_equilibria_are_unchanged(self):
        v = vehicle()
        _close(self, solve_static_equilibrium(v).as_vector(),
               self.expected["equilibrium"]["static"], "static")
        _close(self, solve_steady_corner(v, 15.0, 6.0).as_vector(),
               self.expected["equilibrium"]["steady_corner_15ms_6ms2"], "steady corner")

    def test_forward_plant_initial_state_and_rates_are_unchanged(self):
        plant, _ = _plant(15.0)
        initial = plant.initial_state(15.0)
        expected = self.expected["plant"]
        _close(self, initial, expected["initial_state"], "initial state")
        system = plant.evaluate(0.0, initial, DriverCommand(0.0, 0.0, 0.0))
        _close(self, system.forcing, expected["initial_rates"], "initial rates")
        self.assertEqual(system.mass_matrix is None, expected["initial_mass_matrix_is_none"])
        _close(self, plant.jounce_minimum, expected["jounce_minimum"], "jounce min")
        _close(self, plant.jounce_maximum, expected["jounce_maximum"], "jounce max")

    def test_short_forward_run_is_unchanged(self):
        result = _short_run()
        expected = self.expected["plant"]
        self.assertEqual(float(result.time[-1]), expected["run_final_time"])
        # The adaptive stepper amplifies ulp differences, so the run uses a
        # looser (still tight) tolerance than the single-evaluation checks.
        np.testing.assert_allclose(result.state[:, -1], expected["run_final_state"],
                                   rtol=1e-9, atol=1e-9)


if __name__ == "__main__":
    if "--record" in sys.argv:
        FIXTURE.parent.mkdir(parents=True, exist_ok=True)
        FIXTURE.write_text(json.dumps(record(), indent=1) + "\n")
        print(f"recorded {FIXTURE}")
    else:
        unittest.main()
