"""Task 10 focus tests: the damper and spring setup sweep tool.

What is gated here and where the oracle comes from:

* **Baseline click sweep.** Compression clicks 0-11 at rebound 7 must increase
  the per-axle corner damping ratio strictly (plan Task 10 Step 1). The corner
  damping ratio itself is re-derived here from the compiled damper's own force
  at the 25 mm/s reference velocity, the axle motion ratio, the quarter-car
  masses and the tangent spring/tire rates: a hand calculation that never calls
  the tool's own helpers. It is also compared with the plan's D7 reference
  (0.49 / 0.78 front, 0.43 / 0.68 rear) within 10 %; the plan quotes the
  series-spring ride frequency, this tool the exact two-DoF quarter-car ride
  mode, which is the documented reason for the few-percent difference.
* **Step oracle.** A 2 000 N front-axle step is integrated twice: by the tool's
  pure-NumPy analytic state-space solution and by the Task 7
  ``VerticalForwardPlant`` seven-coordinate linear oracle
  (``tests.test_vertical_forward`` helpers, SciPy ``DOP853`` over the closed-form
  mass/stiffness/damping). The plan's +-15 % first-peak tolerance is applied to
  both the recorded Task 7 numbers and the recomputed oracle.
* **Frequency response.** Displacement per generalized force
  ``H_ii(w) = e_i' (K - w^2 M + i w C)^-1 e_i`` on a defined grid. The heave,
  roll, and pitch peaks must sit near the corresponding damped modal
  frequencies. The rear ARB moves the roll peak and leaves heave exactly alone.
* **Overrides.** Names are derived from what the formulation exposes: the valve
  names of both compiled damper tables plus the compiled suspension scalars.
  Unknown names, out-of-range clicks, and fractional clicks raise ``ValueError``.
  A tmp four-valve synthetic damper (built the way ``tests/test_damper_table.py``
  builds its tables) accepts ``click_<valve>`` for all four valves.
"""

import csv
import hashlib
import json
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.vertical_14dof import (
    sprung_wheel_positions,
)
from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
from simplified_vehicle_models.suspension.damper_table import DamperSetting, damper_force
from simplified_vehicle_models.tires.vertical import tire_deflection_load

from tools.run.sweep_vertical_setup import (
    DEFAULT_FORMULATION,
    POSITION_ROWS,
    REFERENCE_DAMPER_VELOCITY,
    SweepResult,
    apply_overrides,
    sweep_vertical_setup,
    valid_override_names,
    write_csv,
    write_json,
)

ROOT = Path(__file__).resolve().parents[1]
FORMULATION_XML = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
VEHICLE_XML = ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
SUSPENSION_XML = ROOT / "models/suspensions/iem24_murphy_real_suspension.xml"
MODELS = ROOT / "models"

#: Plan "Reference numbers" D7: damping ratio at 25 mm/s, bump click 5 / rebound click 7.
PLAN_DAMPING_REFERENCE = {"front": (0.49, 0.78), "rear": (0.43, 0.68)}
#: Task 7 report, front body-point drop: linear seven-coordinate oracle.
TASK7_LINEAR_ORACLE = {"peak_m": 0.032504, "time_s": 0.0797, "overshoot": 0.2947}

_VEHICLE_CACHE: dict = {}


def compiled_vehicle():
    """Return the compiled IEM24 14-DoF vehicle, cached across the module."""

    if "vehicle" not in _VEHICLE_CACHE:
        _VEHICLE_CACHE["vehicle"] = compile_formulation(FORMULATION_XML).model
    return _VEHICLE_CACHE["vehicle"]


def baseline_clicks(axle_index: int, compression: int, rebound: int) -> DamperSetting:
    """Return the click setting that matches each valve half of one compiled damper."""

    damper = compiled_vehicle().suspension.network.dampers[axle_index]
    return DamperSetting(
        {
            valve.name: (compression if valve.half == "compression" else rebound)
            for valve in damper.table.valves
        }
    )


def hand_corner_damping_ratio(axle_index: int, compression: int, rebound: int) -> float:
    """Re-derive one axle's corner damping ratio from raw compiled data.

    ``zeta = c_wheel / (2 m_s_corner omega_ride)`` with the secant damper rate
    ``c_wheel = (F(v_ref)/v_ref) / MR^2`` at the 25 mm/s reference velocity and
    ``omega_ride`` the lower mode of the two-DoF body-fixed quarter car built
    from the tangent spring rate, the tangent tire rate, and the corner masses.
    """

    vehicle = compiled_vehicle()
    damper = vehicle.suspension.network.dampers[axle_index]
    setting = baseline_clicks(axle_index, compression, rebound)
    force = float(np.asarray(damper_force(damper.table, setting, np.array([REFERENCE_DAMPER_VELOCITY])).force)[0])
    ratio = vehicle.suspension.kinematics.motion_ratio(2 * axle_index, 0.0)
    c_wheel = (force / REFERENCE_DAMPER_VELOCITY) / ratio**2
    q0 = solve_static_equilibrium(vehicle).as_vector()
    x0 = np.asarray(q0)[list(POSITION_ROWS)]
    positions = sprung_wheel_positions(vehicle)
    body = x0[0] + positions[:, 1] * x0[1] - positions[:, 0] * x0[2]
    stiffness = np.asarray(
        tire_deflection_load(vehicle.planar.tire, body - x0[3:], np.zeros(4), 0.0).stiffness,
        dtype=float,
    )
    spring = float(vehicle.suspension.network.springs.axle(axle_index).derivative(0.0))
    fraction = vehicle.sprung_front_fraction if axle_index == 0 else 1.0 - vehicle.sprung_front_fraction
    sprung_corner = vehicle.sprung_mass * fraction / 2.0
    unsprung = vehicle.unsprung_mass_front if axle_index == 0 else vehicle.unsprung_mass_rear
    tire = float(stiffness[0 if axle_index == 0 else 2])
    frequencies = np.sqrt(
        np.sort(
            np.linalg.eigvals(
                np.linalg.solve(
                    np.diag([sprung_corner, unsprung]),
                    np.array([[spring, -spring], [-spring, spring + tire]]),
                )
            ).real
        )
    )
    omega = float(frequencies[0])
    return c_wheel / (2.0 * sprung_corner * omega)


def _synthetic_four_valve_formulation(directory: Path) -> Path:
    """Write a tmp four-valve damper, assembly, and formulation; return the formulation.

    The synthetic table has two compression valves (``lsc`` low band, ``hsc``
    high band) and two rebound valves, built exactly like
    ``tests/test_damper_table.py`` builds its four-valve table. The assembly is
    the real IEM24 assembly with the two damper roles pointed at the synthetic
    XMLs; every other reference is rewritten to an absolute path so nothing else
    moves.
    """

    directory = Path(directory)
    velocity_compression = np.array([0.0, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0])
    velocity_rebound = -velocity_compression[::-1]
    force_compression = np.zeros((velocity_compression.size, 4, 3))
    for index in range(1, velocity_compression.size):
        for low in range(4):
            for high in range(3):
                force_compression[index, low, high] = (
                    100.0 * (1 + low) * np.sqrt(velocity_compression[index])
                    + 50.0 * (1 + high) * velocity_compression[index] ** 1.5
                )
    force_rebound = np.zeros((velocity_rebound.size, 4, 3))
    for index in range(velocity_rebound.size - 1):
        speed = -velocity_rebound[index]
        for low in range(4):
            for high in range(3):
                force_rebound[index, low, high] = -(
                    150.0 * (1 + low) * np.sqrt(speed) + 80.0 * (1 + high) * speed**1.5
                )
    npz_path = directory / "synthetic_four_valve.npz"
    np.savez(
        npz_path,
        schema=np.array("damper-table-v1"),
        velocity_compression=velocity_compression,
        velocity_rebound=velocity_rebound,
        valve_names=np.array(["lsc", "hsc", "lsr", "hsr"]),
        valve_halves=np.array(["compression", "compression", "rebound", "rebound"]),
        valve_bands=np.array(["low", "high", "low", "high"]),
        valve_clicks=np.array([4, 3, 4, 3]),
        force_compression=force_compression,
        force_rebound=force_rebound,
        source_sha256=np.array("synthetic"),
        generator_version=np.array("synthetic-four-valve"),
        corrections=np.array("{}"),
    )
    digest = hashlib.sha256(npz_path.read_bytes()).hexdigest()
    damper_xmls = {}
    for axle, subtype in (("front", "table-front"), ("rear", "table-rear")):
        path = directory / f"synthetic_four_valve_{axle}.xml"
        path.write_text(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            f'<model schema-version="1.0" id="synthetic-four-valve-{axle}" '
            f'type="damper-model" subtype="{subtype}">\n'
            "  <parameters>\n"
            '    <parameter name="click_lsc" value="1" unit="1"/>\n'
            '    <parameter name="click_hsc" value="1" unit="1"/>\n'
            '    <parameter name="click_lsr" value="1" unit="1"/>\n'
            '    <parameter name="click_hsr" value="1" unit="1"/>\n'
            "  </parameters>\n"
            f'  <data role="damper-table" path="{npz_path}" format="npz" '
            f'sha256="{digest}"/>\n'
            "</model>\n"
        )
        damper_xmls[axle] = path
    # The dampers are children of the suspension assembly: write a tmp copy of it
    # with the two damper roles pointed at the synthetic files.
    suspension_text = SUSPENSION_XML.read_text()
    suspension_text = suspension_text.replace('ref="../', f'ref="{MODELS}/')
    suspension_text = suspension_text.replace('path="../', f'path="{MODELS}/')
    suspension_text = suspension_text.replace(
        'id="iem24-murphy-real-suspension"', 'id="synthetic-four-valve-suspension"'
    )
    for axle in ("front", "rear"):
        real = f'{MODELS}/dampers/multimatic_dssv_vc01_{axle}.xml'
        self_check = f'ref="{real}"'
        assert self_check in suspension_text, self_check
        suspension_text = suspension_text.replace(self_check, f'ref="{damper_xmls[axle]}"')
    suspension_path = directory / "synthetic_four_valve_suspension.xml"
    suspension_path.write_text(suspension_text)
    assembly_text = VEHICLE_XML.read_text()
    assembly_text = assembly_text.replace('ref="../', f'ref="{MODELS}/')
    assembly_text = assembly_text.replace('path="../', f'path="{MODELS}/')
    assembly_text = assembly_text.replace(
        'id="iem24-murphy-real-planar"', 'id="synthetic-four-valve-assembly"'
    )
    real_suspension = f'ref="{MODELS}/suspensions/iem24_murphy_real_suspension.xml"'
    assert real_suspension in assembly_text, real_suspension
    assembly_text = assembly_text.replace(real_suspension, f'ref="{suspension_path}"')
    assembly_path = directory / "synthetic_four_valve_assembly.xml"
    assembly_path.write_text(assembly_text)
    formulation_text = FORMULATION_XML.read_text()
    formulation_text = formulation_text.replace(
        'id="iem24-murphy-real-14dof-simple"', 'id="synthetic-four-valve-formulation"'
    )
    source = '<source ref="../vehicles/iem24_murphy_real_planar.xml" />'
    assert source in formulation_text, source
    formulation_text = formulation_text.replace(source, f'<source ref="{assembly_path}" />')
    formulation_path = directory / "synthetic_four_valve_formulation.xml"
    formulation_path.write_text(formulation_text)
    return formulation_path


class OverrideValidationTests(unittest.TestCase):
    """Override names and values are validated against the compiled model."""

    def test_valid_names_come_from_the_compiled_dampers_and_suspension(self):
        names = valid_override_names(compiled_vehicle())
        self.assertIn("click_compression", names)
        self.assertIn("click_rebound", names)
        self.assertIn("front.click_compression", names)
        self.assertIn("rear.click_rebound", names)
        for scalar in ("arb_wheel_rate_front", "arb_wheel_rate_rear",
                       "inertance_front", "inertance_rear",
                       "static_toe_front", "static_toe_rear"):
            with self.subTest(name=scalar):
                self.assertIn(scalar, names)
        self.assertNotIn("click_lsc", names)
        self.assertNotIn("wheel_rate_spring_front", names)

    def test_unknown_name_raises_and_lists_the_valid_names(self):
        with self.assertRaises(ValueError) as caught:
            sweep_vertical_setup(FORMULATION_XML, [{"clicks_compression": 3}], "step-response")
        message = str(caught.exception)
        self.assertIn("clicks_compression", message)
        self.assertIn("click_compression", message)
        self.assertIn("arb_wheel_rate_rear", message)

    def test_out_of_range_and_fractional_clicks_raise(self):
        for value in (12, -1, 6.5, float("nan")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    apply_overrides(compiled_vehicle(), {"click_compression": value})

    def test_negative_arb_and_inertance_raise(self):
        for name, value in (("arb_wheel_rate_rear", -1.0), ("inertance_front", -0.5)):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    apply_overrides(compiled_vehicle(), {name: value})

    def test_apply_overrides_rebuilds_the_setting_and_the_axle_scalars(self):
        vehicle = apply_overrides(
            compiled_vehicle(),
            {"click_compression": 3, "rear.click_rebound": 2, "arb_wheel_rate_rear": 1000.0},
        )
        self.assertEqual(vehicle.suspension.network.dampers[0].setting.clicks["compression"], 3.0)
        self.assertEqual(vehicle.suspension.network.dampers[0].setting.clicks["rebound"], 7.0)
        self.assertEqual(vehicle.suspension.network.dampers[1].setting.clicks["compression"], 3.0)
        self.assertEqual(vehicle.suspension.network.dampers[1].setting.clicks["rebound"], 2.0)
        self.assertEqual(vehicle.suspension.network.arb.rear, 1000.0)
        self.assertEqual(vehicle.suspension.network.arb.front, 0.0)

    def test_axle_qualified_click_names_are_independent(self):
        vehicle = apply_overrides(
            compiled_vehicle(),
            {"front.click_compression": 1, "rear.click_compression": 9},
        )
        self.assertEqual(vehicle.suspension.network.dampers[0].setting.clicks["compression"], 1.0)
        self.assertEqual(vehicle.suspension.network.dampers[1].setting.clicks["compression"], 9.0)


class BaselineClickSweepTests(unittest.TestCase):
    """Compression clicks 0-11 at rebound 7, one 12-setup sweep."""

    @classmethod
    def setUpClass(cls):
        cls.started = time.perf_counter()
        cls.overrides = [{"click_compression": click, "click_rebound": 7} for click in range(12)]
        cls.results = sweep_vertical_setup(FORMULATION_XML, cls.overrides, "step-response")
        cls.elapsed = time.perf_counter() - cls.started

    def test_one_result_per_setup_and_the_override_is_echoed(self):
        self.assertEqual(len(self.results), 12)
        for click, result in enumerate(self.results):
            with self.subTest(click=click):
                self.assertIsInstance(result, SweepResult)
                self.assertEqual(result.overrides["click_compression"], float(click))
                self.assertEqual(result.overrides["click_rebound"], 7.0)
                self.assertEqual(result.study, "step-response")

    def test_compression_click_increases_the_corner_damping_ratio_strictly(self):
        front = [result.front_corner_damping_ratio for result in self.results]
        rear = [result.rear_corner_damping_ratio for result in self.results]
        for name, values in (("front", front), ("rear", rear)):
            with self.subTest(axle=name):
                self.assertTrue(
                    all(b > a for a, b in zip(values, values[1:])),
                    f"{name} corner damping ratios are not strictly increasing: {values}",
                )

    def test_baseline_matches_the_plan_reference_damping_ratios(self):
        baseline = self.results[5]
        measured = {
            "front": (baseline.front_corner_damping_ratio, baseline.front_corner_damping_ratio_rebound),
            "rear": (baseline.rear_corner_damping_ratio, baseline.rear_corner_damping_ratio_rebound),
        }
        for axle, reference in PLAN_DAMPING_REFERENCE.items():
            for value, target in zip(measured[axle], reference):
                with self.subTest(axle=axle, target=target):
                    self.assertLess(abs(value - target) / target, 0.10, measured)

    def test_corner_damping_ratio_matches_an_independent_hand_calculation(self):
        baseline = self.results[5]
        self.assertAlmostEqual(
            baseline.front_corner_damping_ratio, hand_corner_damping_ratio(0, 5, 7), places=9
        )
        self.assertAlmostEqual(
            baseline.rear_corner_damping_ratio, hand_corner_damping_ratio(1, 5, 7), places=9
        )
        # Pinned hand values, so a change of definition or of the damper table fails here.
        self.assertAlmostEqual(baseline.front_corner_damping_ratio, 0.50970, delta=2e-3)
        self.assertAlmostEqual(baseline.rear_corner_damping_ratio, 0.43353, delta=2e-3)
        # A stronger compression click is a stiffer damper at the same velocity.
        stiffer = hand_corner_damping_ratio(0, 11, 7)
        self.assertGreater(stiffer, 3.5 * baseline.front_corner_damping_ratio)

    def test_ride_frequencies_and_modal_metrics_are_reported_per_setup(self):
        baseline = self.results[5]
        self.assertAlmostEqual(baseline.front_ride_frequency_hz, 4.2830, delta=0.01)
        self.assertAlmostEqual(baseline.rear_ride_frequency_hz, 3.5895, delta=0.01)
        self.assertTrue(3.4 < baseline.heave_frequency_hz < 4.7)
        self.assertTrue(7.0 < baseline.pitch_frequency_hz < 9.0)
        self.assertTrue(9.0 < baseline.roll_frequency_hz < 12.0)
        for value in (baseline.heave_damping_ratio, baseline.pitch_damping_ratio,
                      baseline.roll_damping_ratio):
            with self.subTest(value=value):
                self.assertTrue(0.0 < value < 1.0)
        # A damper click cannot move the springs or the tire rates, so the ride
        # frequencies are exactly invariant; the modal frequencies are not,
        # because the damper's zero-velocity slope enters the damping matrix.
        for result in self.results:
            self.assertAlmostEqual(result.front_ride_frequency_hz, baseline.front_ride_frequency_hz, places=9)
            self.assertAlmostEqual(result.rear_ride_frequency_hz, baseline.rear_ride_frequency_hz, places=9)
            self.assertTrue(3.4 < result.heave_frequency_hz < 6.0)
            self.assertTrue(7.0 < result.pitch_frequency_hz < 9.0)
            self.assertTrue(9.0 < result.roll_frequency_hz < 12.0)
        # A very stiff low-speed setting locks the hub towards the body and pushes
        # the coupled heave mode above the plan's 3.4-4.7 Hz band (a reported
        # finding, not a gate on a physical spring change).
        self.assertGreater(max(result.heave_frequency_hz for result in self.results), 4.7)

    def test_baseline_modal_pin(self):
        """Regression pin of the baseline linearized modes (measured 2026-10-16).

        The damped modal frequencies and ratios come from the eigenvalues of the
        linearized 14x14 first-order system; the Task 6 gates pin the *undamped*
        modes (3.83 / 7.30 / 9.90 Hz), which differ because this pin includes the
        damping matrix.
        """

        baseline = self.results[5]
        self.assertAlmostEqual(baseline.heave_frequency_hz, 3.98929, delta=1e-3)
        self.assertAlmostEqual(baseline.heave_damping_ratio, 0.18614, delta=1e-3)
        self.assertAlmostEqual(baseline.roll_frequency_hz, 10.19916, delta=1e-3)
        self.assertAlmostEqual(baseline.roll_damping_ratio, 0.18226, delta=1e-3)
        self.assertAlmostEqual(baseline.pitch_frequency_hz, 7.88933, delta=1e-3)
        self.assertAlmostEqual(baseline.pitch_damping_ratio, 0.20354, delta=1e-3)
        # The reported frequency is |lambda|/(2*pi), not Im(lambda)/(2*pi): the
        # damped frequency is pinned separately and must sit below it.
        self.assertAlmostEqual(baseline.heave_damped_frequency_hz, 3.91956, delta=1e-3)
        self.assertAlmostEqual(baseline.roll_damped_frequency_hz, 10.02833, delta=1e-3)
        self.assertAlmostEqual(baseline.pitch_damped_frequency_hz, 7.72418, delta=1e-3)
        for name in ("heave", "roll", "pitch"):
            with self.subTest(mode=name):
                natural = getattr(baseline, f"{name}_frequency_hz")
                damped = getattr(baseline, f"{name}_damped_frequency_hz")
                ratio = getattr(baseline, f"{name}_damping_ratio")
                self.assertGreater(damped, 0.0)
                self.assertLess(damped, natural)
                # For a complex pole lambda = -sigma + i*w_d:
                # zeta^2 + (Im(lambda)/|lambda|)^2 == 1 exactly.
                self.assertAlmostEqual(ratio**2 + (damped / natural) ** 2, 1.0, places=12)

    def test_step_metrics_respond_to_the_click_without_being_monotone(self):
        """The first-peak drop is not monotone in click; the net effect is."""

        peaks = [result.step_peak_m for result in self.results]
        # Net effect: the stiffest low-speed setting cuts the peak by more than 15 %.
        self.assertLess(peaks[11], 0.85 * peaks[0])
        self.assertLess(
            self.results[11].step_overshoot_fraction,
            0.2 * self.results[0].step_overshoot_fraction,
        )
        self.assertLess(self.results[11].step_settling_time_s, self.results[0].step_settling_time_s)
        # Clicks 0-7 move the first peak by under 2 % in a non-monotone way: the
        # step's first peak is heave/pitch dominated, so the bump damper barely
        # touches it before it becomes stiff enough to lock the hub.
        low = peaks[:8]
        self.assertLess(max(low) / min(low) - 1.0, 0.02, low)
        high = peaks[7:]
        self.assertTrue(all(b < a for a, b in zip(high, high[1:])), high)
        for result in self.results:
            self.assertGreater(result.step_peak_m, 0.0)
            self.assertGreater(result.step_steady_state_m, 0.0)
            self.assertGreater(result.step_time_to_peak_s, 0.0)
            self.assertLess(result.step_time_to_peak_s, 0.35)
            self.assertGreater(result.step_overshoot_fraction, 0.0)
            self.assertGreater(result.step_settling_time_s, result.step_time_to_peak_s)
            self.assertLess(result.step_settling_time_s, 1.2)

    def test_frequency_response_columns_are_empty_for_the_step_study(self):
        for result in self.results:
            self.assertIsNone(result.heave_peak_frequency_hz)
            self.assertIsNone(result.roll_peak_magnitude)

    def test_results_are_deterministic(self):
        repeat = sweep_vertical_setup(
            FORMULATION_XML, self.overrides[:3], "step-response"
        )
        for first, second in zip(self.results[:3], repeat):
            self.assertEqual(first, second)

    def test_sweep_is_fast_enough_for_a_twelve_setup_table(self):
        self.assertLess(self.elapsed, 5.0, f"{self.elapsed:.2f} s for 12 setups")


class StepOracleTests(unittest.TestCase):
    """The tool's linear step against the Task 7 plant oracle and its helpers."""

    @classmethod
    def setUpClass(cls):
        cls.result = sweep_vertical_setup(
            FORMULATION_XML, [{}], "step-response", step_samples=12001
        )[0]

    def test_linear_step_agrees_with_the_recorded_task_7_oracle(self):
        result = self.result
        self.assertLess(
            abs(result.step_peak_m - TASK7_LINEAR_ORACLE["peak_m"]) / TASK7_LINEAR_ORACLE["peak_m"],
            0.15,
            f"peak {result.step_peak_m * 1e3:.3f} mm",
        )
        self.assertLess(
            abs(result.step_time_to_peak_s - TASK7_LINEAR_ORACLE["time_s"]) / TASK7_LINEAR_ORACLE["time_s"],
            0.15,
            f"time to peak {result.step_time_to_peak_s:.4f} s",
        )
        self.assertLess(
            abs(result.step_overshoot_fraction - TASK7_LINEAR_ORACLE["overshoot"])
            / TASK7_LINEAR_ORACLE["overshoot"],
            0.15,
            f"overshoot {result.step_overshoot_fraction:.4f}",
        )
        self.assertLess(
            abs(result.step_steady_state_m - 0.025105) / 0.025105, 0.15,
            f"steady state {result.step_steady_state_m * 1e3:.3f} mm",
        )

    def test_linear_step_matches_the_task_7_seven_coordinate_oracle(self):
        """Recompute the Task 7 linear oracle from its own helpers and compare."""

        try:
            import scipy.integrate
        except ImportError:  # pragma: no cover - SciPy is present in the test env
            self.skipTest("SciPy is needed for the Task 7 linear-oracle helpers")
        from tests.test_vertical_forward import (  # the oracle this gate cross-validates
            closed_form_damping,
            closed_form_mass,
            closed_form_stiffness,
            compiled_vehicle as task7_vehicle,
            with_inertance,
        )
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant

        vehicle = with_inertance(task7_vehicle(), 0.5)
        plant = VerticalForwardPlant(
            vehicle, SpeedPlan(np.array([0.0, 100.0]), np.full(2, 15.0), np.zeros(2))
        )
        state = plant.initial_state(15.0)
        mechanical = state[3:24]
        vertical0 = mechanical[list(POSITION_ROWS)]
        positions = sprung_wheel_positions(vehicle)
        x_front, y_front = float(positions[0, 0]), float(positions[0, 1])
        force = 2000.0
        generalized = np.array([force, 0.0, -force * x_front, 0.0, 0.0, 0.0, 0.0])
        mass = closed_form_mass(vehicle)
        stiffness = closed_form_stiffness(vehicle, vertical0)
        damping = closed_form_damping(vehicle)
        times = np.linspace(0.0, 1.2, 12001)

        def rhs(_time, y):
            position, velocity = y[:7], y[7:]
            return np.r_[
                velocity,
                np.linalg.solve(mass, generalized - stiffness @ position - damping @ velocity),
            ]

        solution = scipy.integrate.solve_ivp(
            rhs, (0.0, 1.2), np.zeros(14), method="DOP853", rtol=1e-12, atol=1e-14,
            t_eval=times,
        )
        self.assertTrue(solution.success, solution.message)
        response = solution.y[0] + y_front * solution.y[1] - x_front * solution.y[2]
        peak = float(response[int(np.argmax(response))])
        time_to_peak = float(times[int(np.argmax(response))])
        steady = float(response[-1])
        overshoot = peak / steady - 1.0
        result = self.result
        self.assertLess(abs(result.step_peak_m / peak - 1.0), 0.01, (result.step_peak_m, peak))
        self.assertLess(abs(result.step_time_to_peak_s / time_to_peak - 1.0), 0.05)
        self.assertLess(abs(result.step_steady_state_m / steady - 1.0), 0.01)
        self.assertLess(abs(result.step_overshoot_fraction / overshoot - 1.0), 0.05)

    def test_settled_state_is_inside_the_declared_band(self):
        """Recompute the response tail and check the 2 % settling definition."""

        from tools.run.sweep_vertical_setup import step_response

        times, response = step_response(compiled_vehicle(), samples=12001)
        steady = float(response[-1])
        band = 0.02 * abs(steady)
        settled = self.result.step_settling_time_s
        index = int(np.searchsorted(times, settled))
        self.assertTrue(np.all(np.abs(response[index:] - steady) <= band))
        if index > 0:
            window = response[index - 1:]
            self.assertTrue(
                np.any(np.abs(window - steady) > band),
                "settling time is not the first crossing of the 2 % band",
            )


class SuspensionScalarOverrideTests(unittest.TestCase):
    """The compiled suspension scalars reach the linear model."""

    @classmethod
    def setUpClass(cls):
        cls.baseline, cls.inertance, cls.toe = sweep_vertical_setup(
            FORMULATION_XML,
            [{}, {"inertance_front": 3.0}, {"static_toe_front": 0.01}],
            "step-response",
        )

    def test_inertance_moves_the_mass_matrix_driven_modes(self):
        """3 kg on the front hub adds 3/MR^2 = 2.72 kg to the jounce diagonal."""

        baseline, loaded = self.baseline, self.inertance
        self.assertLess(loaded.heave_frequency_hz, baseline.heave_frequency_hz)
        self.assertLess(abs(loaded.heave_frequency_hz / baseline.heave_frequency_hz - 1.0), 0.01)
        self.assertGreater(abs(loaded.heave_frequency_hz / baseline.heave_frequency_hz - 1.0), 5e-4)
        self.assertGreater(
            abs(loaded.heave_damping_ratio / baseline.heave_damping_ratio - 1.0), 1e-3
        )
        # The springs and tires are untouched, so the ride frequency and the rear
        # corner damping ratio cannot move.
        self.assertAlmostEqual(
            loaded.front_ride_frequency_hz, baseline.front_ride_frequency_hz, places=9
        )
        self.assertAlmostEqual(
            loaded.rear_corner_damping_ratio, baseline.rear_corner_damping_ratio, places=9
        )

    def test_static_toe_is_accepted_and_inert_at_the_speed_zero_static_state(self):
        """Zero slip at the static state: a static toe offset changes no force."""

        self.assertEqual(self.toe.overrides["static_toe_front"], 0.01)
        for name in ("heave_frequency_hz", "roll_frequency_hz", "pitch_frequency_hz",
                     "heave_damping_ratio", "front_corner_damping_ratio",
                     "step_peak_m", "step_settling_time_s", "step_steady_state_m"):
            with self.subTest(metric=name):
                self.assertEqual(getattr(self.toe, name), getattr(self.baseline, name))


class FrequencyResponseTests(unittest.TestCase):
    """Displacement-per-force peaks and the ARB's roll-only signature."""

    @classmethod
    def setUpClass(cls):
        cls.baseline = sweep_vertical_setup(FORMULATION_XML, [{}], "frequency-response")[0]
        rear = compiled_vehicle().suspension.network.arb.rear
        cls.factors = (0.0, 1.0, 3.0, 5.0, 10.0)
        cls.arb_results = sweep_vertical_setup(
            FORMULATION_XML,
            [{"arb_wheel_rate_rear": factor * rear} for factor in cls.factors],
            "frequency-response",
        )

    def test_peaks_are_reported_with_units_and_sit_near_the_modal_frequencies(self):
        result = self.baseline
        for name, peak_hz, modal_hz in (
            ("heave", result.heave_peak_frequency_hz, result.heave_frequency_hz),
            ("roll", result.roll_peak_frequency_hz, result.roll_frequency_hz),
            ("pitch", result.pitch_peak_frequency_hz, result.pitch_frequency_hz),
        ):
            with self.subTest(mode=name):
                self.assertTrue(0.5 < peak_hz < 30.0)
                self.assertLess(abs(peak_hz - modal_hz) / modal_hz, 0.10, (peak_hz, modal_hz))
        for magnitude in (result.heave_peak_magnitude, result.roll_peak_magnitude,
                          result.pitch_peak_magnitude):
            with self.subTest(magnitude=magnitude):
                self.assertGreater(magnitude, 0.0)
        self.assertAlmostEqual(result.heave_peak_frequency_hz, 3.8433, delta=0.02)
        self.assertAlmostEqual(result.roll_peak_frequency_hz, 9.7138, delta=0.02)
        self.assertAlmostEqual(result.pitch_peak_frequency_hz, 7.4522, delta=0.02)

    def test_step_columns_are_empty_for_the_frequency_study(self):
        self.assertIsNone(self.baseline.step_peak_m)
        self.assertIsNone(self.baseline.step_settling_time_s)

    def test_arb_override_moves_roll_and_leaves_heave_and_pitch_alone(self):
        roll = [result.roll_peak_frequency_hz for result in self.arb_results]
        self.assertTrue(all(b > a for a, b in zip(roll, roll[1:])), roll)
        self.assertGreater(roll[-1] / roll[0] - 1.0, 0.01, roll)
        baseline = self.arb_results[1]
        for result in self.arb_results:
            self.assertAlmostEqual(
                result.heave_peak_frequency_hz, baseline.heave_peak_frequency_hz, places=9
            )
            self.assertAlmostEqual(
                result.heave_peak_magnitude / baseline.heave_peak_magnitude, 1.0, places=9
            )
            self.assertAlmostEqual(
                result.pitch_peak_frequency_hz, baseline.pitch_peak_frequency_hz, places=9
            )


class FourValveDamperTests(unittest.TestCase):
    """A tmp synthetic four-valve damper accepts four click overrides."""

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.formulation = _synthetic_four_valve_formulation(Path(cls.directory.name))
        cls.vehicle = compile_formulation(cls.formulation).model

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_click_names_come_from_the_four_valve_table(self):
        names = valid_override_names(self.vehicle)
        for valve in ("lsc", "hsc", "lsr", "hsr"):
            with self.subTest(valve=valve):
                self.assertIn(f"click_{valve}", names)
                self.assertIn(f"front.click_{valve}", names)
        self.assertNotIn("click_compression", names)
        self.assertIn("arb_wheel_rate_rear", names)

    def test_four_click_overrides_are_accepted(self):
        vehicle = apply_overrides(
            self.vehicle,
            {"click_lsc": 2, "click_hsc": 1, "click_lsr": 3, "click_hsr": 0},
        )
        self.assertEqual(
            dict(vehicle.suspension.network.dampers[0].setting.clicks),
            {"lsc": 2.0, "hsc": 1.0, "lsr": 3.0, "hsr": 0.0},
        )
        self.assertEqual(dict(vehicle.suspension.network.dampers[1].setting.clicks), dict(vehicle.suspension.network.dampers[0].setting.clicks))
        # A valve name that the table does not have is an unknown override here.
        with self.assertRaises(ValueError):
            apply_overrides(self.vehicle, {"click_compression": 5})

    def test_sweep_runs_on_the_synthetic_formulation_and_responds_to_all_four(self):
        setups = [
            {"click_lsc": 0, "click_hsc": 0, "click_lsr": 0, "click_hsr": 0},
            {"click_lsc": 3, "click_hsc": 2, "click_lsr": 3, "click_hsr": 2},
        ]
        results = sweep_vertical_setup(self.formulation, setups, "step-response")
        self.assertEqual(len(results), 2)
        soft, stiff = results
        self.assertGreater(stiff.front_corner_damping_ratio, soft.front_corner_damping_ratio)
        self.assertGreater(stiff.rear_corner_damping_ratio_rebound, soft.rear_corner_damping_ratio_rebound)
        self.assertLess(stiff.step_peak_m, soft.step_peak_m)
        # The high-speed compression valve alone changes the corner damping ratio.
        high_only = sweep_vertical_setup(
            self.formulation, [{"click_lsc": 0, "click_hsc": 2}], "step-response"
        )[0]
        low_only = sweep_vertical_setup(
            self.formulation, [{"click_lsc": 0, "click_hsc": 0}], "step-response"
        )[0]
        self.assertGreater(high_only.front_corner_damping_ratio, low_only.front_corner_damping_ratio)


class OutputTests(unittest.TestCase):
    """The CSV and the JSON summary round-trip with the returned results."""

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.path = Path(cls.directory.name)
        cls.overrides = [{"click_compression": 3, "click_rebound": 7},
                         {"click_compression": 5, "click_rebound": 7}]
        cls.results = sweep_vertical_setup(
            FORMULATION_XML,
            cls.overrides,
            "frequency-response",
            output_csv=cls.path / "sweep.csv",
            output_json=cls.path / "sweep.json",
        )

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_csv_has_one_row_per_setup_and_every_metric_column(self):
        with (self.path / "sweep.csv").open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 2)
        for column in ("click_compression", "click_rebound", "study",
                       "front_ride_frequency_hz", "heave_frequency_hz",
                       "heave_damped_frequency_hz", "front_corner_damping_ratio",
                       "heave_peak_magnitude", "heave_peak_frequency_hz",
                       "step_peak_m"):
            with self.subTest(column=column):
                self.assertIn(column, rows[0])
        for row, result in zip(rows, self.results):
            self.assertEqual(float(row["click_compression"]), result.overrides["click_compression"])
            self.assertEqual(float(row["heave_peak_frequency_hz"]), result.heave_peak_frequency_hz)
            self.assertEqual(row["step_peak_m"], "")

    def test_json_summary_holds_inputs_results_hashes_and_study_definition(self):
        document = json.loads((self.path / "sweep.json").read_text())
        self.assertEqual(document["study"], "frequency-response")
        self.assertIn("tool_version", document)
        self.assertEqual(len(document["results"]), 2)
        self.assertEqual(document["overrides"], self.overrides)
        for key in ("study_definition", "source_hashes", "formulation"):
            with self.subTest(key=key):
                self.assertIn(key, document)
        definition = document["study_definition"]
        self.assertEqual(definition["frequency_min_hz"], 0.5)
        self.assertEqual(definition["frequency_max_hz"], 30.0)
        self.assertIn("observable", definition)
        # The two convention disclosures travel with every summary.
        self.assertIn("natural", definition["modal_frequency_convention"])
        self.assertIn("compression-branch", definition["damping_branch_convention"])
        hashes = document["source_hashes"]
        expected = hashlib.sha256(FORMULATION_XML.read_bytes()).hexdigest()
        self.assertEqual(hashes[str(FORMULATION_XML)], expected)
        self.assertTrue(
            any(path.endswith("multimatic_dssv_vc01.npz") for path in hashes), sorted(hashes)
        )
        for tail in ("sweep_vertical_setup.py", "tires/unitire_transient.py",
                     "compiler/compile.py"):
            with self.subTest(source=tail):
                self.assertTrue(
                    any(path.endswith(tail) for path in hashes), sorted(hashes)
                )
        for row, result in zip(document["results"], self.results):
            self.assertEqual(row["overrides"]["click_compression"], result.overrides["click_compression"])
            self.assertEqual(row["heave_frequency_hz"], result.heave_frequency_hz)
            self.assertIsNone(row["step_peak_m"])

    def test_write_csv_and_write_json_round_trip_after_the_fact(self):
        csv_path = self.path / "again.csv"
        json_path = self.path / "again.json"
        write_csv(self.results, csv_path)
        write_json(
            self.results,
            json_path,
            formulation_xml=FORMULATION_XML,
            study="frequency-response",
            study_definition={"observable": "test"},
            source_hashes={"x": "y"},
        )
        document = json.loads(json_path.read_text())
        self.assertEqual(document["study"], "frequency-response")
        self.assertEqual(document["study_definition"], {"observable": "test"})
        with csv_path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), len(self.results))

    def test_default_formulation_is_the_14dof_simple_vehicle(self):
        self.assertEqual(Path(DEFAULT_FORMULATION).name, FORMULATION_XML.name)
        self.assertEqual(Path(DEFAULT_FORMULATION).resolve(), FORMULATION_XML.resolve())

    def test_cli_writes_the_requested_files_and_the_grid_columns(self):
        from tools.run.sweep_vertical_setup import main

        csv_path = self.path / "cli.csv"
        json_path = self.path / "cli.json"
        code = main([
            "--formulation", str(FORMULATION_XML),
            "--grid", json.dumps({"click_compression": [0, 6, 11], "click_rebound": [7]}),
            "--study", "step-response",
            "--output-csv", str(csv_path),
            "--output-json", str(json_path),
        ])
        self.assertEqual(code, 0)
        with csv_path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 3)
        self.assertEqual([row["click_compression"] for row in rows], ["0.0", "6.0", "11.0"])
        document = json.loads(json_path.read_text())
        self.assertEqual(len(document["results"]), 3)
        self.assertEqual(document["study_definition"]["step_force_n"], 2000.0)

    def test_cli_reports_an_unknown_override_and_rejects_conflicting_sources(self):
        from tools.run.sweep_vertical_setup import main

        code = main([
            "--formulation", str(FORMULATION_XML),
            "--overrides-json", json.dumps([{"nonsense": 1}]),
            "--study", "step-response",
        ])
        self.assertNotEqual(code, 0)
        # argparse rejects --overrides-json together with --grid and an unknown study.
        with self.assertRaises(SystemExit):
            main(["--formulation", str(FORMULATION_XML), "--study", "step-response",
                  "--overrides-json", "[]", "--grid", "{}"])
        with self.assertRaises(SystemExit):
            main(["--formulation", str(FORMULATION_XML), "--study", "bogus"])


if __name__ == "__main__":
    unittest.main()
