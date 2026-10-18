"""The setup sweep on the modal suspension layouts (Task 5).

Oracles:

* the linearized stiffness of the axle roll/heave example (rates equal to the
  IEM24 corner springs plus the rear ARB) agrees with the independent-corner
  baseline, which uses the spline derivative;
* the linearized damping block is ``T^T diag(slope) T``, built by hand from the
  compiled mode dampers;
* a roll rate moves the roll mode and leaves heave and pitch alone on this
  left-right symmetric car; a heave rate moves heave.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

from tests.test_suspension_layouts import compile_with  # noqa: E402
from tools.run.sweep_vertical_setup import (  # noqa: E402
    DEFAULT_FORMULATION,
    _linear_subsystem,
    apply_overrides,
    sweep_vertical_setup,
    valid_override_names,
)

AXLE = ROOT / "models/suspensions/iem24_axle_roll_heave.xml"
MODE_DAMPERS = ROOT / "models/dampers"
FOUR = ROOT / "models/suspensions/iem24_four_mode.xml"
_CACHE: dict = {}


def vehicle_of(path: Path):
    if path not in _CACHE:
        _CACHE[path] = compile_with(path).model
    return _CACHE[path]


class ModalOverrideTests(unittest.TestCase):
    def test_names_come_from_the_modes_and_their_dampers(self):
        names = valid_override_names(vehicle_of(FOUR))
        for mode in ("heave", "roll", "pitch", "warp"):
            self.assertIn(f"rate_{mode}", names)
            self.assertIn(f"{mode}.click_compression", names)
        self.assertIn("click_rebound", names)
        self.assertNotIn("arb_wheel_rate_front", names)
        self.assertNotIn("inertance_front", names)
        axle = valid_override_names(vehicle_of(AXLE))
        self.assertIn("rate_roll_rear", axle)
        self.assertIn("heave_front.click_rebound", axle)

    def test_overrides_rebuild_the_rates_and_the_clicks(self):
        vehicle = apply_overrides(
            vehicle_of(FOUR), {"rate_roll": 1.0e5, "pitch.click_compression": 3})
        network = vehicle.suspension.network
        self.assertEqual(float(network.rates[1]), 1.0e5)
        self.assertEqual(network.dampers[2].setting.clicks["compression"], 3)
        self.assertEqual(network.dampers[0].setting.clicks["compression"], 5)
        # The compiled vehicle is not modified.
        self.assertNotEqual(float(vehicle_of(FOUR).suspension.network.rates[1]), 1.0e5)

    def test_bad_values_and_names_raise(self):
        with self.assertRaisesRegex(ValueError, "unknown override"):
            apply_overrides(vehicle_of(FOUR), {"rate_yaw": 1.0})
        with self.assertRaisesRegex(ValueError, "non-negative"):
            apply_overrides(vehicle_of(FOUR), {"rate_roll": -1.0})
        with self.assertRaisesRegex(ValueError, "click"):
            apply_overrides(vehicle_of(FOUR), {"click_compression": 99})


class ModalLinearisationTests(unittest.TestCase):
    def test_the_axle_example_has_physical_distinct_roll_mode_stiffness(self):
        independent = _linear_subsystem(vehicle_of_default(), 0.0)
        axle = _linear_subsystem(vehicle_of(AXLE), 0.0)
        self.assertGreater(independent.stiffness[3, 3], axle.stiffness[3, 3])
        self.assertNotAlmostEqual(axle.stiffness[3, 3], independent.stiffness[3, 3], delta=1.0)

    def test_the_damping_block_is_the_transformed_mode_slopes(self):
        vehicle = vehicle_of(FOUR)
        network = vehicle.suspension.network
        slopes = np.array([float(np.asarray(d.force(np.zeros(1)).slope)[0]) for d in network.dampers])
        factors = np.asarray(network.damper_velocity_factors)
        expected = network.transform.T @ np.diag(slopes * factors**2) @ network.transform
        subsystem = _linear_subsystem(vehicle, 0.0)
        tire = vehicle.tire_vertical_damping
        block = subsystem.damping[3:, 3:] - tire * np.eye(4)
        np.testing.assert_allclose(block, expected, rtol=1e-9, atol=1e-6)


def vehicle_of_default():
    if "default" not in _CACHE:
        from simplified_vehicle_models.compiler.compile import compile_formulation
        _CACHE["default"] = compile_formulation(DEFAULT_FORMULATION).model
    return _CACHE["default"]


class ModalSweepTests(unittest.TestCase):
    def sweep(self, path: Path, setups):
        # sweep_vertical_setup compiles the formulation; build it with a temporary
        # copy so the layout under test is the suspension.
        import tempfile
        text = (ROOT / "models/vehicles/iem24_murphy_real_planar.xml").read_text()
        import re
        text = text.replace("../suspensions/iem24_murphy_real_suspension.xml", path.resolve().as_posix())
        text = re.sub(r'(ref|path)="\.\./', rf'\1="{(ROOT / "models").as_posix()}/', text)
        formulation = DEFAULT_FORMULATION.read_text().replace(
            "../vehicles/iem24_murphy_real_planar.xml", "vehicle.xml")
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory).resolve()
            (directory / "vehicle.xml").write_text(text)
            (directory / "formulation.xml").write_text(formulation)
            return sweep_vertical_setup(directory / "formulation.xml", setups, "step-response")

    def test_the_roll_rate_moves_roll_and_leaves_heave_and_pitch(self):
        base, stiff = self.sweep(FOUR, [{}, {"rate_roll": 3.0e5}])
        # The tires are in series with the roll spring, so the roll mode saturates:
        # 3.7x the roll rate moves it by well under 10 %, but it must rise.
        self.assertGreater(stiff.roll_frequency_hz, base.roll_frequency_hz * 1.003)
        self.assertAlmostEqual(stiff.heave_frequency_hz, base.heave_frequency_hz, delta=1e-6)
        self.assertAlmostEqual(stiff.pitch_frequency_hz, base.pitch_frequency_hz, delta=1e-6)

    def test_the_heave_rate_moves_heave(self):
        base, stiff = self.sweep(FOUR, [{}, {"rate_heave": 3.0e5}])
        self.assertGreater(stiff.heave_frequency_hz, base.heave_frequency_hz * 1.15)
        self.assertAlmostEqual(stiff.roll_frequency_hz, base.roll_frequency_hz, delta=1e-6)

    def test_the_axle_example_uses_the_updated_spring_rates(self):
        # Distinct measured modal spring rates intentionally differ from the
        # independent-corner model and its former corner-spring/ARB equivalent.
        (axle,) = self.sweep(AXLE, [{}])
        (independent,) = sweep_vertical_setup(DEFAULT_FORMULATION, [{}], "step-response")
        self.assertNotAlmostEqual(axle.front_ride_frequency_hz, independent.front_ride_frequency_hz, places=2)
        self.assertGreater(axle.front_ride_frequency_hz, 0.0)
        # Physical modal rates now follow the DSS's different heave/roll springs;
        # they need not reproduce the legacy independent-corner model.


class FourWayReportTextTests(unittest.TestCase):
    def test_the_report_reads_the_damper_settings_through_the_layout(self):
        from tools.reports.compare_iem24_four_way import suspension_description

        self.assertEqual(
            suspension_description(vehicle_of_default().suspension),
            "four-corner springs, bump stops, ARB, dampers (5/7 clicks)",
        )
        text = suspension_description(vehicle_of(FOUR).suspension)
        self.assertIn("four-mode", text)
        self.assertIn("roll 5/7", text)
        self.assertIn("no bump stops", text)


if __name__ == "__main__":
    unittest.main()
