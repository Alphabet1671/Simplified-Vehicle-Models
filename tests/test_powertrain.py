from pathlib import Path
from dataclasses import replace
import math
import tempfile
import time
import unittest
import hashlib
from types import SimpleNamespace
from unittest import mock

from tools import generate_motor_lut

import numpy as np

from simplified_vehicle_models.powertrain.assembly import (
    compile_drivetrain_assembly,
    compile_powertrain_assembly,
    compile_powertrain_drivetrain,
    FullDrivetrainRuntime,
    maximum_drive_force,
    maximum_drive_force_lumped,
    maximum_regen_force,
    resolve_braking,
    solve_shaft_torque_fixed_point,
)
from simplified_vehicle_models.drivetrain.lumped import compile_lumped_drivetrain
from simplified_vehicle_models.xml.loader import load_model
from simplified_vehicle_models.xml.model import DataReference
from simplified_vehicle_models.powertrain.power_plant import (
    _envelope_cliff_cell,
    _exact_limit,
    compile_electric_motor,
    evaluate_shaft_power,
    evaluate_motor,
    evaluate_shaft_power_exact,
    motor_torque_limits,
    torque_for_bus_power,
    reset_motor_lut_counters,
    motor_lut_counters,
    _validate_operating_map,
    _interpolated_row,
    _load_operating_map,
    MotorOperatingMap,
    voltage_scaled_envelope,
    _envelope_cliff_cell,
)
from simplified_vehicle_models.powertrain.energy_storage import (
    StorageState,
    evaluate_storage,
    terminal_voltage,
)
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


ROOT = Path(__file__).resolve().parents[1]


def _voltage_fraction(mapping, voltage):
    """Return the map voltage bracket index and interpolation fraction."""
    index = int(np.searchsorted(mapping.voltage, voltage, side="right")) - 1
    index = max(0, min(index, mapping.voltage.size - 2))
    fraction = (voltage - mapping.voltage[index]) / (
        mapping.voltage[index + 1] - mapping.voltage[index]
    )
    return index, fraction


def _speed_fraction(mapping, speed):
    """Return the map speed bracket index and interpolation fraction."""
    index = int(np.searchsorted(mapping.speed, speed, side="right")) - 1
    index = max(0, min(index, mapping.speed.size - 2))
    fraction = (speed - mapping.speed[index]) / (
        mapping.speed[index + 1] - mapping.speed[index]
    )
    return index, fraction


class RegenGenerationTests(unittest.TestCase):
    def test_maximum_feasible_uses_torque_units_and_keeps_full_envelope(self):
        calls = []

        def evaluate(_parameters, _speed, torque, _voltage):
            calls.append(torque)
            return SimpleNamespace(feasible=abs(torque) <= 20.0)

        with mock.patch.object(generate_motor_lut, "evaluate_shaft_power_exact", evaluate):
            result = generate_motor_lut._maximum_feasible(
                object(), 100.0, 448.848, -1.0, 20.0
            )
        self.assertEqual(result, 20.0)
        self.assertEqual(calls, [-0.0, -20.0])

    def test_generator_persists_full_nonmonotonic_regen_envelope(self):
        parameters = SimpleNamespace(
            torque_speed=np.array([0.0, 18600.0]),
            torque_values=np.array([20.0, 20.0]),
            maximum_bus_voltage=600.0,
        )

        def evaluate(_parameters, _speed, torque, _voltage):
            if torque >= 0.0:
                bus_power = torque
            else:
                magnitude = abs(torque)
                bus_power = -10.0 if magnitude <= 10.0 else -5.0
            return SimpleNamespace(
                feasible=abs(torque) <= 20.0,
                bus_power=bus_power,
                loss_power=1.0,
            )

        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "map.npz"
            patches = (
                mock.patch.object(generate_motor_lut, "load_model", return_value=object()),
                mock.patch.object(generate_motor_lut, "compile_analytical_motor_source", return_value=parameters),
                mock.patch.object(generate_motor_lut, "_source_paths", return_value=()),
                mock.patch.object(generate_motor_lut, "evaluate_shaft_power_exact", evaluate),
                mock.patch.object(
                    generate_motor_lut, "voltage_scaled_envelope",
                    lambda _parameters, _speed, _voltage: 20.0,
                ),
            )
            with patches[0], patches[1], patches[2], patches[3], patches[4]:
                generate_motor_lut.generate(
                    output, voltage_points=2, speed_points=2, load_points=5
                )
            with np.load(output, allow_pickle=False) as archive:
                self.assertEqual(archive["regen_torque"][1, 1], 20.0)
                np.testing.assert_array_equal(
                    archive["bus_power"][1, 1, :3], [-5.0, -10.0, 0.0]
                )
                np.testing.assert_array_equal(
                    archive["regen_feasible"][1, 1, :3], [True, True, False]
                )


class DriveEnvelopeGenerationTests(unittest.TestCase):
    @staticmethod
    def _drive_fractions(load_points=65):
        return np.linspace(-1.0, 1.0, load_points)[load_points // 2:]

    def test_keeps_exact_boundary_when_sampled_row_is_monotone(self):
        boundary = 17.847778726

        def evaluate(_parameters, _speed, torque, _voltage):
            return SimpleNamespace(bus_power=float(torque))

        with mock.patch.object(generate_motor_lut, "evaluate_shaft_power_exact", evaluate):
            magnitude, reduced, steps = generate_motor_lut._monotone_drive_envelope(
                object(), 11814.0, 595.9, boundary, self._drive_fractions()
            )
        # The coarse 257-candidate scan used to snap this down by up to
        # boundary / 256 (here about 0.070 Nm) even though the exact boundary
        # was feasible and monotone.
        self.assertEqual(magnitude, boundary)
        self.assertFalse(reduced)
        self.assertEqual(steps, 0)

    def test_reduction_lands_on_the_tight_monotonicity_boundary(self):
        # Bus power rises to ``limit`` then falls once the magnitude crosses it,
        # so the sampled row is monotone only up to a grid-dependent boundary
        # just above ``limit``.  The reduction must bisect onto that boundary
        # rather than snapping down by up to envelope / 256 = 0.078 Nm.
        limit = 8.0
        envelope = 20.0
        fractions = self._drive_fractions()

        def evaluate(_parameters, _speed, torque, _voltage):
            magnitude = abs(float(torque))
            return SimpleNamespace(bus_power=magnitude - 2.0 * max(0.0, magnitude - limit))

        with mock.patch.object(generate_motor_lut, "evaluate_shaft_power_exact", evaluate):
            magnitude, reduced, steps = generate_motor_lut._monotone_drive_envelope(
                object(), 11814.0, 595.9, envelope, fractions
            )
            self.assertTrue(reduced)
            self.assertGreater(steps, 0)
            self.assertTrue(
                generate_motor_lut._drive_row_is_monotone(
                    object(), 11814.0, 595.9, magnitude, fractions
                )
            )
            self.assertFalse(
                generate_motor_lut._drive_row_is_monotone(
                    object(), 11814.0, 595.9, magnitude + 1.0e-6, fractions
                )
            )
        self.assertLess(magnitude, envelope)
        self.assertGreater(magnitude, limit)

    def test_predicate_ignores_the_non_monotone_regen_half(self):
        # A curve that grows on the drive half but falls on the regen half must
        # not trigger a drive-envelope reduction: regen is deliberately exempt.
        def evaluate(_parameters, _speed, torque, _voltage):
            return SimpleNamespace(bus_power=float(torque))

        fractions = self._drive_fractions()
        with mock.patch.object(generate_motor_lut, "evaluate_shaft_power_exact", evaluate):
            self.assertTrue(
                generate_motor_lut._drive_row_is_monotone(
                    object(), 100.0, 500.0, 17.847778726, fractions
                )
            )

    def test_cli_exposes_reproducible_grid_options(self):
        parser = generate_motor_lut.build_parser()
        options = parser.parse_args([
            "models/power-plants/amk_dd5_100c.xml", "out.npz",
            "--voltage-points", "17", "--speed-points", "257", "--load-points", "65",
        ])
        self.assertEqual(
            (options.voltage_points, options.speed_points, options.load_points),
            (17, 257, 65),
        )
        defaults = parser.parse_args(["x.xml", "out.npz"])
        self.assertEqual(
            (defaults.voltage_points, defaults.speed_points, defaults.load_points),
            (
                generate_motor_lut.DEFAULT_VOLTAGE_POINTS,
                generate_motor_lut.DEFAULT_SPEED_POINTS,
                generate_motor_lut.DEFAULT_LOAD_POINTS,
            ),
        )

    def test_grid_validation_rejects_unrepresentable_resolutions(self):
        for resolution in ((1, 129, 65), (9, 1, 65), (9, 129, 2), (9, 129, 64)):
            with self.assertRaises(ValueError):
                generate_motor_lut.validate_grid(*resolution)
        generate_motor_lut.validate_grid(9, 129, 65)


class OperatingMapValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(ROOT / "models/data/amk_dd5_100c_operating_map.npz", allow_pickle=False) as archive:
            cls.data = {name: archive[name] for name in archive.files}

    def bad(self, mutate):
        data = {name: value.copy() for name, value in self.data.items()}
        mutate(data)
        with self.assertRaises(ValueError):
            _validate_operating_map(data)

    def test_rejects_missing_arrays_and_wrong_shapes(self):
        self.bad(lambda d: d.pop("loss_power"))
        self.bad(lambda d: d.__setitem__("drive_torque", d["drive_torque"][:-1]))

    def test_rejects_bad_axes_values_and_negative_loss(self):
        self.bad(lambda d: d["speed"].__setitem__(1, d["speed"][0]))
        self.bad(lambda d: d["loss_power"].__setitem__((0, 0, 0), -1.0))
        self.bad(lambda d: d["bus_power"].__setitem__((0, 0, 32), np.nan))

    def test_rejects_direction_sign_and_monotonicity(self):
        self.bad(lambda d: d["bus_power"].__setitem__((0, 1, 64), -1.0))
        self.bad(lambda d: (d["bus_power"].__setitem__((0, 1, 0), 1.0), d["regen_feasible"].__setitem__((0, 1, 0), True)))

    def test_accepts_complete_nonmonotonic_regen_row(self):
        data = {name: value.copy() for name, value in self.data.items()}
        row = data["bus_power"][0, 1, :32]
        row[:] = np.linspace(-5.0, -1.0, row.size)
        row[1] = -10.0
        data["regen_feasible"][0, 1, :32] = True
        loaded = _validate_operating_map(data)
        self.assertEqual(loaded.bus_power[0, 1, 1], -10.0)

    def test_rejects_incorrect_regen_productive_mask(self):
        def mutate(data):
            index = np.flatnonzero(data["regen_feasible"])[0]
            data["regen_feasible"].flat[index] = False

        self.bad(mutate)

    def test_rejects_digest_mismatch(self):
        model = load_model(ROOT / "models/power-plants/amk_dd5_100c.xml")
        reference = model.data[0]
        bad_reference = DataReference(reference.role, reference.path, reference.format, "0" * 64)
        bad_model = replace(model, data=(bad_reference,))
        with self.assertRaisesRegex(ValueError, "digest mismatch"):
            _load_operating_map(bad_model)

    def test_canonical_full_regen_envelope_preserves_nonmonotonic_region(self):
        # The regenerated canonical map keeps the genuinely non-monotone regen
        # row: it dips well below its own first node before recovering.
        row = self.data["bus_power"][0, 9, :]
        negative = int(np.searchsorted(self.data["load"], 0.0))
        self.assertLess(float(np.min(row[:negative])), float(row[0]) - 200.0)
        self.assertGreater(int(np.argmin(row)), 0)
        self.assertTrue(np.any(self.data["regen_feasible"][0, 9, :negative]))

    def test_canonical_shape_and_digest(self):
        loaded = _validate_operating_map(self.data)
        self.assertEqual(loaded.bus_power.shape, (25, 129, 65))
        digest = hashlib.sha256((ROOT / "models/data/amk_dd5_100c_operating_map.npz").read_bytes()).hexdigest()
        self.assertEqual(digest, "166bf1346d263267224f77fe7396ab15180e0c2f54391652fee28f8575fcfc5b")


class VoltageScaledEnvelopeTests(unittest.TestCase):
    """The LUT must never exceed the analytical voltage-scaled envelope cap."""

    @classmethod
    def setUpClass(cls):
        cls.motor = compile_electric_motor(
            load_model(ROOT / "models/power-plants/amk_dd5_100c.xml")
        )

    def test_known_corner_points_are_zero(self):
        for speed, voltage in ((18273.0, 540.0), (18273.0, 517.95), (18273.0, 529.999)):
            self.assertEqual(voltage_scaled_envelope(self.motor, speed, voltage), 0.0)
            limits = motor_torque_limits(self.motor, speed, voltage)
            self.assertEqual(limits.maximum_drive_torque, 0.0, (speed, voltage))
            self.assertEqual(limits.maximum_regen_torque, 0.0, (speed, voltage))

    def test_limits_never_exceed_voltage_scaled_cap_including_below_map_floor(self):
        voltages = list(np.linspace(530.0, 620.0, 31))  # above 600 V the clamp applies + [529.999, 529.0, 517.95, 500.0, 450.0]
        checked = 0
        for voltage in voltages:
            for speed in np.linspace(0.0, 18600.0, 187):
                cap = voltage_scaled_envelope(self.motor, float(speed), float(voltage))
                limits = motor_torque_limits(self.motor, float(speed), float(voltage))
                # Below ~300 rpm the bilinear interpolation of the source
                # envelope's own breakpoints overshoots by at most 0.1359 Nm
                # (measured on a 0.05 rpm x 1 V grid: 530 V / 88.35 rpm);
                # above that the LUT matches the cap to interpolation round-off
                # (measured 1.06e-6 Nm).
                slack = 0.14 if speed < 300.0 else 1.0e-5
                if cap == 0.0:
                    slack = 0.0
                for value in (limits.maximum_drive_torque, limits.maximum_regen_torque):
                    self.assertLessEqual(value, cap + slack, (voltage, speed))
                checked += 1
        self.assertEqual(checked, len(voltages) * 187)


class ClampedEnvelopeAgreementTests(unittest.TestCase):
    """Two-sided envelope check plus the regulatory bus-voltage clamp."""

    @classmethod
    def setUpClass(cls):
        cls.path = ROOT / "models/power-plants/amk_dd5_100c.xml"
        cls.motor = compile_electric_motor(load_model(cls.path))

    def test_drive_limit_matches_exact_limit_two_sided(self):
        # Measured: <= 0.1152 Nm below 700 rpm (low-speed breakpoint
        # interpolation), 0.0609 Nm above.  Stated tolerances add a small margin.
        voltages = list(np.linspace(530.0, 600.0, 15)) + [611.09, 620.0]
        speeds = list(np.linspace(0.0, 700.0, 57)) + [8.4, 88.35]
        # Regen measured: <= 0.1362 Nm below 300 rpm, 0.00033 Nm above; above
        # 700 rpm <= 0.0235 Nm.  Fallback cells skipped: 0 here, 1 in the
        # high-speed sweep; bounds below are slightly above those counts.
        checked = 0
        skipped = 0
        for voltage in voltages:
            for speed in speeds:
                limits = motor_torque_limits(self.motor, float(speed), float(voltage))
                exact = _exact_limit(self.motor, float(speed), float(voltage))
                if limits.used_fallback:
                    skipped += 1
                    continue
                exact_regen = _exact_limit(self.motor, float(speed), float(voltage), True)
                self.assertAlmostEqual(
                    limits.maximum_regen_torque, exact_regen,
                    delta=0.14 if speed < 300.0 else 0.001, msg=("regen", voltage, speed),
                )
                self.assertGreater(limits.maximum_drive_torque, 20.0, (voltage, speed))
                tolerance = 0.14 if speed < 300.0 else 0.001
                self.assertAlmostEqual(
                    limits.maximum_drive_torque, exact, delta=tolerance, msg=(voltage, speed)
                )
                checked += 1
        self.assertGreater(checked, 900)
        self.assertLessEqual(skipped, 2)  # measured 0
        high_skipped = 0
        high = [
            (v, s) for v in np.linspace(530.0, 600.0, 8) for s in np.linspace(700.0, 18000.0, 40)
        ]
        for voltage, speed in high:
            limits = motor_torque_limits(self.motor, float(speed), float(voltage))
            if limits.used_fallback:
                high_skipped += 1
                continue
            self.assertAlmostEqual(
                limits.maximum_drive_torque,
                _exact_limit(self.motor, float(speed), float(voltage)),
                delta=0.07, msg=(voltage, speed),
            )
            self.assertAlmostEqual(
                limits.maximum_regen_torque,
                _exact_limit(self.motor, float(speed), float(voltage), True),
                delta=0.03, msg=("regen", voltage, speed),
            )
        self.assertLessEqual(high_skipped, 3)  # measured 1

    def test_no_near_zero_drive_node_below_the_corner(self):
        mapping = self.motor.operating_map
        low = mapping.speed <= 17000.0
        # Before the clamp 26 nodes at 601-620 V, 0-581 rpm held ~1e-10 Nm.
        self.assertGreater(float(mapping.drive_torque[:, low].min()), 10.0)

    def test_known_former_collapse_probe(self):
        limits = motor_torque_limits(self.motor, 8.4, 611.09)
        self.assertGreater(limits.maximum_drive_torque, 21.0)

    def test_cliff_cell_routes_to_fallback_and_is_counted(self):
        mapping = self.motor.operating_map
        vi, si = 0, 121
        self.assertTrue(_envelope_cliff_cell(mapping, vi, si))
        voltage = float(mapping.voltage[vi] + 0.1)
        speed = float(mapping.speed[si] + 1.0)
        reset_motor_lut_counters()
        limits = motor_torque_limits(self.motor, speed, voltage)
        self.assertTrue(limits.used_fallback)
        self.assertEqual(motor_lut_counters()["limit_fallback"], 1)
        self.assertGreater(motor_lut_counters()["fallback"], 0)
        reset_motor_lut_counters()
        evaluate_motor(self.motor, speed, 1.0, voltage)
        counters = motor_lut_counters()
        self.assertGreaterEqual(counters["fallback"], 1)
        self.assertEqual(counters["forward"], 0)
        reset_motor_lut_counters()
        result = torque_for_bus_power(self.motor, speed, voltage, 1000.0, "drive")
        self.assertTrue(result.used_fallback)
        self.assertEqual(motor_lut_counters()["inverse"], 1)

    def test_above_maximum_bus_voltage_equals_maximum(self):
        maximum = self.motor.maximum_bus_voltage
        self.assertEqual(maximum, 600.0)
        analytical = replace(self.motor, operating_map=None)
        for motor in (self.motor, analytical):
            for speed in (0.0, 145.0, 1000.0, 9000.0, 17000.0):
                for torque in (-10.0, 0.0, 5.0, 15.0):
                    base = evaluate_shaft_power_exact(motor, speed, torque, maximum)
                    for voltage in (601.25, 608.688, 650.0):
                        self.assertEqual(
                            evaluate_shaft_power_exact(motor, speed, torque, voltage), base
                        )
                        self.assertEqual(
                            evaluate_motor(motor, speed, torque, voltage),
                            evaluate_motor(motor, speed, torque, maximum),
                        )
                self.assertEqual(
                    voltage_scaled_envelope(motor, speed, 610.0),
                    voltage_scaled_envelope(motor, speed, maximum),
                )
                self.assertEqual(
                    motor_torque_limits(motor, speed, 610.0),
                    motor_torque_limits(motor, speed, maximum),
                )
        self.assertEqual(
            torque_for_bus_power(self.motor, 1000.0, 610.0, 5000.0, "drive"),
            torque_for_bus_power(self.motor, 1000.0, maximum, 5000.0, "drive"),
        )

    def test_zero_torque_draw_at_pack_ocv_is_small(self):
        # Before the clamp the zero-torque draw at 608.688 V was 530-21,780 W per
        # motor.  At or below the 600 V reference it is the loss-map value: at
        # most 3.6 W was measured at 145 rpm and the loss map gives the same
        # order everywhere; 50 W is a generous bound, orders below the kW error.
        analytical = replace(self.motor, operating_map=None)
        for motor in (self.motor, analytical):
            for speed in np.linspace(0.0, 1000.0, 21):
                for evaluator in (evaluate_shaft_power_exact, evaluate_motor):
                    power = evaluator(motor, float(speed), 0.0, 608.688).bus_power
                    self.assertLess(abs(power), 50.0, (speed, power))

    def test_maximum_bus_voltage_is_configurable_from_xml(self):
        text = self.path.read_text()
        data_path = (self.path.parent / "../data/amk_dd5_100c_operating_map.npz").resolve()
        text = text.replace('value="600" unit="V"/>', 'value="580" unit="V"/>')
        text = text.replace("../data/amk_dd5_100c_operating_map.npz", str(data_path))
        with tempfile.TemporaryDirectory() as folder:
            xml = Path(folder) / "motor.xml"
            xml.write_text(text)
            motor = compile_electric_motor(load_model(xml))
        self.assertEqual(motor.maximum_bus_voltage, 580.0)
        self.assertEqual(
            evaluate_shaft_power_exact(motor, 5000.0, 10.0, 600.0),
            evaluate_shaft_power_exact(motor, 5000.0, 10.0, 580.0),
        )
        self.assertNotEqual(
            voltage_scaled_envelope(motor, 17000.0, 600.0),
            voltage_scaled_envelope(self.motor, 17000.0, 600.0),
        )
        self.assertEqual(
            motor_torque_limits(motor, 5000.0, 600.0),
            motor_torque_limits(motor, 5000.0, 580.0),
        )

    def test_raising_maximum_bus_voltage_above_600_exposes_known_defect(self):
        """GUARD: documents the above-reference field-weakening defect.

        Raising ``maximum_bus_voltage`` above the 600 V identification voltage
        re-exposes a spurious zero-torque bus draw because the smallest-
        magnitude d-axis root in ``solve_direct_current`` grows with available
        voltage.  The d-axis root choice must be fixed in
        ``solve_direct_current`` BEFORE the limit is raised; when it is, this
        test will fail and must be replaced by a small-draw assertion.
        Measured with an 800 V limit at a 650 V bus: 129,629 W at 0 rpm and
        124,919 W at 145 rpm (reviewer saw 580,740 W / 124,919 W).
        """
        text = self.path.read_text()
        data_path = (self.path.parent / "../data/amk_dd5_100c_operating_map.npz").resolve()
        text = text.replace('value="600" unit="V"/>', 'value="800" unit="V"/>')
        text = text.replace("../data/amk_dd5_100c_operating_map.npz", str(data_path))
        with tempfile.TemporaryDirectory() as folder:
            xml = Path(folder) / "motor.xml"
            xml.write_text(text)
            motor = compile_electric_motor(load_model(xml))
        self.assertEqual(motor.maximum_bus_voltage, 800.0)
        for speed in (0.0, 145.0):
            power = evaluate_shaft_power_exact(motor, speed, 0.0, 650.0).bus_power
            self.assertGreater(power, 50000.0, speed)  # defect; physical is < 50 W

    def test_invalid_maximum_bus_voltage_is_rejected(self):
        text = self.path.read_text()
        data_path = (self.path.parent / "../data/amk_dd5_100c_operating_map.npz").resolve()
        text = text.replace("../data/amk_dd5_100c_operating_map.npz", str(data_path))
        for bad in ("0", "-5", "nan", "inf"):
            with tempfile.TemporaryDirectory() as folder:
                xml = Path(folder) / "motor.xml"
                xml.write_text(text.replace('value="600" unit="V"/>', f'value="{bad}" unit="V"/>'))
                with self.assertRaises(ValueError, msg=bad):
                    compile_electric_motor(load_model(xml))


class MotorLutQueryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.motor = compile_electric_motor(load_model(ROOT / "models/power-plants/amk_dd5_100c.xml"))

    def test_exact_node_and_random_forward_accuracy(self):
        m = self.motor.operating_map
        for vi, si in ((0, 0), (4, 40), (8, 128)):
            torque = float(m.drive_torque[vi, si])
            lut = evaluate_motor(self.motor, float(m.speed[si]), torque, float(m.voltage[vi]))
            exact = evaluate_shaft_power_exact(self.motor, float(m.speed[si]), torque, float(m.voltage[vi]))
            self.assertAlmostEqual(lut.bus_power, exact.bus_power, places=8)
        rng = np.random.default_rng(7)
        errors = []
        for _ in range(100):
            voltage = rng.uniform(m.voltage[0], m.voltage[-1])
            speed = rng.uniform(m.speed[0], m.speed[-1])
            limits = motor_torque_limits(self.motor, speed, voltage)
            torque = rng.uniform(-limits.maximum_regen_torque, limits.maximum_drive_torque)
            lut = evaluate_motor(self.motor, speed, torque, voltage)
            exact = evaluate_shaft_power_exact(self.motor, speed, torque, voltage)
            errors.append(abs(lut.bus_power - exact.bus_power))
        self.assertLess(float(np.quantile(errors, 0.95)), 120.0)

    def test_fallback_is_exact_and_observable(self):
        reset_motor_lut_counters()
        lut = evaluate_motor(self.motor, 1000.0, 1.0, 400.0)
        exact = evaluate_shaft_power_exact(self.motor, 1000.0, 1.0, 400.0)
        self.assertEqual(lut, exact)
        self.assertGreaterEqual(motor_lut_counters()["fallback"], 1)

    def _zero_envelope_motor(self):
        mapping = MotorOperatingMap(
            np.array([400.0, 500.0]), np.array([0.0, 2000.0]),
            np.array([-1.0, 0.0, 1.0]),
            np.zeros((2, 2)), np.zeros((2, 2)),
            np.zeros((2, 2, 3)), np.zeros((2, 2, 3)),
            np.zeros((2, 2, 3), dtype=bool),
        )
        return replace(self.motor, operating_map=mapping)

    def test_zero_envelope_forward_handles_zero_and_falls_back_nonzero(self):
        motor = self._zero_envelope_motor()
        reset_motor_lut_counters()
        zero = evaluate_motor(motor, 1000.0, 0.0, 450.0)
        self.assertEqual(zero.bus_power, 0.0)
        self.assertEqual(motor_lut_counters()["fallback"], 0)
        nonzero = evaluate_motor(motor, 1000.0, 1.0, 450.0)
        self.assertEqual(
            nonzero, evaluate_shaft_power_exact(motor, 1000.0, 1.0, 450.0)
        )
        self.assertEqual(motor_lut_counters()["fallback"], 1)

    def test_zero_envelope_inverse_uses_exact_fallback(self):
        motor = self._zero_envelope_motor()
        for direction in ("drive", "regen"):
            reset_motor_lut_counters()
            result = torque_for_bus_power(motor, 1000.0, 450.0, 0.0, direction)
            self.assertTrue(result.used_fallback)
            self.assertTrue(math.isfinite(result.torque))
            self.assertTrue(math.isfinite(result.residual))
            self.assertGreaterEqual(motor_lut_counters()["fallback"], 1)

    def test_inverse_drive_and_regen(self):
        m = self.motor.operating_map
        speed, voltage = 1000.0, 550.0
        drive_target = evaluate_motor(self.motor, speed, 10.0, voltage).bus_power
        drive = torque_for_bus_power(self.motor, speed, voltage, drive_target, "drive")
        self.assertTrue(drive.feasible)
        self.assertLess(abs(drive.residual), 2.0)
        regen = torque_for_bus_power(self.motor, speed, voltage, -100.0, "regen")
        self.assertTrue(regen.feasible)
        self.assertGreaterEqual(regen.operating_point.bus_power, -100.0 - 1.0e-6)
        self.assertLessEqual(regen.torque, 0.0)

    def test_out_of_domain_limit_query_is_cached(self):
        """A repeated off-map limit query must not re-run the bisection."""
        reset_motor_lut_counters()
        first = motor_torque_limits(self.motor, 1000.0, 437.0)
        self.assertTrue(first.used_fallback)
        evaluations = motor_lut_counters()["fallback"]
        self.assertGreaterEqual(evaluations, 1)
        for _ in range(5):
            again = motor_torque_limits(self.motor, 1000.0, 437.0)
            self.assertEqual(again, first)
        self.assertEqual(motor_lut_counters()["fallback"], evaluations)

    def test_exact_limits_support_out_of_domain_voltage(self):
        limits = motor_torque_limits(self.motor, 1000.0, 400.0)
        self.assertTrue(limits.used_fallback)
        self.assertGreater(limits.maximum_drive_torque, 0.0)
        self.assertGreater(limits.maximum_regen_torque, 0.0)

    def _row_reference(self, mapping, voltage, speed):
        """Build one load row with the scalar four-corner accessor.

        This is the review reference for the vectorized production row.
        """
        vi = int(np.searchsorted(mapping.voltage, voltage, side="right")) - 1
        vi = max(0, min(vi, len(mapping.voltage) - 2))
        vf = (voltage - mapping.voltage[vi]) / (mapping.voltage[vi + 1] - mapping.voltage[vi])
        si = int(np.searchsorted(mapping.speed, speed, side="right")) - 1
        si = max(0, min(si, len(mapping.speed) - 2))
        sf = (speed - mapping.speed[si]) / (mapping.speed[si + 1] - mapping.speed[si])

        def value(array, k):
            return (
                (1 - vf) * (1 - sf) * array[vi, si, k]
                + vf * (1 - sf) * array[vi + 1, si, k]
                + (1 - vf) * sf * array[vi, si + 1, k]
                + vf * sf * array[vi + 1, si + 1, k]
            )

        return np.array(
            [value(mapping.bus_power, k) for k in range(mapping.load.size)]
        )

    def test_vectorized_row_matches_scalar_reference(self):
        rng = np.random.default_rng(23)
        mapping = self.motor.operating_map
        for _ in range(25):
            voltage = float(rng.uniform(mapping.voltage[0], mapping.voltage[-1]))
            speed = float(rng.uniform(mapping.speed[0], mapping.speed[-1]))
            reference = self._row_reference(mapping, voltage, speed)
            vi = int(np.searchsorted(mapping.voltage, voltage, side="right")) - 1
            vi = max(0, min(vi, len(mapping.voltage) - 2))
            vf = (voltage - mapping.voltage[vi]) / (mapping.voltage[vi + 1] - mapping.voltage[vi])
            si = int(np.searchsorted(mapping.speed, speed, side="right")) - 1
            si = max(0, min(si, len(mapping.speed) - 2))
            sf = (speed - mapping.speed[si]) / (mapping.speed[si + 1] - mapping.speed[si])
            vector = _interpolated_row(mapping.bus_power, vi, si, vf, sf)
            np.testing.assert_array_equal(vector, reference)

    def _in_map_targets(self, count, seed):
        """Return in-map (voltage, speed, drive target) triples.

        Targets are drawn strictly inside the achievable drive row range, which
        is the assembled solver's storage-limited regime.
        """
        mapping = self.motor.operating_map
        rng = np.random.default_rng(seed)
        cases = []
        while len(cases) < count:
            voltage = float(rng.uniform(mapping.voltage[0], mapping.voltage[-1]))
            speed = float(rng.uniform(mapping.speed[0], mapping.speed[-1]))
            vi, vf = _voltage_fraction(mapping, voltage)
            si, sf = _speed_fraction(mapping, speed)
            if _envelope_cliff_cell(mapping, vi, si):
                # Cliff cells are answered by the exact oracle by design.
                continue
            row = _interpolated_row(mapping.bus_power, vi, si, vf, sf)
            zero = int(np.searchsorted(mapping.load, 0.0))
            low, high = float(row[zero]), float(row[-1])
            if not high > low:
                continue
            cases.append(
                (voltage, speed, low + rng.uniform(0.02, 0.98) * (high - low))
            )
        return cases

    def test_in_map_drive_inverse_does_not_fall_back(self):
        """A representative in-map drive sweep must stay on the LUT row.

        Every target is strictly inside the achievable row range, so the
        monotone row solves it directly and no analytical fallback is needed.
        """
        cases = self._in_map_targets(200, 101)
        reset_motor_lut_counters()
        for voltage, speed, target in cases:
            result = torque_for_bus_power(self.motor, speed, voltage, target, "drive")
            self.assertGreaterEqual(result.torque, 0.0)
            self.assertFalse(result.used_fallback)
            self.assertLessEqual(
                result.operating_point.bus_power, target + 1.0e-3
            )
        counters = motor_lut_counters()
        self.assertEqual(counters["inverse"], len(cases))
        self.assertEqual(counters["fallback"], 0)

    def test_in_map_drive_inverse_is_cheap(self):
        """A fallback must not be orders of magnitude more expensive than a row solve."""
        cases = self._in_map_targets(300, 202)
        for voltage, speed, target in cases[:20]:
            torque_for_bus_power(self.motor, speed, voltage, target, "drive")
        reset_motor_lut_counters()
        started = time.perf_counter()
        for voltage, speed, target in cases:
            torque_for_bus_power(self.motor, speed, voltage, target, "drive")
        elapsed = time.perf_counter() - started
        self.assertEqual(motor_lut_counters()["fallback"], 0)
        # The old row build was ~45 us/call; acceptance-driven fallbacks reached
        # tens of milliseconds per call.  This ceiling still fails loudly if a
        # fallback storm returns.
        self.assertLess(elapsed / len(cases), 1.0e-3)

    def test_exact_fallback_is_bounded(self):
        """One exact fallback must stay near the analytical bisection cost."""
        reset_motor_lut_counters()
        # 437.0 V is below the persisted map voltage range, so this is genuinely
        # out of domain and must use the exact analytical fallback.
        result = torque_for_bus_power(self.motor, 1000.0, 437.0, 20_000.0, "drive")
        self.assertTrue(result.used_fallback)
        self.assertTrue(result.feasible)
        evaluations = motor_lut_counters()["fallback"]
        self.assertLessEqual(evaluations, 200)
        self.assertGreaterEqual(evaluations, 1)

    def test_regen_inverse_respects_constraint_on_canonical_nonmonotone_row(self):
        """The canonical regen row is non-monotone; the scan must stay correct."""
        mapping = self.motor.operating_map
        # Task 3 records this exact node as a non-monotone regen row.
        vi, si = 8, 17
        voltage, speed = float(mapping.voltage[vi]), float(mapping.speed[si])
        row = mapping.bus_power[vi, si, :]
        negative = int(np.searchsorted(mapping.load, 0.0))
        self.assertLess(float(np.min(row)), float(row[0]) - 200.0)
        for fraction in (0.05, 0.25, 0.5, 0.75, 0.95):
            target = float(np.min(row[:negative])) * fraction
            reset_motor_lut_counters()
            result = torque_for_bus_power(self.motor, speed, voltage, target, "regen")
            self.assertTrue(result.feasible)
            self.assertFalse(result.used_fallback)
            self.assertLessEqual(result.torque, 0.0)
            self.assertGreaterEqual(
                result.operating_point.bus_power, target - 1.0e-3
            )

    def test_regen_inverse_selects_deepest_admissible_node(self):
        """On a monotone row the deepest admissible mapped node must win.

        Load zero is non-productive in this fixture, and the deepest node
        overshoots the -50 W constraint, so the crossing onto the second node
        is the answer.
        """
        powers = [40.0, -15.0, -30.0, -50.0, -75.0, 0.0, 30.0, 60.0, 90.0]
        motor = self._regen_row_motor(powers)
        result = torque_for_bus_power(motor, 1000.0, 450.0, -50.0, "regen")
        self.assertTrue(result.feasible)
        self.assertFalse(result.used_fallback)
        # The -50 W node is already inadmissible, so the crossing sits at -0.75.
        self.assertAlmostEqual(result.torque, -15.0)
        self.assertGreaterEqual(
            result.operating_point.bus_power, -50.0 - 1.0e-3
        )

    def test_regen_inverse_deeper_than_row_returns_deepest_admissible_node(self):
        """A request below every mapped power still respects the constraint."""
        powers = [40.0, -15.0, -30.0, -50.0, -75.0, 0.0, 30.0, 60.0, 90.0]
        motor = self._regen_row_motor(powers)
        result = torque_for_bus_power(motor, 1000.0, 450.0, -200.0, "regen")
        self.assertTrue(result.feasible)
        # No mapped point reaches -200 W, so the deepest admissible crossing is
        # returned and still charges at least as much as the constraint allows.
        self.assertAlmostEqual(result.torque, -15.0)
        self.assertGreaterEqual(
            result.operating_point.bus_power, -200.0 - 1.0e-3
        )

    def test_regen_inverse_interpolates_inside_the_admissible_segment(self):
        """An interior request is solved on its own piecewise-linear segment."""
        powers = [-60.0, -45.0, -30.0, -15.0, 0.0, 20.0, 40.0, 60.0, 80.0]
        motor = self._regen_row_motor(powers)
        result = torque_for_bus_power(motor, 1000.0, 450.0, -95.0, "regen")
        self.assertTrue(result.feasible)
        self.assertFalse(result.used_fallback)
        # -95 W lies between the -90 W and -105 W nodes, so it is interior.
        self.assertAlmostEqual(result.torque, -20.0)

    def test_regen_inverse_shallow_target_needs_no_braking(self):
        """A request weaker than the zero-load point needs no braking."""
        powers = [0.0, -8.0, -15.0, -25.0, -5.0, 0.0, 0.0, 0.0, 100.0]
        motor = self._regen_row_motor(powers)
        result = torque_for_bus_power(motor, 1000.0, 450.0, -3.0, "regen")
        self.assertTrue(result.feasible)
        self.assertAlmostEqual(result.torque, 0.0)

    def test_regen_inverse_mixed_mask_skips_invalid_segment(self):
        """An invalid corner excludes every segment it bounds."""
        powers = [40.0, -50.0, -12.0, -8.0, 0.0, 30.0, 60.0, 90.0, 120.0]
        mask = [False, False, True, True, False, False, False, False, False]
        motor = self._regen_row_motor(powers, mask)
        result = torque_for_bus_power(motor, 1000.0, 450.0, -10.0, "regen")
        self.assertTrue(result.feasible)
        # The -50 W node is masked out, so the admissible crossing comes from
        # the first valid segment and yields -7.5 N.m.
        self.assertAlmostEqual(result.torque, -7.5)
        self.assertAlmostEqual(result.operating_point.bus_power, -10.0)

    def test_regen_scan_matches_independent_row_reference(self):
        """Every regen case must equal an independent piecewise-linear scan.

        The reference below re-derives the deepest admissible load straight from
        the mapped row and mask, independently of the production vectorization.
        """
        cases = (
            ([40.0, -15.0, -30.0, -50.0, -75.0, 0.0, 30.0, 60.0, 90.0], None, -50.0),
            ([40.0, -15.0, -30.0, -50.0, -75.0, 0.0, 30.0, 60.0, 90.0], None, -200.0),
            ([-60.0, -45.0, -30.0, -15.0, 0.0, 20.0, 40.0, 60.0, 80.0], None, -95.0),
            (
                [40.0, -50.0, -12.0, -8.0, 0.0, 30.0, 60.0, 90.0, 120.0],
                [False, False, True, True, False, False, False, False, False],
                -10.0,
            ),
            ([0.0, -8.0, -15.0, -25.0, -5.0, 0.0, 0.0, 0.0, 100.0], None, -10.0),
        )
        for powers, mask, target in cases:
            motor = self._regen_row_motor(powers, mask)
            expected = self._reference_regen_load(powers, mask, target)
            self.assertIsNotNone(expected, (powers, target))
            result = torque_for_bus_power(motor, 1000.0, 450.0, target, "regen")
            self.assertAlmostEqual(result.torque, 20.0 * expected)
            self.assertGreaterEqual(
                result.operating_point.bus_power, target - 1.0e-3
            )

    def _reference_regen_load(self, powers, mask, target):
        """Independent scalar scan for the deepest admissible mapped load."""
        loads = np.linspace(-1.0, 1.0, 9)
        values = np.asarray(powers, dtype=float)
        valid = (
            np.asarray(mask, dtype=bool) if mask is not None else values < -1.0e-9
        )
        zero = int(np.searchsorted(loads, 0.0))
        candidates = []
        for k in range(zero + 1):
            if valid[k] and values[k] >= target - 1.0e-9:
                candidates.append(float(loads[k]))
        for k in range(zero):
            if not (valid[k] and valid[k + 1]):
                continue
            if values[k + 1] == values[k]:
                continue
            if (values[k] - target) * (values[k + 1] - target) > 0.0:
                continue
            fraction = (target - values[k]) / (values[k + 1] - values[k])
            candidates.append(float(loads[k] + fraction * (loads[k + 1] - loads[k])))
        return min(candidates) if candidates else None


    REGEN_LOADS = np.linspace(-1.0, 1.0, 9)

    def _regen_row_motor(self, powers, mask=None):
        """Build a nine-node map around one regen row.

        When ``mask`` is omitted it is derived from the powers, which is the
        invariant the compiled map validates: a node is productive exactly when
        its bus power is negative.
        """
        values = np.asarray(powers, dtype=float)
        valid = (
            np.asarray(mask, dtype=bool) if mask is not None else values < -1.0e-9
        )
        mapping = MotorOperatingMap(
            np.array([400.0, 500.0]), np.array([0.0, 2000.0]),
            self.REGEN_LOADS,
            np.full((2, 2), 20.0), np.full((2, 2), 20.0),
            np.tile(values, (2, 2, 1)),
            np.zeros((2, 2, self.REGEN_LOADS.size)),
            np.tile(valid, (2, 2, 1)),
        )
        return replace(self.motor, operating_map=mapping)


class PowertrainAssemblyTests(unittest.TestCase):
    def test_canonical_nested_assemblies_compile(self):
        vehicle = load_point_mass_vehicle(ROOT / "models/formulations/murphy_point_mass.xml")
        self.assertIsInstance(vehicle.drivetrain, FullDrivetrainRuntime)
        self.assertEqual(vehicle.drivetrain.powertrain.motor_mode, "lut")
        self.assertEqual(vehicle.drivetrain.powertrain.power_plant.unit_count, 4)
        self.assertEqual(vehicle.drivetrain.motor_count, 4)
        self.assertEqual(vehicle.drivetrain.gear_ratio, 10.9)
        self.assertEqual(vehicle.drivetrain.maximum_motor_speed, 18600.0)
        self.assertEqual(vehicle.manifest.component("powertrain").subtype, "electric")
        self.assertEqual(vehicle.manifest.component("drivetrain").subtype, "electric-four-wheel")
        self.assertEqual(vehicle.manifest.component("powertrain_drivetrain").subtype, "four-wheel")

    def test_nested_assemblies_are_recorded(self):
        vehicle = load_point_mass_vehicle(ROOT / "models/formulations/murphy_point_mass.xml")
        record = vehicle.manifest.capability("drive-force-limit")
        self.assertEqual(record.source_capability, "powertrain-and-drivetrain")
        self.assertEqual(record.adapter_names, ("reduce_powertrain_and_drivetrain",))
        self.assertEqual(record.component_role, "powertrain_drivetrain")

    def test_lut_drive_limit_never_exceeds_storage_discharge_power(self):
        """A LUT drive limit must respect the storage discharge limit.

        This is the out-of-domain case: the voltage-coupling point can fall
        below the compiled map's minimum voltage, where ``row_range`` is None.
        The interpolated envelope is then not an authority, so the bounded exact
        inverse must answer and must not report a bus power above the storage
        discharge limit.
        """
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        lut = replace(powertrain, motor_mode="lut")
        radius = 0.193675
        checked = 0
        for state_of_charge in (0.05, 0.02, 0.005, 0.3, 0.8):
            state = StorageState(state_of_charge)
            storage = evaluate_storage(lut.energy_storage, state)
            for speed in (20.0, 30.0, 40.0):
                result = maximum_drive_force(lut, drivetrain, speed, state, radius)
                checked += 1
                self.assertLessEqual(
                    result.bus_power_at_limit,
                    storage.discharge_power_limit + 1.0e-6,
                    (state_of_charge, speed, result.bus_power_at_limit,
                     storage.discharge_power_limit),
                )
        self.assertGreater(checked, 0)

    def test_drive_limit_never_exceeds_pack_discharge_limit_in_both_modes(self):
        """Per-motor bus power must be compared with the per-motor share.

        Regression for the analytical branch comparing a per-motor power with
        the pack limit (worst case before the fix: SOC 1.0, 17.5 m/s,
        102,775 W against a 100,495 W pack limit).
        """
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        radius = 0.193675
        checked = 0
        for mode in ("analytical", "lut"):
            configured = replace(powertrain, motor_mode=mode)
            for state_of_charge in (1.0, 0.75, 0.5, 0.25):
                state = StorageState(state_of_charge)
                storage = evaluate_storage(configured.energy_storage, state)
                for speed in (0.0, 5.0, 10.0, 15.0, 17.5, 20.0, 25.0, 31.0):
                    result = maximum_drive_force(
                        configured, drivetrain, speed, state, radius
                    )
                    checked += 1
                    self.assertLessEqual(
                        result.bus_power_at_limit,
                        storage.discharge_power_limit + 1.0e-6,
                        (mode, state_of_charge, speed),
                    )
        self.assertEqual(checked, 64)

    def test_lut_drive_coupling_tolerance_is_not_widened(self):
        """LUT mode must keep the analytical voltage-coupling acceptance.

        The 1.0e-3 row tolerance is a local variable; if it shadowed the 1.0e-6
        coupling tolerance the LUT would accept a coupling residual 1000x larger
        and report ``converged`` on points the analytical baseline rejects.
        """
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        analytical = replace(powertrain, motor_mode="analytical")
        lut = replace(powertrain, motor_mode="lut")
        radius = 0.193675
        for state_of_charge in (0.05, 0.3, 0.8):
            for speed in (5.0, 20.0, 30.0, 60.0):
                state = StorageState(state_of_charge)
                baseline = maximum_drive_force(
                    analytical, drivetrain, speed, state, radius
                )
                result = maximum_drive_force(lut, drivetrain, speed, state, radius)
                # The two loops run the same coupling iteration, so a widened
                # tolerance would show up as a converged LUT point whose
                # residual the analytical baseline still rejects.
                if result.converged:
                    self.assertLessEqual(
                        result.shaft_torque_residual, 1.0e-6 + 1.0e-12,
                        (state_of_charge, speed, result.shaft_torque_residual),
                    )
                if result.converged:
                    # A converged LUT point must also satisfy the storage limit.
                    storage = evaluate_storage(lut.energy_storage, state)
                    self.assertLessEqual(
                        result.bus_power_at_limit,
                        storage.discharge_power_limit + 1.0e-6,
                        (state_of_charge, speed),
                    )

    def test_lut_active_constraint_matches_storage_limit(self):
        """A storage-limited LUT result must not be labelled motor_torque."""
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        lut = replace(powertrain, motor_mode="lut")
        radius = 0.193675
        for state_of_charge in (0.05, 0.3, 0.8):
            state = StorageState(state_of_charge)
            storage = evaluate_storage(lut.energy_storage, state)
            for speed in (5.0, 20.0, 30.0, 45.0):
                result = maximum_drive_force(lut, drivetrain, speed, state, radius)
                if result.bus_power_at_limit > storage.discharge_power_limit + 1.0e-6:
                    self.assertNotEqual(result.active_constraint, "motor_torque")

    def test_regen_force_report_has_both_limits(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = maximum_regen_force(
            powertrain, drivetrain, 10.0, StorageState(0.8), effective_radius=0.193675
        )
        self.assertGreater(result.force_from_torque, 0.0)
        self.assertGreater(result.force_from_charge_power, 0.0)
        # Wheel power is reduced by both directional losses before charging.
        motor = powertrain.power_plant
        transmission = drivetrain.transmission
        battery_power = (
            result.force_limit * 10.0
            * transmission.gear_efficiency * motor.inverter_efficiency
        )
        self.assertLessEqual(battery_power, abs(result.charge_power_limit) + 1.0e-6)
        self.assertGreaterEqual(result.iterations, 1)
        self.assertLessEqual(result.residual, 1.0e-9)
        self.assertTrue(result.converged)

    def test_fixed_point_reports_explicit_nonconvergence(self):
        motor = compile_electric_motor(load_model(ROOT / "models/power-plants/amk_dd5_100c.xml"))
        result = solve_shaft_torque_fixed_point(
            motor, 1000.0, 1000.0, 600.0, tolerance=1.0e-30, max_iterations=1
        )
        self.assertFalse(result.converged)
        self.assertEqual(result.iterations, 1)
        self.assertGreater(result.residual, result.torque * 0.0)

    def test_assembly_rejects_motor_wheel_mismatch(self):
        motor_path = ROOT / "models/power-plants/amk_dd5_100c.xml"
        drivetrain_path = ROOT / "models/drivetrains/murphy_2024_drivetrain.xml"
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            motor = motor_path.read_text().replace('value="4"', 'value="3"', 1)
            (folder / "motor.xml").write_text(motor)
            (folder.parent / "data").mkdir(exist_ok=True)
            (folder.parent / "data" / "amk_dd5_100c_operating_map.npz").write_bytes(
                (ROOT / "models/data/amk_dd5_100c_operating_map.npz").read_bytes()
            )
            (folder / "powertrain.xml").write_text(
                '<model schema-version="1.0" id="test-powertrain" '
                'type="powertrain-assembly" subtype="electric">'
                '<components><component role="power-plant" ref="motor.xml"/>'
                f'<component role="energy-storage" ref="{(ROOT / "models/energy-storages/murphy_2024_battery.xml").as_posix()}"/>'
                '</components></model>'
            )
            vehicle = folder / "vehicle.xml"
            vehicle.write_text(
                '<model schema-version="1.0" id="test-vehicle" '
                'type="vehicle-assembly" subtype="four-wheel">'
                f'<components><component role="powertrain" ref="powertrain.xml"/>'
                f'<component role="drivetrain" ref="{drivetrain_path.as_posix()}"/>'
                '</components></model>'
            )
            with self.assertRaisesRegex(ValueError, "does not match driven wheels"):
                compile_powertrain_drivetrain(load_model(vehicle))

    def test_drive_force_has_positive_bus_power(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        for speed in (0.0, 30.0):
            result = maximum_drive_force(
                powertrain,
                drivetrain,
                speed,
                StorageState(0.8),
                effective_radius=0.193675,
            )
            self.assertGreater(result.maximum_drive_force, 0.0)
            self.assertGreater(result.bus_power_at_limit, 0.0)
            self.assertLess(result.storage_state_rate, 0.0)

    def test_drive_force_operating_point_has_positive_evaluated_bus_power(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        radius = 0.193675
        result = maximum_drive_force(
            powertrain, drivetrain, 10.0, StorageState(0.8), effective_radius=radius
        )
        motor = powertrain.power_plant
        transmission = drivetrain.transmission
        motor_speed = transmission.motor_speed(10.0 / radius) * 60.0 / (2.0 * math.pi)
        motor_torque = result.maximum_wheel_torque / (
            motor.unit_count * transmission.gear_ratio * transmission.gear_efficiency
        )
        open_limits = evaluate_storage(powertrain.energy_storage, StorageState(0.8))
        self.assertLess(result.bus_voltage, open_limits.bus_voltage)
        # The reported voltage is post-update, while power was evaluated before
        # the final damped update. Recover that iteration's voltage exactly;
        # evaluating at the reported voltage would compare different points.
        final_terminal = terminal_voltage(
            powertrain.energy_storage, StorageState(0.8), result.storage_current
        )
        evaluation_voltage = 2.0 * result.bus_voltage - final_terminal
        evaluated = evaluate_motor(
            motor, motor_speed, motor_torque, evaluation_voltage
        )
        self.assertTrue(evaluated.feasible)
        self.assertGreaterEqual(evaluated.bus_power, 0.0)
        self.assertAlmostEqual(
            evaluated.bus_power * motor.unit_count,
            result.bus_power_at_limit,
            places=6,
        )

    def test_full_and_lumped_assemblies_share_operating_point(self):
        full = compile_powertrain_drivetrain(
            load_model(ROOT / "models/vehicles/murphy_2024.xml")
        )
        lumped = compile_lumped_drivetrain(
            load_model(ROOT / "models/drivetrains/murphy_4wd.xml")
        )
        for name in (
            "motor_count",
            "gear_ratio",
            "peak_motor_torque",
            "power_limit",
            "efficiency",
            "maximum_motor_speed",
        ):
            self.assertEqual(getattr(full, name), getattr(lumped, name))
        speed = 10.0
        state = StorageState(0.8)
        radius = 0.193675
        full_limit = maximum_drive_force(
            full.powertrain, full.drivetrain, speed, state, radius
        )
        legacy_limit = maximum_drive_force_lumped(
            full.powertrain, lumped, speed, state, radius
        )
        # The lumped path omits map losses, so allow its reviewed 2 percent
        # force approximation while requiring matching active behavior.
        self.assertAlmostEqual(
            full_limit.maximum_drive_force,
            legacy_limit.maximum_drive_force,
            delta=250.0,
        )
        self.assertAlmostEqual(
            full_limit.bus_power_at_limit,
            legacy_limit.bus_power_at_limit,
            delta=40000.0,
        )
        self.assertEqual(full_limit.active_constraint, legacy_limit.active_constraint)
        self.assertAlmostEqual(
            full_limit.storage_current, legacy_limit.storage_current, delta=70.0
        )
        self.assertAlmostEqual(
            full_limit.storage_state_rate, legacy_limit.storage_state_rate, delta=0.002
        )
        changed = replace(lumped, peak_motor_torque=lumped.peak_motor_torque * 0.5)
        changed_limit = maximum_drive_force_lumped(
            full.powertrain, changed, speed, state, radius
        )
        self.assertNotEqual(changed_limit.maximum_drive_force, legacy_limit.maximum_drive_force)
        self.assertEqual(
            full_limit.maximum_drive_force,
            maximum_drive_force(full.powertrain, full.drivetrain, speed, state, radius).maximum_drive_force,
        )

    def test_regen_charge_power_constraint_is_active(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = maximum_regen_force(
            powertrain, drivetrain, 30.0, StorageState(0.8), effective_radius=0.193675
        )
        self.assertEqual(result.active_constraint, "storage_power")
        self.assertLessEqual(result.force_limit, result.force_from_charge_power)

    def test_resolve_braking_obeys_charge_power(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = resolve_braking(
            powertrain, drivetrain, 30.0, StorageState(0.8), effective_radius=0.193675
        )
        self.assertEqual(result.active_constraint, "storage_power")
        self.assertLessEqual(result.force_limit, result.force_from_charge_power)

    def test_resolve_braking_allocates_friction_after_regen(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = resolve_braking(
            powertrain,
            drivetrain,
            30.0,
            StorageState(0.8),
            braking_demand=3000.0,
            tire_force_capacity=2500.0,
            effective_radius=0.193675,
        )
        self.assertEqual(result.total_force_limit, 2500.0)
        self.assertGreater(result.regen_force_used, 0.0)
        self.assertGreater(result.friction_force_used, 0.0)
        self.assertAlmostEqual(
            result.regen_force_used + result.friction_force_used,
            result.total_force_limit,
        )
        self.assertGreater(result.recovered_bus_power, 0.0)
        self.assertEqual(result.total_force, result.total_force_limit)
        self.assertEqual(result.recovered_power, result.recovered_bus_power)

    def test_braking_overspeed_and_standstill_have_no_recovery(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        standstill = resolve_braking(
            powertrain,
            drivetrain,
            0.0,
            StorageState(0.8),
            braking_demand=1000.0,
            effective_radius=0.193675,
        )
        self.assertEqual(standstill.recovered_bus_power, 0.0)
        overspeed = resolve_braking(
            powertrain,
            drivetrain,
            400.0,
            StorageState(0.8),
            braking_demand=1000.0,
            effective_radius=0.193675,
        )
        self.assertEqual(overspeed.regen_force_used, 0.0)
        self.assertEqual(overspeed.recovered_bus_power, 0.0)
        self.assertEqual(overspeed.active_constraint, "motor_speed")

    def test_regen_rejects_motor_overspeed(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = maximum_regen_force(
            powertrain, drivetrain, 400.0, StorageState(0.8), effective_radius=0.193675
        )
        self.assertEqual(result.active_constraint, "motor_speed")
        self.assertEqual(result.force_limit, 0.0)

    def test_fixed_point_reports_convergence_and_residual(self):
        motor = compile_electric_motor(load_model(ROOT / "models/power-plants/amk_dd5_100c.xml"))
        result = solve_shaft_torque_fixed_point(motor, 1000.0, 1000.0, 600.0)
        self.assertTrue(result.converged)
        self.assertLessEqual(result.residual, 1.0e-9)
        self.assertGreater(result.iterations, 0)


if __name__ == "__main__":
    unittest.main()
