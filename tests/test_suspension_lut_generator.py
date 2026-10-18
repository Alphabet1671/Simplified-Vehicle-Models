import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.model_data import generate_suspension_lut as gen

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "iem24_dss_suspension_rows.csv"
REAL_XLSX = Path("/Users/alphabet1671/Desktop/IEM_24_DSS.xlsx")
REAL_SHA256 = "e436c2f61d46f23614515e34cbce381e45a5e508b9840012c88511f59afac5d8"
K_FRONT = 450 * gen.LBF_IN_TO_N_M
K_REAR = 350 * gen.LBF_IN_TO_N_M


def fixture_sha():
    return hashlib.sha256(FIXTURE.read_bytes()).hexdigest()


def build(cells, **kwargs):
    return gen.build_corner_arrays(
        cells,
        wheel_rate_front_n_m=K_FRONT,
        wheel_rate_rear_n_m=K_REAR,
        static_load_front_n=505.777974,
        static_load_rear_n=646.503401,
        **kwargs,
    )


class ParsingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cells = gen.read_csv_cells(FIXTURE)

    def test_fixture_cells(self):
        self.assertEqual(self.cells["D17"], "31.8")
        self.assertEqual(self.cells["C26"], "0")
        self.assertEqual(self.cells["C29"], "54.2/32.0")

    def test_axle_values(self):
        front = gen.read_axle_cells(self.cells, "front")
        rear = gen.read_axle_cells(self.cells, "rear")
        self.assertEqual(front["jounce_limits"], (-0.0318, 0.0318))
        self.assertEqual(rear["jounce_limits"], (-0.0318, 0.0318))
        self.assertAlmostEqual(front["rc_static"], 0.041275)
        self.assertAlmostEqual(rear["rc_static"], 0.05715)
        self.assertEqual((front["anti_brake"], front["anti_drive"]), (0.542, 0.320))
        self.assertEqual((rear["anti_brake"], rear["anti_drive"]), (0.574, 0.550))
        self.assertEqual((front["static_toe_rad"], rear["static_toe_rad"]), (0.0, 0.0))
        self.assertEqual((front["motion_ratio"], rear["motion_ratio"]), (1.05, 1.1))
        self.assertEqual((front["camber_gain"], rear["camber_gain"]), (25.1, 45.3))
        self.assertAlmostEqual(front["rc_1g_height"], 0.0455)
        self.assertAlmostEqual(rear["rc_1g_height"], 0.0653)
        self.assertAlmostEqual(front["rc_1g_lateral"], -0.006)
        self.assertAlmostEqual(rear["rc_1g_lateral"], -0.008)

    def test_heave_ratio_ignores_roll_ratio(self):
        self.assertEqual(gen.parse_heave_motion_ratio({"C23": "1.05:1 heave / 1.2:1 roll"}, "C23"), 1.05)
        self.assertEqual(gen.parse_heave_motion_ratio({"F23": "1.1:1 heave / 1.3:1 roll"}, "F23"), 1.1)
        with self.assertRaises(gen.SuspensionTableError):
            gen.parse_heave_motion_ratio({"C23": "1.2:1 roll"}, "C23")

    def test_text_or_numeric_toe_and_nonnumeric_failure(self):
        import math
        base = dict(self.cells)
        base["C26"] = 0.5
        self.assertAlmostEqual(gen.read_axle_cells(base, "front")["static_toe_rad"], math.radians(0.5))
        base["C26"] = "-0.25"
        self.assertAlmostEqual(gen.read_axle_cells(base, "front")["static_toe_rad"], math.radians(-0.25))
        for bad in ("abc", "", "n/a"):
            cells = dict(self.cells)
            cells["F26"] = bad
            with self.assertRaises(gen.SuspensionTableError):
                gen.read_axle_cells(cells, "rear")
        for address in ("D17", "C24", "C30", "E31"):
            cells = dict(self.cells)
            cells[address] = "x"
            with self.assertRaises(gen.SuspensionTableError):
                gen.read_axle_cells(cells, "front")
        cells = dict(self.cells)
        del cells["H17"]
        with self.assertRaises(gen.SuspensionTableError):
            gen.read_axle_cells(cells, "rear")

    def test_anti_parse_failure(self):
        with self.assertRaises(gen.SuspensionTableError):
            gen.parse_anti({"C29": "54.2"}, "C29")


class ArrayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cells = gen.read_csv_cells(FIXTURE)
        cls.arrays = build(cls.cells, source_sha256=REAL_SHA256)

    def test_baseline_jounce_steer(self):
        for axle in ("front", "rear"):
            np.testing.assert_array_equal(self.arrays[f"jounce_steer_jounce_{axle}"], [-0.0318, 0.0, 0.0318])
            np.testing.assert_array_equal(self.arrays[f"jounce_steer_toe_{axle}"], np.zeros(3))
        self.assertIn("baseline, no measured jounce-steer data", str(self.arrays["metadata"]))

    def test_spring_curve_static_load_and_rates(self):
        for axle, rate, load in (("front", K_FRONT, 505.777974), ("rear", K_REAR, 646.503401)):
            z, f = self.arrays[f"spring_jounce_{axle}"], self.arrays[f"spring_force_{axle}"]
            self.assertEqual(z[0], -0.0318)
            self.assertEqual(z[-1], 0.0318)
            self.assertTrue(np.all(np.diff(z) > 0.0))
            i = int(np.flatnonzero(z == 0.0)[0])
            self.assertAlmostEqual(f[i], load, places=9)
            self.assertAlmostEqual((f[i + 1] - f[i]) / z[i + 1] / rate, 1.0, delta=1e-9)
            self.assertAlmostEqual((f[i] - f[i - 1]) / -z[i - 1] / rate, 1.0, delta=1e-9)
            self.assertTrue(np.all(np.diff(f) > 0.0))

    def test_roll_centre_table(self):
        import math
        roll = self.arrays["rc_roll_front"]
        self.assertEqual(roll[0], 0.0)
        self.assertAlmostEqual(roll[1], math.radians(0.289))
        np.testing.assert_allclose(self.arrays["rc_height_front"], [0.041275, 0.0455, 0.0455])
        np.testing.assert_allclose(self.arrays["rc_height_rear"], [0.05715, 0.0653, 0.0653])
        np.testing.assert_allclose(self.arrays["rc_lateral_rear"], [0.0, -0.008, -0.008])

    def test_jounce_steer_schema_validation(self):
        limits = (-0.0318, 0.0318)
        good = ([-0.0318, 0.0, 0.0318], [0.0, 0.0, 0.0])
        gen.validate_jounce_steer(*good, limits)
        bad_cases = [
            ([-0.0318, 0.0, 0.0, 0.0318], [0, 0, 0, 0]),                # not strictly rising
            ([0.0, -0.0318, 0.0318], [0, 0, 0]),                        # not rising
            ([-0.0318, 0.01, 0.0318], [0, 0, 0]),                       # no 0 mm knot
            ([-0.0318, 0.0, 0.0318], [0, 0.001, 0]),                    # nonzero reference
            ([-0.0318, 0.0, 0.02], [0, 0, 0]),                          # no upper limit knot
            ([-0.02, 0.0, 0.0318], [0, 0, 0]),                          # no lower limit knot
            ([-0.0318, 0.0, 0.0318], [0, np.nan, 0]),                   # not finite
        ]
        for z, toe in bad_cases:
            with self.assertRaises(gen.SuspensionTableError, msg=str(z)):
                gen.validate_jounce_steer(z, toe, limits)

    def test_measured_nonzero_knots_are_accepted(self):
        arrays = build(
            self.cells,
            jounce_steer_front=([-31.8, 0.0, 31.8], [-0.1, 0.0, 0.2]),
            source_sha256=REAL_SHA256,
        )
        np.testing.assert_allclose(arrays["jounce_steer_toe_front"], np.radians([-0.1, 0.0, 0.2]))
        np.testing.assert_array_equal(arrays["jounce_steer_toe_rear"], np.zeros(3))
        with self.assertRaises(gen.SuspensionTableError):
            build(self.cells, jounce_steer_front=([-31.8, 0.0], [0.0, 0.0]))

    def test_nonpositive_rates_rejected(self):
        with self.assertRaises(gen.SuspensionTableError):
            gen.build_corner_arrays(
                self.cells, wheel_rate_front_n_m=0.0, wheel_rate_rear_n_m=K_REAR,
                static_load_front_n=500.0, static_load_rear_n=600.0,
            )


class CliTests(unittest.TestCase):
    def run_cli(self, directory, *, sha=None, source=FIXTURE):
        directory = Path(directory)
        command = [
            sys.executable, str(ROOT / "tools/model_data/generate_suspension_lut.py"),
            "--dss", str(source), "--dss-sha256", sha or fixture_sha(),
            "--wheel-rate-front-lbf-in", "450", "--wheel-rate-rear-lbf-in", "350",
            "--arb-rear-lbf-in", "120",
            "--output-npz", str(directory / "data" / "corners.npz"),
            "--output-xml", str(directory / "suspensions" / "suspension.xml"),
        ]
        return subprocess.run(command, capture_output=True, text=True, cwd=ROOT)

    def test_wrong_sha_exits_with_error_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_cli(directory, sha="0" * 64)
            self.assertEqual(result.returncode, 2)
            self.assertIn("sha256 mismatch", result.stderr)
            self.assertFalse((Path(directory) / "data").exists())

    def test_identical_runs_give_identical_sha(self):
        digests = []
        for _ in range(2):
            with tempfile.TemporaryDirectory() as directory:
                result = self.run_cli(directory)
                self.assertEqual(result.returncode, 0, result.stderr)
                data = Path(directory) / "data"
                self.assertEqual([p.name for p in data.iterdir()], ["corners.npz"])
                digest = hashlib.sha256((data / "corners.npz").read_bytes()).hexdigest()
                self.assertIn(digest, result.stdout)
                root = Path(directory)
                assembly = (root / "suspensions" / "suspension.xml").read_text()
                self.assertIn('type="suspension-assembly"', assembly)
                self.assertIn('subtype="independent-corner"', assembly)
                # The assembly holds no table; the corner NPZ is pinned by the
                # kinematics and spring part files, which sit in sibling directories.
                self.assertNotIn("corners.npz", assembly)
                parts = (
                    ("suspension-kinematics", "suspension_kinematics.xml", "suspension-kinematics"),
                    ("springs", "suspension_springs.xml", "spring-model"),
                )
                for directory_name, filename, model_type in parts:
                    xml = (root / directory_name / filename).read_text()
                    self.assertIn(f'type="{model_type}"', xml)
                    self.assertIn('subtype="corner-table"', xml)
                    self.assertIn(f'sha256="{digest}"', xml)
                    self.assertIn('path="../data/corners.npz"', xml)
                    self.assertIn(f'ref="../{directory_name}/{filename}"', assembly)
                arb = (root / "anti-roll-bars" / "suspension_arb.xml").read_text()
                self.assertIn('subtype="axle-rate"', arb)
                self.assertIn(f'ref="../anti-roll-bars/suspension_arb.xml"', assembly)
                for role in ("damper-front", "damper-rear"):
                    self.assertIn(f'role="{role}"', assembly)
                digests.append(digest)
        self.assertEqual(digests[0], digests[1])

    def test_committed_files_match_fixture_generation(self):
        cells = gen.read_csv_cells(FIXTURE)
        front, rear = gen.default_static_loads()
        arrays = gen.build_corner_arrays(
            cells, wheel_rate_front_n_m=K_FRONT, wheel_rate_rear_n_m=K_REAR,
            static_load_front_n=front, static_load_rear_n=rear, source_sha256=REAL_SHA256,
        )
        committed = (ROOT / "models" / "data" / "iem24_murphy_real_corners.npz").read_bytes()
        self.assertEqual(gen.npz_bytes(arrays), committed)

    def test_static_loads_are_sprung(self):
        front, rear = gen.default_static_loads()
        self.assertAlmostEqual(front, 506, delta=1)
        self.assertAlmostEqual(rear, 647, delta=1)


@unittest.skipUnless(REAL_XLSX.is_file(), "source IEM_24_DSS.xlsx is not available")
class RealWorkbookTests(unittest.TestCase):
    def test_real_workbook_hash_and_fixture_equality(self):
        self.assertEqual(hashlib.sha256(REAL_XLSX.read_bytes()).hexdigest(), REAL_SHA256)
        real = gen.read_xlsx_cells(REAL_XLSX)
        fixture = gen.read_csv_cells(FIXTURE)
        for address in (
            "D17", "E17", "G17", "H17", "C23", "F23", "C24", "F24", "C26", "F26",
            "C29", "F29", "C30", "F30", "D31", "E31", "G31", "H31",
        ):
            self.assertEqual(str(real[address]), fixture[address], address)
        for axle in ("front", "rear"):
            self.assertEqual(gen.read_axle_cells(real, axle), gen.read_axle_cells(fixture, axle))


if __name__ == "__main__":
    unittest.main()
