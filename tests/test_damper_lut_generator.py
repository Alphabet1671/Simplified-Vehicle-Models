import hashlib
import io
import json
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

import numpy as np

from simplified_vehicle_models.suspension.damper_table import load_damper_npz
from tools.model_data import generate_damper_lut as gen

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "damper_vc01.csv"
REAL_XLSX = Path(
    "/Users/alphabet1671/Dev/FSAE-VD-Personal-Scripts/Vertical Dynamics/DamperTable.xlsx"
)
REAL_SHA256 = "1b6322caaeaf83f507a5d2256ca4652932c1b1bf05e06d8212f60e6114011d32"
CORRECTION = (8, 300.0, 761.0)


def _process(velocity, clicks, force, **kwargs):
    return gen.process_damper_table(velocity, clicks, force, **kwargs)


class DamperTableProcessingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.velocity, cls.clicks, cls.force = gen.read_csv_table(FIXTURE)

    def corrected(self):
        return _process(
            self.velocity, self.clicks, self.force,
            corrections=[CORRECTION], source_sha256=REAL_SHA256,
        )

    def test_fixture_is_raw_sheet_data_with_the_defect(self):
        self.assertEqual(self.force.shape, (12, 25))
        self.assertEqual(self.velocity[0], -500.0)
        self.assertEqual(self.velocity[-1], 500.0)
        self.assertEqual(self.force[8, 20], 671.0)  # sheet V10
        text = FIXTURE.read_text()
        self.assertIn(REAL_SHA256, text.split("click,")[0])

    def test_output_arrays_and_units(self):
        arrays = self.corrected()
        self.assertEqual(arrays["force_compression"].shape, (13, 12))
        self.assertEqual(arrays["force_rebound"].shape, (13, 12))
        self.assertEqual(arrays["velocity_compression"][0], 0.0)
        self.assertEqual(arrays["velocity_compression"][-1], 0.5)
        self.assertEqual(arrays["velocity_rebound"][0], -0.5)
        self.assertEqual(arrays["velocity_rebound"][-1], 0.0)
        self.assertEqual(arrays["valve_clicks"].tolist(), [12, 12])
        self.assertEqual(str(arrays["generator_version"]), "damper-lut-v1")
        self.assertEqual(str(arrays["source_sha256"]), REAL_SHA256)
        # Exact table values: click 5 at +100 mm/s and click 7 at -100 mm/s.
        self.assertEqual(arrays["force_compression"][4, 5], 215.0)
        self.assertEqual(arrays["force_rebound"][8, 7], -267.0)

    def test_correction_is_recorded(self):
        corrections = json.loads(str(self.corrected()["corrections"]))
        self.assertEqual(len(corrections), 1)
        entry = corrections[0]
        self.assertEqual(entry["click"], 8)
        self.assertAlmostEqual(entry["velocity_m_s"], 0.3)
        self.assertEqual((entry["from_N"], entry["to_N"]), (671.0, 761.0))
        self.assertEqual(entry["cell"], "V10")

    def test_corrected_table_is_monotone(self):
        arrays = self.corrected()
        for name, axis in (
            ("force_compression", arrays["velocity_compression"]),
            ("force_rebound", arrays["velocity_rebound"]),
        ):
            magnitude = np.abs(arrays[name])
            speed = np.abs(axis)
            # Non-decreasing with click everywhere.
            self.assertTrue(np.all(np.diff(magnitude, axis=1) >= 0.0), name)
            # Strictly increasing with click wherever |v| >= 25 mm/s.
            strict = speed >= 0.025 - 1.0e-12
            self.assertTrue(np.all(np.diff(magnitude, axis=1)[strict] > 0.0), name)
            # Strictly increasing with |v| for each click.
            order = np.argsort(speed)
            self.assertTrue(np.all(np.diff(magnitude[order], axis=0) > 0.0), name)
            self.assertTrue(np.all(magnitude[speed == 0.0] == 0.0))

    def test_ties_below_25_mm_s_are_physical(self):
        arrays = self.corrected()
        compression = arrays["force_compression"]
        self.assertEqual(compression[1, 0], compression[1, 1])  # +10 mm/s, clicks 0 and 1
        self.assertEqual(compression[1, 2], 17.0)  # click 2 at +10 mm/s

    def test_without_correction_fails_naming_the_cell(self):
        with self.assertRaises(gen.DamperTableError) as caught:
            _process(self.velocity, self.clicks, self.force)
        message = str(caught.exception)
        self.assertIn("V10", message)
        self.assertIn("click 8", message)
        self.assertIn("+300", message)

    def test_wrong_or_unneeded_correction_is_rejected(self):
        with self.assertRaises(gen.DamperTableError):
            _process(self.velocity, self.clicks, self.force, corrections=[(8, 300.0, 700.0)])
        with self.assertRaises(gen.DamperTableError):
            _process(self.velocity, self.clicks, self.force, corrections=[(8, 301.0, 761.0)])
        with self.assertRaises(gen.DamperTableError):
            _process(self.velocity, self.clicks, self.force, corrections=[CORRECTION, (3, 100.0, 173.0)])

    def test_other_defects_are_detected(self):
        bad = self.force.copy()
        bad[:, 14] = bad[:, 14][::-1]  # +25 mm/s no longer increasing with click
        with self.assertRaises(gen.DamperTableError):
            _process(self.velocity, self.clicks, bad, corrections=[CORRECTION])
        bad = self.force.copy()
        bad[3, 15] = 10.0  # not increasing with |v|
        with self.assertRaises(gen.DamperTableError):
            _process(self.velocity, self.clicks, bad, corrections=[CORRECTION])

    def test_unit_conversion_and_positive_rebound_sign(self):
        reference = self.corrected()
        flipped = self.force.copy()
        flipped[:, :12] *= -1.0
        arrays = _process(
            self.velocity * 1.0e-3, self.clicks, flipped / 1000.0,
            velocity_unit="m/s", force_unit="kN", rebound_sign="positive",
            corrections=[(8, 0.3, 0.761)], source_sha256=REAL_SHA256,
        )
        for name in ("force_compression", "force_rebound", "velocity_compression", "velocity_rebound"):
            np.testing.assert_allclose(arrays[name], reference[name], rtol=1.0e-12)

    def test_npz_bytes_are_deterministic_and_loadable(self):
        first = gen.npz_bytes(self.corrected())
        second = gen.npz_bytes(self.corrected())
        self.assertEqual(hashlib.sha256(first).digest(), hashlib.sha256(second).digest())
        with zipfile.ZipFile(io.BytesIO(first)) as archive:
            for info in archive.infolist():
                self.assertEqual(info.date_time, (1980, 1, 1, 0, 0, 0))
                self.assertEqual(info.compress_type, zipfile.ZIP_STORED)
        self.assertNotIn(str(ROOT).encode(), first)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "t.npz"
            path.write_bytes(first)
            table = load_damper_npz(path, hashlib.sha256(first).hexdigest())
        self.assertEqual([v.name for v in table.valves], ["compression", "rebound"])


class DamperGeneratorCliTests(unittest.TestCase):
    def run_cli(self, directory, *extra, source=FIXTURE):
        directory = Path(directory)
        command = [
            sys.executable, str(ROOT / "tools/model_data/generate_damper_lut.py"), str(source),
            "--output-npz", str(directory / "data" / "damper.npz"),
            "--output-xml-front", str(directory / "dampers" / "front.xml"),
            "--output-xml-rear", str(directory / "dampers" / "rear.xml"),
            *extra,
        ]
        return subprocess.run(command, capture_output=True, text=True, cwd=ROOT)

    def test_cli_fails_without_correction_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_cli(directory)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("V10", result.stderr)
            self.assertFalse((Path(directory) / "data").exists())

    def test_cli_is_atomic_deterministic_and_leaves_no_temporary_file(self):
        digests = []
        for _ in range(2):
            with tempfile.TemporaryDirectory() as directory:
                result = self.run_cli(directory, "--correct", "8,300,761")
                self.assertEqual(result.returncode, 0, result.stderr)
                data = Path(directory) / "data"
                self.assertEqual(sorted(p.name for p in data.iterdir()), ["damper.npz"])
                self.assertEqual(
                    sorted(p.name for p in (Path(directory) / "dampers").iterdir()),
                    ["front.xml", "rear.xml"],
                )
                digest = hashlib.sha256((data / "damper.npz").read_bytes()).hexdigest()
                self.assertIn(digest, result.stdout)
                self.assertIn("generate_damper_lut.py", result.stdout)
                front = (Path(directory) / "dampers" / "front.xml").read_text()
                self.assertIn('subtype="table-front"', front)
                self.assertIn(f'sha256="{digest}"', front)
                self.assertIn('path="../data/damper.npz"', front)
                self.assertIn('subtype="table-rear"', (Path(directory) / "dampers" / "rear.xml").read_text())
                digests.append(digest)
        self.assertEqual(digests[0], digests[1])

    def test_committed_vc01_matches_fixture_generation(self):
        committed = (ROOT / "models" / "data" / "multimatic_dssv_vc01.npz").read_bytes()
        velocity, clicks, force = gen.read_csv_table(FIXTURE)
        arrays = gen.process_damper_table(
            velocity, clicks, force, corrections=[CORRECTION], source_sha256=REAL_SHA256
        )
        self.assertEqual(gen.npz_bytes(arrays), committed)


@unittest.skipUnless(REAL_XLSX.is_file(), "source DamperTable.xlsx is not available")
class RealWorkbookTests(unittest.TestCase):
    def test_real_workbook_matches_fixture_and_hash(self):
        self.assertEqual(hashlib.sha256(REAL_XLSX.read_bytes()).hexdigest(), REAL_SHA256)
        real = gen.read_xlsx_table(REAL_XLSX, gen.DEFAULT_SHEET)
        fixture = gen.read_csv_table(FIXTURE)
        for a, b in zip(real, fixture):
            np.testing.assert_array_equal(a, b)

    def test_real_workbook_same_bytes_as_fixture(self):
        velocity, clicks, force = gen.read_xlsx_table(REAL_XLSX, gen.DEFAULT_SHEET)
        with self.assertRaises(gen.DamperTableError) as caught:
            gen.process_damper_table(velocity, clicks, force)
        self.assertIn("V10", str(caught.exception))
        arrays = gen.process_damper_table(
            velocity, clicks, force, corrections=[CORRECTION], source_sha256=REAL_SHA256
        )
        committed = (ROOT / "models" / "data" / "multimatic_dssv_vc01.npz").read_bytes()
        self.assertEqual(gen.npz_bytes(arrays), committed)


if __name__ == "__main__":
    unittest.main()
