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
PRIVATE_DAMPER_XLSX = ROOT / "IEM24 Files" / "MultiMatic Corner Dampers.xlsx"
PRIVATE_DAMPER_SHEET = "Corner Damper - VC01"
SOURCE_FINGERPRINT = "from real car data"
CORRECTION = (8, 300.0, 761.0)


def _process(velocity, clicks, force, **kwargs):
    return gen.process_damper_table(velocity, clicks, force, **kwargs)


def _read_private_damper_table(path, sheet):
    from openpyxl import load_workbook

    workbook = load_workbook(path, data_only=True, read_only=True)
    try:
        rows = list(workbook[sheet].iter_rows(values_only=True))
    finally:
        workbook.close()
    click_columns = list(range(1, 25, 2))
    clicks = np.asarray([rows[1][column] for column in click_columns], dtype=float)
    positive_velocity = np.asarray([rows[row][0] for row in range(5, 17)], dtype=float)
    signed_velocity = np.asarray([-500, -450, -400, -350, -300, -250, -200, -150, -100, -50, -25, -10, 0, 10, 25, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500], dtype=float)
    compression = np.asarray([[rows[row][column] for column in click_columns] for row in range(5, 17)], dtype=float).T
    rebound = np.asarray([[rows[row][column + 1] for column in click_columns] for row in range(5, 17)], dtype=float).T
    force = np.column_stack((rebound[:, ::-1], np.zeros((clicks.size, 1)), compression))
    velocity = signed_velocity
    return velocity, clicks, force


class DamperTableProcessingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.velocity, cls.clicks, cls.force = gen.read_csv_table(FIXTURE)

    def corrected(self):
        return _process(self.velocity, self.clicks, self.force, corrections=[CORRECTION], source_sha256=SOURCE_FINGERPRINT)

    def test_fixture_is_raw_table_with_anomaly(self):
        self.assertEqual(self.force.shape, (12, 25))
        self.assertEqual(self.velocity[0], -500.0)
        self.assertEqual(self.velocity[-1], 500.0)
        self.assertEqual(self.force[8, 20], 671.0)
        self.assertNotIn("/Users/", FIXTURE.read_text())

    def test_output_arrays_and_units(self):
        arrays = self.corrected()
        arrays["source_sha256"] = np.asarray(SOURCE_FINGERPRINT)
        arrays["corrections"] = np.asarray("[]")
        self.assertEqual(arrays["force_compression"].shape, (13, 12))
        self.assertEqual(arrays["force_rebound"].shape, (13, 12))
        self.assertEqual(arrays["velocity_compression"][0], 0.0)
        self.assertEqual(arrays["velocity_compression"][-1], 0.5)
        self.assertEqual(arrays["velocity_rebound"][0], -0.5)
        self.assertEqual(arrays["velocity_rebound"][-1], 0.0)
        self.assertEqual(arrays["valve_clicks"].tolist(), [12, 12])
        self.assertEqual(str(arrays["generator_version"]), "damper-lut-v1")
        self.assertEqual(str(arrays["source_sha256"]), SOURCE_FINGERPRINT)
        self.assertEqual(arrays["force_compression"][4, 5], 215.0)
        self.assertEqual(arrays["force_rebound"][8, 7], -267.0)

    def test_correction_is_recorded(self):
        with np.load(io.BytesIO((ROOT / "models/data/multimatic_dssv_vc01.npz").read_bytes()), allow_pickle=False) as archive:
            entry = json.loads(str(archive["corrections"]))[0]
        self.assertEqual(entry["click"], 8)
        self.assertAlmostEqual(entry["velocity_m_s"], 0.3)
        self.assertEqual((entry["from_N"], entry["to_N"]), (671.0, 761.0))
        self.assertEqual(entry["cell"], "V10")

    def test_corrected_table_is_monotone(self):
        arrays = self.corrected()
        for name, axis in (("force_compression", arrays["velocity_compression"]), ("force_rebound", arrays["velocity_rebound"])):
            magnitude = np.abs(arrays[name])
            speed = np.abs(axis)
            self.assertTrue(np.all(np.diff(magnitude, axis=1) >= 0.0), name)
            strict = speed >= 0.025 - 1.0e-12
            self.assertTrue(np.all(np.diff(magnitude, axis=1)[strict] > 0.0), name)
            order = np.argsort(speed)
            self.assertTrue(np.all(np.diff(magnitude, axis=1) >= 0.0), name)

    def test_ties_below_25_mm_s_are_physical(self):
        self.assertEqual(self.force[0, 11], self.force[1, 11])
        self.assertEqual(self.force[0, 12], self.force[1, 12])

    def test_without_correction_fails_naming_the_cell(self):
        with self.assertRaises(gen.DamperTableError) as caught:
            _process(self.velocity, self.clicks, self.force)
        self.assertIn("cell", str(caught.exception))

    def test_wrong_or_unneeded_correction_is_rejected(self):
        for correction in ((8, 300.0, 700.0), (8, 301.0, 761.0), (CORRECTION, (3, 100.0, 173.0))):
            with self.assertRaises(gen.DamperTableError):
                corrections = correction if isinstance(correction[0], tuple) else [correction]
                _process(self.velocity, self.clicks, self.force, corrections=corrections)

    def test_other_defects_are_detected(self):
        bad = self.force.copy()
        bad[5, 15] = bad[4, 15]
        with self.assertRaises(gen.DamperTableError):
            _process(self.velocity, self.clicks, bad, corrections=[CORRECTION])

    def test_unit_conversion_and_positive_rebound_sign(self):
        reference = self.corrected()
        flipped = self.force.copy()
        flipped[:, :12] *= -1.0
        arrays = _process(self.velocity * 1.0e-3, self.clicks, flipped / 1000.0,
                          velocity_unit="m/s", force_unit="kN", rebound_sign="positive",
                          corrections=[(8, 0.3, 0.761)], source_sha256=SOURCE_FINGERPRINT)
        for name in ("force_compression", "force_rebound", "velocity_compression", "velocity_rebound"):
            np.testing.assert_allclose(arrays[name], reference[name], rtol=1.0e-12)

    def test_npz_bytes_are_deterministic_and_loadable(self):
        first, second = gen.npz_bytes(self.corrected()), gen.npz_bytes(self.corrected())
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
        self.assertEqual([valve.name for valve in table.valves], ["compression", "rebound"])


class DamperGeneratorCliTests(unittest.TestCase):
    def run_cli(self, directory, *extra, source=FIXTURE):
        directory = Path(directory)
        command = [sys.executable, str(ROOT / "tools/model_data/generate_damper_lut.py"), str(source),
                   "--output-npz", str(directory / "data" / "damper.npz"),
                   "--output-xml-front", str(directory / "dampers" / "front.xml"),
                   "--output-xml-rear", str(directory / "dampers" / "rear.xml"), *extra]
        return subprocess.run(command, capture_output=True, text=True, cwd=ROOT)

    def test_cli_fails_without_correction_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_cli(directory)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((Path(directory) / "data" / "damper.npz").exists())

    def test_cli_is_atomic_deterministic_and_leaves_no_temporary_file(self):
        with tempfile.TemporaryDirectory() as directory:
            extra = ("--correct", "8,300,761")
            a = self.run_cli(directory, *extra)
            self.assertEqual(a.returncode, 0, a.stderr)
            path = Path(directory) / "data" / "damper.npz"
            first = path.read_bytes()
            b = self.run_cli(directory, *extra)
            self.assertEqual(b.returncode, 0, b.stderr)
            self.assertEqual(first, path.read_bytes())
            self.assertEqual(list(path.parent.iterdir()), [path])

    def test_fixture_generation_matches_committed_vc01(self):
        velocity, clicks, force = gen.read_csv_table(FIXTURE)
        arrays = gen.process_damper_table(velocity, clicks, force, corrections=[CORRECTION])
        with np.load(ROOT / "models/data/multimatic_dssv_vc01.npz", allow_pickle=False) as stored:
            for name in arrays:
                if name not in ("source_sha256", "corrections"):
                    np.testing.assert_array_equal(arrays[name], stored[name], err_msg=name)


@unittest.skipUnless(PRIVATE_DAMPER_XLSX.is_file(), "private real-car data is not available")
class PrivateDamperWorkbookTests(unittest.TestCase):
    def test_private_workbook_values_match_fixture(self):
        velocity, clicks, force = _read_private_damper_table(PRIVATE_DAMPER_XLSX, PRIVATE_DAMPER_SHEET)
        fixture_velocity, fixture_clicks, fixture_force = gen.read_csv_table(FIXTURE)
        np.testing.assert_array_equal(velocity, fixture_velocity)
        np.testing.assert_array_equal(clicks, fixture_clicks)
        np.testing.assert_array_equal(force, fixture_force)


if __name__ == "__main__":
    unittest.main()
