import re
import shutil
import tempfile
import unittest
from pathlib import Path

from simplified_vehicle_models.compiler.capabilities import (
    DAMPER_FORCE,
    FRONT_DAMPER_FORCE,
    REAR_DAMPER_FORCE,
    DamperForceSource,
)
from simplified_vehicle_models.compiler.registry import component_descriptor
from simplified_vehicle_models.suspension.components import CompiledDamper, compile_damper
from simplified_vehicle_models.xml.loader import load_model

ROOT = Path(__file__).resolve().parents[1]
FRONT = ROOT / "models" / "dampers" / "multimatic_dssv_vc01_front.xml"
REAR = ROOT / "models" / "dampers" / "multimatic_dssv_vc01_rear.xml"
NPZ = ROOT / "models" / "data" / "multimatic_dssv_vc01.npz"


class DamperComponentTests(unittest.TestCase):
    def test_registration_and_distinct_capabilities(self):
        self.assertEqual(component_descriptor("damper-model", "table-front").capability, FRONT_DAMPER_FORCE)
        self.assertEqual(component_descriptor("damper-model", "table-rear").capability, REAR_DAMPER_FORCE)
        self.assertEqual(len({DAMPER_FORCE, FRONT_DAMPER_FORCE, REAR_DAMPER_FORCE}), 3)

    def test_both_xmls_pin_the_same_npz(self):
        texts = [path.read_text() for path in (FRONT, REAR)]
        digests = {re.search(r'sha256="([0-9a-f]{64})"', text).group(1) for text in texts}
        self.assertEqual(len(digests), 1)
        for text in texts:
            self.assertIn('path="../data/multimatic_dssv_vc01.npz"', text)
            self.assertIn('name="click_compression" value="5" unit="1"', text)
            self.assertIn('name="click_rebound" value="7" unit="1"', text)

    def test_exact_baseline_forces(self):
        for path in (FRONT, REAR):
            damper = compile_damper(load_model(path))
            self.assertIsInstance(damper, CompiledDamper)
            self.assertIsInstance(damper, DamperForceSource)
            plus = damper.force(0.1)
            minus = damper.force(-0.1)
            self.assertEqual(float(plus.force), 215.0)
            self.assertEqual(float(minus.force), -267.0)
            self.assertFalse(bool(plus.out_of_range))
            self.assertEqual(float(damper.force(0.0).force), 0.0)
            self.assertGreater(float(plus.slope), 0.0)

    def test_corrected_cell_is_in_the_table(self):
        damper = compile_damper(load_model(FRONT))
        self.assertEqual(damper.table.compression_force[8, 8], 761.0)  # +0.3 m/s, click 8

    def _copy(self, directory, text_edit):
        directory = Path(directory)
        (directory / "data").mkdir()
        (directory / "dampers").mkdir()
        shutil.copy(NPZ, directory / "data" / NPZ.name)
        target = directory / "dampers" / FRONT.name
        target.write_text(text_edit(FRONT.read_text()))
        return target

    def test_changed_sha256_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._copy(directory, lambda t: re.sub(r'sha256="[0-9a-f]{64}"', 'sha256="' + "0" * 64 + '"', t))
            with self.assertRaisesRegex(ValueError, "sha256 mismatch"):
                compile_damper(load_model(path))

    def test_click_out_of_range_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._copy(directory, lambda t: t.replace('name="click_rebound" value="7"', 'name="click_rebound" value="12"'))
            with self.assertRaisesRegex(ValueError, "outside 0..11"):
                compile_damper(load_model(path))

    def test_fractional_click_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._copy(directory, lambda t: t.replace('name="click_rebound" value="7"', 'name="click_rebound" value="6.5"'))
            with self.assertRaisesRegex(ValueError, "integer"):
                compile_damper(load_model(path))

    def test_missing_or_extra_click_parameter_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._copy(directory, lambda t: t.replace('name="click_rebound"', 'name="click_rebound_high"'))
            with self.assertRaisesRegex(ValueError, "missing=.*click_rebound.*unknown=.*click_rebound_high"):
                compile_damper(load_model(path))
        with tempfile.TemporaryDirectory() as directory:
            path = self._copy(
                directory,
                lambda t: t.replace("</parameters>", '  <parameter name="click_extra" value="1" unit="1"/>\n  </parameters>'),
            )
            with self.assertRaisesRegex(ValueError, "unknown=.*click_extra"):
                compile_damper(load_model(path))

    def test_wrong_unit_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._copy(directory, lambda t: t.replace('name="click_rebound" value="7" unit="1"', 'name="click_rebound" value="7" unit="m"'))
            with self.assertRaisesRegex(ValueError, "unit"):
                compile_damper(load_model(path))

    def test_missing_data_declaration_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._copy(directory, lambda t: re.sub(r"\s*<data [^>]*/>", "", t))
            with self.assertRaisesRegex(ValueError, "damper-table"):
                compile_damper(load_model(path))


if __name__ == "__main__":
    unittest.main()
