import json
import subprocess
import sys
import unittest
from pathlib import Path

from simplified_vehicle_models.controls.maneuvers.csv_source import CsvMapping, load_csv_source
from simplified_vehicle_models.controls.maneuvers.types import ManeuverDefinition

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "docs/dev/examples/step-steer.maneuver.json"


class ManeuverDocumentationTests(unittest.TestCase):
    def test_example_is_strict_schema_definition_and_runner_help_works(self):
        data = json.loads(EXAMPLE.read_text())
        definition = ManeuverDefinition.from_dict(data)
        self.assertEqual(definition.to_dict(), data)
        result = subprocess.run([sys.executable, "-m", "tools.maneuver_gui.runner", "--help"],
                                cwd=ROOT, capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--definition", result.stdout)

    def test_csv_example_contract_imports_strict_three_channels(self):
        source = ROOT / "docs/dev/examples/irregular-three-channel.csv"
        mapping = CsvMapping("time", "swa", "accelerator", "brake", "s", "rad", "fraction", "hold")
        loaded = load_csv_source(source, mapping)
        self.assertGreaterEqual(len(loaded.commands), 2)
        self.assertEqual(loaded.mapping, mapping)


if __name__ == "__main__":
    unittest.main()
