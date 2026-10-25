"""Lightweight integrity checks for frozen physical yaw-profile fixtures."""
import hashlib
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures/yaw_profile"


class YawProfileFixtureTests(unittest.TestCase):
    def test_physical_fixture_evidence_and_manifest_hashes(self):
        manifest = json.loads((FIXTURES / "manifest.json").read_text())
        entries = {item["file"]: item for item in manifest["fixtures"]}
        physical = [name for name, item in entries.items() if name.endswith("-window.json") and item.get("source_output", "").startswith("runs/yaw_aware_driver/task1-probes-structured/")]
        self.assertEqual(len(physical), 8)
        for name in physical:
            path = FIXTURES / name
            artifact = json.loads(path.read_text())
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), entries[name]["sha256"])
            self.assertEqual(artifact["fixture_type"], "physical_failed_profile_window")
            self.assertEqual(artifact["source_output_sha256"], entries[name]["source_output_sha256"])
            self.assertEqual(artifact["source_output"], entries[name]["source_output"])
            self.assertTrue(artifact["reproduction"]["command"])
            self.assertTrue(artifact["reproduction"]["settings"])
            evidence = artifact["failure"]["segment_evidence"]
            self.assertTrue(evidence["candidates"])
            self.assertEqual(evidence["segment_index"], artifact["failure"]["failed_segment"])
        self.assertIn("model_revision", manifest)
        self.assertIn("generation_command", manifest)
        self.assertEqual(len(manifest["comparisons"]), 4)
        self.assertTrue(all(item["anchoring_alone_removes_failure"] is False for item in manifest["comparisons"]))


if __name__ == "__main__":
    unittest.main()
