"""Safety tests for maneuver artifact persistence."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from tools.maneuver_gui.artifacts import load_artifact, write_artifact


def sample_definition():
    return {
        "schema": "maneuver-v1",
        "name": "test",
        "vehicle": {"model": "vehicle-hash-input"},
        "calibration": {"gain": 2.0},
    }


def sample_result():
    return SimpleNamespace(
        time_s=np.array([0.0, 0.1]),
        state=np.array([[1.0], [2.0]]),
        state_rate=np.array([[10.0], [10.0]]),
        execution_outcome="completed",
        termination_reason="duration",
    )


class ManeuverArtifactTests(unittest.TestCase):
    def test_generated_run_directory_and_manifest_hashes(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp) / "output"
            run_dir = write_artifact(base, sample_definition(), sample_result())
            self.assertEqual(run_dir.parent, base.resolve())
            self.assertNotEqual(run_dir.name, "output")
            manifest, arrays = load_artifact(run_dir)
            self.assertEqual(manifest["schema"], "maneuver-artifacts-v1")
            self.assertEqual(manifest["version"], 1)
            self.assertEqual(manifest["execution_outcome"], "completed")
            self.assertEqual(
                set(manifest["hashes"]),
                {"definition_sha256", "calibration_sha256", "model_sha256", "result_sha256"},
            )
            np.testing.assert_array_equal(arrays["time_s"], [0.0, 0.1])
            self.assertFalse((Path(temp) / "run_record.npz").exists())

    def test_duplicate_write_rejected_without_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = write_artifact(temp, sample_definition(), sample_result())
            before = (run_dir / "manifest.json").read_bytes()
            with self.assertRaises(FileExistsError):
                write_artifact(temp, sample_definition(), sample_result(), run_id=run_dir.name)
            self.assertEqual((run_dir / "manifest.json").read_bytes(), before)

    def test_required_base_and_unsafe_custom_run_names(self):
        with self.assertRaises(TypeError):
            write_artifact(definition=sample_definition(), result=sample_result())
        for name in ("../escape", "/tmp/escape", "a/b", ".."):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temp:
                with self.assertRaises(ValueError):
                    write_artifact(temp, sample_definition(), sample_result(), run_id=name)
                self.assertEqual(list(Path(temp).iterdir()), [])

    def test_missing_corrupt_npz_and_manifest_schema_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = write_artifact(temp, sample_definition(), sample_result())
            (run_dir / "run_record.npz").unlink()
            with self.assertRaises(ValueError):
                load_artifact(run_dir)
            run_dir = write_artifact(temp, sample_definition(), sample_result())
            (run_dir / "run_record.npz").write_bytes(b"not an npz")
            with self.assertRaises(ValueError):
                load_artifact(run_dir)
            run_dir = write_artifact(temp, sample_definition(), sample_result())
            manifest_path = run_dir / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["schema"] = "unexpected"
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                load_artifact(run_dir)


if __name__ == "__main__":
    unittest.main()
