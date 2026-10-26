import unittest
import tempfile
import hashlib
from pathlib import Path

from tools.run.yaw_study_contract import comparison_identity, finalized_plan_inputs, study_contract, validate_comparison_pair


class StudyContractTests(unittest.TestCase):
    def test_dimension_allowlists_and_output_path_exclusion(self):
        base = {"geometry_spacing_m": 1.0, "control_spacing_m": 40.0,
                "yaw_moment_tolerance": 1e-7, "map_identity": "map-a",
                "vehicle_source_identity": "vehicle-a", "output_dir": "/tmp/a"}
        moved = dict(base, output_dir="/tmp/b")
        self.assertEqual(comparison_identity(base, "geometry"), comparison_identity(moved, "geometry"))
        refined = dict(base, geometry_spacing_m=0.5)
        self.assertEqual(comparison_identity(base, "geometry"), comparison_identity(refined, "geometry"))
        self.assertNotEqual(comparison_identity(base, "geometry"), comparison_identity(
            dict(refined, yaw_moment_tolerance=1e-6), "geometry"))
        self.assertNotEqual(comparison_identity(base, "geometry"), comparison_identity(
            dict(refined, vehicle_source_identity="vehicle-b"), "geometry"))
        with self.assertRaises(ValueError):
            comparison_identity(base, "other")

    def test_contract_marks_incomplete_source_historical_unknown(self):
        contract = study_contract({}, {}, {})
        self.assertEqual(contract["contract_version"], "yaw-study-contract-v1")
        self.assertEqual(contract["provenance"]["source_revision"], "unknown")
        self.assertEqual(contract["provenance"]["source_provenance_status"], "historical_unknown")
        partial = study_contract({}, {}, {"source_revision":"abc", "source_dirty_hashes":"unknown"})
        self.assertEqual(partial["provenance"]["source_provenance_status"], "historical_unknown")
        self.assertEqual(len(contract["canonical_json_sha256"]), 64)

    def test_geometry_refinement_rejects_control_spacing_change(self):
        base = {"geometry_spacing_m": 1.0, "control_spacing_m": 40.0,
                "yaw_moment_tolerance": 1e-7, "map_identity": "map-a"}
        invalid_fine = dict(base, geometry_spacing_m=0.5, control_spacing_m=20.0)
        with self.assertRaisesRegex(ValueError, "control_spacing_m"):
            validate_comparison_pair(base, invalid_fine, "geometry")

    def test_finalized_plan_inputs_binds_immutable_plan(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plan.npz"
            path.write_bytes(b"immutable-plan")
            result = finalized_plan_inputs({}, {}, {"plan_file_digest": "pending", "plan_array_digest": "pending"},
                                           path, "array-digest", map_identity="map", track_identity="track")
            provenance = result["study_contract"]["provenance"]
            self.assertEqual(provenance["plan_array_digest"], "array-digest")
            self.assertEqual(provenance["plan_file_digest"], hashlib.sha256(path.read_bytes()).hexdigest())
            self.assertNotIn("pending", provenance.values())


if __name__ == "__main__":
    unittest.main()
