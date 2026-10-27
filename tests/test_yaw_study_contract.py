import unittest
import tempfile
import hashlib
from pathlib import Path

from tools.run.yaw_study_contract import comparison_identity, finalized_plan_inputs, study_contract, validate_comparison_pair, _hash_mapping


def complete_contract(**overrides):
    values = {"geometry_spacing_m": 1.0, "control_spacing_m": 40.0,
        "yaw_moment_tolerance": 1e-7, "interval_q_tolerance":1e-8, "acceleration_tolerance":1e-7, "map_identity": "map-a", "map_file_sha256": "maphash",
        "map_source_identity": "map-source", "track_identity": "track-a", "track_data_hashes": {"models/tracks/track.xml": "a" * 64},
        "vehicle_configuration_hashes": {"models/vehicles/vehicle.xml": "b" * 64}, "boundary_type": "flying",
        "maximum_speed_mps": 70.0, "warm_start_policy": "cold", "warm_start_digest": "none",
        "solver": {"method": "BDF"}, "duration": 150.0, "maximum_step": .05,
        "rtol": 1e-6, "atol": 1e-8, "method": "BDF", "initial_state": "unknown",
        "initial_state_soc": "unknown", "replay_treatment":"preview"}
    values.update(overrides)
    return values


class StudyContractTests(unittest.TestCase):
    def test_dimension_allowlists_and_output_path_exclusion(self):
        base = dict(complete_contract(), vehicle_source_identity="vehicle-a", output_dir="/tmp/a")
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
        cold = dict(base, warm_start_policy="cold", warm_start_digest="none")
        warm = dict(cold, warm_start_policy="squared_speed_prolongation", warm_start_digest="digest-a")
        self.assertNotEqual(comparison_identity(cold, "geometry"), comparison_identity(warm, "geometry"))
        with self.assertRaisesRegex(ValueError, "warm_start_(policy|digest)"):
            validate_comparison_pair(cold, warm, "geometry")

    def test_hash_mapping_requires_normalized_source_paths(self):
        digest = "a" * 64
        self.assertTrue(_hash_mapping({"src/module.py": digest}))
        self.assertTrue(_hash_mapping({"/workspace/src/module.py": digest}))
        for key in ("file", "", " src/file.py", "src/file.py ", "src/../file.py",
                    "src/./file.py", "src//file.py", "./src/file.py", "src\\\\file.py"):
            with self.subTest(key=key):
                self.assertFalse(_hash_mapping({key: digest}))

    def test_contract_marks_incomplete_source_historical_unknown(self):
        contract = study_contract({}, {}, {})
        self.assertEqual(contract["contract_version"], "yaw-study-contract-v1")
        self.assertEqual(contract["provenance"]["source_revision"], "unknown")
        self.assertEqual(contract["provenance"]["source_provenance_status"], "historical_unknown")
        partial = study_contract({}, {}, {"source_revision":"abc", "source_dirty_hashes":"unknown",
                                         "source_provenance_status":"attested"})
        self.assertEqual(partial["provenance"]["source_provenance_status"], "historical_unknown")
        spoofed = study_contract({}, {}, {"source_revision":"abc", "source_dirty_hashes":{},
                                         "source_provenance_status":"attested"})
        self.assertEqual(spoofed["provenance"]["source_provenance_status"], "historical_unknown")
        malformed = study_contract({}, {}, {"source_revision":"abc", "source_dirty_hashes":"garbage",
                                           "source_provenance_status":"attested"})
        self.assertEqual(malformed["provenance"]["source_provenance_status"], "historical_unknown")
        fabricated = study_contract({}, {}, {"source_revision":"abc", "source_dirty_hashes":{"file": "a" * 64},
                                            "source_provenance_status":"attested"})
        self.assertEqual(fabricated["provenance"]["source_provenance_status"], "historical_unknown")
        self.assertEqual(len(contract["canonical_json_sha256"]), 64)

    def test_geometry_refinement_rejects_control_spacing_change(self):
        base = complete_contract()
        invalid_fine = dict(base, geometry_spacing_m=0.5, control_spacing_m=20.0)
        with self.assertRaisesRegex(ValueError, "control_spacing_m"):
            validate_comparison_pair(base, invalid_fine, "geometry")

    def test_partial_contracts_fail_closed(self):
        base = complete_contract()
        for dimension, omitted in (("geometry", "boundary_type"), ("control", "vehicle_configuration_hashes"),
                                   ("replay", "initial_state_soc")):
            partial = dict(base); partial.pop(omitted)
            with self.subTest(dimension=dimension, omitted=omitted):
                with self.assertRaisesRegex(ValueError, "missing required fields"):
                    comparison_identity(partial, dimension)
                with self.assertRaises(ValueError):
                    validate_comparison_pair(partial, partial, dimension)
        malformed = dict(base, track_data_hashes={})
        with self.assertRaisesRegex(ValueError, "track_data_hashes lacks attestation"):
            comparison_identity(malformed, "geometry")
        malformed = dict(base, vehicle_configuration_hashes="garbage")
        with self.assertRaisesRegex(ValueError, "vehicle_configuration_hashes lacks attestation"):
            comparison_identity(malformed, "geometry")

    def test_replay_manifest_update_records_exact_initialized_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "inputs.json"
            path.write_text(__import__('json').dumps({"study_contract": {"provenance": {"initial_state": "unknown"}}}))
            supplied = [1.0, 12.5, 0.25]
            from tools.run.yaw_study_contract import update_replay_initial_state
            updated = update_replay_initial_state(path, supplied, ["s", "v_x", "soc"])
            provenance = updated["study_contract"]["provenance"]
            self.assertEqual(provenance["initial_state"], {"names": ["s", "v_x", "soc"], "values": supplied})
            self.assertEqual(provenance["initial_state_soc"], .25)
            self.assertEqual(__import__('json').loads(path.read_text()), updated)

    def test_plan_digests_are_run_bindings_not_geometry_identity(self):
        coarse = complete_contract(geometry_spacing_m=2.0, plan_array_digest="array-a", plan_file_digest="file-a")
        fine = complete_contract(geometry_spacing_m=1.0, plan_array_digest="array-b", plan_file_digest="file-b")
        self.assertEqual(comparison_identity(coarse, "geometry"), comparison_identity(fine, "geometry"))
        validate_comparison_pair(coarse, fine, "geometry")

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
