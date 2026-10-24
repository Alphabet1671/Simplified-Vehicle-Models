"""Focused canonical run manifest contract tests."""
from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from simplified_vehicle_models.solvers.run_manifest import (
    load_run_manifest_v1, write_run_manifest_v1,
)
from tests import test_trajectory_artifact as artifact_fixtures
from tools.run.solve_minimum_time_line import _write_trajectory_metadata


class RunManifestTests(unittest.TestCase):
    def sample(self, root):
        members = {}
        for name, content in (("track.xml", b"track"), ("formulation.xml", b"form"),
                              ("chart.npz", b"chart"), ("validation_report.json", b"{}"),
                              ("effective_request.json", b"{}"), ("runtime.json", b"{}"),
                              ("code.json", b"{}"), ("boundary_event_contract.json", b"{}"),
                              ("policy.json", b"{}")):
            (root / name).write_bytes(content)
            members[name] = hashlib.sha256(content).hexdigest()
        return {
            "track": {"path": "track.xml", "sha256": members["track.xml"]},
            "formulation": {"path": "formulation.xml", "sha256": members["formulation.xml"]},
            "chart": {"path": "chart.npz", "sha256": members["chart.npz"]},
            "transitive_dependencies": [],
            "effective_request": {"path": "effective_request.json", "sha256": members["effective_request.json"]},
            "runtime": {"path": "runtime.json", "sha256": members["runtime.json"]},
            "code": {"path": "code.json", "sha256": members["code.json"]},
            "boundary_event_contract": {"path": "boundary_event_contract.json", "sha256": members["boundary_event_contract.json"]},
            "validation_report": {"path": "validation_report.json", "sha256": members["validation_report.json"]},
            "policy": {"path": "policy.json", "sha256": members["policy.json"], "status": "unqualified"},
        }

    def test_write_and_reload_binds_exact_loader_member(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            solution, _ = artifact_fixtures.TrajectoryArtifactTest().make_solution()
            _write_trajectory_metadata(root / "trajectory_metadata.npz", solution)
            write_run_manifest_v1(root, self.sample(root))
            loaded = load_run_manifest_v1(root)
            self.assertEqual(loaded["trajectory"]["filename"], "trajectory_metadata.npz")
            self.assertTrue(loaded["trajectory"]["sha256"])
            self.assertFalse(loaded["qualification"]["approved"])
            (root / "track.xml").write_bytes(b"tampered")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                load_run_manifest_v1(root)

    def test_manifest_refuses_conflicting_same_member_hashes(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            solution, _ = artifact_fixtures.TrajectoryArtifactTest().make_solution()
            _write_trajectory_metadata(root / "trajectory_metadata.npz", solution)
            data = self.sample(root)
            data["transitive_dependencies"] = [{"path": "track.xml", "sha256": "0" * 64}]
            with self.assertRaisesRegex(ValueError, "conflicting hashes"):
                write_run_manifest_v1(root, data)

    def test_manifest_refuses_conflicting_identity_document(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            for name in ("track.xml", "formulation.xml", "chart.npz", "validation_report.json",
                         "runtime.json", "code.json", "boundary_event_contract.json", "policy.json"):
                (root / name).write_text("fixture\\n")
            self.sample(root)
            (root / "effective_request.json").write_text("{}\\n")
            with self.assertRaisesRegex(ValueError, "conflicts"):
                from simplified_vehicle_models.solvers.run_manifest import build_manifest_identities
                build_manifest_identities(root, track="track.xml", formulation="formulation.xml",
                    chart="chart.npz", transitive_dependencies=[], effective_request={"other": 1},
                    runtime={"python": "pinned"}, code={"commit": "x"},
                    boundary_event_contract={"boundary": "rolling"},
                    validation_report="validation_report.json", policy={"status": "unqualified"})

    def test_manifest_wrongtrajectoryformat_rejected_after_digest_rewritten(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            solution, _ = artifact_fixtures.TrajectoryArtifactTest().make_solution()
            _write_trajectory_metadata(root / "trajectory_metadata.npz", solution)
            write_run_manifest_v1(root, self.sample(root))
            target = root / "trajectory_metadata.npz"
            target.write_bytes(b"malformed")
            data = json.loads((root / "run_manifest_v1.json").read_text())
            data["trajectory"]["sha256"] = hashlib.sha256(target.read_bytes()).hexdigest()
            (root / "run_manifest_v1.json").write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                load_run_manifest_v1(root)

    def test_missing_member_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            with self.assertRaises((ValueError, FileNotFoundError)):
                write_run_manifest_v1(root, self.sample(root))

    def test_traversal_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            solution, _ = artifact_fixtures.TrajectoryArtifactTest().make_solution()
            _write_trajectory_metadata(root / "trajectory_metadata.npz", solution)
            data = self.sample(root)
            data["track"]["path"] = "../outside.xml"
            with self.assertRaises(ValueError):
                write_run_manifest_v1(root, data)

    def test_symlink_escape_rejected(self):
        with tempfile.TemporaryDirectory() as td, tempfile.TemporaryDirectory() as outside:
            root, outside_root = Path(td), Path(outside)
            solution, _ = artifact_fixtures.TrajectoryArtifactTest().make_solution()
            _write_trajectory_metadata(root / "trajectory_metadata.npz", solution)
            data = self.sample(root)
            (outside_root / "external.xml").write_bytes(b"out")
            (root / "escaped.xml").symlink_to(outside_root / "external.xml")
            data["track"] = {"path": "escaped.xml", "sha256": hashlib.sha256(b"out").hexdigest()}
            with self.assertRaises(ValueError):
                write_run_manifest_v1(root, data)

    def test_partial_manifest_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "run_manifest_v1.json").write_text("{\"schema\":", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_run_manifest_v1(root)

    def test_wrong_npz_format_rejected_even_when_renamed(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "trajectory_metadata.npz").write_bytes(b"node arrays")
            with self.assertRaisesRegex(ValueError, "invalid loader trajectory"):
                write_run_manifest_v1(root, self.sample(root))

    def test_wrong_node_solution_filename_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "solution.npz").write_bytes(b"node arrays")
            with self.assertRaises(ValueError):
                write_run_manifest_v1(root, self.sample(root), trajectory_path="solution.npz")
