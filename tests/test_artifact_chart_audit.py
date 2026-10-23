"""Focused tests for bounded all-element cached-extension chart aggregation."""
import json
import unittest
from dataclasses import replace
from unittest.mock import patch

from simplified_vehicle_models.solvers.artifact_chart_audit import (
    ArtifactAuditSettings, audit_artifact_chart,
)
from simplified_vehicle_models.solvers.chart_element_certificate import ChartElementSettings
from simplified_vehicle_models.solvers import artifact_chart_check
from tests.test_artifact_chart_check import make_solution, chart
from tools.run.solve_minimum_time_line import _write_trajectory_metadata
from simplified_vehicle_models.solvers.trajectory_artifact import load_trajectory_artifact
import tempfile
from pathlib import Path


class ArtifactChartAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / "small.npz"
        _write_trajectory_metadata(self.path, make_solution())
        self.loaded = load_trajectory_artifact(self.path)

    def tearDown(self):
        self.temp.cleanup()

    def test_real_three_element_artifact_audits_all_stored_math_only(self):
        result = audit_artifact_chart(self.loaded, chart())
        self.assertTrue(result["all_elements_chart_constraints_proven"], result)
        self.assertEqual(result["requested_element_count"], 5)
        self.assertEqual(result["attempted_element_count"], 5)
        self.assertEqual(result["proven_element_count"], 5)
        self.assertEqual(result["unattempted_element_indices"], [])
        self.assertEqual(result["scope"], "stored_chart_mathematics_over_all_source_elements_only")
        for key in ("accepted", "solver_artifact_qualified", "physical_feasibility_certified",
                    "original_vehicle_validated", "event_qualified", "U3_qualified", "policy_approved"):
            self.assertFalse(result[key])
        self.assertTrue(result["finite_public_eval_not_certified"])
        json.dumps(result, allow_nan=False)

    def test_failure_exception_partial_and_early_stop_never_claim_whole(self):
        from simplified_vehicle_models.solvers.artifact_chart_audit import _snapshot
        _, fingerprint = _snapshot(self.loaded)
        proven = {"chart_constraints_proven": True, "status": "chart_constraints_proven", "reason": "ok",
                  "input_fingerprint_sha256": fingerprint}
        failed = {"chart_constraints_proven": False, "status": "not_proven", "reason": "test"}
        with patch("simplified_vehicle_models.solvers.artifact_chart_audit.check_artifact_chart_element", return_value=proven):
            out = audit_artifact_chart(self.loaded, chart(), settings=ArtifactAuditSettings(maximum_elements=2))
            self.assertFalse(out["all_elements_chart_constraints_proven"])
            self.assertEqual(out["reason"], "source_element_budget")
            self.assertEqual(out["attempted_element_count"], 0)
            self.assertEqual(len(out["unattempted_element_indices"]), 5)
        with patch("simplified_vehicle_models.solvers.artifact_chart_audit.check_artifact_chart_element",
                   side_effect=[proven, failed, proven, proven, proven]):
            out = audit_artifact_chart(self.loaded, chart(), settings=ArtifactAuditSettings(stop_on_unproven=True))
        self.assertFalse(out["all_elements_chart_constraints_proven"])
        self.assertEqual(out["unattempted_element_indices"], [2, 3, 4])
        self.assertEqual(out["reason"], "elements_not_proven")
        with patch("simplified_vehicle_models.solvers.artifact_chart_audit.check_artifact_chart_element",
                   side_effect=RuntimeError("boom")), patch(
                       "simplified_vehicle_models.solvers.artifact_chart_audit._monotonic",
                       wraps=__import__("simplified_vehicle_models.solvers.artifact_chart_audit", fromlist=["_monotonic"])._monotonic), patch(
                       "simplified_vehicle_models.solvers.artifact_chart_audit._snapshot",
                       wraps=__import__("simplified_vehicle_models.solvers.artifact_chart_audit", fromlist=["_snapshot"])._snapshot):
            out = audit_artifact_chart(self.loaded, chart(), settings=ArtifactAuditSettings(stop_on_unproven=True))
        self.assertFalse(out["all_elements_chart_constraints_proven"])
        self.assertEqual(out["attempted_element_count"], 1)
        self.assertEqual(len(out["unattempted_element_indices"]), 4)
        json.dumps(out, allow_nan=False)

    def test_settings_types_and_empty_input_fail_closed(self):
        for settings in (False, "180", ArtifactAuditSettings(maximum_elements=True),
                         ArtifactAuditSettings(stop_on_unproven=1)):
            out = audit_artifact_chart(self.loaded, chart(), settings=settings)
            self.assertFalse(out["all_elements_chart_constraints_proven"])
        out = audit_artifact_chart(self.loaded, chart(), element_settings=False)
        self.assertFalse(out["all_elements_chart_constraints_proven"])
        self.assertEqual(out["reason"], "invalid_element_settings")
        empty = replace(self.loaded, node_station=self.loaded.node_station[:1])
        out = audit_artifact_chart(empty, chart())
        self.assertFalse(out["all_elements_chart_constraints_proven"])
        self.assertEqual(out["reason"], "empty_source_mesh")

    def test_chart_mutation_during_delegate_invalidates_global_identity(self):
        from simplified_vehicle_models.solvers.artifact_chart_audit import _chart_fingerprint
        from simplified_vehicle_models.solvers.artifact_chart_check import check_artifact_chart_element
        surface = chart()
        original = check_artifact_chart_element
        saved = surface.acceleration_max.copy()
        try:
            def mutate(*args, **kwargs):
                result = original(*args, **kwargs)
                surface.acceleration_max.setflags(write=True)
                surface.acceleration_max[0, 0] += .125
                surface.acceleration_max.setflags(write=False)
                return result
            with patch("simplified_vehicle_models.solvers.artifact_chart_audit.check_artifact_chart_element",
                       side_effect=mutate):
                out = audit_artifact_chart(self.loaded, surface)
            self.assertFalse(out["all_elements_chart_constraints_proven"])
            self.assertEqual(out["reason"], "chart_changed")
            self.assertFalse(out["element_results"][0]["chart_constraints_proven"])
        finally:
            surface.acceleration_max.setflags(write=True)
            surface.acceleration_max[:] = saved
            surface.acceleration_max.setflags(write=False)
        self.assertEqual(_chart_fingerprint(surface), out["chart_fingerprint_sha256"])

    def test_identity_guards_and_exact_criteria(self):
        from fractions import Fraction
        from simplified_vehicle_models.solvers.artifact_chart_audit import _snapshot
        _, fingerprint = _snapshot(self.loaded)
        good = {"chart_constraints_proven": True, "status": "chart_constraints_proven",
                "input_fingerprint_sha256": fingerprint}
        wrong = dict(good, input_fingerprint_sha256="0"*64)
        with patch("simplified_vehicle_models.solvers.artifact_chart_audit.check_artifact_chart_element",
                   return_value=wrong):
            result = audit_artifact_chart(self.loaded, chart(), settings=ArtifactAuditSettings(stop_on_unproven=True))
        self.assertFalse(result["all_elements_chart_constraints_proven"])
        self.assertEqual(result["reason"], "element_source_fingerprint_mismatch")
        self.assertEqual(result["attempted_element_count"], 1)

        real = artifact_chart_check._snapshot
        calls = {"count": 0}
        def fail_at_final_snapshot(loaded):
            calls["count"] += 1
            if calls["count"] >= 3:
                raise RuntimeError("changed snapshot")
            return real(loaded)
        with patch("simplified_vehicle_models.solvers.artifact_chart_audit.check_artifact_chart_element",
                   return_value=good), patch(
                       "simplified_vehicle_models.solvers.artifact_chart_audit._snapshot",
                       side_effect=fail_at_final_snapshot):
            result = audit_artifact_chart(self.loaded, chart())
        self.assertFalse(result["all_elements_chart_constraints_proven"])
        self.assertEqual(result["reason"], "final_identity_error:RuntimeError:changed snapshot")

        with patch("simplified_vehicle_models.solvers.artifact_chart_audit.check_artifact_chart_element",
                   return_value=good):
            result = audit_artifact_chart(self.loaded, chart(), minimum_fold=Fraction(1, 3))
        self.assertEqual(result["criteria_exact"]["minimum_fold"], {"numerator": 1, "denominator": 3})
        json.dumps(result, allow_nan=False)

    def test_wall_deadline_before_and_after_delegate(self):
        from simplified_vehicle_models.solvers.artifact_chart_audit import ArtifactAuditSettings
        from simplified_vehicle_models.solvers.artifact_chart_audit import _snapshot
        _, fingerprint = _snapshot(self.loaded)
        real = {"chart_constraints_proven": True, "status": "chart_constraints_proven",
                "input_fingerprint_sha256": fingerprint}
        ticks = iter((0., 0., 0., 0., 2., 2., 2., 2., 2., 2., 2., 2.))
        with patch("simplified_vehicle_models.solvers.artifact_chart_audit._monotonic",
                   side_effect=lambda: next(ticks, 2.)):
            out = audit_artifact_chart(self.loaded, chart(), settings=ArtifactAuditSettings(maximum_wall_seconds=1.))
        self.assertFalse(out["all_elements_chart_constraints_proven"])
        self.assertEqual(out["reason"], "wall_budget_exhausted")
        ticks = iter((0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 2., 2., 2., 2.))
        with patch("simplified_vehicle_models.solvers.artifact_chart_audit._monotonic",
                   side_effect=lambda: next(ticks, 2.)), \
             patch("simplified_vehicle_models.solvers.artifact_chart_audit.check_artifact_chart_element",
                   return_value=real):
            out = audit_artifact_chart(self.loaded, chart(), settings=ArtifactAuditSettings(maximum_wall_seconds=1.))
        self.assertFalse(out["all_elements_chart_constraints_proven"])
        self.assertEqual(out["reason"], "wall_budget_exhausted")


if __name__ == "__main__":
    unittest.main()
