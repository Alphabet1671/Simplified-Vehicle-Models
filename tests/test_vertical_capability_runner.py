"""Runner integration contracts for opt-in vertical tested-speed restrictions."""
import unittest
from unittest.mock import patch

from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import numpy as np
from tools.run.run_iem24_forward_driver import (
    parse_args, source_files, policy_identity_settings, warm_start_matches,
    apply_vertical_corner_restriction, vertical_contact_valid,
)
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope



class VerticalCapabilityRunnerTests(unittest.TestCase):
    def test_budget_defaults_zero_and_requires_nonnegative_integer(self):
        args = parse_args(["--track", "models/tracks/2024_michigan_endurance_multilap_closed.xml"])
        self.assertEqual(args.vertical_corner_samples, 0)
        for value in ("-1", "1.5", "nope"):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                parse_args(["--vertical-corner-samples", value])

    def test_explicit_timeout_above_default_allowed_for_both_models(self):
        for model in ("planar-7dof", "14dof-simple"):
            args = parse_args(["--model", model, "--timeout-seconds", "600"])
            self.assertEqual(args.timeout_seconds, 600.)

    def test_budget_only_allowed_for_vertical_model(self):
        with self.assertRaises(SystemExit):
            parse_args(["--vertical-corner-samples", "4"])
        args = parse_args(["--model", "14dof-simple", "--vertical-corner-samples", "4"])
        self.assertEqual(args.vertical_corner_samples, 4)

    def test_capability_source_only_in_enabled_vertical_closure(self):
        track = "models/tracks/2024_michigan_endurance_multilap_closed.xml"
        capability = "vertical_capability.py"
        planar = [p.name for p in source_files("planar-7dof", track)]
        vertical_default = [p.name for p in source_files("14dof-simple", track)]
        vertical_enabled = [p.name for p in source_files("14dof-simple", track,
                                                           vertical_corner_samples=8)]
        self.assertNotIn(capability, planar)
        self.assertNotIn(capability, vertical_default)
        self.assertIn(capability, vertical_enabled)

    def test_policy_identity_distinguishes_sample_budget_and_result_digest(self):
        args = parse_args(["--model", "14dof-simple", "--vertical-corner-samples", "2"])
        args.track_is_closed = True
        args.builder_parameters = {"method": "sampled-quasi-static-v1"}
        args.vertical_capability_semantic_sha256 = "abc"
        first = policy_identity_settings(args)
        args.vertical_capability_semantic_sha256 = "def"
        second = policy_identity_settings(args)
        self.assertNotEqual(first, second)
        args.vertical_corner_samples = 3
        third = policy_identity_settings(args)
        self.assertNotEqual(second, third)

    def test_mocked_restriction_rebuilds_plan_and_writes_semantic_artifact(self):
        plan = SpeedPlan(np.array([0., 1., 2.]), np.array([4., 5., 4.]),
                         np.array([.1, .2, .1]), is_closed=True)
        envelope = BrakingEnvelope(np.array([0., 1., 2.]), np.array([.1, .2, .1]),
            np.array([5., 5., 5.]), np.array([4., 4., 4.]), np.array([2., 2.]),
            np.array([1, 1, 1]), True, 8., {"segment_speed_domain_max": (5., 5.)})
        from simplified_vehicle_models.solvers.vertical_capability import VerticalCapResult
        from simplified_vehicle_models.solvers.braking_envelope import restrict_braking_envelope as real_restrict_braking_envelope
        evidence = VerticalCapResult(caps=np.array([3., 4., 3.]), samples=(), reasons=("restricted",)*3,
            provenance="vertical_14dof_tested_speed_restriction_v1", query_count=2, max_queries=4,
            caveat="tested only")
        with TemporaryDirectory() as temporary, \
             patch("simplified_vehicle_models.solvers.vertical_capability.constrain_corner_speeds",
                   return_value=evidence), \
             patch("simplified_vehicle_models.solvers.braking_envelope.restrict_braking_envelope") as restrict:
            restricted = BrakingEnvelope(envelope.distance, envelope.curvature, evidence.caps,
                np.array([3., 3.5, 3.]), envelope.deceleration, envelope.binding_index,
                True, envelope.maximum_speed, envelope.provenance)
            restrict.side_effect = lambda original, caps, *, provenance=None: real_restrict_braking_envelope(
                original, caps, provenance=provenance)

            updated, result, detail = apply_vertical_corner_restriction(
                object(), plan, envelope, max_queries=4, is_closed=True,
                artifact_path=Path(temporary) / "vertical_corner_samples.json")
            restrict.assert_called_once()
            self.assertEqual(result.corner_speed.tolist(), evidence.caps.tolist())
            self.assertTrue(np.all(result.entry_speed <= envelope.entry_speed))
            self.assertTrue(np.all(updated.speed <= plan.speed))
            self.assertEqual(detail["query_count"], 2)
            self.assertTrue(Path(temporary, "vertical_corner_samples.json").is_file())
            self.assertEqual(len(detail["semantic_sha256"]), 64)

    def test_contact_gate_does_not_treat_table_extension_as_failure(self):
        self.assertTrue(vertical_contact_valid(True, 0., network_state_valid=True))
        self.assertFalse(vertical_contact_valid(True, -0.1, network_state_valid=True))
        self.assertFalse(vertical_contact_valid(True, float("nan"), network_state_valid=True))
        self.assertFalse(vertical_contact_valid(True, 0., network_state_valid=False))
        from tools.run.run_iem24_forward_driver import acceptance_gate
        self.assertTrue(acceptance_gate({
            "lap_completed": True, "track_is_closed": False,
            "integration_completed": True, "path_tracking_valid": True,
            "resolution_valid": True, "checked_rhs_physics_valid": True,
            "contact_valid": True, "out_of_range_evaluations": 100,
        }))

    def test_warm_start_rejects_changed_capability_semantics(self):
        args = parse_args(["--model", "14dof-simple", "--vertical-corner-samples", "2"])
        args.track_is_closed = True
        args.builder_parameters = {"method": "sampled-quasi-static-v1"}
        args.vertical_capability_semantic_sha256 = "current"
        identity = policy_identity_settings(args)
        previous = {"status": "Completed", "lap_completed": True,
                    "source_hashes": {"source.py": "hash"},
                    "pedal_map": {"semantic_sha256": "no-pedal-map"},
                    "settings": {**{key: (str(value) if isinstance(value, Path) else value)
                                      for key, value in vars(args).items()
                                      if key not in {"pedal_map_config_payload", "pedal_map_config", "pedal_map_file_sha256"}},
                                 "track": str(args.track), "longitudinal_policy": "braking-envelope",
                                 "envelope_deceleration_margin": .6, "envelope_speed_points": 9},
                    "longitudinal_policy": {"policy": "braking-envelope",
                         "deceleration_margin": .6, "speed_query_points": 9,
                         "settings_identity_sha256": __import__("hashlib").sha256(
                             __import__("json").dumps(identity, sort_keys=True,
                                 separators=(",", ":"), allow_nan=False).encode()).hexdigest()}}
        self.assertTrue(warm_start_matches(previous, {"source.py": "hash"}, args,
                                           "no-pedal-map", identity))
        changed = {**identity, "builder": {**identity["builder"],
                    "vertical_capability_semantic_sha256": "other"}}
        self.assertFalse(warm_start_matches(previous, {"source.py": "hash"}, args,
                                            "no-pedal-map", changed))
        changed_budget = {**identity, "builder": {**identity["builder"],
                          "vertical_corner_samples": 3}}
        self.assertFalse(warm_start_matches(previous, {"source.py": "hash"}, args,
                                            "no-pedal-map", changed_budget))


if __name__ == "__main__":
    unittest.main()
