"""Fail-closed consumer approval-record verifier contract."""
import unittest

from simplified_vehicle_models.solvers.convergence_policy import NUMERICAL_POLICY_V1
from simplified_vehicle_models.solvers.consumer_approval_record import (
    ApprovalStatus,
    verify_consumer_approval_record,
)


class ConsumerApprovalRecordTests(unittest.TestCase):
    def test_proposed_policy_cannot_self_approve(self):
        result = verify_consumer_approval_record(
            {"policy_identifier": NUMERICAL_POLICY_V1.identifier,
             "policy_sha256": NUMERICAL_POLICY_V1.sha256,
             "consumer_allocation": {"consumer_owner": "claimant"}},
            policy=NUMERICAL_POLICY_V1,
            executed_identity={}, required_evidence={},
        )
        self.assertEqual(result.status, ApprovalStatus.UNAPPROVED_POLICY)
        self.assertFalse(result.qualified)
        self.assertFalse(result.accepted)
        self.assertFalse(result.policy_approved)
        self.assertIsNone(result.geometry_state_convergence)

    def test_proposed_policy_preserves_valid_normalized_snapshots(self):
        record = {"payload": {"items": ["record"]}}
        evidence = {"grid": {"passed": True, "trace": ["grid"]}}
        result = verify_consumer_approval_record(record, policy=NUMERICAL_POLICY_V1,
            executed_identity={}, required_evidence=evidence)
        self.assertEqual(result.status, ApprovalStatus.UNAPPROVED_POLICY)
        self.assertEqual(result.record["payload"]["items"], ("record",))
        self.assertEqual(result.evidence["grid"]["trace"], ("grid",))
        with self.assertRaises(TypeError):
            result.record["payload"]["items"] = ()
        detached = result.to_dict()
        detached["record"]["payload"]["items"].append("changed")
        detached["evidence"]["grid"]["trace"].append("changed")
        self.assertEqual(result.record["payload"]["items"], ("record",))
        self.assertEqual(result.evidence["grid"]["trace"], ("grid",))

    def test_proposed_policy_preserves_valid_input_when_other_is_malformed(self):
        record = {"payload": {"items": ["record"]}}
        malformed_evidence = {"bad": object()}
        result = verify_consumer_approval_record(record, policy=NUMERICAL_POLICY_V1,
            executed_identity={}, required_evidence=malformed_evidence)
        self.assertEqual(result.status, ApprovalStatus.UNAPPROVED_POLICY)
        self.assertEqual(result.record["payload"]["items"], ("record",))
        self.assertIsNone(result.evidence)

        malformed_record = {"bad": float("nan")}
        evidence = {"grid": {"passed": True, "trace": ["grid"]}}
        result = verify_consumer_approval_record(malformed_record, policy=NUMERICAL_POLICY_V1,
            executed_identity={}, required_evidence=evidence)
        self.assertEqual(result.status, ApprovalStatus.UNAPPROVED_POLICY)
        self.assertIsNone(result.record)
        self.assertEqual(result.evidence["grid"]["trace"], ("grid",))

    def test_proposed_policy_handles_cyclic_and_deep_payloads_independently(self):
        cyclic_record = []
        cyclic_record.append(cyclic_record)
        valid_evidence = {"grid": {"passed": True, "trace": ["grid"]}}
        result = verify_consumer_approval_record(cyclic_record, policy=NUMERICAL_POLICY_V1,
            executed_identity={}, required_evidence=valid_evidence)
        self.assertEqual(result.status, ApprovalStatus.UNAPPROVED_POLICY)
        self.assertIsNone(result.record)
        self.assertEqual(result.evidence["grid"]["trace"], ("grid",))

        cyclic_evidence = []
        cyclic_evidence.append(cyclic_evidence)
        valid_record = {"payload": ["record"]}
        result = verify_consumer_approval_record(valid_record, policy=NUMERICAL_POLICY_V1,
            executed_identity={}, required_evidence=cyclic_evidence)
        self.assertEqual(result.status, ApprovalStatus.UNAPPROVED_POLICY)
        self.assertEqual(result.record["payload"], ("record",))
        self.assertIsNone(result.evidence)

        deep_record = "leaf"
        for _ in range(1200):
            deep_record = [deep_record]
        result = verify_consumer_approval_record(deep_record, policy=NUMERICAL_POLICY_V1,
            executed_identity={}, required_evidence=valid_evidence)
        self.assertEqual(result.status, ApprovalStatus.UNAPPROVED_POLICY)
        self.assertIsNone(result.record)
        self.assertEqual(result.evidence["grid"]["trace"], ("grid",))

    def test_proposed_policy_precedes_malformed_inputs(self):
        for record, identity, evidence in (
            ({"bad": float("nan")}, {}, {}),
            ({1: "non-string key"}, {}, {}),
            ({}, float("inf"), {}),
            ({}, {}, {"bad": object()}),
        ):
            result = verify_consumer_approval_record(record, policy=NUMERICAL_POLICY_V1,
                executed_identity=identity, required_evidence=evidence)
            self.assertEqual(result.status, ApprovalStatus.UNAPPROVED_POLICY)
            self.assertFalse(result.qualified)
            self.assertFalse(result.accepted)
            self.assertFalse(result.policy_approved)
            self.assertIsNone(result.geometry_state_convergence)

    def _approved_case(self):
        from types import SimpleNamespace
        policy = SimpleNamespace(status="approved", requirements_status="approved",
                                 identifier="consumer-policy", sha256="policy-hash")
        identity = {
            "source": {"sha256": "source"}, "runtime": {"python": "pinned"},
            "chart_reserve": {"reserve": "r"}, "boundary": {"kind": "open"},
            "representation": {"state": "W"}, "realized_basis_grid": [0.0, 1.0],
            "seed": {"sha256": "seed"}, "reference": {"sha256": "ref"},
            "replay": {"sha256": "replay"},
        }
        record = {
            "policy_identifier": "consumer-policy", "policy_sha256": "policy-hash",
            "consumer_allocation": {
                "consumer_owner": "owner", "review_owner": "reviewer", "allocation_id": "a1",
                "dimensional_allocations": {name: "opaque" for name in
                    ("time", "xy", "speed", "curvature", "length", "clearance")},
                "uncertainty": "declared",
            }, "executed_identity": identity,
        }
        evidence = {name: {"passed": True, "trace": {"artifact": name}} for name in
                    ("grid", "seed", "reference", "replay", "original_model")}
        evidence.update({"maximum_accuracy_certified": False, "meets_accuracy_budget": None})
        return policy, identity, record, evidence

    def test_brief_policy_identifier_is_accepted_before_uncertified_evidence_gate(self):
        from types import SimpleNamespace
        approved_policy = SimpleNamespace(
            status="approved", requirements_status="approved",
            identifier="consumer-policy", sha256="policy-hash",
        )
        identity = {
            "source": {"sha256": "source"}, "runtime": {"python": "pinned"},
            "chart_reserve": {"reserve": "r"}, "boundary": {"kind": "open"},
            "representation": {"state": "W"}, "realized_basis_grid": [0.0, 1.0],
            "seed": {"sha256": "seed"}, "reference": {"sha256": "ref"},
            "replay": {"sha256": "replay"},
        }
        record = {
            "policy_identifier": "consumer-policy", "policy_sha256": "policy-hash",
            "consumer_allocation": {
                "consumer_owner": "owner", "review_owner": "reviewer", "allocation_id": "a1",
                "dimensional_allocations": {name: "opaque" for name in
                    ("time", "xy", "speed", "curvature", "length", "clearance")},
                "uncertainty": "declared",
            },
            "executed_identity": identity,
        }
        evidence = {name: {"passed": True, "trace": {"artifact": name}} for name in
                    ("grid", "seed", "reference", "replay", "original_model")}
        evidence.update({"maximum_accuracy_certified": False, "meets_accuracy_budget": None})
        result = verify_consumer_approval_record(
            record, policy=approved_policy, executed_identity=identity, required_evidence=evidence)
        self.assertEqual(result.status, ApprovalStatus.QUALIFICATION_UNRESOLVED)
        self.assertFalse(result.qualified)
        self.assertFalse(result.accepted)

    def test_allocation_required_and_schema_checked(self):
        policy, identity, record, evidence = self._approved_case()
        for allocation in (None, {}, {"consumer_owner": "owner"},
                           {"consumer_owner": "owner", "review_owner": "r", "allocation_id": "a",
                            "dimensional_allocations": {"time": "x"}, "uncertainty": "u"}):
            candidate = dict(record, consumer_allocation=allocation)
            result = verify_consumer_approval_record(candidate, policy=policy,
                                                     executed_identity=identity, required_evidence=evidence)
            self.assertEqual(result.status, ApprovalStatus.MISSING_CONSUMER_ALLOCATION)

    def test_allocation_values_and_uncertainty_must_be_meaningful(self):
        policy, identity, record, evidence = self._approved_case()
        for value in (None, True, False, "", "  ", {}, []):
            allocation = dict(record["consumer_allocation"])
            dims = dict(allocation["dimensional_allocations"])
            dims["time"] = value
            allocation["dimensional_allocations"] = dims
            candidate = dict(record, consumer_allocation=allocation)
            self.assertEqual(verify_consumer_approval_record(candidate, policy=policy,
                executed_identity=identity, required_evidence=evidence).status,
                ApprovalStatus.MISSING_CONSUMER_ALLOCATION)
            allocation = dict(record["consumer_allocation"], uncertainty=value)
            candidate = dict(record, consumer_allocation=allocation)
            self.assertEqual(verify_consumer_approval_record(candidate, policy=policy,
                executed_identity=identity, required_evidence=evidence).status,
                ApprovalStatus.MISSING_CONSUMER_ALLOCATION)

    def test_each_evidence_requires_meaningful_trace(self):
        policy, identity, record, evidence = self._approved_case()
        for name in ("grid", "seed", "reference", "replay", "original_model"):
            for replacement in ({"passed": True}, {"passed": True, "trace": None},
                                {"passed": True, "trace": "  "}, {"passed": True, "trace": []}):
                candidate = dict(evidence, **{name: replacement})
                self.assertEqual(verify_consumer_approval_record(record, policy=policy,
                    executed_identity=identity, required_evidence=candidate).status,
                    ApprovalStatus.MISSING_REQUIRED_EVIDENCE)

    def test_carrier_snapshots_are_immutable_and_to_dict_is_detached(self):
        policy, identity, record, evidence = self._approved_case()
        result = verify_consumer_approval_record(record, policy=policy,
            executed_identity=identity, required_evidence=evidence)
        with self.assertRaises(TypeError):
            result.record["new"] = "mutation"
        with self.assertRaises(TypeError):
            result.evidence["grid"]["passed"] = False
        detached = result.to_dict()
        detached["record"]["consumer_allocation"]["dimensional_allocations"]["time"] = "changed"
        detached["evidence"]["grid"]["trace"]["artifact"] = "changed"
        self.assertEqual(result.record["consumer_allocation"]["dimensional_allocations"]["time"], "opaque")
        self.assertEqual(result.evidence["grid"]["trace"]["artifact"], "grid")

    def test_each_identity_field_is_bound_exactly(self):
        policy, identity, record, evidence = self._approved_case()
        fields = ("source", "runtime", "chart_reserve", "boundary", "representation",
                  "realized_basis_grid", "seed", "reference", "replay")
        for field in fields:
            changed = dict(identity)
            changed[field] = {"different": True}
            result = verify_consumer_approval_record(record, policy=policy,
                                                     executed_identity=changed, required_evidence=evidence)
            self.assertEqual(result.status, ApprovalStatus.IDENTITY_MISMATCH, field)
            self.assertEqual(result.mismatch_field, field)

    def test_missing_and_failed_evidence_are_rejected(self):
        policy, identity, record, evidence = self._approved_case()
        for name in ("grid", "seed", "reference", "replay", "original_model"):
            missing = dict(evidence)
            del missing[name]
            self.assertEqual(verify_consumer_approval_record(record, policy=policy,
                executed_identity=identity, required_evidence=missing).status,
                ApprovalStatus.MISSING_REQUIRED_EVIDENCE)
            failed = dict(evidence, **{name: {"passed": False}})
            self.assertEqual(verify_consumer_approval_record(record, policy=policy,
                executed_identity=identity, required_evidence=failed).status,
                ApprovalStatus.MISSING_REQUIRED_EVIDENCE)

    def test_bad_json_and_nonfinite_inputs_fail_closed(self):
        policy, identity, record, evidence = self._approved_case()
        for bad in ({1: "non-string key"}, {"x": float("nan")}, {"x": float("inf")}):
            result = verify_consumer_approval_record(bad, policy=policy,
                                                     executed_identity=identity, required_evidence=evidence)
            self.assertEqual(result.status, ApprovalStatus.INVALID_RECORD)
            self.assertFalse(result.qualified)
            self.assertFalse(result.accepted)

    def test_tolerance_argument_is_not_consumed(self):
        policy, identity, record, evidence = self._approved_case()
        with self.assertRaises(TypeError):
            verify_consumer_approval_record(record, policy=policy, executed_identity=identity,
                required_evidence=evidence, tolerances={"anything": 0})

    def test_complete_evidence_never_qualifies_sampled_uncertified_result(self):
        policy, identity, record, evidence = self._approved_case()
        result = verify_consumer_approval_record(record, policy=policy,
                                                 executed_identity=identity, required_evidence=evidence)
        self.assertEqual(result.status, ApprovalStatus.QUALIFICATION_UNRESOLVED)
        self.assertFalse(result.qualified)
        self.assertFalse(result.accepted)
        self.assertIsNone(result.geometry_state_convergence)
        self.assertTrue(result.policy_approved)
        payload = result.to_dict()
        self.assertIs(payload["qualified"], False)
        self.assertIs(payload["accepted"], False)
        self.assertIsNone(payload["geometry_state_convergence"])
        self.assertTrue(result.to_json())

    def test_malformed_and_boolean_allocation_keys_fail_closed_json(self):
        policy, identity, record, evidence = self._approved_case()
        candidate = dict(record)
        candidate[1] = "invalid"
        self.assertEqual(verify_consumer_approval_record(candidate, policy=policy,
            executed_identity=identity, required_evidence=evidence).status, ApprovalStatus.INVALID_RECORD)
        candidate = dict(record, consumer_allocation=dict(record["consumer_allocation"], uncertainty=True))
        self.assertEqual(verify_consumer_approval_record(candidate, policy=policy,
            executed_identity=identity, required_evidence=evidence).status,
            ApprovalStatus.MISSING_CONSUMER_ALLOCATION)


if __name__ == "__main__":
    unittest.main()
