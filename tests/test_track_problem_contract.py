import copy
import hashlib
import json
import math
import unittest

from simplified_vehicle_models.solvers.track_problem_contract import validate_problem_contract


def request(**overrides):
    value = {
        "schema": "track_problem_v1",
        "topology": "open",
        "source_stations": [4.0, 4.75, 7.0],
        "lower_offset": [-1.0, -0.5, -2.0],
        "upper_offset": [1.0, 0.25, 3.0],
        "boundary": {"start_mode": "rolling", "start_speed_mps": 2.0, "finish_mode": "free"},
    }
    value.update(overrides)
    return value


class ProblemContractTest(unittest.TestCase):
    def test_standing_is_not_silently_turned_into_rolling(self):
        value = request(boundary={"start_mode": "standing", "start_speed_mps": 0.0, "finish_mode": "free"})
        result = validate_problem_contract(value)
        self.assertEqual(result["status"], "unsupported")
        self.assertIn("standing_requires_coupled_event_solver", result["reasons"])
        self.assertEqual(result["normalized_contract"]["boundary"]["start_mode"], "standing")

    def test_positive_open_rolling_free_is_structurally_supported(self):
        self.assertEqual(validate_problem_contract(request())["status"], "supported")

    def test_closed_flying_periodic_is_supported_but_legacy_zero_is_not(self):
        boundary = {"start_mode": "flying", "start_speed_mps": 12.0, "finish_mode": "periodic"}
        self.assertEqual(validate_problem_contract(request(topology="closed", boundary=boundary))["status"], "supported")
        boundary["start_speed_mps"] = 0.0
        self.assertEqual(validate_problem_contract(request(topology="closed", boundary=boundary))["status"], "invalid")

    def test_supported_boundary_matrix_and_unsupported_boundaries(self):
        cases = [
            ("open", "rolling", 3., "free", None, "supported", None),
            ("open", "flying", 3., "free", None, "unsupported", "boundary_solver_not_implemented"),
            ("open", "rolling", 3., "fixed_speed", 4., "unsupported", "finish_speed_constraint_not_implemented"),
            ("open", "rolling", 3., "fixed_speed", 0., "unsupported", "terminal_rest_requires_event_solver"),
            ("closed", "rolling", 3., "free", None, "unsupported", "closed_boundary_not_implemented"),
            ("closed", "flying", 3., "free", None, "unsupported", "closed_boundary_not_implemented"),
            ("closed", "flying", 3., "periodic", None, "supported", None),
        ]
        for topology, start, speed, finish, finish_speed, status, reason in cases:
            with self.subTest(topology=topology, start=start, finish=finish, finish_speed=finish_speed):
                boundary = {"start_mode": start, "start_speed_mps": speed, "finish_mode": finish}
                if finish_speed is not None:
                    boundary["finish_speed_mps"] = finish_speed
                result = validate_problem_contract(request(topology=topology, boundary=boundary))
                self.assertEqual(result["status"], status)
                if reason:
                    self.assertIn(reason, result["reasons"])

    def test_invalid_values_and_no_defaults(self):
        invalid = [
            request(boundary={"start_mode": "rolling", "start_speed_mps": 2.}),
            request(boundary={"start_mode": "rolling", "start_speed_mps": 2., "finish_mode": "periodic"}),
            request(boundary={"start_mode": "rolling", "start_speed_mps": True, "finish_mode": "free"}),
            request(boundary={"start_mode": [], "start_speed_mps": 2., "finish_mode": "free"}),
            request(topology=[]),
            request(source_stations=[0., 1., 1.]),
            request(source_stations=[0., 0.]),
            request(lower_offset=[0.1, 0.2, 0.3]),
            request(upper_offset=[-0.1, -0.2, -0.3]),
            request(lower_offset=[0., 0., 0.], upper_offset=[0., 0., 0.]),
            request(upper_offset=[1., -1., 1.]),
            request(upper_offset=[1., math.inf, 1.]),
            request(source_stations=[0., 1., 10**10000]),
            request(extra=True),
            request(schema="track_problem_v2"),
            request(topology="open", boundary={"start_mode": "rolling", "start_speed_mps": -1., "finish_mode": "free"}),
        ]
        for value in invalid:
            with self.subTest(value=value):
                self.assertEqual(validate_problem_contract(value)["status"], "invalid")

    def test_hash_is_canonical_json_and_input_is_not_mutated(self):
        original = request()
        before = copy.deepcopy(original)
        result = validate_problem_contract(original)
        self.assertEqual(original, before)
        self.assertTrue(json.dumps(result, allow_nan=False))
        normalized = result["normalized_contract"]
        expected = hashlib.sha256(json.dumps(normalized, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        self.assertEqual(result["normalized_hash"], expected)
        self.assertEqual(result["normalized_hash"], validate_problem_contract(json.loads(json.dumps(original)))["normalized_hash"])
        reordered = dict(reversed(list(original.items())))
        self.assertEqual(result["normalized_hash"], validate_problem_contract(reordered)["normalized_hash"])

    def test_arbitrary_origin_nonuniform_positive_width_is_valid(self):
        result = validate_problem_contract(request())
        self.assertNotEqual(result["status"], "invalid")
        self.assertEqual(result["normalized_contract"]["source_stations"], [4.0, 4.75, 7.0])


if __name__ == "__main__":
    unittest.main()
