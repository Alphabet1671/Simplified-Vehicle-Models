import unittest

from tools.gates.check_14dof_simple_gates import travel_gate


class UnlimitedTravelGateTests(unittest.TestCase):
    def test_requires_true_finite_ordered_jounce_diagnostic(self):
        base = {
            "suspension_travel_policy": "unlimited",
            "suspension_travel_enforced": False,
            "jounce_travel_valid": True,
            "jounce_accepted_minimum_m": -0.04,
            "jounce_accepted_maximum_m": 0.05,
        }
        # travel_gate gathers optional bump-stop display evidence from files;
        # a missing directory is handled as unavailable evidence.
        result = travel_gate("/nonexistent/unlimited-attempt", base, {})
        self.assertTrue(result["pass"])
        for update in (
            {"jounce_travel_valid": False},
            {"suspension_travel_enforced": True},

            {"jounce_accepted_minimum_m": float("nan")},
            {"jounce_accepted_minimum_m": 0.06, "jounce_accepted_maximum_m": 0.05},
        ):
            with self.subTest(update=update):
                summary = {**base, **update}
                self.assertFalse(travel_gate("/nonexistent/unlimited-attempt", summary, {})["pass"])

    def test_historical_policy_fallback_is_unchanged(self):
        summary = {"jounce_travel_valid": True}
        self.assertTrue(travel_gate("/nonexistent/historical-attempt", summary, {})["pass"])


if __name__ == "__main__":
    unittest.main()
