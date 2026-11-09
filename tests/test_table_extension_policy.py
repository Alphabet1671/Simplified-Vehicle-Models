"""Unlimited suspension-table extension is descriptive, not a contact limit."""
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from tools.gates.check_14dof_simple_gates import table_domain_gate

class TableExtensionPolicyTests(unittest.TestCase):
    def evaluate(self, **changes):
        summary = {"suspension_travel_policy": "unlimited", "suspension_travel_enforced": False,
                   "out_of_range_evaluations": 17, "maximum_damper_velocity": 9.,
                   "jounce_accepted_minimum_m": -.2, "jounce_accepted_maximum_m": .2}
        summary.update(changes)
        tables = {"spring_jounce_front": np.array([-.03, .03]),
                  "jounce_steer_jounce_front": np.array([-.03, .03]),
                  "jounce_steer_toe_front": np.zeros(2)}
        damper = {"velocity_compression": np.array([0., 1.]),
                  "velocity_rebound": np.array([0., 1.])}
        with patch("tools.gates.check_14dof_simple_gates.vertical_plant"), \
             patch("tools.gates.check_14dof_simple_gates.column", return_value=0), \
             patch("tools.gates.check_14dof_simple_gates.trace_of", return_value=np.zeros((2, 1))), \
             patch("tools.gates.check_14dof_simple_gates.relative_attempt", return_value="fixture"):
            return table_domain_gate(Path("fixture"), summary, tables, damper)

    def test_table_extensions_do_not_reject_unlimited_suspension(self):
        self.assertTrue(self.evaluate()["pass"])

    def test_network_and_invalid_diagnostics_still_reject(self):
        for payload in ({"network_state_valid": False}, {"maximum_damper_velocity": float("nan")},
                        {"jounce_accepted_minimum_m": 1.}, {"jounce_accepted_maximum_m": None}):
            with self.subTest(payload=payload):
                self.assertFalse(self.evaluate(**payload)["pass"])

    def test_historical_bounded_policy_keeps_its_original_counter(self):
        self.assertFalse(self.evaluate(suspension_travel_policy="bounded")["pass"])
        self.assertTrue(self.evaluate(suspension_travel_policy="bounded", out_of_range_evaluations=0)["pass"])
