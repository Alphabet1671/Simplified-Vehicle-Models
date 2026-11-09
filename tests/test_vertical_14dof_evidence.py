"""Task 9 evidence gates for the vertical 14DoF rung.

The plan's Task 9 acceptance gate is the plan's **gate table**, not the runner's
``usable`` flag. This module reads the committed evidence of the acceptance run:

* the newest ``docs/validation/iem24-murphy/14dof_simple_trials/attempt_*/``
  ``summary.json`` (the runner's in-run gates and diagnostics), and
* its ``gates.json``, written by ``tools/gates/check_14dof_simple_gates.py``,
  which carries the plan's seven gates with their measured evidence and names
  the periodicity and planar-reference attempts.

Gate-name mapping (the plan's table -> the runner's summary keys). ``gates.json``
records this mapping itself in ``gate_source_mapping``; it is restated here so a
reader of the test sees which runner field backs which gate:

===============  ==========================================================
plan gate        runner summary key(s)
===============  ==========================================================
completed        ``lap_completed`` + ``status`` + elapsed <= requested wall
contact_valid    ``contact_valid``
travel_valid     ``jounce_travel_valid`` (the runner's name for it)
table_domain_valid ``out_of_range_evaluations == 0``
periodicity_valid  ``periodicity_valid`` of the warm-started attempt
===============  ==========================================================
"""
import hashlib
import json
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from tools.gates.check_14dof_simple_gates import (
    column, read_trace, tolerance_by_name,
)

ROOT = Path(__file__).resolve().parents[1]
TRIALS = ROOT / "docs/validation/iem24-murphy/14dof_simple_trials"
GATE_TOOL = ROOT / "tools/gates/check_14dof_simple_gates.py"
PLANAR_REFERENCE = ("docs/validation/iem24-murphy/seven_dof_track_pair/original/"
                    "attempt_1792298175493764000")

#: What a reader must run when the gate table is missing or stale.
GATE_TOOL_COMMAND = (
    "PYTHONPATH=src:. python3 tools/gates/check_14dof_simple_gates.py "
    "--attempt <cold 14dof-simple attempt> --periodicity <warm baseline attempt> "
    "--planar-reference " + PLANAR_REFERENCE)

#: The plan's Task 9 acceptance gates, in plan order.
PLAN_GATES = ("completed", "contact_valid", "travel_valid", "table_domain_valid",
              "periodicity_valid")

#: plan gate -> the runner summary key whose value the gate must agree with.
RUNNER_GATE_KEYS = {
    "completed": "lap_completed",
    "contact_valid": "contact_valid",
    "travel_valid": "jounce_travel_valid",
    "table_domain_valid": "out_of_range_evaluations",
    "periodicity_valid": "periodicity_valid",
}

#: Evidence keys every gate record must carry.
REQUIRED_EVIDENCE = {
    "completed": (
        "lap_time_s", "simulated_seconds", "track_length_m", "distance_completed_m",
        "requested_wall_seconds", "solver_wall_budget_seconds", "solver_elapsed_seconds",
        "elapsed_wall_seconds", "peak_memory_bytes", "accepted_steps", "evaluations",
        "rejected_domain_trials", "within_requested_wall_seconds"),
    "contact_valid": (
        "minimum_contact_load_n", "contact_load_limit_n", "lifted_wheel_evaluations",
        "out_of_range_evaluations", "jounce_travel_valid", "lap_completed"),
    "travel_valid": (
        "jounce_travel_limits_m", "jounce_accepted_minimum_m", "jounce_accepted_maximum_m",
        "jounce_gate_scope", "jounce_evaluation_minimum_m", "jounce_evaluation_maximum_m",
        "dense_midpoint_samples", "bump_stop_blend_start_m", "bump_stop_entry_count",
        "bump_stop_entry_count_per_wheel", "bump_stop_peak_wheel_force_n",
        "minimum_wheel_force_n", "jounce_extremes_per_wheel_m",
        "maximum_absolute_jounce_m", "bump_stop_scope"),
    "table_domain_valid": (
        "out_of_range_evaluations", "out_of_range_scope", "maximum_damper_velocity_mps",
        "damper_table_velocity_limit_mps", "damper_velocity_headroom_fraction",
        "spring_curve_jounce_domain_m", "jounce_steer_jounce_domain_m",
        "jounce_accepted_extremes_m", "jounce_steer_baseline_toe_rad",
        "roll_centre_table_limit_deg", "maximum_absolute_roll_deg",
        "roll_centre_query_clipped"),
    "periodicity_valid": (
        "lap_completed", "lap_time_s", "maximum_tolerance_ratio", "worst_state",
        "worst_state_absolute_delta", "worst_state_tolerance", "periodic_state_count",
        "verified_state_count", "vertical_tolerances", "tire_state_tolerances",
        "recorded_tolerances_match_plant", "warm_start_source"),
}

#: Evidence keys that must hold a value, not just exist, when a gate passes.
#: ``peak_memory_bytes`` is excluded because ``resource`` is unavailable on some
#: platforms; every other key is measurable on the acceptance run.
NON_NULL_EVIDENCE = {
    gate: tuple(key for key in keys if key != "peak_memory_bytes")
    for gate, keys in REQUIRED_EVIDENCE.items()
}


def sha256_file(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def acceptance_attempt() -> Path:
    """Return the newest acceptance attempt that carries a gate table.

    A new attempt written straight into the trials root has no ``gates.json``
    until the gate tool runs, and the table would then be stale. That case, and a
    trials root with no annotated attempt at all, must produce one clear message
    naming the command to run — never a bare ``FileNotFoundError`` from reading a
    missing file, and never a silent pass on an older table.
    """
    attempts = sorted(path for path in TRIALS.glob("attempt_*") if path.is_dir())
    where = TRIALS.relative_to(ROOT)
    if not attempts:
        raise AssertionError(
            f"no acceptance attempt under {where}; run the Task 9 lap, then the gate tool:\n"
            f"  {GATE_TOOL_COMMAND}")
    annotated = [path for path in attempts if (path / "gates.json").is_file()]
    if not annotated:
        raise AssertionError(
            f"no attempt under {where} carries gates.json (the newest attempt is "
            f"{attempts[-1].name}); run the gate tool:\n  {GATE_TOOL_COMMAND}")
    if annotated[-1] != attempts[-1]:
        raise AssertionError(
            f"the newest attempt {attempts[-1].name} has no gates.json, so the gate table "
            f"{annotated[-1].name}/gates.json is stale relative to it; re-run the gate tool "
            f"with --attempt {attempts[-1]}:\n  {GATE_TOOL_COMMAND}")
    return annotated[-1]


def attempt_summary(record: dict) -> dict:
    """Return the summary of the attempt one gate record came from."""
    return read_json(ROOT / record["source_attempt"] / "summary.json")


class Vertical14DofEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.attempt = acceptance_attempt()
        cls.summary = read_json(cls.attempt / "summary.json")
        cls.table = read_json(cls.attempt / "gates.json")
        cls.gates = cls.table["gates"]

    # -- the artifact set -------------------------------------------------

    def test_acceptance_attempt_is_a_completed_14dof_lap_with_a_gate_table(self):
        self.assertEqual(self.summary["model"], "14dof-simple")
        self.assertEqual(self.summary["status"], "Completed")
        self.assertIs(self.summary["lap_completed"], True)
        self.assertIsNotNone(self.summary["lap_time"])
        self.assertEqual(self.table["attempt"],
                         self.attempt.relative_to(ROOT).as_posix())

    def test_the_paired_attempts_are_recorded_and_exist(self):
        for key in ("periodicity_attempt", "planar_reference_attempt"):
            directory = ROOT / self.table[key]
            with self.subTest(key=key):
                self.assertTrue((directory / "summary.json").is_file(), key)
        self.assertEqual(self.table["planar_reference_attempt"], PLANAR_REFERENCE)

    # -- the plan's gate table -------------------------------------------

    def test_every_plan_gate_is_recorded_with_a_pass_flag(self):
        self.assertEqual(tuple(self.gates), PLAN_GATES)
        for name in PLAN_GATES:
            with self.subTest(gate=name):
                self.assertIn("pass", self.gates[name])
                self.assertIsInstance(self.gates[name]["pass"], bool)

    def test_every_gate_records_its_definition_source_and_runner_keys(self):
        self.assertEqual(tuple(self.table["gate_source_mapping"]), PLAN_GATES)
        for name in PLAN_GATES:
            record = self.gates[name]
            with self.subTest(gate=name):
                self.assertTrue(record["definition"].strip())
                self.assertTrue(record["source_attempt"])
                self.assertIsInstance(record["runner_keys"], dict)
                self.assertIsInstance(record["evidence"], dict)

    def test_every_gate_carries_its_evidence_fields(self):
        for name in PLAN_GATES:
            with self.subTest(gate=name):
                missing = [key for key in REQUIRED_EVIDENCE[name]
                           if key not in self.gates[name]["evidence"]]
                self.assertEqual(missing, [], f"{name} is missing evidence {missing}")

    def test_every_passing_gate_has_measured_evidence(self):
        for name in PLAN_GATES:
            record = self.gates[name]
            if not record["pass"]:
                continue
            with self.subTest(gate=name):
                empty = [key for key in NON_NULL_EVIDENCE[name]
                         if record["evidence"][key] is None]
                self.assertEqual(empty, [], f"{name} passes with empty evidence {empty}")

    def test_gate_verdicts_agree_with_the_runner_summaries_they_cite(self):
        for name in PLAN_GATES:
            record = self.gates[name]
            summary = attempt_summary(record)
            for key, value in record["runner_keys"].items():
                with self.subTest(gate=name, key=key):
                    self.assertEqual(summary.get(key), value)
                    self.assertIn(key, summary)
            runner_key = RUNNER_GATE_KEYS[name]
            if runner_key is None:
                continue
            with self.subTest(gate=name, runner_key=runner_key):
                self.assertEqual(record["runner_keys"][runner_key], summary.get(runner_key))
                if name == "table_domain_valid":
                    self.assertEqual(record["pass"], summary[runner_key] == 0)
                else:
                    self.assertEqual(record["pass"], summary[runner_key] is True)

    def test_passing_table_domain_and_travel_gates_match_the_runner_keys(self):
        self.assertEqual(self.gates["table_domain_valid"]["pass"],
                         self.summary["out_of_range_evaluations"] == 0)
        self.assertEqual(self.gates["travel_valid"]["pass"],
                         self.summary["jounce_travel_valid"])

    def test_travel_evidence_agrees_with_the_runner_gate_inputs(self):
        evidence = self.gates["travel_valid"]["evidence"]
        accepted = max(abs(evidence["jounce_accepted_minimum_m"]),
                       abs(evidence["jounce_accepted_maximum_m"]))
        trace_extreme = evidence["maximum_absolute_jounce_m"]
        lower, upper = evidence["jounce_travel_limits_m"]
        # The gate input is the accepted nodes plus the per-interval dense
        # bounds, so it can only be wider than the trace-node extreme, and the
        # two must agree to the interval-bound padding (nanometres here).
        self.assertLessEqual(trace_extreme, accepted + 1e-9)
        self.assertLessEqual(accepted, trace_extreme + 1e-6)
        self.assertGreater(evidence["jounce_accepted_maximum_m"], 0.)
        self.assertLess(evidence["jounce_accepted_minimum_m"], 0.)
        self.assertLess(trace_extreme, min(abs(lower), abs(upper)))
        self.assertEqual(evidence["bump_stop_entry_count"],
                         sum(evidence["bump_stop_entry_count_per_wheel"]))
        if trace_extreme < evidence["bump_stop_blend_start_m"]:
            self.assertEqual(evidence["bump_stop_entry_count"], 0)
        self.assertGreater(evidence["bump_stop_peak_wheel_force_n"],
                           evidence["minimum_wheel_force_n"])
        # The per-wheel extremes bracket the whole-trace extreme.
        self.assertAlmostEqual(
            max(abs(value) for extremes in evidence["jounce_extremes_per_wheel_m"]
                for value in extremes), trace_extreme, places=12)

    def test_table_domain_evidence_is_inside_every_compiled_table(self):
        evidence = self.gates["table_domain_valid"]["evidence"]
        self.assertLessEqual(evidence["maximum_damper_velocity_mps"],
                             evidence["damper_table_velocity_limit_mps"])
        self.assertIs(evidence["roll_centre_query_clipped"], False)
        self.assertLess(evidence["maximum_absolute_roll_deg"],
                        evidence["roll_centre_table_limit_deg"])
        lower, upper = evidence["spring_curve_jounce_domain_m"]
        accepted_lower, accepted_upper = evidence["jounce_accepted_extremes_m"]
        self.assertGreaterEqual(accepted_lower, lower)
        self.assertLessEqual(accepted_upper, upper)
        self.assertEqual(evidence["jounce_steer_baseline_toe_rad"], 0.0)

    # -- provenance -------------------------------------------------------

    def test_recorded_source_hashes_agree_with_the_working_files(self):
        hashes = self.summary["source_hashes"]
        self.assertTrue(hashes)
        for name, recorded in hashes.items():
            path = Path(name)
            if not path.is_absolute():
                path = ROOT / path
            with self.subTest(source=name):
                self.assertTrue(path.is_file(), name)
                self.assertEqual(sha256_file(path), recorded, name)
        self.assertEqual(self.table["source_hash_agreement"]["mismatches"], [])
        self.assertTrue(self.table["source_hash_agreement"]["all_sources_match"])
        self.assertEqual(self.table["source_hash_agreement"]["sources_checked"], len(hashes))

    def test_the_gate_tool_and_model_provenance_are_recorded(self):
        self.assertEqual(self.table["gate_tool"]["path"],
                         GATE_TOOL.relative_to(ROOT).as_posix())
        self.assertEqual(self.table["gate_tool"]["sha256"], sha256_file(GATE_TOOL))
        provenance = self.summary["model_provenance"]
        self.assertEqual(provenance["transient_tire_doi"], "10.3390/machines10070527")
        for key in ("steady_tire_xml_sha256", "transient_tire_xml_sha256"):
            self.assertEqual(provenance[key],
                             self.summary["source_hashes"][provenance[
                                 "transient_tire_xml" if key.startswith("transient")
                                 else "steady_tire_xml"]])

    # -- the paired runs --------------------------------------------------

    def test_periodicity_attempt_is_a_warm_start_of_the_acceptance_lap(self):
        evidence = self.gates["periodicity_valid"]["evidence"]
        self.assertEqual(evidence["warm_start_source"],
                         self.attempt.relative_to(ROOT).as_posix())
        self.assertTrue(evidence["recorded_tolerances_match_plant"])
        self.assertEqual(evidence["vertical_tolerances"]["heave"], 5.0e-4)
        self.assertEqual(evidence["vertical_tolerances"]["roll"], 5.0e-3)
        self.assertEqual(evidence["vertical_tolerances"]["jounce_rate_fl"], 1.0e-2)
        self.assertEqual(evidence["tire_state_tolerances"]["kappa_eff"], 2.0e-3)
        self.assertEqual(evidence["tire_state_tolerances"]["lateral_deformation"], 2.0e-4)
        self.assertEqual(evidence["tire_state_tolerances"]["trail_angle"], 2.0e-3)

    def test_cross_fidelity_reference_is_the_bounded_corrected_planar_lap(self):
        evidence = self.table["reports"]["cross_fidelity"]["evidence"]
        self.assertEqual(evidence["reference_model"], "planar-7dof")
        self.assertEqual(evidence["reference_attempt"], PLANAR_REFERENCE)
        self.assertTrue(evidence["both_laps_completed"])
        # The 7DoF reference is the Task 8 Step 5 corrected-geometry lap; it is
        # `usable=false` (path tracking and periodicity), which the comparison
        # must disclose rather than hide behind its own passing row.
        self.assertIs(evidence["reference_usable"], False)
        self.assertIs(evidence["reference_path_tracking_valid"], False)
        self.assertGreater(evidence["reference_maximum_lateral_error_m"], 0.25)
        self.assertEqual(evidence["speed_trace_file"], "cross_fidelity_speed.csv")

    def test_no_gate_claims_a_quality_that_the_summary_denies(self):
        # The acceptance lap is a cold, non-periodic transient: the periodicity
        # gate is decided on the warm attempt, never claimed for the cold lap.
        self.assertIs(self.summary["periodicity_valid"], False)
        self.assertIs(self.summary["usable"], False)
        self.assertIs(self.summary["path_tracking_valid"], False)

    def test_plan_gates_block_is_the_plan_table_with_its_own_names(self):
        self.assertEqual(tuple(self.table["plan_gates"]), PLAN_GATES)
        for name in PLAN_GATES:
            with self.subTest(gate=name):
                self.assertIsInstance(self.table["plan_gates"][name], bool)
                self.assertEqual(self.table["plan_gates"][name], self.gates[name]["pass"])
        self.assertEqual(self.table["all_gates_pass"],
                         all(self.table["plan_gates"][name] for name in PLAN_GATES))
        # The removed checks are named, not silently dropped, and the
        # cross-fidelity comparison is a report, never a gate.
        self.assertEqual(set(self.table["removed_gates"]),
                         {"refinement_validated", "between_step_validation_complete"})
        self.assertNotIn("cross_fidelity_reported", self.table["plan_gates"])
        self.assertNotIn("pass", self.table["reports"]["cross_fidelity"])
        for name in ("completed", "travel_valid", "table_domain_valid"):
            with self.subTest(plan_gate=name):
                self.assertNotIn(name, self.summary)
        self.assertIn("jounce_travel_valid", self.summary)
        self.assertIn("out_of_range_evaluations", self.summary)

    def test_trace_provenance_is_recorded_for_the_three_attempts(self):
        traces = self.table["traces"]
        self.assertEqual(set(traces), {"attempt", "periodicity_attempt",
                                       "planar_reference_attempt"})
        for role, record in traces.items():
            with self.subTest(role=role):
                self.assertTrue(record["trace"].endswith("trace.csv"))
                self.assertIsInstance(record["present"], bool)
                if record["present"]:
                    self.assertRegex(record["sha256"], r"^[0-9a-f]{64}$")
                    self.assertGreater(record["bytes"], 0)
                    self.assertEqual(record["columns"], 39 if "planar" not in role else 13)
        if traces["attempt"]["present"]:
            self.assertEqual(traces["attempt"]["rows"], self.summary["accepted_steps"] + 1)
        for role in ("periodicity_attempt",):
            directory = ROOT / self.table[role]
            summary = read_json(directory / "summary.json")
            if traces[role]["present"]:
                with self.subTest(role=role):
                    self.assertEqual(traces[role]["rows"], summary["accepted_steps"] + 1)

    def test_planar_reference_provenance_is_recorded_not_gated(self):
        agreement = self.table["planar_reference_source_hash_agreement"]
        self.assertEqual(agreement["sources_checked"], 14)
        self.assertFalse(agreement["all_sources_match"])
        # The reference is a historical lap, produced by the Task 8 runner before its
        # final review fixes. Three recorded sources differ from the working tree: the
        # runner itself, and forward_driver.py / planar_forward.py, which later gained
        # the additive run-record hook (a step_sink callback and last_evaluation
        # attributes; it changes no number). Every shared model file matches.
        expected = {"run_iem24_forward_driver.py", "forward_driver.py", "planar_forward.py"}
        self.assertEqual({entry["source"].rsplit("/", 1)[-1]
                          for entry in agreement["mismatches"]}, expected)
        self.assertEqual(len(agreement["mismatches"]), len(expected))
        for entry in agreement["mismatches"]:
            self.assertEqual(entry["reason"], "sha256 differs")
        mismatched = [entry for entry in agreement["sources"] if not entry["matches"]]
        self.assertEqual(len(mismatched), len(expected))
        for entry in mismatched:
            self.assertNotEqual(entry["recorded_sha256"], entry["working_sha256"])
        for entry in agreement["sources"]:
            if entry["source"].rsplit("/", 1)[-1] in expected:
                continue
            with self.subTest(source=entry["source"]):
                self.assertTrue(entry["matches"])
        # A provenance difference must not fail a gate.
        self.assertTrue(all(self.table["plan_gates"][name] for name in PLAN_GATES))

    def test_cross_fidelity_evidence_quotes_both_sampling_grids(self):
        evidence = self.table["reports"]["cross_fidelity"]["evidence"]
        self.assertEqual(evidence["matched_sample_count"], 200)
        self.assertEqual(evidence["fine_grid_sample_count"], 2001)
        # Both densities are recorded, and the finer grid can only find a larger
        # extreme. Each extreme's distance is recorded with its own grid, so a
        # document can quote either one without mixing sampling densities.
        self.assertLessEqual(evidence["speed_difference_maximum_mps"],
                             evidence["fine_grid_speed_difference_maximum_mps"])
        for key in ("speed_difference_maximum_distance_m",
                    "fine_grid_speed_difference_maximum_distance_m"):
            with self.subTest(key=key):
                self.assertGreater(evidence[key], 0.)
                self.assertLessEqual(evidence[key], evidence["matched_distance_m"])
        self.assertLessEqual(evidence["tail_maximum_speed_difference_mps"],
                             evidence["speed_difference_maximum_mps"])
        self.assertLessEqual(evidence["fine_grid_tail_maximum_speed_difference_mps"],
                             evidence["fine_grid_speed_difference_maximum_mps"])


class PinnedTraceEvidenceTests(unittest.TestCase):
    """Re-derive the trace-based gate evidence from the pinned traces.

    ``gates.json`` is written by the gate tool, so on its own it is a claim about
    itself. These tests re-derive the periodicity gate and the cross-fidelity
    comparison from the raw traces with the gate tool's own functions and compare
    them with the recorded table, so a perturbed trace, a zeroed summary delta or
    an edited record fails instead of passing.

    The traces are gitignored per-attempt artifacts: in a fresh checkout they are
    absent, and the re-derivation tests skip with a message naming what to
    regenerate. When a trace is *present* but does not match the sha256 recorded
    in ``gates.json``, ``test_present_traces_match_their_pinned_sha256`` fails
    (a changed trace is tampering or a stale pin) while the re-derivation tests
    skip, because they cannot re-derive the pinned numbers from a different file.
    """

    @classmethod
    def setUpClass(cls):
        cls.attempt = acceptance_attempt()
        cls.table = read_json(cls.attempt / "gates.json")
        cls.summary = read_json(cls.attempt / "summary.json")
        cls.periodicity = ROOT / cls.table["periodicity_attempt"]
        cls.reference = ROOT / cls.table["planar_reference_attempt"]

    def record(self, role: str) -> dict:
        return self.table["traces"][role]

    def pinned_trace(self, role: str):
        """Return one trace, or skip when it cannot be re-derived."""
        record = self.record(role)
        if not record["present"]:
            self.skipTest(
                f"{record['trace']} is absent (a gitignored per-attempt artifact), so the "
                f"trace-derived evidence cannot be re-derived here; regenerate the attempt "
                f"and re-run the gate tool")
        path = ROOT / record["trace"]
        digest = sha256_file(path)
        if digest != record["sha256"]:
            self.skipTest(
                f"{record['trace']} sha256 {digest} does not match the pinned "
                f"{record['sha256']}; re-derive after regenerating the attempt and re-running "
                f"the gate tool")
        return read_trace(path)

    def test_present_traces_match_their_pinned_sha256(self):
        for role in ("attempt", "periodicity_attempt", "planar_reference_attempt"):
            record = self.record(role)
            if not record["present"]:
                continue  # gitignored and absent: nothing to check here
            with self.subTest(role=role):
                self.assertRegex(record["sha256"], r"^[0-9a-f]{64}$")
                self.assertEqual(sha256_file(ROOT / record["trace"]), record["sha256"],
                                 f"{record['trace']} does not match the gate table's pinned "
                                 f"sha256; re-run the gate tool after regenerating the attempt")

    def test_periodicity_closure_is_reproduced_from_the_pinned_trace(self):
        trace = self.pinned_trace("periodicity_attempt")
        summary = read_json(self.periodicity / "summary.json")
        evidence = self.table["gates"]["periodicity_valid"]["evidence"]
        names = tuple(VerticalForwardPlant.state_names)
        indices = list(VerticalForwardPlant.periodic_indices)
        closure = trace[-1, 1:][indices] - trace[0, 1:][indices]
        recorded = np.asarray(summary["periodic_state_delta"], dtype=float)
        tolerances = np.asarray(summary["periodic_state_tolerances"], dtype=float)
        self.assertEqual(closure.shape, recorded.shape)
        np.testing.assert_array_equal(closure, recorded)
        self.assertGreater(float(np.max(np.abs(recorded))), 0.,
                           "an all-zero closure would make the periodicity gate vacuous")
        ratio = np.abs(closure) / tolerances
        worst = int(np.argmax(ratio))
        self.assertEqual(float(np.max(ratio)), evidence["maximum_tolerance_ratio"])
        self.assertEqual(float(np.max(ratio)), float(np.max(np.abs(recorded) / tolerances)))
        self.assertEqual(names[indices[worst]], evidence["worst_state"])
        self.assertEqual(float(abs(closure[worst])), evidence["worst_state_absolute_delta"])
        self.assertLessEqual(float(np.max(ratio)), 1.)
        # The plant's own tolerances, not a copy recorded by the runner.
        self.assertTrue(np.array_equal(
            tolerances, np.asarray(VerticalForwardPlant.periodic_tolerances, dtype=float)))

    def test_cross_fidelity_speed_trace_is_reproduced_from_the_pinned_traces(self):
        vertical = self.pinned_trace("attempt")
        planar = self.pinned_trace("planar_reference_attempt")
        evidence = self.table["reports"]["cross_fidelity"]["evidence"]
        last = min(float(vertical[-1, column(VerticalForwardPlant, "s")]),
                   float(planar[-1, column(PlanarForwardPlant, "s")]))
        self.assertEqual(last, evidence["matched_distance_m"])
        distance = np.linspace(0., last, evidence["matched_sample_count"])
        vertical_speed = np.hypot(
            np.interp(distance, vertical[:, column(VerticalForwardPlant, "s")],
                      vertical[:, column(VerticalForwardPlant, "v_x")]),
            np.interp(distance, vertical[:, column(VerticalForwardPlant, "s")],
                      vertical[:, column(VerticalForwardPlant, "v_y")]))
        planar_speed = np.hypot(
            np.interp(distance, planar[:, column(PlanarForwardPlant, "s")],
                      planar[:, column(PlanarForwardPlant, "v_x")]),
            np.interp(distance, planar[:, column(PlanarForwardPlant, "s")],
                      planar[:, column(PlanarForwardPlant, "v_y")]))
        difference = vertical_speed - planar_speed
        self.assertEqual(float(np.sqrt(np.mean(difference ** 2))),
                         evidence["speed_difference_rmse_mps"])
        self.assertEqual(float(np.max(np.abs(difference))),
                         evidence["speed_difference_maximum_mps"])
        self.assertEqual(float(distance[int(np.argmax(np.abs(difference)))]),
                         evidence["speed_difference_maximum_distance_m"])
        # The tracked CSV is that same 200-sample comparison, so a tampered CSV
        # fails here even though the traces are unchanged.
        tracked = np.loadtxt(self.attempt / evidence["speed_trace_file"],
                             delimiter=",", skiprows=1)
        self.assertEqual(tracked.shape[0], evidence["matched_sample_count"])
        np.testing.assert_allclose(tracked[:, 0], distance, rtol=0., atol=5.e-7)
        np.testing.assert_allclose(tracked[:, 3], difference, rtol=0., atol=1.e-9)


if __name__ == "__main__":
    unittest.main()
