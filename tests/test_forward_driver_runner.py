"""Check the hard deadline and attempt retention for forward experiments."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import hashlib
from types import SimpleNamespace

from tools.reports.export_iem24_forward_correlation import verify_track_provenance
from tools.run.run_iem24_forward_driver import (
    DEFAULT_TRACK, LEGACY_TRACK, acceptance_gate, main, parse_args, tire_xml_path, track_files,
    warm_start_matches,
)

CANONICAL_TRACK = "models/tracks/2024_michigan_endurance_multilap_closed.xml"
ORIGINAL_TRACK = "models/tracks/2024_michigan_endurance_closed.xml"
UNSIGNED_TRACK = "models/tracks/2024_michigan_endurance_v11.xml"


class RunnerTests(unittest.TestCase):
    def test_saved_periodic_deltas_reject_attempt(self):
        # Preserve the observed seam deltas without depending on a deleted
        # untracked attempt directory from another session.
        deltas = [-0.1515106233, 0.0033867603, -0.5347458893, 0.2913062140,
                  0.0175601269, -2.7892290513, -3.1768848901, -1.7244119826,
                  -2.9018926387, -0.0104695314]
        tolerances = [.001, .001, .01, .01, .001, .01, .01, .01, .01, .001]
        summary = dict(lap_completed=True, periodic_state_delta=deltas,
                       periodic_state_tolerances=tolerances, periodicity_valid=True,
                       integration_completed=True, path_tracking_valid=True,
                       resolution_valid=True, checked_rhs_physics_valid=True,
                       between_step_validation_complete=True,
                       refinement_validated=True, contact_valid=True)
        self.assertEqual(len(deltas), len(tolerances))
        self.assertTrue(any(abs(delta) > tolerance
                            for delta, tolerance in zip(deltas, tolerances)))
        self.assertFalse(acceptance_gate(summary))
        self.assertFalse(acceptance_gate(summary))

    def test_acceptance_gate_rejects_invalid_periodic_vectors(self):
        valid = {
            "lap_completed": True,
            "periodic_state_delta": [0.0, -0.5],
            "periodic_state_tolerances": [0.01, 0.5],
            "periodicity_valid": True,
            "integration_completed": True,
            "path_tracking_valid": True,
            "resolution_valid": True,
            "checked_rhs_physics_valid": True,
            "between_step_validation_complete": True,
            "refinement_validated": True,
            "contact_valid": True,
        }
        self.assertTrue(acceptance_gate(valid))
        for delta, tolerances in (
            ([], []), ([0.0], []), ([float("inf")], [float("inf")]),
            ([float("nan")], [1.0]), ([0.0], [float("inf")]),
            ([0.0], [-0.1]), (["0"], [1.0]), ([0.0], ["1"]),
        ):
            with self.subTest(delta=delta, tolerances=tolerances):
                candidate = dict(valid, periodic_state_delta=delta,
                                 periodic_state_tolerances=tolerances)
                self.assertFalse(acceptance_gate(candidate))

    def test_limits_reject_nonfinite_and_over_budget_values(self):
        for arguments in (["--timeout-seconds", "301"], ["--timeout-seconds", "nan"],
                          ["--duration", "0"], ["--spacing", ".51"],
                          ["--rtol", "-1"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(arguments)

    def test_timeout_retains_parent_record_and_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch("tools.run.run_iem24_forward_driver.subprocess.run",
                       side_effect=subprocess.TimeoutExpired("child", 1.)) as run:
                with contextlib.redirect_stdout(io.StringIO()):
                    result = main(["--output-root", temporary, "--timeout-seconds", "1"])
            self.assertEqual(result, 124)
            self.assertEqual(run.call_args.kwargs["timeout"], 1.)
            attempts = list(Path(temporary).iterdir())
            self.assertEqual(len(attempts), 1)
            parent = json.loads((attempts[0] / "parent.json").read_text())
            self.assertEqual(parent["status"], "Timed_Out")
            self.assertTrue((attempts[0] / "run.log").exists())

    def test_attempts_do_not_overwrite_each_other(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch("tools.run.run_iem24_forward_driver.subprocess.run",
                       return_value=subprocess.CompletedProcess([], 0)):
                with contextlib.redirect_stdout(io.StringIO()):
                    main(["--output-root", temporary])
                    main(["--output-root", temporary])
            self.assertEqual(len(list(Path(temporary).iterdir())), 2)


    def test_track_defaults_to_canonical_and_accepts_explicit_track(self):
        self.assertEqual(parse_args([]).track, Path(DEFAULT_TRACK))
        self.assertEqual(str(DEFAULT_TRACK), CANONICAL_TRACK)
        self.assertEqual(str(LEGACY_TRACK), ORIGINAL_TRACK)
        self.assertEqual(parse_args(["--track", ORIGINAL_TRACK]).track, Path(ORIGINAL_TRACK))
        self.assertEqual(parse_args(["--track", CANONICAL_TRACK]).track, Path(CANONICAL_TRACK))
        absolute = Path(CANONICAL_TRACK).resolve()
        self.assertEqual(parse_args(["--track", str(absolute)]).track, Path(CANONICAL_TRACK))

    def test_missing_and_unsigned_tracks_are_rejected(self):
        for track in ("models/tracks/does_not_exist.xml", UNSIGNED_TRACK):
            with self.subTest(track=track):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    parse_args(["--track", track])

    def test_track_files_resolve_csv_and_tire(self):
        xml, csv = track_files(CANONICAL_TRACK)
        self.assertEqual(csv, Path("models/tracks/2024_michigan_endurance_multilap_closed.csv"))
        self.assertEqual(tire_xml_path(), Path("models/tires/iem24_murphy_real_unitire.xml"))

    def test_track_is_forwarded_to_child_command(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch("tools.run.run_iem24_forward_driver.subprocess.run",
                       return_value=subprocess.CompletedProcess([], 0)) as run:
                with contextlib.redirect_stdout(io.StringIO()):
                    main(["--output-root", temporary, "--track", CANONICAL_TRACK])
            command = run.call_args.args[0]
            self.assertEqual(command[command.index("--track") + 1], CANONICAL_TRACK)

    def test_warm_start_requires_matching_track_and_hashes(self):
        hashes = {"track.xml": "a", "track.csv": "b", "tire.xml": "c", "form.xml": "d"}
        args = parse_args(["--track", CANONICAL_TRACK])
        keys = dict(planner="conservative", maximum_speed=8., lateral_acceleration=None,
                    pedal_travel_time=.175, spacing=.5)
        previous = {"status": "Completed", "lap_completed": True,
                    "source_hashes": dict(hashes),
                    "settings": dict(keys, track=CANONICAL_TRACK)}
        self.assertTrue(warm_start_matches(previous, hashes, args))
        for change in ({"track": ORIGINAL_TRACK}, {"planner": "live"}):
            candidate = dict(previous, settings=dict(previous["settings"], **change))
            self.assertFalse(warm_start_matches(candidate, hashes, args))
        for name in hashes:  # any track/CSV/tire/formulation hash change rejects
            changed = dict(hashes, **{name: "x"})
            self.assertFalse(warm_start_matches(dict(previous, source_hashes=changed), hashes, args))
        # Historical attempts have no recorded track: they mean the original
        # track, never the (new) default.
        legacy = dict(previous, settings=dict(keys))
        self.assertFalse(warm_start_matches(legacy, hashes, args))
        self.assertFalse(warm_start_matches(legacy, hashes, parse_args([])))
        self.assertTrue(warm_start_matches(legacy, hashes,
                                           parse_args(["--track", ORIGINAL_TRACK])))
        # A new-default run is seeded only by attempts that recorded that track.
        self.assertTrue(warm_start_matches(previous, hashes, parse_args([])))

    def test_export_checks_track_setting_and_recorded_hashes(self):
        paths = {"track_xml": CANONICAL_TRACK,
                 "track_csv": "models/tracks/2024_michigan_endurance_multilap_closed.csv",
                 "tire_xml": "models/tires/iem24_murphy_real_unitire.xml",
                 "formulation_xml": "models/formulations/iem24_murphy_real_planar_7dof.xml",
                 "vehicle_xml": "models/vehicles/iem24_murphy_real_planar.xml"}
        hashes = {path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                  for path in paths.values()}
        summary = {"settings": {"track": CANONICAL_TRACK}, "model_inputs": paths,
                   "source_hashes": hashes}
        self.assertEqual(verify_track_provenance(summary), CANONICAL_TRACK)
        bad = dict(summary, source_hashes=dict(hashes, **{paths["track_csv"]: "0"}))
        with self.assertRaises(ValueError):
            verify_track_provenance(bad)
        bad = dict(summary, settings={"track": ORIGINAL_TRACK})
        with self.assertRaises(ValueError):
            verify_track_provenance(bad)
        with self.assertRaises(ValueError):  # a named track must carry hashes
            verify_track_provenance({"settings": {"track": CANONICAL_TRACK}})
        # Historical attempt without a track setting replays on the original
        # track, not the new default.
        self.assertEqual(verify_track_provenance({"settings": {}}), ORIGINAL_TRACK)
        self.assertNotEqual(ORIGINAL_TRACK, str(DEFAULT_TRACK))


if __name__ == "__main__":
    unittest.main()
