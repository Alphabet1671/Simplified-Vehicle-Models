"""Check the hard deadline and attempt retention for forward experiments."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import hashlib

import numpy as np

from simplified_vehicle_models.compiler.capabilities import (
    SUSPENSION_FORCES, TRANSIENT_COMBINED_SLIP,
)
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.driver_maps import (
    PedalActuatorCalibration,
)
from simplified_vehicle_models.solvers.forward_driver import (
    DriverObservation, ForwardResult, PreviewDriver, RateSystem, SpeedPlan, solve_forward,
)
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.solvers.vertical_forward import (
    VerticalForwardPlant as VerticalPlantLayout,
)
from tools.reports.export_iem24_forward_correlation import verify_track_provenance
from tools.run.run_iem24_forward_driver import (
    DEFAULT_TRACK, LEGACY_TRACK, MODEL_NAMES, acceptance_gate, child, jounce_travel_limits, main,
    model_inputs, model_spec, parse_args, peak_memory_bytes, policy_identity_settings, canonical_json, solver_wall_budget, source_files,
    state_index, tire_xml_path, trace_header, track_files, validated_periodic_tolerances,
    wall_margin_seconds, warm_start_matches, wheel_speed_indices,
)

CANONICAL_TRACK = "models/tracks/2024_michigan_endurance_multilap_closed.xml"
ORIGINAL_TRACK = "models/tracks/2024_michigan_endurance_closed.xml"
UNSIGNED_TRACK = "models/tracks/2024_michigan_endurance_v11.xml"

BASELINE_FIXTURE = Path(__file__).resolve().parent / "fixtures/forward_driver_runner_planar_baseline.json"

PLANAR_STATE_NAMES = (
    "s", "n", "heading_error", "v_x", "v_y", "yaw_rate",
    "omega_fl", "omega_fr", "omega_rl", "omega_rr", "soc", "steering",
)
PLANAR_PERIODIC_INDICES = (1, 2, 3, 4, 5, 6, 7, 8, 9, 11)
PLANAR_PERIODIC_TOLERANCES = np.array([.001, .001, .01, .01, .001, .01, .01, .01, .01, .001])

#: Summary keys that legitimately differ between two runs (wall clock, memory,
#: recorded hashes/environment) or that Task 8 added after the baseline fixture
#: was recorded. Every other key must keep its recorded value.
VOLATILE_SUMMARY_KEYS = (
    "setup_seconds", "child_wall_seconds", "source_hashes", "model_inputs", "environment",
    "settings", "error", "warm_start_source", "model", "requested_wall_seconds",
    "solver_wall_budget_seconds", "solver_elapsed_seconds", "elapsed_wall_seconds",
    "peak_memory_bytes", "peak_memory_source", "pedal_map", "runtime_model_identity",
)

REQUIRED_14DOF_SOURCES = (
    "models/vehicles/iem24_murphy_real_planar.xml",
    "models/formulations/iem24_murphy_real_14dof_simple.xml",
    "models/tires/iem24_murphy_real_unitire.xml",
    "models/tires/iem24_murphy_real_unitire_transient.xml",
    "models/suspensions/iem24_murphy_real_suspension.xml",
    "models/suspension-kinematics/iem24_murphy_real_suspension_kinematics.xml",
    "models/springs/iem24_murphy_real_suspension_springs.xml",
    "models/anti-roll-bars/iem24_murphy_real_suspension_arb.xml",
    "models/dampers/multimatic_dssv_vc01_front.xml",
    "models/dampers/multimatic_dssv_vc01_rear.xml",
    "models/data/multimatic_dssv_vc01.npz",
    "models/data/iem24_murphy_real_corners.npz",
    "src/simplified_vehicle_models/solvers/forward_driver.py",
    "src/simplified_vehicle_models/solvers/forward_powertrain.py",
    "src/simplified_vehicle_models/solvers/vertical_forward.py",
    "src/simplified_vehicle_models/solvers/vertical_equilibrium.py",
    "src/simplified_vehicle_models/formulations/vertical_14dof.py",
    "src/simplified_vehicle_models/tires/unitire_transient.py",
    "src/simplified_vehicle_models/suspension/damper_table.py",
    "src/simplified_vehicle_models/suspension/corner.py",
    "src/simplified_vehicle_models/suspension/components.py",
    "src/simplified_vehicle_models/suspension/elements.py",
    "src/simplified_vehicle_models/suspension/kinematics.py",
    "src/simplified_vehicle_models/suspension/network.py",
    "src/simplified_vehicle_models/suspension/layouts/independent_corner.py",
    "src/simplified_vehicle_models/suspension/monotone_spline.py",
)


class FakePlanarPlant:
    """Stand-in planar plant with the recorded layout and fixed diagnostics.

    The same fake (with the same fixed numbers) recorded
    ``fixtures/forward_driver_runner_planar_baseline.json`` from the pre-Task-8
    runner, so comparing this fake's run against that fixture is a direct
    header/summary regression of the planar path.
    """

    state_size = 12
    state_names = PLANAR_STATE_NAMES
    periodic_indices = PLANAR_PERIODIC_INDICES
    periodic_tolerances = PLANAR_PERIODIC_TOLERANCES

    def __init__(self, vehicle=None, plan=None):
        self.vehicle = vehicle
        self.plan = plan
        self.evaluations = 3
        self.minimum_contact_load = 812.5
        self.maximum_load_residual = 0.0
        self.maximum_power_residual = 0.0
        self.maximum_load_iterations = 4
        self.load_method_counts = {"newton": 2, "fixed-point": 1}
        self.minimum_load_jacobian_determinant = 0.75
        self.lifted_wheel_evaluations = 0
        self.speed_regulated_evaluations = 0
        self.maximum_motor_speed_ratio = 0.4
        self.last_time = 1.0
        self.last_distance = 5.0

    def initial_state(self, speed: float) -> np.ndarray:
        state = np.zeros(self.state_size)
        state[0], state[1], state[2] = 0.0, 0.02, -0.01
        state[3], state[4], state[5] = float(speed), 0.05, 0.1
        state[6:10] = 20.0
        state[10] = 0.829085
        state[11] = 0.03
        return state

    def observe(self, state) -> DriverObservation:
        return DriverObservation(float(state[0]), float(state[1]), float(state[2]),
                                 float(state[3]), float(state[5]), float(state[11]),
                                 float(state[4]), 1.0)

    def evaluate(self, time, state, command) -> RateSystem:
        return RateSystem(np.zeros(self.state_size))


class FakeVerticalPlant(FakePlanarPlant):
    """Stand-in 38-state vertical plant; diagnostics are overridable per test."""

    state_size = VerticalPlantLayout.state_size
    state_names = VerticalPlantLayout.state_names
    periodic_indices = VerticalPlantLayout.periodic_indices
    periodic_tolerances = VerticalPlantLayout.periodic_tolerances
    out_of_range_evaluations = 0
    #: Diagnostics over every evaluation, including rejected trials (the plant's
    #: own ``minimum_jounce``/``maximum_jounce``).
    maximum_jounce = 0.012
    minimum_jounce = -0.004
    maximum_damper_velocity = 0.5
    #: Jounce written into the accepted states the stub solver returns.
    jounce_initial = 0.002

    def initial_state(self, speed: float) -> np.ndarray:
        state = np.zeros(self.state_size)
        state[0], state[1], state[2] = 0.0, 0.02, -0.01
        state[3], state[4], state[5] = float(speed), 0.05, 0.0
        state[9], state[10], state[11] = 0.012, 0.0, 0.0
        state[12:16] = 20.0
        state[16:20] = self.jounce_initial
        state[20:24] = 0.0
        state[24] = 0.829085
        state[25] = 0.03
        return state

    def observe(self, state) -> DriverObservation:
        return DriverObservation(float(state[0]), float(state[1]), float(state[2]),
                                 float(state[3]), float(state[8]), float(state[25]),
                                 float(state[4]), 1.0)


def vertical_fake(**overrides):
    """Return a fake vertical plant class with the given attributes overridden.

    ``FakePlanarPlant.__init__`` sets instance attributes, so a plain subclass
    class attribute would be shadowed; this rebuilds the instance values.
    """

    def __init__(self, vehicle=None, plan=None):
        FakeVerticalPlant.__init__(self, vehicle, plan)
        for name, value in overrides.items():
            setattr(self, name, value)

    return type("FakeVerticalPlantVariant", (FakeVerticalPlant,), {"__init__": __init__})


def current_policy_warm_start(settings, args):
    """Add mandatory current-envelope identity to synthetic prior summaries."""
    from tools.run.run_iem24_forward_driver import policy_identity_settings
    identity = policy_identity_settings(args)
    settings = dict(settings, longitudinal_policy="braking-envelope",
                    envelope_deceleration_margin=args.envelope_deceleration_margin,
                    envelope_speed_points=args.envelope_speed_points,
                    longitudinal_policy_settings_identity=identity,
                    longitudinal_policy_settings_identity_sha256=hashlib.sha256(
                        canonical_json(identity).encode()).hexdigest())
    policy = {"policy": "braking-envelope",
        "deceleration_margin": args.envelope_deceleration_margin,
        "speed_query_points": args.envelope_speed_points,
        "settings_identity": identity,
        "settings_identity_sha256": hashlib.sha256(canonical_json(identity).encode()).hexdigest()}
    return {"status": "Completed", "lap_completed": True,
        "settings": settings, "longitudinal_policy": policy}


def run_fake_child(model, plant_class, lap_completed=True, dense_bounds=None, extra_args=()):
    """Run one child attempt with a fake plant and a stubbed solver.

    Returns the attempt directory, the solver keyword arguments the runner used
    and the child return code.
    """
    root = Path(tempfile.mkdtemp())
    args = parse_args(["--output-root", str(root), "--model", model, *extra_args])
    args.child_attempt = root
    calls = []

    def stub_solve_forward(plant, driver, initial_state, **kwargs):
        calls.append(kwargs)
        initial = np.asarray(initial_state, dtype=float)
        state = np.tile(initial[:, None], (1, 2))
        state[0] = np.array([0.0, 0.25])
        return ForwardResult(
            time=np.array([0.0, 0.5]), state=state, lap_completed=lap_completed,
            wall_seconds=1.25, evaluations=11, message="stub", rejected_domain_trials=0,
            pedal=np.array([0.5, 0.4]), midpoint_sample_validation_complete=True,
            dense_bounds=dense_bounds, dense_polynomials=None, dense_samples=0,
            dense_defect_scaled_max=0.0,
        )

    spec = model_spec(model)
    with patch(f"{spec.plant_module}.{spec.plant_class}", plant_class), patch(
            "simplified_vehicle_models.solvers.forward_driver.solve_forward", stub_solve_forward):
        with contextlib.redirect_stdout(io.StringIO()):
            code = child(args)
    return root, calls, code


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

    def test_warm_start_mismatch_rejected_before_saved_arrays_read(self):
        hashes = {"track.xml": "a"}
        args = parse_args([])
        settings = {"maximum_speed": 8., "lateral_acceleration": None,
                    "pedal_travel_time": .175, "spacing": .5, "preview_seconds": .15,
                    "heading_gain": 1.5, "lateral_gain": 1.5, "speed_gain": 2.,
                    "lateral_margin": None, "track": str(args.track)}
        previous = {"status": "Completed", "lap_completed": True, "source_hashes": hashes,
                    "settings": settings, "pedal_map": {"semantic_sha256": "different"}}
        self.assertFalse(warm_start_matches(previous, hashes, args, "current"))

    def _physical_payload(self):
        return {"units": {"throttle_input": "1", "throttle_effective_request": "1", "brake_input": "1", "brake_effective_request": "1"},
                "throttle_fraction": {"axis": [0, .2, 1], "value": [0, .1, 1]},
                "brake_force_fraction": {"axis": [0, .25, 1], "value": [0, .1, 1]}}

    def test_invalid_map_variants_fail_before_attempt_or_spawn(self):
        cases = [b'{"schema_version": 1, "bad": NaN}',
                 b'{"schema_version": 2}',
                 json.dumps({"schema_version": 1, "demand_map": None}).encode(),
                 b'{"schema_version": 1, "mystery": 1}']
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.json"
            for payload in cases:
                path.write_bytes(payload)
                with self.subTest(payload=payload), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    parse_args(["--pedal-map-config", str(path)])
            with patch("tools.run.run_iem24_forward_driver.subprocess.run") as spawn:
                path.write_bytes(cases[-1])
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(["--output-root", temporary, "--pedal-map-config", str(path)])
                spawn.assert_not_called()

    def test_track_is_forwarded_to_child_command(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch("tools.run.run_iem24_forward_driver.subprocess.run",
                       return_value=subprocess.CompletedProcess([], 0)) as run:
                with contextlib.redirect_stdout(io.StringIO()):
                    main(["--output-root", temporary, "--track", CANONICAL_TRACK])
            command = run.call_args.args[0]
            self.assertEqual(command[command.index("--track") + 1], CANONICAL_TRACK)

    def test_asymmetric_slew_flags_validate_forward_and_match_warm_starts(self):
        flags = ("--throttle-apply-rate", "4", "--throttle-release-rate", "6",
                 "--brake-apply-rate", "5", "--brake-release-rate", "6")
        args = parse_args(list(flags))
        self.assertEqual(args.throttle_apply_rate, 4.)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--throttle-apply-rate", "4"])
        with tempfile.TemporaryDirectory() as temporary:
            with patch("tools.run.run_iem24_forward_driver.subprocess.run",
                       return_value=subprocess.CompletedProcess([], 0)) as run:
                with contextlib.redirect_stdout(io.StringIO()):
                    main(["--output-root", temporary, *flags])
            command = run.call_args.args[0]
            for flag, value in zip(flags[::2], flags[1::2]):
                self.assertEqual(command[command.index(flag) + 1], value)
            parent = json.loads(next(Path(temporary).glob("attempt_*/parent.json")).read_text())
        expected = {"schema_version": 1, "effective_normalized_rates": {
            "throttle_apply_rate": 4., "throttle_release_rate": 6.,
            "brake_apply_rate": 5., "brake_release_rate": 6.}}
        self.assertEqual(parent["pedal_slew"], expected)
        settings = {"maximum_speed": 8.,
                    "lateral_acceleration": None, "pedal_travel_time": .175,
                    "spacing": .5, "preview_seconds": .15, "heading_gain": 1.5,
                    "lateral_gain": 1.5, "speed_gain": 2., "lateral_margin": None,
                    "track": str(args.track), "throttle_apply_rate": 4.,
                    "throttle_release_rate": 6., "brake_apply_rate": 5.,
                    "brake_release_rate": 6.}
        previous = current_policy_warm_start(settings, args)
        previous["source_hashes"] = {"source": "hash"}
        self.assertTrue(warm_start_matches(previous, {"source": "hash"}, args))
        changed = dict(settings, brake_release_rate=7.)
        self.assertFalse(warm_start_matches(dict(previous, settings=changed),
                                            {"source": "hash"}, args))
        omitted = parse_args([])
        legacy_settings = {k: v for k, v in settings.items() if not k.endswith("_rate")}
        legacy = current_policy_warm_start(legacy_settings, omitted)
        legacy["source_hashes"] = {"source": "hash"}
        self.assertTrue(warm_start_matches(legacy, {"source": "hash"}, omitted))

        root, calls, code = run_fake_child("planar-7dof", FakePlanarPlant,
                                           extra_args=flags)
        self.assertEqual(code, 0)
        self.assertIsNone(calls[0].get("initial_pedal"))
        summary = json.loads((root / "summary.json").read_text())
        expected = {"schema_version": 1, "effective_normalized_rates": {
            "throttle_apply_rate": 4., "throttle_release_rate": 6.,
            "brake_apply_rate": 5., "brake_release_rate": 6.}}
        self.assertEqual(summary["pedal_slew"], expected)
        self.assertEqual({name: summary["settings"][name] for name in
                          ("throttle_apply_rate", "throttle_release_rate",
                           "brake_apply_rate", "brake_release_rate")},
                         expected["effective_normalized_rates"])

        class Sink:
            rows = 2
            def close(self, manifest, created_utc):
                self.manifest = manifest
                return Path("run_record.npz")
        sink = Sink()
        from tools.run.run_iem24_forward_driver import write_run_record
        args_record = parse_args([*flags, "--record"])
        write_run_record(sink, args_record, summary, {}, model_spec("planar-7dof"),
                         SimpleNamespace(time=np.array([0., 1.]), lap_completed=True,
                                         wall_seconds=1.), 0.)
        self.assertEqual(sink.manifest["pedal_slew"], expected)

    def test_driver_settings_are_parsed_recorded_and_match_warm_starts(self):
        args = parse_args(["--preview-seconds", ".25", "--heading-gain", "2.0",
                           "--lateral-gain", ".75", "--track", str(DEFAULT_TRACK)])
        self.assertEqual(args.preview_seconds, .25)
        self.assertEqual(args.heading_gain, 2.0)
        self.assertEqual(args.lateral_gain, .75)
        root, _, code = run_fake_child(
            "planar-7dof", FakePlanarPlant,
            extra_args=("--preview-seconds", ".25", "--heading-gain", "2.0",
                        "--lateral-gain", ".75"))
        self.assertEqual(code, 0)
        recorded = json.loads((root / "summary.json").read_text())["settings"]
        self.assertEqual({name: recorded[name] for name in
                          ("preview_seconds", "heading_gain", "lateral_gain")},
                         {"preview_seconds": .25, "heading_gain": 2.0,
                          "lateral_gain": .75})
        hashes = {"track.xml": "a"}
        settings = {"maximum_speed": 8.,
                    "lateral_acceleration": None, "pedal_travel_time": .175,
                    "spacing": .5, "preview_seconds": .25,
                    "heading_gain": 2.0, "lateral_gain": .75,
                    "track": str(DEFAULT_TRACK)}
        previous = current_policy_warm_start(settings, args)
        previous["source_hashes"] = hashes
        self.assertTrue(warm_start_matches(previous, hashes, args))
        for key, value in (("preview_seconds", .15), ("heading_gain", 1.5),
                           ("lateral_gain", 1.5)):
            changed = dict(settings, **{key: value})
            self.assertFalse(warm_start_matches(dict(previous, settings=changed), hashes, args))

    def test_envelope_child_binds_calibration_builds_once_and_writes_accepted_policy(self):
        from dataclasses import replace
        from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope
        from simplified_vehicle_models.solvers.driver_maps import parse_pedal_map_config
        from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy

        payload = {"schema_version": 1,
                   "physical_calibration": self._physical_payload()}
        calibration = parse_pedal_map_config(payload).physical_calibration
        root = Path(tempfile.mkdtemp())
        (root / "calibration.json").write_text(json.dumps(payload))
        args = parse_args(["--output-root", str(root), "--duration", ".05",
                           "--pedal-map-config", str(root / "calibration.json")])
        args.child_attempt = root / "attempt"
        args.child_attempt.mkdir()
        calls = {"compile": 0, "build": 0, "solve": 0}
        from dataclasses import dataclass
        @dataclass(frozen=True)
        class Controller:
            actuator_calibration: object = None
        @dataclass(frozen=True)
        class Assembly:
            controller: object
        @dataclass(frozen=True)
        class Runtime:
            drivetrain: object
            powertrain: object
        @dataclass(frozen=True)
        class Vehicle:
            wheelbase: float
            steering_maximum_angle: float
            steering_maximum_rate: float
            maximum_pedal_force: float
            wheel_inertia: float
            drivetrain: object
            controller: object
        original_controller = Controller()
        assembly = Assembly(original_controller)
        compiled_vehicle = Vehicle(1.5, .5, 2., 1000., 1.,
            Runtime(assembly, SimpleNamespace(energy_storage=SimpleNamespace(initial_state_of_charge=.8))),
            original_controller)
        track = SimpleNamespace(lap_length=1., path_coordinates=object(), is_closed=False)
        plan = SpeedPlan(np.array([0., 1.]), np.array([1., 1.]), np.zeros(2), is_closed=False)
        envelope = BrakingEnvelope(np.array([0., 1.]), np.zeros(2), np.array([8., 8.]),
            np.array([8., 8.]), np.array([2.]), np.array([0, 1]), False, 8.,
            {"method": "synthetic", "segment_speed_domain_max": (8.,), "oracle_calls": 9})

        def compile_stub(_):
            calls["compile"] += 1
            return SimpleNamespace(model=compiled_vehicle)

        def build_stub(distance, curvature, vehicle, **kwargs):
            calls["build"] += 1
            self.assertEqual(vehicle.controller.actuator_calibration, calibration)
            self.assertEqual(vehicle.drivetrain.drivetrain.controller.actuator_calibration, calibration)
            np.testing.assert_array_equal(distance, [0., .5, 1.])
            np.testing.assert_array_equal(curvature, np.zeros_like(distance))
            self.assertEqual(kwargs["storage_state"].state_of_charge, .8)
            return envelope

        def solve_stub(plant, driver, initial_state, **kwargs):
            calls["solve"] += 1
            self.assertIsInstance(driver.envelope_policy, EnvelopePolicy)
            self.assertEqual(driver.plan, plan)
            driver = replace(driver, pedal_travel_time=.175)
            # Actual solve_forward accepted-state callback exercises the production stages.
            # One accepted callback must preserve the ordinary sink and use only
            # the cached decision, without an additional plant observation/evaluation.
            before = plant.observations
            result = solve_forward(plant, driver, initial_state, **kwargs)
            self.assertGreater(plant.observations, before)
            return result

        class CountingPlant(FakePlanarPlant):
            state_size = 12
            state_names = PLANAR_STATE_NAMES
            periodic_indices = PLANAR_PERIODIC_INDICES
            periodic_tolerances = PLANAR_PERIODIC_TOLERANCES
            def __init__(self, vehicle=None, plan=None):
                super().__init__(vehicle, plan)
                self.observations = 0
            def observe(self, state):
                self.observations += 1
                return super().observe(state)

        spec = model_spec("planar-7dof")
        with patch("simplified_vehicle_models.compiler.compile.compile_formulation", side_effect=compile_stub), \
             patch("simplified_vehicle_models.tracks.profile.load_track", return_value=track), \
             patch("simplified_vehicle_models.tracks.path_coordinates.evaluate_path",
                   return_value=(None, None, None, np.zeros(3))), \
             patch("simplified_vehicle_models.solvers.forward_driver.build_speed_plan", return_value=plan), \
             patch("simplified_vehicle_models.solvers.braking_envelope.build_braking_envelope", side_effect=build_stub), \
             patch("tools.run.run_iem24_forward_driver.plant_class_for", return_value=CountingPlant), \
             patch("simplified_vehicle_models.solvers.forward_driver.solve_forward", side_effect=solve_stub):
            with contextlib.redirect_stdout(io.StringIO()):
                code = child(args)
        self.assertEqual(code, 2)  # runner status is rejected for synthetic model, after solver succeeds
        self.assertEqual(calls, {"compile": 1, "build": 1, "solve": 1})
        original_attempt = args.child_attempt
        summary = json.loads((original_attempt / "summary.json").read_text())
        policy = summary["longitudinal_policy"]
        self.assertEqual(policy["policy"], "braking-envelope")
        self.assertEqual(policy["array_sha256"]["distance"], hashlib.sha256(np.array([0., 1.]).tobytes()).hexdigest())
        with np.load(args.child_attempt / "envelope_arrays.npz") as arrays:
            np.testing.assert_array_equal(arrays["distance"], [0., 1.])
        self.assertIn("src/simplified_vehicle_models/solvers/braking_envelope.py", summary["source_hashes"])
        self.assertIn("braking-envelope", json.loads((args.child_attempt / "summary.json").read_text())["settings"]["longitudinal_policy"])
        decisions = [json.loads(line) for line in (args.child_attempt / "accepted_policy.jsonl").read_text().splitlines()]
        self.assertIn("reference_speed", decisions[0]["decision"])
        self.assertIn("requested_command", decisions[0])
        self.assertIn("mapped_command", decisions[0])
        self.assertIn("applied_command", decisions[0])
        self.assertNotEqual(decisions[0]["requested_command"], decisions[0]["applied_command"])
        sidecar = json.loads((args.child_attempt / "accepted_policy.json").read_text())
        self.assertEqual(sidecar["status"], "complete")
        self.assertEqual(sidecar["accepted_decision_count"], len(decisions))
        self.assertEqual(summary["accepted_policy_decisions"], len(decisions))
        args.child_attempt = root / "attempt_failure"
        args.child_attempt.mkdir()
        def solve_then_fail(plant, driver, initial_state, **kwargs):
            calls["solve"] += 1
            self.assertIsInstance(driver.envelope_policy, EnvelopePolicy)
            self.assertEqual(driver.plan, plan)
            driver = replace(driver, pedal_travel_time=.175)
            before = plant.observations
            result = solve_forward(plant, driver, initial_state, **kwargs)
            self.assertGreater(plant.observations, before)
            raise RuntimeError("controlled post-acceptance failure")
        with patch("simplified_vehicle_models.compiler.compile.compile_formulation", side_effect=compile_stub), \
             patch("simplified_vehicle_models.tracks.profile.load_track", return_value=track), \
             patch("simplified_vehicle_models.tracks.path_coordinates.evaluate_path",
                   return_value=(None, None, None, np.zeros(3))), \
             patch("simplified_vehicle_models.solvers.forward_driver.build_speed_plan", return_value=plan), \
             patch("simplified_vehicle_models.solvers.braking_envelope.build_braking_envelope", side_effect=build_stub), \
             patch("tools.run.run_iem24_forward_driver.plant_class_for", return_value=CountingPlant), \
             patch("simplified_vehicle_models.solvers.forward_driver.solve_forward", side_effect=solve_then_fail):
            with contextlib.redirect_stdout(io.StringIO()):
                failed_code = child(args)
        self.assertEqual(failed_code, 2)
        failure_marker = json.loads((args.child_attempt / "accepted_policy.json").read_text())
        self.assertEqual(failure_marker["status"], "accepted_prefix_incomplete")
        self.assertEqual(failure_marker["accepted_decision_count"], len(decisions))
        failed_rows = [json.loads(line) for line in (args.child_attempt / "accepted_policy.jsonl").read_text().splitlines()]
        self.assertEqual(len(failed_rows), len(decisions))
        args.child_attempt = root / "attempt_setup_failure"
        args.child_attempt.mkdir()
        with patch("simplified_vehicle_models.compiler.compile.compile_formulation",
                   side_effect=RuntimeError("setup failure original")):
            with contextlib.redirect_stdout(io.StringIO()):
                setup_code = child(args)
        self.assertEqual(setup_code, 2)
        setup_summary = json.loads((args.child_attempt / "summary.json").read_text())
        self.assertIn("setup failure original", setup_summary["error"])
        self.assertEqual(setup_summary["longitudinal_policy"]["settings_identity_status"],
                         "unavailable_before_policy_build")
        self.assertIsNone(setup_summary["longitudinal_policy"]["settings_identity_sha256"])
        self.assertNotIn("settings_identity", setup_summary["longitudinal_policy"])

    def test_policy_warm_start_matches_and_rejects_different_configuration(self):
        hashes = {"track": "x"}
        args = parse_args([])
        common = dict(maximum_speed=8., lateral_acceleration=None,
                      pedal_travel_time=.175, spacing=.5, preview_seconds=.15,
                      heading_gain=1.5, lateral_gain=1.5, track=DEFAULT_TRACK)
        identity = policy_identity_settings(args)
        identity["builder"].update(storage_state_of_charge=.8, is_closed=False,
                                   builder_parameters={"method": "synthetic"})
        prior = {"status": "Completed", "lap_completed": True, "source_hashes": hashes,
                 "settings": dict(common, longitudinal_policy="braking-envelope",
                     envelope_deceleration_margin=.6, envelope_speed_points=9),
                 "longitudinal_policy": {"policy": "braking-envelope",
                 "deceleration_margin": .6, "speed_query_points": 9,
                 "settings_identity_sha256": hashlib.sha256(canonical_json(identity).encode()).hexdigest()}}
        self.assertTrue(warm_start_matches(prior, hashes, args, policy_settings_identity=identity))
        self.assertFalse(warm_start_matches(dict(prior, longitudinal_policy={
            "policy": "braking-envelope", "deceleration_margin": .5,
            "speed_query_points": 9}), hashes, args, policy_settings_identity=identity))
        changed_identity = json.loads(canonical_json(identity))
        changed_identity["settings"]["response_allowance_s"] = .2
        self.assertFalse(warm_start_matches(prior, hashes, args, policy_settings_identity=changed_identity))
        changed_identity = json.loads(canonical_json(identity))
        changed_identity["builder"]["builder_parameters"]["method"] = "different"
        self.assertFalse(warm_start_matches(prior, hashes, args, policy_settings_identity=changed_identity))
        self.assertFalse(warm_start_matches(dict(prior, settings=dict(prior["settings"],
            envelope_deceleration_margin=.5)), hashes, args))
        self.assertFalse(warm_start_matches(dict(prior, longitudinal_policy={
            "policy": "braking-envelope", "deceleration_margin": .6,
            "speed_query_points": 11}), hashes, args))

    def test_warm_start_requires_matching_track_and_hashes(self):
        hashes = {"track.xml": "a", "track.csv": "b", "tire.xml": "c", "form.xml": "d"}
        args = parse_args(["--track", CANONICAL_TRACK])
        keys = dict(maximum_speed=8., lateral_acceleration=None,
                    pedal_travel_time=.175, spacing=.5, preview_seconds=.15,
                    heading_gain=1.5, lateral_gain=1.5, speed_gain=2.,
                    lateral_margin=None, track=CANONICAL_TRACK)
        previous = current_policy_warm_start(keys, args)
        previous["source_hashes"] = dict(hashes)
        self.assertTrue(warm_start_matches(previous, hashes, args))
        for change in ({"track": ORIGINAL_TRACK},):
            candidate = dict(previous, settings=dict(previous["settings"], **change))
            self.assertFalse(warm_start_matches(candidate, hashes, args))
        for name in hashes:  # any track/CSV/tire/formulation hash change rejects
            changed = dict(hashes, **{name: "x"})
            self.assertFalse(warm_start_matches(dict(previous, source_hashes=changed), hashes, args))
        # Historical attempts have no recorded track: they mean the original
        # track, never the (new) default.
        legacy = current_policy_warm_start({key:value for key,value in keys.items() if key != "track"}, args)
        legacy["source_hashes"] = dict(hashes)
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


class ForwardRunnerModelTests(unittest.TestCase):
    """Task 8: the runner is driven by the plant metadata of the selected model."""

    def test_default_model_is_the_planar_rung(self):
        self.assertEqual(MODEL_NAMES, ("planar-7dof", "14dof-simple"))
        self.assertEqual(parse_args([]).model, "planar-7dof")
        self.assertEqual(
            model_spec("planar-7dof").formulation,
            Path("models/formulations/iem24_murphy_real_planar_7dof.xml"),
        )
        self.assertEqual(
            model_spec("14dof-simple").formulation,
            Path("models/formulations/iem24_murphy_real_14dof_simple.xml"),
        )
        with self.assertRaises(ValueError):
            model_spec("planar-14dof")

    def test_planar_fake_plant_reproduces_the_recorded_header_and_summary(self):
        baseline = json.loads(BASELINE_FIXTURE.read_text())
        root, calls, code = run_fake_child("planar-7dof", FakePlanarPlant)
        self.assertEqual(code, 0)
        header = (root / "trace.csv").read_text().splitlines()[0].split(",")
        self.assertEqual(header, baseline["trace_header"])
        self.assertEqual(tuple(header), trace_header(FakePlanarPlant()))
        self.assertEqual(
            header, ["time"] + list(PlanarForwardPlant.state_names),
        )
        summary = json.loads((root / "summary.json").read_text())
        for key in baseline["stable_summary"]:
            self.assertIn(key, summary)
        stable = {key: value for key, value in summary.items()
                  if key not in VOLATILE_SUMMARY_KEYS}
        self.assertEqual(stable, baseline["stable_summary"])
        # Task 8 adds the model identity and the requested/elapsed wall budget.
        self.assertEqual(summary["model"], "planar-7dof")
        self.assertEqual(summary["requested_wall_seconds"], 300.)
        self.assertEqual(summary["solver_wall_budget_seconds"], 295.)
        self.assertEqual(summary["solver_elapsed_seconds"], 1.25)
        self.assertGreater(summary["elapsed_wall_seconds"], 0.)
        self.assertGreater(summary["peak_memory_bytes"], 0.)
        # The parent watchdog keeps the full requested timeout; the solver gets
        # the shorter budget after setup.
        self.assertEqual(calls[0]["maximum_wall_seconds"], 300.)
        # The solver budget is the recorded budget minus the recorded setup time,
        # both of which the runner publishes; no absolute setup-time assumption.
        self.assertAlmostEqual(calls[0]["wall_seconds"],
                               summary["solver_wall_budget_seconds"] - summary["setup_seconds"],
                               delta=0.5)
        self.assertGreater(calls[0]["wall_seconds"], 0.)

    def test_14dof_header_has_thirty_nine_columns(self):
        header = trace_header(FakeVerticalPlant())
        self.assertEqual(len(header), 39)
        self.assertEqual(header[0], "time")
        self.assertEqual(len(VerticalPlantLayout.state_names), 38)
        self.assertEqual(header[1:], VerticalPlantLayout.state_names)

    def test_14dof_fake_plant_records_vertical_evidence(self):
        root, calls, code = run_fake_child("14dof-simple", FakeVerticalPlant)
        self.assertEqual(code, 0)
        header = (root / "trace.csv").read_text().splitlines()[0].split(",")
        self.assertEqual(len(header), 39)
        summary = json.loads((root / "summary.json").read_text())
        self.assertEqual(summary["state_size"], 38)
        self.assertEqual(summary["model"], "14dof-simple")
        self.assertEqual(summary["requested_wall_seconds"], 300.)
        # A 300 s parent timeout leaves 2 % (6 s), above the 5 s floor.
        self.assertEqual(summary["solver_wall_budget_seconds"], 294.)
        self.assertEqual(calls[0]["maximum_wall_seconds"], 300.)
        self.assertAlmostEqual(calls[0]["wall_seconds"],
                               summary["solver_wall_budget_seconds"] - summary["setup_seconds"],
                               delta=0.5)
        self.assertEqual(summary["wheel_inertia"], 0.2064512)
        # contact_valid comes from the replay diagnostics and the travel bound.
        self.assertTrue(summary["contact_valid"])
        self.assertTrue(summary["jounce_travel_valid"])
        self.assertEqual(summary["out_of_range_evaluations"], 0)
        self.assertEqual(summary["jounce_travel_limits_m"], [-0.0318, 0.0318])
        # The gate reads the accepted states (jounce_initial); the plant's
        # all-evaluation extremes stay separate diagnostics.
        self.assertEqual(summary["jounce_accepted_minimum_m"], 0.002)
        self.assertEqual(summary["jounce_accepted_maximum_m"], 0.002)
        self.assertEqual(summary["jounce_evaluation_minimum_m"], -0.004)
        self.assertEqual(summary["jounce_evaluation_maximum_m"], 0.012)
        self.assertIn("accepted states only", summary["jounce_gate_scope"])
        self.assertEqual(summary["maximum_damper_velocity"], 0.5)
        self.assertEqual(summary["minimum_contact_load_n"], 812.5)
        # Vertical runs record their narrow dense state-domain result; this is
        # distinct from planar load-closure and is not full between-step validation.
        self.assertFalse(summary["dense_polynomial_domain_certified"])
        self.assertIsInstance(summary["dense_polynomial_domain"], dict)
        self.assertFalse(summary["between_step_validation_complete"])
        self.assertIsNone(summary["contact_nonnegativity_conditions_hold"])
        self.assertIsNone(summary["contact_nonnegativity_conditions"])

    def test_14dof_model_provenance_records_the_tire_layer(self):
        root, _, _ = run_fake_child("14dof-simple", FakeVerticalPlant)
        summary = json.loads((root / "summary.json").read_text())
        provenance = summary["model_provenance"]
        transient = Path("models/tires/iem24_murphy_real_unitire_transient.xml")
        steady = Path("models/tires/iem24_murphy_real_unitire.xml")
        self.assertEqual(provenance["transient_tire_xml"], str(transient))
        self.assertEqual(provenance["steady_tire_xml"], str(steady))
        # The DOI is read from the hashed XML, not repeated as a literal.
        self.assertEqual(provenance["transient_tire_doi"], "10.3390/machines10070527")
        self.assertIn("10.3390/machines10070527", transient.read_text())
        self.assertEqual(provenance["transient_tire_xml_sha256"],
                         summary["source_hashes"][str(transient)])
        self.assertEqual(provenance["steady_tire_xml_sha256"],
                         summary["source_hashes"][str(steady)])
        self.assertEqual(provenance["transient_tire_parameters"],
                         {"longitudinal_length_scale": 1.0, "lateral_length_scale": 1.0,
                          "trail_pole_ratio": 1.0, "trail_lead_ratio": 0.25})
        self.assertIn("PROPOSED", provenance["note"])
        # Planar attempts record no vertical provenance.
        planar_root, _, _ = run_fake_child("planar-7dof", FakePlanarPlant)
        planar = json.loads((planar_root / "summary.json").read_text())
        self.assertNotIn("model_provenance", planar)

    def test_14dof_early_failure_still_writes_a_strict_summary(self):
        # A 14DoF child that fails before its first successful plant evaluation
        # leaves the jounce diagnostics at ±inf (the plant's initial sentinels);
        # the summary must still be written as strict JSON with those fields
        # mapped to None.
        never_evaluated = vertical_fake(maximum_jounce=-float("inf"),
                                        minimum_jounce=float("inf"))
        root, _, code = run_fake_child(
            "14dof-simple", never_evaluated,
            extra_args=("--warm-start", "/tmp/task8-does-not-exist"))
        self.assertEqual(code, 2)
        summary = json.loads((root / "summary.json").read_text())
        self.assertEqual(summary["status"], "Rejected")
        self.assertIn("FileNotFoundError", summary["error"])
        self.assertIsNone(summary["jounce_evaluation_minimum_m"])
        self.assertIsNone(summary["jounce_evaluation_maximum_m"])
        self.assertEqual(summary["out_of_range_evaluations"], 0)
        self.assertNotIn("contact_valid", summary)
        # The written text is strict JSON, and the failure left diagnostics.
        text = (root / "summary.json").read_text()
        self.assertNotIn("Infinity", text)
        json.loads(text, parse_constant=lambda value: self.fail(f"non-strict constant {value}"))
        self.assertTrue((root / "error.txt").exists())

    def test_14dof_contact_valid_rejects_travel_or_table_excursions(self):
        root, _, code = run_fake_child("14dof-simple",
                                       vertical_fake(jounce_initial=0.05))
        self.assertEqual(code, 0)
        summary = json.loads((root / "summary.json").read_text())
        self.assertFalse(summary["jounce_travel_valid"])
        self.assertFalse(summary["contact_valid"])
        self.assertEqual(summary["jounce_accepted_maximum_m"], 0.05)
        root, _, _ = run_fake_child("14dof-simple", vertical_fake(out_of_range_evaluations=2))
        summary = json.loads((root / "summary.json").read_text())
        self.assertFalse(summary["contact_valid"])
        root, _, _ = run_fake_child("14dof-simple", vertical_fake(minimum_contact_load=-1.0))
        summary = json.loads((root / "summary.json").read_text())
        self.assertFalse(summary["contact_valid"])

    def test_14dof_trial_jounce_excursion_is_a_diagnostic_not_a_gate_failure(self):
        # The vertical plant lets a Newton trial reach limits ± 10 mm, and its
        # minimum_jounce/maximum_jounce count every evaluation. A legal trial
        # excursion must not fail the travel gate while it is still reported.
        root, _, code = run_fake_child(
            "14dof-simple",
            vertical_fake(minimum_jounce=-0.0418, maximum_jounce=0.0418))
        self.assertEqual(code, 0)
        summary = json.loads((root / "summary.json").read_text())
        self.assertTrue(summary["jounce_travel_valid"])
        self.assertTrue(summary["contact_valid"])
        self.assertEqual(summary["jounce_accepted_minimum_m"], 0.002)
        self.assertEqual(summary["jounce_accepted_maximum_m"], 0.002)
        self.assertEqual(summary["jounce_evaluation_minimum_m"], -0.0418)
        self.assertEqual(summary["jounce_evaluation_maximum_m"], 0.0418)

    def test_14dof_travel_gate_covers_the_dense_interval_bounds(self):
        # A midpoint excursion inside an accepted interval must fail the gate
        # even though every accepted node is inside the travel.
        bounds = np.zeros((2, 38, 2))
        bounds[:, 16:20, 0], bounds[:, 16:20, 1] = -0.002, 0.002
        bounds[1, 17, 1] = 0.04
        root, _, _ = run_fake_child("14dof-simple", FakeVerticalPlant, dense_bounds=bounds)
        summary = json.loads((root / "summary.json").read_text())
        self.assertEqual(summary["jounce_accepted_maximum_m"], 0.04)
        self.assertFalse(summary["jounce_travel_valid"])
        self.assertFalse(summary["contact_valid"])
        self.assertIn("midpoint", summary["jounce_gate_scope"])

    def test_14dof_boundary_jounce_at_the_travel_limit_passes(self):
        root, _, _ = run_fake_child("14dof-simple",
                                    vertical_fake(jounce_initial=0.0318))
        summary = json.loads((root / "summary.json").read_text())
        self.assertEqual(summary["jounce_accepted_maximum_m"], 0.0318)
        self.assertTrue(summary["jounce_travel_valid"])
        self.assertTrue(summary["contact_valid"])

    def test_14dof_duration_limit_is_not_contact_valid(self):
        root, _, code = run_fake_child("14dof-simple", FakeVerticalPlant,
                                       lap_completed=False)
        self.assertEqual(code, 0)
        summary = json.loads((root / "summary.json").read_text())
        self.assertEqual(summary["status"], "Duration_Limit")
        self.assertFalse(summary["lap_completed"])
        self.assertFalse(summary["contact_valid"])
        # The underlying evidence is still recorded.
        self.assertTrue(summary["jounce_travel_valid"])
        self.assertEqual(summary["out_of_range_evaluations"], 0)
        self.assertEqual(summary["minimum_contact_load_n"], 812.5)

    def test_source_hashes_cover_each_model(self):
        sources = {Path(path).resolve() for path in source_files("14dof-simple", DEFAULT_TRACK)}
        for required in REQUIRED_14DOF_SOURCES:
            self.assertIn(Path(required).resolve(), sources)
        controller = Path("src/simplified_vehicle_models/drivetrain/controller.py").resolve()
        self.assertIn(controller, sources)
        planar = {Path(path).resolve() for path in source_files("planar-7dof", DEFAULT_TRACK)}
        self.assertIn(controller, planar)
        # The shared powertrain closure moved in Task 7, so the planar rung also
        # hashes it; no vertical-only source may enter the planar list.
        self.assertIn(Path("src/simplified_vehicle_models/solvers/forward_powertrain.py").resolve(),
                      planar)
        self.assertIn(Path("src/simplified_vehicle_models/solvers/planar_forward.py").resolve(),
                      planar)
        for vertical_only in ("src/simplified_vehicle_models/solvers/vertical_forward.py",
                              "src/simplified_vehicle_models/solvers/vertical_equilibrium.py",
                              "src/simplified_vehicle_models/tires/unitire_transient.py"):
            self.assertNotIn(Path(vertical_only).resolve(), planar)
        # Two sources may not share a basename: the snapshot writes by basename.
        names = [Path(path).name for path in source_files("14dof-simple", DEFAULT_TRACK)]
        self.assertEqual(len(set(names)), len(names))

    def test_recorded_14dof_hashes_name_the_component_files(self):
        root, _, _ = run_fake_child("14dof-simple", FakeVerticalPlant)
        summary = json.loads((root / "summary.json").read_text())
        hashed = {Path(path).resolve() for path in summary["source_hashes"]}
        for required in ("models/dampers/multimatic_dssv_vc01_front.xml",
                         "models/dampers/multimatic_dssv_vc01_rear.xml",
                         "models/suspension-kinematics/iem24_murphy_real_suspension_kinematics.xml",
                         "models/springs/iem24_murphy_real_suspension_springs.xml",
                         "models/anti-roll-bars/iem24_murphy_real_suspension_arb.xml",
                         "models/data/multimatic_dssv_vc01.npz",
                         "models/data/iem24_murphy_real_corners.npz",
                         "models/suspensions/iem24_murphy_real_suspension.xml",
                         "models/tires/iem24_murphy_real_unitire_transient.xml"):
            self.assertIn(Path(required).resolve(), hashed, required)
        self.assertTrue((root / "source_snapshot").is_dir())
        snapshot = {path.name for path in (root / "source_snapshot").iterdir()}
        self.assertIn("multimatic_dssv_vc01.npz", snapshot)
        self.assertIn("iem24_murphy_real_corners.npz", snapshot)

    def test_14dof_component_roles_resolve_to_distinct_components(self):
        inputs = model_inputs("14dof-simple", DEFAULT_TRACK)
        self.assertEqual(inputs["tire_xml"],
                         Path("models/tires/iem24_murphy_real_unitire.xml"))
        self.assertEqual(inputs["tire_transient_xml"],
                         Path("models/tires/iem24_murphy_real_unitire_transient.xml"))
        self.assertEqual(inputs["suspension_xml"],
                         Path("models/suspensions/iem24_murphy_real_suspension.xml"))
        # The suspension assembly nests five children; each is recorded under
        # "suspension_<child>_xml" with its table under "..._data".
        self.assertEqual(inputs["suspension_kinematics_xml"], Path(
            "models/suspension-kinematics/iem24_murphy_real_suspension_kinematics.xml"))
        self.assertEqual(inputs["suspension_spring_xml"], Path(
            "models/springs/iem24_murphy_real_suspension_springs.xml"))
        self.assertEqual(inputs["suspension_arb_xml"], Path(
            "models/anti-roll-bars/iem24_murphy_real_suspension_arb.xml"))
        self.assertEqual(inputs["suspension_damper_front_xml"],
                         Path("models/dampers/multimatic_dssv_vc01_front.xml"))
        self.assertEqual(inputs["suspension_damper_rear_xml"],
                         Path("models/dampers/multimatic_dssv_vc01_rear.xml"))
        self.assertEqual(inputs["suspension_damper_front_data"],
                         Path("models/data/multimatic_dssv_vc01.npz"))
        self.assertEqual(inputs["suspension_kinematics_data"],
                         Path("models/data/iem24_murphy_real_corners.npz"))
        self.assertEqual(inputs["suspension_spring_data"],
                         Path("models/data/iem24_murphy_real_corners.npz"))
        for path in inputs.values():
            self.assertTrue(Path(path).is_file(), path)
        # The planar model resolves only its own roles.
        planar = model_inputs("planar-7dof", DEFAULT_TRACK)
        self.assertEqual(planar["tire_xml"], Path("models/tires/iem24_murphy_real_unitire.xml"))
        for key in ("tire_transient_xml", "suspension_xml", "suspension_kinematics_xml",
                    "suspension_damper_front_xml", "suspension_damper_rear_xml"):
            self.assertNotIn(key, planar)
        compiled = compile_formulation(str(model_spec("14dof-simple").formulation))
        roles = {record.role for record in compiled.manifest.loaded_components}
        self.assertTrue({"suspension", "tire-transient", "suspension/kinematics",
                         "suspension/spring", "suspension/arb", "suspension/damper-front",
                         "suspension/damper-rear"} <= roles)
        components = {compiled.manifest.capability(capability).component_model_id
                      for capability in (SUSPENSION_FORCES, TRANSIENT_COMBINED_SLIP)}
        self.assertEqual(len(components), 2)
        model_ids = {record.model_id for record in compiled.manifest.loaded_components
                     if record.role.startswith("suspension/")}
        self.assertEqual(len(model_ids), 5)
        self.assertEqual(compiled.manifest.unused_components, ())


class WallBudgetTests(unittest.TestCase):
    """Task 8 wall-time contract: 5 s for 7DoF, 2 % (floor 5 s) for 14DoF."""

    def test_margins_per_model(self):
        self.assertEqual(wall_margin_seconds(300., "planar-7dof"), 5.)
        self.assertEqual(solver_wall_budget(300., "planar-7dof"), 295.)
        self.assertEqual(wall_margin_seconds(900., "14dof-simple"), 18.)
        self.assertEqual(solver_wall_budget(900., "14dof-simple"), 882.)
        # A 900 s parent watchdog permits at least an 870 s solver budget.
        self.assertGreaterEqual(solver_wall_budget(900., "14dof-simple"), 870.)
        # The 2 % margin never drops below its 5 s floor.
        self.assertEqual(wall_margin_seconds(100., "14dof-simple"), 5.)
        self.assertEqual(solver_wall_budget(100., "14dof-simple"), 95.)
        # 2 % of a 300 s parent timeout is 6 s, so the floor is not binding.
        self.assertEqual(wall_margin_seconds(300., "14dof-simple"), 6.)
        self.assertEqual(solver_wall_budget(300., "14dof-simple"), 294.)
        with self.assertRaises(ValueError):
            solver_wall_budget(900., "planar-14dof")

    def test_14dof_900_second_parent_watchdog_is_used_and_recorded(self):
        # Exercise the parent watchdog itself: it must wait for the requested
        # 900 s, keep a shorter solver budget and record both.
        with tempfile.TemporaryDirectory() as temporary:
            with patch("tools.run.run_iem24_forward_driver.subprocess.run",
                       side_effect=subprocess.TimeoutExpired("child", 900.)) as run:
                with contextlib.redirect_stdout(io.StringIO()):
                    result = main(["--model", "14dof-simple", "--timeout-seconds", "900",
                                   "--output-root", temporary])
            self.assertEqual(result, 124)
            self.assertEqual(run.call_args.kwargs["timeout"], 900.)
            attempt, = Path(temporary).iterdir()
            parent = json.loads((attempt / "parent.json").read_text())
            self.assertEqual(parent["status"], "Timed_Out")
            self.assertEqual(parent["model"], "14dof-simple")
            self.assertEqual(parent["requested_wall_seconds"], 900.)
            self.assertEqual(parent["timeout_seconds"], 900.)
            self.assertEqual(parent["solver_wall_budget_seconds"], 882.)
            self.assertGreaterEqual(parent["solver_wall_budget_seconds"], 870.)
            self.assertTrue((attempt / "run.log").exists())

    def test_planar_rejects_long_timeouts_and_coarse_spacing(self):
        for arguments in (["--timeout-seconds", "301"], ["--timeout-seconds", "900"],
                          ["--spacing", ".51"], ["--model", "planar-14dof"],
                          ["--model", "14dof-simple", "--timeout-seconds", "nan"]):
            with self.subTest(arguments=arguments):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    parse_args(arguments)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--model", "14dof-simple", "--spacing", ".51"])
        # Only the 14DoF model accepts a longer parent timeout.
        args = parse_args(["--model", "14dof-simple", "--timeout-seconds", "900"])
        self.assertEqual(args.timeout_seconds, 900.)
        self.assertEqual(args.spacing, .5)

    def test_solve_forward_wall_ceiling(self):
        plan = SpeedPlan(np.array([0., 1.]), np.array([1., 1.]), np.zeros(2))
        from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope
        from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy
        envelope = BrakingEnvelope(np.array([0., .5, 1.]), np.zeros(3), np.ones(3), np.ones(3),
            np.full(2, 5.), np.zeros(3, dtype=int), True, 8., {"fixture": True})
        driver = PreviewDriver(plan, 1.5, .5, 2., 1000., envelope_policy=EnvelopePolicy(envelope))

        class Plant:
            state_size = 1

            def observe(self, state):
                return DriverObservation(state[0], 0., 0., 1., 0., 0.)

            def evaluate(self, time, state, command):
                return RateSystem(np.array([1.]))

        initial = np.array([0.])
        # The 14DoF ceiling permits a longer solver budget than the 7DoF default.
        result = solve_forward(Plant(), driver, initial, duration=.01,
                               wall_seconds=870., maximum_wall_seconds=900.)
        self.assertFalse(result.lap_completed)
        # The default ceiling still refuses the relaxation of the 7DoF limit.
        with self.assertRaises(ValueError):
            solve_forward(Plant(), driver, initial, duration=.01, wall_seconds=301.)
        with self.assertRaises(ValueError):
            solve_forward(Plant(), driver, initial, duration=.01, wall_seconds=870.)
        with self.assertRaises(ValueError):
            solve_forward(Plant(), driver, initial, duration=.01, wall_seconds=870.,
                          maximum_wall_seconds=300.)
        for bad in (0., -1., float("nan"), float("inf")):
            with self.subTest(maximum_wall_seconds=bad):
                with self.assertRaises(ValueError):
                    solve_forward(Plant(), driver, initial, duration=.01, wall_seconds=.01,
                                  maximum_wall_seconds=bad)

    def test_peak_memory_is_reported_in_bytes(self):
        value = peak_memory_bytes()
        self.assertIsNotNone(value)
        self.assertGreater(value, 0.)
        self.assertLess(value, 1.e13)


class PlantMetadataValidationTests(unittest.TestCase):
    def test_planar_indices_are_the_previous_literals(self):
        # The runner used to hard-code these columns; the names must resolve to
        # exactly the same indices so planar traces keep their meaning.
        plant = PlanarForwardPlant
        for name, index in (("s", 0), ("n", 1), ("v_x", 3), ("v_y", 4),
                            ("yaw_rate", 5), ("soc", 10), ("steering", 11)):
            self.assertEqual(state_index(plant, name), index, name)
        self.assertEqual(wheel_speed_indices(plant), (6, 7, 8, 9))
        self.assertEqual(plant.state_size, 12)
        self.assertEqual(plant.state_names[2], "heading_error")
        np.testing.assert_array_equal(validated_periodic_tolerances(plant),
                                      PLANAR_PERIODIC_TOLERANCES)
        # The vertical plant publishes the same path columns and longer tail.
        vertical = VerticalPlantLayout
        for name, index in (("s", 0), ("n", 1), ("soc", 24), ("steering", 25)):
            self.assertEqual(state_index(vertical, name), index, name)
        self.assertEqual(wheel_speed_indices(vertical), (12, 13, 14, 15))

    def test_trace_header_rejects_unknown_state_names(self):
        with self.assertRaises(ValueError):
            state_index(FakePlanarPlant(), "omega_fx")
        with self.assertRaises(ValueError):
            wheel_speed_indices(type("NoWheels", (), {"state_names": ("s", "n")}))

    def test_periodic_tolerance_vector_must_match_the_plant_layout(self):
        class Short:
            state_size = 4
            periodic_indices = (1, 2, 3)
            periodic_tolerances = np.array([.1, .1])

        class Long:
            state_size = 4
            periodic_indices = (1, 2)
            periodic_tolerances = np.array([.1, .1, .1])

        class NonFinite:
            state_size = 4
            periodic_indices = (1, 2)
            periodic_tolerances = np.array([.1, np.inf])

        class Negative:
            state_size = 4
            periodic_indices = (1, 2)
            periodic_tolerances = np.array([.1, -.1])

        for plant in (Short(), Long(), NonFinite(), Negative()):
            with self.subTest(plant=type(plant).__name__):
                with self.assertRaises(ValueError):
                    validated_periodic_tolerances(plant)
        # The real plants publish the frozen planar index list and a full-coverage
        # vertical list; a shift in either must fail this test.
        self.assertEqual(PlanarForwardPlant.periodic_indices, (1, 2, 3, 4, 5, 6, 7, 8, 9, 11))
        self.assertEqual(VerticalPlantLayout.periodic_indices,
                         tuple(index for index in range(38) if index not in (0, 24)))
        for plant in (PlanarForwardPlant, VerticalPlantLayout):
            self.assertEqual(len(validated_periodic_tolerances(plant)),
                             len(plant.periodic_indices))
        np.testing.assert_array_equal(validated_periodic_tolerances(FakePlanarPlant()),
                                      PLANAR_PERIODIC_TOLERANCES)

    def test_vertical_periodic_tolerances_follow_the_plan_tiers_by_name(self):
        """Every one of the 36 periodic states has the plan's tolerance for its tier.

        The plan's Task 9 row fixes three vertical magnitudes (0.5 mm, 0.005 rad,
        0.01 m/s) and three tire-state magnitudes (0.002, 0.2 mm, 0.002 rad). This
        pins each *name* to its tier, so a tolerance cannot drift between states
        of the same kind, and all 36 names must be classified.
        """
        plant = VerticalPlantLayout
        names = tuple(plant.state_names)
        indices = tuple(plant.periodic_indices)
        tolerances = validated_periodic_tolerances(plant)
        self.assertEqual(len(indices), 36)
        expected = {}
        for name in names:
            if name in ("s", "soc"):
                continue  # not periodic: distance resets, charge is consumed
            if name.startswith("jounce_rate"):
                expected[name] = 1.0e-2          # m/s
            elif name in ("n", "heave") or name.startswith("jounce_"):
                expected[name] = 5.0e-4          # 0.5 mm
            elif name.startswith("kappa_eff"):
                expected[name] = 2.0e-3
            elif name.startswith("lateral_deformation"):
                expected[name] = 2.0e-4          # 0.2 mm
            elif name.startswith("trail_angle"):
                expected[name] = 2.0e-3          # rad
            elif name in ("heading_error", "roll", "pitch", "steering",
                          "roll_rate", "pitch_rate", "yaw_rate"):
                expected[name] = 5.0e-3          # rad or rad/s
            elif name in ("v_x", "v_y", "v_z") or name.startswith("wheel_speed"):
                expected[name] = 1.0e-2          # m/s or rad/s
            else:
                self.fail(f"unclassified periodic state {name!r}")
        self.assertEqual(len(expected), 36)
        self.assertEqual(set(expected), {names[index] for index in indices})
        np.testing.assert_array_equal(
            tolerances, np.array([expected[names[index]] for index in indices]))
        # The tire states the plan names explicitly, in the layout's own order.
        for wheel in ("fl", "fr", "rl", "rr"):
            self.assertEqual(expected[f"kappa_eff_{wheel}"], 2.0e-3)
            self.assertEqual(expected[f"lateral_deformation_{wheel}"], 2.0e-4)
            self.assertEqual(expected[f"trail_angle_{wheel}"], 2.0e-3)

    def test_periodic_indices_must_lie_inside_the_state_layout(self):
        class OutOfRange:
            state_size = 4
            periodic_indices = (1, 99)
            periodic_tolerances = np.array([.1, .1])

        class Negative:
            state_size = 4
            periodic_indices = (-1,)
            periodic_tolerances = np.array([.1])

        class NonInteger:
            state_size = 4
            periodic_indices = (1.5,)
            periodic_tolerances = np.array([.1])

        for plant in (OutOfRange(), Negative(), NonInteger()):
            with self.subTest(plant=type(plant).__name__):
                with self.assertRaisesRegex(ValueError, "inside the state layout"):
                    validated_periodic_tolerances(plant)

    def test_returned_tolerances_are_a_copy_of_the_plant_vector(self):
        class Plant:
            state_size = 4
            state_names = ("s", "n", "v_x", "soc")
            periodic_indices = (1, 2)
            periodic_tolerances = np.array([.1, .2])

        returned = validated_periodic_tolerances(Plant)
        returned[0] = 9.
        self.assertEqual(float(Plant.periodic_tolerances[0]), .1)

    def test_trace_header_requires_unique_names_matching_the_state_size(self):
        class Good:
            state_size = 3
            state_names = ("s", "v_x", "soc")

        class Repeated:
            state_size = 3
            state_names = ("s", "s", "soc")

        class WrongLength:
            state_size = 3
            state_names = ("s", "soc")

        self.assertEqual(trace_header(Good()), ("time", "s", "v_x", "soc"))
        for plant in (Repeated(), WrongLength()):
            with self.assertRaises(ValueError):
                trace_header(plant)


class JounceTravelTests(unittest.TestCase):
    """The travel interval is exactly the intersection of the axle limits."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(
            str(model_spec("14dof-simple").formulation)).model

    def test_travel_limits_are_the_intersection_of_the_axle_limits(self):
        front = self.vehicle.suspension.kinematics.front.jounce_limits
        rear = self.vehicle.suspension.kinematics.rear.jounce_limits
        lower, upper = jounce_travel_limits(self.vehicle)
        self.assertEqual(lower, max(front[0], rear[0]))
        self.assertEqual(upper, min(front[1], rear[1]))
        self.assertEqual([lower, upper], [-0.0318, 0.0318])

    def test_travel_limits_follow_differing_axle_limits(self):
        # The helper reads only the axle limits, so a duck-typed vehicle with
        # different front and rear travel pins the intersection arithmetic
        # (the compiled assembly validates equal axle travel at compile time).
        def kinematics(front, rear):
            return SimpleNamespace(jounce_limits=np.array([front, front, rear, rear]))

        vehicle = SimpleNamespace(
            suspension=SimpleNamespace(kinematics=kinematics((-0.0318, 0.02), (-0.02, 0.05))))
        lower, upper = jounce_travel_limits(vehicle)
        self.assertEqual([lower, upper], [-0.02, 0.02])

    def test_travel_limits_use_both_axle_bounds(self):
        def kinematics(front, rear):
            return SimpleNamespace(jounce_limits=np.array([front, front, rear, rear]))

        vehicle = SimpleNamespace(
            suspension=SimpleNamespace(kinematics=kinematics((-0.05, 0.0318), (-0.0318, 0.05))))
        lower, upper = jounce_travel_limits(vehicle)
        self.assertEqual([lower, upper], [-0.0318, 0.0318])


class TestLongitudinalPolicyCLI(unittest.TestCase):
    def test_removed_cli_alternatives_are_rejected(self):
        for argv in (["--planner", "live"], ["--longitudinal-policy", "schedule"]):
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                parse_args(argv)

    def test_envelope_policy_is_default(self):
        self.assertEqual(parse_args([]).longitudinal_policy, "braking-envelope")


if __name__ == "__main__":
    unittest.main()
