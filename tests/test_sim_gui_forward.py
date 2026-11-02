import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tools.sim_gui.settings import RunSettings, solution_model_for, validate


class ForwardSettingsTests(unittest.TestCase):
    def test_defaults_roundtrip_and_legacy_default_preserved(self):
        self.assertEqual(RunSettings().solver_kind, "yaw-aware")
        settings = RunSettings(solver_kind="forward-driver")
        self.assertEqual(RunSettings.from_json(settings.to_json()), settings)
        self.assertEqual(settings.forward_duration, 300.)
        self.assertEqual(settings.forward_wall_seconds, 600.)
        self.assertEqual(solution_model_for(settings, "14dof-simple"), "planar-7dof")
        with self.assertRaisesRegex(ValueError, "point-mass is unsupported"):
            solution_model_for(settings, "point-mass")

    def test_strict_fields_and_ranges(self):
        for key, value in (("forward_deceleration_margin", 1.01), ("forward_spacing", .6),
                           ("forward_wall_seconds", 601.), ("forward_speed_points", True),
                           ("forward_speed_points", 1), ("forward_duration", float("inf")),
                           ("forward_duration", True), ("forward_feedforward_distance", 0)):
            with self.subTest(key=key, value=value):
                settings = RunSettings(solver_kind="forward-driver", **{key: value})
                with self.assertRaises(ValueError):
                    validate(settings, "planar-7dof", Path("unused"))
        with self.assertRaisesRegex(ValueError, "yaw map, mode"):
            validate(RunSettings(solver_kind="forward-driver", yaw_map="legacy"), "planar-7dof", Path("unused"))

    def test_invalid_before_compile(self):
        from tools.sim_gui import execute
        from tools.sim_gui.study import CaseSpec, StudyDefinition
        settings = RunSettings(solver_kind="forward-driver", forward_deceleration_margin=1.1)
        definition = StudyDefinition("bad", "models/formulations/iem24_murphy_real_planar_7dof.xml",
            "models/tracks/2024_michigan_endurance_closed.xml", "planar-7dof", {}, settings,
            "one-at-a-time", ())
        case = CaseSpec("bad", "bad", "planar-7dof", definition.track, {}, {})
        with mock.patch("tools.sim_gui.execute.compile_formulation") as compile_model, \
             mock.patch("tools.sim_gui.variant.write_variant") as write_variant:
            with self.assertRaises(ValueError):
                execute.prepare_case(Path("/tmp/unused"), definition, case)
            compile_model.assert_not_called()
            write_variant.assert_not_called()

    def test_run_case_dispatches_to_forward_adapter(self):
        from tools.sim_gui.run_yaw_case import run_case
        settings = RunSettings(solver_kind="forward-driver")
        with tempfile.TemporaryDirectory() as folder:
            case_dir = Path(folder) / "case"
            case_dir.mkdir()
            attempt = Path(folder) / "attempt"
            payload = {"solver_kind": "forward-driver", "settings": settings.to_json(),
                "model": "planar-7dof", "track": "unused", "formulation": "unused.xml",
                "input_identity": {}}
            (case_dir / "case.json").write_text(json.dumps(payload))
            with mock.patch("tools.sim_gui.forward_execute._run_impl", side_effect=RuntimeError("synthetic child failure")):
                self.assertEqual(run_case(case_dir, attempt), 4)
            failure = json.loads((attempt / "summary.json").read_text())
            self.assertEqual(failure["message"], "synthetic child failure")
            self.assertIn("accepted_policy_prefix_rows", failure)

    def test_identity_rejected_before_compile(self):
        from tools.sim_gui.forward_execute import run
        settings = RunSettings(solver_kind="forward-driver")
        case = {"solver_kind": "forward-driver", "model": "planar-7dof", "formulation": "missing.xml",
                "track": "missing-track.xml", "argv": [], "input_identity": {}}
        with tempfile.TemporaryDirectory() as folder, mock.patch(
                "tools.sim_gui.forward_execute.compile_formulation") as compile_model:
            attempt = Path(folder) / "attempt"
            attempt.mkdir()
            self.assertEqual(run(case, attempt, settings, Path(folder)), 4)
            compile_model.assert_not_called()


class ForwardFinishTests(unittest.TestCase):
    def test_contract_requires_counts_and_real_record(self):
        import numpy as np
        from tools.sim_gui.execute import PreparedCase, finish_case
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            case_dir = root / "study" / "cases" / "case_000"
            attempt = case_dir / "attempt_1"
            attempt.mkdir(parents=True)
            track = Path("models/tracks/2024_michigan_endurance_closed.xml").resolve()
            formulation = (case_dir / "model" / "formulation.xml")
            formulation.parent.mkdir(parents=True)
            formulation.write_text("selected-formulation")
            settings = RunSettings(solver_kind="forward-driver")
            from tools.sim_gui.settings import forward_driver_config
            settings_hash = hashlib.sha256(json.dumps(settings.to_json(), sort_keys=True,
                separators=(",", ":"), allow_nan=False).encode()).hexdigest()
            identity = {"solver_kind": "forward-driver", "source_model": "planar-7dof",
                "solution_model": "planar-7dof", "track": str(track),
                "track_sha256": hashlib.sha256(track.read_bytes()).hexdigest(),
                "formulation": str(formulation.resolve()),
                "formulation_sha256": hashlib.sha256(formulation.read_bytes()).hexdigest(),
                "settings": settings.to_json(), "settings_sha256": settings_hash,
                "driver_config": forward_driver_config(settings), "yaw_map": None}
            (case_dir / "case.json").write_text(json.dumps({"input_identity": identity, "settings": settings.to_json(),
                "driver_config": identity["driver_config"]}))
            from simplified_vehicle_models.results.run_record import RunRecorder, sha256_of
            source_base = root / "study"
            source_base.mkdir(parents=True, exist_ok=True)
            source_a, source_b, source_c = (source_base / name for name in ("source-a.xml", "source-b.xml", "source-c.xml"))
            for index, source in enumerate((source_a, source_b, source_c)):
                source.write_text(f"source-{index}")
            source_hashes = {source.name: sha256_of(source) for source in (source_a, source_b, source_c)}
            record = attempt / "run_record.npz"
            recorder = RunRecorder(record, state_names=("s",), state_units=("m",),
                control_names=(), channel_names=())
            recorder.append(0., 0, [0.])
            recorder.append(1., 1, [1.])
            recorder.append(2., 2, [2.])
            recorder.close({"sources": source_hashes})
            digest = hashlib.sha256(record.read_bytes()).hexdigest()
            envelope = attempt / "envelope_arrays.npz"
            np.savez(envelope, distance=np.arange(3.), curvature=np.zeros(3), corner_speed=np.ones(3),
                     entry_speed=np.ones(3), deceleration=np.ones(3), binding_index=np.zeros(3))
            envelope_digest = hashlib.sha256(envelope.read_bytes()).hexdigest()
            policy = attempt / "accepted_policy.jsonl"
            policy.write_text("{}\n{}\n{}\n")
            policy_digest = hashlib.sha256(policy.read_bytes()).hexdigest()
            inputs = {**identity, "formulation": str(formulation.resolve()), "track": str(track),
                "formulation_sha256": identity["formulation_sha256"], "track_sha256": identity["track_sha256"],
                "run_record_sha256": digest, "completed_forward_result": True, "lap_completed": True,
                "settings": settings.to_json(), "driver_config": identity["driver_config"],
                "settings_sha256": settings_hash,
                "midpoint_sample_validation_complete": True, "accepted_states_validated": 3,
                "dense_samples_validated": 2, "accepted_policy_file": "accepted_policy.jsonl",
                "accepted_policy_rows": 3, "accepted_policy_sha256": policy_digest,
                "source_hashes": source_hashes}
            (attempt / "inputs.json").write_text(json.dumps(inputs))
            summary = {"status": "completed", "lap_completed": True, "midpoint_sample_validation_complete": True,
                "accepted_steps": 2, "accepted_states_validated": 3, "midpoint_samples": 2,
                "accepted_policy_rows": 3, "forward_driver_config": identity["driver_config"],
                "dense_samples_validated": 2, "validation_counts": {"accepted_states_validated": 3, "dense_probes_validated": 2},
                "settings_sha256": settings_hash, "run_record_sha256": digest,
                "sampled_tracking_pass": False, "envelope_arrays_sha256": envelope_digest,
                "accepted_policy_sha256": policy_digest, "accepted_policy_rows": 3,
                "source_hashes": inputs["source_hashes"]}
            (attempt / "summary.json").write_text(json.dumps(summary))
            prepared = PreparedCase(case_dir, formulation, case_dir / "model/vehicle.xml", [],
                                    model="planar-7dof", timeout_seconds=660, solver_kind="forward-driver")
            args = dict(return_code=0, timed_out=False, cancelled=False, wall_seconds=1, command=[])
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "completed")
            self.assertFalse(summary["sampled_tracking_pass"], "tracking evidence is not the completion gate")
            source_b.write_text("mutated")
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "failed")
            source_b.write_text("source-1")
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "completed")
            inputs["accepted_states_validated"] = True
            (attempt / "inputs.json").write_text(json.dumps(inputs))
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "failed")
            inputs["accepted_states_validated"] = 3
            (attempt / "inputs.json").write_text(json.dumps(inputs))
            summary["midpoint_samples"] = None
            (attempt / "summary.json").write_text(json.dumps(summary))
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "failed")

    def test_synthetic_forward_solver_keeps_requested_applied_stages(self):
        import numpy as np
        from simplified_vehicle_models.solvers.forward_driver import (DriverCommand, DriverObservation,
            RateSystem, SpeedPlan, solve_forward)
        from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope
        from simplified_vehicle_models.solvers.envelope_driver import EnvelopeDriverSettings, EnvelopePolicy
        from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
        from tools.sim_gui import forward_execute
        plan = SpeedPlan(np.array([0., 100.]), np.array([5., 5.]), np.zeros(2), is_closed=False)
        envelope = BrakingEnvelope(np.array([0., 100.]), np.zeros(2), np.array([60., 60.]),
            np.array([60., 60.]), np.array([4.]), np.array([0, 0]), False, 60.,
            {"method": "synthetic", "segment_speed_domain_max": (60.,)})
        class Plant:
            state_size = 1
            def observe(self, state):
                return DriverObservation(float(state[0]), 0., 0., 5., 0., 0.)
            def evaluate(self, time, state, command):
                return RateSystem(np.array([max(command.throttle, 0.) - command.brake_force * .001]))
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
        driver = PreviewDriver(plan, 1., .5, 2., 1000., envelope_policy=EnvelopePolicy(envelope,
            EnvelopeDriverSettings(feedforward_distance_m=2.)), pedal_travel_time=.02)
        samples = []
        result = solve_forward(Plant(), driver, np.array([0.]), duration=.04, wall_seconds=2.,
            maximum_wall_seconds=2., maximum_step=.01, initial_pedal=0., step_sink=samples.append)
        self.assertTrue(result.midpoint_sample_validation_complete)
        self.assertTrue(samples)
        sample = samples[-1]
        self.assertIsNotNone(sample.requested_command)
        self.assertIsNotNone(sample.mapped_command)
        self.assertIsNotNone(sample.policy_diagnostics)
        self.assertIsNotNone(sample.pedal)


class ForwardWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PyQt6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_visibility_and_roundtrip(self):
        from tools.sim_gui.qt.settings_tab import SettingsTab
        widget = SettingsTab()
        widget.show()
        widget.solver_kind.setCurrentIndex(widget.solver_kind.findData("forward-driver"))
        self.app.processEvents()
        self.assertTrue(widget.forward_note.isVisible())
        self.assertTrue(widget.forward_duration.isVisible())
        self.assertFalse(widget.yaw_map.isVisible())
        self.assertTrue(widget.yaw_map.isHidden())
        def label_for(control):
            for row in range(widget._form.rowCount()):
                field = widget._form.itemAt(row, widget._form.ItemRole.FieldRole)
                if field and field.widget() is control:
                    return widget._form.itemAt(row, widget._form.ItemRole.LabelRole).widget()
            self.fail("control has no form label")
        self.assertFalse(label_for(widget.forward_duration).isHidden())
        self.assertTrue(widget.map_label.isHidden())
        widget.forward_feedforward_distance.setValue(2.25)
        settings = widget.settings()
        self.assertEqual(settings.solver_kind, "forward-driver")
        self.assertEqual(RunSettings.from_json(settings.to_json()), settings)
        widget.solver_kind.setCurrentIndex(widget.solver_kind.findData("yaw-aware"))
        self.app.processEvents()
        self.assertFalse(widget.forward_note.isVisible())
        self.assertTrue(widget.forward_duration.isHidden())
        self.assertFalse(widget.spacing.isHidden())
        self.assertFalse(widget.yaw_map.isHidden())
        self.assertTrue(label_for(widget.forward_duration).isHidden())
        widget.solver_kind.setCurrentIndex(widget.solver_kind.findData("point-mass"))
        self.app.processEvents()
        self.assertFalse(widget.point_mass_duration.isHidden())
        self.assertTrue(widget.spacing.isHidden())
        widget.solver_kind.setCurrentIndex(widget.solver_kind.findData("forward-driver"))
        self.app.processEvents()
        self.assertEqual(widget.forward_feedforward_distance.value(), 2.25)
        self.assertFalse(widget.forward_duration.isHidden())
        self.assertTrue(widget.yaw_map.isHidden())


if __name__ == "__main__":
    unittest.main()
