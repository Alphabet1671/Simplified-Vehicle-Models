import tempfile
import hashlib
import json
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from tools.sim_gui.yaw_map import MapBuildRequest, accept_yaw_map, inspect_yaw_map, map_build_command
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.formulations.yaw_map_artifact import build_identity, save_yaw_map
from simplified_vehicle_models.formulations.yaw_map_workflow import YawMapWorkflowSettings
from tools.run.run_yaw_aware_forward_driver import anchored_axis, physical_map_speed_limit
from tools.sim_gui.execute import prepare_case
from tools.sim_gui.run_yaw_case import _verify_yaw_map_identity
from tools.sim_gui.settings import RunSettings
from tools.sim_gui.study import CaseSpec, StudyDefinition
from tools.sim_gui.vehicle import VehicleConfiguration

ROOT = Path(__file__).resolve().parents[1]
PLANAR = ROOT / 'models/formulations/iem24_murphy_real_planar_7dof.xml'
FOURTEEN = ROOT / 'models/formulations/iem24_murphy_real_14dof.xml'


class SimGuiYawMapTests(unittest.TestCase):
    def _identity(self, formulation):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
        from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
        compiled = compile_formulation(formulation).model
        if isinstance(compiled, VerticalVehicleParameters):
            compiled = compiled.planar
        vehicle = point_mass_view_of_planar(compiled)
        speed = np.linspace(0., physical_map_speed_limit(vehicle), 12)
        ax = anchored_axis(-6., 3., 9)
        return build_identity(vehicle, compiled.yaw_inertia, YawMapWorkflowSettings(), speed, ax)

    @staticmethod
    def _capability():
        return YawCapabilityMap([0., 10.], [-1., 1.], [-20., 20.], np.ones((2,2,2)),
            np.ones((2,2,2)), np.ones((2,2,2), bool), 1., 'physical_no_tv_v1', 'test')

    def test_missing_candidate_returns_fresh_request_and_build_command(self):
        with tempfile.TemporaryDirectory() as d:
            request = inspect_yaw_map(PLANAR, '', Path(d))
            self.assertIsInstance(request, MapBuildRequest)
            self.assertEqual(request.formulation, PLANAR.resolve())
            self.assertEqual(request.output.parent, Path(d).resolve())
            self.assertIn('--formulation', map_build_command(request))
            self.assertIn('--output', map_build_command(request))

    def test_valid_candidate_is_accepted_without_build(self):
        identity = self._identity(PLANAR)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'valid.npz'
            save_yaw_map(path, self._capability(), report={'complete': True}, identity=identity)
            with patch('tools.sim_gui.yaw_map.build_identity', return_value=identity):
                result = inspect_yaw_map(PLANAR, str(path), Path(d) / 'cache')
            self.assertEqual(result, path.resolve())

    def test_bad_candidates_request_replacement_and_acceptance_is_strict(self):
        identity = self._identity(PLANAR)
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            for label, writer, expected in (
                ('corrupt', lambda p: p.write_bytes(b'bad'), identity),
                ('incomplete', lambda p: save_yaw_map(p, self._capability(), report={'complete': False}, identity=identity), identity),
                ('stale', lambda p: save_yaw_map(p, self._capability(), report={'complete': True}, identity='old'), identity),
            ):
                candidate = root / f'{label}.npz'; writer(candidate)
                with patch('tools.sim_gui.yaw_map.build_identity', return_value=expected):
                    request = inspect_yaw_map(PLANAR, str(candidate), root / 'cache')
                self.assertIsInstance(request, MapBuildRequest, label)
                self.assertIn('candidate', request.reason.lower())
            request = MapBuildRequest(PLANAR, None, root / 'out.npz', identity, 'test')
            built = root / 'out.npz'
            save_yaw_map(built, self._capability(), report={'complete': True}, identity=identity)
            self.assertEqual(accept_yaw_map(request, built), built.resolve())

    def test_matching_cache_is_reused_but_different_tire_choices_change_identity(self):
        identity = self._identity(PLANAR)
        with tempfile.TemporaryDirectory() as d:
            cache = Path(d)
            failed_attempt = cache / f'{identity}-failed.npz'
            failed_attempt.write_bytes(b'incomplete attempt')
            artifact = cache / f'{identity}-accepted.npz'
            save_yaw_map(artifact, self._capability(), report={'complete': True}, identity=identity)
            with patch('tools.sim_gui.yaw_map.build_identity', return_value=identity):
                result = inspect_yaw_map(PLANAR, '', cache)
            self.assertEqual(result, artifact.resolve())
        vehicle_a = point_mass_view_of_planar(compile_formulation(PLANAR).model)
        from dataclasses import replace
        planar = compile_formulation(PLANAR).model
        vehicle_b = point_mass_view_of_planar(replace(planar, tire=replace(planar.tire, mu_0_x=planar.tire.mu_0_x * .9)))
        speeds = np.linspace(0., 50., 12); ax = anchored_axis(-6., 3., 9); settings = YawMapWorkflowSettings()
        first = build_identity(vehicle_a, compile_formulation(PLANAR).model.yaw_inertia, settings, speeds, ax)
        second = build_identity(vehicle_b, compile_formulation(PLANAR).model.yaw_inertia, settings, speeds, ax)
        self.assertNotEqual(first, second)

    def test_prepare_case_resolver_finalizes_materialized_map_identity(self):
        source = PLANAR
        track = ROOT / "models/tracks/mis_ev_endurance_2024_linear.xml"
        configuration = VehicleConfiguration.open(source)
        definition = StudyDefinition("resolver", str(source), str(track), "planar-7dof",
            {key: str(path) for key, path in configuration.choices.items()}, RunSettings(), "one-at-a-time", ())
        case = CaseSpec("case_000", "baseline", "planar-7dof", str(track), {}, {})
        with tempfile.TemporaryDirectory() as folder:
            artifact = Path(folder) / "resolved.npz"
            seen = []
            def resolver(formulation, settings):
                seen.append((formulation, settings))
                save_yaw_map(artifact, self._capability(), report={"complete": True},
                             identity=self._identity(formulation))
                return artifact
            prepared = prepare_case(Path(folder), definition, case, yaw_map_resolver=resolver)
            payload = json.loads((prepared.directory / "case.json").read_text())
            self.assertEqual(seen[0][0], prepared.formulation)
            self.assertEqual(payload["settings"]["yaw_map"], str(artifact.resolve()))
            self.assertEqual(payload["argv"][payload["argv"].index("--yaw-map") + 1], str(artifact.resolve()))
            self.assertEqual(payload["input_identity"]["yaw_map_sha256"], hashlib.sha256(artifact.read_bytes()).hexdigest())

    def test_real_tiny_artifact_acceptance_finalizes_case_and_rejects_tampering(self):
        from simplified_vehicle_models.formulations.yaw_map_artifact import FIELDS
        from tools.sim_gui.yaw_map import MapBuildRequest
        track = ROOT / "models/tracks/mis_ev_endurance_2024_linear.xml"
        configuration = VehicleConfiguration.open(PLANAR)
        definition = StudyDefinition("boundary", str(PLANAR), str(track), "planar-7dof",
            {key: str(path) for key, path in configuration.choices.items()}, RunSettings(), "one-at-a-time", ())
        case = CaseSpec("case_000", "baseline", "planar-7dof", str(track), {}, {})
        identity = self._identity(PLANAR)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            artifact = root / "tiny.npz"
            capability = self._capability()
            save_yaw_map(artifact, capability, report={"complete": True}, identity=identity)
            request = MapBuildRequest(PLANAR.resolve(), None, artifact, identity, "synthetic boundary")
            accepted = accept_yaw_map(request, artifact)
            self.assertEqual(accepted, artifact.resolve())
            prepared = prepare_case(root / "study", definition, case, yaw_map_resolver=lambda *_: accepted)
            payload = json.loads((prepared.directory / "case.json").read_text())
            self.assertEqual(payload["settings"]["yaw_map"], str(accepted))
            self.assertEqual(payload["argv"][payload["argv"].index("--yaw-map") + 1], str(accepted))
            self.assertEqual(payload["input_identity"]["yaw_map_sha256"], hashlib.sha256(accepted.read_bytes()).hexdigest())
            self.assertEqual(prepared.argv, payload["argv"])

            tampered = root / "tampered.npz"
            with np.load(artifact, allow_pickle=False) as archive:
                arrays = {name: np.asarray(archive[name]).copy() for name in archive.files}
            arrays[FIELDS[0]].flat[0] += 1
            with tampered.open("wb") as stream:
                np.savez_compressed(stream, **arrays)
            bad_request = MapBuildRequest(PLANAR.resolve(), None, tampered, identity, "tamper test")
            with self.assertRaisesRegex(ValueError, "integrity"):
                accept_yaw_map(bad_request, tampered)

    def test_resolver_runs_for_each_distinct_materialized_variant(self):
        track = ROOT / "models/tracks/mis_ev_endurance_2024_linear.xml"
        cfg = VehicleConfiguration.open(PLANAR)
        source14 = ROOT / "models/formulations/iem24_murphy_real_14dof.xml"
        cfg14 = VehicleConfiguration.open(source14)
        alternate = ROOT / "models/tires/iem24_murphy_real_unitire_transient.xml"
        definition = StudyDefinition("variants", str(source14), str(track), "14dof-simple",
            {key: str(path) for key, path in cfg14.choices.items()}, RunSettings(), "one-at-a-time", ())
        cases = (CaseSpec("case_000", "baseline", "14dof-simple", str(track), {}, {}),
                 CaseSpec("case_001", "alternate", "14dof-simple", str(track), {"tire": str(alternate)}, {}))
        with tempfile.TemporaryDirectory() as folder:
            seen = []
            def resolver(formulation, settings):
                seen.append((Path(formulation).read_bytes(), settings))
                artifact = Path(folder) / f"map-{len(seen)}.npz"
                save_yaw_map(artifact, self._capability(), report={"complete": True}, identity=self._identity(Path(formulation)))
                return artifact
            for case in cases:
                prepare_case(Path(folder), definition, case, yaw_map_resolver=resolver)
            self.assertEqual(len(seen), 2)
            self.assertNotEqual(seen[0][0], seen[1][0])

    def test_prepare_case_keeps_default_strict_and_skips_point_mass_resolver(self):
        track = ROOT / "models/tracks/mis_ev_endurance_2024_linear.xml"
        source = ROOT / "models/formulations/iem24_murphy_real_point_mass.xml"
        configuration = VehicleConfiguration.open(source)
        definition = StudyDefinition("point", str(source), str(track), "point-mass",
            {key: str(path) for key, path in configuration.choices.items()}, RunSettings(solver_kind="point-mass"), "one-at-a-time", ())
        case = CaseSpec("case_000", "baseline", "point-mass", str(track), {}, {})
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, "completed yaw-map artifact is required"):
                prepare_case(Path(folder), StudyDefinition("yaw", str(PLANAR), str(track), "planar-7dof", {}, RunSettings(), "one-at-a-time", ()),
                             CaseSpec("strict", "strict", "planar-7dof", str(track), {}, {}))
            resolver = unittest.mock.Mock(side_effect=AssertionError("point-mass must not resolve"))
            prepare_case(Path(folder), definition, case, yaw_map_resolver=resolver)
            resolver.assert_not_called()

    def test_changed_map_is_rejected_before_suite_launch(self):
        with tempfile.TemporaryDirectory() as folder:
            map_path = Path(folder) / "map.npz"
            map_path.write_bytes(b"accepted bytes")
            case = {"input_identity": {"yaw_map": str(map_path.resolve()),
                                       "yaw_map_sha256": hashlib.sha256(map_path.read_bytes()).hexdigest()}}
            map_path.write_bytes(b"mutated before execution")
            with patch("tools.run.run_yaw_lap_suite.run") as suite_run:
                with self.assertRaisesRegex(ValueError, "immutable case identity"):
                    _verify_yaw_map_identity(case, map_path.resolve())
                suite_run.assert_not_called()

    def test_resolver_failure_does_not_finalize_case_manifest(self):
        track = ROOT / "models/tracks/mis_ev_endurance_2024_linear.xml"
        configuration = VehicleConfiguration.open(PLANAR)
        definition = StudyDefinition("resolver", str(PLANAR), str(track), "planar-7dof",
            {key: str(path) for key, path in configuration.choices.items()}, RunSettings(), "one-at-a-time", ())
        case = CaseSpec("case_000", "baseline", "planar-7dof", str(track), {}, {})
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(RuntimeError, "build failed"):
                prepare_case(Path(folder), definition, case, yaw_map_resolver=lambda *_: (_ for _ in ()).throw(RuntimeError("build failed")))
            self.assertFalse((Path(folder) / "cases/case_000/case.json").exists())

    def test_14dof_reduction_matches_lap_suite(self):
        from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
        compiled = compile_formulation(FOURTEEN).model
        self.assertIsInstance(compiled, VerticalVehicleParameters)
        reduced = point_mass_view_of_planar(compiled.planar)
        self.assertEqual(reduced, point_mass_view_of_planar(compiled.planar))
        from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
        from tools.sim_gui.yaw_map import _compiled
        planar = _compiled(FOURTEEN)
        self.assertNotIsInstance(planar, VerticalVehicleParameters)
        evaluator = PhysicalYawEvaluator(point_mass_view_of_planar(planar), time_budget_seconds=10.)
        cell = evaluator.cell(15., -1., 0.)
        self.assertNotEqual(cell.status.value, 'unresolved', cell.detail)