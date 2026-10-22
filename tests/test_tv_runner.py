"""Tests for the controller-aware minimum-time runner modes."""

import contextlib
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from tools.run import solve_minimum_time_line as runner
from simplified_vehicle_models.controls.planning import (
    CONTROL_POLICIES, ControlResponse, controller_hash as policy_controller_hash,
)
from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.tracks.types import PathCoordinates


class _CapPolicy:
    """A controller policy whose only limit is 2 m/s² of drive."""

    name = "cap-two"
    provenance_hash = "cap-two-v1"
    is_stateful = False

    def evaluate(self, demand):
        feasible = demand.longitudinal_acceleration <= 2.0
        return ControlResponse(
            feasible, min(demand.longitudinal_acceleration, 2.0), 0.0,
        )


class _NeverPolicy:
    name = "never"
    provenance_hash = "never-v1"
    is_stateful = False

    def evaluate(self, demand):
        return ControlResponse(False, 0.0, 0.0)


def _surface(hash_value="hash", *, mode="ideal", controller_hash="",
             feasible_mask=None):
    speed = np.linspace(1.0, 30.0, 4)
    curvature = np.linspace(-0.2, 0.2, 4)
    shape = (speed.size, curvature.size)
    return CapabilitySurface(
        speed, curvature, np.full(shape, 3.0), np.full(shape, -8.0),
        np.full(curvature.size, 25.0), hash_value, mode, controller_hash,
        feasible_mask,
    )


def _distance():
    return np.array([0.0, 1.0, 2.0, 3.0, 4.0])


def _solution(lap_time, *, spline=False):
    distance = _distance()
    values = {
        "distance": distance,
        "offset": np.zeros(distance.size),
        "heading_error": np.zeros(distance.size),
        "curvature": np.zeros(distance.size),
        "speed": np.full(distance.size, 10.0),
        "acceleration": np.zeros(distance.size),
        "lap_time": lap_time,
        "iterations": 5,
        "maximum_violation": 0.0,
        "status": "Solve_Succeeded",
    }
    if spline:
        values["coefficients"] = np.zeros(4)
    else:
        values["curvature_rate"] = np.zeros(distance.size)
    return SimpleNamespace(**values)


def _track():
    distance = _distance()
    return SimpleNamespace(
        is_closed=True, has_edges=True, width_provenance="assumed",
        lap_length=4.0, distance=distance,
    )


def _corridor():
    distance = _distance()
    return SimpleNamespace(
        distance=distance, minimum=np.full(distance.size, -2.0),
        maximum=np.full(distance.size, 2.0), provenance="assumed",
        is_closed=True,
    )


def _path():
    distance = _distance()
    return PathCoordinates(
        distance=distance,
        x_north=np.array([0.0, 1.0, 2.0, 3.0, 0.0]),
        y_east=np.array([0.0, 0.0, 1.0, 2.0, 0.0]),
        heading=np.zeros(distance.size), curvature=np.zeros(distance.size),
        is_closed=True,
    )


class _RunnerFixture:
    """Patch the runner's I/O and solver seams for one focused pipeline run."""

    def __init__(self, testcase, *, spline, stages, zero, closed_loop=False,
                 surface=None, correction=None, track=None, corridor=None):
        self.testcase = testcase
        self.spline = spline
        self.stages = stages
        self.zero = zero
        self.closed_loop = closed_loop
        self.surface = surface if surface is not None else _surface()
        self.correction = correction
        self.stack = contextlib.ExitStack()
        self.testcase.addCleanup(self.stack.close)
        self.track = track if track is not None else _track()
        self.corridor = corridor if corridor is not None else _corridor()
        self.directory = None
        self.mocks = {}

    def enter(self, args):
        def patch_(target, name, **kwargs):
            mock = self.stack.enter_context(patch.object(target, name, **kwargs))
            self.mocks[name] = mock
            return mock

        patch_(runner, "load_track",
               side_effect=[self.track] * (1 if self.closed_loop else 2))
        patch_(runner, "corridor_from_track", return_value=self.corridor)
        patch_(runner, "compile_formulation",
               return_value=SimpleNamespace(model=object()))
        patch_(runner, "point_mass_view_of_planar", return_value=object())
        patch_(runner, "load_capability_surface", return_value=self.surface)
        patch_(runner, "capability_hash", return_value=self.surface.provenance_hash)
        centreline = SimpleNamespace(lap_time=58.0, speed=np.full(5, 10.0))
        replay = SimpleNamespace(lap_time=57.95, speed=np.full(5, 10.0))
        patch_(runner, "solve_lap",
               side_effect=[centreline] if self.closed_loop else [centreline, replay])
        patch_(runner, "solve_line", return_value=self.zero)
        spacings = tuple(float(item) for item in str(args.spacings).split(","))

        def ladder(*_args, stage_callback=None, **_kwargs):
            for spacing, stage in zip(spacings, self.stages):
                if stage_callback is not None:
                    stage_callback(spacing, stage, 0.1)
            return list(self.stages)

        patch_(runner, "solve_ladder", side_effect=ladder)
        patch_(runner, "solve_spline_line",
               side_effect=[self.zero] + list(self.stages))
        patch_(runner, "_spline_curvature_rate",
               side_effect=lambda _track, _spline, solution: np.zeros(
                   solution.distance.size))
        patch_(runner, "path_from_solution", return_value=_path())
        patch_(runner, "_write_rebuilt_track",
               return_value=(Path(args.output) / "racing_line_track.xml",
                             Path(args.output) / "racing_line_track.csv"))
        for name in (
            "check_zero_corridor_matches_envelope", "check_free_line_not_slower",
            "check_corridor_containment", "check_mesh_convergence",
            "check_replay_agreement",
        ):
            patch_(runner, name, return_value=True)
        if self.correction is not None:
            patch_(runner, "correct_to_replay", return_value=self.correction)
        return self


def _args(directory, **overrides):
    source = Path(directory) / "source.xml"
    source.write_text(
        '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
        '<metadata><title>fixture</title></metadata><data ref="geometry.csv" />'
        '</model>',
        encoding="utf-8",
    )
    values = {
        "track": source,
        "formulation": Path("formulation.xml"),
        "surface": Path("surface.npz"),
        "output": Path(directory) / "output",
        "margin": 0.10,
        "half_width": None,
        "spacings": "5,2",
        "offset_spacing": None,
        "path_smoothing": 0.0,
        "control_mode": "ideal",
        "control_policy": None,
        "replay_adapter": None,
        "solver": None,
        "legacy_solver": False,
        "compare_control_modes": None,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class ControlModeValidationTest(unittest.TestCase):
    def test_control_mode_is_required_to_be_known(self):
        with patch("sys.argv", [
            "runner", "--track", "x", "--formulation", "y", "--surface", "z",
            "--output", "out", "--control-mode", "not-a-mode",
        ]):
            with self.assertRaises((ValueError, SystemExit)):
                runner.main()

    def test_ideal_and_allocation_are_distinct_summary_modes(self):
        ideal = runner._control_summary("ideal", "", "ideal")
        allocation = runner._control_summary("allocation", "hash", "allocation")
        self.assertEqual(ideal["control_mode"], "ideal")
        self.assertNotEqual(ideal["control_mode"], allocation["control_mode"])
        self.assertEqual(allocation["controller_hash"], "hash")
        self.assertEqual(allocation["capability_mode"], "allocation")

    def test_closed_loop_requires_a_replay_adapter(self):
        with self.assertRaisesRegex(ValueError, "replay adapter"):
            runner._validate_control_args(SimpleNamespace(
                control_mode="closed_loop", replay_adapter=None,
                control_policy=None,
            ))

    def test_allocation_requires_a_controller_policy(self):
        with self.assertRaisesRegex(ValueError, "controller policy"):
            runner._validate_control_args(SimpleNamespace(
                control_mode="allocation", replay_adapter=None,
                control_policy=None,
            ))

    def test_unknown_mode_is_rejected_by_the_helper(self):
        with self.assertRaisesRegex(ValueError, "unknown control mode"):
            runner._validate_control_args(SimpleNamespace(
                control_mode="not-a-mode", replay_adapter=None,
                control_policy=None,
            ))
        with self.assertRaisesRegex(ValueError, "unknown control mode"):
            runner._control_summary("not-a-mode", "", "ideal")

    def test_ideal_needs_neither_a_policy_nor_a_replay_adapter(self):
        self.assertEqual(runner._validate_control_args(SimpleNamespace(
            control_mode="ideal", replay_adapter=None, control_policy=None,
        )), "ideal")
        self.assertEqual(runner._validate_control_args(SimpleNamespace(
            control_mode="closed_loop", replay_adapter="point-mass",
            control_policy=None,
        )), "closed_loop")


class SolverSelectionTest(unittest.TestCase):
    def test_ideal_and_allocation_default_to_the_legacy_transcription(self):
        for mode in ("ideal", "allocation"):
            with self.subTest(mode=mode):
                self.assertEqual(runner._resolve_solver(SimpleNamespace(
                    control_mode=mode, solver=None, legacy_solver=False,
                )), "legacy")

    def test_closed_loop_requires_an_explicit_spline_solver(self):
        with self.assertRaisesRegex(ValueError, "explicit --solver spline"):
            runner._resolve_solver(SimpleNamespace(
                control_mode="closed_loop", solver=None, legacy_solver=False,
            ))

    def test_explicit_solver_and_legacy_alias(self):
        self.assertEqual(runner._resolve_solver(SimpleNamespace(
            control_mode="ideal", solver="spline", legacy_solver=False,
        )), "spline")
        self.assertEqual(runner._resolve_solver(SimpleNamespace(
            control_mode="allocation", solver="spline", legacy_solver=False,
        )), "spline")
        self.assertEqual(runner._resolve_solver(SimpleNamespace(
            control_mode="ideal", solver=None, legacy_solver=True,
        )), "legacy")
        self.assertEqual(runner._resolve_solver(SimpleNamespace(
            control_mode="allocation", solver=None, legacy_solver=True,
        )), "legacy")

    def test_legacy_alias_conflicts_with_an_explicit_spline_solver(self):
        with self.assertRaisesRegex(ValueError, "conflicts"):
            runner._resolve_solver(SimpleNamespace(
                control_mode="ideal", solver="spline", legacy_solver=True,
            ))

    def test_closed_loop_rejects_the_legacy_solver(self):
        for legacy_solver in (False, True):
            with self.subTest(legacy_solver=legacy_solver):
                with self.assertRaisesRegex(ValueError, "explicit --solver spline"):
                    runner._resolve_solver(SimpleNamespace(
                        control_mode="closed_loop", solver="legacy",
                        legacy_solver=legacy_solver,
                    ))


class PipelineModeTest(unittest.TestCase):
    def test_ideal_legacy_summary_records_control_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(directory)
            zero = _solution(57.9)
            stages = [_solution(57.95), _solution(57.9)]
            fixture = _RunnerFixture(
                self, spline=False, stages=stages, zero=zero,
            ).enter(args)
            summary = runner.run(args)
            self.assertEqual(summary["control_mode"], "ideal")
            self.assertEqual(summary["requested_control_mode"], "ideal")
            self.assertEqual(summary["controller_hash"], "")
            self.assertEqual(summary["capability_mode"], "ideal")
            self.assertEqual(summary["solver"], "legacy")
            self.assertEqual(summary["rounds"], 0)
            self.assertFalse(summary["accepted"])
            self.assertEqual(summary["replay_time_s"], 57.95)
            self.assertAlmostEqual(summary["replay_lap_s"], 57.95)
            self.assertEqual(summary["solver_tolerance"], 1.0e-6)
            self.assertIsNone(summary["spline_type"])
            self.assertIn("track_xml", summary["track_source_hashes"])
            self.assertEqual(
                len(summary["track_source_hashes"]["track_xml"]), 64,
            )
            self.assertTrue((Path(directory) / "output" / "summary.json").is_file())
            fixture.mocks["solve_ladder"].assert_called_once()

    def test_spline_solver_uses_the_spline_ladder_and_summary_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(directory, solver="spline")
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero,
            ).enter(args)
            summary = runner.run(args)
            self.assertEqual(fixture.mocks["solve_spline_line"].call_count, 3)
            fixture.mocks["solve_ladder"].assert_not_called()
            self.assertEqual(summary["solver"], "spline")
            self.assertEqual(summary["control_mode"], "ideal")
            self.assertEqual(summary["spline_control_spacing_m"], 10.0)
            self.assertEqual(summary["spline_type"], "closed-periodic")
            self.assertEqual(summary["stages"][-1]["spacing_m"], 2.0)

    def test_allocation_samples_a_hash_named_cache(self):
        CONTROL_POLICIES["cap-two"] = _CapPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "cap-two", None)
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="allocation", control_policy="cap-two",
                solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            _RunnerFixture(
                self, spline=True, stages=stages, zero=zero,
            ).enter(args)
            summary = runner.run(args)
            expected_hash = policy_controller_hash(_CapPolicy())
            self.assertEqual(summary["control_mode"], "allocation")
            self.assertEqual(summary["capability_mode"], "allocation")
            self.assertEqual(summary["controller_hash"], expected_hash)
            self.assertIn(expected_hash, summary["capability_cache"])
            self.assertTrue(Path(summary["capability_cache"]).is_file())
            self.assertEqual(summary["capability_metadata"]["policy_name"], "cap-two")
            self.assertEqual(summary["saturation"], [])
            from simplified_vehicle_models.formulations.point_mass_capability import (
                load_capability_surface,
            )
            cached = load_capability_surface(summary["capability_cache"])
            self.assertLessEqual(float(np.max(cached.acceleration_max)), 2.0 + 1e-9)

    def test_allocation_defaults_to_the_legacy_solver(self):
        CONTROL_POLICIES["cap-two"] = _CapPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "cap-two", None)
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="allocation", control_policy="cap-two",
            )
            zero = _solution(57.9)
            stages = [_solution(57.95), _solution(57.9)]
            fixture = _RunnerFixture(
                self, spline=False, stages=stages, zero=zero,
            ).enter(args)
            summary = runner.run(args)
            self.assertEqual(summary["solver"], "legacy")
            self.assertEqual(summary["control_mode"], "allocation")
            self.assertEqual(summary["capability_mode"], "allocation")
            fixture.mocks["solve_ladder"].assert_called_once()
            fixture.mocks["solve_spline_line"].assert_not_called()
            self.assertTrue(Path(summary["capability_cache"]).is_file())

    def test_infeasible_allocation_cells_fail_before_output_or_solve(self):
        CONTROL_POLICIES["never"] = _NeverPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "never", None)
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="allocation", control_policy="never",
                solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            _RunnerFixture(
                self, spline=True, stages=stages, zero=zero,
            ).enter(args)
            with patch.object(runner, "solve_spline_line") as spline_solve:
                with self.assertRaisesRegex(
                        ValueError, "infeasible.*zero-acceleration"):
                    runner.run(args)
            spline_solve.assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_closed_loop_requires_a_registered_replay_adapter(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="closed_loop", replay_adapter="missing",
                solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            _RunnerFixture(
                self, spline=True, stages=stages, zero=zero, closed_loop=True,
            ).enter(args)
            with patch.object(runner, "solve_spline_line") as spline_solve:
                with self.assertRaisesRegex(ValueError, "unknown replay adapter"):
                    runner.run(args)
            spline_solve.assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_closed_loop_requires_an_explicit_spline_solver_before_output(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="closed_loop",
                replay_adapter="point-mass",
            )
            zero = _solution(57.9)
            stages = [_solution(57.95), _solution(57.9)]
            fixture = _RunnerFixture(
                self, spline=False, stages=stages, zero=zero, closed_loop=True,
            ).enter(args)
            with self.assertRaisesRegex(ValueError, "explicit --solver spline"):
                runner.run(args)
            fixture.mocks["compile_formulation"].assert_not_called()
            fixture.mocks["solve_spline_line"].assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_open_course_closed_loop_is_rejected_before_output(self):
        open_track = SimpleNamespace(
            is_closed=False, has_edges=True, width_provenance="assumed",
            lap_length=4.0, distance=_distance(),
        )
        open_corridor = SimpleNamespace(
            distance=_distance(), minimum=np.full(5, -2.0),
            maximum=np.full(5, 2.0), provenance="assumed", is_closed=False,
        )
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="closed_loop",
                replay_adapter="point-mass", solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero, closed_loop=True,
                track=open_track, corridor=open_corridor,
            ).enter(args)
            with self.assertRaisesRegex(ValueError, "open course"):
                runner.run(args)
            fixture.mocks["compile_formulation"].assert_not_called()
            fixture.mocks["solve_spline_line"].assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_closed_loop_mesh_convergence_is_recorded_unavailable(self):
        from simplified_vehicle_models.solvers.closed_loop_line import CorrectionResult

        correction = CorrectionResult(
            rounds=1, replay_time=57.9,
            speed_caps=(np.full(5, 10.0),), cap_margin=0.1, accepted=True,
            feasible=True,
        )
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="closed_loop",
                replay_adapter="point-mass", solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            _RunnerFixture(
                self, spline=True, stages=stages, zero=zero, closed_loop=True,
                correction=correction,
            ).enter(args)
            summary = runner.run(args)
            self.assertIsNone(summary["checks"]["mesh_convergence"])
            self.assertIsNone(summary["checks"]["zero_corridor_matches_envelope"])
            self.assertIn("check_unavailable", summary)
            self.assertIn("corrected line",
                          summary["check_unavailable"]["mesh_convergence"])
            self.assertIn("zero_corridor_matches_envelope",
                          summary["check_unavailable"])
            self.assertTrue(summary["checks"]["corridor_containment"])
            self.assertTrue(summary["checks"]["replay_agreement"])
            self.assertFalse(summary["accepted"])

    def test_recorded_mode_mismatch_is_an_error(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(directory)
            zero = _solution(57.9)
            stages = [_solution(57.95), _solution(57.9)]
            fixture = _RunnerFixture(
                self, spline=False, stages=stages, zero=zero,
            ).enter(args)
            mismatched = SimpleNamespace(
                surface=fixture.surface, capability_mode="allocation",
                controller_hash="wrong", saturation=(), metadata={},
                cache_path=None,
            )
            with patch.object(runner, "_select_capability",
                              return_value=mismatched):
                with self.assertRaisesRegex(
                        ValueError,
                        "requested control mode 'ideal'.*recorded 'allocation'"):
                    runner.run(args)

    def test_unknown_policy_fails_before_output(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="allocation", control_policy="missing",
                solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            _RunnerFixture(
                self, spline=True, stages=stages, zero=zero,
            ).enter(args)
            with patch.object(runner, "compile_formulation") as compile_formulation:
                with self.assertRaisesRegex(ValueError, "unknown control policy"):
                    runner.run(args)
            compile_formulation.assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_closed_loop_reports_correction_rounds_and_replay_time(self):
        from simplified_vehicle_models.solvers.closed_loop_line import CorrectionResult

        correction = CorrectionResult(
            rounds=2, replay_time=57.9,
            speed_caps=(np.array([10.0, 10.0, 10.0, 10.0, 10.0]),),
            cap_margin=0.1, accepted=True, feasible=True,
        )
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="closed_loop",
                replay_adapter="point-mass", solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero, closed_loop=True,
                correction=correction,
            ).enter(args)
            summary = runner.run(args)
            self.assertEqual(summary["control_mode"], "closed_loop")
            self.assertEqual(summary["capability_mode"], "ideal")
            self.assertEqual(summary["replay_adapter"], "point-mass")
            self.assertFalse(summary["controller_aware"])
            self.assertEqual(summary["rounds"], 2)
            self.assertFalse(summary["accepted"])
            self.assertEqual(summary["replay_time_s"], 57.9)
            self.assertTrue(summary["checks"]["replay_agreement"])
            self.assertIsNone(summary["checks"]["mesh_convergence"])
            self.assertIsNone(summary["checks"]["zero_corridor_matches_envelope"])
            self.assertIsNone(summary["checks"]["free_line_not_slower"])
            self.assertIn("free_line_not_slower", summary["check_unavailable"])
            self.assertEqual(summary["replay_kind"], "point_mass_parity")
            fixture.mocks["check_free_line_not_slower"].assert_not_called()

    def test_surface_cache_provenance_is_validated(self):
        ideal = _surface()
        runner._validate_surface_cache(ideal, "ideal", None)
        allocation = _surface(
            mode="allocation", controller_hash=policy_controller_hash(_CapPolicy()),
        )
        runner._validate_surface_cache(allocation, "allocation", _CapPolicy())
        with self.assertRaisesRegex(ValueError, "launder"):
            runner._validate_surface_cache(allocation, "ideal", None)
        with self.assertRaisesRegex(ValueError, "controller hash mismatch"):
            runner._validate_surface_cache(allocation, "allocation", _NeverPolicy())
        with self.assertRaisesRegex(ValueError, "requires.*control-policy"):
            runner._validate_surface_cache(allocation, "allocation", None)

        class StatefulPolicy(_CapPolicy):
            is_stateful = True

        with self.assertRaisesRegex(ValueError, "stateful"):
            runner._validate_surface_cache(
                allocation, "closed_loop", StatefulPolicy(),
            )

    def test_allocation_cache_cannot_be_used_as_the_ideal_surface(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(directory)
            allocation = _surface(
                mode="allocation",
                controller_hash=policy_controller_hash(_CapPolicy()),
            )
            fixture = _RunnerFixture(
                self, spline=False, stages=[_solution(57.9), _solution(57.9)],
                zero=_solution(57.9), surface=allocation,
            ).enter(args)
            with self.assertRaisesRegex(ValueError, "launder"):
                runner.run(args)
            fixture.mocks["solve_line"].assert_not_called()
            fixture.mocks["solve_ladder"].assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_closed_loop_with_a_policy_rejects_a_controller_ignoring_adapter(self):
        CONTROL_POLICIES["cap-two"] = _CapPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "cap-two", None)
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="closed_loop", control_policy="cap-two",
                replay_adapter="point-mass", solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero, closed_loop=True,
            ).enter(args)
            with self.assertRaisesRegex(ValueError, "consumes the controller"):
                runner.run(args)
            fixture.mocks["solve_spline_line"].assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_closed_loop_with_a_consuming_adapter_records_controller_replay(self):
        from simplified_vehicle_models.controls.replay import (
            REPLAY_ADAPTERS, register_replay,
        )
        from simplified_vehicle_models.solvers.closed_loop_line import CorrectionResult

        class ConsumingReplay:
            consumes_controller = True

            def __init__(self, vehicle, lap_mode):
                self.vehicle = vehicle
                self.lap_mode = lap_mode

            def __call__(self, path, controller):
                raise AssertionError("the patched correction loop must not replay")

        register_replay("test-consuming", ConsumingReplay)
        self.addCleanup(REPLAY_ADAPTERS.pop, "test-consuming", None)
        CONTROL_POLICIES["cap-two"] = _CapPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "cap-two", None)
        correction = CorrectionResult(
            rounds=1, replay_time=57.9,
            speed_caps=(np.full(5, 10.0),), cap_margin=0.1,
            accepted=True, feasible=True,
        )
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="closed_loop", control_policy="cap-two",
                replay_adapter="test-consuming", solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            _RunnerFixture(
                self, spline=True, stages=stages, zero=zero, closed_loop=True,
                correction=correction,
            ).enter(args)
            summary = runner.run(args)
            self.assertTrue(summary["controller_aware"])
            self.assertEqual(summary["replay_kind"], "controller_adapter")
            self.assertFalse(summary["accepted"])

    def test_allocation_gates_use_same_surface_totals_and_unavailable_references(self):
        CONTROL_POLICIES["cap-two"] = _CapPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "cap-two", None)
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="allocation", control_policy="cap-two",
                solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero,
            ).enter(args)
            summary = runner.run(args)
            self.assertIsNone(summary["checks"]["zero_corridor_matches_envelope"])
            self.assertIsNone(summary["checks"]["replay_agreement"])
            self.assertIn("zero_corridor_matches_envelope",
                          summary["check_unavailable"])
            self.assertIn("replay_agreement", summary["check_unavailable"])
            fixture.mocks["check_free_line_not_slower"].assert_called_once_with(
                centreline=57.9, free=57.9, tolerance=0.0,
            )
            fixture.mocks["check_zero_corridor_matches_envelope"].assert_not_called()
            fixture.mocks["check_replay_agreement"].assert_not_called()
            self.assertFalse(summary["accepted"])
            self.assertTrue(summary["controller_aware"])
            self.assertEqual(summary["replay_kind"], "point_mass_parity")

    def test_allocation_open_uses_same_surface_capability_envelope(self):
        CONTROL_POLICIES["cap-two"] = _CapPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "cap-two", None)
        open_track = SimpleNamespace(
            is_closed=False, has_edges=True, width_provenance="assumed",
            lap_length=4.0, distance=_distance(),
            curvature=np.zeros(_distance().size),
        )
        open_corridor = SimpleNamespace(
            distance=_distance(), minimum=np.full(5, -2.0),
            maximum=np.full(5, 2.0), provenance="assumed", is_closed=False,
        )
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="allocation", control_policy="cap-two",
                solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero,
                track=open_track, corridor=open_corridor,
            ).enter(args)
            with (
                patch.object(runner, "launch_time", return_value=0.5),
                patch.object(runner, "capability_envelope_time",
                             return_value=10.5) as envelope,
            ):
                summary = runner.run(args)
            # The reference is the allocation surface, not the ideal vehicle.
            self.assertEqual(envelope.call_args.args[1].mode, "allocation")
            fixture.mocks["check_zero_corridor_matches_envelope"].assert_called_once_with(
                open_loop=10.5, ocp=58.4, tolerance=0.01,
            )
            fixture.mocks["check_free_line_not_slower"].assert_called_once_with(
                centreline=10.5, free=58.4, tolerance=0.002,
            )
            self.assertTrue(summary["checks"]["zero_corridor_matches_envelope"])
            self.assertTrue(summary["checks"]["free_line_not_slower"])
            self.assertEqual(summary["zero_corridor_reference_s"], 10.5)
            self.assertEqual(summary["spline_type"], "open-natural")
            self.assertEqual(summary["replay_kind"], "point_mass_parity")
            self.assertFalse(summary["accepted"])

    def test_failed_available_gate_makes_accepted_false(self):
        CONTROL_POLICIES["cap-two"] = _CapPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "cap-two", None)
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="allocation", control_policy="cap-two",
                solver="spline",
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero,
            ).enter(args)
            fixture.mocks["check_corridor_containment"].return_value = False
            summary = runner.run(args)
            self.assertFalse(summary["checks"]["corridor_containment"])
            self.assertFalse(summary["accepted"])

    def test_closed_loop_replay_gate_uses_times_and_feasibility(self):
        from tools.gates.check_minimum_time_line import (
            check_replay_agreement as real_replay_gate,
        )
        from simplified_vehicle_models.solvers.closed_loop_line import CorrectionResult

        def correction(replay_time, feasible):
            return CorrectionResult(
                rounds=1, replay_time=replay_time,
                speed_caps=(np.full(5, 10.0),), cap_margin=0.1,
                accepted=True, feasible=feasible,
            )

        cases = (
            ("time", 60.0, True, False),
            ("feasible", 57.9, False, False),
            ("match", 57.9, True, True),
        )
        for name, replay_time, feasible, expected in cases:
            with self.subTest(case=name, replay_time=replay_time,
                              feasible=feasible):
                with tempfile.TemporaryDirectory() as directory:
                    args = _args(
                        directory, control_mode="closed_loop",
                        replay_adapter="point-mass", solver="spline",
                    )
                    zero = _solution(57.9, spline=True)
                    stages = [_solution(57.95, spline=True),
                              _solution(57.9, spline=True)]
                    fixture = _RunnerFixture(
                        self, spline=True, stages=stages, zero=zero,
                        closed_loop=True,
                        correction=correction(replay_time, feasible),
                    ).enter(args)
                    fixture.mocks["check_replay_agreement"].side_effect = (
                        real_replay_gate
                    )
                    summary = runner.run(args)
                    self.assertEqual(
                        summary["checks"]["replay_agreement"], expected,
                    )
                    self.assertFalse(summary["accepted"])
                    self.assertEqual(summary["validation_status"], "incomplete" if expected else "failed")

    def test_spline_rejects_path_smoothing_before_output_or_solver(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(directory, solver="spline", path_smoothing=1.0)
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(self, spline=True, stages=stages, zero=zero).enter(args)
            with self.assertRaisesRegex(ValueError, "smoothing is not permitted"):
                runner.run(args)
            fixture.mocks["compile_formulation"].assert_not_called()
            fixture.mocks["solve_spline_line"].assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_existing_run_artifact_rejects_before_solver_or_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(directory, solver="spline")
            output = Path(args.output)
            output.mkdir()
            expected = {}
            for name in ("line.csv", "trajectory.npz", "summary.json"):
                payload = f"prior {name}\n".encode()
                (output / name).write_bytes(payload)
                expected[name] = payload
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(self, spline=True, stages=stages, zero=zero).enter(args)
            with self.assertRaisesRegex(FileExistsError, "already exists"):
                runner.run(args)
            fixture.mocks["compile_formulation"].assert_not_called()
            fixture.mocks["solve_spline_line"].assert_not_called()
            for name, payload in expected.items():
                self.assertEqual((output / name).read_bytes(), payload)

    def test_closed_loop_rejects_path_smoothing(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(
                directory, control_mode="closed_loop",
                replay_adapter="point-mass", solver="spline",
                path_smoothing=1.0,
            )
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            _RunnerFixture(
                self, spline=True, stages=stages, zero=zero, closed_loop=True,
            ).enter(args)
            with self.assertRaisesRegex(ValueError, "path-smoothing"):
                runner.run(args)
            self.assertFalse((Path(directory) / "output").exists())

    def test_allocation_without_policy_fails_before_output_or_solver(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(directory, control_mode="allocation", solver="spline")
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero,
            ).enter(args)
            with self.assertRaisesRegex(ValueError, "controller policy"):
                runner.run(args)
            fixture.mocks["compile_formulation"].assert_not_called()
            fixture.mocks["solve_spline_line"].assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())

    def test_closed_loop_without_adapter_fails_before_output_or_solver(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _args(directory, control_mode="closed_loop", solver="spline")
            zero = _solution(57.9, spline=True)
            stages = [_solution(57.95, spline=True), _solution(57.9, spline=True)]
            fixture = _RunnerFixture(
                self, spline=True, stages=stages, zero=zero, closed_loop=True,
            ).enter(args)
            with self.assertRaisesRegex(ValueError, "replay adapter"):
                runner.run(args)
            fixture.mocks["compile_formulation"].assert_not_called()
            fixture.mocks["solve_spline_line"].assert_not_called()
            self.assertFalse((Path(directory) / "output").exists())


class CliWiringTest(unittest.TestCase):
    def test_control_flags_reach_the_run_namespace(self):
        with patch.object(runner, "run", return_value={"checks": {}}) as run:
            status = runner.main([
                "--track", "track.xml", "--formulation", "formulation.xml",
                "--surface", "surface.npz", "--output", "out",
                "--control-mode", "closed_loop", "--control-policy", "policy",
                "--replay-adapter", "point-mass", "--solver", "spline",
            ])
        self.assertEqual(status, 1)
        args = run.call_args.args[0]
        self.assertEqual(args.control_mode, "closed_loop")
        self.assertEqual(args.control_policy, "policy")
        self.assertEqual(args.replay_adapter, "point-mass")
        self.assertEqual(args.solver, "spline")
        self.assertIsNone(args.compare_control_modes)

    def test_legacy_solver_alias_is_wired(self):
        with patch.object(runner, "run", return_value={"checks": {}}) as run:
            self.assertEqual(runner.main([
                "--track", "track.xml", "--formulation", "formulation.xml",
                "--surface", "surface.npz", "--output", "out",
                "--legacy-solver",
            ]), 1)
        self.assertTrue(run.call_args.args[0].legacy_solver)

    def test_compare_control_modes_flag_dispatches(self):
        with patch.object(
                runner, "compare_control_modes",
                return_value={"all_accepted": True}) as compare:
            status = runner.main([
                "--track", "track.xml", "--formulation", "formulation.xml",
                "--surface", "surface.npz", "--output", "out",
                "--compare-control-modes",
            ])
        self.assertEqual(status, 0)
        compare.assert_called_once()
        self.assertEqual(compare.call_args.args[0].compare_control_modes, "")
        self.assertIsNone(compare.call_args.args[1])

    def test_compare_control_modes_custom_path_is_forwarded(self):
        custom = Path("custom-comparison.json")
        with patch.object(
                runner, "compare_control_modes",
                return_value={"all_accepted": True}) as compare:
            status = runner.main([
                "--track", "track.xml", "--formulation", "formulation.xml",
                "--surface", "surface.npz", "--output", "out",
                "--solver", "spline",
                "--compare-control-modes", str(custom),
            ])
        self.assertEqual(status, 0)
        self.assertEqual(compare.call_args.args[1], custom)

    def test_comparison_failure_changes_the_exit_code(self):
        with patch.object(
                runner, "compare_control_modes",
                return_value={"all_accepted": False}) as compare:
            status = runner.main([
                "--track", "track.xml", "--formulation", "formulation.xml",
                "--surface", "surface.npz", "--output", "out",
                "--compare-control-modes",
            ])
        self.assertEqual(status, 1)
        compare.assert_called_once()

    def test_unavailable_gate_blocks_strict_validation(self):
        self.assertFalse(runner._checks_pass({
            "mesh_convergence": None, "corridor_containment": True,
        }))
        self.assertFalse(runner._checks_pass({
            "mesh_convergence": False, "corridor_containment": True,
        }))
        with patch.object(
                runner, "run",
                return_value={"checks": {"mesh_convergence": None},
                              "accepted": True}):
            self.assertEqual(runner.main([
                "--track", "track.xml", "--formulation", "formulation.xml",
                "--surface", "surface.npz", "--output", "out",
            ]), 1)


class ControlModeComparisonTest(unittest.TestCase):
    def _comparison_args(self, directory):
        return _args(
            directory, control_policy="cap-two", replay_adapter="point-mass",
            solver="spline", compare_control_modes="",
        )

    def _summary_for(self, mode):
        return {
            "control_mode": mode,
            "controller_hash": "" if mode == "ideal" else "hash",
            "capability_mode": "ideal" if mode == "ideal" else "allocation",
            "solver": "spline",
            "ocp_free_line_including_launch_s": {
                "ideal": 57.9, "allocation": 58.2, "closed_loop": 58.0,
            }[mode],
            "replay_time_s": {
                "ideal": 57.9, "allocation": 58.2, "closed_loop": 57.99,
            }[mode],
            "maximum_violation": 0.0,
            "rounds": 2 if mode == "closed_loop" else 0,
            "accepted": True,
            "validation_status": "numerically_validated",
            "saturation": ["braking"] if mode == "allocation" else [],
            "checks": {
                "zero_corridor_matches_envelope": (
                    True if mode == "ideal" else None
                ),
                "free_line_not_slower": (
                    None if mode == "closed_loop" else True
                ),
                "corridor_containment": True,
                "mesh_convergence": None if mode == "closed_loop" else True,
                "replay_agreement": None if mode == "allocation" else True,
            },
        }

    def test_comparison_runs_every_mode_and_refuses_overwrite(self):
        CONTROL_POLICIES["cap-two"] = _CapPolicy
        self.addCleanup(CONTROL_POLICIES.pop, "cap-two", None)
        with tempfile.TemporaryDirectory() as directory:
            args = self._comparison_args(directory)
            comparison_path = Path(directory) / "comparison.json"

            def fake_run(mode_args):
                return self._summary_for(mode_args.control_mode)

            with (
                patch.object(runner, "run", side_effect=fake_run) as run,
                patch.object(runner, "_line_offset_rms",
                             side_effect=[0.05, 0.02]),
            ):
                comparison = runner.compare_control_modes(args, comparison_path)
            self.assertEqual(run.call_count, 3)
            self.assertEqual(
                [call.args[0].control_mode for call in run.call_args_list],
                ["ideal", "allocation", "closed_loop"],
            )
            for call in run.call_args_list:
                self.assertEqual(call.args[0].solver, "spline")
            self.assertTrue(comparison_path.is_file())
            stored = json.loads(comparison_path.read_text(encoding="utf-8"))
            self.assertEqual(
                set(stored["modes"]), {"ideal", "allocation", "closed_loop"},
            )
            self.assertEqual(stored["line_offset_rms_m"]["ideal"], 0.0)
            self.assertEqual(stored["line_offset_rms_m"]["allocation"], 0.05)
            self.assertEqual(stored["line_offset_rms_m"]["closed_loop"], 0.02)
            self.assertEqual(stored["replay_time_s"]["closed_loop"], 57.99)
            self.assertEqual(stored["saturation"]["allocation"], ["braking"])
            self.assertEqual(stored["solver"], "spline")
            self.assertTrue(stored["all_accepted"])
            self.assertEqual(stored["failed_gates"]["ideal"], [])
            self.assertEqual(stored["failed_gates"]["allocation"], [])
            self.assertEqual(stored["failed_gates"]["closed_loop"], [])
            self.assertIsNone(stored["mesh_converged"]["closed_loop"])
            self.assertEqual(
                stored["unavailable_gates"]["closed_loop"],
                ["zero_corridor_matches_envelope", "free_line_not_slower",
                 "mesh_convergence"],
            )
            self.assertEqual(
                stored["unavailable_gates"]["allocation"],
                ["zero_corridor_matches_envelope", "replay_agreement"],
            )
            self.assertTrue(
                stored["modes"]["ideal"]["checks"]["corridor_containment"])
            with self.assertRaises(FileExistsError):
                runner.compare_control_modes(args, comparison_path)

    def test_comparison_honors_the_cli_path(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self._comparison_args(directory)
            custom = Path(directory) / "custom-comparison.json"
            args.compare_control_modes = str(custom)
            default = Path(directory) / "output" / "control_mode_comparison.json"

            def fake_run(mode_args):
                return self._summary_for(mode_args.control_mode)

            with (
                patch.object(runner, "run", side_effect=fake_run),
                patch.object(runner, "_line_offset_rms",
                             side_effect=[0.05, 0.02]),
            ):
                runner.compare_control_modes(args)
            self.assertTrue(custom.is_file())
            self.assertFalse(default.exists())

    def test_comparison_requires_an_explicit_spline_solver(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self._comparison_args(directory)
            args.solver = None
            with patch.object(runner, "run") as run:
                with self.assertRaisesRegex(ValueError, "explicit --solver spline"):
                    runner.compare_control_modes(
                        args, Path(directory) / "comparison.json")
            run.assert_not_called()

    def test_comparison_uses_every_gate_and_reports_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self._comparison_args(directory)
            comparison_path = Path(directory) / "comparison.json"

            def fake_run(mode_args):
                summary = self._summary_for(mode_args.control_mode)
                if mode_args.control_mode == "ideal":
                    summary["validation_status"] = "failed"
                    summary["accepted"] = False
                    summary["checks"]["corridor_containment"] = False
                return summary

            with (
                patch.object(runner, "run", side_effect=fake_run),
                patch.object(runner, "_line_offset_rms",
                             side_effect=[0.05, 0.02]),
            ):
                comparison = runner.compare_control_modes(
                    args, comparison_path)
            self.assertFalse(comparison["all_accepted"])
            self.assertEqual(comparison["modes"]["ideal"]["status"], "failed")
            self.assertEqual(comparison["failed_gates"]["ideal"],
                             ["corridor_containment"])
            self.assertEqual(comparison["modes"]["allocation"]["status"], "ok")
            self.assertEqual(comparison["modes"]["closed_loop"]["status"], "ok")
            self.assertEqual(
                comparison["modes"]["closed_loop"]["unavailable_gates"],
                ["zero_corridor_matches_envelope", "free_line_not_slower",
                 "mesh_convergence"],
            )

    def test_comparison_requires_policy_and_adapter_before_running(self):
        with tempfile.TemporaryDirectory() as directory:
            for overrides, message in (
                ({"control_policy": None}, "control-policy"),
                ({"replay_adapter": None}, "replay-adapter"),
            ):
                with self.subTest(overrides=overrides):
                    args = self._comparison_args(directory)
                    for name, value in overrides.items():
                        setattr(args, name, value)
                    with patch.object(runner, "run") as run:
                        with self.assertRaisesRegex(ValueError, message):
                            runner.compare_control_modes(
                                args, Path(directory) / "comparison.json")
                    run.assert_not_called()

    def test_comparison_refuses_an_existing_path_before_running(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self._comparison_args(directory)
            comparison_path = Path(directory) / "comparison.json"
            comparison_path.write_text("existing\n", encoding="utf-8")
            with patch.object(runner, "run") as run:
                with self.assertRaises(FileExistsError):
                    runner.compare_control_modes(args, comparison_path)
            run.assert_not_called()
            self.assertEqual(comparison_path.read_text(encoding="utf-8"),
                             "existing\n")

    def test_comparison_rejects_the_legacy_solver(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self._comparison_args(directory)
            args.solver = "legacy"
            with patch.object(runner, "run") as run:
                with self.assertRaisesRegex(ValueError, "requires the spline solver"):
                    runner.compare_control_modes(
                        args, Path(directory) / "comparison.json")
            run.assert_not_called()
