"""Focused contracts for the diagnostic yaw-aware runner."""
import contextlib
import io
import json
import tempfile
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

from tools.run import run_yaw_aware_forward_driver as runner
from simplified_vehicle_models.planning.yaw_aware_line import YawLineSettings


class YawAwareForwardRunnerTests(unittest.TestCase):
    def test_parser_accepts_no_tv(self):
        with tempfile.TemporaryDirectory() as directory:
            args = runner.parse_args(["--track", "track.xml", "--output-dir", directory, "--map", "no-tv"])
        self.assertEqual(args.map, "no-tv")
        self.assertEqual(args.model, "planar-7dof")
        self.assertIsNone(args.maximum_speed)
        self.assertGreater(args.control_spacing, 0.)

    def test_parser_accepts_explicit_positive_speed_limiter(self):
        with tempfile.TemporaryDirectory() as directory:
            args = runner.parse_args(["--track", "track.xml", "--output-dir", directory,
                                     "--maximum-speed", "12.5"])
        self.assertEqual(args.maximum_speed, 12.5)

    def test_physical_speed_domain_uses_point_mass_drive_envelope(self):
        vehicle = object()
        with patch("simplified_vehicle_models.formulations.point_mass.maximum_vehicle_speed", return_value=20.), \
             patch("simplified_vehicle_models.formulations.point_mass.maximum_acceleration_limit",
                   side_effect=lambda _v, speed, _k: 1. if speed < 13.25 else 0.):
            limit = runner.physical_map_speed_limit(vehicle)
        self.assertTrue(__import__("math").isfinite(limit))
        self.assertAlmostEqual(limit, 13.25, places=6)

    def test_prepare_output_refuses_existing_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileExistsError):
                runner.prepare_output(directory)

    def test_parser_rejects_vertical_with_clear_planar_only_error(self):
        stderr = io.StringIO()
        with tempfile.TemporaryDirectory() as directory, self.assertRaises(SystemExit), contextlib.redirect_stderr(stderr):
            runner.parse_args(["--track", "track.xml", "--output-dir", directory,
                               "--model", "vertical-14dof"])
        self.assertIn("planar only", stderr.getvalue())

    def test_record_metadata_contains_reproducibility_sections(self):
        with tempfile.TemporaryDirectory() as directory:
            track_file = Path(directory) / "track.xml"
            track_file.write_text('<track><data ref="track.csv"/><data role="width" ref="width.csv"/></track>')
            data_file = Path(directory) / "track.csv"
            data_file.write_text("station,x\\n")
            (Path(directory) / "width.csv").write_text("station,width\\n")
            args = runner.parse_args(["--track", str(track_file), "--output-dir", str(Path(directory) / "out")])
            result = SimpleNamespace(lap_completed=False, time=[0.], wall_seconds=.2)
            capability = SimpleNamespace(source_kind="physical", source_identity="vehicle", yaw_inertia=3.,
                                         speed=__import__("numpy").array([0., 1.]),
                                         longitudinal_acceleration=__import__("numpy").array([-1., 1.]),
                                         lateral_acceleration=__import__("numpy").array([-2., 2.]))
            plan = SimpleNamespace(status="ok", settings=None, shape_metrics={"length_m": 10.},
                                   profile=SimpleNamespace(lap_time=2.))
            metadata = runner.record_metadata(args, SimpleNamespace(), SimpleNamespace(name="test"),
                                              capability, plan, result)
        for section in ("producer", "model", "sources", "track", "yaw_capability", "planning", "integrator", "segment"):
            self.assertTrue(metadata[section], section)
        self.assertIn("line.csv", metadata["track"]["optimized_path_identity"])
        self.assertEqual(len(metadata["sources"]), 4)
        self.assertTrue(any("width.csv" in path for path in metadata["sources"]))

    def test_run_record_close_failure_writes_failed_summary_without_record(self):
        with tempfile.TemporaryDirectory() as directory:
            track_file = Path(directory) / "track.xml"
            track_file.write_text('<track><data file="track.csv"/></track>')
            (Path(directory) / "track.csv").write_text("station,x\\n")
            args = SimpleNamespace(output_dir=Path(directory) / "run", record=True)
            result = SimpleNamespace(lap_completed=False, time=[0.], wall_seconds=.2)
            with \
                 patch("simplified_vehicle_models.compiler.compile.compile_formulation", return_value=SimpleNamespace(model=SimpleNamespace(yaw_inertia=3., wheelbase=2., steering_maximum_angle=.5, steering_maximum_rate=.5, maximum_pedal_force=1.))), \
                 patch("simplified_vehicle_models.tracks.profile.load_track", return_value=SimpleNamespace(path_coordinates=object(), has_edges=True)), \
                 patch("simplified_vehicle_models.tracks.line_trajectory.ReferenceCurve"), \
                 patch("simplified_vehicle_models.tracks.corridor.corridor_from_track"), \
                 patch("simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar"), \
                 patch.object(runner, "physical_map_speed_limit", return_value=30.), \
                 patch("simplified_vehicle_models.formulations.yaw_capability.YawCapabilityGrid", return_value=SimpleNamespace(speed=__import__("numpy").array([0., 1.]), longitudinal_acceleration=__import__("numpy").array([-1., 1.]), lateral_acceleration=__import__("numpy").array([-2., 2.]))), \
                 patch("simplified_vehicle_models.formulations.yaw_capability.build_physical_yaw_map", return_value=SimpleNamespace(source_kind="physical", source_identity="test", yaw_inertia=3., speed=__import__("numpy").array([0., 1.]), longitudinal_acceleration=__import__("numpy").array([-1., 1.]), lateral_acceleration=__import__("numpy").array([-2., 2.]))), \
                 patch("simplified_vehicle_models.planning.yaw_aware_line.YawLineSettings", return_value=YawLineSettings()), \
                 patch("simplified_vehicle_models.planning.yaw_aware_line.optimize_yaw_aware_line", return_value=SimpleNamespace(status="ok", settings=None, shape_metrics={}, diagnostics={}, profile=SimpleNamespace(lap_time=1.), speed_plan=SimpleNamespace(speed=[1.]))), \
                 patch("simplified_vehicle_models.planning.yaw_aware_record_path.path_coordinates_from_yaw_line", return_value=SimpleNamespace(distance=[0.], reference_distance=[0.], x_north=[0.], y_east=[0.], heading=[0.], curvature=[0.])), \
                 patch("simplified_vehicle_models.solvers.planar_forward.PlanarForwardPlant"), \
                 patch("simplified_vehicle_models.solvers.forward_driver.PreviewDriver"), \
                 patch("simplified_vehicle_models.solvers.forward_driver.solve_forward", return_value=result), \
                 patch.object(runner, "make_record_sink", return_value=SimpleNamespace(close=unittest.mock.Mock(side_effect=RuntimeError("no accepted step was recorded")))):
                # Supply all arguments used after parsing without invoking argparse.
                for key, value in {"track": track_file, "model": "planar-7dof", "map": "no-tv",
                    "maximum_speed": 30., "yaw_speed_points": 2, "yaw_ax_points": 2, "yaw_ay_points": 2,
                    "lateral_margin": .15, "control_spacing": 60., "planning_grid_spacing": 8.,
                    "planner_iterations": 1, "planner_passes": 1, "preview_seconds": .1,
                    "heading_gain": 1., "lateral_gain": 1., "speed_gain": 1., "duration": 1.,
                    "wall_seconds": .5, "maximum_step": .02, "rtol": 1.e-6, "method": "Radau",
                    "record_precision": "float32"}.items():
                    setattr(args, key, value)
                with patch.object(runner.np, "savetxt"):
                    summary = runner.run(args)
            self.assertEqual(summary["status"], "failed")
            self.assertEqual(summary["failure_stage"], "record")
            self.assertFalse(summary["lap_completed"])
            self.assertFalse(summary["qualified"])
            self.assertIn("no accepted step", summary["exception"])
            self.assertTrue((args.output_dir / "summary.json").exists())
            self.assertFalse((args.output_dir / "run_record.npz").exists())

    def test_record_sink_uses_optimized_geometry_argument(self):
        line = object()
        plant = object()
        fake = object()
        with patch("simplified_vehicle_models.results.plant_channels.RunRecordSink",
                   return_value=fake) as sink_type:
            sink = runner.make_record_sink(plant, line, Path("record.npz"), "float64")
        self.assertIs(sink, fake)
        self.assertIs(sink_type.call_args.args[1], line)
        self.assertEqual(sink_type.call_args.kwargs["precision"], "float64")


if __name__ == "__main__":
    unittest.main()
