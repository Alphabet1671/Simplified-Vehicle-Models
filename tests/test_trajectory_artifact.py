"""Round-trip and fail-closed tests for serialized analytic trajectories."""
from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.trajectory_artifact import (
    ExactTrajectoryDriverReference, load_trajectory_artifact,
)
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from tools.run.solve_minimum_time_line import _write_trajectory_metadata
from simplified_vehicle_models.solvers.qss_spline_ocp import (
    _SplineStateEvaluator, TIME_QUADRATURE_NAME, _time_quadrature_metadata,
)
from simplified_vehicle_models.solvers.forward_driver import (
    PreviewDriver, DriverObservation, SpeedPlan,
)


class TrajectoryArtifactTest(unittest.TestCase):
    def make_solution(self, closed=False, control="quadratic_acceleration_v1", fit_periodic=None):
        n = 41
        s = np.linspace(0., 400., n)
        angle = np.linspace(0, 2*np.pi, n)
        north = 100*np.cos(angle)
        east = 100*np.sin(angle)
        if not closed:
            north = s * 0.7
            east = 20*np.sin(s/80.)
            if fit_periodic:
                north = 100*np.cos(angle)
                east = 100*np.sin(angle)
                north[-1], east[-1] = north[0], east[0]
        else:
            north[-1], east[-1] = north[0], east[0]
        track = SimpleNamespace(path_coordinates=SimpleNamespace(distance=s, x_north=north,
                            y_east=east, is_closed=closed), lap_length=400., is_closed=closed)
        reference = ReferenceCurve(track, interpolation_periodic=fit_periodic)
        spline = OffsetSpline(track, 35.)
        coeff = 2*np.sin(np.linspace(0, 2*np.pi, spline.control_count, endpoint=not closed))
        if closed:
            coeff[-1] = coeff[0]
        trajectory = OffsetTrajectory(reference, spline, coeff)
        mesh = np.linspace(s[0], s[-1], 21)
        speed = 12 + 2*np.sin(np.linspace(0, np.pi, mesh.size))
        acc = np.linspace(-1.5, 1.5, mesh.size)
        mid = np.linspace(2., -2., mesh.size-1)
        evaluator = _SplineStateEvaluator(trajectory, mesh, speed, acc, mid,
                                          control_representation=control)
        solution = SimpleNamespace(trajectory=trajectory, midpoint_acceleration=mid,
            coefficients=coeff, distance=mesh, speed=speed, acceleration=acc,
            state_representation="energy_hs_v1", control_representation=control,
            time_quadrature=TIME_QUADRATURE_NAME,
            time_quadrature_metadata=_time_quadrature_metadata())
        return solution, evaluator

    def roundtrip(self, closed=False, control="quadratic_acceleration_v1"):
        solution, evaluator = self.make_solution(closed, control)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/"trajectory.npz"
            _write_trajectory_metadata(path, solution)
            loaded = load_trajectory_artifact(path)
            self.assertEqual(loaded.source_sha256, __import__('hashlib').sha256(path.read_bytes()).hexdigest())
            self.assertFalse(loaded.accepted)
            self.assertFalse(loaded.physical_validated)
            self.assertGreater(loaded.physical_time, 0.0)
            from simplified_vehicle_models.solvers.qss_spline_ocp import physical_time_quadrature
            self.assertAlmostEqual(
                loaded.physical_time,
                physical_time_quadrature(solution.trajectory, evaluator, solution.distance),
                places=12,
            )
            self.assertFalse(loaded.trajectory.coefficients.flags.writeable)
            with self.assertRaises(TypeError):
                loaded.metadata["is_closed"] = not closed
            queries = np.linspace(solution.trajectory.reference.start,
                                  solution.trajectory.reference.end, 13)
            got = loaded.evaluate_reference(queries)
            want_g = solution.trajectory.evaluate(queries)
            want_s = evaluator.evaluate(queries)
            np.testing.assert_allclose(got["north"], want_g.north, atol=1e-12)
            np.testing.assert_allclose(got["east"], want_g.east, atol=1e-12)
            np.testing.assert_allclose(got["speed"], want_s["speed"], atol=1e-12)
            np.testing.assert_allclose(got["acceleration"], want_s["acceleration"], atol=1e-12)
            driver = ExactTrajectoryDriverReference(loaded)
            reference_speed_plan = SpeedPlan(
                driver.distance, driver.speed, driver.curvature, is_closed=closed)
            distinct_acceleration_witness = False
            for d in np.linspace(0., driver.length, 9, endpoint=not closed):
                path_station = (solution.trajectory.station_map.path_end if d == driver.length
                                else d + solution.trajectory.station_map.path_start)
                src = solution.trajectory.station_map.to_reference(path_station)
                exp = evaluator.evaluate(src)
                actual = driver.sample(d)
                self.assertAlmostEqual(actual[0], exp["speed"], places=12)
                self.assertAlmostEqual(actual[2], exp["acceleration"], places=12)
                if 0.0 < d < driver.length and not np.isclose(
                        actual[2], reference_speed_plan.sample(d)[2], atol=1e-4):
                    distinct_acceleration_witness = True
            self.assertTrue(distinct_acceleration_witness)
            # PreviewDriver consumes the exact bridge as a plan-like signal; this
            # CPU-only call verifies feedforward comes from physical evaluator a.
            preview = PreviewDriver(driver, 2.5, .6, 2.0, 2000.0, preview_seconds=0.0)
            probe_distance = float(driver.distance[len(driver.distance) // 2])
            target_speed, _, exact_feedforward = driver.sample(probe_distance)
            obs = DriverObservation(probe_distance, 0.0, 0.0, target_speed, 0.0, 0.0)
            command = preview.command(obs)
            expected_throttle = np.clip(exact_feedforward / 6., 0., 1.)
            expected_brake = np.clip(-exact_feedforward / 6., 0., 1.)
            self.assertAlmostEqual(command.throttle, expected_throttle, places=12)
            self.assertAlmostEqual(command.brake_force / preview.maximum_brake_force,
                                   expected_brake, places=12)
            if not closed:
                with self.assertRaises(ValueError):
                    driver.sample(-1.)
            else:
                end_expected = evaluator.evaluate(solution.trajectory.reference.end)["acceleration"]
                self.assertAlmostEqual(driver.sample(driver.length)[2], end_expected, places=12)
                np.testing.assert_allclose(driver.sample(driver.length + 1.), driver.sample(1.), atol=1e-10)

    def test_open_roundtrip_both_control_representations(self):
        for control in ("quadratic_acceleration_v1", "quadratic_energy_rate_over_metric_v1"):
            with self.subTest(control=control):
                self.roundtrip(False, control)

    def test_closed_roundtrip(self):
        self.roundtrip(True)
    def test_writer_legacy_topology_keeps_schema1(self):
        for closed in (False, True):
            solution, _ = self.make_solution(closed)
            with tempfile.TemporaryDirectory() as td:
                path = Path(td) / "legacy.npz"
                _write_trajectory_metadata(path, solution)
                with np.load(path, allow_pickle=False) as archive:
                    meta = json.loads(str(archive["metadata"].item()))
                self.assertEqual(meta["schema_version"], 1)
                self.assertNotIn("source_interpolation_periodic", meta)

    def test_open_domain_periodic_fit_roundtrip_preserves_geometry(self):
        from dataclasses import replace
        track = load_track("tracks_out/fsc_may_2026.xml")
        opened = replace(track, is_closed=False,
                         path_coordinates=replace(track.path_coordinates, is_closed=False))
        reference = ReferenceCurve(opened, interpolation_periodic=True)
        spline = OffsetSpline(opened, 35.)
        coeff = np.linspace(-0.00437, 0.00437, spline.control_count)
        trajectory = OffsetTrajectory(reference, spline, coeff)
        mesh = np.linspace(reference.start, reference.end, 41)
        speed = np.full(mesh.size, 18.)
        acc = np.zeros(mesh.size)
        mid = np.zeros(mesh.size - 1)
        evaluator = _SplineStateEvaluator(trajectory, mesh, speed, acc, mid)
        solution = SimpleNamespace(trajectory=trajectory, midpoint_acceleration=mid,
            coefficients=coeff, distance=mesh, speed=speed, acceleration=acc,
            state_representation="energy_hs_v1", control_representation="quadratic_acceleration_v1",
            time_quadrature=TIME_QUADRATURE_NAME, time_quadrature_metadata=_time_quadrature_metadata())
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "periodic-fit-open.npz"
            _write_trajectory_metadata(path, solution)
            loaded = load_trajectory_artifact(path)
            self.assertEqual(loaded.metadata["schema_version"], 2)
            self.assertIs(loaded.metadata["source_interpolation_periodic"], True)
            self.assertFalse(loaded.trajectory.reference.is_closed)
            self.assertTrue(loaded.trajectory.reference.interpolation_periodic)
            queries = np.linspace(solution.trajectory.reference.start,
                                  solution.trajectory.reference.end, 1001)
            got = loaded.trajectory.reference.evaluate(queries)
            want = solution.trajectory.reference.evaluate(queries)
            for field in ("north", "east", "north_s", "east_s", "north_ss", "east_ss",
                          "north_sss", "east_sss", "metric", "metric_s", "curvature", "curvature_s"):
                np.testing.assert_allclose(getattr(got, field), getattr(want, field), atol=1e-12)
            self.assertTrue(np.any(np.abs(solution.coefficients) > 0.004))
            self.assertGreater(np.max(np.abs(loaded.trajectory.coefficients)), 0.004)
            with self.assertRaises(ValueError):
                loaded.trajectory.reference.evaluate(solution.trajectory.reference.start - 1e-6)
            with self.assertRaises(ValueError):
                loaded.trajectory.reference.evaluate(solution.trajectory.reference.end + 1e-6)

    def test_schema2_requires_typed_fit_flag_and_schema1_rejects_conflict(self):
        solution, _ = self.make_solution(False, fit_periodic=True)
        with tempfile.TemporaryDirectory() as td:
            good = Path(td) / "good.npz"
            _write_trajectory_metadata(good, solution)
            with np.load(good, allow_pickle=False) as z:
                pristine = {k: np.array(z[k], copy=True) for k in z.files}
            bad = Path(td) / "bad.npz"
            for value in (None, "true", 1):
                data = {k: np.array(v, copy=True) for k, v in pristine.items()}
                meta = json.loads(str(data["metadata"].item()))
                if value is None:
                    del meta["source_interpolation_periodic"]
                else:
                    meta["source_interpolation_periodic"] = value
                data["metadata"] = np.array(json.dumps(meta))
                np.savez_compressed(bad, **data)
                with self.assertRaisesRegex(ValueError, "source_interpolation_periodic"):
                    load_trajectory_artifact(bad)
            data = {k: np.array(v, copy=True) for k, v in pristine.items()}
            meta = json.loads(str(data["metadata"].item()))
            meta["schema_version"] = 1
            data["metadata"] = np.array(json.dumps(meta))
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "conflicts"):
                load_trajectory_artifact(bad)
            for version in (3, True):
                meta["schema_version"] = version
                data["metadata"] = np.array(json.dumps(meta))
                np.savez_compressed(bad, **data)
                with self.assertRaisesRegex(ValueError, "unknown trajectory artifact schema"):
                    load_trajectory_artifact(bad)
            meta["schema_version"] = 2
            meta["is_closed"] = True
            meta["source_interpolation_periodic"] = False
            data["metadata"] = np.array(json.dumps(meta))
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "invalid source interpolation topology"):
                load_trajectory_artifact(bad)

    def test_rejects_malformed_artifacts_table_driven(self):
        solution, _ = self.make_solution()
        with tempfile.TemporaryDirectory() as td:
            original = Path(td) / "original.npz"
            _write_trajectory_metadata(original, solution)
            with np.load(original, allow_pickle=False) as archive:
                pristine = {key: np.array(archive[key], copy=True) for key in archive.files}

            cases = (
                ("source_xy", "stored map", lambda data, meta: data["reference_north"].__setitem__(10, data["reference_north"][10] + .1)),
                ("control_grid", "control grid", lambda data, meta: data["control_station"].__setitem__(2, data["control_station"][2] + .1)),
                ("bad_node_length", "state mesh shapes", lambda data, meta: data.__setitem__("node_speed", data["node_speed"][:-1])),
                ("nonfinite_source", "finite one-dimensional", lambda data, meta: data["reference_north"].__setitem__(4, np.nan)),
                ("nonincreasing_source", "source reference grid", lambda data, meta: data["source_reference_station"].__setitem__(4, data["source_reference_station"][3])),
                ("nonfinite_map", "finite one-dimensional", lambda data, meta: data["path_station"].__setitem__(3, np.inf)),
                ("node_coverage", "state mesh must cover", lambda data, meta: data["node_station"].__setitem__(0, data["node_station"][0] + .1)),
                ("missing_state", "state representation", lambda data, meta: meta.pop("state_representation")),
                ("missing_time", "time quadrature", lambda data, meta: meta.pop("time_quadrature")),
                ("missing_time_metadata", "time quadrature metadata", lambda data, meta: meta.pop("time_quadrature_metadata")),
            )
            bad = Path(td) / "bad.npz"
            for name, message, mutate in cases:
                with self.subTest(case=name):
                    data = {key: np.array(value, copy=True) for key, value in pristine.items()}
                    metadata = json.loads(str(data["metadata"].item()))
                    mutate(data, metadata)
                    data["metadata"] = np.array(json.dumps(metadata))
                    np.savez_compressed(bad, **data)
                    with self.assertRaisesRegex(ValueError, message):
                        load_trajectory_artifact(bad)

    def test_rejects_tampered_map_and_unknown_representations(self):
        solution, _ = self.make_solution()
        with tempfile.TemporaryDirectory() as td:
            original = Path(td)/"original.npz"
            _write_trajectory_metadata(original, solution)
            with np.load(original, allow_pickle=False) as z:
                data = {k: np.array(z[k], copy=True) for k in z.files}
            data["path_station"][2] += .01
            bad = Path(td)/"bad.npz"
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "stored path map"):
                load_trajectory_artifact(bad)
            data["path_station"] = np.array(np.load(original)["path_station"])
            meta = json.loads(str(data["metadata"].item()))
            meta["state_representation"] = "mystery"
            data["metadata"] = np.array(json.dumps(meta))
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "state representation"):
                load_trajectory_artifact(bad)
            meta["schema_version"] = True
            data["metadata"] = np.array(json.dumps(meta))
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "schema"):
                load_trajectory_artifact(bad)
            meta["schema_version"] = 1
            meta["state_representation"] = "energy_hs_v1"
            meta["time_quadrature"] = "unknown"
            data["metadata"] = np.array(json.dumps(meta))
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "time quadrature"):
                load_trajectory_artifact(bad)
            meta["time_quadrature"] = TIME_QUADRATURE_NAME
            meta.pop("control_representation", None)
            data["metadata"] = np.array(json.dumps(meta))
            np.savez_compressed(bad, **data)
            legacy = load_trajectory_artifact(bad)
            self.assertEqual(legacy.control_repr, "quadratic_acceleration_v1")
            meta["control_representation"] = "quadratic_acceleration_v1"
            data["metadata"] = np.array(json.dumps(meta))
            data["coefficients"][0] += 0.5
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "stored path map"):
                load_trajectory_artifact(bad)
            data["coefficients"][0] -= 0.5
            meta["control_representation"] = "unknown_control"
            data["metadata"] = np.array(json.dumps(meta))
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "control representation"):
                load_trajectory_artifact(bad)
            meta["control_representation"] = "quadratic_acceleration_v1"
            data["metadata"] = np.array(json.dumps(meta))
            data["node_speed"][3] = np.nan
            np.savez_compressed(bad, **data)
            with self.assertRaisesRegex(ValueError, "finite one-dimensional"):
                load_trajectory_artifact(bad)


if __name__ == "__main__":
    unittest.main()
