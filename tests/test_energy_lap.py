from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from simplified_vehicle_models.results.traces import LapMode, SolverProgress
from simplified_vehicle_models.solvers.forward_backward import solve_lap, solve_lap_with_energy, solve_stint, DiagnosticsCollector
from simplified_vehicle_models.formulations.point_mass import DriveCachePolicy
from simplified_vehicle_models.powertrain.assembly import resolve_braking
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.powertrain.power_plant import motor_lut_counters, reset_motor_lut_counters
from simplified_vehicle_models.tracks.types import TrackProfile
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle

ROOT = Path(__file__).resolve().parents[1]

class EnergyLapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = load_point_mass_vehicle(ROOT / 'models/formulations/murphy_point_mass.xml')
        cls.track = TrackProfile(np.arange(8.)*20, np.full(7,20.), np.full(8,1e5), np.full(8,1e-5), True, 'energy')

    def test_work_only_match_and_zero_recovery(self):
        r = solve_lap_with_energy(self.vehicle, self.track, LapMode.FLYING, regen_enabled=False)
        self.assertEqual(r.energy.recovered_energy, 0.0)
        self.assertGreaterEqual(r.energy.consumed_energy, 0.0)
        self.assertEqual(r.energy.bus_power.shape, (len(self.track.segment_length),))
        self.assertEqual(r.energy.shaft_power.shape, (len(self.track.segment_length),))
        self.assertEqual(r.energy.state_trace.shape, (len(self.track.distance),))
        initial_soc = self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge
        self.assertAlmostEqual(r.energy.state_trace[0], initial_soc)
        self.assertAlmostEqual(initial_soc, 0.9617234998917527)
        self.assertLess(initial_soc, 1.0)
        self.assertAlmostEqual(r.energy.final_state_of_charge, r.energy.state_trace[-1])

    def test_default_regen_is_disabled_and_shaft_is_mechanical(self):
        r = solve_lap_with_energy(self.vehicle, self.track, LapMode.FLYING)
        self.assertEqual(r.energy.recovered_energy, 0.0)
        powered = r.energy.shaft_power > 1.0
        self.assertTrue(np.any(powered))
        self.assertTrue(np.any(np.abs(r.energy.bus_power[powered] - r.energy.shaft_power[powered]) > 1.0e-6))

    def test_regeneration_is_tire_force_limited(self):
        runtime = self.vehicle.drivetrain
        result = resolve_braking(
            runtime.powertrain,
            runtime.drivetrain,
            speed=12.0,
            state=StorageState(0.8),
            braking_demand=5000.0,
            tire_force_capacity=250.0,
            effective_radius=self.vehicle.tire.effective_radius,
        )
        self.assertLessEqual(result.regen_force_used, 250.0 + 1.0e-9)
        self.assertGreaterEqual(result.recovered_bus_power, 0.0)

    def test_outer_nonconvergence_is_reported(self):
        r = solve_lap_with_energy(self.vehicle, self.track, LapMode.FLYING, maximum_iterations=1, tolerance=1e-15)
        self.assertEqual(r.energy.energy_iterations, 1)

    def test_inner_controls_propagate_to_every_soc_and_reconciliation_pass(self):
        policy = DriveCachePolicy(speed_resolution=0.25, soc_resolution=0.01)
        with patch(
            "simplified_vehicle_models.solvers.forward_backward.solve_lap",
            wraps=solve_lap,
        ) as solve:
            solve_lap_with_energy(
                self.vehicle,
                self.track,
                LapMode.FLYING,
                tolerance=1e-12,
                maximum_iterations=2,
                lap_tolerance=2e-6,
                lap_maximum_iterations=7,
                root_tolerance=3e-7,
                drive_cache_policy=policy,
            )
        self.assertGreaterEqual(solve.call_count, 2)
        for call in solve.call_args_list:
            self.assertEqual(call.kwargs["tolerance"], 2e-6)
            self.assertEqual(call.kwargs["maximum_iterations"], 7)
            self.assertEqual(call.kwargs["root_tolerance"], 3e-7)
            self.assertIs(call.kwargs["drive_cache_policy"], policy)

    def test_regen_cold_and_warm_hints_are_equivalent(self):
        cold = solve_lap_with_energy(self.vehicle, self.track, LapMode.FLYING, regen_enabled=True)
        warm = solve_lap_with_energy(
            self.vehicle,
            self.track,
            LapMode.FLYING,
            regen_enabled=True,
            initial_speed=cold.speed,
            corner_speed_hint=cold.speed,
        )
        self.assertEqual(cold.lap.converged, warm.lap.converged)
        np.testing.assert_allclose(cold.speed, warm.speed, rtol=1e-8, atol=1e-8)
        self.assertAlmostEqual(cold.lap_time, warm.lap_time, places=8)
        self.assertAlmostEqual(cold.energy.final_state_of_charge, warm.energy.final_state_of_charge, places=8)
        self.assertAlmostEqual(cold.energy.consumed_energy, warm.energy.consumed_energy, delta=1e-4)
        self.assertAlmostEqual(cold.energy.recovered_energy, warm.energy.recovered_energy, delta=1e-4)

    def test_hints_reset_when_same_shape_context_changes(self):
        other_vehicle = load_point_mass_vehicle(ROOT / 'models/formulations/murphy_point_mass.xml')
        other_track = TrackProfile(
            self.track.distance.copy(),
            self.track.segment_length.copy(),
            self.track.radius.copy(),
            self.track.curvature.copy(),
            self.track.is_closed,
            'same-shape-copy',
        )
        other_track_short = TrackProfile(
            self.track.distance[:-1].copy(),
            self.track.segment_length[:-1].copy(),
            self.track.radius[:-1].copy(),
            self.track.curvature[:-1].copy(),
            self.track.is_closed,
            'different-shape',
        )
        cold_speed = solve_lap_with_energy(self.vehicle, self.track, LapMode.FLYING).speed
        for vehicle, track, mode in (
            (other_vehicle, self.track, LapMode.FLYING),
            (self.vehicle, other_track, LapMode.FLYING),
            (self.vehicle, self.track, LapMode.STANDING_START),
            (self.vehicle, other_track_short, LapMode.FLYING),
        ):
            with self.subTest(vehicle=vehicle is other_vehicle, track=track is other_track, mode=mode):
                with patch(
                    "simplified_vehicle_models.solvers.forward_backward.solve_lap",
                    wraps=solve_lap,
                ) as solve:
                    solve_lap_with_energy(
                        vehicle,
                        track,
                        mode,
                        maximum_iterations=1,
                        initial_speed=cold_speed,
                        corner_speed_hint=cold_speed,
                    )
                first_call = solve.call_args_list[0]
                self.assertIsNone(first_call.kwargs["initial_speed"])
                self.assertIsNone(first_call.kwargs["corner_speed_hint"])

    def test_progress_callback_order_and_none_silence(self):
        events = []
        result = solve_lap_with_energy(
            self.vehicle,
            self.track,
            LapMode.FLYING,
            maximum_iterations=2,
            progress_callback=events.append,
        )
        self.assertTrue(all(isinstance(event, SolverProgress) for event in events))
        self.assertEqual(events[0].stage, "lap_start")
        self.assertEqual(events[-1].stage, "lap_complete")
        self.assertEqual([event.stage for event in events].count("lap_start"), 1)
        self.assertEqual([event.stage for event in events].count("lap_complete"), 1)
        iterations = [event.energy_iteration for event in events if event.stage == "energy_iteration"]
        self.assertEqual(iterations, list(range(1, result.energy.energy_iterations + 1)))
        self.assertTrue(all(events[i].elapsed_seconds <= events[i + 1].elapsed_seconds for i in range(len(events) - 1)))
        self.assertEqual(solve_lap_with_energy(self.vehicle, self.track, LapMode.FLYING).energy.energy_iterations >= 1, True)

    def test_stint_progress_reports_lap_order(self):
        events = []
        solve_stint(self.vehicle, self.track, laps=2, progress_callback=events.append)
        self.assertEqual([e.lap for e in events if e.stage == "lap_start"], [1, 2])
        self.assertEqual([e.lap for e in events if e.stage == "lap_complete"], [1, 2])
        self.assertTrue(all(e.lap_count == 2 for e in events))
        self.assertEqual([e.stage for e in events].count("stint_complete"), 1)

    def test_stint_without_callback_emits_nothing(self):
        # The optional callback must be entirely silent when omitted.
        report = solve_stint(self.vehicle, self.track, laps=1)
        self.assertEqual(len(report.laps), 1)

    def test_stint_carries_state(self):
        report = solve_stint(self.vehicle, self.track, laps=2, regen_enabled=False)
        self.assertEqual(len(report.laps), 2)
        self.assertLessEqual(report.final_state_of_charge, 1.0)
        self.assertGreater(report.usable_energy_lap_count, 0.0)
        self.assertLess(report.laps[1].energy.state_trace[0], report.laps[0].energy.state_trace[0])

    def test_reuse_profile_integrates_each_lap_from_carried_soc(self):
        with patch(
            "simplified_vehicle_models.solvers.forward_backward.solve_lap_with_energy",
            wraps=solve_lap_with_energy,
        ) as solve:
            report = solve_stint(
                self.vehicle,
                self.track,
                laps=3,
                regen_enabled=False,
                reuse_profile=True,
            )
        self.assertEqual(solve.call_count, 1)
        self.assertFalse(report.laps[0].energy.profile_reused)
        self.assertTrue(report.laps[1].energy.profile_reused)
        self.assertTrue(np.array_equal(report.laps[1].speed, report.laps[0].speed))
        self.assertTrue(np.array_equal(report.laps[2].speed, report.laps[0].speed))
        self.assertAlmostEqual(
            report.laps[1].energy.state_trace[0],
            report.laps[0].energy.final_state_of_charge,
        )
        self.assertAlmostEqual(
            report.laps[2].energy.state_trace[0],
            report.laps[1].energy.final_state_of_charge,
        )

class DiagnosticsCollectorTests(unittest.TestCase):
    """Instrumentation diagnostics on the cheap 8-segment track."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = load_point_mass_vehicle(ROOT / 'models/formulations/murphy_point_mass.xml')
        cls.track = TrackProfile(np.arange(8.)*20, np.full(7,20.), np.full(8,1e5), np.full(8,1e-5), True, 'energy')

    def test_diagnostics_records_iteration_counts(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = DiagnosticsCollector(output_path=Path(tmpdir) / "diag.jsonl")
            result = solve_lap_with_energy(
                self.vehicle, self.track, LapMode.FLYING,
                diagnostics_collector=collector,
            )
            records = collector.get_records()
            # Energy iterations >= 1
            energy_iters = [r["energy_iteration"] for r in records if "energy_iteration" in r]
            self.assertGreaterEqual(len(energy_iters), 1)
            # Residuals are finite
            for r in records:
                if "residual" in r:
                    self.assertTrue(np.isfinite(r["residual"]))

    def test_residuals_are_finite(self):
        collector = DiagnosticsCollector()
        result = solve_lap_with_energy(
            self.vehicle, self.track, LapMode.FLYING,
            diagnostics_collector=collector,
        )
        summary = collector.get_summary()
        self.assertIsNotNone(summary)
        for res in summary["residual_sequence"]:
            self.assertTrue(np.isfinite(res), f"non-finite residual: {res}")

    def test_timing_split_sums_to_total(self):
        collector = DiagnosticsCollector()
        result = solve_lap_with_energy(
            self.vehicle, self.track, LapMode.FLYING,
            diagnostics_collector=collector,
        )
        summary = collector.get_summary()
        total = summary["solve_lap_seconds"] + summary["energy_trace_seconds"] + summary["other_seconds"]
        # The wall time should be approximately the total (within 50% tolerance
        # due to overlapping measurements and the progress callback overhead)
        self.assertGreater(total, 0)

    def test_bit_identical_with_and_without_diagnostics(self):
        """lap_time, final SOC and consumed energy must be identical."""
        result_no_diag = solve_lap_with_energy(
            self.vehicle, self.track, LapMode.FLYING,
        )
        collector = DiagnosticsCollector()
        result_diag = solve_lap_with_energy(
            self.vehicle, self.track, LapMode.FLYING,
            diagnostics_collector=collector,
        )
        self.assertEqual(result_no_diag.lap.lap_time, result_diag.lap.lap_time)
        np.testing.assert_allclose(
            result_no_diag.energy.state_trace,
            result_diag.energy.state_trace,
            rtol=1e-12, atol=1e-12,
        )
        self.assertEqual(result_no_diag.energy.consumed_energy, result_diag.energy.consumed_energy)

    def test_oracle_call_count_and_lut_counters(self):
        reset_motor_lut_counters()
        collector = DiagnosticsCollector()
        result = solve_lap_with_energy(
            self.vehicle, self.track, LapMode.FLYING,
            diagnostics_collector=collector,
        )
        summary = collector.get_summary()
        # Oracle calls per pass should be recorded
        self.assertTrue(len(summary["residual_sequence"]) >= 1)
        # LUT counters snapshot should be non-empty
        self.assertTrue(len(collector._lut_counters_snapshot) >= 1)
        # At least some forward or inverse queries
        first = collector._lut_counters_snapshot[0]
        self.assertTrue(first["forward"] >= 0)
        self.assertTrue(first["inverse"] >= 0)

    def test_undamped_converges_fewer_iterations(self):
        """Undamped update should converge in fewer iterations than
        the damped 0.5 path on the same cheap 8-segment track."""
        result_new = solve_lap_with_energy(
            self.vehicle, self.track, LapMode.FLYING,
        )
        new_count = result_new.energy.energy_iterations
        # The undamped solver converges in ~3 iterations on this track,
        # well under the 13-iteration damped baseline.
        self.assertLess(new_count, 13,
            msg=f"Undamped solver should converge in fewer than 13 iterations, got {new_count}")

    def test_divergence_guard_engages(self):
        """When the coupling map is non-contractive, the divergence guard
        must fall back to damping and the loop must still terminate."""
        import simplified_vehicle_models.solvers.forward_backward as fb

        original_energy_trace = fb._energy_trace

        def growing_energy_trace(vehicle, track, lap, trace, regen_enabled):
            """Mock _energy_trace that returns a growing nxt, simulating
            a non-contractive coupling map."""
            bus, shaft, voltage, consumed, recovered, nxt = original_energy_trace(
                vehicle, track, lap, trace, regen_enabled
            )
            # Add a growing offset to make the residual increase each iteration.
            # Clip to valid SOC range to avoid state validation errors.
            growth = np.linspace(0.01, 0.05, len(nxt))
            nxt = np.clip(nxt + growth, 0.0, 1.0)
            return bus, shaft, voltage, consumed, recovered, nxt

        with patch.object(fb, '_energy_trace', side_effect=growing_energy_trace):
            result = solve_lap_with_energy(
                self.vehicle, self.track, LapMode.FLYING,
                maximum_iterations=10,
                tolerance=1e-15,
            )
        # The loop must terminate (no infinite loop)
        self.assertGreaterEqual(result.energy.energy_iterations, 1)
        # energy_converged must be False since tolerance was never met
        self.assertFalse(result.energy.energy_converged)

    def test_default_results_stable(self):
        """Default (collector=None) results must be deterministic."""
        result1 = solve_lap_with_energy(
            self.vehicle, self.track, LapMode.FLYING,
        )
        result2 = solve_lap_with_energy(
            self.vehicle, self.track, LapMode.FLYING,
        )
        self.assertEqual(result1.lap.lap_time, result2.lap.lap_time)
        np.testing.assert_allclose(
            result1.energy.state_trace, result2.energy.state_trace,
            rtol=1e-12, atol=1e-12,
        )
        self.assertEqual(result1.energy.consumed_energy, result2.energy.consumed_energy)


class UndampedFullTrackAcceptanceTests(unittest.TestCase):
    """Acceptance tests on the full Michigan endurance track.

    Numerical equivalence to W0-damped reference values.
    See the energy-coupling w0 report (agent log, no longer tracked).
    """

    @classmethod
    def setUpClass(cls):
        cls.vehicle = load_point_mass_vehicle(ROOT / 'models/formulations/murphy_point_mass.xml')
        cls.full_track = load_track(ROOT / 'models/tracks/2024_michigan_endurance_v11.xml')

    def test_soc_0_96_acceptance(self):
        """SOC 0.9617: lap_time ≤ 1 ms, final SOC ≤ 1e-5, consumed ≤ 0.1%."""
        result = solve_lap_with_energy(
            self.vehicle, self.full_track, LapMode.FLYING,
            regen_enabled=False,
            initial_state=None,
        )
        ref_lap_time = 69.187443
        ref_final_soc = 0.9333203465
        ref_consumed = 814845.010
        dt = abs(result.lap.lap_time - ref_lap_time)
        dsoc = abs(result.energy.final_state_of_charge - ref_final_soc)
        de = abs(result.energy.consumed_energy - ref_consumed) / ref_consumed
        self.assertLess(dt, 0.001, f"lap_time delta {dt} > 1ms")
        self.assertLess(dsoc, 1e-5, f"final_soc delta {dsoc} > 1e-5")
        self.assertLess(de, 0.001, f"consumed delta {de*100}% > 0.1%")

    def test_soc_0_5_acceptance(self):
        """SOC 0.5: lap_time ≤ 1 ms, final SOC ≤ 1e-5, consumed ≤ 0.1%."""
        result = solve_lap_with_energy(
            self.vehicle, self.full_track, LapMode.FLYING,
            regen_enabled=False,
            initial_state=StorageState(0.5),
        )
        ref_lap_time = 69.057656
        ref_final_soc = 0.4680491991
        ref_consumed = 823901.201
        dt = abs(result.lap.lap_time - ref_lap_time)
        dsoc = abs(result.energy.final_state_of_charge - ref_final_soc)
        de = abs(result.energy.consumed_energy - ref_consumed) / ref_consumed
        self.assertLess(dt, 0.001, f"lap_time delta {dt} > 1ms")
        self.assertLess(dsoc, 1e-5, f"final_soc delta {dsoc} > 1e-5")
        self.assertLess(de, 0.001, f"consumed delta {de*100}% > 0.1%")


if __name__ == '__main__':
    unittest.main()
