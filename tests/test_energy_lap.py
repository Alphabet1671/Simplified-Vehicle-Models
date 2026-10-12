from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap_with_energy, solve_stint
from simplified_vehicle_models.powertrain.assembly import resolve_braking
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.tracks.types import TrackProfile
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
        self.assertAlmostEqual(r.energy.state_trace[0], 1.0)
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

if __name__ == '__main__':
    unittest.main()
