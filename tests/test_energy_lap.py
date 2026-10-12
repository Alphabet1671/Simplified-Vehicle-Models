from pathlib import Path
import unittest
import numpy as np
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap_with_energy, solve_stint
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

    def test_outer_nonconvergence_is_reported(self):
        r = solve_lap_with_energy(self.vehicle, self.track, LapMode.FLYING, maximum_iterations=1, tolerance=1e-15)
        self.assertEqual(r.energy.energy_iterations, 1)

    def test_stint_carries_state(self):
        report = solve_stint(self.vehicle, self.track, laps=2, regen_enabled=False)
        self.assertEqual(len(report.laps), 2)
        self.assertLessEqual(report.final_state_of_charge, 1.0)
        self.assertGreater(report.usable_energy_lap_count, 0.0)
        self.assertLess(report.laps[1].energy.state_trace[0], report.laps[0].energy.state_trace[0])

if __name__ == '__main__':
    unittest.main()
