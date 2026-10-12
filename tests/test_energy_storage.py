import math
import unittest
import numpy as np
from simplified_vehicle_models.powertrain.energy_storage import compile_battery_pack, evaluate_storage, storage_state, solve_current, integrate_storage
from simplified_vehicle_models.xml.loader import load_model

class EnergyStorageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parameters=compile_battery_pack(load_model("models/energy-storages/murphy_2024_battery.xml"))
    def test_descending_curve_is_reversed(self):
        self.assertEqual(self.parameters.state_of_charge_axis[0],0.0)
        self.assertEqual(self.parameters.state_of_charge_axis[-1],1.0)
        self.assertAlmostEqual(self.parameters.open_circuit_voltage_axis[0],3.117)
    def test_power_anchors_and_capacity(self):
        self.assertAlmostEqual(self.parameters.cell_capacity,14.146,places=3)
        limits=evaluate_storage(self.parameters,storage_state(1.0))
        self.assertAlmostEqual(limits.discharge_power_limit,100495.0059,delta=1000)
        self.assertAlmostEqual(limits.charge_power_limit,-66179.4485,delta=1000)
    def test_round_trip_both_directions(self):
        for soc in (0,.015,.5,1):
            for power in (1000,-1000):
                state=storage_state(soc); current,feasible=solve_current(self.parameters,state,power)
                self.assertTrue(feasible)
                voc=self.parameters.series_count*float(np.interp(soc,self.parameters.state_of_charge_axis,self.parameters.open_circuit_voltage_axis))
                resistance=self.parameters.series_count*float(np.interp(soc,self.parameters.state_of_charge_axis,self.parameters.resistance_axis))
                self.assertAlmostEqual(voc*current-current*current*resistance,power,delta=1e-9)
    def test_charge_conservation(self):
        initial=storage_state(.5); charged=integrate_storage(self.parameters,initial,-1000,10)
        discharged=integrate_storage(self.parameters,charged,1000,10)
        self.assertAlmostEqual(discharged.state_of_charge,initial.state_of_charge,places=5)
    def test_energy_and_invalid_inputs(self):
        limits=evaluate_storage(self.parameters,storage_state(.5))
        self.assertGreater(limits.usable_energy,0); self.assertLess(limits.usable_energy, self.parameters.pack_capacity*4.227)
        with self.assertRaises(ValueError): evaluate_storage(self.parameters,storage_state(1.5))
        with self.assertRaises(ValueError): solve_current(self.parameters,storage_state(-.1),1)
    def test_infeasible_discharge_clamps_without_nan(self):
        current,feasible=solve_current(self.parameters,storage_state(1),1e9)
        self.assertFalse(feasible); self.assertTrue(math.isfinite(current))

if __name__ == "__main__": unittest.main()
