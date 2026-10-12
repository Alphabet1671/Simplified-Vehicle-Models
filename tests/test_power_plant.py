import math
import unittest
from simplified_vehicle_models.powertrain.power_plant import compile_electric_motor, maximum_motor_torque, calculate_available_voltage, solve_direct_current, evaluate_shaft_power
from simplified_vehicle_models.xml.loader import load_model

class PowerPlantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.parameters=compile_electric_motor(load_model("models/power-plants/amk_dd5_100c.xml"))
    def test_zero_speed_and_limit(self):
        p=self.parameters
        self.assertAlmostEqual(maximum_motor_torque(p,0,600/math.sqrt(3)),p.torque_values[0])
        self.assertEqual(p.maximum_motor_speed,18600)
        with self.assertRaises(ValueError): maximum_motor_torque(p,18601,300)
    def test_voltage_scaling(self):
        p=self.parameters; a,b=200,300
        t1=maximum_motor_torque(p,10000,a); t2=maximum_motor_torque(p,10000,b)
        self.assertGreater(t2,t1)
        self.assertAlmostEqual(maximum_motor_torque(p,10000,b), maximum_motor_torque(p,10000*a/b,a), delta=1e-9)
    def test_reference_basis_proof(self):
        p=self.parameters; available=600/math.sqrt(3)
        required=available
        self.assertLess(abs(math.sqrt(3)*required-600)/600,.05)
        result=solve_direct_current(p,16300,0,available)
        self.assertTrue(result.feasible)
    def test_reference_correction_and_efficiency(self):
        p=self.parameters; result=evaluate_shaft_power(p,1000,10,600)
        self.assertEqual(result.field_weakening_correction,0.0)
        for speed in (0,1000,10000,18000):
            for torque in (-10,0,10):
                out=evaluate_shaft_power(p,speed,torque,600)
                self.assertGreaterEqual(out.efficiency,0); self.assertLessEqual(out.efficiency,1)
    def test_infeasible_voltage_and_regen(self):
        p=self.parameters
        self.assertFalse(solve_direct_current(p,16300,1000,1).feasible)
        regen=evaluate_shaft_power(p,1000,-10,600)
        self.assertLess(abs(regen.bus_power),abs(1000*2*math.pi/60*-10))
    def test_map_range_rejection(self):
        with self.assertRaises(ValueError): evaluate_shaft_power(self.parameters,21000,1,600)

if __name__ == "__main__": unittest.main()
