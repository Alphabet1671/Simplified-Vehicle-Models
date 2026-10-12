import math
import unittest

import numpy as np

from simplified_vehicle_models.powertrain.energy_storage import (
    EnergyStorageParameters,
    StorageState,
    compile_battery_pack,
    evaluate_storage,
    integrate_storage,
    solve_current,
)
from simplified_vehicle_models.xml.loader import load_model


class EnergyStorageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parameters = compile_battery_pack(
            load_model("models/energy-storages/murphy_2024_battery.xml")
        )

    def test_descending_curve_is_reversed(self):
        parameters = self.parameters
        self.assertEqual(parameters.state_of_charge_axis[0], 0.0)
        self.assertEqual(parameters.state_of_charge_axis[-1], 1.0)
        self.assertAlmostEqual(parameters.open_circuit_voltage_axis[0], 3.117)

    def test_capacity_units_and_power_limits(self):
        parameters = self.parameters
        self.assertAlmostEqual(parameters.pack_capacity_ah, 14.146, places=3)
        self.assertAlmostEqual(
            parameters.pack_capacity_joules,
            14.146 * 3600.0,
            places=6,
        )
        limits = evaluate_storage(parameters, StorageState(1.0))
        self.assertAlmostEqual(limits.discharge_power_limit, 100495.0059, delta=1000)
        self.assertAlmostEqual(limits.charge_power_limit, -66179.4485, delta=1000)

    def test_source_resistance_is_positive_and_monotone_where_expected(self):
        resistances = self.parameters.resistance_axis
        self.assertTrue(all(value > 0.0 for value in resistances))
        self.assertGreater(resistances[0], resistances[-1])
        self.assertTrue(
            all(
                first <= second
                for first, second in zip(
                    self.parameters.state_of_charge_axis,
                    self.parameters.state_of_charge_axis[1:],
                )
            )
        )

    def test_round_trip_both_directions(self):
        parameters = self.parameters
        for state_of_charge in (0.0, 0.015, 0.5, 1.0):
            for power in (1000.0, -1000.0):
                state = StorageState(state_of_charge)
                current, feasible = solve_current(parameters, state, power)
                self.assertTrue(feasible)
                voltage = parameters.series_count * float(
                    np.interp(
                        state_of_charge,
                        parameters.state_of_charge_axis,
                        parameters.open_circuit_voltage_axis,
                    )
                )
                resistance = parameters.series_count * float(
                    np.interp(
                        state_of_charge,
                        parameters.state_of_charge_axis,
                        parameters.resistance_axis,
                    )
                )
                self.assertAlmostEqual(
                    voltage * current - current**2 * resistance,
                    power,
                    delta=1.0e-9,
                )

    def test_charge_conservation(self):
        initial = StorageState(0.5)
        charged = integrate_storage(self.parameters, initial, -1000.0, 10.0)
        discharged = integrate_storage(self.parameters, charged, 1000.0, 10.0)
        self.assertAlmostEqual(
            discharged.state_of_charge,
            initial.state_of_charge,
            places=5,
        )

    def test_power_limit_clamps_current_request(self):
        limits = evaluate_storage(self.parameters, StorageState(1.0))
        current, feasible = solve_current(
            self.parameters,
            StorageState(1.0),
            limits.discharge_power_limit * 2.0,
        )
        self.assertFalse(feasible)
        self.assertTrue(math.isfinite(current))
        self.assertLessEqual(current, self.parameters.discharge_current_limit)

    def test_discharge_limit_crosses_80_kw_at_low_soc(self):
        high = evaluate_storage(self.parameters, StorageState(1.0))
        low = evaluate_storage(self.parameters, StorageState(0.0))
        self.assertGreater(high.discharge_power_limit, 80000.0)
        self.assertLess(low.discharge_power_limit, 80000.0)

    def test_energy_and_invalid_inputs(self):
        limits = evaluate_storage(self.parameters, StorageState(0.5))
        self.assertGreater(limits.usable_energy, 0.0)
        with self.assertRaises(ValueError):
            evaluate_storage(self.parameters, StorageState(1.5))
        with self.assertRaises(ValueError):
            solve_current(self.parameters, StorageState(-0.1), 1.0)


if __name__ == "__main__":
    unittest.main()
