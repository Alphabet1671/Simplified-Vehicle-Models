import unittest
from dataclasses import replace

from simplified_vehicle_models.drivetrain.transmission import (
    compile_fixed_gear_transmission,
    motor_speed_from_wheel_speed,
    wheel_torque_from_motor_torque,
)
from simplified_vehicle_models.xml.loader import load_model


class TransmissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parameters = compile_fixed_gear_transmission(
            load_model("models/transmissions/murphy_2024_fixed_gear.xml")
        )

    def test_speed_conversion(self):
        self.assertAlmostEqual(motor_speed_from_wheel_speed(self.parameters, 100.0), 1090.0)

    def test_motoring_torque_conversion(self):
        torques = wheel_torque_from_motor_torque(self.parameters, 20.0)
        self.assertEqual(torques.tolist(), [51.775, 51.775, 51.775, 51.775])

    def test_regeneration_torque_conversion(self):
        torques = wheel_torque_from_motor_torque(self.parameters, -20.0)
        expected = -20.0 * 10.9 / 0.95 / 4.0
        for torque in torques:
            self.assertAlmostEqual(torque, expected)

    def test_equal_side_split_and_front_fraction(self):
        parameters = replace(self.parameters, front_torque_fraction=0.75)
        torques = wheel_torque_from_motor_torque(parameters, 20.0)
        self.assertAlmostEqual(torques[0], torques[1])
        self.assertAlmostEqual(torques[2], torques[3])
        self.assertAlmostEqual(torques[0] * 2.0 / (torques[2] * 2.0), 3.0)

    def test_invalid_parameters(self):
        model = load_model("models/transmissions/murphy_2024_fixed_gear.xml")
        for name, value in (
            ("gear_ratio", 0.0),
            ("gear_efficiency", 0.0),
            ("gear_efficiency", 1.1),
            ("front_torque_fraction", -0.1),
            ("front_torque_fraction", 1.1),
        ):
            parameters = dict(model.parameters)
            parameters[name] = value
            invalid = replace(model, parameters=parameters)
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                compile_fixed_gear_transmission(invalid)


if __name__ == "__main__":
    unittest.main()
