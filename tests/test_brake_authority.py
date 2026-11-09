import unittest
from dataclasses import replace

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.brake_authority import (
    BrakeAuthorityContext, BrakeAuthorityPredictor,
)
from simplified_vehicle_models.formulations.point_mass import maximum_deceleration_limit


class BrakeAuthorityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model

    def context(self, **kwargs):
        kwargs.setdefault("wheel_radii_m", (.29, .29, .29, .29))
        kwargs.setdefault("normal_loads_n", (3000., 3000., 3000., 3000.))
        return BrakeAuthorityContext(kwargs.pop("speed_mps", 15.), **kwargs)

    def test_hydraulic_limit_is_not_point_mass_tire_oracle(self):
        v = self.vehicle
        low = replace(v, maximum_pedal_force=.25*v.maximum_pedal_force)
        a = BrakeAuthorityPredictor(v).predict(self.context(), v.maximum_pedal_force).deceleration_mps2
        b = BrakeAuthorityPredictor(low).predict(self.context(), low.maximum_pedal_force).deceleration_mps2
        point_mass = compile_formulation("models/formulations/iem24_murphy_real_point_mass.xml").model
        pm_reference = maximum_deceleration_limit(point_mass, 15., 0.)
        self.assertLess(b, a)
        self.assertGreater(pm_reference, 0.)
        self.assertNotAlmostEqual(pm_reference, a, places=2)

    def test_monotonic_and_plateau_and_shortfall(self):
        p = BrakeAuthorityPredictor(self.vehicle)
        values = [p.predict(self.context(), f*self.vehicle.maximum_pedal_force).deceleration_mps2
                  for f in (0., .1, .2, .4, .8, 1.)]
        self.assertTrue(np.all(np.diff(values) >= -1e-10))
        request = p.request(self.context(), 4.0)
        self.assertGreater(request.pedal_force_n, 0.)
        self.assertAlmostEqual(request.deceleration_mps2, 4.0, delta=.05)
        result = p.request(self.context(), values[-1]+5.)
        self.assertFalse(result.in_domain is False)
        self.assertGreater(result.shortfall_mps2, 0.)
        self.assertEqual(result.limiting_constraint, "maximum_pedal_force")
        self.assertFalse(result.wheel_torques_nm.flags.writeable)

    def test_invalid_required_context_is_validated_before_math(self):
        p = BrakeAuthorityPredictor(self.vehicle)
        with self.assertRaisesRegex(ValueError, "explicit loaded"):
            p.request(BrakeAuthorityContext(15.), 3.)
        with self.assertRaises(ValueError):
            p.request(self.context(wheel_radii_m=(0., .29, .29, .29)), 3.)

    def test_regen_unsupported_and_domain_flags(self):
        with self.assertRaisesRegex(ValueError, "regenerative"):
            BrakeAuthorityPredictor(replace(self.vehicle,
                controller=replace(self.vehicle.controller, regen_enabled=True,
                                regen_pedal_force_limit=1., regen_motor_torque_limit=1.)))
        p = BrakeAuthorityPredictor(self.vehicle, maximum_abs_curvature_inv_m=.05)
        for ctx in (self.context(speed_mps=0.5), self.context(curvature_inv_m=.06),
                    self.context(normal_loads_n=(3000., 0., 3000., 3000.))):
            self.assertFalse(p.predict(ctx, 100.).in_domain)
        with self.assertRaises(ValueError):
            p.predict(self.context(), -1.)
        for bad_target in (-1., float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                p.predict(self.context(), 10., bad_target)
        with self.assertRaises(ValueError):
            p.predict(self.context(regen_enabled=True), 10.)

    def test_signed_curvature_and_contact_capacity(self):
        p = BrakeAuthorityPredictor(self.vehicle, maximum_abs_curvature_inv_m=.1)
        left = p.predict(self.context(curvature_inv_m=.05), 250.)
        right = p.predict(self.context(curvature_inv_m=-.05), 250.)
        self.assertTrue(left.in_domain and right.in_domain)
        np.testing.assert_allclose(left.wheel_torques_nm, right.wheel_torques_nm)
        limited = p.predict(self.context(longitudinal_capacity_n=(10., 10., 10., 10.)), 250., 8.)
        self.assertEqual(limited.limiting_constraint, "tire_contact_capacity")
        self.assertTrue(limited.in_domain)
        self.assertGreater(limited.shortfall_mps2, 0.)
        self.assertLess(np.max(-limited.wheel_torques_nm), 10. * .29 + 1e-9)
        self.assertLess(limited.deceleration_mps2, 8.)
        self.assertLess(limited.pedal_force_n, 250.)
        replay = p.predict(self.context(), limited.pedal_force_n)
        np.testing.assert_allclose(replay.wheel_torques_nm, limited.wheel_torques_nm, rtol=0, atol=1e-8)

    def test_nonlinear_calibration_uses_exact_forward_map_and_prefix_replay(self):
        from simplified_vehicle_models.controls.brake_inverse import _map
        from simplified_vehicle_models.solvers.driver_maps import MonotoneTable1D, PedalActuatorCalibration
        table = MonotoneTable1D((0., .25, .75, 1.), (0., .5, .5, 1.), "1", "1")
        calibration = PedalActuatorCalibration(table, table)
        vehicle = replace(self.vehicle, maximum_brake_pressure=1.0e9,
                          controller=replace(self.vehicle.controller, actuator_calibration=calibration))
        p = BrakeAuthorityPredictor(vehicle)
        for fraction in (.501, .502, .75):
            force = vehicle.maximum_pedal_force * fraction
            prediction = p.predict(self.context(), force)
            expected = _map(vehicle, force).wheel_torques
            np.testing.assert_allclose(prediction.wheel_torques_nm, expected, rtol=0, atol=1e-8)
        cap = self.context(longitudinal_capacity_n=(20., 20., 20., 20.))
        limited = p.predict(cap, force, 20.)
        replay = _map(vehicle, limited.pedal_force_n).wheel_torques
        np.testing.assert_allclose(limited.wheel_torques_nm, replay, rtol=0, atol=1e-8)
        limited_reference = p.predict(self.context(), force)
        capped_reference = np.asarray(limited_reference.wheel_torques_nm).copy()
        caps = np.asarray(cap.longitudinal_capacity_n) * np.asarray(cap.wheel_radii_m)
        prefix = min(1.0, float(np.min(caps / -capped_reference)))
        np.testing.assert_allclose(limited.wheel_torques_nm, prefix*capped_reference, rtol=0, atol=1e-8)

    def test_constructor_rejects_unsupported_force_ratio_curve(self):
        from dataclasses import replace
        from simplified_vehicle_models.controls.brakes import PedalRatioCurve
        curve = PedalRatioCurve(np.array([0., 100., 200.]), np.array([10., 1., .1]))
        with self.assertRaisesRegex(ValueError, "not monotone"):
            BrakeAuthorityPredictor(replace(self.vehicle, pedal_curve=curve))

    def test_context_and_output_are_immutable_and_validate(self):
        with self.assertRaisesRegex(ValueError, "explicit loaded"):
            BrakeAuthorityPredictor(self.vehicle).predict(BrakeAuthorityContext(15.), 10.)
        for bad in (
            dict(wheel_radii_m=(0., .29, .29, .29)),
            dict(normal_loads_n=(3000., -1., 3000., 3000.)),
            dict(longitudinal_capacity_n=(1., -1., 1., 1.)),
            dict(soc=1.01),
            dict(speed_mps=0.),
        ):
            with self.assertRaises(ValueError):
                self.context(**bad)
        context = self.context()
        with self.assertRaises(Exception):
            context.speed_mps = 20.
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                BrakeAuthorityPredictor(self.vehicle).predict(self.context(), bad)


if __name__ == "__main__":
    unittest.main()
