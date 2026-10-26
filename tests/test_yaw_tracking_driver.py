import unittest
import numpy as np
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan,DriverObservation
from simplified_vehicle_models.solvers.yaw_tracking_driver import ModelAwareYawDriver


class TrackingTests(unittest.TestCase):
    def driver(self,**kw):
        args=dict(plan=SpeedPlan(np.array([0.,10.,20.]),np.full(3,10.),np.zeros(3)),wheelbase=2.,
            maximum_steering=.5,maximum_steering_rate=2.,maximum_brake_force=100.,mass=100.,
            force_speed=(0.,30.),drive_force=(1000.,1000.),drag_force=(100.,100.),
            brake_pedal=(0.,100.),brake_wheel_force=(0.,2000.))
        args.update(kw);return ModelAwareYawDriver(**args)

    def obs(self,**kw):
        args=dict(distance=5.,lateral_error=0.,heading_error=0.,speed=10.,yaw_rate=0.,steering=0.)
        args.update(kw);return DriverObservation(**args)

    def test_drag_compensation_at_target_speed(self):
        command=self.driver().command(self.obs())
        self.assertAlmostEqual(command.throttle,.1);self.assertEqual(command.brake_force,0.)

    def test_acceleration_force_scaling_saturates_without_brake_overlap(self):
        command=self.driver().command(self.obs(speed=0.))
        self.assertEqual(command.throttle,1.);self.assertEqual(command.brake_force,0.)

    def test_braking_inverts_force_table_and_governor_remains(self):
        command=self.driver().command(self.obs(speed=15.))
        self.assertEqual(command.throttle,0.);self.assertAlmostEqual(command.brake_force,45.)
        locked=self.driver().command(self.obs(speed=15.,wheel_speed_margin=0.))
        self.assertEqual(locked.brake_force,0.)

    def test_lateral_error_correction_and_mirror_symmetry(self):
        a=self.driver().command(self.obs(lateral_error=.2))
        b=self.driver().command(self.obs(lateral_error=-.2))
        self.assertLess(a.steering_rate,0.)
        self.assertAlmostEqual(a.steering_rate,-b.steering_rate)

    def test_seam_commands_are_periodic(self):
        a=self.driver().command(self.obs(distance=.001))
        b=self.driver().command(self.obs(distance=20.001))
        self.assertEqual(a,b)

    def test_factory_uses_actual_vehicle_brake_and_motor_tables(self):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        vehicle=compile_formulation(str(root/'models/formulations/iem24_murphy_real_planar_7dof.xml')).model
        driver=ModelAwareYawDriver.for_vehicle(self.driver().plan,vehicle,point_mass_view_of_planar(vehicle))
        self.assertGreater(max(driver.drive_force),0.)
        self.assertGreater(max(driver.brake_wheel_force),0.)
        self.assertTrue(driver.has_pedal_dynamics)
        self.assertLessEqual(driver.command(self.obs()).throttle,1.)

    def test_nonmonotone_brake_table_rejected(self):
        with self.assertRaises(ValueError):self.driver(brake_wheel_force=(2.,1.))
