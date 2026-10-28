import unittest
from pathlib import Path
import numpy as np
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.yaw_capability import (
    YawCapabilityGrid, YawCapabilityMap, TvLimits,
    build_physical_yaw_map, build_ideal_tv_yaw_map,
)

ROOT = Path(__file__).resolve().parents[1]

class YawMapTest(unittest.TestCase):
    def test_grid_order_bounds_and_immutable_copy(self):
        speeds = np.array([0., 10.])
        grid = YawCapabilityGrid(speeds, [-2., 1.], [-3., 0., 3.])
        speeds[0] = 99.
        self.assertEqual(grid.speed[0], 0.)
        self.assertFalse(grid.speed.flags.writeable)
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            YawCapabilityGrid([1., 0.], [0.], [0.])
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            YawCapabilityGrid([-1.], [0.], [0.])

    def test_map_shape_bounds_provenance_and_readonly(self):
        axes = ([1., 2.], [-1., 1.], [-2., 2.]); shape=(2,2,2); source=np.ones(shape)
        chart=YawCapabilityMap(*axes, source, source, np.ones(shape,bool), 100., "physical_no_tv_v1", "vehicle-hash")
        source[0,0,0]=10
        self.assertEqual(chart.yaw_moment_into_turn[0,0,0],1.)
        self.assertFalse(chart.feasible_mask.flags.writeable)
        self.assertFalse(chart.yaw_moment_into_turn.flags.writeable)
        with self.assertRaisesRegex(ValueError,"nonnegative"):
            YawCapabilityMap(*axes, -source, source, np.ones(shape,bool),100.,"physical_no_tv_v1","id")
        with self.assertRaisesRegex(ValueError,"provenance"):
            YawCapabilityMap(*axes, source, source, np.ones(shape,bool),100.,"unknown","id")

    def test_tv_limit_validation(self):
        with self.assertRaises(ValueError): TvLimits(float("nan"))
        with self.assertRaises(ValueError): TvLimits(-1.)
        with self.assertRaises(ValueError): TvLimits(0., max_hydraulic_force_difference=float("inf"))
        with self.assertRaises(ValueError): TvLimits(0., max_hydraulic_force_difference=-1.)

    def test_builder_variants_and_ideal_upper_bound(self):
        model=compile_formulation(str(ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")).model
        vehicle=point_mass_view_of_planar(model)
        grid=YawCapabilityGrid([5.], [0.], [-1.,0.,1.])
        inertia=model.yaw_inertia
        no_tv=build_physical_yaw_map(vehicle,grid,yaw_inertia=inertia)
        physical=build_physical_yaw_map(vehicle,grid,TvLimits(10.,True,True),yaw_inertia=inertia)
        ideal=build_ideal_tv_yaw_map(vehicle,grid,yaw_inertia=inertia)
        self.assertEqual(no_tv.source_kind,"physical_no_tv_v1")
        self.assertEqual(physical.source_kind,"physical_tv_v1")
        self.assertEqual(ideal.source_kind,"physical_ideal_tv_v1")
        self.assertEqual(no_tv.yaw_inertia,inertia)
        self.assertTrue(np.all(ideal.feasible_mask >= physical.feasible_mask))
        for mask in (no_tv.feasible_mask,physical.feasible_mask):
            common=mask & ideal.feasible_mask
            self.assertTrue(np.all(ideal.yaw_moment_into_turn[common] + 1e-5 >= no_tv.yaw_moment_into_turn[common]))
        self.assertFalse(physical.yaw_moment_into_turn.flags.writeable)

    def test_hydraulic_vectoring_uses_only_explicit_force_limit(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from simplified_vehicle_models.formulations import yaw_capability
        model=compile_formulation(str(ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")).model
        vehicle=point_mass_view_of_planar(model)
        observed=[]
        def fake_linprog(*args, **kwargs):
            observed.append((kwargs["A_ub"].copy(), kwargs["b_ub"].copy(),
                             kwargs["A_eq"].copy(), kwargs["b_eq"].copy()))
            return SimpleNamespace(success=True, fun=0.)
        with patch.object(yaw_capability, "_require_linprog", return_value=fake_linprog):
            for force_limit in (0., 500.):
                yaw_capability._cell_lp(vehicle, 5., -3., 0.,
                    TvLimits(100., False, True,
                             max_hydraulic_force_difference=force_limit), 1.)
        # Difference rows contain one +1 and one -1 in the hydraulic [8:12]
        # variables. The motor torque cap is identical in both LPs; the
        # hydraulic difference allowance changes from zero to 500 N.
        def limits(pair):
            a,b,*_=pair
            return [float(bound) for row,bound in zip(a,b)
                    if np.count_nonzero(row[8:12]) == 2
                    and np.count_nonzero(row[:8]) == 0
                    and np.count_nonzero(row[12:]) == 0]
        self.assertEqual(limits(observed[0]), [0., 0., 0., 0.])
        self.assertEqual(limits(observed[1]), [500., 500., 500., 500.])
        # Hydraulic vectoring does not remove the front/rear axle-total bias
        # equality. Its coefficient vector is (1-bias) per front wheel and
        # -bias per rear wheel in hydraulic variables [8:12].
        front_share = vehicle.front_brake_share
        expected = np.array([0., 0., 0., 0., 0., 0., 0., 0.,
                             1.-front_share, 1.-front_share, -front_share, -front_share, 0., 0.])
        for _, _, aeq, beq in observed:
            self.assertTrue(any(np.array_equal(row, expected) and bound == 0.
                                for row, bound in zip(aeq, beq)))

    def test_rear_driven_vehicle_can_brake_with_front_bias(self):
        # Hydraulic brakes are on all four wheels. A rear-driven car with a
        # 0.65 front brake bias must support straight-line braking; before the
        # fix every ax < 0 cell beyond aero drag was infeasible.
        from simplified_vehicle_models.formulations.yaw_capability import _cell_lp
        model=compile_formulation(str(ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")).model
        vehicle=point_mass_view_of_planar(model)
        self.assertNotIn("fl", vehicle.drivetrain.drivetrain.transmission.driven_wheels)
        self.assertGreater(vehicle.front_brake_share, 0.)
        for ax in (-1., -3., -6.):
            self.assertIsNotNone(_cell_lp(vehicle, 15., ax, 0., None, 1.), ax)
            self.assertIsNotNone(_cell_lp(vehicle, 15., ax, 0., None, -1.), ax)

    def test_planar_and_point_mass_brake_splits_match(self):
        from simplified_vehicle_models.formulations.yaw_capability import _cell_lp
        model=compile_formulation(str(ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")).model
        point_mass=point_mass_view_of_planar(model)
        self.assertAlmostEqual(point_mass.front_brake_share, model.front_brake_share)
        self.assertIsNotNone(_cell_lp(point_mass, 15., -3., 0., None, 1.))

    def test_legacy_drivetrain_is_rejected(self):
        from dataclasses import replace
        from simplified_vehicle_models.parameters.drivetrain import DrivetrainParameters
        model=compile_formulation(str(ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")).model
        vehicle=point_mass_view_of_planar(model)
        runtime=vehicle.drivetrain
        legacy=DrivetrainParameters(runtime.motor_count,runtime.gear_ratio,runtime.peak_motor_torque,
                                    runtime.power_limit,runtime.efficiency,runtime.maximum_motor_speed)
        with self.assertRaisesRegex(TypeError,"FullDrivetrainRuntime"):
            build_physical_yaw_map(replace(vehicle,drivetrain=legacy),YawCapabilityGrid([5.],[0.],[0.]),yaw_inertia=model.yaw_inertia)

    def test_steady_envelope_requires_exact_zero_acceleration_and_speed_is_checked(self):
        axes=([1.], [0.5,1.5], [-1.,1.]); shape=(1,2,2); values=np.ones(shape)
        chart=YawCapabilityMap(*axes,values,values,np.ones(shape,bool),1.,"physical_no_tv_v1","id")
        with self.assertRaisesRegex(ValueError,"exact zero"):
            chart.steady_envelope()
        with self.assertRaisesRegex(ValueError,"nonnegative"):
            YawCapabilityMap([-1.], [0.], [0.], np.zeros((1,1,1)), np.zeros((1,1,1)),
                             np.ones((1,1,1),bool),1.,"physical_no_tv_v1","id")
        with self.assertRaisesRegex(ValueError,"provenance"):
            YawCapabilityMap([1.], [0.], [0.], np.zeros((1,1,1)), np.zeros((1,1,1)),
                             np.ones((1,1,1),bool),1.,"physical_no_tv_v1","")

if __name__ == "__main__": unittest.main()
