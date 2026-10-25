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

if __name__ == "__main__": unittest.main()
