from pathlib import Path
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
from simplified_vehicle_models.formulations.yaw_map_parallel import ParallelYawEvaluator

def main():
    model=compile_formulation(str(Path('models/formulations/iem24_murphy_real_planar_7dof.xml').resolve())).model
    vehicle=point_mass_view_of_planar(model); points=[(5.,-1.,0.),(5.,0.,0.),(5.,1.,1.)]
    expected=PhysicalYawEvaluator(vehicle).cells_many(points)
    with ParallelYawEvaluator(vehicle,workers=2) as parallel: actual=parallel.cells_many(points)
    print([c.status for c in expected]); print([c.status for c in actual])
    assert [c.status for c in expected]==[c.status for c in actual]
    assert [c.into_turn for c in expected]==[c.into_turn for c in actual]
if __name__=='__main__': main()
