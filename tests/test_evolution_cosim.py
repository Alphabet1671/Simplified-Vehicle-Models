import time, unittest
import numpy as np
from pathlib import Path
from tools.run.benchmark_rollout import _setup
from simplified_vehicle_models.evolution.cosim import (
    CosimSettings, TvInputs, simulate, state_index, tv_plant_class)
from simplified_vehicle_models.evolution.tv_controllers import PassthroughTV
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant

ROOT = Path(__file__).resolve().parents[1]
F = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
T = ROOT / "models/tracks/fsc_may_endurance_2026_width.xml"
_setup_plain = _setup
def _setup(f, t):
    return _setup_plain(f, t, tv=True)

class LayoutTests(unittest.TestCase):
    def test_state_index_uses_published_names(self):
        p = state_index(PlanarForwardPlant)
        v = state_index(VerticalForwardPlant)
        self.assertEqual((p["yaw_rate"], p["wheels"], p["steering"]), (5, (6, 7, 8, 9), 11))
        self.assertEqual((v["yaw_rate"], v["wheels"], v["steering"]), (8, (12, 13, 14, 15), 25))

    def test_tv_subclass_is_cached_and_passthrough_is_identical(self):
        self.assertIs(tv_plant_class(PlanarForwardPlant), tv_plant_class(PlanarForwardPlant))
        _, plan, plant, driver = _setup(F, T)
        base = _setup_plain(F, T)[2]
        x = plant.initial_state(float(plan.speed[0]))
        c = driver.command(plant.observe(x))
        np.testing.assert_array_equal(plant.evaluate(0.0, x, c).solve(), base.evaluate(0.0, x, c).solve())

class CountingTV(PassthroughTV):
    def __init__(self):
        super().__init__(); self.steps = 0
    def step(self, inputs: TvInputs):
        self.steps += 1
        return super().step(inputs)

class CosimTests(unittest.TestCase):
    def test_controller_steps_once_per_tick(self):
        _, plan, plant, driver = _setup(F, T)
        tv = CountingTV()
        result = simulate(plant, driver, tv, plant.initial_state(float(plan.speed[0])),
                          CosimSettings(controller_dt=0.01), stop_distance=1e9,
                          max_time=0.5, deadline=time.monotonic() + 60)
        self.assertTrue(result.completed, result.failure)
        self.assertEqual(tv.steps, 50)
        self.assertEqual(result.tv_torque.shape, (51, 4))

    def test_passthrough_matches_reference_solver(self):
        _, plan, plant, driver = _setup(F, T)
        x0 = plant.initial_state(float(plan.speed[0]))
        a = simulate(plant, driver, PassthroughTV(), x0, CosimSettings(controller_dt=0.002),
                     stop_distance=1e9, max_time=1.0, deadline=time.monotonic() + 60)
        b = simulate(plant, driver, PassthroughTV(), x0, CosimSettings(controller_dt=0.001),
                     stop_distance=1e9, max_time=1.0, deadline=time.monotonic() + 60)
        np.testing.assert_allclose(a.state[-1][:6], b.state[-1][:6], rtol=0, atol=0.02)

    def test_deadline_stops_cleanly(self):
        _, plan, plant, driver = _setup(F, T)
        r = simulate(plant, driver, PassthroughTV(), plant.initial_state(float(plan.speed[0])),
                     CosimSettings(), stop_distance=1e9, max_time=100.0,
                     deadline=time.monotonic() + 0.2)
        self.assertFalse(r.completed)
        self.assertEqual(r.failure, "deadline")
