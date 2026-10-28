import time
import unittest
from pathlib import Path
import numpy as np
from tools.run.benchmark_rollout import _setup
from simplified_vehicle_models.evolution.cosim import CosimSettings, simulate
from simplified_vehicle_models.evolution.tv_controllers import PassthroughTV
from simplified_vehicle_models.evolution.tick_integrators import REGISTERED

ROOT = Path(__file__).resolve().parents[1]
F = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
T = ROOT / "models/tracks/fsc_may_endurance_2026_width.xml"
TOL = np.array([0.5, 0.05, np.inf, 0.1, np.inf, 0.02])

class IntegratorAccuracy(unittest.TestCase):
    def test_each_registered_integrator_matches_reference(self):
        _, plan, plant, driver = _setup(F, T, tv=True)
        x0 = plant.initial_state(float(plan.speed[0]))
        ref = simulate(plant, driver, PassthroughTV(), x0,
                       CosimSettings(controller_dt=0.002), stop_distance=1e9,
                       max_time=2.0, deadline=time.monotonic() + 120)
        self.assertTrue(ref.completed, ref.failure)
        for name in REGISTERED:
            with self.subTest(integrator=name):
                _, plan, plant, driver = _setup(F, T, tv=True)
                r = simulate(plant, driver, PassthroughTV(), x0,
                             CosimSettings(controller_dt=0.01, integrator=name),
                             stop_distance=1e9, max_time=2.0,
                             deadline=time.monotonic() + 120)
                self.assertTrue(r.completed, r.failure)
                error = np.abs(r.state[-1][:6] - ref.state[-1][:6])
                self.assertTrue(np.all(error <= TOL), (name, error))

    def test_deadline_checked_during_step(self):
        _, plan, plant, driver = _setup(F, T, tv=True)
        result = simulate(plant, driver, PassthroughTV(),
                          plant.initial_state(float(plan.speed[0])),
                          CosimSettings(controller_dt=0.01, integrator="radau"),
                          stop_distance=1e9, max_time=1.0, deadline=time.monotonic()-1)
        self.assertEqual(result.failure, "deadline")
