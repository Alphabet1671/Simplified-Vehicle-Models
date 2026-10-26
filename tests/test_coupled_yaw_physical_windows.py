"""Frozen physical windows where the legacy greedy profile fails.

Each fixture records a MIS/FSC fine-grid window and its physical yaw map. The
legacy forward/backward sweep raised a transition error on every window; the
coupled solver must return a profile that passes independent validation.
"""
import json
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
from simplified_vehicle_models.solvers.coupled_yaw_profile import solve_coupled_profile
from simplified_vehicle_models.solvers.yaw_profile_validation import validate_profile

FIXTURES = Path(__file__).parent / "fixtures" / "yaw_profile"


def load_window(name):
    doc = json.loads((FIXTURES / name).read_text())
    w, m = doc["window"], doc["map_evidence"]
    a = lambda x: np.asarray(x, float)
    cap = YawCapabilityMap(a(m["speed"]), a(m["ax"]), a(m["ay"]), a(m["yaw_moment_into_turn"]),
                           a(m["yaw_moment_out_of_turn"]), np.asarray(m["feasible_mask"], bool),
                           float(m["yaw_inertia"]), "physical_no_tv_v1", m["identity"])
    distance = a(w["distance_m"]); distance = distance - distance[0]
    path = YawPath(distance, a(w["curvature_per_m"]), a(w["curvature_rate_per_m2"]), False)
    return doc, path, cap


class PhysicalWindowTests(unittest.TestCase):
    def _check(self, name):
        doc, path, cap = load_window(name)
        self.assertEqual(doc["failure"]["status"], "failed")
        vmax = float(cap.speed[-1])
        result = solve_coupled_profile(path, cap, boundary="standing", maximum_speed=vmax,
                                       settings=CoupledProfileSettings())
        speed = np.sqrt(result["q"])
        validation = validate_profile(path, cap, speed, maximum_speed=vmax, boundary="standing",
                                      settings=CoupledProfileSettings())
        self.assertTrue(validation.valid, validation)
        self.assertEqual(speed[0], 0.)
        self.assertEqual(result["diagnostics"]["status"], "feasible_sampled")

    def test_mis_raw_centerline_window(self):
        self._check("mis-raw-centerline-original-window.json")

    def test_fsc_zero_offset_window(self):
        self._check("fsc-zero-original-window.json")


if __name__ == "__main__":
    unittest.main()
