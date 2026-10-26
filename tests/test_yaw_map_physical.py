import unittest
from unittest.mock import patch
import numpy as np
from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
from simplified_vehicle_models.formulations.yaw_capability import DirectYawCell, YawCapabilityGrid
from simplified_vehicle_models.formulations.yaw_map_domain import ProbeStatus


class PhysicalCacheTests(unittest.TestCase):
    def test_shared_cache_reuses_both_moments_at_map_node(self):
        vehicle=object(); evaluator=PhysicalYawEvaluator(vehicle)
        with patch('simplified_vehicle_models.formulations.yaw_map_physical.direct_yaw_cell',
                   return_value=DirectYawCell(ProbeStatus.FEASIBLE,10.,20.)) as direct, \
             patch('simplified_vehicle_models.formulations.yaw_map_physical.capability_hash',return_value='fixture'):
            self.assertEqual(evaluator(10.,0.,20.).status,ProbeStatus.FEASIBLE)
            cap=evaluator.build(vehicle,YawCapabilityGrid([10.],[0.],[20.]),yaw_inertia=1.)
            evaluator(10.,0.,20.)
        self.assertEqual(direct.call_count,1)
        np.testing.assert_equal(cap.yaw_moment_into_turn,[[[10.]]])
        np.testing.assert_equal(cap.yaw_moment_out_of_turn,[[[20.]]])
