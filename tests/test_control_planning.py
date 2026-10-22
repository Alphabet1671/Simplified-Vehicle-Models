import math
import unittest

from simplified_vehicle_models.controls.planning import (
    ControlResponse, PathDemand, controller_hash,
)


class _ExamplePolicy:
    name = 'example'
    provenance_hash = 'example-v1'
    is_stateful = False

    def __init__(self, cap):
        self.cap = cap

    def evaluate(self, demand):
        feasible = demand.longitudinal_acceleration <= self.cap
        return ControlResponse(feasible, min(demand.longitudinal_acceleration, self.cap), 0.0)


class _SlotsPolicy:
    __slots__ = ('cap', 'name', 'provenance_hash', 'is_stateful')

    def __init__(self, cap):
        self.cap = cap
        self.name = 'slots'
        self.provenance_hash = 'slots-v1'
        self.is_stateful = False

    def evaluate(self, demand):
        return ControlResponse(demand.longitudinal_acceleration <= self.cap,
                               min(demand.longitudinal_acceleration, self.cap), 0.0)


class PlanningContractTest(unittest.TestCase):
    def test_demand_requires_finite_positive_speed_and_finite_values(self):
        with self.assertRaises(ValueError):
            PathDemand(0.0, 0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            PathDemand(10.0, float('nan'), 0.0, 0.0, 0.0)

    def test_controller_hash_is_stable_and_parameter_sensitive(self):
        a = _ExamplePolicy(2.0)
        b = _ExamplePolicy(2.0)
        c = _ExamplePolicy(1.0)
        self.assertEqual(controller_hash(a), controller_hash(b))
        self.assertNotEqual(controller_hash(a), controller_hash(c))

    def test_response_reports_saturation_without_changing_sign(self):
        response = _ExamplePolicy(2.0).evaluate(PathDemand(20.0, 0.01, 5.0, 4.0, 0.2))
        self.assertFalse(response.feasible)
        self.assertEqual(response.longitudinal_acceleration, 2.0)
        self.assertEqual(controller_hash(_ExamplePolicy(2.0)),
                         controller_hash(_ExamplePolicy(2.0)))

    def test_slots_policy_and_array_parameters_are_hashable(self):
        import numpy as np
        first = _SlotsPolicy(np.array([1.0, 2.0]))
        second = _SlotsPolicy(np.array([1.0, 2.0]))
        third = _SlotsPolicy(np.array([1.0, 3.0]))
        self.assertEqual(controller_hash(first), controller_hash(second))
        self.assertNotEqual(controller_hash(first), controller_hash(third))
