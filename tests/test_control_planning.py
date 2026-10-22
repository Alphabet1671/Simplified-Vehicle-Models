import unittest

from simplified_vehicle_models.controls.planning import (
    CONTROL_POLICIES, ControlResponse, PathDemand, build_policy, controller_hash,
    register_policy,
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


class _StatefulPolicy(_ExamplePolicy):
    is_stateful = True


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

    def test_controller_hash_distinguishes_stateful_policies(self):
        stateless = _ExamplePolicy(2.0)
        stateful = _StatefulPolicy(2.0)
        self.assertEqual(stateless.name, stateful.name)
        self.assertEqual(stateless.provenance_hash, stateful.provenance_hash)
        self.assertNotEqual(controller_hash(stateless), controller_hash(stateful))

    def test_controller_hash_rejects_unsupported_parameter_types(self):
        with self.assertRaises(TypeError):
            controller_hash(_ExamplePolicy(object()))

    def test_response_reports_saturation_without_changing_sign(self):
        response = _ExamplePolicy(2.0).evaluate(PathDemand(20.0, 0.01, 5.0, 4.0, 0.2))
        self.assertFalse(response.feasible)
        self.assertEqual(response.longitudinal_acceleration, 2.0)
        self.assertEqual(controller_hash(_ExamplePolicy(2.0)),
                         controller_hash(_ExamplePolicy(2.0)))

    def test_saturation_is_coerced_to_a_stable_tuple(self):
        from_list = ControlResponse(True, 0.0, 0.0, ['lateral', 'longitudinal'])
        from_tuple = ControlResponse(True, 0.0, 0.0, ('lateral', 'longitudinal'))
        self.assertEqual(from_list.saturation, ('lateral', 'longitudinal'))
        self.assertEqual(from_list, from_tuple)
        self.assertEqual(hash(from_list), hash(from_tuple))
        with self.assertRaises(ValueError):
            ControlResponse(True, 0.0, 0.0, 'lateral')
        with self.assertRaises(ValueError):
            ControlResponse(True, 0.0, 0.0, 3)

    def test_slots_policy_and_array_parameters_are_hashable(self):
        import numpy as np
        first = _SlotsPolicy(np.array([1.0, 2.0]))
        second = _SlotsPolicy(np.array([1.0, 2.0]))
        third = _SlotsPolicy(np.array([1.0, 3.0]))
        self.assertEqual(controller_hash(first), controller_hash(second))
        self.assertNotEqual(controller_hash(first), controller_hash(third))

    def test_registry_registers_builds_and_rejects_unknown_and_invalid_names(self):
        name = 'test_cap_policy'
        register_policy(name, _ExamplePolicy)
        self.addCleanup(CONTROL_POLICIES.pop, name, None)
        built = build_policy(name, cap=2.0)
        self.assertIsInstance(built, _ExamplePolicy)
        self.assertEqual(built.cap, 2.0)
        with self.assertRaises(KeyError):
            build_policy('missing-policy')
        for invalid in ('', None, 3):
            with self.assertRaises(ValueError):
                register_policy(invalid, _ExamplePolicy)
