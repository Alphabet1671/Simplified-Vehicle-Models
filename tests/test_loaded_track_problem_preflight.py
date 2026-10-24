import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.track_problem_contract import validate_problem_contract
from simplified_vehicle_models.solvers.loaded_track_problem import preflight_loaded_problem
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
from simplified_vehicle_models.tracks.types import PathCoordinates


class LoadedTrackProblemTest(unittest.TestCase):
    def setUp(self):
        s = np.linspace(0.0, 10.0, 11)
        coords = PathCoordinates(s, np.zeros_like(s), s, np.full_like(s, np.pi / 2), np.zeros_like(s), False)
        self.track = SimpleNamespace(path_coordinates=coords, is_closed=False,
                                     source_reference_interpolation_periodic=False,
                                     distance=s, curvature=np.zeros_like(s))
        self.reference = ReferenceCurve(self.track)
        self.corridor = SimpleNamespace(distance=s, minimum=np.full_like(s, -1.0),
                                        maximum=np.full_like(s, 1.0), is_closed=False,
                                        provenance="assumed")
        self.surface = SimpleNamespace(speed_floor=2.0, curvature=np.linspace(-1., 1., 5),
                                       speed=np.linspace(2., 30., 5))
        self.settings = SimpleNamespace(start_speed=3.0, minimum_speed=2.0,
                                         maximum_heading_error=0.5, maximum_heading_error_rate=1.0)
        self.contract = {"schema": "track_problem_v1", "topology": "open",
                         "source_stations": s.tolist(), "lower_offset": [-1.] * len(s),
                         "upper_offset": [1.] * len(s),
                         "boundary": {"start_mode": "rolling", "start_speed_mps": 3., "finish_mode": "free"}}

    def run_check(self, contract=None, settings=None):
        pure = validate_problem_contract(self.contract if contract is None else contract)
        return preflight_loaded_problem(pure, track=self.track, reference=self.reference,
                                        corridor=self.corridor, surface=self.surface,
                                        settings=self.settings if settings is None else settings,
                                        mesh=np.linspace(0., 10., 5), constraint_sites=np.linspace(1., 9., 5))

    def test_open_rolling_contract_binds_actual_reference_and_corridor(self):
        self.assertIs(type(self.reference), ReferenceCurve)
        result = self.run_check()
        self.assertEqual(result["status"], "supported")
        self.assertEqual(result["contract_hash"], validate_problem_contract(self.contract)["normalized_hash"])
        self.assertEqual(result["corridor_policy"], "legacy-abs-source-curvature-v1")
        self.assertEqual(result["corridor_identity"]["policy"], "legacy-abs-source-curvature-v1")
        self.assertEqual(result["corridor_identity"]["provenance"], "assumed")
        self.assertEqual(result["sampling"], "diagnostic_only_not_continuous_certificate")

    def test_corridor_policy_identity_keeps_policy_distinct_from_width_provenance(self):
        from simplified_vehicle_models.tracks.corridor import SIGNED_POLICY
        descriptor = {"schema": "test-v1", "nested": {"guard": 0.2}}
        self.corridor.policy = SIGNED_POLICY
        self.corridor.policy_descriptor = descriptor
        self.corridor.policy_hash = "abc123"
        result = self.run_check()
        self.assertEqual(result["corridor_policy"], SIGNED_POLICY)
        self.assertEqual(result["corridor_identity"]["policy"], SIGNED_POLICY)
        self.assertEqual(result["corridor_identity"]["provenance"], "assumed")
        self.assertEqual(result["corridor_identity"]["descriptor"], descriptor)
        self.assertEqual(result["corridor_identity"]["policy_sha256"], "abc123")

    def test_contract_source_grid_must_match_loaded_corridor(self):
        altered = dict(self.contract, source_stations=[0., 4., 10.])
        self.assertEqual(self.run_check(altered)["status"], "invalid")

    def test_closed_contract_is_not_claimed_when_optimizer_does_not_enforce_start(self):
        s = np.linspace(0., 10., 11)
        self.track.is_closed = True
        self.track.path_coordinates = PathCoordinates(s, np.cos(2*np.pi*s/10), np.sin(2*np.pi*s/10),
                                                     np.zeros_like(s), np.full_like(s, .1), True)
        self.track.source_reference_interpolation_periodic = True
        self.reference = ReferenceCurve(self.track)
        self.corridor.is_closed = True
        c = dict(self.contract, topology="closed", boundary={"start_mode":"flying", "start_speed_mps":3., "finish_mode":"periodic"})
        result = self.run_check(c)
        self.assertEqual(result["status"], "unsupported")
        self.assertIn("requested_boundary_not_enforced", result["reasons"])

    def test_below_combined_settings_and_chart_floor_is_rejected_not_clamped(self):
        c = dict(self.contract, boundary={"start_mode":"rolling", "start_speed_mps":1., "finish_mode":"free"})
        self.settings.start_speed = 1.0
        result = self.run_check(c)
        self.assertEqual(result["status"], "unsupported")
        self.assertIn("start_speed_below_model_floor", result["reasons"])

    def test_signed_chart_uses_physical_speed_domain_api(self):
        from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
        self.surface = SignedTransientCapability(u=np.array([0., 1.]), curvature=np.array([-1., 1.]),
            acceleration_max=np.ones((2, 2)), acceleration_min=-np.ones((2, 2)),
            domain_speed_values=np.array([8., 8.]), speed_floor=2., provenance_hash="test")
        self.assertEqual(self.run_check()["status"], "supported")

    def test_legacy_is_explicit_diagnostic(self):
        result = preflight_loaded_problem(None, track=self.track, reference=None,
                                         corridor=None, surface=None, settings=None,
                                         mesh=None, constraint_sites=None)
        self.assertEqual(result["status"], "legacy_diagnostic")
        self.assertIsNone(result["contract_hash"])


if __name__ == "__main__":
    unittest.main()
