"""Durability contracts for fixed-path yaw probe output."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from tools.run import probe_coupled_yaw_profile as probe


class ProbeArtifactTests(unittest.TestCase):
    def test_writes_path_map_and_explicit_legacy_failure_when_profile_raises(self):
        capability = SimpleNamespace(feasible_mask=np.ones((2, 2, 2), bool),
            yaw_moment_into_turn=np.ones((2, 2, 2)), yaw_moment_out_of_turn=np.ones((2, 2, 2)),
            yaw_inertia=2., source_identity="probe-test")
        args = SimpleNamespace(track=Path("track.xml"), geometry="raw-centerline", spacing=.05,
            speed_points=2, ax_points=2, ay_points=2, maximum_speed=10., anchored=False,
            output=Path(tempfile.gettempdir()) / "unused.json")
        path = SimpleNamespace(distance=np.array([0., .05, .1]), curvature=np.zeros(3),
            curvature_rate=np.zeros(3), is_closed=False)
        with tempfile.TemporaryDirectory() as directory:
            args.output = Path(directory) / "probe.json"
            with patch("simplified_vehicle_models.compiler.compile.compile_formulation",
                       return_value=SimpleNamespace(model=SimpleNamespace(yaw_inertia=2.))), \
                 patch("simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar", return_value=object()), \
                 patch.object(probe.runner, "physical_map_speed_limit", return_value=10.), \
                 patch("simplified_vehicle_models.tracks.profile.load_track", return_value=object()), \
                 patch.object(probe, "_sample_geometry", return_value=path), \
                 patch("simplified_vehicle_models.formulations.yaw_capability.YawCapabilityGrid") as grid_type, \
                 patch("simplified_vehicle_models.formulations.yaw_capability.build_physical_yaw_map", return_value=capability), \
                 patch("simplified_vehicle_models.solvers.yaw_aware_profile._MapInterpolator"), \
                 patch("simplified_vehicle_models.solvers.yaw_aware_profile.yaw_aware_profile", side_effect=ValueError("no profile")):
                grid_type.return_value.speed = np.array([0., 10.])
                grid_type.return_value.longitudinal_acceleration = np.array([-1., 1.])
                grid_type.return_value.lateral_acceleration = np.array([-2., 2.])
                result = probe.run(args)
            artifact = json.loads(args.output.read_text())
        self.assertEqual(result["profile"]["status"], "failed")
        self.assertTrue(artifact["path"]["distance"])
        self.assertTrue(artifact["map"]["feasible_mask"])
        self.assertIsNone(artifact["profile"]["failed_segment"])
        self.assertIn("does not expose", artifact["profile"]["segment_evidence_note"])


if __name__ == "__main__":
    unittest.main()
