"""Focused regression contracts for the consolidated hybrid fix wave."""
from __future__ import annotations

import hashlib
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from simplified_vehicle_models.solvers import longitudinal_capability as capability
from simplified_vehicle_models.solvers.hybrid_setup import (
    HYBRID_EXECUTION_ALLOCATOR_ID, HYBRID_HORIZON_S, HYBRID_SETUP_VERSION, HYBRID_SLABS,
)
from simplified_vehicle_models.solvers.hybrid_driver import HYBRID_POLICY_VERSION
from simplified_vehicle_models.solvers.hybrid_speed_plan import HYBRID_PLAN_VERSION
from simplified_vehicle_models.solvers.longitudinal_capability import PathKinematicContext
from tools.run import run_iem24_forward_driver as runner


class HybridFinalContracts(unittest.TestCase):
    def test_source_identity_rejects_aliases_symlinks_and_noncanonical_keys(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as td:
            root = Path(td)
            source_dir = root / "src/simplified_vehicle_models/solvers"
            source_dir.mkdir(parents=True)
            (source_dir / "fake.py").write_bytes(b"closure")
            real = root / "models/data.csv"
            real.parent.mkdir(parents=True); real.write_bytes(b"safe")
            digest = hashlib.sha256(b"safe").hexdigest()
            alias = root / "models/alias.csv"; alias.symlink_to(real)
            (root / "models/linked").symlink_to(root / "models", target_is_directory=True)
            old = capability.__file__
            try:
                capability.__file__ = str(source_dir / "fake.py")
                for bad in ("/etc/passwd", "../outside", "models//data.csv", "models/./data.csv", "models\\alias.csv"):
                    with self.subTest(path=bad), self.assertRaises(ValueError):
                        capability._source_closure_hash({bad: digest})
                with self.assertRaisesRegex(ValueError, "symlink"):
                    capability._source_closure_hash({"models/alias.csv": digest})
                with self.assertRaisesRegex(ValueError, "symlink"):
                    capability._source_closure_hash({"models/linked/data.csv": digest})
                self.assertTrue(capability._source_closure_hash({"models/data.csv": digest}))
            finally:
                capability.__file__ = old

    def test_frenet_progress_brake_inverse_context_is_analytic(self):
        kin = PathKinematicContext(14., 1.2, .12, .3, .08, .025, .001, .4, True)
        c, s = math.cos(kin.heading_error_rad), math.sin(kin.heading_error_rad)
        B = 1. - kin.curvature_inv_m * kin.lateral_error_m
        progress = (kin.vx_mps*c - kin.vy_mps*s) / B
        self.assertNotAlmostEqual(progress, kin.vx_mps)
        desired = 9.5
        inferred_body_vx = (desired * B + kin.vy_mps*s) / c
        recovered = (inferred_body_vx*c - kin.vy_mps*s) / B
        self.assertAlmostEqual(recovered, desired, places=12)
        bad = PathKinematicContext(14., 1.2, math.pi/2, .3, .08, .025, .001, .4, True)
        self.assertLess(abs(math.cos(bad.heading_error_rad)), 1.e-6)

    def test_policy_versions_and_setup_constants_are_shared(self):
        identity = runner.policy_identity_settings(runner.parse_args([
            "--model", "14dof-simple", "--longitudinal-mode", "hybrid-drive-brake-v1", "--initial-speed", "1"]))
        self.assertEqual(identity["hybrid_setup_version"], HYBRID_SETUP_VERSION)
        self.assertEqual(identity["hybrid_plan_version"], HYBRID_PLAN_VERSION)
        self.assertEqual(identity["hybrid_policy_version"], HYBRID_POLICY_VERSION)
        self.assertEqual((identity["hybrid_horizon_s"], identity["hybrid_slabs"]), (HYBRID_HORIZON_S, HYBRID_SLABS))
        self.assertEqual(identity["hybrid_max_extra_forecasts"],8)
        self.assertEqual(identity["hybrid_bracket_resolution"],1e-8)
        self.assertEqual(identity["hybrid_max_total_transitions"],330)
        self.assertEqual(HYBRID_EXECUTION_ALLOCATOR_ID, "independent-wheel-torque-v1")

    def test_runner_uses_one_hybrid_plan_for_steering(self):
        import numpy as np
        from types import SimpleNamespace
        hybrid=SimpleNamespace(distance_m=np.array([0.,1.]),speed_mps=np.array([2.,3.]),curvature_inv_m=np.zeros(2),is_closed=False,
            sample=lambda s: SimpleNamespace(speed_mps=2.+s,curvature_inv_m=0.,acceleration_mps2=.25))
        from tools.run.run_iem24_forward_driver import _hybrid_steering_plan
        adapter=_hybrid_steering_plan(hybrid)
        self.assertIs(adapter.speed,hybrid.speed_mps)
        self.assertEqual(adapter.sample(.5),(2.5,0.,.25))
        with patch("simplified_vehicle_models.solvers.forward_driver.build_speed_plan",side_effect=AssertionError("no second plan")) as build:
            self.assertIs(adapter.speed,hybrid.speed_mps)
            build.assert_not_called()


if __name__ == "__main__":
    unittest.main()
