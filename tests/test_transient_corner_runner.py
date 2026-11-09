"""Opt-in transient corner planning integration tests."""
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import hashlib
import json

import numpy as np

from tools.run.run_iem24_forward_driver import (
    apply_transient_corner_restriction, parse_args, source_files,
    DEFAULT_TRACK, policy_identity_settings,
)
from simplified_vehicle_models.solvers.braking_envelope import build_braking_envelope
from simplified_vehicle_models.compiler.compile import compile_formulation


class TransientCornerRunnerTests(unittest.TestCase):
    def test_parser_defaults_off_and_flag_on(self):
        self.assertFalse(parse_args([]).transient_corner_planning)
        self.assertTrue(parse_args(["--transient-corner-planning"]).transient_corner_planning)

    def test_source_closure_only_when_enabled(self):
        baseline = source_files("planar-7dof", DEFAULT_TRACK)
        enabled = source_files("planar-7dof", DEFAULT_TRACK, transient_corner_planning=True)
        yaw = Path("src/simplified_vehicle_models/solvers/yaw_transition.py")
        self.assertNotIn(yaw, baseline)
        self.assertIn(yaw, enabled)

    def test_identity_flag_is_strict(self):
        args = SimpleNamespace(longitudinal_policy="braking-envelope", speed_gain=2.,
            envelope_deceleration_margin=.6, envelope_speed_points=9, maximum_speed=8.,
            lateral_margin=.9, transient_corner_planning=True,
            yaw_transition_semantic_sha256="abc", track_is_closed=True)
        identity = policy_identity_settings(args)
        self.assertTrue(identity["builder"]["transient_corner_planning"])
        self.assertEqual(identity["builder"]["yaw_transition_semantic_sha256"], "abc")
        args.transient_corner_planning = False
        self.assertNotEqual(identity, policy_identity_settings(args))

    def test_warm_identity_accepts_exact_then_rejects_changed_transient_evidence(self):
        from tools.run.run_iem24_forward_driver import warm_start_matches, canonical_json
        args=parse_args(['--transient-corner-planning'])
        args.yaw_transition_semantic_sha256='evidence'
        identity=policy_identity_settings(args)
        settings={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
        previous={'status':'Completed','lap_completed':True,'source_hashes':{'source':'hash'},
            'pedal_map':{'semantic_sha256':'no-pedal-map'},'settings':settings,
            'longitudinal_policy':{'policy':'braking-envelope','deceleration_margin':.6,
                'speed_query_points':9,'settings_identity_sha256':hashlib.sha256(canonical_json(identity).encode()).hexdigest()}}
        self.assertTrue(warm_start_matches(previous,{'source':'hash'},args,'no-pedal-map',identity))
        changed={**identity,'builder':{**identity['builder'],'yaw_transition_semantic_sha256':'other'}}
        self.assertFalse(warm_start_matches(previous,{'source':'hash'},args,'no-pedal-map',changed))
        args.transient_corner_planning=False
        self.assertFalse(warm_start_matches(previous,{'source':'hash'},args,'no-pedal-map',policy_identity_settings(args)))

    def test_closed_restriction_preserves_seam_and_reproducible_digest(self):
        from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope
        vehicle=compile_formulation('models/formulations/iem24_murphy_real_planar_7dof.xml').model
        x=np.array([0.,1.,3.,4.]);k=np.array([0.,.1,-.1,0.]);caps=np.full(4,15.)
        envelope=BrakingEnvelope(x,k,caps,caps,np.full(3,2.),np.array([0,1,2,0]),True,20.,
                                {'segment_speed_domain_max':(15.,15.,15.)})
        plan=SimpleNamespace(distance=x,curvature=k,speed=caps,is_closed=True)
        with tempfile.TemporaryDirectory() as tmp:
            first,env_first,detail=apply_transient_corner_restriction(vehicle,plan,envelope,Path(tmp)/'first.json')
            second,env_second,repeat=apply_transient_corner_restriction(vehicle,plan,envelope,Path(tmp)/'second.json')
            self.assertEqual(first.speed[0],first.speed[-1])
            self.assertEqual(env_first.entry_speed[0],env_first.entry_speed[-1])
            self.assertEqual(detail['semantic_sha256'],repeat['semantic_sha256'])
            self.assertEqual(detail['artifact_sha256'],repeat['artifact_sha256'])
            np.testing.assert_array_equal(first.speed,second.speed)
            np.testing.assert_array_equal(env_first.corner_speed,env_second.corner_speed)

    def test_applies_restricted_caps_and_writes_artifact(self):
        vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model
        x = np.linspace(0., 10., 11)
        k = np.array([0., .01, .02, .04, .08, .12, .08, .04, .02, .01, 0.])
        envelope = build_braking_envelope(x, k, vehicle, maximum_speed=25., lateral_margin=1.,
            deceleration_margin=.6, speed_query_points=3, is_closed=False)
        plan = SimpleNamespace(distance=x, curvature=k, speed=np.full(len(x), 20.), is_closed=False)
        with tempfile.TemporaryDirectory() as tmp:
            restricted_plan, restricted, detail = apply_transient_corner_restriction(
                vehicle, plan, envelope, artifact_path=Path(tmp) / "yaw.json")
            self.assertTrue(np.all(restricted.corner_speed <= envelope.corner_speed))
            self.assertTrue(np.all(restricted_plan.speed <= plan.speed))
            self.assertEqual(restricted_plan.speed[0],0.)
            self.assertFalse(restricted_plan.is_closed)
            self.assertTrue((Path(tmp) / detail["file"]).is_file())
            self.assertTrue(detail["semantic_sha256"])


if __name__ == "__main__":
    unittest.main()
