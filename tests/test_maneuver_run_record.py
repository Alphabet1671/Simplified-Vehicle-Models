import json
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from types import MappingProxyType, SimpleNamespace

import numpy as np

from simplified_vehicle_models.controls.maneuvers.types import RobotCommand, ManeuverDefinition
from simplified_vehicle_models.results.maneuver_run_record import write_maneuver_run_record
from simplified_vehicle_models.results.run_record import load_run_record
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from tools.anim.resample import FrameSource
from tools.maneuver_gui.artifacts import write_artifacts


class ManeuverRunRecordTests(unittest.TestCase):
    def _artifact(self, directory):
        definition = {"schema": "maneuver-v1", "name": "synthetic", "preset": "step_steer",
            "backend": "simulation", "vehicle": {"model": "models/formulations/iem24_murphy_real_14dof_simple.xml"},
            "initial": {"speed_mps": 10., "x_m": 0., "y_m": 0., "heading_rad": 0.},
            "clock": {"controller_tick_s": .01},
            "steering": {"mode": "peak_swa", "amplitude_rad": .2, "direction": 1},
            "longitudinal": {"mode": "constant_accelerator", "accelerator": .1},
            "profile": {"lead_in_s": 0., "rise_s": .1, "hold_s": .2, "release_s": .1,
                        "recovery_s": .1, "duration_s": .5},
            "limits": {"max_swa_rad": 1., "max_swa_rate_rad_s": 2.},
            "termination": {"duration_s": .5}, "calibration": None, "csv": None}
        definition = ManeuverDefinition.from_dict(definition).to_dict()
        assert "state_names" not in definition
        time = np.array([0., .25, .5]); state = np.arange(3*38, dtype=float).reshape(3, 38)/10
        rows = tuple(MappingProxyType({"position-x": [float(i)], "position-y": [float(2*i)],
                                      "heading-world": [0.], "speed": [float(5+i)]}) for i in range(3))
        result = SimpleNamespace(time_s=time, state=state, state_rate=-state,
            applied_commands=tuple(RobotCommand(.1*i, .2*i, 0.) for i in range(3)),
            channels=rows, execution_outcome="completed", termination_reason="duration")
        artifact = directory / "artifact"
        artifact.mkdir()
        write_artifacts(artifact, definition, result, {}, model_kind="14dof-simple")
        return artifact, definition, time, state

    def test_exports_real_hash_checked_artifact_evidence_and_animator_compatibility(self):
        with tempfile.TemporaryDirectory() as tmp:
            artifact, definition, time, state = self._artifact(Path(tmp))
            target = Path(tmp) / "accepted.runrec"
            with patch.object(VerticalForwardPlant, "__init__", side_effect=AssertionError("physics")), \
                 patch.object(VerticalForwardPlant, "evaluate", side_effect=AssertionError("physics")):
                write_maneuver_run_record(target, definition, artifact)
            record = load_run_record(target, verify_sources=False)
            np.testing.assert_array_equal(record.time, time)
            np.testing.assert_array_equal(record.state, state)
            frames = FrameSource(record, 4)
            self.assertEqual((frames.times[0], frames.times[-1]), (time[0], time[-1]))
            self.assertTrue(frames.has_channel("position-x")); self.assertTrue(frames.has_channel("speed"))

    def test_planar_model_exports_cartesian_kinematics_without_fabricating_forces(self):
        with tempfile.TemporaryDirectory() as tmp:
            definition = ManeuverDefinition.from_dict({
                "schema":"maneuver-v1", "name":"planar", "preset":"step_steer", "backend":"simulation",
                "vehicle":{"model":"models/formulations/iem24_murphy_real_planar_7dof.xml"},
                "initial":{"speed_mps":10.,"x_m":0.,"y_m":0.,"heading_rad":0.},
                "clock":{"controller_tick_s":.01}, "steering":{"mode":"peak_swa","amplitude_rad":.2},
                "longitudinal":{"mode":"constant_accelerator","accelerator":.1},
                "profile":{"duration_s":.5}, "limits":{}, "termination":{"duration_s":.5},
                "calibration":None,"csv":None}).to_dict()
            from simplified_vehicle_models.solvers.maneuver_planar import ManeuverPlanarPlant
            state=np.zeros((2,len(ManeuverPlanarPlant.state_names))); state[:,0]=[3.,4.]; state[:,1]=[5.,6.]; state[:,2]=[.1,.2]; state[:,3]=[3.,4.]; state[:,4]=[4.,3.]
            result=SimpleNamespace(time_s=np.array([0.,.1]),state=state,state_rate=np.zeros_like(state),
                applied_commands=(RobotCommand(0.,0.,0.),)*2,channels=({},{}),execution_outcome="completed",termination_reason="duration")
            artifact=Path(tmp)/"artifact"; artifact.mkdir(); write_artifacts(artifact,definition,result,{},model_kind="planar-7dof")
            target=Path(tmp)/"run_record.npz"; write_maneuver_run_record(target,definition,artifact,model_kind="planar-7dof")
            record=load_run_record(target,verify_sources=False)
            self.assertEqual(record.manifest.state_names,tuple(ManeuverPlanarPlant.state_names))
            self.assertNotIn("tire-vertical-force",record.manifest.channel_names)
            frames=FrameSource(record,10); self.assertTrue(frames.has_channel("position-x")); self.assertAlmostEqual(frames.scalar_at("speed",0),5.)

    def test_rejects_channel_tamper_and_noncompleted(self):
        with tempfile.TemporaryDirectory() as tmp:
            artifact, definition, _, _ = self._artifact(Path(tmp))
            path = artifact / "channels.json"
            data = json.loads(path.read_text()); data[0]["speed"] = [999]; path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "corruption"):
                write_maneuver_run_record(Path(tmp)/"x.runrec", definition, artifact)
        with tempfile.TemporaryDirectory() as tmp:
            artifact, definition, _, _ = self._artifact(Path(tmp))
            manifest_path = artifact / "manifest.json"
            manifest = json.loads(manifest_path.read_text()); manifest["execution_outcome"] = "stopped"
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                write_maneuver_run_record(Path(tmp)/"x.runrec", definition, artifact)


if __name__ == "__main__":
    unittest.main()
