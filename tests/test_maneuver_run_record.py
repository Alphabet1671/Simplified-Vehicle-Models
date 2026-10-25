import tempfile
import unittest
from pathlib import Path
from types import MappingProxyType
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.controls.maneuvers.types import RobotCommand
from simplified_vehicle_models.results.maneuver_run_record import write_maneuver_run_record
from simplified_vehicle_models.results.run_record import load_run_record
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from tools.anim.resample import FrameSource


class ManeuverRunRecordTests(unittest.TestCase):
    def test_exports_accepted_evidence_without_physics_evaluation(self):
        names = tuple(VerticalForwardPlant.state_names)
        self.assertEqual(len(names), 38)
        time = np.array([0., .25, .5])
        state = np.arange(3 * 38, dtype=float).reshape(3, 38) / 10
        rate = -state
        # FrameSource uses these to check adapter outputs stay available.
        channels = tuple(MappingProxyType({"position-x": [float(i)], "position-y": [float(2*i)],
                          "speed": [float(5+i)], "not-catalogued": [99]})
                         for i in range(3))
        commands = tuple(RobotCommand(.1*i, .2*i, .0) for i in range(3))
        result = type("Result", (), {"time_s": time, "state": state,
                    "state_rate": rate, "applied_commands": commands,
                    "channels": channels})()
        definition = {"schema": "maneuver-v1", "maneuver_id": "synthetic",
                      "profile": {"duration_s": .5}, "state_names": names}
        source = {"schema": "maneuver-artifacts-v1", "version": 1,
                  "execution_outcome": "completed", "model_kind": "14dof-simple",
                  "definition": definition, "hashes": {"result_sha256": "a" * 64},
                  "state_names": names, "vehicle_geometry": {"wheelbase_m": 2.5},
                  "artifact_labels": {"review": ("accepted", "synthetic")},
                  "numeric_metadata": np.array([1, 2], dtype=np.int64)}
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "accepted.runrec"
            # If conversion ever instantiates/evaluates plant physics, fail the test.
            with patch.object(VerticalForwardPlant, "__init__", side_effect=AssertionError("physics")), \
                 patch.object(VerticalForwardPlant, "evaluate", side_effect=AssertionError("physics")):
                write_maneuver_run_record(target, definition, result,
                                          model_kind="14dof-simple", source_manifest=source)
            record = load_run_record(target, verify_sources=False)
        np.testing.assert_array_equal(record.time, time)
        np.testing.assert_array_equal(record.state, state)
        np.testing.assert_array_equal(record.state_rate, rate)
        self.assertEqual(record.manifest.control_names,
                         ("swa-rad", "accelerator-travel", "brake-travel"))
        np.testing.assert_array_equal(record.control,
            [[c.swa_rad, c.accelerator, c.brake] for c in commands])
        self.assertEqual(record.manifest.channel_names,
                         ("speed", "position-x", "position-y"))
        self.assertIn("maneuver-artifact-sha256=sha256:" + "a"*64,
                      record.manifest.notes)
        self.assertEqual(record.manifest.vehicle_geometry["wheelbase_m"], 2.5)
        frames = FrameSource(record, 4)
        self.assertEqual(frames.times[0], time[0])
        self.assertEqual(frames.times[-1], time[-1])
        self.assertTrue(frames.has_channel("position-x"))
        self.assertTrue(frames.has_channel("speed"))
        np.testing.assert_array_equal(record.channel_values("position-x"), [0., 1., 2.])

    def test_rejects_forged_model_kind_state_names_and_manifest(self):
        names = tuple(VerticalForwardPlant.state_names)
        definition = {"schema": "maneuver-v1", "state_names": names}
        result = type("Result", (), {"time_s": np.array([0.]),
                    "state": np.zeros((1, 38)), "state_rate": np.zeros((1, 38)),
                    "applied_commands": (RobotCommand(0, 0, 0),),
                    "channels": (MappingProxyType({}),)})()
        source = {"schema": "maneuver-artifacts-v1", "version": 1,
                  "execution_outcome": "completed", "model_kind": "14dof-simple",
                  "definition": definition, "hashes": {"result_sha256": "b" * 64},
                  "state_names": names}
        with self.assertRaisesRegex(ValueError, "model_kind"):
            write_maneuver_run_record("unused", definition, result,
                                      model_kind="14dof", source_manifest=source)
        forged = dict(source, state_names=names[:-1] + ("forged",))
        with self.assertRaisesRegex(ValueError, "canonical"):
            write_maneuver_run_record("unused", definition, result,
                                      model_kind="14dof-simple", source_manifest=forged)
        for bad in (dict(source, schema="untrusted"),
                    dict(source, version=2),
                    dict(source, execution_outcome="failed"),
                    dict(source, model_kind="planar-7dof"),
                    dict(source, hashes={"result_sha256": "not-a-hash"}),
                    dict(source, definition={"schema": "maneuver-v1", "state_names": ("wrong",)})):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    write_maneuver_run_record("unused", definition, result,
                                              model_kind="14dof-simple", source_manifest=bad)


if __name__ == "__main__":
    unittest.main()
