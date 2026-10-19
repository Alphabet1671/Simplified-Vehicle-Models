"""Check the shared run result file: write, read, validate and resample."""
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

import numpy as np

from simplified_vehicle_models.results.run_record import (
    CHANNEL_CATALOGUE, RunRecorder, RunRecordError, RunRecordManifest, frame_times,
    load_run_record, resample_run_record, sha256_of, unpack_validity,
)

STATES = ("s", "v_x", "yaw_rate")
CONTROLS = ("throttle", "brake-force")
CHANNELS = ("speed", "tire-vertical-force", "roll")


def make_record(directory, times=(0.0, 0.4, 1.0), precision="float32", sources=None,
                maximum_step=None, channels=CHANNELS, name="record.npz"):
    recorder = RunRecorder(Path(directory) / name, state_names=STATES, control_names=CONTROLS,
                           channel_names=channels, precision=precision)
    for index, t in enumerate(times):
        recorder.append(
            t, index, [t**3 - t, 2.0 * t, 1.0 - t], [3 * t**2 - 1.0, 2.0, -1.0], [0.5, 10.0 * t],
            {"speed": [2.0 * t], "tire-vertical-force": [100 + t, 200 + t, 300 + t, 400 + t],
             "roll": [0.01 * t]})
    integrator = {} if maximum_step is None else {"maximum_step": maximum_step}
    return recorder.close({"sources": sources or {}, "integrator": integrator,
                           "model": {"plant_class": "Synthetic"}},
                          created_utc="2026-10-16T00:00:00Z")


class RunRecordRoundTripTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def test_round_trip_preserves_names_units_rows_and_precision(self):
        path = make_record(self.directory.name)
        record = load_run_record(path)
        manifest = record.manifest
        self.assertEqual(manifest.state_names, STATES)
        self.assertEqual(manifest.control_names, CONTROLS)
        self.assertEqual(manifest.channel_names, CHANNELS)
        self.assertEqual(manifest.channel_widths, (1, 4, 1))
        self.assertEqual(manifest.channel_units, ("m/s", "N", "rad"))
        self.assertEqual(manifest.rows, 3)
        self.assertEqual(manifest.precision, "float32")
        self.assertEqual(record.state.dtype, np.float32)
        self.assertEqual(record.time.dtype, np.float64)
        np.testing.assert_allclose(record.channel_values("tire-vertical-force")[1],
                                   [100.4, 200.4, 300.4, 400.4], rtol=1e-6)
        self.assertEqual(record.channel_values("speed").shape, (3,))
        self.assertTrue(np.all(record.channel_validity("roll")))

    def test_float64_payload_is_exact(self):
        record = load_run_record(make_record(self.directory.name, precision="float64"))
        self.assertEqual(record.state.dtype, np.float64)
        self.assertEqual(record.state[1, 1], 0.8)

    def test_container_loads_without_pickle_and_arrays_are_read_only(self):
        path = make_record(self.directory.name)
        with np.load(path, allow_pickle=False) as archive:
            self.assertIn("manifest.json", archive.files)
            self.assertEqual(archive["time"].shape, (3,))
        record = load_run_record(path)
        with self.assertRaises(ValueError):
            record.state[0, 0] = 1.0

    def test_manifest_is_first_member_and_round_trips(self):
        path = make_record(self.directory.name)
        with zipfile.ZipFile(path) as archive:
            self.assertEqual(archive.namelist()[0], "manifest.json")
            text = archive.read("manifest.json")
        manifest = RunRecordManifest.from_json(text)
        self.assertEqual(RunRecordManifest.from_json(manifest.to_json()), manifest)
        self.assertEqual(json.loads(text)["schema"], "run-record-v1")

    def test_channel_widths_follow_the_catalogue(self):
        for name in CHANNELS:
            self.assertIn(name, CHANNEL_CATALOGUE)
        self.assertEqual(CHANNEL_CATALOGUE["tire-vertical-force"].width, 4)
        self.assertEqual(CHANNEL_CATALOGUE["roll-quasi-static"].width, 1)

    def test_uncatalogued_channel_is_rejected(self):
        with self.assertRaises(ValueError):
            RunRecorder(Path(self.directory.name) / "x.npz", state_names=STATES,
                        channel_names=("not-a-channel",))

    def test_missing_channel_value_is_stored_invalid_not_zero(self):
        recorder = RunRecorder(Path(self.directory.name) / "x.npz", state_names=STATES,
                               channel_names=("speed",))
        recorder.append(0.0, 0, [0, 0, 0], channels={"speed": [1.0]})
        recorder.append(0.1, 1, [0, 0, 0], channels={"speed": [float("nan")]})
        recorder.append(0.2, 2, [0, 0, 0], channels={})
        record = load_run_record(recorder.close())
        self.assertEqual(record.channel_validity("speed").tolist(), [True, False, False])
        self.assertTrue(np.all(np.isnan(record.channel_values("speed")[1:])))

    def test_chunked_buffering_matches_a_single_chunk(self):
        times = np.linspace(0.0, 1.0, 11)
        paths = []
        for chunk_rows, name in ((3, "a.npz"), (4096, "b.npz")):
            recorder = RunRecorder(Path(self.directory.name) / name, state_names=STATES,
                                   control_names=CONTROLS, channel_names=("speed",),
                                   chunk_rows=chunk_rows)
            for index, t in enumerate(times):
                recorder.append(t, index, [t, t, t], [1, 1, 1], [0, 0], {"speed": [t]})
            paths.append(recorder.close(created_utc="x"))
        self.assertEqual(paths[0].read_bytes(), paths[1].read_bytes())


class RunRecordValidationTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def rewrite(self, path, member, replacement):
        """Rewrite one npy member of a record with different array content."""
        out = Path(self.directory.name) / "edited.npz"
        with zipfile.ZipFile(path) as source, zipfile.ZipFile(out, "w") as target:
            for item in source.infolist():
                data = source.read(item.filename)
                if item.filename == member:
                    data = replacement(data)
                target.writestr(item, data)
        return out

    def test_v6_nan_state_is_rejected(self):
        recorder = RunRecorder(Path(self.directory.name) / "nan.npz", state_names=STATES)
        recorder.append(0.0, 0, [0, 0, 0])
        recorder.append(0.1, 1, [0, float("nan"), 0])
        with self.assertRaises(RunRecordError) as raised:
            load_run_record(recorder.close())
        self.assertEqual(raised.exception.check, "V6")

    def test_v2_truncated_array_is_rejected(self):
        path = make_record(self.directory.name)
        broken = self.rewrite(path, "state.npy", lambda data: data[:-40])
        with self.assertRaises(RunRecordError) as raised:
            load_run_record(broken)
        self.assertEqual(raised.exception.check, "V2")

    def test_v2_row_count_disagreeing_with_manifest(self):
        path = make_record(self.directory.name)
        def shorter(data):
            buffer = __import__("io").BytesIO(data)
            array = np.load(buffer)[:-1]
            out = __import__("io").BytesIO()
            np.save(out, array)
            return out.getvalue()
        broken = self.rewrite(path, "time.npy", shorter)
        with self.assertRaises(RunRecordError) as raised:
            load_run_record(broken)
        self.assertEqual(raised.exception.check, "V2")

    def test_v3_time_must_increase(self):
        recorder = RunRecorder(Path(self.directory.name) / "t.npz", state_names=STATES)
        recorder.append(0.0, 0, [0, 0, 0])
        recorder.append(0.0, 1, [0, 0, 0])
        with self.assertRaises(RunRecordError) as raised:
            load_run_record(recorder.close())
        self.assertEqual(raised.exception.check, "V3")

    def test_v1_missing_manifest_and_bad_schema(self):
        path = make_record(self.directory.name)
        no_manifest = Path(self.directory.name) / "none.npz"
        with zipfile.ZipFile(path) as source, zipfile.ZipFile(no_manifest, "w") as target:
            for item in source.infolist():
                if item.filename != "manifest.json":
                    target.writestr(item, source.read(item.filename))
        with self.assertRaises(RunRecordError) as raised:
            load_run_record(no_manifest)
        self.assertEqual(raised.exception.check, "V1")
        bad = self.rewrite(path, "manifest.json",
                           lambda data: data.replace(b"run-record-v1", b"run-record-v9"))
        with self.assertRaises(RunRecordError) as raised:
            load_run_record(bad)
        self.assertEqual(raised.exception.check, "V1")

    def test_v10_changed_source_is_detected_unless_verification_is_off(self):
        source = Path(self.directory.name) / "model.xml"
        source.write_text("<a/>")
        path = make_record(self.directory.name, sources={"model.xml": sha256_of(source)})
        self.assertEqual(load_run_record(path, root=self.directory.name).rows, 3)
        source.write_text("<b/>")
        with self.assertRaises(RunRecordError) as raised:
            load_run_record(path, root=self.directory.name)
        self.assertEqual(raised.exception.check, "V10")
        self.assertEqual(load_run_record(path, verify_sources=False).rows, 3)
        source.unlink()
        with self.assertRaises(RunRecordError):
            load_run_record(path, root=self.directory.name)

    def test_v11_coarse_flag(self):
        coarse = load_run_record(make_record(self.directory.name, maximum_step=0.02, name="c.npz"))
        fine = load_run_record(make_record(self.directory.name, maximum_step=0.005, name="f.npz"))
        self.assertTrue(coarse.manifest.coarse_for_120_fps)
        self.assertFalse(fine.manifest.coarse_for_120_fps)

    def test_v12_two_captures_are_byte_identical(self):
        first = make_record(self.directory.name, name="one.npz")
        second = make_record(self.directory.name, name="two.npz")
        self.assertEqual(first.read_bytes(), second.read_bytes())

    def test_empty_run_is_rejected(self):
        recorder = RunRecorder(Path(self.directory.name) / "e.npz", state_names=STATES)
        with self.assertRaises(RunRecordError):
            recorder.close()


class ResampleTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def test_grid_is_uniform_and_inside_the_record(self):
        record = load_run_record(make_record(self.directory.name, precision="float64"))
        frames = resample_run_record(record, 2.0)
        np.testing.assert_allclose(frames.time, [0.0, 0.5, 1.0])
        self.assertEqual(frames.frames, 3)
        grid = frame_times(0.0, 73.563, 120.0)
        self.assertEqual(grid.size, int(np.floor(73.563 * 120)) + 1)
        self.assertLessEqual(grid[-1], 73.563)
        np.testing.assert_allclose(np.diff(grid), 1 / 120)

    def test_held_quantities_use_the_last_knot_at_or_before_the_frame(self):
        record = load_run_record(make_record(self.directory.name, precision="float64"))
        frames = resample_run_record(record, 2.0)
        self.assertEqual(frames.source_step.tolist(), [0, 1, 2])
        np.testing.assert_allclose(frames.channel_values("speed"), [0.0, 0.8, 2.0])
        np.testing.assert_allclose(frames.control[:, 1], [0.0, 4.0, 10.0])

    def test_source_step_is_nondecreasing_and_never_in_the_future(self):
        record = load_run_record(make_record(self.directory.name, times=np.linspace(0, 2, 9),
                                             precision="float64"))
        frames = resample_run_record(record, 120.0)
        self.assertTrue(np.all(np.diff(frames.source_step) >= 0))
        self.assertTrue(np.all(record.time[frames.source_step] <= frames.time + 1e-12))

    def test_state_at_a_knot_equals_the_knot_exactly(self):
        times = (0.0, 0.25, 0.75, 1.0)
        record = load_run_record(make_record(self.directory.name, times=times, precision="float64"))
        frames = resample_run_record(record, 4.0)
        # The 4 fps grid is 0, .25, .5, .75, 1: frames 0, 1, 3, 4 sit on the four knots.
        np.testing.assert_allclose(frames.time, [0.0, 0.25, 0.5, 0.75, 1.0])
        np.testing.assert_array_equal(frames.state[[0, 1, 3, 4]], record.state)

    def test_cubic_is_reproduced_with_recorded_slopes(self):
        times = (0.0, 0.1, 0.35, 0.4, 0.9, 1.0)
        record = load_run_record(make_record(self.directory.name, times=times, precision="float64"))
        frames = resample_run_record(record, 100.0)
        t = frames.time
        np.testing.assert_allclose(frames.state[:, 0], t**3 - t, atol=1e-9)
        np.testing.assert_allclose(frames.state[:, 1], 2 * t, atol=1e-9)

    def test_no_overshoot_between_equal_knots_without_recorded_rates(self):
        recorder = RunRecorder(Path(self.directory.name) / "n.npz", state_names=("x",),
                               precision="float64")
        for index, (t, x) in enumerate(zip((0, 1, 2, 3), (0, 1, 1, 2))):
            recorder.append(float(t), index, [float(x)])
        record = load_run_record(recorder.close())
        frames = resample_run_record(record, 50.0)
        middle = frames.state[(frames.time >= 1) & (frames.time <= 2), 0]
        self.assertTrue(np.all(np.abs(middle - 1.0) < 1e-12))
        self.assertTrue(np.all((frames.state >= 0) & (frames.state <= 2)))

    def test_invalid_channel_stays_invalid_when_held(self):
        recorder = RunRecorder(Path(self.directory.name) / "i.npz", state_names=("x",),
                               channel_names=("speed",))
        recorder.append(0.0, 0, [0.0], channels={"speed": [1.0]})
        recorder.append(1.0, 1, [1.0], channels={"speed": [float("nan")]})
        recorder.append(2.0, 2, [2.0], channels={"speed": [3.0]})
        frames = resample_run_record(load_run_record(recorder.close()), 2.0)
        self.assertEqual(frames.channel_validity("speed").tolist(), [True, True, False, False, True])

    def test_range_outside_the_record_is_rejected(self):
        record = load_run_record(make_record(self.directory.name))
        with self.assertRaises(ValueError):
            resample_run_record(record, 120.0, t_end=5.0)

    def test_unpack_validity_round_trip(self):
        valid = np.random.default_rng(0).random((5, 11)) > 0.5
        from simplified_vehicle_models.results.run_record import pack_validity
        np.testing.assert_array_equal(unpack_validity(pack_validity(valid), 11), valid)


if __name__ == "__main__":
    unittest.main()
