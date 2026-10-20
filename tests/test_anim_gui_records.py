import json
import tempfile
import unittest
from pathlib import Path

from tools.anim_gui import records


class RecordsTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)

    def study(self, cases):
        path = self.root / "study.json"
        path.write_text(json.dumps({"schema": "sim-study-v1", "cases": cases}), encoding="utf-8")
        return path

    def test_study_lists_only_cases_with_a_record(self):
        (self.root / "a").mkdir()
        (self.root / "a" / "run_record.npz").write_bytes(b"x")
        path = self.study([{"id": "c1", "label": "Case one", "run_record": "a/run_record.npz"},
                           {"id": "c2", "label": "Case two", "run_record": None},
                           {"id": "c3", "label": "Gone", "run_record": "missing/run_record.npz"},
                           {"id": "c4", "label": "Escape", "run_record": "../outside.npz"}])
        self.assertEqual([c.label for c in records.study_records(path)], ["Case one"])

    def test_bad_study_files(self):
        bad = self.root / "bad.json"
        bad.write_text("{", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "cannot read"):
            records.study_records(bad)
        bad.write_text(json.dumps({"schema": "other", "cases": []}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "not a sim-study-v1"):
            records.study_records(bad)

    def test_find_records(self):
        for name in ("x/y", "z"):
            (self.root / name).mkdir(parents=True)
            (self.root / name / "run_record.npz").write_bytes(b"x")
        self.assertEqual([c.label for c in records.find_records(self.root)], ["z", "x/y"])

    def test_remove_partials_video(self):
        output = self.root / "v.mp4"
        partial = self.root / "v.partial.mp4"
        partial.write_bytes(b"x")
        output.write_bytes(b"keep")
        self.assertEqual(records.remove_partials(output), [partial])
        self.assertTrue(output.exists() and not partial.exists())
        self.assertEqual(records.remove_partials(output), [])

    def test_remove_partials_png_only_when_the_folder_is_all_frames(self):
        frames = self.root / "frames"
        frames.mkdir()
        (frames / "frame_000000.png").write_bytes(b"x")
        self.assertEqual(records.remove_partials(frames), [frames])
        self.assertFalse(frames.exists())
        mixed = self.root / "mixed"
        mixed.mkdir()
        (mixed / "frame_000000.png").write_bytes(b"x")
        (mixed / "notes.txt").write_text("mine")
        self.assertEqual(records.remove_partials(mixed), [])
        self.assertTrue((mixed / "notes.txt").exists())


if __name__ == "__main__":
    unittest.main()
