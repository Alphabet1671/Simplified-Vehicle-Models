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
        for payload in ({"schema": "other", "cases": []}, [], "text"):
            bad.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "not a sim-study-v1"):
                records.study_records(bad)
        bad.write_text(json.dumps({"schema": "sim-study-v1", "cases": [None]}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "invalid case"):
            records.study_records(bad)
        bad.write_text(json.dumps({"schema": "sim-study-v1", "cases": [{"run_record": 3}]}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "invalid run_record"):
            records.study_records(bad)

    def test_find_records(self):
        for name in ("x/y", "z"):
            (self.root / name).mkdir(parents=True)
            (self.root / name / "run_record.npz").write_bytes(b"x")
        self.assertEqual([c.label for c in records.find_records(self.root)], ["z", "x/y"])
        self.assertEqual(records.find_records(self.root, limit=0), [])
        for limit in (-1, True, 1.5):
            with self.assertRaisesRegex(ValueError, "non-negative integer"):
                records.find_records(self.root, limit=limit)

    def test_remove_partials_video(self):
        output = self.root / "v.mp4"
        partial = self.root / "v.partial.mp4"
        partial.write_bytes(b"x")
        output.write_bytes(b"keep")
        self.assertEqual(records.remove_partials(output), [partial])
        self.assertTrue(output.exists() and not partial.exists())
        self.assertEqual(records.remove_partials(output), [])
        partial.mkdir()
        self.assertEqual(records.remove_partials(output), [])
        self.assertTrue(partial.exists())

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
        nested = self.root / "nested"
        nested.mkdir()
        (nested / "frame_000000.png").mkdir()
        self.assertEqual(records.remove_partials(nested), [])
        self.assertTrue(nested.exists())
        symlinked = self.root / "symlinked"
        symlinked.mkdir()
        target = self.root / "target.txt"
        target.write_text("keep")
        (symlinked / "frame_000000.png").symlink_to(target)
        self.assertEqual(records.remove_partials(symlinked), [])
        self.assertTrue(symlinked.exists() and target.exists())


if __name__ == "__main__":
    unittest.main()
