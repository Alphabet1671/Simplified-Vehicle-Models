"""The canonical ten-lap MIS track is the active default; historical replay stays old."""
import ast
from pathlib import Path
import unittest

from tools.reports import plot_iem24_endurance_correlation as endurance_plot
from tools.run.run_iem24_forward_driver import DEFAULT_TRACK, LEGACY_TRACK

ROOT = Path(__file__).resolve().parents[1]
CANONICAL = "models/tracks/2024_michigan_endurance_multilap_closed.xml"
ORIGINAL = "models/tracks/2024_michigan_endurance_closed.xml"


def source(name):
    (path,) = (ROOT / "tools").glob(f"*/{name}")
    return path.read_text(encoding="utf-8")


def track_option_defaults(name):
    """Literal defaults of every ``--track`` add_argument call in a tool."""
    tree = ast.parse(source(name))
    return [ast.literal_eval(keyword.value.args[0])
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "add_argument"
            and node.args and ast.literal_eval(node.args[0]) == "--track"
            for keyword in node.keywords if keyword.arg == "default"]


class DefaultTrackTests(unittest.TestCase):
    def test_new_animation_tool_basenames_are_glob_unambiguous(self):
        animation_files = sorted((ROOT / "tools" / "anim_gui").glob("*.py"))
        all_tool_files = [path for path in (ROOT / "tools").glob("*") if path.is_dir()
                          for path in path.glob("*.py")]
        animation_files = [path for path in animation_files if path.name != "__init__.py"]
        basenames = {}
        for path in all_tool_files:
            basenames.setdefault(path.name, []).append(path)
        for path in animation_files:
            self.assertEqual(basenames[path.name], [path], path.name)

    def test_runner_default_is_canonical_and_legacy_is_original(self):
        self.assertEqual(str(DEFAULT_TRACK), CANONICAL)
        self.assertEqual(str(LEGACY_TRACK), ORIGINAL)
        self.assertTrue((ROOT / CANONICAL).is_file())

    def test_stint_and_three_way_comparisons_default_to_canonical(self):
        for name in ("compare_iem24_murphy_stint.py", "compare_iem24_three_way.py"):
            self.assertEqual(track_option_defaults(name), [CANONICAL], name)

    def test_point_mass_comparison_and_speed_cap_use_canonical(self):
        self.assertIn(f'TRACK = ROOT / "{CANONICAL}"', source("compare_planar_point_mass.py"))
        self.assertIn(f'TRACK = "{CANONICAL}"', source("diagnose_speed_cap.py"))

    def test_endurance_plot_defaults_canonical_and_rejects_mismatched_export(self):
        self.assertEqual(str(endurance_plot.DEFAULT_TRACK), CANONICAL)
        endurance_plot.check_export_track({"track": CANONICAL}, endurance_plot.DEFAULT_TRACK)
        endurance_plot.check_export_track({"track": ORIGINAL}, Path(ORIGINAL))
        # An export with no recorded track is a historical (original-track) run.
        endurance_plot.check_export_track({}, Path(ORIGINAL))
        with self.assertRaises(ValueError):
            endurance_plot.check_export_track({}, endurance_plot.DEFAULT_TRACK)
        with self.assertRaises(ValueError):
            endurance_plot.check_export_track({"track": ORIGINAL}, endurance_plot.DEFAULT_TRACK)

    def test_attempt_consumers_use_recorded_track_with_original_fallback(self):
        for name in ("recompute_forward_driver_accelerations.py",
                     "plot_iem24_forward_driver_comparison.py"):
            text = source(name)
            self.assertIn(f'LEGACY_TRACK = "{ORIGINAL}"', text, name)
            self.assertIn('load_track(settings.get("track") or LEGACY_TRACK)', text, name)
            self.assertNotIn(CANONICAL, text, name)

    def test_trace_analysis_follows_attempt_track(self):
        from tools.diagnostics.analyze_trace_commands import LEGACY_TRACK as legacy, attempt_track
        import json, tempfile
        self.assertEqual(legacy, ORIGINAL)
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "trace.csv"
            self.assertEqual(attempt_track(trace), ORIGINAL)  # no summary: historical
            (Path(directory) / "summary.json").write_text(json.dumps({"settings": {}}))
            self.assertEqual(attempt_track(trace), ORIGINAL)
            (Path(directory) / "summary.json").write_text(
                json.dumps({"settings": {"track": CANONICAL}}))
            self.assertEqual(attempt_track(trace), CANONICAL)
            self.assertEqual(attempt_track(trace, ORIGINAL), ORIGINAL)


if __name__ == "__main__":
    unittest.main()
