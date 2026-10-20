"""Check the additive changes to tools/tracks that the track GUI relies on."""
import inspect
import unittest
from xml.etree import ElementTree

try:
    from tools.tracks import build_closed_track
except ImportError:  # SciPy is an optional extra; the module imports it at the top
    build_closed_track = None
from tools.tracks import open_course
from tools.tracks.open_course import build_course, straight


class OpenCourseTextTests(unittest.TestCase):
    def setUp(self):
        self.course = build_course([straight(10.0)], maximum_spacing=1.0)

    def write(self, **keywords):
        arguments = dict(model_id="t", title="T", csv_name="t.csv", comments=())
        arguments.update(keywords)
        return open_course.xml_text(self.course, **arguments)

    def test_awkward_text_parses(self):
        text = self.write(model_id='a"b&c', title="Tom & Jerry <1>")
        root = ElementTree.fromstring(text)
        self.assertEqual(root.get("id"), 'a"b&c')
        self.assertEqual(root.findtext("./metadata/title"), "Tom & Jerry <1>")

    def test_plain_text_is_unchanged(self):
        self.assertIn('id="t"', self.write())
        self.assertIn("<title>T</title>", self.write())

    def test_a_comment_cannot_close_itself(self):
        for bad in ("a -- b", "ends with -"):
            with self.assertRaises(ValueError):
                self.write(comments=[bad])
        self.assertIn("<!-- fine -->", self.write(comments=["fine"]))


@unittest.skipIf(build_closed_track is None, "SciPy is not installed")
class ClosedTrackKeywordTests(unittest.TestCase):
    def test_defaults_keep_the_committed_values(self):
        parameters = inspect.signature(build_closed_track.write_xml).parameters
        self.assertEqual(parameters["model_id"].default, "2024-michigan-endurance-closed")
        self.assertEqual(parameters["title"].default, "2024 Michigan Endurance Closed")
        self.assertEqual(build_closed_track.TRACK_TITLE, parameters["title"].default)
        self.assertEqual(parameters["model_id"].kind, inspect.Parameter.KEYWORD_ONLY)


if __name__ == "__main__":
    unittest.main()
