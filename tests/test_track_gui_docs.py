import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class TrackGuiDocsTests(unittest.TestCase):
    def test_the_guide_names_the_start_command(self):
        text = (ROOT / "docs" / "dev" / "track-gui.md").read_text(encoding="utf-8")
        self.assertIn("python3 tools/track_gui/app.py", text)

    def test_tools_layout_lists_the_packages_and_links_the_guide(self):
        text = (ROOT / "docs" / "dev" / "tools-layout.md").read_text(encoding="utf-8")
        self.assertRegex(text, r"\| `tools/track_gui/` \|[^\n]*\[track-gui\.md\]\(track-gui\.md\)")
        self.assertRegex(text, r"\| `tools/gui_common/` \|[^\n]*safe file writes")

    def test_the_guide_documents_mode_specific_save_rules_and_radius_limit(self):
        text = (ROOT / "docs" / "dev" / "track-gui.md").read_text(encoding="utf-8")
        self.assertIn("open-course mode", text)
        self.assertIn("closed-track mode", text)
        self.assertIn("does not pass overwrite or generated-file confirmation flags", text)
        self.assertIn("does not constrain the built geometry", text)
        self.assertIn("tests.test_track_gui_docs", text)

    def test_the_default_output_folder_is_ignored(self):
        lines = (ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()
        self.assertIn("/tracks_out/", lines)


if __name__ == "__main__":
    unittest.main()
