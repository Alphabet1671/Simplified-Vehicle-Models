import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class VehicleGuiDocsTests(unittest.TestCase):
    def test_the_guide_names_the_start_command(self):
        text = (ROOT / "docs" / "dev" / "vehicle-gui.md").read_text(encoding="utf-8")
        self.assertIn("python3 tools/vehicle_gui/app.py", text)

    def test_tools_layout_lists_the_package_and_links_the_guide(self):
        text = (ROOT / "docs" / "dev" / "tools-layout.md").read_text(encoding="utf-8")
        self.assertRegex(text, r"\| `tools/vehicle_gui/` \|[^\n]*\[vehicle-gui\.md\]\(vehicle-gui\.md\)")

    def test_the_default_output_folder_is_ignored(self):
        lines = (ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()
        self.assertIn("/vehicles_out/", lines)

    def test_the_guide_matches_reviewed_save_and_revert_behavior(self):
        text = (ROOT / "docs" / "dev" / "vehicle-gui.md").read_text(encoding="utf-8")
        self.assertIn("It does not restore a previous component choice.", text)
        self.assertIn("This file is always written.", text)
        self.assertIn("The save bar's file name sets the output stem.", text)
        self.assertIn("The model id is stored inside that file.", text)
        self.assertIn("The `models/sweeps/` folder is exempt.", text)
        self.assertNotIn("und[o]es the edits and the swap", text)
        self.assertNotIn("when it or a component below it changed", text)

    def test_the_guide_splits_the_save_failure_warning(self):
        text = (ROOT / "docs" / "dev" / "vehicle-gui.md").read_text(encoding="utf-8")
        self.assertIn("If rollback itself fails, cleanup continues. The original error remains raised.", text)


if __name__ == "__main__":
    unittest.main()
