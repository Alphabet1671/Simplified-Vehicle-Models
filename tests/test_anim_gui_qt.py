import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

try:
    from PyQt6.QtCore import QEventLoop, QTimer
    from PyQt6.QtWidgets import QApplication, QMessageBox
except ImportError:
    QApplication = None


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
def setUpModule():
    global app
    app = QApplication.instance() or QApplication([])


if QApplication is not None:
    from tests.test_animation_cli import record_file
    from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
    from tests.test_run_record_runner import real_child
    from tools.anim_gui import inspect_plan
    from tools.anim_gui.qt.main_window import MainWindow

HAVE_FFMPEG = shutil.which("ffmpeg") is not None


def wait_for_idle(window, seconds=90):
    loop = QEventLoop()
    timer = QTimer()
    timer.timeout.connect(lambda: loop.quit() if window.mode == "idle" else None)
    timer.start(50)
    QTimer.singleShot(int(seconds * 1000), loop.quit)
    loop.exec()
    timer.stop()
    return window.mode == "idle"


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class WindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.synthetic, _ = record_file(cls._tmp.name, BASE + WHEEL + DYNAMIC)
        root, code, _ = real_child("planar-7dof", "--record")
        assert code == 0
        cls.real = root / "run_record.npz"

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def setUp(self):
        self.warnings = []
        original = (QMessageBox.question, QMessageBox.warning, QMessageBox.information)
        QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.StandardButton.Yes)
        QMessageBox.warning = staticmethod(lambda *a, **k: self.warnings.append(a[2]))
        QMessageBox.information = staticmethod(lambda *a, **k: self.warnings.append(a[2]))
        self.addCleanup(lambda: (setattr(QMessageBox, "question", original[0]), setattr(QMessageBox, "warning", original[1]),
                                 setattr(QMessageBox, "information", original[2])))
        self.window = MainWindow()
        self.addCleanup(self.window.close)
        self.out = tempfile.TemporaryDirectory()
        self.addCleanup(self.out.cleanup)

    def open(self, path):
        self.window.record.setText(str(path))
        self.window.load_record()

    def test_a_good_record_shows_the_plan_and_enables_export(self):
        self.open(self.synthetic)
        self.assertIn("frames: 73 at 120 fps", self.window.plan_text.toPlainText())
        self.assertTrue(self.window.export_button.isEnabled())
        self.assertEqual(self.window.problem.text(), "")
        self.assertTrue(self.window.output.text().endswith(".mp4") or self.window.current_format() == "png")

    def test_a_bad_file_shows_the_message_and_disables_export(self):
        bad = Path(self.out.name) / "bad.npz"
        bad.write_bytes(b"nope")
        self.open(bad)
        self.assertIn("not a valid run record", self.window.problem.text())
        self.assertFalse(self.window.export_button.isEnabled())

    def test_changing_the_range_updates_the_plan(self):
        self.open(self.synthetic)
        self.window.use_range.setChecked(True)
        self.window.start.setValue(0.1)
        self.window.end.setValue(0.3)
        self.window.refresh_plan()
        self.assertIn("frames: 25 at 120 fps", self.window.plan_text.toPlainText())

    def test_odd_size_is_explained_not_exported(self):
        self.open(self.synthetic)
        self.window.width.setValue(641)
        self.window.refresh_plan()
        self.assertIn("even", self.window.problem.text())
        self.assertFalse(self.window.export_button.isEnabled())

    def test_without_ffmpeg_mp4_is_disabled_and_png_is_chosen(self):
        with mock.patch.object(inspect_plan, "find_ffmpeg", side_effect=inspect_plan.EncoderUnavailable("ffmpeg was not found on PATH")):
            window = MainWindow()
            self.addCleanup(window.close)
        self.assertFalse(window.format.model().item(window.format.findData("mp4")).isEnabled())
        self.assertEqual(window.current_format(), "png")
        self.assertIn("MP4 export is off", window.encoder_note.text())

    def test_study_lists_cases(self):
        import json
        study = Path(self.out.name) / "study.json"
        shutil.copy(self.real, Path(self.out.name) / "run_record.npz")
        study.write_text(json.dumps({"schema": "sim-study-v1", "cases": [
            {"id": "a", "label": "Case A", "run_record": "run_record.npz"}, {"id": "b", "label": "B", "run_record": None}]}))
        self.assertTrue(self.window.open_study(study))
        self.assertEqual(self.window.cases.count(), 1)
        self.assertEqual(self.window.record.text(), str(Path(self.out.name, "run_record.npz").resolve()))
        self.assertTrue(self.window.export_button.isEnabled())

    def test_png_export_of_a_real_record(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        self.window.output.setText(str(Path(self.out.name) / "frames"))
        self.window.width.setValue(320)
        self.window.height.setValue(240)
        self.window.rate.setValue(20.0)
        self.window.refresh_plan()
        self.window.start_export()
        self.assertEqual(self.window.mode, "export")
        self.assertFalse(self.window.export_button.isEnabled())
        self.assertTrue(self.window.cancel_button.isEnabled())
        self.assertTrue(wait_for_idle(self.window), self.window.log.toPlainText())
        self.assertGreater(len(list((Path(self.out.name) / "frames").glob("frame_*.png"))), 5)
        self.assertEqual(self.window.bar.value(), 1000)
        self.assertIn("Done", self.window.statusBar().currentMessage())

    def test_cancel_removes_the_unfinished_frames(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        frames = Path(self.out.name) / "frames"
        self.window.output.setText(str(frames))
        self.window.width.setValue(1280)
        self.window.height.setValue(720)
        self.window.rate.setValue(120.0)
        self.window.refresh_plan()
        self.window.start_export()
        loop = QEventLoop()
        QTimer.singleShot(2500, loop.quit)
        loop.exec()
        self.window.cancel()
        self.assertTrue(wait_for_idle(self.window, 30))
        self.assertFalse(frames.exists())
        self.assertIn("Cancelled", self.window.statusBar().currentMessage())
        self.assertTrue(self.window.export_button.isEnabled())

    def test_existing_empty_output_requires_two_confirmations(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        target = Path(self.out.name) / "empty-frames"
        target.mkdir()
        self.window.output.setText(str(target))
        answers = iter([QMessageBox.StandardButton.Yes, QMessageBox.StandardButton.No])
        original = QMessageBox.question
        QMessageBox.question = staticmethod(lambda *a, **k: next(answers))
        self.addCleanup(lambda: setattr(QMessageBox, "question", original))
        self.window.start_export()
        self.assertEqual(self.window.mode, "idle")
        self.assertTrue(target.is_dir())

    def test_models_output_requires_explicit_protection_confirmation(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        target = Path(__file__).resolve().parents[1] / "models" / "__anim_gui_test_frames__"
        self.window.output.setText(str(target))
        original = QMessageBox.question
        QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.StandardButton.No)
        self.addCleanup(lambda: setattr(QMessageBox, "question", original))
        self.window.start_export()
        self.assertEqual(self.window.mode, "idle")
        self.assertFalse(target.exists())

    def test_relative_record_defaults_are_repo_rooted(self):
        record = Path(self.out.name) / "run_record.npz"
        shutil.copy(self.real, record)
        self.window.record.setText("run_record.npz")
        with mock.patch("tools.anim_gui.qt.main_window.REPOSITORY_ROOT", Path(self.out.name)):
            with mock.patch("tools.anim_gui.qt.main_window.Path.cwd", return_value=Path(self.out.name) / "other"):
                self.window.load_record()
        self.assertEqual(Path(self.window.output.text()).parent, (Path(__file__).resolve().parents[1] / "videos").resolve())

    def test_relative_stl_is_rooted_in_plan_and_preview_settings(self):
        self.open(self.real)
        self.window.stl_body.setText("shapes/body.stl")
        with mock.patch("tools.anim_gui.qt.main_window.REPOSITORY_ROOT", Path(self.out.name)):
            settings = self.window.current_settings()
            with mock.patch("tools.anim_gui.qt.main_window.inspect_plan.inspect", return_value=self.window.summary) as inspect:
                self.window.refresh_plan()
            with mock.patch.object(self.window, "_run") as run:
                self.window.start_preview()
        expected = (Path(self.out.name) / "shapes/body.stl").resolve()
        self.assertEqual(settings.stl_body, expected)
        self.assertEqual(inspect.call_args.args[0].stl_body, expected)
        self.assertEqual(run.call_args.args[0].stl_body, expected)

    def test_relative_output_is_rooted_at_repository(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        relative = "videos/__anim_gui_relative_test__"
        self.window.output.setText(relative)
        with mock.patch("tools.anim_gui.qt.main_window.REPOSITORY_ROOT", Path(self.out.name)):
            with mock.patch("tools.anim_gui.qt.main_window.Path.cwd", return_value=Path(self.out.name) / "other"):
                with mock.patch.object(self.window, "_run") as run:
                    self.window.start_export()
        run.assert_called_once()
        actual = run.call_args.args[0]
        self.assertEqual(actual.output, (Path(self.out.name) / relative).resolve())

    def test_ancestor_symlink_outputs_are_refused(self):
        self.open(self.real)
        for fmt, suffix in (("png", "frames"), ("mp4", "video.mp4")):
            with self.subTest(fmt=fmt):
                self.window.format.setCurrentIndex(self.window.format.findData(fmt))
                real_dir = Path(self.out.name) / f"real-{fmt}"
                real_dir.mkdir(exist_ok=True)
                link = Path(self.out.name) / f"link-{fmt}"
                link.symlink_to(real_dir, target_is_directory=True)
                self.window.output.setText(str(link / suffix))
                self.window.start_export()
                self.assertEqual(self.window.mode, "idle")
                self.assertFalse((real_dir / suffix).exists())

    def test_dangling_ancestor_symlink_is_refused(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        link = Path(self.out.name) / "dangling"
        link.symlink_to(Path(self.out.name) / "missing", target_is_directory=True)
        self.window.output.setText(str(link / "frames"))
        self.window.start_export()
        self.assertEqual(self.window.mode, "idle")

    def test_symlink_output_is_refused(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        target = Path(self.out.name) / "frames"
        real_target = Path(self.out.name) / "real-frames"
        real_target.mkdir()
        target.symlink_to(real_target, target_is_directory=True)
        self.window.output.setText(str(target))
        self.window.start_export()
        self.assertEqual(self.window.mode, "idle")
        self.assertTrue(real_target.is_dir())

    def test_failed_export_preserves_preexisting_png_frames(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        frames = Path(self.out.name) / "existing-frames"
        frames.mkdir()
        sentinel = frames / "frame_000001.png"
        sentinel.write_bytes(b"existing")
        self.window.output.setText(str(frames))
        self.window._active_output = frames
        self.window._preexisting_output = True
        self.window.mode = "export"
        self.window._finished(2, "done")
        self.assertTrue(sentinel.exists())
        self.assertTrue(frames.exists())

    def test_nonzero_done_export_removes_only_partial_outputs(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("mp4"))
        output = Path(self.out.name) / "result.mp4"
        partial = output.with_name("result.partial.mp4")
        partial.write_bytes(b"partial")
        unrelated = Path(self.out.name) / "keep.txt"
        unrelated.write_text("keep")
        self.window.output.setText(str(output))
        self.window._active_output = output
        self.window.mode = "export"
        self.window._finished(2, "done")
        self.assertFalse(partial.exists())
        self.assertTrue(unrelated.exists())
        self.assertEqual(self.window.mode, "idle")

    def test_the_synthetic_record_fails_with_the_exporters_message(self):
        # A synthetic record names no track, so a real render exits 2 and the GUI must say why.
        self.open(self.synthetic)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        self.window.output.setText(str(Path(self.out.name) / "frames"))
        self.window.start_export()
        self.assertTrue(wait_for_idle(self.window))
        self.assertIn("refused", self.window.statusBar().currentMessage())
        self.assertIn("cannot load the track", self.window.statusBar().currentMessage())

    def test_preview_shows_a_still(self):
        self.open(self.real)
        self.window.time_slider.setValue(500)
        self.window.start_preview()
        self.assertEqual(self.window.mode, "preview")
        self.assertTrue(wait_for_idle(self.window), self.window.log.toPlainText())
        pixmap = self.window.picture.pixmap()
        self.assertFalse(pixmap is None or pixmap.isNull(), self.window.picture.text())

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg is not installed")
    def test_mp4_export_and_cancel_leave_no_partial_file(self):
        self.open(self.real)
        video = Path(self.out.name) / "v.mp4"
        self.window.format.setCurrentIndex(self.window.format.findData("mp4"))
        self.window.output.setText(str(video))
        self.window.width.setValue(320)
        self.window.height.setValue(240)
        self.window.rate.setValue(30.0)
        self.window.refresh_plan()
        self.window.start_export()
        self.assertTrue(wait_for_idle(self.window), self.window.log.toPlainText())
        self.assertTrue(video.is_file() and video.stat().st_size > 0)
        self.assertFalse(video.with_name("v.partial.mp4").exists())

    def test_app_main_quits_after_the_hook(self):
        from tools.anim_gui import app as app_module
        os.environ["ANIM_GUI_QUIT_AFTER_MS"] = "50"
        self.addCleanup(os.environ.pop, "ANIM_GUI_QUIT_AFTER_MS", None)
        self.assertEqual(app_module.main([str(self.synthetic)]), 0)


if __name__ == "__main__":
    unittest.main()
