import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import sys
import unittest

try:
    from PyQt6.QtCore import QEventLoop, QTimer
    from PyQt6.QtWidgets import QApplication
except ImportError:
    QApplication = None


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
def setUpModule():
    global app
    app = QApplication.instance() or QApplication([])


if QApplication is not None:
    from tools.gui_common.qt.process import ProcessJob


def run_job(arguments, *, timeout=None, cancel_after_ms=None, wait_ms=20000):
    job = ProcessJob()
    lines, result = [], []
    loop = QEventLoop()
    job.line.connect(lines.append)
    job.finished.connect(lambda code, state: (result.append((code, state)), loop.quit()))
    job.start(arguments, timeout_seconds=timeout)
    if cancel_after_ms is not None:
        QTimer.singleShot(cancel_after_ms, job.cancel)
    QTimer.singleShot(wait_ms, loop.quit)
    loop.exec()
    return job, lines, result


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class ProcessJobTests(unittest.TestCase):
    def test_lines_and_exit_code(self):
        _, lines, result = run_job(["-c", "import sys; print('a'); print('b', file=sys.stderr); sys.exit(3)"])
        self.assertEqual(result, [(3, "done")])
        self.assertEqual(sorted(lines), ["a", "b"])

    def test_carriage_returns_split_lines(self):
        _, lines, result = run_job(["-c", "import sys; sys.stdout.write('x\\ry\\rz'); sys.stdout.flush()"])
        self.assertEqual(lines, ["x", "y", "z"])
        self.assertEqual(result[0][1], "done")

    def test_child_sees_the_repository_on_the_path(self):
        _, lines, _ = run_job(["-c", "import tools.gui_common, simplified_vehicle_models; print('ok')"])
        self.assertEqual(lines, ["ok"])

    def test_cancel_kills_the_child(self):
        job, _, result = run_job(["-c", "import time; time.sleep(30)"], cancel_after_ms=300)
        self.assertEqual(result[0][1], "cancelled")
        self.assertFalse(job.running())

    def test_timeout_kills_the_child(self):
        _, _, result = run_job(["-c", "import time; time.sleep(30)"], timeout=0.3)
        self.assertEqual(result[0][1], "timed_out")

    def test_missing_program_reports_failed_to_start(self):
        job = ProcessJob()
        result, loop = [], QEventLoop()
        job.finished.connect(lambda code, state: (result.append((code, state)), loop.quit()))
        job.start([], program="/nonexistent/program")
        QTimer.singleShot(5000, loop.quit)
        loop.exec()
        self.assertEqual(result, [(-1, "failed_to_start")])

    def test_second_start_while_running_is_refused(self):
        job = ProcessJob()
        job.start(["-c", "import time; time.sleep(5)"])
        with self.assertRaises(RuntimeError):
            job.start(["-c", "pass"])
        job.cancel()
        loop = QEventLoop(); job.finished.connect(lambda *_: loop.quit()); QTimer.singleShot(3000, loop.quit); loop.exec()


if __name__ == "__main__":
    unittest.main()
