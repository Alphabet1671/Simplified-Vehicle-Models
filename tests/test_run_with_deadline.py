import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from tools.run.run_with_deadline import run_with_deadline


class DeadlineTests(unittest.TestCase):
    def test_exit_status_is_preserved(self):
        self.assertEqual(run_with_deadline([sys.executable,'-c','raise SystemExit(7)'],seconds=2),7)

    def test_watchdog_marks_child_for_same_group_suite_execution(self):
        code='import os; raise SystemExit(0 if os.environ.get("YAW_PROCESS_WATCHDOG_ACTIVE")=="1" else 1)'
        self.assertEqual(run_with_deadline([sys.executable,'-c',code],seconds=2),0)

    def test_oversize_and_invalid_limits_are_rejected(self):
        for seconds in (0.,-1.,float('nan'),1801.):
            with self.subTest(seconds=seconds),self.assertRaises(ValueError):
                run_with_deadline([sys.executable,'-c','pass'],seconds=seconds)

    def test_timeout_kills_descendants_not_just_parent(self):
        with tempfile.TemporaryDirectory() as directory:
            marker=Path(directory)/'escaped.txt'
            grandchild='import time,pathlib; time.sleep(.8); pathlib.Path('+repr(str(marker))+').write_text("escaped")'
            parent='import subprocess,sys,time; subprocess.Popen([sys.executable,"-c",'+repr(grandchild)+']); time.sleep(30)'
            result=run_with_deadline([sys.executable,'-c',parent],seconds=.25)
            self.assertEqual(result,124)
            # If only the direct parent died, the grandchild would write this.
            time.sleep(1.)
            self.assertFalse(marker.exists())
