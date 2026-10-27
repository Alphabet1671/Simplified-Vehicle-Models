"""Launch the native TTC tire correlation GUI."""
from __future__ import annotations
import os,sys
from pathlib import Path
from tools.gui_common import REPOSITORY_ROOT

def main(argv=None):
    try:
        from PyQt6.QtCore import QTimer
        from PyQt6.QtWidgets import QApplication
        from tools.tire_gui.qt.main_window import MainWindow
    except ImportError as exc:
        print("Tire GUI dependencies are missing. Install with: python3 -m pip install -e '.[tire-gui]'",file=sys.stderr)
        return 2
    app=QApplication(argv or sys.argv); window=MainWindow()
    args=(argv or sys.argv)[1:]
    if args: window.open_path(Path(args[0]))
    window.show()
    delay=int(os.environ.get('TIRE_GUI_QUIT_AFTER_MS','0') or 0)
    if delay>0:QTimer.singleShot(delay,app.quit)
    return app.exec()

if __name__=='__main__':raise SystemExit(main())
