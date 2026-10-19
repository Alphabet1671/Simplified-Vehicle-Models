"""Start the simulation GUI application."""
from __future__ import annotations

import os
from pathlib import Path
import sys

if __package__ in {None, ""}:
    _ROOT = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(_ROOT / "src"), str(_ROOT)]

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from tools.sim_gui.qt.main_window import MainWindow


def main(argv=None) -> int:
    """Create the application, open an optional model, and run its event loop."""
    application = QApplication.instance() or QApplication(sys.argv if argv is None else argv)
    application.setApplicationName("Simulation GUI")
    window = MainWindow()
    args = list(sys.argv[1:] if argv is None else argv)
    if args:
        window.open_model(Path(args[0]))
    window.show()
    delay = os.environ.get("SIM_GUI_QUIT_AFTER_MS")
    if delay:
        QTimer.singleShot(int(delay), application.quit)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
