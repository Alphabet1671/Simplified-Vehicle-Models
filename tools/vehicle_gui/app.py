"""Start the vehicle GUI: ``PYTHONPATH=src:. python3 tools/vehicle_gui/app.py [model.xml]``."""
from __future__ import annotations
import os
import sys
from pathlib import Path
if __package__ in {None, ""}:
    ROOT = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(ROOT / "src"), str(ROOT)]
from PyQt6.QtCore import QTimer  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402
from tools.vehicle_gui.qt.main_window import MainWindow  # noqa: E402

def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    application = QApplication.instance() or QApplication([sys.argv[0], *arguments])
    application.setApplicationName("Vehicle GUI")
    window = MainWindow()
    if arguments: window.open_path(Path(arguments[0]))
    window.show()
    delay = os.environ.get("VEHICLE_GUI_QUIT_AFTER_MS")
    if delay: QTimer.singleShot(int(delay), application.quit)
    return application.exec()
if __name__ == "__main__": raise SystemExit(main())
