"""Launch the shared simulation GUI directly in Maneuvers mode."""
from __future__ import annotations

import os
from pathlib import Path
import sys

if __package__ in {None, ""}:
    _ROOT = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(_ROOT / "src"), str(_ROOT)]


def main(argv=None) -> int:
    """Open the maneuver tab, optionally loading a vehicle model XML."""
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", nargs="?", help="vehicle model XML to open")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    from PyQt6.QtCore import QTimer
    from PyQt6.QtWidgets import QApplication
    from tools.sim_gui.qt.main_window import MainWindow

    application = QApplication.instance() or QApplication(sys.argv)
    application.setApplicationName("Maneuver GUI")
    window = MainWindow()
    window.tabs.setCurrentWidget(window.maneuver_tab)
    if args.model:
        window.open_model(Path(args.model))
    window.show()
    delay = os.environ.get("SIM_GUI_QUIT_AFTER_MS")
    if delay:
        QTimer.singleShot(int(delay), application.quit)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
