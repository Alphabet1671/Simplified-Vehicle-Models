"""Start the animation GUI: ``PYTHONPATH=src:. python3 tools/anim_gui/app.py [run_record.npz]``."""
from __future__ import annotations

import os
import sys
from pathlib import Path

if __package__ in {None, ""}:
    ROOT = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(ROOT / "src"), str(ROOT)]

from PyQt6.QtCore import QTimer  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

from tools.anim_gui.qt.main_window import MainWindow  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    application = QApplication.instance() or QApplication([sys.argv[0], *arguments])
    application.setApplicationName("Animation GUI")
    window = MainWindow()
    if arguments:
        window.record.setText(arguments[0])
        window.load_record()
    window.show()
    delay = os.environ.get("ANIM_GUI_QUIT_AFTER_MS")
    if delay:
        QTimer.singleShot(int(delay), application.quit)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
