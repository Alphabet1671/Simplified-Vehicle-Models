"""Launch the native TTC tire correlation GUI."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

if __package__ in {None, ""}:
    _ROOT = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(_ROOT / "src"), str(_ROOT)]


def main(argv=None) -> int:
    """Create the tire GUI and optionally open an input file."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", help="steady or self-contained transient tire XML to open")
    parser.add_argument("--steady", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    if args.steady is not None:
        parser.error('--steady is obsolete: transient tire XMLs are self-contained; pass one XML path')
    try:
        from PyQt6.QtCore import QTimer
        from PyQt6.QtWidgets import QApplication
        from tools.tire_gui.qt.main_window import MainWindow
    except ImportError:
        print(
            "Tire GUI dependencies are missing. Install with: "
            "python3 -m pip install -e '.[tire-gui]'",
            file=sys.stderr,
        )
        return 2
    app = QApplication.instance() or QApplication(sys.argv)
    window = MainWindow()
    if args.path:
        if not window.open_path(Path(args.path)):
            print(window.status.text(), file=sys.stderr)
            window.deleteLater()
            return 2
    window.show()
    delay = int(os.environ.get("TIRE_GUI_QUIT_AFTER_MS", "0") or 0)
    if delay > 0:
        QTimer.singleShot(delay, app.quit)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
