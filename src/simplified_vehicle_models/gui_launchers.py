"""Console-script adapters for GUI tools that live in the source checkout.

The GUI tools are intentionally not installed as a ``tools`` package. These
entry points locate their sibling scripts through this package's editable
source location, independent of the launcher's working directory.
"""
from __future__ import annotations

import runpy
import sys
from pathlib import Path


def _tool_script(relative: str) -> Path:
    """Resolve a GUI script from the editable source checkout."""
    source_root = Path(__file__).resolve().parents[2]
    script = source_root / "tools" / relative / "app.py"
    if not script.is_file():
        raise RuntimeError(
            f"GUI source is unavailable at {script}. Install this project in editable "
            "mode from a checkout containing the tools directory, or run the GUI "
            "directly from that checkout."
        )
    return script


def _run(relative: str, argv: list[str] | None = None) -> int:
    script = _tool_script(relative)
    source_root = script.parents[2]
    sys.path[:0] = [str(source_root / "src"), str(source_root)]
    sys.argv = [str(script), *(sys.argv[1:] if argv is None else argv)]
    namespace = runpy.run_path(str(script), run_name="_gui_launcher_app")
    return int(namespace["main"](sys.argv[1:]))


def tire_main() -> int:
    """Entry point for the TTC tire correlation GUI."""
    return _run("tire_gui")


def maneuver_main() -> int:
    """Entry point for the standalone maneuver mode."""
    return _run("maneuver_gui")
