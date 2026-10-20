"""Shared pieces of the track, vehicle and animation GUIs.

Everything here is Qt-free except the ``tools.gui_common.qt`` subpackage.
"""
from __future__ import annotations

import os
import tempfile
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def child_environment() -> dict[str, str]:
    """Return the environment for a repository child process."""
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(REPOSITORY_ROOT / "src"), str(REPOSITORY_ROOT)]
    )
    environment.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "mplcfg"))
    return environment
