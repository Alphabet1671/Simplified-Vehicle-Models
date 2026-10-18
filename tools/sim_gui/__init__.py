"""Simulation GUI core without Qt."""
from __future__ import annotations

import os
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def child_environment() -> dict[str, str]:
    """Return the environment for a repository child process."""
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(REPOSITORY_ROOT / "src"), str(REPOSITORY_ROOT)]
    )
    environment.setdefault(
        "MPLCONFIGDIR", str(Path(os.environ.get("TMPDIR", "/tmp")) / "mplcfg")
    )
    return environment
