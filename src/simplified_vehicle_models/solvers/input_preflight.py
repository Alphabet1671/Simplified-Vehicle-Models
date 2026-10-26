"""Bounded filesystem admission checks for worker-owned model inputs."""
from __future__ import annotations

import os
import stat
from pathlib import Path


def preflight_inputs(inputs):
    """Return the first typed unavailable-input diagnostic, or ``None``.

    Only metadata operations are performed; contents and hashes remain the
    worker's normal responsibility after admission.
    """
    for role, supplied in inputs:
        path = Path(supplied)
        try:
            info = path.stat()
        except FileNotFoundError:
            cause = "not_found"
        except PermissionError:
            cause = "permission_denied"
        except OSError as error:
            cause = f"filesystem_error:{type(error).__name__}"
        else:
            if not stat.S_ISREG(info.st_mode):
                cause = "wrong_kind"
            else:
                continue
        display_path = os.path.abspath(os.path.expanduser(os.fspath(path)))
        return {"status": "missing_input" if cause == "not_found" else "setup_unavailable",
                "failure_stage": "setup", "solver_attempted": False,
                "accepted": False, "role": str(role),
                "path": display_path, "cause": cause}
    return None
