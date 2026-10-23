"""Capture immutable, actual runtime provenance for no-TV matrix attempts."""
from __future__ import annotations

import hashlib
import platform
import subprocess
import sys
from pathlib import Path
from typing import Callable

import importlib.metadata
import numpy


# These are implementation inputs, rather than incidental files in a checkout.
IMPLEMENTATION_FILES = (
    Path("tools/diagnostics/verify_no_tv_minimum_lap.py"),
    Path("src/simplified_vehicle_models/solvers/qss_spline_ocp.py"),
    Path("src/simplified_vehicle_models/solvers/profile_line_seed.py"),
    Path("src/simplified_vehicle_models/tracks/line_trajectory.py"),
    Path("src/simplified_vehicle_models/solvers/line_validation.py"),
    Path("src/simplified_vehicle_models/formulations/point_mass_capability.py"),
    Path("src/simplified_vehicle_models/formulations/signed_capability.py"),
    Path("src/simplified_vehicle_models/solvers/fixed_path_energy_nlp.py"),
    Path("src/simplified_vehicle_models/solvers/capability_profile.py"),
    Path("tools/diagnostics/no_tv_provenance.py"),
)


def _git(repo: Path, *args: str) -> str:
    return subprocess.check_output(("git", "-C", str(repo), *args), text=True).strip()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def runtime_provenance(repo: Path, *, git: Callable[..., str] = _git) -> dict:
    """Return actual HEAD, tracked dirty manifest, code hashes, and runtimes.

    ``git`` is injectable solely for deterministic tests; production always uses
    the checkout's git executable before any child is spawned.
    """
    root = Path(repo).resolve()
    head = git(root, "rev-parse", "HEAD")
    dirty = git(root, "status", "--porcelain", "--untracked-files=no")
    code_files = [root / relative for relative in IMPLEMENTATION_FILES]
    missing = [str(path) for path in code_files if not path.is_file()]
    if missing:
        raise FileNotFoundError("implementation provenance file missing: " + ", ".join(missing))
    return {
        "git_head": head,
        "dirty_tracked_manifest": dirty.splitlines(),
        "code_sha256": {str(path.relative_to(root)): _sha256(path) for path in code_files},
        "runtime": {
            "python": sys.version,
            "python_implementation": platform.python_implementation(),
            "numpy": numpy.__version__,
            "casadi": _package_version("casadi"),
        },
    }


def resolve_base_commit(repo: Path, supplied: str | None, *, git: Callable[..., str] = _git) -> str:
    """Resolve an optional label and require it to identify the actual HEAD."""
    root = Path(repo).resolve()
    head = git(root, "rev-parse", "HEAD")
    if supplied is None:
        return head
    try:
        resolved = git(root, "rev-parse", "--verify", f"{supplied}^{{commit}}")
    except subprocess.CalledProcessError as error:
        raise ValueError("--base-commit must resolve to the actual current HEAD") from error
    if resolved != head:
        raise ValueError("--base-commit must resolve to the actual current HEAD; use a separate baseline label for history")
    return head


__all__ = ["IMPLEMENTATION_FILES", "resolve_base_commit", "runtime_provenance"]
