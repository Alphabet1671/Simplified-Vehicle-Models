"""Report the Python environment actually executing the no-TV tools."""
from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
import platform
import re
import sys
from pathlib import Path

TESTED_PYTHON = (3, 13)
TESTED_VERSIONS = {"numpy": "2.5.4", "scipy": "1.17.1", "casadi": "3.8.1"}


def _version(name: str, importer=importlib.import_module, metadata=importlib.metadata) -> dict:
    """Combine installed distribution metadata with the module really imported."""
    try:
        installed = metadata.version(name)
    except metadata.PackageNotFoundError:
        installed = None
    module_name = "casadi" if name == "casadi" else name
    try:
        module = importer(module_name)
    except (ImportError, OSError) as exc:
        return {"installed_version": installed, "executed_version": None,
                "module_file": None, "error": f"{type(exc).__name__}: {exc}"}
    executed = getattr(module, "__version__", None)
    return {"installed_version": installed, "executed_version": str(executed) if executed is not None else None,
            "module_file": str(getattr(module, "__file__", "")) or None, "error": None}


def collect_runtime(*, importer=importlib.import_module, metadata=importlib.metadata,
                    py_version=None, implementation=None, system_platform=None, machine=None) -> dict:
    """Return actual interpreter, platform, distribution, and imported-module facts."""
    version_info = sys.version_info if py_version is None else py_version
    return {
        "python": {"version": sys.version, "version_info": list(version_info[:3]),
                   "implementation": implementation or platform.python_implementation(),
                   "executable": sys.executable},
        "platform": {"platform": system_platform or platform.platform(),
                     "system": platform.system(), "machine": machine or platform.machine()},
        "packages": {name: _version(name, importer, metadata) for name in TESTED_VERSIONS},
    }


def _numeric_version(value: str | None):
    if not value or not re.fullmatch(r"\d+(?:\.\d+)*", value):
        return None
    return tuple(int(part) for part in value.split("."))


def check_runtime(*, require_exact=False, **collect_args) -> dict:
    """Collect a manifest and evaluate documented minimum/exact compatibility."""
    report = collect_runtime(**collect_args)
    problems = []
    py = tuple(report["python"]["version_info"][:2])
    if py < (3, 12):
        problems.append("Python >= 3.12 is required")
    for package, minimum, maximum in (("numpy", (2,), None), ("scipy", (1, 11), None),
                                     ("casadi", (3, 6), (4,))):
        item = report["packages"][package]
        actual = item["executed_version"]
        numeric = _numeric_version(actual)
        if not actual:
            problems.append(f"{package} could not be imported")
        elif numeric is None:
            problems.append(f"{package} version {actual!r} is not a supported numeric version")
        else:
            if item["installed_version"] != actual:
                problems.append(f"{package} installed metadata {item['installed_version']!r} differs from executed module {actual!r}")
            if numeric < minimum or (maximum is not None and numeric >= maximum):
                problems.append(f"{package} {actual} is outside the supported range")
        if require_exact:
            if py != TESTED_PYTHON:
                problems.append("exact runtime requires Python 3.13")
            if actual != TESTED_VERSIONS[package]:
                problems.append(f"exact runtime requires {package}=={TESTED_VERSIONS[package]}")
    report["compatibility"] = {"compatible": not problems, "require_exact": bool(require_exact),
                                "problems": problems}
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-exact", action="store_true", help="require the documented tested versions")
    parser.add_argument("--output", type=Path, help="write JSON to a new path (never overwrite)")
    args = parser.parse_args(argv)
    report = check_runtime(require_exact=args.require_exact)
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        try:
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(rendered)
        except FileExistsError:
            parser.error(f"refusing to overwrite existing output: {args.output}")
    print(rendered, end="")
    return 0 if report["compatibility"]["compatible"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
