#!/usr/bin/env python3
"""Archive retired GUI bridge source outside Git with verified hashes.

The archive is intentionally local/ignored. It preserves the former GUI bridge
while the active simulation GUI dispatches ``tools.sim_gui.run_yaw_case``.
"""
from __future__ import annotations

import argparse
from datetime import UTC, datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RETIRED_FILES = (Path("tools/sim_gui/run_case.py"),)
REASON = "Superseded simulation-GUI child bridge; active GUI dispatches run_yaw_case.py."


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def archive(output_root: Path, *, now: datetime | None = None) -> tuple[Path, Path]:
    output_root = Path(output_root)
    timestamp = (now or datetime.now(UTC)).strftime("%Y%m%dT%H%M%SZ")
    destination = output_root / f"retired-gui-bridge-{timestamp}"
    archive_path = output_root / f"retired-gui-bridge-{timestamp}.zip"
    if destination.exists() or archive_path.exists():
        raise FileExistsError(f"refusing to overwrite archive destination: {destination}")
    sources = []
    for relative in RETIRED_FILES:
        source = REPOSITORY_ROOT / relative
        if not source.is_file():
            raise FileNotFoundError(f"retired source is missing: {relative}")
        sources.append((relative, source, sha256(source), source.stat().st_size))
    destination.mkdir(parents=True)
    tracked = destination / "tracked-files"
    manifest_files = []
    try:
        for relative, source, digest, byte_count in sources:
            copied = tracked / relative
            copied.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, copied)
            if sha256(copied) != digest:
                raise RuntimeError(f"copied archive hash differs: {relative}")
            manifest_files.append({"path": relative.as_posix(), "sha256": digest,
                                   "bytes": byte_count, "reason": REASON})
        head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPOSITORY_ROOT,
                                       text=True).strip()
        branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=REPOSITORY_ROOT,
                                         text=True).strip()
        manifest = {"created_utc": timestamp, "branch": branch, "head": head,
                    "reason": REASON, "files": manifest_files}
        manifest_path = destination / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        with zipfile.ZipFile(archive_path, "x", compression=zipfile.ZIP_DEFLATED) as bundle:
            for member in sorted(path for path in destination.rglob("*") if path.is_file()):
                bundle.write(member, member.relative_to(destination).as_posix())
        with zipfile.ZipFile(archive_path) as bundle:
            expected = {path.relative_to(destination).as_posix() for path in destination.rglob("*") if path.is_file()}
            if set(bundle.namelist()) != expected:
                raise RuntimeError("ZIP members differ from verified archive directory")
            for member in expected:
                staged = destination / member
                if bundle.read(member) != staged.read_bytes():
                    raise RuntimeError(f"ZIP payload differs from verified archive directory: {member}")
            for entry in manifest_files:
                member = f"tracked-files/{entry['path']}"
                if sha256(destination / member) != entry["sha256"]:
                    raise RuntimeError(f"manifest hash differs from staged archive: {entry['path']}")
                if hashlib.sha256(bundle.read(member)).hexdigest() != entry["sha256"]:
                    raise RuntimeError(f"manifest hash differs from ZIP payload: {entry['path']}")
    except Exception:
        shutil.rmtree(destination, ignore_errors=True)
        archive_path.unlink(missing_ok=True)
        raise
    return destination, archive_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=Path("local-backups"))
    args = parser.parse_args(argv)
    destination, bundle = archive(args.output_root)
    print(destination)
    print(bundle)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
