"""Run simulation GUI studies without Qt."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
import json
from pathlib import Path
import subprocess
import threading
import time
import os
from typing import Callable

from . import REPOSITORY_ROOT, child_environment
from .execute import (PreparedCase, child_command, finish_case, new_attempt,
                      prepare_case, validate_case)
from .study import StudyDefinition, StudyIndex, expand

class _ProcessRegistry:
    """Live child processes and the stop flag of one run_study call."""

    def __init__(self) -> None:
        self._processes = set()
        self._lock = threading.Lock()
        self.stop = threading.Event()

    def add(self, process) -> None:
        """Track a child; terminate it at once if the run is already stopping."""
        with self._lock:
            self._processes.add(process)
            stopping = self.stop.is_set()
        if stopping and process.poll() is None:
            # A worker can start its child after terminate_all() took its snapshot.
            # Reap it here so pool.shutdown() cannot wait for the full run timeout.
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def discard(self, process) -> None:
        with self._lock:
            self._processes.discard(process)

    def terminate_all(self) -> None:
        """Terminate every live child, then kill the ones still alive after 5 s."""
        with self._lock:
            self.stop.set()
            processes = tuple(self._processes)
        for process in processes:
            if process.poll() is None:
                process.terminate()
        deadline = time.monotonic() + 5
        for process in processes:
            remaining = max(0, deadline - time.monotonic())
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def default_parallel() -> int:
    return max(1, min(4, (os.cpu_count() or 2) // 2))


def _study_directory(output_root: Path, definition: StudyDefinition) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    root = Path(output_root)
    for suffix in ("", *[f"_{index}" for index in range(2, 10000)]):
        directory = root / f"{definition.name}_{stamp}{suffix}"
        try:
            directory.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            continue
        return directory
    raise FileExistsError(f"could not allocate study directory for {definition.name!r}")


def run_study(definition: StudyDefinition, output_root: Path = REPOSITORY_ROOT / "runs",
              parallel: int = 1, on_update: Callable[[str, dict], None] | None = None) -> Path:
    """Prepare and run all valid cases in a study, returning its output directory."""
    if parallel < 1:
        raise ValueError("parallel must be at least 1")
    study_dir = _study_directory(Path(output_root), definition)
    cases = expand(definition)
    index = StudyIndex.create(study_dir, definition, cases)
    lock = threading.Lock()
    registry = _ProcessRegistry()

    def update(case_id: str, **fields):
        with lock:
            index.update(case_id, **fields)
        if on_update is not None:
            on_update(case_id, fields)

    prepared: list[tuple[PreparedCase, object]] = []
    for case in cases:
        try:
            item = prepare_case(study_dir, definition, case)
            error = validate_case(item)
        except Exception as exc:
            item, error = None, f"{type(exc).__name__}: {exc}"
        if error is not None:
            update(case.id, directory=f"cases/{case.id}", status="invalid", error=error)
        else:
            update(case.id, directory=f"cases/{case.id}", status="pending", error=None)
            prepared.append((item, case))

    def worker(item: PreparedCase, case):
        if registry.stop.is_set():
            update(case.id, status="cancelled", error="cancelled by interrupt")
            return {"status": "cancelled"}
        update(case.id, status="running")
        try:
            attempt = new_attempt(item)
            command = child_command(item, attempt)
            if registry.stop.is_set():
                update(case.id, status="cancelled", error="cancelled by interrupt")
                return {"status": "cancelled"}
            started = time.perf_counter()
            timed_out = False
            return_code = None
            with (attempt / "run.log").open("w") as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           cwd=REPOSITORY_ROOT, env=child_environment())
                registry.add(process)
                try:
                    return_code = process.wait(timeout=item.timeout_seconds)
                except subprocess.TimeoutExpired:
                    timed_out = True
                    process.kill()
                    return_code = process.wait()
                finally:
                    registry.discard(process)
            fields = finish_case(item, attempt, return_code=return_code,
                                 timed_out=timed_out, cancelled=registry.stop.is_set(),
                                 wall_seconds=time.perf_counter() - started, command=command)
        except Exception as error:
            fields = {"status": "failed", "error": f"{type(error).__name__}: {error}"}
        update(case.id, **fields)
        return fields

    pool = ThreadPoolExecutor(max_workers=parallel)
    futures = []
    for item, case in prepared:
        if registry.stop.is_set():
            break
        futures.append(pool.submit(worker, item, case))
    try:
        for future in as_completed(futures):
            future.result()
    except BaseException:
        registry.stop.set()
        registry.terminate_all()
        pool.shutdown(wait=True, cancel_futures=True)
        # The pool has stopped, so no worker writes any more. Cancel only the
        # cases that no worker finished; a case a worker cancelled stays as is.
        with lock:
            unfinished = [row["id"] for row in index.cases
                          if row.get("status") in {"pending", "running"}]
        for case_id in unfinished:
            update(case_id, status="cancelled", error="cancelled by interrupt")
        raise
    else:
        pool.shutdown(wait=True)
    return study_dir


def main(argv=None):
    """Run a study from a JSON definition and return its process exit code."""
    parser = argparse.ArgumentParser(description="Run a simulation GUI study")
    parser.add_argument("--definition", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=REPOSITORY_ROOT / "runs")
    parser.add_argument("--parallel", type=int, default=default_parallel())
    args = parser.parse_args(argv)
    if args.parallel < 1:
        parser.error("--parallel must be at least 1")
    definition = StudyDefinition.from_json(json.loads(args.definition.read_text()))
    directory = run_study(definition, args.output_root, args.parallel)
    payload = json.loads((directory / "study.json").read_text())
    valid = [case for case in payload["cases"] if case.get("status") != "invalid"]
    return 0 if valid and all(case.get("status") == "completed" for case in valid) else 1


if __name__ == "__main__":
    raise SystemExit(main())
