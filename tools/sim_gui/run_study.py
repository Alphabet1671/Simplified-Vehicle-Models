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

_ACTIVE_PROCESSES = set()
_PROCESS_LOCK = threading.Lock()
_STOP_REQUESTED = threading.Event()


def _register_process(process):
    with _PROCESS_LOCK:
        _ACTIVE_PROCESSES.add(process)


def _unregister_process(process):
    with _PROCESS_LOCK:
        _ACTIVE_PROCESSES.discard(process)


def _stop_processes():
    with _PROCESS_LOCK:
        processes = tuple(_ACTIVE_PROCESSES)
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
        if _STOP_REQUESTED.is_set():
            update(case.id, status="cancelled", error="cancelled by interrupt")
            return {"status": "cancelled"}
        update(case.id, status="running")
        try:
            attempt = new_attempt(item)
            command = child_command(item, attempt)
            started = time.perf_counter()
            timed_out = False
            return_code = None
            with (attempt / "run.log").open("w") as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           cwd=REPOSITORY_ROOT, env=child_environment())
                _register_process(process)
                try:
                    return_code = process.wait(timeout=item.timeout_seconds)
                except subprocess.TimeoutExpired:
                    timed_out = True
                    process.kill()
                    return_code = process.wait()
                finally:
                    _unregister_process(process)
            fields = finish_case(item, attempt, return_code=return_code,
                                 timed_out=timed_out, cancelled=False,
                                 wall_seconds=time.perf_counter() - started, command=command)
        except Exception as error:
            fields = {"status": "failed", "error": f"{type(error).__name__}: {error}"}
        update(case.id, **fields)
        return fields

    _STOP_REQUESTED.clear()
    pool = ThreadPoolExecutor(max_workers=parallel)
    futures = []
    for item, case in prepared:
        if _STOP_REQUESTED.is_set():
            break
        futures.append(pool.submit(worker, item, case))
    try:
        for future in as_completed(futures):
            future.result()
    except BaseException:
        _STOP_REQUESTED.set()
        _stop_processes()
        for item, case in prepared:
            payload = json.loads((study_dir / "study.json").read_text())
            current = next(row for row in payload["cases"] if row["id"] == case.id)
            if current.get("status") in {"pending", "running"}:
                update(case.id, status="cancelled", error="cancelled by interrupt")
        pool.shutdown(wait=False, cancel_futures=True)
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
