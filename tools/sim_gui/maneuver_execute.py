"""Native sim-study-v1 adapter for simulation-only maneuver cases.

The runner retains its immutable artifact directory below a queue attempt.  This
module makes the queue attempt the normal sim-suite result boundary and writes
its canonical RunRecord there after artifact verification.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import sys
import time
from typing import Any

from tools.sim_gui.study import StudyIndex, _atomic


@dataclass(frozen=True, slots=True)
class ManeuverStudyDefinition:
    """Serializable collection of immutable simulation-only maneuver inputs."""
    name: str
    source: str
    definitions: tuple[dict[str, Any], ...]

    def to_json(self) -> dict[str, Any]:
        return {"schema": "maneuver-study-v1", "name": self.name,
                "source": self.source, "definitions": list(self.definitions)}

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "ManeuverStudyDefinition":
        if data.get("schema") != "maneuver-study-v1":
            raise ValueError("unsupported maneuver study schema")
        return cls(str(data["name"]), str(data["source"]),
                   tuple(dict(row) for row in data["definitions"]))


@dataclass(frozen=True, slots=True)
class ManeuverCaseSpec:
    """One immutable maneuver case with the same index identity fields as a lap case."""
    id: str
    label: str
    definition: dict[str, Any]


@dataclass(frozen=True, slots=True)
class PreparedManeuverCase:
    """Queue-compatible materialized maneuver case."""
    directory: Path
    definition_path: Path
    timeout_seconds: float
    duration: float


def expand_maneuvers(definition: ManeuverStudyDefinition) -> tuple[ManeuverCaseSpec, ...]:
    """Create one standard case per validated serialized maneuver definition."""
    cases = []
    for index, item in enumerate(definition.definitions):
        if item.get("schema") != "maneuver-v1" or item.get("backend") != "simulation":
            raise ValueError("maneuver studies contain simulation maneuver-v1 definitions only")
        cases.append(ManeuverCaseSpec(f"case_{index:03d}", str(item.get("name") or f"maneuver {index + 1}"),
                                      json.loads(json.dumps(item))))
    if not cases:
        raise ValueError("maneuver study has no definitions")
    return tuple(cases)


def create_maneuver_study(directory: Path, definition: ManeuverStudyDefinition,
                          cases: tuple[ManeuverCaseSpec, ...]) -> StudyIndex:
    """Write native sim-study-v1 index and maneuver-study-v1 definition file."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    # Reuse StudyIndex's exact durable index construction, with a case object
    # whose serialized fields deliberately match the common queue identity.
    class _IndexDefinition:
        name = definition.name
        def to_json(self): return definition.to_json()
    @dataclass(frozen=True)
    class _IndexCase:
        id: str
        label: str
        model: str = "maneuver"
        track: str = ""
        choices: dict[str, str] = None
        sweep_points: dict[str, dict] = None
        def __post_init__(self):
            object.__setattr__(self, "choices", self.choices or {})
            object.__setattr__(self, "sweep_points", self.sweep_points or {})
    index_cases = tuple(_IndexCase(case.id, case.label) for case in cases)
    return StudyIndex.create(directory, _IndexDefinition(), index_cases)


def prepare_maneuver_case(study_dir: Path, definition: ManeuverStudyDefinition,
                          case: ManeuverCaseSpec) -> PreparedManeuverCase:
    """Materialize one native case directory and immutable maneuver input."""
    directory = Path(study_dir) / "cases" / case.id
    directory.mkdir(parents=True, exist_ok=True)
    definition_path = directory / "case.json"
    _atomic(definition_path, case.definition)
    duration = float((case.definition.get("termination") or {}).get(
        "duration_s", (case.definition.get("profile") or {}).get("duration_s", 0.0)))
    if duration <= 0:
        raise ValueError("maneuver case requires a positive duration")
    timeout = max(305.0, duration + 5.0)
    return PreparedManeuverCase(directory, definition_path, timeout, duration)


def new_maneuver_attempt(prepared: PreparedManeuverCase) -> Path:
    attempt = prepared.directory / f"attempt_{time.time_ns()}"
    attempt.mkdir(parents=True, exist_ok=False)
    return attempt


def maneuver_child_command(prepared: PreparedManeuverCase, attempt: Path) -> list[str]:
    """Run only the simulation-only maneuver runner below a queue attempt."""
    return [sys.executable, "-u", "-m", "tools.maneuver_gui.runner", "--definition",
            str(prepared.definition_path), "--output-root", str(attempt.parent),
            "--attempt-directory", str(attempt)]


def maneuver_progress(line: str, duration: float) -> float | None:
    try:
        event = json.loads(line)
        return max(0.0, min(1.0, float(event["fraction"])))
    except (ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None


def _native_result(attempt: Path) -> tuple[Path, dict]:
    """Read the runner's immutable artifact at the standard queue attempt root."""
    artifact = Path(attempt) / "result"
    if not artifact.is_dir():
        raise ValueError("missing completed maneuver result")
    summary = json.loads((Path(attempt) / "summary.json").read_text(encoding="utf-8"))
    return artifact, summary


def finish_maneuver_case(prepared: PreparedManeuverCase, attempt: Path, *, return_code: int | None,
                         timed_out: bool, cancelled: bool, wall_seconds: float,
                         command) -> dict[str, Any]:
    """Finish an ordinary queue case and write standard RunRecord if completed."""
    status = "cancelled" if cancelled else "timed_out" if timed_out else "failed"
    summary: dict[str, Any] | None = None
    record = None
    error = None
    if not cancelled and not timed_out and return_code == 0:
        try:
            artifact, summary = _native_result(attempt)
            if summary.get("execution_outcome") != "completed":
                raise ValueError("maneuver evidence is not completed")
            from tools.maneuver_gui.artifacts import load_artifact_evidence
            manifest, _arrays, _channels = load_artifact_evidence(artifact)
            if manifest.get("model_kind") != "14dof-simple":
                raise ValueError("standard maneuver RunRecord requires completed 14dof-simple evidence")
            from simplified_vehicle_models.results.maneuver_run_record import write_maneuver_run_record
            record = attempt / "run_record.npz"
            write_maneuver_run_record(record, manifest["definition"], artifact,
                                      model_kind="14dof-simple")
            status = "completed"
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            error = f"{type(exc).__name__}: {exc}"
    parent_status = {"completed": "Child_Completed", "cancelled": "Cancelled",
                     "timed_out": "Timed_Out"}.get(status, "Child_Rejected")
    parent = {"status": parent_status, "return_code": return_code, "total_wall_seconds": wall_seconds,
              "within_wall_limit": not timed_out, "timeout_seconds": prepared.timeout_seconds,
              "requested_wall_seconds": prepared.timeout_seconds, "elapsed_wall_seconds": wall_seconds,
              "model": "14dof-simple", "command": list(command), "attempt": str(attempt)}
    _atomic(attempt / "parent.json", parent)
    if summary is not None:
        _atomic(attempt / "summary.json", summary)
    study_root = prepared.directory.parent.parent
    return {"status": status, "attempt": attempt.relative_to(study_root).as_posix(),
            "run_record": record.relative_to(study_root).as_posix() if record else None,
            "summary": summary if error is None else {"error": error}}
