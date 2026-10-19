"""Shared run result file: one self-describing capture of a forward simulation.

The capture is a single ZIP/NPZ container holding the state, the solver's own
state derivative, the realized control, and the algebraic outputs ("channels")
at every accepted integrator step, plus a ``manifest.json`` that names every
column and pins the sources. See
``docs/superpowers/specs/2026-10-16-run-result-file.md``.

This module is NumPy-only and never recomputes physics: a consumer resamples the
recorded values, it does not re-evaluate a plant.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline

SCHEMA = "run-record-v1"
MANIFEST_MEMBER = "manifest.json"
PRECISIONS = {"float32": np.float32, "float64": np.float64}
#: A zip entry date is fixed so two captures of one run are byte-identical.
_ZIP_EPOCH = (1980, 1, 1, 0, 0, 0)
#: Frame rate whose period a record's steps are compared against (check V11).
COARSE_FRAME_RATE = 120.0


class RunRecordError(ValueError):
    """A record violates one of the checks V1-V12 of the specification."""

    def __init__(self, check: str, message: str):
        super().__init__(f"{check}: {message}")
        self.check = check


@dataclass(frozen=True)
class ChannelSpec:
    """One named algebraic output: ``width`` is 1 or one value per wheel (4)."""

    name: str
    width: int
    unit: str
    frame: str
    description: str


def _catalogue(*rows: tuple) -> dict[str, ChannelSpec]:
    return {row[0]: ChannelSpec(*row) for row in rows}


#: Exactly the channels of specification section 6. Wheel order is FL, FR, RL, RR.
CHANNEL_CATALOGUE: dict[str, ChannelSpec] = _catalogue(
    ("speed", 1, "m/s", "world", "hypot(v-x, v-y)"),
    ("body-slip-angle", 1, "rad", "vehicle", "atan2(v-y, v-x)"),
    ("heading-world", 1, "rad", "world", "path heading at s plus heading error"),
    ("position-x", 1, "m", "world", "world X (north) of the CG"),
    ("position-y", 1, "m", "world", "world Y (east) of the CG"),
    ("wheel-position-x", 4, "m", "world", "world X of each wheel centre"),
    ("wheel-position-y", 4, "m", "world", "world Y of each wheel centre"),
    ("wheel-steering-angle", 4, "rad", "vehicle", "road-wheel angle incl. jounce steer"),
    ("wheel-loaded-radius", 4, "m", "-", "loaded tire radius"),
    ("acceleration-longitudinal", 1, "m/s^2", "vehicle", "state rate plus rotating-frame term"),
    ("acceleration-lateral", 1, "m/s^2", "vehicle", "state rate plus rotating-frame term"),
    ("yaw-acceleration", 1, "rad/s^2", "vehicle", "yaw-rate state rate"),
    ("acceleration-vertical", 1, "m/s^2", "vehicle", "14-DoF only"),
    ("roll", 1, "rad", "vehicle", "attitude state, positive right side down"),
    ("pitch", 1, "rad", "vehicle", "attitude state, positive nose up"),
    ("roll-rate", 1, "rad/s", "vehicle", "attitude rate state"),
    ("pitch-rate", 1, "rad/s", "vehicle", "attitude rate state"),
    ("heave", 1, "m", "vehicle", "heave state, positive down"),
    ("roll-quasi-static", 1, "rad", "vehicle", "APPROXIMATE: lateral acceleration times a gradient"),
    ("pitch-quasi-static", 1, "rad", "vehicle", "APPROXIMATE: longitudinal acceleration times a gradient"),
    ("tire-longitudinal-force", 4, "N", "tire", "Fx in the steered tire frame"),
    ("tire-lateral-force", 4, "N", "tire", "Fy in the steered tire frame"),
    ("tire-aligning-moment", 4, "N*m", "tire", "Mz in the steered tire frame"),
    ("tire-vertical-force", 4, "N", "tire", "normal load Fz"),
    ("tire-slip-ratio", 4, "1", "tire", "slip ratio"),
    ("tire-slip-angle", 4, "rad", "tire", "slip angle"),
    ("tire-force-in-body-x", 4, "N", "vehicle", "tire Fx rotated into body axes"),
    ("tire-force-in-body-y", 4, "N", "vehicle", "tire Fy rotated into body axes"),
    ("wheel-drive-torque", 4, "N*m", "vehicle", "powertrain wheel torque, signed"),
    ("throttle", 1, "1", "-", "realized driver throttle"),
    ("brake-force", 1, "N", "-", "realized driver brake pedal force"),
    ("brake-fraction", 1, "1", "-", "brake-force over the maximum pedal force"),
    ("steering", 1, "rad", "vehicle", "road-wheel steering angle"),
    ("steering-rate", 1, "rad/s", "vehicle", "realized steering rate"),
    ("bus-power", 1, "W", "-", "battery bus power"),
    ("bus-voltage", 1, "V", "-", "battery bus voltage"),
    ("bus-current", 1, "A", "-", "battery bus current"),
    ("soc", 1, "1", "-", "state of charge"),
)

_MANIFEST_DICT_FIELDS = ("producer", "model", "sources", "integrator", "segment", "track",
                         "vehicle_geometry")


@dataclass(frozen=True)
class RunRecordManifest:
    """The JSON document of the specification (section 4)."""

    state_names: tuple[str, ...]
    state_units: tuple[str, ...]
    control_names: tuple[str, ...]
    control_units: tuple[str, ...]
    channel_names: tuple[str, ...]
    channel_widths: tuple[int, ...]
    channel_units: tuple[str, ...]
    channel_frames: tuple[str, ...]
    rows: int
    precision: str = "float32"
    schema: str = SCHEMA
    created_utc: str = ""
    coarse_for_120_fps: bool = False
    producer: dict = field(default_factory=dict)
    model: dict = field(default_factory=dict)
    sources: dict = field(default_factory=dict)
    integrator: dict = field(default_factory=dict)
    segment: dict = field(default_factory=dict)
    track: dict = field(default_factory=dict)
    vehicle_geometry: dict = field(default_factory=dict)
    notes: tuple[str, ...] = ()

    def to_json(self) -> str:
        document = {
            "schema": self.schema, "created_utc": self.created_utc,
            "producer": self.producer, "model": self.model, "sources": self.sources,
            "integrator": self.integrator, "segment": self.segment, "track": self.track,
            "vehicle_geometry": self.vehicle_geometry,
            "state_names": list(self.state_names), "state_units": list(self.state_units),
            "control_names": list(self.control_names),
            "control_units": list(self.control_units),
            "channel_names": list(self.channel_names),
            "channel_widths": list(self.channel_widths),
            "channel_units": list(self.channel_units),
            "channel_frames": list(self.channel_frames),
            "rows": self.rows, "precision": self.precision,
            "coarse_for_120_fps": self.coarse_for_120_fps, "notes": list(self.notes),
        }
        return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"

    @classmethod
    def from_json(cls, text: str | bytes) -> "RunRecordManifest":
        try:
            document = json.loads(text)
        except (ValueError, UnicodeDecodeError) as error:
            raise RunRecordError("V1", f"manifest.json does not parse: {error}") from error
        if not isinstance(document, dict) or document.get("schema") != SCHEMA:
            raise RunRecordError("V1", f"schema must be {SCHEMA!r}")
        try:
            return cls(
                state_names=tuple(document["state_names"]),
                state_units=tuple(document["state_units"]),
                control_names=tuple(document["control_names"]),
                control_units=tuple(document["control_units"]),
                channel_names=tuple(document["channel_names"]),
                channel_widths=tuple(int(w) for w in document["channel_widths"]),
                channel_units=tuple(document["channel_units"]),
                channel_frames=tuple(document["channel_frames"]),
                rows=int(document["rows"]), precision=document["precision"],
                schema=document["schema"], created_utc=document.get("created_utc", ""),
                coarse_for_120_fps=bool(document.get("coarse_for_120_fps", False)),
                notes=tuple(document.get("notes", ())),
                **{key: dict(document.get(key, {})) for key in _MANIFEST_DICT_FIELDS},
            )
        except (KeyError, TypeError, ValueError) as error:
            raise RunRecordError("V1", f"manifest.json is incomplete: {error!r}") from error

    @property
    def channel_columns(self) -> dict[str, slice]:
        """Column slice of every channel inside the flat ``channel`` array."""

        columns, start = {}, 0
        for name, width in zip(self.channel_names, self.channel_widths):
            columns[name] = slice(start, start + width)
            start += width
        return columns

    @property
    def channel_column_count(self) -> int:
        return int(sum(self.channel_widths))


@dataclass(frozen=True)
class RunRecord:
    """A loaded record. Arrays are read-only; ``None`` means "not recorded"."""

    manifest: RunRecordManifest
    time: np.ndarray
    step_index: np.ndarray | None
    state: np.ndarray | None
    state_rate: np.ndarray | None
    control: np.ndarray | None
    channel: np.ndarray | None
    channel_valid: np.ndarray | None

    @property
    def rows(self) -> int:
        return int(self.time.shape[0])

    def has_channel(self, name: str) -> bool:
        return name in self.manifest.channel_names

    def channel_values(self, name: str) -> np.ndarray:
        """Recorded values of one channel: ``(rows,)`` or ``(rows, width)``."""

        if self.channel is None or name not in self.manifest.channel_names:
            raise KeyError(f"channel {name!r} is not in this record")
        block = self.channel[:, self.manifest.channel_columns[name]]
        return block[:, 0] if block.shape[1] == 1 else block

    def channel_validity(self, name: str) -> np.ndarray:
        """Boolean ``(rows,)``: whether the channel holds a value at each step."""

        if self.channel_valid is None or name not in self.manifest.channel_names:
            raise KeyError(f"channel {name!r} is not in this record")
        return unpack_validity(self.channel_valid, len(self.manifest.channel_names))[
            :, self.manifest.channel_names.index(name)]

    def state_column(self, name: str) -> np.ndarray:
        if self.state is None or name not in self.manifest.state_names:
            raise KeyError(f"state {name!r} is not in this record")
        return self.state[:, self.manifest.state_names.index(name)]

    def control_column(self, name: str) -> np.ndarray:
        if self.control is None or name not in self.manifest.control_names:
            raise KeyError(f"control {name!r} is not in this record")
        return self.control[:, self.manifest.control_names.index(name)]


def pack_validity(valid: np.ndarray) -> np.ndarray:
    """Pack a boolean ``(rows, n_channel)`` array into the on-disk bitmap."""

    return np.packbits(np.asarray(valid, dtype=bool), axis=1, bitorder="little")


def unpack_validity(bitmap: np.ndarray, channels: int) -> np.ndarray:
    return np.unpackbits(bitmap, axis=1, count=channels, bitorder="little").astype(bool)


def sha256_of(path: Path) -> str:
    return "sha256:" + hashlib.sha256(Path(path).read_bytes()).hexdigest()


class RunRecorder:
    """Collect accepted-step rows and write the container on :meth:`close`.

    Rows are compacted to the payload precision in blocks of ``chunk_rows``, so a
    long run holds only compact arrays, never lists of Python floats.
    """

    def __init__(self, path, *, state_names, state_units=None, control_names=(),
                 control_units=None, channel_names=(), precision: str = "float32",
                 chunk_rows: int = 4096):
        if precision not in PRECISIONS:
            raise ValueError(f"precision must be one of {sorted(PRECISIONS)}")
        self.path = Path(path)
        self.state_names = tuple(state_names)
        self.control_names = tuple(control_names)
        self.channel_names = tuple(channel_names)
        for label, names in (("state", self.state_names), ("control", self.control_names),
                             ("channel", self.channel_names)):
            if len(set(names)) != len(names):
                raise ValueError(f"{label} names must be unique")
        unknown = [name for name in self.channel_names if name not in CHANNEL_CATALOGUE]
        if unknown:
            raise ValueError(f"channels are not in the catalogue: {unknown}")
        self.state_units = tuple(state_units) if state_units is not None else (
            ("",) * len(self.state_names))
        self.control_units = tuple(control_units) if control_units is not None else (
            ("",) * len(self.control_names))
        if (len(self.state_units) != len(self.state_names)
                or len(self.control_units) != len(self.control_names)):
            raise ValueError("units must match the names")
        self.precision = precision
        self.chunk_rows = int(chunk_rows)
        self._dtype = PRECISIONS[precision]
        self._widths = tuple(CHANNEL_CATALOGUE[name].width for name in self.channel_names)
        self._columns = int(sum(self._widths))
        self._pending: list[tuple] = []
        self._blocks: list[tuple] = []
        self._closed = False

    def append(self, time, step_index, state, state_rate=None, control=None, channels=None):
        """Add one accepted step. A missing or ``None`` channel is stored invalid."""

        if self._closed:
            raise RuntimeError("the recorder is closed")
        state = np.asarray(state, dtype=float)
        if state.shape != (len(self.state_names),):
            raise ValueError("state must match the declared state names")
        rate = None if state_rate is None else np.asarray(state_rate, dtype=float)
        if rate is not None and rate.shape != state.shape:
            raise ValueError("state_rate must match the state")
        control = np.asarray(() if control is None else control, dtype=float)
        if control.shape != (len(self.control_names),):
            raise ValueError("control must match the declared control names")
        row = np.full(self._columns, np.nan)
        valid = np.zeros(len(self.channel_names), dtype=bool)
        supplied = channels or {}
        unexpected = set(supplied) - set(self.channel_names)
        if unexpected:
            raise ValueError(f"channels were not declared: {sorted(unexpected)}")
        start = 0
        for index, (name, width) in enumerate(zip(self.channel_names, self._widths)):
            value = supplied.get(name)
            if value is not None:
                value = np.asarray(value, dtype=float).reshape(-1)
                if value.shape != (width,):
                    raise ValueError(f"channel {name!r} needs {width} value(s)")
                if np.all(np.isfinite(value)):
                    row[start:start + width] = value
                    valid[index] = True
            start += width
        self._pending.append((float(time), int(step_index), state, rate, control, row, valid))
        if len(self._pending) >= self.chunk_rows:
            self._flush()

    def _flush(self):
        if not self._pending:
            return
        times, steps, states, rates, controls, rows, valids = zip(*self._pending)
        have_rates = all(rate is not None for rate in rates)
        self._blocks.append((
            np.array(times, dtype=float), np.array(steps, dtype=np.int32),
            np.array(states).astype(self._dtype),
            np.array(rates).astype(self._dtype) if have_rates else None,
            np.array(controls).reshape(len(times), -1).astype(self._dtype),
            np.array(rows).astype(self._dtype), np.array(valids)))
        self._pending = []

    def close(self, metadata: dict | None = None, *, created_utc: str = "") -> Path:
        """Write the container and return its path."""

        if self._closed:
            raise RuntimeError("the recorder is closed")
        self._flush()
        self._closed = True
        if not self._blocks:
            raise RunRecordError("V2", "a run record needs at least one accepted step")
        metadata = dict(metadata or {})
        time = np.concatenate([block[0] for block in self._blocks])
        step = np.concatenate([block[1] for block in self._blocks])
        state = np.concatenate([block[2] for block in self._blocks])
        rate_blocks = [block[3] for block in self._blocks]
        rate = None if any(block is None for block in rate_blocks) else np.concatenate(rate_blocks)
        control = np.concatenate([block[4] for block in self._blocks])
        channel = np.concatenate([block[5] for block in self._blocks])
        valid = np.concatenate([block[6] for block in self._blocks])
        integrator = dict(metadata.get("integrator", {}))
        maximum_step = integrator.get("maximum_step")
        if maximum_step is None:
            maximum_step = float(np.max(np.diff(time))) if time.size > 1 else 0.0
        manifest = RunRecordManifest(
            state_names=self.state_names, state_units=self.state_units,
            control_names=self.control_names, control_units=self.control_units,
            channel_names=self.channel_names, channel_widths=self._widths,
            channel_units=tuple(CHANNEL_CATALOGUE[n].unit for n in self.channel_names),
            channel_frames=tuple(CHANNEL_CATALOGUE[n].frame for n in self.channel_names),
            rows=int(time.size), precision=self.precision, created_utc=created_utc,
            coarse_for_120_fps=bool(float(maximum_step) > 1.0 / COARSE_FRAME_RATE + 1.e-12),
            producer=dict(metadata.get("producer", {})), model=dict(metadata.get("model", {})),
            sources=dict(metadata.get("sources", {})), integrator=integrator,
            segment=dict(metadata.get("segment", {})), track=dict(metadata.get("track", {})),
            vehicle_geometry=dict(metadata.get("vehicle_geometry", {})),
            notes=tuple(metadata.get("notes", ())))
        arrays = {"time": time, "step_index": step, "state": state, "control": control}
        if rate is not None:
            arrays["state_rate"] = rate
        if self.channel_names:
            arrays["channel"] = channel
            arrays["channel_valid"] = pack_validity(valid)
        write_container(self.path, manifest, arrays)
        return self.path


def write_container(path, manifest: RunRecordManifest, arrays: dict) -> None:
    """Write ``manifest.json`` first, then each array, with fixed zip metadata."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_STORED) as archive:
        info = zipfile.ZipInfo(MANIFEST_MEMBER, date_time=_ZIP_EPOCH)
        info.compress_type = zipfile.ZIP_STORED
        archive.writestr(info, manifest.to_json().encode("utf-8"))
        for name, array in arrays.items():
            info = zipfile.ZipInfo(f"{name}.npy", date_time=_ZIP_EPOCH)
            info.compress_type = zipfile.ZIP_STORED
            with archive.open(info, "w", force_zip64=True) as member:
                np.lib.format.write_array(member, np.ascontiguousarray(array),
                                          allow_pickle=False)


_OPTIONAL = ("step_index", "state", "state_rate", "control", "channel", "channel_valid")


def load_run_record(path, *, verify_sources: bool = True, root=None) -> RunRecord:
    """Read and validate a record (checks V1-V3, V6 and V10).

    ``root`` is the directory the manifest's relative ``sources`` paths resolve
    against; it defaults to the working directory.
    """

    try:
        archive = np.load(path, allow_pickle=False)
    except Exception as error:  # an unreadable container is a V1 failure
        raise RunRecordError("V1", f"cannot read the container: {error}") from error
    with archive:
        if MANIFEST_MEMBER not in archive.files:
            raise RunRecordError("V1", "manifest.json is missing")
        manifest = RunRecordManifest.from_json(bytes(archive[MANIFEST_MEMBER]))
        if "time" not in archive.files:
            raise RunRecordError("V2", "the required `time` array is missing")
        try:
            arrays = {name: np.array(archive[name]) for name in ("time", *_OPTIONAL)
                      if name in archive.files}
        except Exception as error:  # truncated member
            raise RunRecordError("V2", f"an array is unreadable or truncated: {error}") from error
    for array in arrays.values():
        array.setflags(write=False)
    record = RunRecord(manifest, arrays["time"], arrays.get("step_index"), arrays.get("state"),
                       arrays.get("state_rate"), arrays.get("control"), arrays.get("channel"),
                       arrays.get("channel_valid"))
    validate_record(record)
    if verify_sources:
        check_sources(manifest, Path(root) if root is not None else Path.cwd())
    return record


def validate_record(record: RunRecord) -> None:
    manifest, rows = record.manifest, record.rows
    if manifest.precision not in PRECISIONS:
        raise RunRecordError("V1", f"unknown precision {manifest.precision!r}")
    for label, names in (("state", manifest.state_names), ("control", manifest.control_names),
                         ("channel", manifest.channel_names)):
        if len(set(names)) != len(names):
            raise RunRecordError("V2", f"{label} names are not unique")
    if record.time.ndim != 1 or rows != manifest.rows:
        raise RunRecordError("V2", f"time has {record.time.shape} but the manifest says "
                                   f"{manifest.rows} rows")
    expected = {"step_index": (rows,),
                "state": (rows, len(manifest.state_names)),
                "state_rate": (rows, len(manifest.state_names)),
                "control": (rows, len(manifest.control_names)),
                "channel": (rows, manifest.channel_column_count),
                "channel_valid": (rows, -(-len(manifest.channel_names) // 8))}
    for name, shape in expected.items():
        array = getattr(record, name)
        if array is not None and array.shape != shape:
            raise RunRecordError("V2", f"{name} has shape {array.shape}, expected {shape}")
    if len(manifest.channel_names) and (record.channel is None or record.channel_valid is None):
        raise RunRecordError("V2", "channel names are declared but channel arrays are missing")
    if len(manifest.channel_widths) != len(manifest.channel_names):
        raise RunRecordError("V2", "channel_widths must match channel_names")
    if not np.all(np.isfinite(record.time)) or np.any(np.diff(record.time) <= 0.0):
        raise RunRecordError("V3", "time must be finite and strictly increasing")
    for name in ("state", "state_rate", "control"):
        array = getattr(record, name)
        if array is not None and not np.all(np.isfinite(array)):
            raise RunRecordError("V6", f"{name} contains a non-finite value")
    if record.channel is not None:
        valid = unpack_validity(record.channel_valid, len(manifest.channel_names))
        for index, name in enumerate(manifest.channel_names):
            block = record.channel[:, manifest.channel_columns[name]]
            if not np.all(np.isfinite(block[valid[:, index]])):
                raise RunRecordError("V6", f"channel {name!r} is non-finite where it is valid")


def check_sources(manifest: RunRecordManifest, root: Path) -> None:
    """V10: every hashed source must match the working tree."""

    for relative, digest in manifest.sources.items():
        file = root / relative
        if not file.is_file():
            raise RunRecordError("V10", f"source file is missing: {relative}")
        if sha256_of(file) != digest:
            raise RunRecordError("V10", f"source file changed since capture: {relative}")


# -- resampling ---------------------------------------------------------------


@dataclass(frozen=True)
class FrameSet:
    """The record on a uniform frame grid (specification section 7)."""

    time: np.ndarray
    state: np.ndarray | None
    state_rate: np.ndarray | None
    control: np.ndarray | None
    channel: np.ndarray | None
    channel_valid: np.ndarray | None
    #: Row of the enclosing accepted step (the last one at or before each frame).
    source_step: np.ndarray
    manifest: RunRecordManifest

    @property
    def frames(self) -> int:
        return int(self.time.shape[0])

    def channel_values(self, name: str) -> np.ndarray:
        block = self.channel[:, self.manifest.channel_columns[name]]
        return block[:, 0] if block.shape[1] == 1 else block

    def channel_validity(self, name: str) -> np.ndarray:
        return unpack_validity(self.channel_valid, len(self.manifest.channel_names))[
            :, self.manifest.channel_names.index(name)]


def frame_times(t_start: float, t_end: float, frame_rate: float) -> np.ndarray:
    """Return ``t_k = t_start + k / rate`` for ``k = 0 .. floor(span * rate)``."""

    if not (np.isfinite(frame_rate) and frame_rate > 0.0):
        raise ValueError("frame_rate must be positive and finite")
    count = int(np.floor((t_end - t_start) * frame_rate + 1.e-9)) + 1
    return np.minimum(t_start + np.arange(count) / frame_rate, t_end)


def _hermite(time, state, rate, query, index):
    h = time[index + 1] - time[index]
    t = ((query - time[index]) / h)[:, None]
    h = h[:, None]
    t2, t3 = t * t, t * t * t
    return ((2 * t3 - 3 * t2 + 1) * state[index] + (t3 - 2 * t2 + t) * h * rate[index]
            + (-2 * t3 + 3 * t2) * state[index + 1] + (t3 - t2) * h * rate[index + 1])


def resample_run_record(record: RunRecord, frame_rate: float, *, t_start: float | None = None,
                        t_end: float | None = None) -> FrameSet:
    """Resample a record to a uniform grid.

    * ``state``: cubic Hermite through the knots. When ``state_rate`` was
      recorded its values are the slopes, which is exact for a cubic and
      reproduces the solver's own derivative at every knot; otherwise a
      Fritsch-Carlson monotone spline is used.
    * ``state_rate``, ``control``, ``channel``: zero-order hold. A channel is an
      algebraic output of its step and does not exist between steps.
    """

    time = record.time
    first = float(time[0]) if t_start is None else float(t_start)
    last = float(time[-1]) if t_end is None else float(t_end)
    if first < time[0] or last > time[-1] or last < first:
        raise ValueError("the requested range must lie inside the record")
    grid = frame_times(first, last, frame_rate)
    held = np.clip(np.searchsorted(time, grid, side="right") - 1, 0, time.size - 1)
    state = None
    if record.state is not None:
        values = record.state.astype(float)
        if time.size == 1:
            state = np.repeat(values, grid.size, axis=0)
        elif record.state_rate is not None:
            interval = np.clip(held, 0, time.size - 2)
            state = _hermite(time, values, record.state_rate.astype(float), grid, interval)
        else:
            state = np.column_stack([MonotoneSpline(time, values[:, k])(grid)
                                     for k in range(values.shape[1])])
    def hold(array):
        return None if array is None else array[held]
    return FrameSet(grid, state, hold(record.state_rate), hold(record.control),
                    hold(record.channel), hold(record.channel_valid), held, record.manifest)
