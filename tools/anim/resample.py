"""Frame access for the animation exporter.

``FrameSource`` puts a run record on the video frame grid once, then answers
per-frame questions without touching the record again. It never invents data:
a channel the record does not hold is ``None``, an invalid sample is masked, and
``require`` raises ``MissingChannelError`` so the exporter fails loudly instead of
drawing a zero.

Sampling follows the run-record specification: states are cubic Hermite
interpolated, channels and controls are held from the enclosing accepted step.
"""
from __future__ import annotations

import numpy as np

from simplified_vehicle_models.results.run_record import (
    CHANNEL_CATALOGUE, COARSE_FRAME_RATE, RunRecord, resample_run_record, unpack_validity,
)

#: Wheel order of every record: front-left, front-right, rear-left, rear-right.
WHEELS = ("fl", "fr", "rl", "rr")
#: State-name patterns for the wheel angular speed (planar, then vertical plant).
WHEEL_SPEED_PATTERNS = ("omega_{}", "wheel_speed_{}")

#: scale key -> (how it is computed, channels it needs)
_SCALE_CHANNELS = {
    "speed": ("speed",),
    "acceleration": ("acceleration-longitudinal", "acceleration-lateral"),
    "steering": ("steering",),
    "tire_vertical_force": ("tire-vertical-force",),
    "tire_force": ("tire-longitudinal-force", "tire-lateral-force"),
    "tire_moment": ("tire-aligning-moment",),
    "yaw_acceleration": ("yaw-acceleration",),
    "brake_force": ("brake-force",),
}


class MissingChannelError(KeyError):
    """A channel the caller requires is not in the record."""

    def __str__(self) -> str:  # KeyError would repr() the message
        return str(self.args[0]) if self.args else ""


class FrameSource:
    """A run record on a uniform frame grid.

    ``channel_names`` limits which channels are reachable (``None`` means all the
    record holds); a requested channel the record lacks simply stays absent.
    ``allow_quasi_static`` lets the attitude come from the APPROXIMATE
    ``roll-quasi-static`` / ``pitch-quasi-static`` pair when the record has no
    dynamic ``roll`` / ``pitch``.
    """

    def __init__(self, record: RunRecord, frame_rate: float, channel_names=None, *,
                 t_start: float | None = None, t_end: float | None = None,
                 allow_quasi_static: bool = False):
        self.record = record
        self.frame_rate = float(frame_rate)
        self.allow_quasi_static = bool(allow_quasi_static)
        manifest = record.manifest
        if channel_names is not None:
            unknown = [name for name in channel_names if name not in CHANNEL_CATALOGUE]
            if unknown:
                raise ValueError(f"channels are not in the catalogue: {unknown}")
        available = [name for name in manifest.channel_names
                     if channel_names is None or name in channel_names]
        self.channel_names: tuple[str, ...] = tuple(available)
        self.model_name = str(manifest.model.get("model")
                              or manifest.model.get("plant_class") or "this model")
        self.frames = resample_run_record(record, self.frame_rate, t_start=t_start, t_end=t_end)
        self.times: np.ndarray = self.frames.time
        self.source_step: np.ndarray = self.frames.source_step
        self.state_names: tuple[str, ...] = manifest.state_names
        self.control_names: tuple[str, ...] = manifest.control_names
        self._columns = manifest.channel_columns
        self._valid = (unpack_validity(self.frames.channel_valid, len(manifest.channel_names))
                       if self.frames.channel_valid is not None else None)
        self._wheel_angle: np.ndarray | None = None
        self._scales = self._compute_scales()
        self.warnings: tuple[str, ...] = self._compute_warnings()

    # -- sizes and state -------------------------------------------------------------

    @property
    def count(self) -> int:
        return int(self.times.size)

    def state_at(self, index: int) -> np.ndarray:
        if self.frames.state is None:
            raise MissingChannelError("the record holds no state")
        return self.frames.state[index]

    def state_value(self, name: str, index: int) -> float | None:
        """One named state at a frame, or ``None`` when the model has no such state."""

        if self.frames.state is None or name not in self.state_names:
            return None
        return float(self.frames.state[index, self.state_names.index(name)])

    def control_at(self, name: str, index: int) -> float | None:
        if self.frames.control is None or name not in self.control_names:
            return None
        return float(self.frames.control[index, self.control_names.index(name)])

    # -- channels --------------------------------------------------------------------

    def has_channel(self, name: str) -> bool:
        return name in self.channel_names

    def require(self, name: str) -> str:
        """Return ``name`` or raise ``MissingChannelError``."""

        if name not in self.channel_names:
            raise MissingChannelError(
                f"channel {name!r} is not available in {self.model_name}")
        return name

    def channel_valid(self, name: str, index: int) -> bool:
        if name not in self.channel_names or self._valid is None:
            return False
        return bool(self._valid[index, self.record.manifest.channel_names.index(name)])

    def channel_at(self, name: str, index: int):
        """Channel value at a frame.

        ``None`` when the record does not hold the channel; a fully masked array
        when the sample is invalid (never a stand-in number); otherwise a float
        array of the channel width.
        """

        if name not in self.channel_names:
            return None
        width = CHANNEL_CATALOGUE[name].width
        if not self.channel_valid(name, index):
            return np.ma.masked_all(width)
        return np.array(self.frames.channel[index, self._columns[name]], dtype=float)

    def series(self, name: str) -> np.ndarray | None:
        """The whole channel on the frame grid: ``(frames, width)`` floats, ``NaN``
        wherever the sample is invalid, ``None`` when the channel is absent."""

        if name not in self.channel_names:
            return None
        values = np.array(self.frames.channel[:, self._columns[name]], dtype=float)
        valid = self._valid[:, self.record.manifest.channel_names.index(name)]
        values[~valid] = np.nan
        return values

    def attitude_series(self) -> np.ndarray | None:
        """``(frames, 2)`` of ``(roll, pitch)`` with ``NaN`` where invalid, or ``None``."""

        source = self.attitude_source
        if source == "none":
            return None
        names = ("roll", "pitch") if source == "dynamic" else (
            "roll-quasi-static", "pitch-quasi-static")
        return np.column_stack([self.series(name)[:, 0] for name in names])

    def scalar_at(self, name: str, index: int) -> float | None:
        """A width-1 channel as a float; ``None`` when absent or invalid."""

        value = self.channel_at(name, index)
        if value is None or np.ma.is_masked(value):
            return None
        return float(value[0])

    # -- attitude --------------------------------------------------------------------

    @property
    def attitude_source(self) -> str:
        """``"dynamic"``, ``"quasi-static"`` (opted in) or ``"none"``."""

        if self.has_channel("roll") and self.has_channel("pitch"):
            return "dynamic"
        if (self.allow_quasi_static and self.has_channel("roll-quasi-static")
                and self.has_channel("pitch-quasi-static")):
            return "quasi-static"
        return "none"

    def attitude_at(self, index: int) -> tuple[float, float] | None:
        """``(roll, pitch)`` in radians, or ``None`` when absent or invalid."""

        source = self.attitude_source
        if source == "none":
            return None
        names = ("roll", "pitch") if source == "dynamic" else (
            "roll-quasi-static", "pitch-quasi-static")
        roll, pitch = (self.scalar_at(name, index) for name in names)
        return None if roll is None or pitch is None else (roll, pitch)

    # -- wheel spin ------------------------------------------------------------------

    def wheel_spin_angles(self) -> np.ndarray | None:
        """Wheel spin angle about body ``+y`` per frame, ``(frames, 4)`` in radians.

        A wheel rolling forward turns about ``-y`` (top moves forward in an
        x-forward, z-down frame), so the angle is minus the integral of the wheel
        speed state, trapezoidal on the frame grid and zero at the first frame.
        ``None`` when the model has no wheel-speed states.
        """

        if self._wheel_angle is not None:
            return self._wheel_angle
        columns = None
        for pattern in WHEEL_SPEED_PATTERNS:
            names = [pattern.format(wheel) for wheel in WHEELS]
            if all(name in self.state_names for name in names):
                columns = [self.state_names.index(name) for name in names]
                break
        if columns is None or self.frames.state is None:
            return None
        speed = self.frames.state[:, columns]
        step = np.diff(self.times)[:, None]
        increments = 0.5 * (speed[1:] + speed[:-1]) * step
        angle = np.zeros_like(speed)
        angle[1:] = -np.cumsum(increments, axis=0)
        self._wheel_angle = angle
        return angle

    # -- fixed scales ----------------------------------------------------------------

    def _valid_values(self, name: str) -> np.ndarray | None:
        if name not in self.channel_names:
            return None
        values = self.record.channel_values(name)
        valid = self.record.channel_validity(name)
        return np.asarray(values, dtype=float)[valid]

    def _compute_scales(self) -> dict[str, float]:
        """Run-wide full-scale values, from every recorded step (not the frames)."""

        scales: dict[str, float] = {}
        for key, names in _SCALE_CHANNELS.items():
            arrays = [self._valid_values(name) for name in names]
            if any(array is None for array in arrays):
                continue
            if key in ("acceleration", "tire_force"):
                rows = min(array.shape[0] for array in arrays)
                if rows == 0:
                    continue
                # Rows where both components are valid are not guaranteed to align
                # after masking, so build the magnitude from the unmasked channels.
                a = np.asarray(self.record.channel_values(names[0]), dtype=float)
                b = np.asarray(self.record.channel_values(names[1]), dtype=float)
                ok = (self.record.channel_validity(names[0])
                      & self.record.channel_validity(names[1]))
                magnitude = np.hypot(a[ok], b[ok])
            else:
                magnitude = np.abs(arrays[0])
            if magnitude.size == 0:
                continue
            peak = float(np.max(magnitude))
            # An identically zero channel has no scale; 1.0 avoids dividing by zero
            # and keeps every arrow at zero length.
            scales[key] = peak if peak > 0.0 else 1.0
        return scales

    def scales(self) -> dict[str, float]:
        """Fixed full-scale values; identical on every call and for every frame."""

        return dict(self._scales)

    def arrow_length(self, key: str, value: float, maximum_length: float) -> float:
        """Arrow length for ``value``: ``maximum_length`` at the run full scale."""

        if key not in self._scales:
            raise MissingChannelError(f"no {key!r} scale: its channel is not in the record")
        return float(maximum_length) * float(value) / self._scales[key]

    # -- warnings --------------------------------------------------------------------

    def _compute_warnings(self) -> tuple[str, ...]:
        notes: list[str] = []
        manifest = self.record.manifest
        if manifest.coarse_for_120_fps and self.frame_rate >= COARSE_FRAME_RATE:
            maximum = manifest.integrator.get("maximum_step")
            detail = f" (maximum step {maximum} s)" if maximum is not None else ""
            notes.append(
                f"coarse record: steps are longer than 1/{COARSE_FRAME_RATE:g} s{detail}; "
                "held channels repeat across frames, so motion is not smooth at "
                f"{self.frame_rate:g} fps")
        dynamic = self.has_channel("roll") and self.has_channel("pitch")
        quasi = self.has_channel("roll-quasi-static") and self.has_channel("pitch-quasi-static")
        if not dynamic and quasi:
            if self.allow_quasi_static:
                notes.append(f"roll/pitch: not available in {self.model_name}; "
                             "showing roll-quasi-static and pitch-quasi-static "
                             "(APPROXIMATE: acceleration times a gradient)")
            else:
                notes.append(f"roll/pitch: not available in {self.model_name}; "
                             "roll-quasi-static and pitch-quasi-static exist but are "
                             "approximate and hidden; pass --quasi-static-attitude to show them")
        return tuple(notes)
