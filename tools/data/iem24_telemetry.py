"""Condition IEM24 Murphy navigation-unit telemetry into vehicle-body channels.

The exported MATLAB file stores each logged channel as a ``Time``/``Value``/
``Units`` structure on one shared 100 Hz time base.  The navigation unit is
mounted at a yaw skew relative to the car, so its ``X``/``Y`` channels mix the
longitudinal and lateral components.  This module estimates that skew from the
velocity channels and rotates the horizontal components into the vehicle body
frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.io import loadmat
from scipy.signal import butter, filtfilt

#: Name of the channel that carries the shared time base.
MAT_TIME_CHANNEL = "xtime"

#: Channels consumed from the log.  Everything else in the export is ignored.
UNIT_FRAME_CHANNELS: tuple[str, ...] = (
    "AccelX",
    "AccelY",
    "GyroZ",
    "VelX",
    "VelY",
    "Tq_RL_Req",
    "Tq_RR_Req",
    "VNorth",
    "VEast",
)

#: Scalar channels that pass through without a frame rotation.
SCALAR_CHANNELS: tuple[str, ...] = ("VPack",)

#: Every channel this validation reads from the export.
LOGGED_CHANNELS: tuple[str, ...] = UNIT_FRAME_CHANNELS + SCALAR_CHANNELS

#: Channels whose horizontal ``X``/``Y`` pair rotates into the body frame.
ROTATED_PAIRS: tuple[tuple[str, str], ...] = (
    ("VelX", "VelY"),
    ("AccelX", "AccelY"),
)


@dataclass(frozen=True, slots=True)
class NavChannels:
    """Unit-frame telemetry on the shared 100 Hz time base."""

    time: np.ndarray
    values: dict[str, np.ndarray]

    @property
    def sample_interval(self) -> float:
        """Return the median sample interval in seconds."""

        return float(np.median(np.diff(self.time)))

    def __getitem__(self, name: str) -> np.ndarray:
        return self.values[name]


@dataclass(frozen=True, slots=True)
class SkewEstimate:
    """Yaw skew of the navigation unit relative to the vehicle body."""

    angle_deg: float
    sample_count: int
    angle_interquartile_deg: float
    lateral_velocity_std_mps: float


@dataclass(frozen=True, slots=True)
class LapBoundary:
    """One navigation-unit position closure marking a lap boundary."""

    start_index: int
    end_index: int
    closure_radius_m: float
    lap_time_s: float
    distance_m: float
    start_speed_mps: float = 0.0

    @property
    def standing_start(self) -> bool:
        """Return whether the lap began from rest rather than at flying speed."""

        return self.start_speed_mps < 1.0


def load_nav_channels(
    mat_path: Path | str, channels: Sequence[str] = LOGGED_CHANNELS
) -> NavChannels:
    """Read the requested channels from the exported MATLAB telemetry file."""

    path = Path(mat_path)
    requested = [MAT_TIME_CHANNEL, *channels]
    payload = loadmat(path, variable_names=requested)
    missing = [name for name in requested if name not in payload]
    if missing:
        raise ValueError(f"{path}: missing channels {', '.join(missing)}")
    time = _mat_values(payload[MAT_TIME_CHANNEL])
    if time.size < 2:
        raise ValueError(f"{path}: the time channel is empty")
    values: dict[str, np.ndarray] = {}
    for name in channels:
        series = _mat_values(payload[name])
        if series.size != time.size:
            raise ValueError(
                f"{path}: channel {name} has {series.size} samples, "
                f"expected {time.size}"
            )
        values[name] = series
    return NavChannels(time=time, values=values)


def _mat_values(entry: np.ndarray) -> np.ndarray:
    """Unwrap one ``Time``/``Value``/``Units`` MATLAB structure."""

    if entry.dtype.names is None:
        raise ValueError("MAT entry is not a channel structure")
    return np.asarray(entry[0, 0]["Value"][0], dtype=np.float64)


def estimate_nav_skew_deg(
    channels: NavChannels,
    *,
    minimum_speed_mps: float = 8.0,
    maximum_abs_yaw_rate_rad_s: float = 0.05,
    window: tuple[float, float] | None = None,
) -> SkewEstimate:
    """Estimate the nav-unit yaw skew from straight, fast running.

    While the car travels almost straight, its velocity vector lies along the
    body longitudinal axis, so the direction of the measured velocity vector is
    the mounting skew.  The median over those samples is robust to the small
    sideslip that remains.
    """

    mask = np.ones(channels.time.size, dtype=bool)
    if window is not None:
        mask &= (channels.time >= window[0]) & (channels.time <= window[1])
    speed = np.hypot(channels["VelX"], channels["VelY"])
    mask &= speed > minimum_speed_mps
    mask &= np.abs(channels["GyroZ"]) < maximum_abs_yaw_rate_rad_s
    if np.count_nonzero(mask) < 200:
        raise ValueError("not enough straight, fast samples to estimate the skew")
    angles = np.degrees(np.arctan2(channels["VelY"][mask], channels["VelX"][mask]))
    angle_deg = float(np.median(angles))
    iqr = float(np.percentile(angles, 75) - np.percentile(angles, 25))
    cos_a, sin_a = np.cos(np.radians(angle_deg)), np.sin(np.radians(angle_deg))
    lateral = -channels["VelX"][mask] * sin_a + channels["VelY"][mask] * cos_a
    return SkewEstimate(
        angle_deg=angle_deg,
        sample_count=int(np.count_nonzero(mask)),
        angle_interquartile_deg=iqr,
        lateral_velocity_std_mps=float(np.std(lateral)),
    )


def rotate_to_body_frame(
    channels: NavChannels, angle_deg: float
) -> NavChannels:
    """Rotate the horizontal channel pairs into the vehicle body frame."""

    radians = np.radians(angle_deg)
    cos_a, sin_a = float(np.cos(radians)), float(np.sin(radians))
    values = dict(channels.values)
    for forward_name, lateral_name in ROTATED_PAIRS:
        forward = channels[forward_name]
        lateral = channels[lateral_name]
        values[forward_name] = forward * cos_a + lateral * sin_a
        values[lateral_name] = -forward * sin_a + lateral * cos_a
    return NavChannels(time=channels.time, values=values)


def zero_phase_lowpass(
    values: np.ndarray, sample_interval: float, cutoff_hz: float, order: int = 1
) -> np.ndarray:
    """Apply a zero-phase Butterworth low-pass filter.

    The default first order keeps step-shaped inputs, such as a brake-pressure
    or torque request, from overshooting their physical limits.
    """

    signal = np.asarray(values, dtype=np.float64)
    if signal.size < 3 * (order + 1):
        return signal.copy()
    nyquist = 0.5 / sample_interval
    if not 0.0 < cutoff_hz < nyquist:
        raise ValueError("cutoff must lie between zero and the Nyquist frequency")
    b, a = butter(order, cutoff_hz / nyquist, btype="low")
    return np.asarray(filtfilt(b, a, signal), dtype=np.float64)


def integrate_position(
    time: np.ndarray, north_velocity: np.ndarray, east_velocity: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate north/east velocity into a local plane position."""

    north = np.concatenate(([0.0], np.cumsum(0.5 * (north_velocity[:-1] + north_velocity[1:]) * np.diff(time))))
    east = np.concatenate(([0.0], np.cumsum(0.5 * (east_velocity[:-1] + east_velocity[1:]) * np.diff(time))))
    return north, east


def travel_distance(time: np.ndarray, speed: np.ndarray) -> np.ndarray:
    """Integrate speed into distance travelled since the first sample."""

    return np.concatenate(
        ([0.0], np.cumsum(0.5 * (speed[:-1] + speed[1:]) * np.diff(time)))
    )


def segment_laps(
    channels: NavChannels,
    *,
    window: tuple[float, float],
    minimum_lap_time_s: float = 45.0,
    maximum_lap_time_s: float = 140.0,
    maximum_closure_radius_m: float = 30.0,
) -> list[LapBoundary]:
    """Split a stint into laps using navigation-unit position closure.

    The circuit is closed, so the car returns to its previous lap-boundary
    position once per lap.  Searching for that closest return resegments the
    stint without relying on start/finish instrumentation.
    """

    time = channels.time
    inside = np.flatnonzero((time >= window[0]) & (time <= window[1]))
    if inside.size < 3:
        raise ValueError("the stint window contains too few samples")
    north, east = integrate_position(time, channels["VNorth"], channels["VEast"])
    speed = np.hypot(channels["VelX"], channels["VelY"])
    distance = travel_distance(time, speed)
    interval = channels.sample_interval
    minimum_gap = max(int(round(minimum_lap_time_s / interval)), 1)
    span = max(int(round(maximum_lap_time_s / interval)), minimum_gap + 1)

    boundaries = [int(inside[0])]
    radii = [0.0]
    while True:
        lower = boundaries[-1] + minimum_gap
        upper = min(lower + (span - minimum_gap), int(inside[-1]) + 1)
        if upper <= lower:
            break
        reference = boundaries[-1]
        radius = np.hypot(
            north[lower:upper] - north[reference], east[lower:upper] - east[reference]
        )
        offset = int(np.argmin(radius))
        if float(radius[offset]) > maximum_closure_radius_m:
            break
        boundaries.append(lower + offset)
        radii.append(float(radius[offset]))

    laps: list[LapBoundary] = []
    for start, end, closure in zip(boundaries[:-1], boundaries[1:], radii[1:]):
        laps.append(
            LapBoundary(
                start_index=start,
                end_index=end,
                closure_radius_m=closure,
                lap_time_s=float(time[end] - time[start]),
                distance_m=float(distance[end] - distance[start]),
                start_speed_mps=float(speed[start]),
            )
        )
    if not laps:
        raise ValueError("no complete lap was found in the stint window")
    return laps


def lap_distance_axis(
    time: np.ndarray, speed: np.ndarray, lap: LapBoundary
) -> np.ndarray:
    """Return distance along one lap, starting at zero on its boundary."""

    span = slice(lap.start_index, lap.end_index + 1)
    cumulative = travel_distance(time[span], speed[span])
    return cumulative - cumulative[0]
