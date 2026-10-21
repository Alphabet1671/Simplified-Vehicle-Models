"""Turn the accepted steps of a forward run into a run record.

``RunRecordSink`` is the ``step_sink`` of ``solve_forward``. For every accepted
step it reads the algebraic outputs the plant computed *in the same evaluation as
the state* (``plant.last_evaluation``, ``plant.last_powertrain``) and stores them
as catalogue channels. Nothing is re-evaluated, so a recorded tire force is the
force the integrator used, not a replay.

A channel the plant cannot supply is left out of the record; it is never
zero-filled. The attitude channels exist only for a plant with roll and pitch
states. The labelled quasi-static estimates are off unless requested.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from simplified_vehicle_models.formulations.planar_7dof import (
    rotate_tire_forces_to_body, wheel_positions, wheel_steering_angles,
)
from simplified_vehicle_models.results.run_record import (
    CHANNEL_CATALOGUE, RunRecorder, sha256_of,
)
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path

WHEELS = ("fl", "fr", "rl", "rr")
GRAVITY = 9.80665
#: Placeholder quasi-static gradients (deg per g), from the DSS anti values.
#: Used only when ``quasi_static_attitude`` is requested (record spec D3, D4).
QUASI_STATIC_ROLL_DEG_PER_G = 0.289
QUASI_STATIC_PITCH_DEG_PER_G = 0.20

CONTROL_NAMES = ("throttle", "brake-force", "steering-rate")
CONTROL_UNITS = ("1", "N", "rad/s")
#: Recorded only for a driver with pedal dynamics; absent otherwise, never NaN.
PEDAL_NAME, PEDAL_UNIT = "pedal", "1"

_STATE_UNITS = {"s": "m", "n": "m", "heading_error": "rad", "yaw_rate": "rad/s",
                "roll_rate": "rad/s", "pitch_rate": "rad/s", "soc": "1", "steering": "rad",
                "heave": "m", "roll": "rad", "pitch": "rad", "v_x": "m/s", "v_y": "m/s",
                "v_z": "m/s"}


def state_unit(name: str) -> str:
    """Unit of a plant state name; the empty string when it is not known."""

    if name in _STATE_UNITS:
        return _STATE_UNITS[name]
    prefix = name.rsplit("_", 1)[0]
    if prefix in ("omega", "wheel_speed"):
        return "rad/s"
    if prefix == "jounce":
        return "m"
    if prefix == "jounce_rate":
        return "m/s"
    return {"kappa_eff": "1", "lateral_deformation": "m", "trail_angle": "rad"}.get(prefix, "")


def _planar(vehicle):
    """The planar parameter layer of either vehicle type."""

    return getattr(vehicle, "planar", vehicle)


def _rotation(psi: float) -> np.ndarray:
    """Body-to-world rotation in (X north, Y east): body x -> (cos, sin)."""

    cosine, sine = np.cos(psi), np.sin(psi)
    return np.array([[cosine, -sine], [sine, cosine]])


class RunRecordSink:
    """Record each accepted step of ``solve_forward`` into a run record.

    ``path`` is the track's ``PathCoordinates`` (world geometry of the arc length
    coordinate). Call :meth:`close` after ``solve_forward`` returns.
    """

    def __init__(self, plant, path, output, *, precision: str = "float32",
                 quasi_static_attitude: bool = False):
        self.plant = plant
        self.path = path
        self.output = Path(output)
        self.precision = precision
        self.names = tuple(plant.state_names)
        self.index = {name: i for i, name in enumerate(self.names)}
        self.planar = _planar(plant.vehicle)
        self.has_attitude = all(name in self.index for name in ("roll", "pitch", "heave"))
        self.quasi_static = bool(quasi_static_attitude) and not self.has_attitude
        self.positions = np.asarray(wheel_positions(self.planar), dtype=float)
        self.channel_names: tuple[str, ...] | None = None
        self.recorder: RunRecorder | None = None
        self.has_pedal = False
        self.rows = 0
        self._step_sizes: list[float] = []
        self._last_time: float | None = None

    # -- channel extraction -------------------------------------------------

    def channels(self, sample) -> dict[str, np.ndarray]:
        """Return the catalogue channels of one accepted step."""

        plant, state, rates = self.plant, sample.state, sample.rates
        evaluation, point = plant.last_evaluation, plant.last_powertrain
        if evaluation is None or point is None:
            raise RuntimeError("the plant has no evaluation to record")
        i = self.index
        v_x, v_y = state[i["v_x"]], state[i["v_y"]]
        yaw_rate = state[i["yaw_rate"]]
        out: dict[str, np.ndarray] = {}

        # Pose: the path point at s, offset by n along the path's right normal.
        x_north, y_east, heading, _ = evaluate_path(self.path, state[i["s"]])
        normal = np.array([-np.sin(heading), np.cos(heading)])
        cg = np.array([float(x_north), float(y_east)]) + state[i["n"]] * normal
        psi = float(heading) + state[i["heading_error"]]
        wheels = cg + self.positions @ _rotation(psi).T
        out["speed"] = [np.hypot(v_x, v_y)]
        out["body-slip-angle"] = [np.arctan2(v_y, v_x)]
        out["heading-world"] = [psi]
        out["position-x"], out["position-y"] = [cg[0]], [cg[1]]
        out["wheel-position-x"], out["wheel-position-y"] = wheels[:, 0], wheels[:, 1]

        # Acceleration: state rate plus the rotating-frame term (see the plants).
        ax, ay = state_to_body_acceleration(plant, state, rates)
        out["acceleration-longitudinal"] = [ax]
        out["acceleration-lateral"] = [ay]
        out["yaw-acceleration"] = [rates[i["yaw_rate"]]]

        # Tires. A vertical evaluation packs (Fx, Fy, Mz) as columns; a planar
        # one carries separate arrays. The road-wheel angle includes jounce steer
        # for the vertical plant, which is what its tire-frame rotation used.
        angles = road_wheel_angles(plant, state)
        if hasattr(evaluation, "tire_forces"):
            fx, fy, mz = (np.asarray(evaluation.tire_forces)[:, k] for k in range(3))
            radius = np.asarray(plant_radii(plant, evaluation))
            slip = {}
        else:
            fx = np.asarray(evaluation.longitudinal_forces)
            fy = np.asarray(evaluation.lateral_forces)
            mz = np.asarray(evaluation.aligning_moments)
            radius = np.asarray(evaluation.effective_radii)
            slip = {"tire-slip-ratio": evaluation.slip_ratios,
                    "tire-slip-angle": evaluation.slip_angles}
        body_x, body_y = rotate_tire_forces_to_body(fx, fy, angles)
        out["wheel-steering-angle"] = angles
        out["wheel-loaded-radius"] = radius
        out["tire-longitudinal-force"], out["tire-lateral-force"] = fx, fy
        out["tire-aligning-moment"] = mz
        out["tire-vertical-force"] = np.asarray(evaluation.normal_loads)
        out["tire-force-in-body-x"], out["tire-force-in-body-y"] = body_x, body_y
        out.update(slip)
        out["wheel-drive-torque"] = np.asarray(point.wheel_torques)

        # Attitude: only a plant with attitude states supplies it.
        if self.has_attitude:
            out["roll"], out["pitch"] = [state[i["roll"]]], [state[i["pitch"]]]
            out["roll-rate"], out["pitch-rate"] = [state[i["roll_rate"]]], [state[i["pitch_rate"]]]
            out["heave"] = [state[i["heave"]]]
            out["acceleration-vertical"] = [rates[i["v_z"]]]
        elif self.quasi_static:
            # Right side down is positive roll, so a right turn (ay > 0) rolls
            # the body left: negative. Squat (ax > 0) is nose up: positive.
            out["roll-quasi-static"] = [-np.radians(QUASI_STATIC_ROLL_DEG_PER_G) * ay / GRAVITY]
            out["pitch-quasi-static"] = [np.radians(QUASI_STATIC_PITCH_DEG_PER_G) * ax / GRAVITY]

        # Driver and energy.
        command = sample.command
        out["throttle"], out["brake-force"] = [command.throttle], [command.brake_force]
        out["brake-fraction"] = [command.brake_force / self.planar.maximum_pedal_force]
        out["steering"] = [state[i["steering"]]]
        out["steering-rate"] = [command.steering_rate]
        out["bus-power"], out["bus-voltage"] = [point.bus_power], [point.voltage]
        out["bus-current"], out["soc"] = [point.current], [state[i["soc"]]]
        return out

    # -- sink protocol ------------------------------------------------------

    def __call__(self, sample) -> None:
        values = self.channels(sample)
        if self.recorder is None:
            self.channel_names = tuple(name for name in CHANNEL_CATALOGUE if name in values)
            self.has_pedal = sample.pedal is not None
            self.recorder = RunRecorder(
                self.output, state_names=self.names,
                state_units=[state_unit(name) for name in self.names],
                control_names=CONTROL_NAMES + ((PEDAL_NAME,) if self.has_pedal else ()),
                control_units=CONTROL_UNITS + ((PEDAL_UNIT,) if self.has_pedal else ()),
                channel_names=self.channel_names, precision=self.precision)
        control = [sample.command.throttle, sample.command.brake_force,
                   sample.command.steering_rate]
        if self.has_pedal:
            control.append(sample.pedal)
        self.recorder.append(sample.time, sample.index, sample.state, sample.rates, control,
                             {name: values[name] for name in self.channel_names})
        if self._last_time is not None:
            self._step_sizes.append(sample.time - self._last_time)
        self._last_time = sample.time
        self.rows += 1

    def close(self, metadata: dict | None = None, *, created_utc: str = "") -> Path:
        """Write the record. ``metadata`` is merged over the geometry defaults."""

        if self.recorder is None:
            raise RuntimeError("no accepted step was recorded")
        merged = {"vehicle_geometry": vehicle_geometry(self.planar, self.plant),
                  "track": {"lap_length_m": float(self.path.lap_length),
                            "path_coordinates": True, "heading_unwrapped": True}}
        merged.update(metadata or {})
        merged.setdefault("integrator", {})
        merged["integrator"].setdefault(
            "maximum_step", float(max(self._step_sizes)) if self._step_sizes else 0.0)
        if self.quasi_static:
            merged["notes"] = list(merged.get("notes", ())) + [
                "roll-quasi-static and pitch-quasi-static are APPROXIMATE estimates "
                f"({QUASI_STATIC_ROLL_DEG_PER_G} and {QUASI_STATIC_PITCH_DEG_PER_G} deg per g), "
                "not simulated states"]
        return self.recorder.close(merged, created_utc=created_utc)


def state_to_body_acceleration(plant, state, rates) -> tuple[float, float]:
    """Body-axis ``(ax, ay)`` of one step from the plant's own definition."""

    names = tuple(plant.state_names)
    v_x, v_y, r = (state[names.index(n)] for n in ("v_x", "v_y", "yaw_rate"))
    return (float(rates[names.index("v_x")] - r * v_y),
            float(rates[names.index("v_y")] + r * v_x))


def road_wheel_angles(plant, state) -> np.ndarray:
    """Road-wheel angle of each wheel, as the plant rotated its tire forces.

    Steering applies to the front axle; the vertical plant adds static toe and
    jounce steer, exactly as ``evaluate_vertical_dynamics`` does.
    """

    names = tuple(plant.state_names)
    angles = np.asarray(wheel_steering_angles(float(state[names.index("steering")]), 4),
                        dtype=float)
    suspension = getattr(plant.vehicle, "suspension", None)
    if suspension is not None and "jounce_fl" in names:
        jounce = np.array([state[names.index(f"jounce_{wheel}")] for wheel in WHEELS])
        angles = angles + suspension.kinematics.toe(jounce)
    return angles


def plant_radii(plant, evaluation) -> np.ndarray:
    """Loaded radius of each tire for an evaluation that does not carry it."""

    from simplified_vehicle_models.tires.unitire_simple import loaded_radius

    return np.asarray(loaded_radius(_planar(plant.vehicle).tire,
                                    np.asarray(evaluation.normal_loads)))


def vehicle_geometry(planar, plant=None) -> dict:
    """Body-axis geometry copied from the compiled vehicle (never re-derived)."""

    geometry = {
        "wheelbase_m": float(planar.wheelbase),
        "track_width_m": float(planar.track_width),
        "cg_height_m": float(planar.cg_height),
        "front_axle_to_cg_m": float(planar.front_axle_to_cg),
        "half_track_m": float(planar.half_track),
        "wheel_positions_body_m": np.asarray(wheel_positions(planar)).tolist(),
        "sprung_cg_height_m": None,
    }
    vehicle = getattr(plant, "vehicle", None)
    if hasattr(vehicle, "sprung_cg_height"):
        geometry["sprung_cg_height_m"] = float(vehicle.sprung_cg_height)
    return geometry


def source_hashes(paths) -> dict:
    """``sha256:`` digests keyed by repo-relative path, for the manifest."""

    return {str(path): sha256_of(Path(path)) for path in paths}
