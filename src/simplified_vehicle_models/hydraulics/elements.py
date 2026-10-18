"""Hydraulic elements: orifices and polytropic gas accumulators (NumPy only).

Units are SI: pressure Pa (gauge), volume m^3, flow m^3/s, area m^2.

Orifice. The pressure drop of a restriction with flow ``q`` is the mixed
laminar and turbulent law

    dp = r * q + c * q * |q|        (r > 0, c >= 0)

It is odd and strictly monotone, so it inverts in closed form,

    q = sign(dp) * 2 |dp| / (r + sqrt(r^2 + 4 c |dp|)),

with the slope ``dq/ddp = 1 / (r + 2 c |q|)``. The slope is finite at zero flow
(a pure turbulent law would have an infinite one), which keeps the circuit
Newton solve well posed. There is no table, so there is no domain to leave.

Accumulator. A gas volume ``g`` at absolute pressure ``P = P0 (V0 / g)^n``, where
``P0`` is the absolute precharge pressure at the precharge gas volume ``V0``.
The fluid volume in the accumulator is ``V0 - g``. The membrane rests on its stop
when ``g = V0`` (no fluid inside, pressure at the precharge), so ``g`` may not
exceed ``V0``. A minimum gas volume ``g_min`` stands for the compressed-gas stop.
A gas volume outside ``[g_min, V0]`` raises ``DomainError``: the model does not
extrapolate past its mechanical stops.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.suspension.errors import DomainError

#: Atmospheric pressure [Pa]. All circuit pressures are gauge; the gas law is absolute.
ATMOSPHERE = 101325.0


def _vector(name: str, value, size: int | None = None) -> np.ndarray:
    array = np.array(value, dtype=float).reshape(-1)
    if size is not None and array.shape != (size,):
        raise ValueError(f"{name} must hold {size} values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite")
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class OrificeSet:
    """A set of orifices, each ``dp = linear * q + quadratic * q * |q|``.

    ``maximum_pressure_drop`` is optional. It models a finite calibrated table:
    the algebraic law still returns its value, while ``flow_with_flag`` exposes
    the out-of-range flag for the plant/gates. The default is unbounded, which is
    the PROPOSED analytic valve used by the example circuit.
    """

    linear: np.ndarray
    quadratic: np.ndarray
    maximum_pressure_drop: np.ndarray | None = None

    def __post_init__(self) -> None:
        linear = _vector("linear", self.linear)
        quadratic = _vector("quadratic", self.quadratic, linear.size)
        if self.maximum_pressure_drop is None:
            maximum = np.full(linear.size, np.inf)
        else:
            maximum = _vector("maximum_pressure_drop", self.maximum_pressure_drop, linear.size)
        if np.any(linear <= 0.0):
            raise ValueError("orifice linear resistance must be positive")
        if np.any(quadratic < 0.0):
            raise ValueError("orifice quadratic coefficient must be nonnegative")
        if np.any(maximum <= 0.0):
            raise ValueError("orifice maximum pressure drop must be positive")
        object.__setattr__(self, "linear", linear)
        object.__setattr__(self, "quadratic", quadratic)
        object.__setattr__(self, "maximum_pressure_drop", maximum)

    @property
    def size(self) -> int:
        return int(self.linear.size)

    def pressure_drop(self, flow) -> np.ndarray:
        q = np.asarray(flow, dtype=float)
        return self.linear * q + self.quadratic * q * np.abs(q)

    def flow(self, drop) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(q, dq/d(drop))`` for a pressure drop across each orifice."""

        dp = np.asarray(drop, dtype=float)
        magnitude = np.abs(dp)
        q = np.sign(dp) * 2.0 * magnitude / (
            self.linear + np.sqrt(self.linear**2 + 4.0 * self.quadratic * magnitude))
        slope = 1.0 / (self.linear + 2.0 * self.quadratic * np.abs(q))
        return q, slope

    def flow_with_flag(self, drop) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return flow, slope and one out-of-range flag for each pressure drop."""

        dp = np.asarray(drop, dtype=float)
        q, slope = self.flow(dp)
        return q, slope, np.abs(dp) > self.maximum_pressure_drop


@dataclass(frozen=True)
class CylinderSet:
    """Wheel cylinders represented by signed wheel-referred piston areas [m²].

    Positive jounce-rate is compression. ``flow`` is positive when a wheel's
    cylinder delivers fluid into its node; a rod-side chamber therefore has a
    negative area entry.
    """

    areas: np.ndarray  # (N nodes, 4 wheels), m²

    def __post_init__(self) -> None:
        areas = np.asarray(self.areas, dtype=float)
        if areas.ndim != 2 or areas.shape[1] != 4 or not np.all(np.isfinite(areas)):
            raise ValueError("cylinder areas must be a finite (N, 4) matrix")
        areas.setflags(write=False)
        object.__setattr__(self, "areas", areas)

    @property
    def size(self) -> int:
        return int(self.areas.shape[0])

    def flow(self, jounce_rate) -> np.ndarray:
        """Return signed fluid delivery [m³/s] for four wheel jounce rates."""

        rate = np.asarray(jounce_rate, dtype=float)
        if rate.shape != (4,) or not np.all(np.isfinite(rate)):
            raise ValueError("jounce_rate must hold four finite values")
        return self.areas @ rate

    def force(self, pressure) -> np.ndarray:
        """Return signed wheel forces [N] for node pressures [Pa]."""

        pressure = np.asarray(pressure, dtype=float)
        if pressure.shape != (self.size,) or not np.all(np.isfinite(pressure)):
            raise ValueError(f"pressure must hold {self.size} finite values")
        return self.areas.T @ pressure


@dataclass(frozen=True)
class AccumulatorSet:
    """A set of polytropic gas accumulators."""

    precharge_pressure: np.ndarray  # gauge [Pa] at the precharge volume
    precharge_volume: np.ndarray  # gas volume at the precharge [m^3]
    exponent: np.ndarray  # polytropic exponent n
    minimum_volume: np.ndarray  # compressed-gas stop [m^3]

    def __post_init__(self) -> None:
        p0 = _vector("precharge_pressure", self.precharge_pressure)
        size = p0.size
        v0 = _vector("precharge_volume", self.precharge_volume, size)
        n = _vector("exponent", self.exponent, size)
        gmin = _vector("minimum_volume", self.minimum_volume, size)
        if np.any(p0 + ATMOSPHERE <= 0.0):
            raise ValueError("absolute precharge pressure must be positive")
        if np.any(v0 <= 0.0):
            raise ValueError("precharge volume must be positive")
        if np.any(n < 1.0) or np.any(n > 2.0):
            raise ValueError("polytropic exponent must lie in [1, 2]")
        if np.any(gmin <= 0.0) or np.any(gmin >= v0):
            raise ValueError("minimum gas volume must lie in (0, precharge volume)")
        for name, value in (("precharge_pressure", p0), ("precharge_volume", v0),
                            ("exponent", n), ("minimum_volume", gmin)):
            object.__setattr__(self, name, value)

    @property
    def size(self) -> int:
        return int(self.precharge_volume.size)

    def check(self, gas_volume) -> np.ndarray:
        """Return ``gas_volume`` after checking it against the mechanical stops."""

        g = np.asarray(gas_volume, dtype=float)
        if g.shape != (self.size,) or not np.all(np.isfinite(g)):
            raise ValueError("accumulator state must hold one finite gas volume per accumulator")
        tolerance = 1.0e-9 * self.precharge_volume
        if np.any(g < self.minimum_volume - tolerance) or np.any(g > self.precharge_volume + tolerance):
            raise DomainError(
                f"accumulator gas volume {g.tolist()} m^3 outside "
                f"[{self.minimum_volume.tolist()}, {self.precharge_volume.tolist()}]"
            )
        return g

    def pressure(self, gas_volume) -> np.ndarray:
        """Return the gauge pressure at ``gas_volume`` (checked against the stops)."""

        g = self.check(gas_volume)
        absolute = (self.precharge_pressure + ATMOSPHERE) * (self.precharge_volume / g) ** self.exponent
        return absolute - ATMOSPHERE

    def stiffness(self, gas_volume) -> np.ndarray:
        """Return ``-dP/dg`` [Pa/m^3] at ``gas_volume``."""

        g = self.check(gas_volume)
        absolute = (self.precharge_pressure + ATMOSPHERE) * (self.precharge_volume / g) ** self.exponent
        return self.exponent * absolute / g

    def volume_at(self, gauge_pressure) -> np.ndarray:
        """Return the gas volume at a gauge pressure, clipped at the membrane stop.

        Below the precharge pressure the accumulator holds no fluid and the gas
        stays at ``V0``. Above the compressed-gas stop it raises ``DomainError``.
        """

        absolute = np.asarray(gauge_pressure, dtype=float) + ATMOSPHERE
        if np.any(absolute <= 0.0):
            raise DomainError("accumulator pressure at or below vacuum")
        g = self.precharge_volume * ((self.precharge_pressure + ATMOSPHERE) / absolute) ** (
            1.0 / self.exponent)
        g = np.minimum(g, self.precharge_volume)
        if np.any(g < self.minimum_volume):
            raise DomainError("accumulator pressure beyond the compressed-gas stop")
        return g
