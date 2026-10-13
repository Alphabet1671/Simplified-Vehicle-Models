"""Shared numerical helpers for the lookup tables.

The motor map and the cell curve both need interpolation.
The module also holds one smoothing helper.

Every interpolation function rejects extrapolation.
Every function keeps the shape of the query.

Units follow the caller. The module holds no physical constants.
"""

from __future__ import annotations

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


def validate_monotone(abscissa: FloatArray, name: str) -> None:
    """Check that the abscissa is strictly increasing.

    A duplicate or a decreasing pair raises ``ValueError``.
    A non-finite value raises ``ValueError``.
    The message names the argument and the index of the offending pair.
    """
    grid = np.asarray(abscissa, dtype=np.float64)
    if grid.ndim != 1:
        raise ValueError(f"The abscissa '{name}' must be one-dimensional.")
    if grid.size < 2:
        raise ValueError(f"The abscissa '{name}' must hold at least two values.")
    if not bool(np.all(np.isfinite(grid))):
        offenders = np.flatnonzero(~np.isfinite(grid))
        index = int(offenders[0])
        raise ValueError(
            f"The abscissa '{name}' must hold finite values. "
            f"Index {index} holds {float(grid[index])}."
        )

    step = np.diff(grid)
    offenders = np.flatnonzero(step <= 0.0)
    if offenders.size:
        index = int(offenders[0])
        raise ValueError(
            f"The abscissa '{name}' is not strictly increasing. "
            f"Index {index} holds {float(grid[index]):g} and index "
            f"{index + 1} holds {float(grid[index + 1]):g}."
        )


def _check_query_range(
    abscissa: FloatArray,
    query: FloatArray,
    name: str,
) -> None:
    """Raise ``ValueError`` when a query lies outside the abscissa range.

    A non-finite query raises ``ValueError``.
    The message names the query, the range, and the argument.
    """
    low = float(abscissa[0])
    high = float(abscissa[-1])
    values = np.asarray(query, dtype=np.float64)
    finite = np.isfinite(values)
    if not bool(np.all(finite)):
        flat = np.flatnonzero(~finite.reshape(-1))
        offender = float(values.reshape(-1)[flat[0]])
        raise ValueError(
            f"The query {offender} for argument '{name}' is not finite. "
            "The interpolation needs a finite query."
        )

    outside = (values < low) | (values > high)
    if not bool(np.any(outside)):
        return

    flat = np.flatnonzero(outside.reshape(-1))
    offender = float(values.reshape(-1)[flat[0]])
    raise ValueError(
        f"The query {offender:g} for argument '{name}' is outside the "
        f"abscissa range [{low:g}, {high:g}]. Extrapolation is not allowed."
    )


def interpolate_1d(
    abscissa: FloatArray,
    values: FloatArray,
    query: FloatArray,
) -> FloatArray:
    """Return the linear interpolation of one monotone table.

    A scalar query gives a scalar result.
    An array query gives a result with the same shape.
    A query outside the abscissa range raises ``ValueError``.
    """
    grid = np.asarray(abscissa, dtype=np.float64)
    table = np.asarray(values, dtype=np.float64)
    validate_monotone(grid, "abscissa")
    if table.shape != grid.shape:
        raise ValueError(
            "The values must have the same shape as the abscissa. "
            f"The abscissa holds {grid.size} values and the table holds "
            f"{table.size} values."
        )

    _check_query_range(grid, query, "query")
    return np.interp(np.asarray(query, dtype=np.float64), grid, table)


def interpolate_2d(
    abscissa: FloatArray,
    ordinate: FloatArray,
    values: FloatArray,
    query_abscissa: FloatArray,
    query_ordinate: FloatArray,
) -> FloatArray:
    """Return the bilinear interpolation of one monotone table.

    The values array has the shape ``(abscissa.size, ordinate.size)``.
    The two queries broadcast against each other.
    A query outside either range raises ``ValueError``.
    """
    grid_a = np.asarray(abscissa, dtype=np.float64)
    grid_o = np.asarray(ordinate, dtype=np.float64)
    table = np.asarray(values, dtype=np.float64)
    validate_monotone(grid_a, "abscissa")
    validate_monotone(grid_o, "ordinate")
    if table.shape != (grid_a.size, grid_o.size):
        raise ValueError(
            "The values must have the shape "
            f"({grid_a.size}, {grid_o.size}). The table has the shape "
            f"{table.shape}."
        )

    _check_query_range(grid_a, query_abscissa, "query_abscissa")
    _check_query_range(grid_o, query_ordinate, "query_ordinate")

    point_a, point_o = np.broadcast_arrays(
        np.asarray(query_abscissa, dtype=np.float64),
        np.asarray(query_ordinate, dtype=np.float64),
    )

    index_a = np.searchsorted(grid_a, point_a, side="right") - 1
    index_o = np.searchsorted(grid_o, point_o, side="right") - 1
    index_a = np.clip(index_a, 0, grid_a.size - 2)
    index_o = np.clip(index_o, 0, grid_o.size - 2)

    span_a = grid_a[index_a + 1] - grid_a[index_a]
    span_o = grid_o[index_o + 1] - grid_o[index_o]
    weight_a = (point_a - grid_a[index_a]) / span_a
    weight_o = (point_o - grid_o[index_o]) / span_o

    value_00 = table[index_a, index_o]
    value_01 = table[index_a, index_o + 1]
    value_10 = table[index_a + 1, index_o]
    value_11 = table[index_a + 1, index_o + 1]

    lower = value_00 * (1.0 - weight_o) + value_01 * weight_o
    upper = value_10 * (1.0 - weight_o) + value_11 * weight_o
    return lower * (1.0 - weight_a) + upper * weight_a


def smooth(values: FloatArray, window: int) -> FloatArray:
    """Return a centred moving average of one array.

    The output keeps the length of the input.
    The window shrinks at both edges, so the output stays defined.
    A window below one, above the array length, or even raises
    ``ValueError``.
    """
    table = np.asarray(values, dtype=np.float64)
    if table.ndim != 1:
        raise ValueError("The values must be one-dimensional.")

    size = int(window)
    if size != window:
        raise ValueError(f"The window must be a whole number. Got {window!r}.")
    if size < 1:
        raise ValueError(f"The window must be at least one. Got {size}.")
    if size > table.size:
        raise ValueError(
            f"The window must not exceed the array length. Got {size} for "
            f"an array of length {table.size}."
        )
    if size % 2 == 0:
        raise ValueError(f"The window must be odd. Got {size}.")

    if size == 1:
        return table.copy()

    kernel = np.ones(size, dtype=np.float64)
    total = np.convolve(table, kernel, mode="same")
    count = np.convolve(np.ones(table.size, dtype=np.float64), kernel, mode="same")
    return total / count
