"""Exceptions shared by the suspension subpackage."""
from __future__ import annotations


class DomainError(ValueError):
    """A query lies outside the domain on which an interpolant is defined.

    The interpolant never extrapolates silently; the caller decides how to
    extend (for example the damper table extends linearly and flags the point).
    """
