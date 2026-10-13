"""Declare the vehicle axis and sign conventions.

Every model in this project uses the SAE J670 vehicle frame.

Axis senses:

- ``+x`` points forward.
- ``+y`` points to the right.
- ``+z`` points down.

The frame is right handed because the cross product of ``+x`` and ``+y`` is
``+z``.

Sign conventions:

- A positive yaw rate turns the vehicle to the right.
- A right turn is therefore positive.
- Track curvature ``kappa`` is positive in a right turn and negative in a left
  turn.
- A positive lateral acceleration points to the right.
- A right turn moves vertical load to the left wheels.
- A positive vertical tire load is the compressive load magnitude.

The reference MATLAB tire functions use the same lateral sense and the same
vertical sense. The Python tire boundary therefore needs no lateral sign
conversion. Verify the longitudinal sense at the boundary.

Wheel order is always ``(FL, FR, RL, RR)`` for every four-wheel array.
"""

from __future__ import annotations


GRAVITY = 9.81

#: Stable identity of the vehicle axis convention.
VEHICLE_AXIS_CONVENTION = "sae-j670"

#: The forward, right, and down axis names of the vehicle frame.
AXIS_FORWARD = "x"
AXIS_RIGHT = "y"
AXIS_DOWN = "z"

#: Sign of the track curvature for a right turn.
RIGHT_TURN_CURVATURE_SIGN = 1.0

#: Sign of the track curvature for a left turn.
LEFT_TURN_CURVATURE_SIGN = -1.0

#: Wheel order of every four-wheel array.
WHEEL_ORDER = ("fl", "fr", "rl", "rr")
