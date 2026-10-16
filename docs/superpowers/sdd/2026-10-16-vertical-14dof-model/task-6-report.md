# Task 6 report — vertical static equilibrium subset

## Status

Implemented only the `VerticalState` value type and `solve_static_equilibrium` solver in the scoped Task 6 subset. No vertical dynamics, steady-corner solver, or tire-state-rate integration is included. Tire-state rates are deferred until the mechanics/evaluator API exists; the static tire-target initializer is independently available in `unitire_transient.steady_rolling_state`.

## Implementation

- Added frozen `VerticalState` with the documented 21 mechanical state fields, vector conversion, shape validation, and finite-value validation.
- Added a NumPy-only Newton solve over heave, roll, pitch, and four jounces.
- Equilibrium includes the vertical tire law, spring and ARB forces, unsprung gravity, sprung weight, speed-dependent aero downforce, and front/rear aero balance.
- Added static load oracle checks at zero speed and 12 m/s. The zero-jounce preload oracle applies at the zero-speed design point only; at 12 m/s the fixed-preload springs correctly compress under aero downforce, and the test checks total normal load and the aero-adjusted front axle share instead.

## Tests

- Focused: `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_vertical_equilibrium.py' -q` — 2 tests passed.
- Tire-state zero-rate assertion for the plant's initialized steady tire targets is deferred to plant initialization/Task 7; it requires a plant evaluator and is not stubbed in this static-equilibrium-only subset.

## Scope notes

The Task 6 brief includes subsequent dynamics and steady-corner work. Those are intentionally not represented as completed by this subset report. In particular, the brief's zero tire-state-rate assertion requires using the tire's steady target with an evaluator; that belongs with the dynamics task and is not claimed here.
