# Task 6 report: hydraulically interconnected suspension layout

Plan: [2026-10-17-suspension-layouts.md](../../plans/2026-10-17-suspension-layouts.md), Task 6.

## Design

The hydraulic network uses Q4 option (a): one integrated gas-fraction state per
accumulator node. Orifice lines are algebraic and carry no line-volume or flow
state. A hydraulic state is appended after the existing 38 vertical-forward
states, so all established independent-corner indices remain unchanged. A
vehicle-specific plant class publishes the extended `state_size`, `state_names`,
periodic indices and tolerances.

The circuit is stored in a pinned `hydraulic-circuit-v1` NPZ because XML carries
scalar values and external csv/npz data, not a graph. The XML keeps `line_scale`
and the SHA-256. The example is a PROPOSED per-axle roll-decoupled circuit with
four wheel-cylinder/accumulator nodes and FL-FR / RL-RR lines.

## Changes

* `src/simplified_vehicle_models/hydraulics/elements.py`: `CylinderSet`,
  monotone analytic `OrificeSet` with optional range flags, and bounded
  polytropic `AccumulatorSet`.
* `src/simplified_vehicle_models/hydraulics/circuit.py`: connected-node circuit,
  pressure equilibrium, volume conservation, algebraic line flow and static
  charge solving.
* `src/simplified_vehicle_models/hydraulics/table.py`: schema and SHA-256-pinned
  NPZ loader.
* `src/simplified_vehicle_models/suspension/layouts/hydraulic.py` and
  `suspension/hydraulic_components.py`: nested XML assembly, static-load check,
  travel/accumulator checks and stateful force network.
* `tools/model_data/generate_hydraulic_circuit.py`: deterministic NPZ/XML
  generator.
* Example files:
  `models/hydraulics/iem24_roll_decoupled_circuit.xml`,
  `models/data/iem24_roll_decoupled_circuit.npz`, and
  `models/suspensions/iem24_roll_decoupled_hydraulic.xml`.
* The evaluator accepts `network_state`; the runner obtains a matching plant
  class from the compiled network. The gate tool reports and validates hydraulic
  state bounds under `network_state_*` keys.

## Verification

* `tests/test_hydraulics.py`: **29 tests**, including cylinder sign/area, orifice
  inversion/monotonicity/range flag, accumulator pressure/stops, circuit charge
  conservation and equal-pressure equilibrium.
* `tests/test_suspension_hydraulic.py`: **6 tests**, including XML compilation,
  initial-state bounds, independent-gas and modal reduction oracles, and a 60 m
  BDF forward run.
* The 60 m run completed in approximately **4.6 s** on the development machine,
  versus **3.8 s** for the corresponding independent-corner smoke run; it had
  zero out-of-range evaluations, zero lifted-wheel evaluations and a minimum
  contact load of **786.4 N**.
* The complete suite was started as `/tmp/slp/suite_t6.log`; its final exit marker
  is recorded after the job completes. `git diff --check` and `compileall` pass
  for the current working tree.

## Caveats

This is an analytic proposed circuit, not a measured hydraulic fit. The example's
cylinder areas, precharge, gas volumes, polytropic exponent and orifice coefficients
are all marked **PROPOSED**. The mode-damper convention remains wheel-referred with
no motion ratio, as documented for Task 4. The existing acceptance evidence remains
tied to the default independent-corner 38-state vehicle and is not replaced by the
hydraulic smoke run.
