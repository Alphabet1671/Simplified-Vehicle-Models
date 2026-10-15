# Simplified Vehicle Models

This repository contains a NumPy point-mass vehicle model and spatial lap solver.
The model reads immutable vehicle parameters and prescribed track data from XML.

## Requirements

- Python 3.12 or newer
- NumPy 2.0 or newer

Install the package with `python -m pip install -e .`.

## Run the MIS example

```bash
python examples/run_mis_point_mass.py
```

The command prints JSON results for flying-lap and standing-start conditions.

## Run the tests

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

See [the MIS validation report](docs/validation/mis-point-mass.md) for assumptions and benchmark comparisons.

## Documentation

| Document | Contents |
|---|---|
| [MIS 2024 report standard](docs/standards/mis-2024-report-standard.md) | The required format for any MIS 2024 simulated-vs-measured report |
| [IEM24 Murphy stint report](docs/validation/iem24-murphy/report.md) | Reference implementation of that standard |
| [MIS point-mass validation](docs/validation/mis-point-mass.md) | Assumptions and benchmark comparisons |
| [Closed signed track validation](docs/validation/closed-track.md) | The signed MIS centreline rebuilt from the logged telemetry |
| [Package layout](docs/dev/package-layout.md) | The `src/simplified_vehicle_models/` tree |

`AGENTS.md` points coding agents at the report standard, and
`tests/test_mis_report_standard.py` checks every generated report against it.
