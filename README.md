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
