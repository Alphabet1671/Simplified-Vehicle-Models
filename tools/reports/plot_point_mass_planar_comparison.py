#!/usr/bin/env python3
"""Retired compatibility entrypoint for the historical point-mass/planar plot."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.reports.compare_planar_point_mass import main as point_mass_main


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        help="deprecated; the retired planar plot is no longer generated",
    )
    parser.parse_known_args()
    print(
        "Planar OCP plotting is retired and unavailable. "
        "Running the supported point-mass-only comparison instead."
    )
    point_mass_main()


if __name__ == "__main__":
    main()
