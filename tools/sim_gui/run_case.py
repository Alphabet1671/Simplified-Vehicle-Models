"""Compatibility entry point for retired point-mass GUI child dispatch."""
from __future__ import annotations


def main(argv=None):
    del argv
    raise SystemExit("point-mass adaptation unavailable until yaw GUI runner migration")


if __name__ == "__main__":
    raise SystemExit(main())
