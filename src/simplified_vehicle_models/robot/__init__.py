"""Gated fake-only robot execution contract."""
from .contract import RobotBackend, RobotCapabilities, Telemetry
from .fake import FakeRobotBackend
from .session import RobotSession

__all__ = ["RobotBackend", "RobotCapabilities", "Telemetry", "FakeRobotBackend", "RobotSession"]
