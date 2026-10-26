"""Deterministic fake backend used only for contract and fault testing."""
from __future__ import annotations
from dataclasses import replace
from .contract import RobotCapabilities, Telemetry


class FakeRobotBackend:
    """Unmistakably fake backend; it has no actuator, bus, or vendor interface."""
    label = "FAKE ROBOT BACKEND — NO PHYSICAL OUTPUT"

    def __init__(self, clock_ns, *, commissioned=True, watchdog_ns=0):
        self.clock_ns = clock_ns
        self._capabilities = RobotCapabilities(
            "fake-robot", "fake-calibration", ("open_loop",),
            ("swa_rad", "accelerator", "brake", "speed_mps", "x_m", "y_m", "heading_rad", "ay_mps2", "enabled", "interlock_ok"),
            1.0, 2.0, 1.0, 1.0, 10_000_000, 20_000_000, 50_000_000,
            ("controlled_stop",), commissioned,
        )
        self.connected = False
        self.armed_hash = None
        self.commands = []
        self.stop_requests = []
        self.disconnect_next = False
        self.deadline_miss = False
        self.watchdog_ns = watchdog_ns
        self._last_activity_ns = 0
        self._telemetry = Telemetry(0, 0., 0., 0., 0., 0., 0., 0., 0., True, True, 0)

    def capabilities(self): return self._capabilities
    def connect(self): self.connected = True; self._last_activity_ns = self.clock_ns()
    def disconnect(self): self.connected = False
    def arm(self, definition_sha256):
        if not self.connected: raise ConnectionError("fake backend disconnected")
        self.armed_hash = definition_sha256; self._last_activity_ns = self.clock_ns()
    def telemetry(self):
        if self.disconnect_next: self.connected = False; raise ConnectionError("injected fake disconnect")
        if not self.connected: raise ConnectionError("fake backend disconnected")
        return self._telemetry
    def send(self, command, deadline_ns):
        now = self.clock_ns()
        if self.deadline_miss or now > deadline_ns: raise TimeoutError("injected fake deadline miss")
        self.commands.append((command, deadline_ns)); self._last_activity_ns = now
    def request_stop(self, reason): self.stop_requests.append(reason); self._last_activity_ns = self.clock_ns()
    def set_telemetry(self, **values): self._telemetry = replace(self._telemetry, **values)
    def watchdog_check(self):
        if self.watchdog_ns and self.connected and self.clock_ns() - self._last_activity_ns > self.watchdog_ns:
            self.request_stop("fake_watchdog_expired")
