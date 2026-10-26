"""Safety-gated robot session state machine for an externally supplied backend."""
from __future__ import annotations
import hashlib, json
from .contract import RobotBackend


class RobotSession:
    """Absolute-deadline session; it refuses any uncommissioned backend."""
    def __init__(self, backend: RobotBackend, clock_ns):
        self.backend, self.clock_ns = backend, clock_ns
        self.state = "DISCONNECTED"
        self.reason = None
        self._definition = None
        self._hash = None
        self._next_tick = None
        self._stopped = False

    def validate(self, definition):
        data = definition.to_dict() if hasattr(definition, "to_dict") else dict(definition)
        capabilities = self.backend.capabilities()
        if data.get("backend") != "physical": raise ValueError("robot session requires explicit physical backend definition")
        if not capabilities.commissioned: raise ValueError("physical backend is not commissioned")
        required = {"swa_rad", "accelerator", "brake", "speed_mps", "enabled", "interlock_ok"}
        if not required.issubset(capabilities.telemetry_fields): raise ValueError("required robot telemetry is unavailable")
        calibration = data.get("calibration")
        if not calibration or calibration.get("id") != capabilities.calibration_id: raise ValueError("calibration does not match robot capability")
        # A new validation invalidates every earlier arm/fault/stop snapshot.
        # The operator must explicitly arm the newly hashed definition again.
        self._definition = data
        self._hash = hashlib.sha256(json.dumps(data, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        self._stopped = False
        self.state, self.reason = "VALIDATED", None
        return self._hash

    def arm(self):
        if self.state != "VALIDATED": raise RuntimeError("validate before explicit user arm")
        try:
            self.backend.connect()
            telemetry = self.backend.telemetry()
        except ConnectionError:
            self._fault("arm_disconnect")
            return
        if not telemetry.enabled or not telemetry.interlock_ok:
            self._fault("arm_preflight_interlock")
            return
        self.backend.arm(self._hash)
        self.state = "ARMED"

    def start(self):
        if self.state != "ARMED": raise RuntimeError("explicit arm required before start")
        self._next_tick = self.clock_ns()
        self.state = "RUNNING"

    def tick(self):
        if self.state != "RUNNING": return
        now, capabilities = self.clock_ns(), self.backend.capabilities()
        if now > self._next_tick + capabilities.deadline_ns:
            self._fault("deadline_missed")
            return
        try:
            telemetry = self.backend.telemetry()
        except ConnectionError:
            self._fault("disconnect")
            return
        if now - telemetry.monotonic_ns > capabilities.freshness_ns:
            self._fault("telemetry_stale")
            return
        if not telemetry.enabled or not telemetry.interlock_ok:
            self._fault("interlock_lost")
            return
        command = self._definition.get("command")
        if command is None:
            self._fault("missing_command")
            return
        from simplified_vehicle_models.controls.maneuvers.types import RobotCommand
        try:
            self.backend.send(RobotCommand(**command), self._next_tick + capabilities.deadline_ns)
        except (TimeoutError, ConnectionError):
            self._fault("command_deadline")
            return
        # Skip missed periods rather than catch-up burst commands.
        self._next_tick = max(self._next_tick + capabilities.tick_ns, now + capabilities.tick_ns)

    def stop(self, reason="operator_stop"):
        if self.state in {"FAULT", "STOPPED", "DISCONNECTED"}: return
        self._request_stop_once(reason)
        self.state, self.reason = "STOPPED", reason

    def _fault(self, reason):
        if self.state == "FAULT": return
        self._request_stop_once(reason)
        self.state, self.reason = "FAULT", reason

    def _request_stop_once(self, reason):
        if not self._stopped:
            self.backend.request_stop(reason)
            self._stopped = True
