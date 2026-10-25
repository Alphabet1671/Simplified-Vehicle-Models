# Task 10 report — gated robot contract and fake harness

## Scope

Added a vendor-neutral robot contract and deterministic **fake-only** backend:

- `RobotCapabilities`, `Telemetry`, and `RobotBackend` describe required commissioning, calibration, timing, bounds, stop, and telemetry interfaces.
- `RobotSession` implements explicit `DISCONNECTED → VALIDATED → ARMED → RUNNING` flow, absolute deadlines, no catch-up burst after lateness, single controlled-stop request on faults, explicit revalidation/rearm after faults, and an uncommissioned-device refusal.
- `FakeRobotBackend` is labeled `FAKE ROBOT BACKEND — NO PHYSICAL OUTPUT`; it supports deterministic stale telemetry, disconnect, deadline miss, interlock loss, stop requests, and an independent fake watchdog.

There is no CAN, analog, vendor API, generic actuator transport, or physical-backend GUI enablement. The fake watchdog test exercises fake protocol behavior only and is not evidence of actual safety equipment.

## Verification

- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_robot_session.py' -q`: **6 passed**.
- `QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_*.py' -q`: **97 passed**.
- `git diff --check`: passed.
