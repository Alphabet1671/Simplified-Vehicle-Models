import unittest
from simplified_vehicle_models.robot import FakeRobotBackend, RobotSession


class Clock:
    def __init__(self): self.value = 0
    def __call__(self): return self.value
    def advance(self, ns): self.value += ns


def definition():
    return {"backend":"physical", "calibration":{"id":"fake-calibration"},
            "command":{"swa_rad":.1, "accelerator":.2, "brake":0.}}


class RobotSessionTests(unittest.TestCase):
    def _running(self):
        clock = Clock(); fake = FakeRobotBackend(clock); session = RobotSession(fake, clock)
        session.validate(definition()); session.arm(); session.start()
        fake.set_telemetry(monotonic_ns=clock())
        return clock, fake, session

    def test_uncommissioned_refused_and_no_auto_arm(self):
        clock = Clock(); fake = FakeRobotBackend(clock, commissioned=False); session = RobotSession(fake, clock)
        with self.assertRaisesRegex(ValueError, "not commissioned"): session.validate(definition())
        self.assertEqual(session.state, "DISCONNECTED")

    def test_happy_tick_and_operator_stop_once(self):
        clock, fake, session = self._running()
        session.tick(); self.assertEqual(len(fake.commands), 1)
        session.stop("operator_stop"); session.stop("again")
        self.assertEqual(session.state, "STOPPED")
        self.assertEqual(fake.stop_requests, ["operator_stop"])

    def test_late_tick_fault_has_no_burst_or_repeat_stop(self):
        clock, fake, session = self._running()
        clock.advance(fake.capabilities().deadline_ns + 1)
        session.tick(); session.tick()
        self.assertEqual(session.state, "FAULT")
        self.assertEqual(session.reason, "deadline_missed")
        self.assertEqual(fake.stop_requests, ["deadline_missed"])
        self.assertEqual(fake.commands, [])

    def test_stale_disconnect_and_interlock_faults(self):
        for name, prepare, reason in (
            ("stale", lambda c,f: f.set_telemetry(monotonic_ns=-(f.capabilities().freshness_ns+1)), "telemetry_stale"),
            ("disconnect", lambda c,f: setattr(f, "disconnect_next", True), "disconnect"),
            ("interlock", lambda c,f: f.set_telemetry(interlock_ok=False, monotonic_ns=c()), "interlock_lost"),
        ):
            with self.subTest(name=name):
                clock, fake, session = self._running(); prepare(clock, fake); session.tick()
                self.assertEqual(session.state, "FAULT")
                self.assertEqual(session.reason, reason)
                self.assertEqual(len(fake.stop_requests), 1)

    def test_capability_gap_rejected_and_fault_requires_revalidate_arm(self):
        clock = Clock(); fake = FakeRobotBackend(clock)
        fake._capabilities = fake._capabilities.__class__(
            fake._capabilities.backend_id, fake._capabilities.calibration_id,
            fake._capabilities.command_modes, ("speed_mps",), fake._capabilities.max_swa_rad,
            fake._capabilities.max_swa_rate_rad_s, fake._capabilities.max_accelerator,
            fake._capabilities.max_brake, fake._capabilities.tick_ns,
            fake._capabilities.deadline_ns, fake._capabilities.freshness_ns,
            fake._capabilities.stop_modes, True)
        with self.assertRaisesRegex(ValueError, "telemetry"):
            RobotSession(fake, clock).validate(definition())
        clock, fake, session = self._running(); fake.disconnect_next = True; session.tick()
        self.assertEqual(session.state, "FAULT")
        session.validate(definition())
        self.assertEqual(session.state, "VALIDATED")
        self.assertTrue(session._stopped is False)
        fake.disconnect_next = False
        session.arm(); session.start()
        fake.set_telemetry(monotonic_ns=clock()); session.tick()
        self.assertEqual(len(fake.commands), 1)

    def test_command_deadline_and_fake_watchdog(self):
        clock, fake, session = self._running(); fake.deadline_miss = True; session.tick()
        self.assertEqual(session.reason, "command_deadline")
        watchdog = FakeRobotBackend(clock, watchdog_ns=10); watchdog.connect(); clock.advance(11); watchdog.watchdog_check()
        self.assertEqual(watchdog.stop_requests, ["fake_watchdog_expired"])


if __name__ == "__main__": unittest.main()
