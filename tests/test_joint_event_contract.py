import math
import unittest
import json
import simplified_vehicle_models.solvers.joint_event_contract as contract
from simplified_vehicle_models.solvers.joint_event_contract import validate_restricted_joint_event

analytic_constant_acceleration_rest_to_distance = contract.analytic_constant_acceleration_rest_to_distance


class JointEventContractTests(unittest.TestCase):
    def test_analytic_rest_to_distance_reference(self):
        time_s, terminal_speed_mps = analytic_constant_acceleration_rest_to_distance(
            8.0, 2.0
        )
        self.assertEqual(time_s, 2.0 * math.sqrt(2.0))
        self.assertEqual(terminal_speed_mps, math.sqrt(32.0))

    def test_valid_straight_two_phase_api(self):
        result = validate_restricted_joint_event(
            launch={"start_time_s": 0.0, "end_time_s": 1.0,
                    "start_ell": 0.0, "end_ell": 10.0,
                    "samples": [
                        {"ell": 0.0, "x": 0.0, "y": 0.0, "heading": 0.0, "v": 0.0, "a": 2.0},
                        {"ell": 10.0, "x": 10.0, "y": 0.0, "heading": 0.0, "v": 4.0, "a": 2.0}]},
            spatial={"start_time_s": 1.0, "end_time_s": 3.0,
                     "start_ell": 10.0, "end_ell": 30.0,
                     "samples": [
                         {"ell": 10.0, "x": 10.0, "y": 0.0, "heading": 0.0, "v": 4.0, "a": 3.0},
                         {"ell": 30.0, "x": 30.0, "y": 0.0, "heading": 0.0, "v": 8.0, "a": 3.0}]},
            course_start=0.0, transition=10.0, course_finish=30.0,
        )
        self.assertEqual(result.status.value, "VALID")

    def test_valid_join_and_total_accounting(self):
        result = validate_restricted_joint_event(**base())
        self.assertEqual(result.status.value, "VALID")
        self.assertFalse(result.accepted)
        self.assertFalse(result.physical_feasibility_certified)
        self.assertFalse(result.event_qualified)
        self.assertEqual(result.total_time_s, 3.0)
        self.assertEqual(result.acceleration_join, "acceleration_jump_permitted")
        json.dumps(result.to_dict(), allow_nan=False)

    def test_geometry_join_components_are_exact(self):
        for field in ("ell", "x", "y", "heading", "v"):
            args = base()
            args["spatial"]["samples"][0][field] += 0.1
            self.assertEqual(run(**args).status.value, "INVALID", field)

    def test_nonzero_unwrapped_headings_are_retained(self):
        args = base()
        args["launch"]["samples"][-1].update(x=10.0, y=2.0, heading=0.2)
        args["spatial"]["samples"][0].update(x=10.0, y=2.0, heading=0.2)
        self.assertEqual(run(**args).status.value, "VALID")

    def test_phase_measure_partition_failures(self):
        for shift in (-1.0, 1.0):
            args = base()
            args["spatial"]["start_ell"] += shift
            self.assertEqual(run(**args).status.value, "INVALID")
        args = base()
        args["launch"]["end_ell"] = 30.0
        self.assertEqual(run(**args).status.value, "INVALID")

    def test_standing_and_positive_spatial_speed(self):
        self.assertEqual(validate().status.value, "VALID")
        args = base()
        args["spatial"]["samples"][1]["v"] = 0.0
        self.assertEqual(run(**args).status.value, "INVALID")

    def test_terminal_modes(self):
        self.assertEqual(run(**base(), terminal_mode="fixed", terminal_speed_mps=4.0).status.value, "VALID")
        self.assertEqual(run(**base(), terminal_mode="fixed", terminal_speed_mps=5.0).status.value, "INVALID")
        self.assertEqual(run(**base(), terminal_mode="free").status.value, "UNSUPPORTED")

    def test_timed_interval_endpoint_pairs(self):
        for pair in (("course_start", "transition"), ("transition", "course_finish"), ("course_start", "course_finish")):
            self.assertEqual(run(**base(), timed_interval={"start": pair[0], "finish": pair[1], "duration_s": {("course_start", "transition"): 1.0, ("transition", "course_finish"): 2.0, ("course_start", "course_finish"): 3.0}[pair]}).status.value, "VALID")
        for pair in (("transition", "course_start"), ("interior", "course_finish")):
            self.assertEqual(run(**base(), timed_interval={"start": pair[0], "finish": pair[1], "duration_s": 1.0}).status.value, "UNSUPPORTED")

    def test_rolling_and_periodic_are_unsupported(self):
        self.assertEqual(run(**base(), launch_mode="rolling").status.value, "UNSUPPORTED")
        self.assertEqual(run(**base(), periodic=True).status.value, "UNSUPPORTED")

    def test_malformed_data_is_invalid_and_serializable(self):
        args = base()
        args["launch"]["samples"][0]["v"] = True
        result = run(**args)
        self.assertEqual(result.status.value, "INVALID")
        json.dumps(result.to_dict(), allow_nan=False)


def base():
    return dict(
        launch={"start_time_s": 0.0, "end_time_s": 1.0, "start_ell": 0.0, "end_ell": 10.0,
                "samples": [dict(ell=0.0, x=0.0, y=0.0, heading=0.0, v=0.0, a=2.0),
                            dict(ell=10.0, x=10.0, y=0.0, heading=0.0, v=4.0, a=2.0)]},
        spatial={"start_time_s": 1.0, "end_time_s": 3.0, "start_ell": 10.0, "end_ell": 30.0,
                 "samples": [dict(ell=10.0, x=10.0, y=0.0, heading=0.0, v=4.0, a=3.0),
                             dict(ell=30.0, x=30.0, y=0.0, heading=0.0, v=8.0, a=3.0)]},
        course_start=0.0, transition=10.0, course_finish=30.0,
    )


def run(**kwargs):
    return contract.validate_restricted_joint_event(**kwargs)


def validate():
    return run(**base())


if __name__ == "__main__":
    unittest.main()
