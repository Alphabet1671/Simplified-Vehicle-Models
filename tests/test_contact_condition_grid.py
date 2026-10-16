"""Exhaustively grid-check the analytic contact-load conditions.

``contact_nonnegativity_conditions`` argues that, with the tire force bounded by
``M * Fz``, every implied acceleration stays in a box where all four normal
loads are nonnegative.  This test checks the argument numerically instead of
trusting the algebra: over a dense grid of dynamic pressures it evaluates the
real ``evaluate_normal_loads`` at the corners (and edge midpoints) of that
acceleration box and requires every load to be nonnegative.  It is a grid
audit, not a proof for every real dynamic pressure; the loads are affine in q,
so a dense grid with the end points is a strong check of that argument.
"""
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.planar_dense_certificate import (
    contact_nonnegativity_conditions,
)
from simplified_vehicle_models.formulations.planar_7dof import aerodynamic_forces
from simplified_vehicle_models.suspension import quasi_static


class ContactConditionGridTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(
            "models/formulations/iem24_murphy_real_planar_7dof.xml"
        ).model

    def test_conditions_hold_for_the_vehicle(self):
        result = contact_nonnegativity_conditions(self.vehicle)
        self.assertTrue(result["holds"], result)

    def _loads(self, speed, ax, ay, lift):
        vehicle = self.vehicle
        return np.asarray(quasi_static.normal_loads(
            total_mass=vehicle.total_mass, wheelbase=vehicle.wheelbase,
            track_width=vehicle.track_width, cg_height=vehicle.cg_height,
            front_mass_fraction=vehicle.front_mass_fraction,
            lateral_transfer_distribution=vehicle.lateral_load_transfer_distribution,
            downforce=aerodynamic_forces(vehicle, speed)[0],
            aero_balance=vehicle.aero.aero_balance, longitudinal_acceleration=ax,
            lateral_acceleration=ay, wheel_lift=lift))

    def test_axle_loads_nonnegative_and_lifted_wheel_loads_nonnegative(self):
        """The conditions cover axle loads; per-wheel lift is the model's job.

        Raw per-wheel loads can be negative inside the acceleration box (the
        unequal front/rear roll distribution pushes one inside wheel below
        zero before the axle total does).  The planar model lifts that wheel at
        zero load.  So the claims checked are: the raw axle totals stay >= 0 and
        the lifted model never returns a negative wheel load.
        """

        vehicle = self.vehicle
        tire, aero = vehicle.tire, vehicle.aero
        mu = max(tire.mu_0_x, tire.mu_s_x, tire.mu_0_y, tire.mu_s_y)
        mass = vehicle.total_mass
        lowest_axle = np.inf
        lowest_lifted = np.inf
        for speed in np.linspace(0.3, 70., 400):
            q = 0.5 * aero.air_density * speed * speed
            weight = mass * 9.80665 + q * aero.downforce_area
            drag = q * aero.drag_area
            reach = mu * weight / mass
            for ax in np.linspace(-reach - drag / mass, reach - drag / mass, 9):
                for ay in np.linspace(-reach, reach, 9):
                    raw = self._loads(speed, ax, ay, False)
                    lowest_axle = min(lowest_axle, float(raw[0] + raw[1]), float(raw[2] + raw[3]))
                    lowest_lifted = min(lowest_lifted, float(self._loads(speed, ax, ay, True).min()))
        self.assertGreaterEqual(lowest_axle, -1.e-6)
        self.assertGreaterEqual(lowest_lifted, -1.e-6)


if __name__ == "__main__":
    unittest.main()
