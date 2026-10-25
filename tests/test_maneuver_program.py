import math
import unittest
from simplified_vehicle_models.controls.maneuvers.types import ManeuverDefinition, Observation
from simplified_vehicle_models.controls.maneuvers.program import ManeuverProgram
from simplified_vehicle_models.controls.maneuvers.csv_source import CsvMapping, CsvSource
from simplified_vehicle_models.controls.maneuvers.types import RobotCommand


def payload(preset='straight_maintain', steer=None, longitudinal=None):
    return {'schema':'maneuver-v1','name':'test','preset':preset,'backend':'simulation','vehicle':{'model':'test'},
      'initial':{'speed_mps':10.,'x_m':0.,'y_m':0.,'heading_rad':0.},'clock':{'controller_tick_s':.1},
      'steering':steer or {'mode':'locked_swa','target_heading_rad':0.,'direction':1},
      'longitudinal':longitudinal or {'mode':'constant_accelerator','accelerator':.2},
      'profile':{'lead_in_s':0.,'rise_s':1.,'ramp_s':1.,'hold_s':1.,'release_s':1.,'recovery_s':0.,'duration_s':4.},
      'limits':{'max_swa_rad':.5,'max_swa_rate_rad_s':2.,'max_speed_mps':50.,'max_distance_m':1000.},
      'termination':{'duration_s':4.},'calibration':None,'csv':None}

class ManeuverProgramTests(unittest.TestCase):
    def obs(self, **changes):
        values=dict(time_s=0.,x_m=0.,y_m=0.,heading_rad=0.,speed_mps=10.,yaw_rate_rad_s=0.,swa_rad=0.,ay_mps2=0.)
        values.update(changes); return Observation(**values)

    def test_constant_accelerator_and_locked_swa_ignore_observation(self):
        p=ManeuverProgram(ManeuverDefinition.from_dict(payload()))
        a=p.tick(self.obs(),.1); b=p.tick(self.obs(heading_rad=2.,y_m=50.,speed_mps=1.),.1)
        self.assertEqual((a.swa_rad,a.accelerator),(b.swa_rad,b.accelerator))
        self.assertEqual(a.accelerator,.2)

    def test_cruise_negative_demand_brakes_without_overlap(self):
        data=payload('straight_maintain', {'mode':'locked_swa','target_heading_rad':0.}, {'mode':'cruise_pid','target_speed_mps':10.,'kp':1.,'ki':0.,'kd':0.})
        p=ManeuverProgram(ManeuverDefinition.from_dict(data)); c=p.tick(self.obs(speed_mps=12.),.1)
        self.assertGreater(c.brake,0.); self.assertEqual(c.accelerator,0.)

    def test_path_control_heading_initial_and_low_speed_freeze(self):
        data=payload('straight_maintain', {'mode':'path_pid','target_heading_rad':0.,'kp':1.,'ki':1.,'kd':0.,'minimum_speed_mps':1.}, {'mode':'constant_accelerator','accelerator':.1})
        p=ManeuverProgram(ManeuverDefinition.from_dict(data))
        self.assertEqual(p.tick(self.obs(),.1).swa_rad,0.)
        first=p.tick(self.obs(y_m=2.,speed_mps=0.),.1)
        p.reset(); second=p.tick(self.obs(y_m=2.,speed_mps=0.),.1)
        self.assertEqual(first,second)
        self.assertLess(first.swa_rad,0.)

    def test_circle_geometry_both_directions(self):
        for direction in (-1,1):
            data=payload('circular_skidpad', {'mode':'radius_pid','target_radius_m':10.,'direction':direction,'kp':1.,'ki':0.,'kd':0.}, {'mode':'constant_accelerator','accelerator':.1})
            p=ManeuverProgram(ManeuverDefinition.from_dict(data))
            c=p.tick(self.obs(x_m=11.,y_m=0.,speed_mps=10.),.1)
            self.assertEqual(math.copysign(1,c.swa_rad),direction)

    def test_path_pid_derivative_responds_to_changing_path_error(self):
        data=payload('straight_maintain', {'mode':'path_pid','target_heading_rad':0.,'kp':0.,'ki':0.,'kd':1.,'minimum_speed_mps':1.}, {'mode':'constant_accelerator','accelerator':.1})
        p=ManeuverProgram(ManeuverDefinition.from_dict(data))
        a=p.tick(self.obs(y_m=0.),.1)
        b=p.tick(self.obs(y_m=1.),.1)
        self.assertNotEqual(a.swa_rad,b.swa_rad)

    def test_profile_records_declared_phases_and_releases_to_zero(self):
        data=payload('straight_accel_brake', longitudinal={'mode':'profile','start_accelerator':0.,'end_accelerator':.8,'start_brake':0.,'end_brake':0.})
        p=ManeuverProgram(ManeuverDefinition.from_dict(data))
        expected=[(0.,'rise'),(1.,'ramp'),(2.,'hold'),(3.,'release'),(4.,'complete')]
        for time,phase in expected:
            command=p.tick(self.obs(time_s=time),.1)
            self.assertEqual(p._last_phase,phase)
            self.assertFalse(command.accelerator and command.brake)
        self.assertEqual(p.tick(self.obs(time_s=4.),.1).accelerator,0.)

    def test_csv_owns_all_channels_and_duration_termination(self):
        data=payload('csv_playback', {'mode':'csv'}, {'mode':'csv'})
        data['csv']={'mapping':{},'interpolation':'hold'}
        mapping=CsvMapping('t','s','a','b','s','rad','fraction','hold')
        source=CsvSource((0.,1.),(RobotCommand(.1,.2,0.),RobotCommand(.2,0.,.3)),'x',mapping,b'')
        p=ManeuverProgram(ManeuverDefinition.from_dict(data),source)
        self.assertEqual(p.tick(self.obs(time_s=.5),.1), source.sample(.5))
        self.assertEqual(p.termination_reason(self.obs(time_s=4.)), 'duration')
        self.assertEqual(p.breakpoints()[0],0.)

if __name__ == '__main__': unittest.main()
