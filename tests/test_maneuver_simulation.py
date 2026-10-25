import unittest
import numpy as np

from simplified_vehicle_models.solvers.maneuver_simulation import SimulationSettings, simulate_maneuver


class Solved:
    def __init__(self, rates): self.rates = rates
    def solve(self): return self.rates


class Plant:
    state_size = 1
    def evaluate(self, t, y, c): return Solved(np.array([float(c)]))
    def observe(self, t, y, c): return float(y[0])


class Program:
    def __init__(self, duration=.1, source=None):
        self.data = {'termination': {'duration_s': duration}}
        self.csv_source = source
        self.tick_count = 0
    def tick(self, observation, dt):
        self.tick_count += 1
        return float(self.tick_count)
    def breakpoints(self): return self.csv_source.breakpoints() if self.csv_source else ()


class ManeuverSimulationTests(unittest.TestCase):
    def test_sampled_tick_count_and_rhs_independence(self):
        a, b = Program(), Program()
        ra = simulate_maneuver(Plant(), a, [0.], SimulationSettings())
        rb = simulate_maneuver(Plant(), b, [0.], SimulationSettings(rtol=1e-4, atol=1e-6))
        self.assertEqual(a.tick_count, 10)
        self.assertEqual(b.tick_count, 10)
        np.testing.assert_array_equal(ra.requested_commands, rb.requested_commands)
        self.assertEqual(ra.termination_reason, 'duration')
        self.assertEqual(len(ra.time_s), len(ra.state_rate))
        self.assertFalse(ra.state.flags.writeable)
        self.assertIsInstance(ra.requested_commands, tuple)
        self.assertIsInstance(ra.applied_commands, tuple)
        with self.assertRaises(AttributeError): ra.applied_commands.append(9)

    def test_source_knot_does_not_tick_pid(self):
        class Source:
            def breakpoints(self): return (.015,)
            def sample(self, t): return 2. if t >= .015 else 1.
        p = Program(source=Source())
        r = simulate_maneuver(Plant(), p, [0.], SimulationSettings())
        self.assertEqual(p.tick_count, 10)
        self.assertTrue(any(abs(t-.015)<1e-12 for t in r.time_s))

    def test_csv_ends_before_duration_without_extrapolation(self):
        class Source:
            time_s = (0., .04)
            def breakpoints(self): return self.time_s
            def sample(self, t):
                if t > .04: raise AssertionError('extrapolation attempted')
                return 1.
        p = Program(source=Source())
        r = simulate_maneuver(Plant(), p, [0.], SimulationSettings())
        self.assertEqual(r.termination_reason, 'source_end')
        self.assertAlmostEqual(r.time_s[-1], .04)

    def test_duration_at_nonintegral_tick(self):
        p=Program(.105); r=simulate_maneuver(Plant(),p,[0.],SimulationSettings())
        self.assertEqual(p.tick_count,11)
        self.assertAlmostEqual(r.time_s[-1],.105)

    def test_wall_budget_and_cancel_are_classified(self):
        r=simulate_maneuver(Plant(),Program(),[0.],SimulationSettings(max_wall_s=1e-12))
        self.assertEqual(r.termination_reason,'wall_budget')
        r=simulate_maneuver(Plant(),Program(),[0.],SimulationSettings(cancel=lambda: True))
        self.assertEqual(r.termination_reason,'cancelled')

    def test_model_domain_failure_is_classified(self):
        class Bad(Plant):
            def evaluate(self,t,y,c): raise type('TrialStateError',(Exception,),{})('bad')
        r=simulate_maneuver(Bad(),Program(),[0.],SimulationSettings())
        self.assertEqual(r.termination_reason,'model_limit')

    def test_zero_speed_root(self):
        class SpeedPlant:
            state_size=4
            minimum_speed = .1
            def evaluate(self,t,y,c):
                if y[3] <= self.minimum_speed:
                    raise type('ModelDomainError',(Exception,),{})('outside domain')
                return Solved(np.array([0.,0.,0.,-1.]))
            def observe(self,t,y,c): return float(y[3])
        class BrakeProgram(Program):
            def __init__(self): self.data={'termination':{'duration_s':2.}}; self.csv_source=None; self.tick_count=0
            def tick(self,o,dt): self.tick_count+=1; return 0.
        result=simulate_maneuver(SpeedPlant(),BrakeProgram(),[0.,0.,0.,1.],SimulationSettings())
        self.assertEqual(result.termination_reason,'zero_speed')
        self.assertEqual(result.events[0]['kind'],'zero_speed')
        expected_threshold = .1 + max(1e-6, 1e-3 * max(1., .1))
        self.assertAlmostEqual(result.state[-1, 3], expected_threshold, places=8)
        self.assertAlmostEqual(result.events[0]['speed_threshold_mps'], expected_threshold)
        self.assertGreater(result.events[0]['speed_threshold_mps'], SpeedPlant.minimum_speed)
        self.assertEqual(result.termination_outcome if hasattr(result, 'termination_outcome') else result.execution_outcome, 'stopped')

    def test_initial_speed_between_domain_minimum_and_floor_stops_with_evidence(self):
        class SpeedPlant:
            state_size=4
            minimum_speed=.1
            def evaluate(self,t,y,c):
                if y[3] <= self.minimum_speed:
                    raise type('ModelDomainError',(Exception,),{})('outside domain')
                return Solved(np.array([0.,0.,0.,-1.]))
            def observe(self,t,y,c): return float(y[3])
            def accepted_outputs(self,t,y,c): return {'accepted_speed': float(y[3])}
        class BrakeProgram(Program):
            def __init__(self): self.data={'termination':{'duration_s':2.}}; self.csv_source=None; self.tick_count=0
            def tick(self,o,dt): self.tick_count+=1; return 0.
        start_speed=.1005
        result=simulate_maneuver(SpeedPlant(),BrakeProgram(),[0.,0.,0.,start_speed],SimulationSettings())
        self.assertEqual(result.execution_outcome,'stopped')
        self.assertEqual(result.termination_reason,'zero_speed')
        self.assertEqual(result.events[0]['kind'],'zero_speed')
        self.assertAlmostEqual(result.events[0]['speed_threshold_mps'],.101)
        self.assertAlmostEqual(result.events[0]['state_speed_mps'],start_speed)
        self.assertAlmostEqual(result.state_rate[0,3],-1.)
        self.assertAlmostEqual(result.channels[0]['accepted_speed'],start_speed)
        self.assertNotEqual(result.termination_reason,'model_limit')

    def test_mutable_command_evidence_is_snapshotted(self):
        class CommandProgram(Program):
            def __init__(self):
                super().__init__(.01)
                self.command = {'requested': np.array([1.]), 'applied': [2.]}
            def tick(self, observation, dt): return self.command
        p = CommandProgram()
        result = simulate_maneuver(Plant(), p, [0.], SimulationSettings())
        p.command['requested'][0] = 9.
        p.command['applied'][0] = 8.
        self.assertEqual(result.requested_commands[0]['requested'][0], 1.)
        self.assertEqual(result.applied_commands[-1]['applied'], (2.,))
        with self.assertRaises(ValueError): result.requested_commands[0]['requested'][0] = 3.
        with self.assertRaises(TypeError): result.applied_commands[-1]['new'] = 1


if __name__ == '__main__': unittest.main()
