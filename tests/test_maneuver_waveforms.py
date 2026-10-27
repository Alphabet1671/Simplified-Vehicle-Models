import unittest
from simplified_vehicle_models.controls.maneuvers.waveforms import SteeringWaveform

class WaveformTests(unittest.TestCase):
    def test_step_boundaries(self):
        w=SteeringWaveform('step', .4, 1, 2, 1)
        self.assertEqual(w.sample(.99), 0)
        self.assertEqual(w.sample(1), .4)
        self.assertEqual(w.sample(3), .4)
        self.assertEqual(w.sample(3.5), .2)
        self.assertEqual(w.sample(4), 0)
        self.assertEqual(w.breakpoints(), (0.,1.,3.,4.))
    def test_ramp_and_sine(self):
        w=SteeringWaveform('ramp', .4, 1, 2, 1)
        self.assertEqual(w.sample(2.), .2)
        self.assertEqual(w.sample(3), .4)
        self.assertEqual(w.sample(4), 0)
        s=SteeringWaveform('sine', .4, 1, 2, 1, .5)
        self.assertAlmostEqual(s.sample(1.5), .4)
        self.assertEqual(s.sample(3), 0)
    def test_invalid(self):
        for kw in ({'kind':'bad'}, {'active_s':0}, {'amplitude_rad':float('nan')}):
            args=dict(kind='step',amplitude_rad=.2,lead_in_s=0,active_s=1,release_s=0);args.update(kw)
            with self.assertRaises(ValueError): SteeringWaveform(**args)
if __name__=='__main__': unittest.main()
