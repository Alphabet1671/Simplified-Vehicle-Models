"""All consumers opt into the same fail-closed vehicle-limit build policy."""
import argparse
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from tools.run.yaw_map_options import add_yaw_map_options, resolve_yaw_map, IncompleteYawMapError


class MapOptionsTests(unittest.TestCase):
    def test_vehicle_limit_is_default_not_one_g(self):
        parser=argparse.ArgumentParser(); add_yaw_map_options(parser)
        args=parser.parse_args([])
        self.assertEqual(args.yaw_map_mode,'vehicle-limit')
        self.assertEqual(args.yaw_ay_spacing,.5)

    def test_default_dispatch_uses_workflow_and_rejects_incomplete_result(self):
        args=SimpleNamespace()
        audit=SimpleNamespace(complete=False)
        result=SimpleNamespace(complete=False, capability=object(), audit=audit)
        with patch('tools.run.yaw_map_options.build_identity',return_value='id'), \
             patch('simplified_vehicle_models.formulations.yaw_map_workflow.build_yaw_map_workflow',return_value=result) as build, \
             patch('tools.run.yaw_map_options.asdict',return_value={'complete':False}):
            with self.assertRaises(IncompleteYawMapError):
                resolve_yaw_map(object(),1.,[0.,10.],[-1.,0.,1.],args)
        self.assertEqual(build.call_count,1)

    def test_explicit_limited_mode_is_labelled_not_qualified(self):
        args=SimpleNamespace(yaw_map_mode='limited-grid')
        with patch('simplified_vehicle_models.formulations.yaw_capability.build_physical_yaw_map',return_value='cap'):
            cap,report=resolve_yaw_map(object(),1.,[0.,10.],[-1.,0.,1.],args,limited_ay=[-9.81,0.,9.81])
        self.assertEqual(cap,'cap');self.assertFalse(report['complete'])
        self.assertIn('limited-domain',report['scope'])
