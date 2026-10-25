import hashlib
import math
from unittest import mock

import tempfile
import unittest
from pathlib import Path
from simplified_vehicle_models.controls.maneuvers.csv_source import CsvMapping, load_csv_source

class CsvTests(unittest.TestCase):
    def load(self,text,mapping=None):
        d=tempfile.TemporaryDirectory(); self.addCleanup(d.cleanup); p=Path(d.name)/'i.csv'; p.write_text(text,encoding='utf-8')
        return p,load_csv_source(p,mapping or CsvMapping('t','s','a','b','s','deg','percent','hold'))
    def test_import_and_hash(self):
        p,s=self.load('t,s,a,b\n0,0,20,0\n1,30,20,0\n2,0,0,0\n')
        self.assertEqual(s.sample(.5).swa_rad,0); self.assertAlmostEqual(s.sample(1).swa_rad,math.pi/6)
        self.assertEqual(s.sample(2).accelerator,0); self.assertEqual(s.source_bytes,p.read_bytes()); self.assertEqual(s.source_sha256,hashlib.sha256(s.source_bytes).hexdigest())
        with self.assertRaises(ValueError): s.sample(2.01)
        with self.assertRaises(ValueError): s.sample(-.1)
    def test_linear_crossing_and_rebase(self):
        with self.assertRaises(ValueError): self.load('t,s,a,b\n0,0,100,0\n1,0,0,100\n',CsvMapping('t','s','a','b','s','deg','percent','linear'))
        with self.assertRaises(ValueError): self.load('t,s,a,b\n1,0,0,0\n2,0,0,0\n')
        _,s=self.load('t,s,a,b\n1,0,0,0\n2,0,0,0\n',CsvMapping('t','s','a','b','s','deg','percent','hold',rebase=True))
        self.assertEqual(s.time_s[0],0)
    def test_bad_data_and_delimiters(self):
        for content in ('t,s,a,b\n0,0,0,0\n0,1,0,0\n', 't,s,a,b\n0,0,nan,0\n1,0,0,0\n', 't,s,a,b\n0,0,101,0\n1,0,0,0\n'):
            with self.subTest(content=content), self.assertRaises(ValueError): self.load(content)
        m=CsvMapping('t','s','a','b','ms','rad','fraction','hold',delimiter=';',decimal=',')
        _,s=self.load('\ufefft;s;a;b\n0;0;0;0\n1000;0;0;0\n',m); self.assertEqual(s.time_s[-1],1)
    def test_limits_before_read_and_rows(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'large.csv'
            with mock.patch.object(Path,'stat',return_value=mock.Mock(st_size=100*1024*1024+1)), mock.patch.object(Path,'read_bytes',side_effect=AssertionError('must not read')):
                with self.assertRaisesRegex(ValueError,'100 MiB'): load_csv_source(p,CsvMapping('t','s','a','b','s','deg','percent','hold'))
        mapping=CsvMapping('t','s','a','b','s','deg','percent','hold')
        with mock.patch('simplified_vehicle_models.controls.maneuvers.csv_source._MAX_ROWS',2):
            with self.assertRaisesRegex(ValueError,'1,000,000'):
                self.load('t,s,a,b\n0,0,0,0\n1,0,0,0\n2,0,0,0\n',mapping)
    def test_invalid_mapping_header_and_envelopes(self):
        for mapping in (CsvMapping('missing','s','a','b','s','deg','percent','hold'),):
            with self.assertRaises(ValueError): self.load('t,s,a,b\n0,0,0,0\n1,0,0,0\n',mapping)
        for content in ('t,s,a,b\n0,-181,0,0\n1,0,0,0\n','t,s,a,b\n0,0,-1,0\n1,0,0,0\n','t,s,a,b\n0,0,0,0\n1,0,0,101\n','t,s,a,a\n0,0,0,0\n1,0,0,0\n'):
            with self.subTest(content=content),self.assertRaises(ValueError):self.load(content)
    def test_malformed_rows_and_numeric_values(self):
        malformed = (
            ('t,s,a,b\n0,0,0\n1,0,0,0\n', 'row width'),
            ('t,s,a,b\n0,0,0,0\n1,0,0,"unterminated\n', 'quoting'),
            ('t,s,a,b\n0,0,,0\n1,0,0,0\n', 'empty mapped value'),
            ('t,s,a,b\n0,0,not-a-number,0\n1,0,0,0\n', 'invalid numeric'),
            ('t,s,a,b\n0,0,inf,0\n1,0,0,0\n', 'nonfinite numeric'),
        )
        for content, label in malformed:
            with self.subTest(case=label), self.assertRaises(ValueError):
                self.load(content)
    def test_explicit_timebase(self):
        m=CsvMapping(None,'s','a','b','s','rad','fraction','hold',sample_dt_s=.5)
        _,s=self.load('s,a,b\n0,0,0\n1,0,0\n',m); self.assertEqual(tuple(s.time_s),(0,.5))
if __name__=='__main__': unittest.main()
