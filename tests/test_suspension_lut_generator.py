import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.model_data import generate_suspension_lut as gen

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "iem24_dss_suspension_rows.csv"
PRIVATE_XLSX = ROOT / "IEM24 Files" / "IEM_24_DSS.xlsx"
K_FRONT = 450 * gen.LBF_IN_TO_N_M
K_REAR = 350 * gen.LBF_IN_TO_N_M


def fixture_sha():
    return hashlib.sha256(FIXTURE.read_bytes()).hexdigest()


def build(cells, **kwargs):
    return gen.build_corner_arrays(
        cells,
        wheel_rate_front_n_m=K_FRONT,
        wheel_rate_rear_n_m=K_REAR,
        static_load_front_n=505.777974,
        static_load_rear_n=646.503401,
        **kwargs,
    )


class RealDataTests(unittest.TestCase):
    def test_private_source_directory_is_git_ignored(self):
        result = subprocess.run(
            ["git", "check-ignore", "-q", "sample-private-source.xlsx"],
            cwd=ROOT,
            check=False,
        )
        self.assertEqual(result.returncode, 0)


@unittest.skipUnless(PRIVATE_XLSX.is_file(), "private real-car data is not available")
class PrivateWorkbookTests(unittest.TestCase):
    def test_private_workbook_and_fixture_values_match(self):
        real = gen.read_xlsx_cells(PRIVATE_XLSX)
        fixture = gen.read_csv_cells(FIXTURE)
        for address in (
            "D17", "E17", "G17", "H17", "C23", "F23", "C24", "F24", "C26", "F26",
            "C29", "F29", "C30", "F30", "D31", "E31", "G31", "H31",
        ):
            self.assertEqual(str(real[address]), fixture[address], address)
        for axle in ("front", "rear"):
            self.assertEqual(gen.read_axle_cells(real, axle), gen.read_axle_cells(fixture, axle))


if __name__ == "__main__":
    unittest.main()
