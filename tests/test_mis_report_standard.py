"""Enforce the MIS 2024 report standard on generated validation reports.

The standard lives in ``docs/standards/mis-2024-report-standard.md``.  Every
``report.md`` under ``docs/validation/`` is checked here, so a report that drifts
out of format fails the suite rather than passing review unnoticed.
"""

import re
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
STANDARD_PATH = REPOSITORY_ROOT / "docs/standards/mis-2024-report-standard.md"
VALIDATION_ROOT = REPOSITORY_ROOT / "docs/validation"

#: Required second-level headings, in the order the standard fixes them.
REQUIRED_SECTIONS = (
    "Configuration",
    "Measured data",
    "Trace comparison",
    "Reading the comparison",
    "Caveats",
    "Reproduce",
)

#: Channels every comparison figure and metric table must cover.
REQUIRED_CHANNELS = (
    "Speed",
    "Longitudinal acceleration",
    "Lateral acceleration",
    "Yaw rate",
    "Motor torque request",
    "Pack terminal voltage",
)

#: Metric-table columns the standard requires.
REQUIRED_METRIC_COLUMNS = (
    "RMSE",
    "MAE",
    "Bias (sim − meas)",
    "Correlation",
    "Normalised RMSE",
)

#: Figures every report must embed.
REQUIRED_FIGURES = ("comparison.png", "lap_spread.png", "simulated_lap.png")

IMAGE_LINK = re.compile(r"!\[(?P<alt>[^\]]*)\]\((?P<target>[^)]+)\)")


def report_paths() -> list[Path]:
    """Return every generated report the standard covers."""

    return sorted(VALIDATION_ROOT.glob("**/report.md"))


class MisReportStandardTests(unittest.TestCase):
    def test_the_standard_document_exists_and_is_linked(self):
        self.assertTrue(STANDARD_PATH.is_file())
        text = STANDARD_PATH.read_text(encoding="utf-8")
        self.assertIn("MIS 2024 report standard", text)
        for section in REQUIRED_SECTIONS:
            self.assertIn(section, text)

    def test_at_least_one_report_is_covered(self):
        self.assertTrue(
            report_paths(),
            "no docs/validation/**/report.md exists for the standard to cover",
        )

    def test_every_required_section_appears_in_order(self):
        for path in report_paths():
            with self.subTest(report=str(path.relative_to(REPOSITORY_ROOT))):
                text = path.read_text(encoding="utf-8")
                headings = [
                    line[3:].strip()
                    for line in text.splitlines()
                    if line.startswith("## ")
                ]
                remaining = list(REQUIRED_SECTIONS)
                for heading in headings:
                    if remaining and heading == remaining[0]:
                        remaining.pop(0)
                self.assertEqual(
                    remaining,
                    [],
                    f"missing or out-of-order sections: {remaining}",
                )

    def test_metric_table_covers_every_required_channel(self):
        for path in report_paths():
            with self.subTest(report=str(path.relative_to(REPOSITORY_ROOT))):
                text = path.read_text(encoding="utf-8")
                table = _metric_table(text)
                for channel in REQUIRED_CHANNELS:
                    self.assertIn(
                        channel,
                        table,
                        f"metric table is missing a {channel!r} row",
                    )
                for column in REQUIRED_METRIC_COLUMNS:
                    self.assertIn(
                        column,
                        table,
                        f"metric table is missing the {column!r} column",
                    )

    def test_every_embedded_figure_exists_on_disk(self):
        for path in report_paths():
            text = path.read_text(encoding="utf-8")
            links = IMAGE_LINK.findall(text)
            with self.subTest(report=str(path.relative_to(REPOSITORY_ROOT))):
                self.assertTrue(links, "report embeds no figures")
                for alt, target in links:
                    self.assertTrue(alt.strip(), f"empty alt text for {target}")
                    self.assertFalse(
                        target.startswith("http"),
                        "report figures must be local files",
                    )
                    resolved = (path.parent / target).resolve()
                    self.assertTrue(resolved.is_file(), f"missing figure {target}")

    def test_required_figures_are_embedded(self):
        for path in report_paths():
            text = path.read_text(encoding="utf-8")
            embedded = {target for _, target in IMAGE_LINK.findall(text)}
            with self.subTest(report=str(path.relative_to(REPOSITORY_ROOT))):
                for figure in REQUIRED_FIGURES:
                    self.assertIn(figure, embedded)

    def test_caveats_and_reproduction_are_not_empty(self):
        for path in report_paths():
            text = path.read_text(encoding="utf-8")
            with self.subTest(report=str(path.relative_to(REPOSITORY_ROOT))):
                caveats = _section_body(text, "Caveats")
                self.assertGreater(
                    len(caveats.strip()), 80, "the caveats section is too thin"
                )
                reproduce = _section_body(text, "Reproduce")
                self.assertIn("```", reproduce, "no command block to reproduce with")
                self.assertIn("python3", reproduce)

    def test_every_report_links_back_to_the_standard(self):
        for path in report_paths():
            text = path.read_text(encoding="utf-8")
            with self.subTest(report=str(path.relative_to(REPOSITORY_ROOT))):
                targets = [
                    target
                    for _, target in re.findall(
                        r"\[([^\]]+)\]\(([^)]+)\)", text
                    )
                ]
                resolved = [
                    (path.parent / target).resolve()
                    for target in targets
                    if not target.startswith("http")
                ]
                self.assertIn(
                    STANDARD_PATH.resolve(),
                    resolved,
                    "the report does not link back to the report standard",
                )

    def test_document_link_targets_exist(self):
        for path in report_paths():
            text = path.read_text(encoding="utf-8")
            with self.subTest(report=str(path.relative_to(REPOSITORY_ROOT))):
                for _, target in re.findall(r"\[([^\]]+)\]\(([^)]+)\)", text):
                    if target.startswith("http") or target.startswith("#"):
                        continue
                    self.assertTrue(
                        (path.parent / target).resolve().exists(),
                        f"broken link {target}",
                    )

    def test_sign_convention_and_filter_are_declared(self):
        for path in report_paths():
            text = path.read_text(encoding="utf-8")
            with self.subTest(report=str(path.relative_to(REPOSITORY_ROOT))):
                self.assertIn("sim − meas", text)
                self.assertRegex(
                    text,
                    r"low-pass filter\w*[^.]{0,120}?[\d.]+\s*Hz",
                    "the report does not state the comparison filter cutoff",
                )
                self.assertRegex(
                    text,
                    r"median[^.]{0,60}flying laps",
                    "the report does not state the measured lap statistic",
                )


def _section_body(text: str, heading: str) -> str:
    """Return the body of one second-level section."""

    marker = f"## {heading}"
    start = text.find(marker)
    if start < 0:
        return ""
    start += len(marker)
    end = text.find("\n## ", start)
    return text[start:] if end < 0 else text[start:end]


def _metric_table(text: str) -> str:
    """Return the body of the trace-comparison metric table."""

    body = _section_body(text, "Trace comparison")
    lines = [line for line in body.splitlines() if line.startswith("|")]
    return "\n".join(lines)


if __name__ == "__main__":
    unittest.main()
