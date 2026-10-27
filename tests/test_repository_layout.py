"""Repository layout rules: where Markdown lives and which paths are ignored."""

import subprocess
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]

#: Markdown files that are not a directory ``README.md``.
OTHER_MARKDOWN = frozenset({
    "AGENTS.md",
    "docs/model-data/provenance.md",
    "docs/validation/iem24-murphy/seven_dof_track_pair/tube_v2_protocol.md",
})

#: Agent workspaces and scratch directories that must never be tracked.
IGNORED_AGENT_PATHS = (
    ".superpowers/",
    ".superpowers/sdd/scratch.md",
    ".worktrees/",
    ".worktrees/point-mass/README.md",
    ".claude/",
    ".codex/",
    ".cursor/",
    ".gemini/",
    ".opencode/",
    ".agents/",
    ".omx/",
    ".serena/",
)


def _git(*args, check=True):
    return subprocess.run(
        ["git", *args],
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        check=check,
    )


def _tracked_files(*pathspec):
    out = _git("ls-files", "-z", "--", *pathspec).stdout
    return [p for p in out.split("\0") if p]


class MarkdownPlacementTests(unittest.TestCase):
    def test_tracked_markdown_is_a_directory_readme_or_allowed(self):
        offenders = [
            path
            for path in _tracked_files("*.md")
            if Path(path).name != "README.md" and path not in OTHER_MARKDOWN
        ]
        self.assertEqual(offenders, [], "central documentation is not tracked: %s" % offenders)

    def test_root_entry_points_exist(self):
        for name in ("README.md", "AGENTS.md"):
            self.assertTrue((REPOSITORY_ROOT / name).is_file(), name)

    def test_every_tracked_directory_with_files_has_a_readme(self):
        tracked = [p for p in _tracked_files() if "/attempt_" not in p]
        directories = {str(Path(p).parent) for p in tracked}
        missing = sorted(d for d in directories if (d + "/README.md").lstrip("./") not in tracked
                         and not (d == "." and "README.md" in tracked))
        self.assertEqual(missing, [], "directories without README.md: %s" % missing)

    def test_model_data_provenance_stays_with_the_data_docs(self):
        self.assertFalse((REPOSITORY_ROOT / "models/data/provenance.md").exists())
        self.assertIn("docs/model-data/provenance.md", _tracked_files("docs/model-data"))

    def test_agent_working_docs_are_not_tracked(self):
        self.assertEqual(_tracked_files(".superpowers"), [])
        self.assertEqual(_tracked_files("docs/superpowers"), [])
        self.assertEqual(_tracked_files("docs/standards"), [])


class AgentIgnoreTests(unittest.TestCase):
    def test_agent_workspaces_are_ignored(self):
        for path in IGNORED_AGENT_PATHS:
            with self.subTest(path=path):
                result = _git("check-ignore", "-q", "--no-index", path, check=False)
                self.assertEqual(result.returncode, 0, "%s is not ignored" % path)

    def test_docs_superpowers_is_ignored(self):
        result = _git(
            "check-ignore", "-q", "--no-index",
            "docs/superpowers/sdd/example/task-1-report.md",
            check=False,
        )
        self.assertEqual(result.returncode, 0)

    def test_no_ignored_agent_paths_are_tracked(self):
        for prefix in (".superpowers", ".worktrees", ".claude", ".codex", ".cursor",
                       ".gemini", ".opencode", ".agents", ".omx", ".serena"):
            with self.subTest(prefix=prefix):
                self.assertEqual(_tracked_files(prefix), [])


class AttemptArtifactIgnoreTests(unittest.TestCase):
    ATTEMPT = "docs/validation/iem24-murphy/x_trials/attempt_1792000000000000000/"

    def test_heavy_attempt_artifacts_are_ignored(self):
        for name in ("trace.csv", "pedal.csv", "run.log", "dense_polynomials.npz",
                     "simulated_lap_7dof.csv", "source_snapshot/planar.py"):
            with self.subTest(name=name):
                result = _git("check-ignore", "-q", "--no-index", self.ATTEMPT + name,
                              check=False)
                self.assertEqual(result.returncode, 0)

    def test_attempt_summaries_and_certificates_are_not_ignored(self):
        for name in ("summary.json", "parent.json", "tube_certificate.json",
                     "between_step_certificate.json", "refinement_comparison.json",
                     "simulated_lap_7dof_summary.json", "error.txt"):
            with self.subTest(name=name):
                result = _git("check-ignore", "-q", "--no-index", self.ATTEMPT + name,
                              check=False)
                self.assertEqual(result.returncode, 1)


if __name__ == "__main__":
    unittest.main()
