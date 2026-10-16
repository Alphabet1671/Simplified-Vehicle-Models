"""Repository layout rules: where Markdown lives and which paths are ignored."""

import subprocess
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]

#: Markdown files allowed at the repository root (navigation entry points).
ROOT_MARKDOWN = frozenset({"README.md", "AGENTS.md"})

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
    def test_tracked_markdown_is_under_docs_except_root_entry_points(self):
        offenders = [
            path
            for path in _tracked_files("*.md")
            if path not in ROOT_MARKDOWN and not path.startswith("docs/")
        ]
        self.assertEqual(offenders, [], "Markdown outside docs/: %s" % offenders)

    def test_root_entry_points_exist(self):
        for name in sorted(ROOT_MARKDOWN):
            self.assertTrue((REPOSITORY_ROOT / name).is_file(), name)

    def test_model_data_provenance_moved_to_docs(self):
        self.assertFalse((REPOSITORY_ROOT / "models/data/provenance.md").exists())
        self.assertIn("docs/model-data/provenance.md", _tracked_files("docs/model-data"))

    def test_sdd_logs_are_tracked_under_docs_only(self):
        self.assertEqual(_tracked_files(".superpowers"), [])
        self.assertTrue(
            [p for p in _tracked_files("docs/superpowers/sdd") if p.endswith(".md")],
            "expected tracked SDD Markdown under docs/superpowers/sdd/",
        )


class AgentIgnoreTests(unittest.TestCase):
    def test_agent_workspaces_are_ignored(self):
        for path in IGNORED_AGENT_PATHS:
            with self.subTest(path=path):
                result = _git("check-ignore", "-q", "--no-index", path, check=False)
                self.assertEqual(result.returncode, 0, "%s is not ignored" % path)

    def test_docs_sdd_is_not_ignored(self):
        result = _git(
            "check-ignore", "-q", "--no-index",
            "docs/superpowers/sdd/example/task-1-report.md",
            check=False,
        )
        self.assertEqual(result.returncode, 1)

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
