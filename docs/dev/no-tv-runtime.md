# Reproducible no-TV solver runtime

The `no-tv` optional dependency set declares the solver's optional SciPy and
CasADi requirements; the base package remains unchanged. The tested runtime
recorded here is Python 3.13 with NumPy 2.5.4, SciPy 1.17.1, and CasADi 3.8.1.
These versions are a tested combination, not a promise that wheels exist for
every OS, CPU architecture, or Python build. Installation success and runtime
compatibility do not qualify solver numerical results.

## Clean isolated installation

Use a dedicated virtual environment (shown outside the checkout to avoid
untracked environment files), and the Python 3.13 interpreter:

```sh
python3.13 -m venv "$HOME/.venvs/no-tv-runtime"
"$HOME/.venvs/no-tv-runtime/bin/python" -m pip install -r requirements/no-tv-runtime-py313.txt
"$HOME/.venvs/no-tv-runtime/bin/python" -m pip install --no-deps -e .
"$HOME/.venvs/no-tv-runtime/bin/python" -m tools.diagnostics.no_tv_runtime --require-exact
```

The runtime check does not install, download, or run an NLP. It checks the
interpreter and platform, distribution metadata, and the versions and paths of
the modules actually imported. A mismatch between distribution metadata and
executed module is reported explicitly (for example, when `PYTHONPATH` shadows
the installed package). `--require-exact` additionally enforces Python 3.13 and
the three versions above. The ordinary compatibility check only enforces
Python >=3.12, NumPy >=2, SciPy >=1.11, and CasADi >=3.6,<4.

For an audit artifact, choose a new path; the CLI uses exclusive creation and
refuses existing files and symlinks:

```sh
"$HOME/.venvs/no-tv-runtime/bin/python" -m tools.diagnostics.no_tv_runtime \
  --require-exact --output /path/to/new-runtime-manifest.json
```

Preserve that manifest with the solver attempt, along with its inputs, code
revision, and attempt-specific provenance. The legacy `/tmp` CasADi wheel
location is a research setup, not a production installation prerequisite. A
runtime manifest documents what executed; it does not substitute for
independent numerical acceptance or all-track solver qualification.
