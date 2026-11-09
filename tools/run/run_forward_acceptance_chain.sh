#!/usr/bin/env bash
# Run a cold seed and a warm flying lap with identical vehicle, track and controls.
# Usage: ... MODEL OUT PREVIEW HEADING LATERAL [cold|warm|both] [extra runner args...]
# PYTHON may select the interpreter; CHAIN_TIMEOUT selects the per-run wall budget.
set -euo pipefail
if [ "$#" -lt 5 ]; then
  echo "usage: $0 MODEL OUT PREVIEW HEADING LATERAL [cold|warm|both] [runner args...]" >&2
  exit 2
fi
MODEL=$1; OUT=$2; PREVIEW=$3; HEADING=$4; LATERAL=$5
shift 5
STAGE=${1:-both}
if [ "$#" -gt 0 ]; then shift; fi
PYTHON=${PYTHON:-python3}
case "$MODEL" in
  planar-7dof) STEP=0.02; TIMEOUT=${CHAIN_TIMEOUT:-600.0};;
  14dof-simple) STEP=0.01; TIMEOUT=${CHAIN_TIMEOUT:-1800.0};;
  *) echo "unknown model $MODEL" >&2; exit 2;;
esac
case "$STAGE" in
  cold|warm|both) ;;
  *) echo "unknown stage $STAGE (expected cold, warm or both; refinement stages were removed)" >&2; exit 2;;
esac
ARGS=(--model "$MODEL" --maximum-speed 60.0 --pedal-travel-time 0.175
      --maximum-step "$STEP" --rtol 1e-5 --method BDF --spacing 0.5 --duration 300.0
      --timeout-seconds "$TIMEOUT" --preview-seconds "$PREVIEW" --heading-gain "$HEADING"
      --lateral-gain "$LATERAL")
# Stage-owned arguments must not be overridden by the optional shared settings.
for ARG in "$@"; do
  case "$ARG" in
    --output-root|--output-root=*|--warm-start|--warm-start=*)
      echo "output-root and warm-start are owned by the acceptance chain" >&2; exit 2;;
  esac
done
if [ "$STAGE" = cold ] || [ "$STAGE" = both ]; then
  PYTHONPATH=src:. "$PYTHON" tools/run/run_iem24_forward_driver.py \
    "${ARGS[@]}" "$@" --output-root "$OUT/cold"
fi
if [ "$STAGE" = warm ] || [ "$STAGE" = both ]; then
  # Select the newest completed seed from this chain, never an incomplete attempt.
  COLD=$("$PYTHON" - "$OUT/cold" <<'PY'
import json
from pathlib import Path
import sys
for path in sorted(Path(sys.argv[1]).glob('attempt_*'), reverse=True):
    try:
        summary = json.loads((path / 'summary.json').read_text())
    except (OSError, ValueError):
        continue
    if summary.get('status') == 'Completed' and summary.get('lap_completed') is True:
        print(path)
        break
else:
    raise SystemExit('no completed cold seed found')
PY
)
  PYTHONPATH=src:. "$PYTHON" tools/run/run_iem24_forward_driver.py \
    "${ARGS[@]}" "$@" --output-root "$OUT/warm" --warm-start "$COLD"
fi
