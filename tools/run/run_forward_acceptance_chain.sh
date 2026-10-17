#!/usr/bin/env bash
# Run one forward-driver acceptance chain (cold -> warm/refined/finer/fourth) with fixed steering gains.
# Usage: run_forward_acceptance_chain.sh MODEL(planar-7dof|14dof-simple) OUT PREVIEW HEADING LATERAL STAGE
# STAGE: cold | warm | refined | finer | fourth. Warm-based stages read OUT/cold/attempt_*.
# Settings mirror docs/validation/iem24-murphy/seven_dof_track_pair/new/final_acceptance_run6.
set -euo pipefail
MODEL=$1; OUT=$2; PREVIEW=$3; HEADING=$4; LATERAL=$5; STAGE=$6
case "$MODEL" in
  planar-7dof) BASE_STEP=0.02; TIMEOUT=300.0;;
  14dof-simple) BASE_STEP=0.005; TIMEOUT=${CHAIN_TIMEOUT:-1800.0};;
  *) echo "unknown model $MODEL" >&2; exit 2;;
esac
case "$STAGE" in
  cold|warm) STEP=$BASE_STEP; RTOL=1e-05;;
  refined) STEP=$(python3 -c "print($BASE_STEP/2)"); RTOL=5e-06;;
  finer) STEP=$(python3 -c "print($BASE_STEP/4)"); RTOL=2.5e-06;;
  fourth) STEP=$(python3 -c "print($BASE_STEP/8)"); RTOL=1.25e-06;;
  *) echo "unknown stage $STAGE" >&2; exit 2;;
esac
ARGS=(--model "$MODEL" --planner live --maximum-speed 60.0 --pedal-travel-time 0.175
      --maximum-step "$STEP" --rtol "$RTOL" --method BDF --spacing 0.5 --duration 300.0
      --timeout-seconds "$TIMEOUT" --preview-seconds "$PREVIEW" --heading-gain "$HEADING"
      --lateral-gain "$LATERAL" --output-root "$OUT/$STAGE")
if [ "$STAGE" != cold ]; then
  COLD=$(ls -d "$OUT"/cold/attempt_* | tail -1)
  ARGS+=(--warm-start "$COLD")
fi
PYTHONPATH=src:. exec python3 tools/run/run_iem24_forward_driver.py "${ARGS[@]}"
