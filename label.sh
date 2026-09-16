#!/usr/bin/env bash
set -eu
KIT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
RUN_PATH="${1:-}"
if [ -z "$RUN_PATH" ]; then read -r -p 'Paste the RUN DIRECTORY: ' RUN_PATH; fi
read -r -p 'Operator: ' OPERATOR_LABEL
printf '\nLabel each attempt, including failures. Reusing a trial ID records a correction.\n'
while true; do
  read -r -p 'Trial ID (example: 001; empty finishes): ' TRIAL_ID
  [ -n "$TRIAL_ID" ] || break
  read -r -p 'Outcome [success / partial / failure / invalid / aborted / unknown]: ' OUTCOME
  read -r -p 'Notes (what happened, intervention, video timestamp): ' TRIAL_NOTES
  "${PYTHON:-python3}" "$KIT_DIR/g1_session.py" mark "$RUN_PATH" --kind label \
    --trial "$TRIAL_ID" --outcome "$OUTCOME" --operator "$OPERATOR_LABEL" --notes "$TRIAL_NOTES"
done
