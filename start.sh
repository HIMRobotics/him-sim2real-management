#!/usr/bin/env bash
set -eu
KIT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
printf '\nG1 session recorder — records only; start your policy separately.\n\n'
read -r -p 'Run name (example: policy-a-standing-01): ' RUN_NAME
read -r -p 'Policy name / Arena submission ID: ' POLICY_LABEL
read -r -p 'Operator name(s): ' OPERATOR_LABEL
read -r -p 'G1 name or serial: ' ROBOT_LABEL
read -r -p 'Robot network interface [auto]: ' ROBOT_INTERFACE
read -r -p 'DDS domain [0]: ' DDS_DOMAIN
read -r -p 'Task, success definition, policy version, and any setup notes: ' RUN_NOTES
exec "${PYTHON:-python3}" "$KIT_DIR/g1_session.py" record \
  --name "$RUN_NAME" --policy "$POLICY_LABEL" --operator "$OPERATOR_LABEL" \
  --robot "$ROBOT_LABEL" --interface "${ROBOT_INTERFACE:-auto}" --domain "${DDS_DOMAIN:-0}" --notes "$RUN_NOTES"
