#!/bin/bash
export KRPC_ADDRESS=172.22.80.1
export PYTHONPATH=.

echo "Running Apogee Test..."
./scripts/apogee_test.sh "$@"

LATEST_CSV="logs/latest/telemetry.csv"

if [ -f "$LATEST_CSV" ]; then
    echo "Scoring latest run..."
    .venv/bin/python scripts/score_landing.py "$LATEST_CSV"
else
    echo "No telemetry.csv found in logs/latest/"
fi
