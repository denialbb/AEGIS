#!/bin/bash
export PYTHONPATH=.

# Simple wrapper that invokes the flight recorder script.
set -euo pipefail

# Resolve directory of this script relative to repository root
BASE_DIR="$(dirname "$(readlink -f "$0")")"
source_dir="$(dirname "$BASE_DIR")"
# Ensure script is in the repo root
cd "$source_dir"

# Execute the recorder Python script
exec .venv/bin/python scripts/flight_recorder.py "$@"
