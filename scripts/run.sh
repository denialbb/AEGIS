#!/bin/bash
export KRPC_ADDRESS=172.22.80.1
export PYTHONPATH=.
echo "Starting AEGIS. Targeting KRPC Server at $KRPC_ADDRESS"
.venv/bin/python src/main.py --hud "$@"
