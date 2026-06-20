#!/bin/bash
export KRPC_ADDRESS=172.22.80.1
export PYTHONPATH=.
echo "Starting Engine Test. Targeting KRPC Server at $KRPC_ADDRESS"
.venv/bin/python scripts/test_engines.py "$@"
