#!/bin/bash
export KRPC_ADDRESS=172.22.80.1
export PYTHONPATH=.
echo "Starting AEGIS Optuna tuner. Targeting KRPC at $KRPC_ADDRESS"
.venv/bin/python scripts/tune_config_optuna.py "$@"
