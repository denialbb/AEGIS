#!/bin/bash
export KRPC_ADDRESS=172.22.80.1
export PYTHONPATH=.
echo "Starting AEGIS apogee test. Targeting KRPC Server at $KRPC_ADDRESS"

# Run the save loading setup then AEGIS
.venv/bin/python -c '
import os
import sys
import krpc
import time
import src.config as config

address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
print(f"Connecting to KSP at {address}...")

try:
    conn = krpc.connect(name="AEGIS_Setup", address=address)
    print("Connected.")
except Exception as e:
    print(f"Failed to connect to kRPC: {e}")
    print("Ensure KSP is running with kRPC server enabled.")
    sys.exit(1)

# Load the savefile "AEGIS-1"
try:
    print("Loading savefile AEGIS-1...")
    conn.space_center.load("AEGIS-1")
    time.sleep(2.0)  # Let physics settle after load
    print("Save loaded successfully.")
except Exception as e:
    print(f"Warning: Could not load savefile: {e}")
    print("Proceeding anyway...")

conn.close()
print("Disconnected. Starting AEGIS...")
'

echo ""
echo "Running AEGIS mission..."
.venv/bin/python src/main.py "$@"