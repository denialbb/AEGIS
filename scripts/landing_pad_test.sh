#!/bin/bash
export KRPC_ADDRESS=172.22.80.1
export PYTHONPATH=.
echo "Starting AEGIS landing pad test. Targeting KRPC Server at $KRPC_ADDRESS"

echo ""
echo "=== Step 1: Load landing_pad save ==="
.venv/bin/python -c '
import os, sys, krpc, time
import src.config as config
address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
print(f"Connecting to KSP at {address}...")
try:
    conn = krpc.connect(name="AEGIS_Setup", address=address)
    print("Connected.")
except Exception as e:
    print(f"Failed to connect: {e}")
    sys.exit(1)
try:
    print("Loading savefile landing_pad...")
    conn.space_center.load("landing_pad")
    time.sleep(2.0)
    print("Save loaded successfully.")
except Exception as e:
    print(f"Warning: Could not load savefile: {e}")
conn.close()
print("Connection closed.")
'

echo ""
echo "=== Step 2: Validate NED/ECEF/EKF invariants ==="
.venv/bin/python scripts/validate_ned_invariants.py
VALIDATE_RESULT=$?
if [ $VALIDATE_RESULT -ne 0 ]; then
    echo "WARNING: Invariant validation failed ($VALIDATE_RESULT). Continuing..."
fi

echo ""
echo "=== Step 3: Run AEGIS mission ==="
.venv/bin/python src/main.py "$@"
