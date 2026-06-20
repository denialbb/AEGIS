#!/bin/bash
export KRPC_ADDRESS=172.22.80.1
export PYTHONPATH=.
echo "Debug landing pad reference frame. Targeting KRPC Server at $KRPC_ADDRESS"

.venv/bin/python -c '
import os
import krpc
import numpy as np
import src.config as config

address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
print(f"Connecting to KSP at {address}...")

conn = krpc.connect(name="Debug", address=address)
print("Connected.")

# Load landing pad save
conn.space_center.load("landing_pad")
import time
time.sleep(2.0)

vessel = conn.space_center.active_vessel
body = vessel.orbit.body

# Create the same reference frame as AEGIS uses
target_lat = config.TARGET_LAT
target_lon = config.TARGET_LON

ref_frame = conn.space_center.ReferenceFrame.create_relative(
    body.reference_frame,
    position=body.surface_position(target_lat, target_lon, body.reference_frame),
)

# Get up vector
pad_pos = np.array(body.surface_position(target_lat, target_lon, body.reference_frame))
up_vector = pad_pos / np.linalg.norm(pad_pos)

# Get vessel state in this frame
vessel_pos = np.array(vessel.position(ref_frame))
vessel_vel = np.array(vessel.flight(ref_frame).velocity)
vessel_alt = vessel.flight(ref_frame).surface_altitude
vessel_sit = vessel.situation.name

print(f"\n=== REFERENCE FRAME DEBUG ===")
print(f"Landing pad position (body frame): {pad_pos}")
print(f"Up vector (unit): {up_vector}")
print(f"Vessel position (ref_frame): {vessel_pos}")
print(f"Vessel velocity (ref_frame): {vessel_vel}")
print(f"Vessel surface altitude: {vessel_alt:.2f} m")
print(f"Vessel situation: {vessel_sit}")
print(f"\nAltitude from up_vector projection: {np.dot(vessel_pos, up_vector):.2f} m")
print(f"Lateral offset magnitude: {np.linalg.norm(vessel_pos - np.dot(vessel_pos, up_vector)*up_vector):.2f} m")

conn.close()
'