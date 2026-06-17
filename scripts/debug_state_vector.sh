#!/bin/bash
export KRPC_ADDRESS=172.22.80.1
export PYTHONPATH=.
echo "Debug state vector on landing pad"

.venv/bin/python << 'EOF'
import os
import krpc
import numpy as np
import src.config as config
import time

address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
conn = krpc.connect(name="Debug", address=address)

# Load landing pad save
conn.space_center.load("landing_pad")
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

# Get vessel state
vessel_pos = np.array(vessel.position(ref_frame))
vessel_vel = np.array(vessel.flight(ref_frame).velocity)
vessel_alt = vessel.flight(ref_frame).surface_altitude

print(f"up_vector: {up_vector}")
print(f"vessel_pos (ref_frame): {vessel_pos}")
print(f"vessel_vel (ref_frame): {vessel_vel}")
print(f"vessel_alt: {vessel_alt:.2f} m")
print(f"dot(vessel_pos, up_vector): {np.dot(vessel_pos, up_vector):.2f} m")
print(f"Expected state_vector[:3]: {vessel_pos}")
print(f"Expected state_vector[3:]: {vessel_vel}")

# Simulate what compute_target_state does for POWERED_DESCENT
est_alt = float(np.dot(vessel_pos, up_vector))
target_pos = est_alt * up_vector
target_vel = np.zeros(3)

print(f"\nFor POWERED_DESCENT:")
print(f"  est_alt: {est_alt:.2f} m")
print(f"  target_pos: {target_pos}")
print(f"  target_vel: {target_vel}")
print(f"  pos_err = target - current: {target_pos - vessel_pos}")
print(f"  pos_err_vert: {np.dot(target_pos - vessel_pos, up_vector):.2f} m")

conn.close()
EOF