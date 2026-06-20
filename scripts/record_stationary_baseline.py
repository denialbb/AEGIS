#!/usr/bin/env python3
import time
import sys
import os
import krpc
import numpy as np
import csv
from scipy.spatial.transform import Rotation as R

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.common.geometry import ecef_to_ned
from src.telemetry.sensors import SensorModels
import src.config as config

def main():
    print("Connecting to kRPC...")
    try:
        conn = krpc.connect(
            name="Stationary Baseline Test",
            address="172.22.80.1",
            rpc_port=50000,
            stream_port=50001
        )
    except Exception as e:
        print(f"Error connecting: {e}")
        sys.exit(1)

    print("Loading save 'MK2_pad'...")
    try:
        conn.space_center.load("MK2_pad")
    except Exception as e:
        print(f"Error loading save: {e}")
        sys.exit(1)

    time.sleep(2.0)  # Wait for scene load and physics to settle

    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body

    pad_ecef = vessel.position(body.reference_frame)
    up_vector, ned_quat, _, _ = ecef_to_ned(np.array(pad_ecef))
    ned_frame = conn.space_center.ReferenceFrame.create_relative(
        body.reference_frame,
        rotation=tuple(ned_quat)
    )

    print("Initializing SensorModels...")
    sensors = SensorModels(conn, vessel, ned_frame, up_vector)

    # Let the streams populate
    for _ in range(5):
        sensors.poll()
        time.sleep(0.1)

    print("Recording telemetry (5 seconds at ~20Hz)...")
    
    os.makedirs("logs", exist_ok=True)
    log_file = "logs/stationary_baseline.csv"
    
    records = []
    
    dt = 1.0 / 20.0
    for i in range(100):
        # Poll returns: noisy_alt, sf_body, attitude, mass, aero_body, situation, omega_body, vel, gravity_ned, raw_gyro
        data = sensors.poll()
        records.append({
            'tick': i,
            'time': time.time(),
            'alt': data[0],
            'sf_x': data[1][0], 'sf_y': data[1][1], 'sf_z': data[1][2],
            'q_x': data[2][0], 'q_y': data[2][1], 'q_z': data[2][2], 'q_w': data[2][3],
            'omega_x': data[6][0], 'omega_y': data[6][1], 'omega_z': data[6][2],
            'grav_x': data[8][0], 'grav_y': data[8][1], 'grav_z': data[8][2],
            'gyro_x': data[9][0], 'gyro_y': data[9][1], 'gyro_z': data[9][2],
        })
        time.sleep(dt)

    with open(log_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)

    print(f"Recorded 100 samples to {log_file}")
    
    # ---------------------------------------------------------
    # Analyze Data
    # ---------------------------------------------------------
    
    sf_body_arr = np.array([[r['sf_x'], r['sf_y'], r['sf_z']] for r in records])
    omega_arr = np.array([[r['omega_x'], r['omega_y'], r['omega_z']] for r in records])
    gyro_arr = np.array([[r['gyro_x'], r['gyro_y'], r['gyro_z']] for r in records])
    
    sf_mean = np.mean(sf_body_arr, axis=0)
    sf_std = np.std(sf_body_arr, axis=0)
    
    omega_mean = np.mean(omega_arr, axis=0)
    omega_std = np.std(omega_arr, axis=0)
    
    print("\n--- Sensor Baseline Analysis ---")
    print(f"Specific Force Body Mean (m/s²): X={sf_mean[0]:.4f}, Y={sf_mean[1]:.4f}, Z={sf_mean[2]:.4f}")
    print(f"Specific Force Body StdDev     : X={sf_std[0]:.4f}, Y={sf_std[1]:.4f}, Z={sf_std[2]:.4f}")
    
    print(f"Omega Body Mean (rad/s)        : X={omega_mean[0]:.4f}, Y={omega_mean[1]:.4f}, Z={omega_mean[2]:.4f}")
    print(f"Omega Body StdDev              : X={omega_std[0]:.4f}, Y={omega_std[1]:.4f}, Z={omega_std[2]:.4f}")

    # ---------------------------------------------------------
    # Reference Frame Diagnostic
    # ---------------------------------------------------------
    
    last = records[-1]
    q = np.array([last['q_x'], last['q_y'], last['q_z'], last['q_w']])
    rot = R.from_quat(q)
    gravity_ned = np.array([last['grav_x'], last['grav_y'], last['grav_z']])
    
    # When stationary, coordinate acceleration is 0. 
    # Therefore, Specific Force (proper acceleration) should be exactly -gravity.
    # sf_ned = 0 - gravity_ned = -gravity_ned
    expected_sf_ned = -gravity_ned
    
    # sf_body = rot.inv().apply(sf_ned)
    expected_sf_body = rot.inv().apply(expected_sf_ned)
    
    actual_sf_body = np.array([last['sf_x'], last['sf_y'], last['sf_z']])
    
    print("\n--- Reference Frame Diagnostic (Last Tick) ---")
    print(f"Attitude Quaternion [x,y,z,w]: {q}")
    print(f"Gravity in NED               : {gravity_ned}")
    print(f"Expected Specific Force NED  : {expected_sf_ned}")
    print(f"Expected Specific Force Body : {expected_sf_body}")
    print(f"Actual Specific Force Body   : {actual_sf_body}")
    
    error = np.linalg.norm(expected_sf_body - actual_sf_body)
    print(f"\nMagnitude of difference between Expected and Actual SF: {error:.4f} m/s²")
    
    if error < 1.0:
        print("VERDICT: PASS. The attitude quaternion perfectly maps world gravity into the body frame!")
    else:
        print("VERDICT: FAIL. The expected specific force (rotated gravity) does not match what the accelerometer measures in the body frame. The quaternion convention or basis vectors are incorrect.")

if __name__ == "__main__":
    main()
