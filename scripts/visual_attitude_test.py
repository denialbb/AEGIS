#!/usr/bin/env python3
import krpc
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_vessel_basis_in_ned(conn, vessel, ned_frame):
    # Transform body axes [1,0,0] (Right), [0,1,0] (Forward/Nose), [0,0,1] (Down/Belly) to NED frame
    v_right = np.array(conn.space_center.transform_direction((1, 0, 0), vessel.reference_frame, ned_frame))
    v_forward = np.array(conn.space_center.transform_direction((0, 1, 0), vessel.reference_frame, ned_frame))
    v_down = np.array(conn.space_center.transform_direction((0, 0, 1), vessel.reference_frame, ned_frame))
    return v_right, v_forward, v_down

def log_attitude_comparison(conn, vessel, ned_frame, step_name):
    # 1. Get physical basis vectors in NED frame
    v_right, v_forward, v_down = get_vessel_basis_in_ned(conn, vessel, ned_frame)
    
    # 2. Get raw kRPC quaternion
    raw_q = vessel.flight(ned_frame).rotation
    q = np.array(raw_q)
    # Ensure standard normalization
    q = q / np.linalg.norm(q)
    if q[3] < 0:
        q = -q
        
    # 3. Direct quaternion rotation matrix columns (SciPy)
    r_direct = R.from_quat(q)
    m_direct = r_direct.as_matrix()
    col_x_dir = m_direct[:, 0]
    col_y_dir = m_direct[:, 1]
    col_z_dir = m_direct[:, 2]
    
    # 4. Conjugated quaternion rotation matrix columns (SciPy)
    q_conj = np.array([-q[0], -q[1], -q[2], q[3]])
    if q_conj[3] < 0:
        q_conj = -q_conj
    r_conj = R.from_quat(q_conj)
    m_conj = r_conj.as_matrix()
    col_x_conj = m_conj[:, 0]
    col_y_conj = m_conj[:, 1]
    col_z_conj = m_conj[:, 2]
    
    print(f"\n--- {step_name} Telemetry & Frame Comparison ---")
    print(f"Raw kRPC quaternion (q): {raw_q}")
    print("\n--- Physical Vessel Axes in NED frame (Ground Truth) ---")
    print(f"Vessel RIGHT (X) in NED:   {v_right}")
    print(f"Vessel NOSE  (Y) in NED:   {v_forward}")
    print(f"Vessel BELLY (Z) in NED:   {v_down}")
    
    print("\n--- Assumption A: Direct Raw Quaternion is Body->NED ---")
    print(f"Col X (maps Body X):       {col_x_dir}")
    print(f"Col Y (maps Body Y/Nose):  {col_y_dir}")
    print(f"Col Z (maps Body Z/Belly): {col_z_dir}")
    
    print("\n--- Assumption B: Conjugated Quaternion is Body->NED (Current sensors.py) ---")
    print(f"Col X (maps Body X):       {col_x_conj}")
    print(f"Col Y (maps Body Y/Nose):  {col_y_conj}")
    print(f"Col Z (maps Body Z/Belly): {col_z_conj}")
    
    # Check error norms
    err_dir_x = np.linalg.norm(v_right - col_x_dir)
    err_dir_y = np.linalg.norm(v_forward - col_y_dir)
    err_dir_z = np.linalg.norm(v_down - col_z_dir)
    total_err_dir = err_dir_x + err_dir_y + err_dir_z
    
    err_conj_x = np.linalg.norm(v_right - col_x_conj)
    err_conj_y = np.linalg.norm(v_forward - col_y_conj)
    err_conj_z = np.linalg.norm(v_down - col_z_conj)
    total_err_conj = err_conj_x + err_conj_y + err_conj_z
    
    print(f"\nTotal axis discrepancy:")
    print(f"  - Direct Raw Quaternion (A): {total_err_dir:.6f}")
    print(f"  - Conjugated Quaternion (B): {total_err_conj:.6f}")
    
    if total_err_dir < total_err_conj:
        print(">> VERDICT: Direct Raw Quaternion matches physical basis! (Assumption A is correct)")
    else:
        print(">> VERDICT: Conjugated Quaternion matches physical basis! (Assumption B is correct)")

def run_visual_test():
    print("Connecting to kRPC...")
    try:
        conn = krpc.connect(
            name="Visual Attitude Test",
            address="172.22.80.1",
            rpc_port=50000,
            stream_port=50001
        )
    except Exception as e:
        print(f"Error connecting: {e}")
        sys.exit(1)

    print("Loading save 'AEGIS MK2'...")
    try:
        conn.space_center.load("AEGIS MK2")
    except Exception as e:
        print(f"Error loading save: {e}")
        sys.exit(1)

    time.sleep(2.0)  # Wait for scene load and physics to settle

    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    
    # Construct NED frame just like main.py
    # We must import geometry to match AEGIS behavior
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from src.common.geometry import ecef_to_ned
    
    pad_ecef = vessel.position(body.reference_frame)
    _, ned_quat, _, _ = ecef_to_ned(np.array(pad_ecef))
    ned_frame = conn.space_center.ReferenceFrame.create_relative(
        body.reference_frame,
        rotation=tuple(ned_quat)
    )

    # Disable SAS, enable RCS
    vessel.control.sas = False
    vessel.control.rcs = True
    
    print("\n--- Starting Visual Attitude Test with Quaternion Logging ---")
    log_attitude_comparison(conn, vessel, ned_frame, "INITIAL STATE (STABLE)")
    time.sleep(2.0)

    # 1. PITCH DOWN
    print("\n[COMMAND] PITCH DOWN...")
    vessel.control.pitch = 0.5
    time.sleep(1.0)
    log_attitude_comparison(conn, vessel, ned_frame, "DURING PITCH DOWN")
    time.sleep(0.5)
    vessel.control.pitch = 0.0
    print("[STOP] Command cleared. Let settle...")
    time.sleep(3.0)

    # 2. PITCH UP
    print("\n[COMMAND] PITCH UP...")
    vessel.control.pitch = -0.5
    time.sleep(1.0)
    log_attitude_comparison(conn, vessel, ned_frame, "DURING PITCH UP")
    time.sleep(0.5)
    vessel.control.pitch = 0.0
    print("[STOP] Command cleared. Let settle...")
    time.sleep(3.0)

    # 3. YAW LEFT
    print("\n[COMMAND] YAW LEFT...")
    vessel.control.yaw = -0.5
    time.sleep(1.0)
    log_attitude_comparison(conn, vessel, ned_frame, "DURING YAW LEFT")
    time.sleep(0.5)
    vessel.control.yaw = 0.0
    print("[STOP] Command cleared. Let settle...")
    time.sleep(3.0)

    # 4. YAW RIGHT
    print("\n[COMMAND] YAW RIGHT...")
    vessel.control.yaw = 0.5
    time.sleep(1.0)
    log_attitude_comparison(conn, vessel, ned_frame, "DURING YAW RIGHT")
    time.sleep(0.5)
    vessel.control.yaw = 0.0
    print("[STOP] Command cleared.")
    
    print("\n--- Visual Attitude Test Concluded ---")

if __name__ == "__main__":
    run_visual_test()
