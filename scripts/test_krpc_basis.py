#!/usr/bin/env python3
import krpc
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.common.geometry import ecef_to_ned

def run_test():
    print("Connecting to kRPC...")
    conn = krpc.connect(name="Basis Test", address="172.22.80.1", rpc_port=50000, stream_port=50001)
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    
    pad_ecef = vessel.position(body.reference_frame)
    R_ecef_to_ned, ned_quat, _, _ = ecef_to_ned(np.array(pad_ecef))
    ned_frame = conn.space_center.ReferenceFrame.create_relative(
        body.reference_frame,
        rotation=tuple(ned_quat)
    )
    
    # Let's get the vessel's X, Y, Z axes in NED frame
    print("Querying vessel axes in NED frame...")
    vessel_x = vessel.direction(ned_frame) # KSP direction is Y (forward). Let's check docs.
    # Actually, KSP vessel.direction is the Y axis (forward).
    # KSP vessel.right is the X axis (right).
    # KSP vessel.up is the -Z axis (top)?
    v_forward = conn.space_center.transform_direction((0, 1, 0), vessel.reference_frame, ned_frame)
    v_right = conn.space_center.transform_direction((1, 0, 0), vessel.reference_frame, ned_frame)
    v_down = conn.space_center.transform_direction((0, 0, 1), vessel.reference_frame, ned_frame)
    
    print(f"Vessel Forward (Nose) in NED: {v_forward}")
    print(f"Vessel Right in NED:          {v_right}")
    print(f"Vessel Down (Belly) in NED:   {v_down}")
    
    # Now let's see what the quaternion gives us
    q_krpc = vessel.flight(ned_frame).rotation
    print(f"\nkRPC rotation quat: {q_krpc}")
    
    # Try interpreting it as scipy Right-Handed
    rot = R.from_quat(q_krpc)
    # The columns of rot.as_matrix() should be the basis vectors if it maps Body -> NED
    # Or rot.inv().as_matrix() if it maps NED -> Body
    mat = rot.as_matrix()
    print(f"\nScipy R.from_quat(q_krpc).as_matrix() columns:")
    print(f"Col X: {mat[:, 0]}")
    print(f"Col Y: {mat[:, 1]}")
    print(f"Col Z: {mat[:, 2]}")
    
    mat_inv = rot.inv().as_matrix()
    print(f"\nScipy R.from_quat(q_krpc).inv().as_matrix() columns:")
    print(f"Col X: {mat_inv[:, 0]}")
    print(f"Col Y: {mat_inv[:, 1]}")
    print(f"Col Z: {mat_inv[:, 2]}")
    
    print("\nCompare Col Y (Forward) to Vessel Forward.")
    print("\nExtracting Euler Angles from M_body_to_ned (ZXY intrinsic):")
    # Z = Yaw, X = Pitch, Y = Roll
    euler_zxy = rot.as_euler("ZXY", degrees=True)
    print(f"Body->NED ZXY (Yaw, Pitch, Roll): {euler_zxy}")
    
    print("\nExtracting Euler Angles from M_ned_to_body (ZXY intrinsic):")
    euler_inv_zxy = rot.inv().as_euler("ZXY", degrees=True)
    print(f"NED->Body ZXY (Yaw, Pitch, Roll): {euler_inv_zxy}")
    
    print("\nWait, KSP flight.pitch is 87.01 (nose up) in surface frame.")
    print("If KSP Pitch is 87, it means the angle between Forward (Y) and Horizon (XY plane) is 87.")
    print("In NED, Horizon is XY plane. Forward is Col Y. Col Y is [0, 0, -1].")
    print("Angle between [0,0,-1] and XY plane is 90 degrees! (Pitch UP).")
    
    print("Therefore, our attitude is correct! Mahony just drifted!")
    
    # Test Torque Generation
    print("\n--- Testing Torque Generation ---")
    # Suppose we want to accelerate North (X axis in NED).
    # Since gravity is Down (Z axis in NED), to accelerate North we must tilt Nose North.
    # So target_up_ned is tilted slightly North. Let's say pitch=20 degrees.
    import math
    pitch_rad = math.radians(20)
    # Nose tilted North means Up vector (which normally points -Z) now has a +X component.
    target_up_ned = np.array([math.sin(pitch_rad), 0.0, -math.cos(pitch_rad)])
    print(f"Target Up in NED: {target_up_ned}")
    
    target_up_body = rot.inv().apply(target_up_ned)
    print(f"Target Up in Body: {target_up_body}")
    
    # Error axis = Nose [0, 1, 0] x target_up_body
    err_axis = np.cross(np.array([0.0, 1.0, 0.0]), target_up_body)
    print(f"Error Axis (Torque command) in Body: {err_axis}")
    
    print("\nLet's verify if this torque pitches the nose NORTH.")
    print("If we apply torque around err_axis, does it pitch the nose towards North?")
    print("Vessel Right in NED is Col X. Vessel Down is Col Z.")
    err_axis_ned = rot.apply(err_axis)
    print(f"Error Axis in NED: {err_axis_ned}")
    
    # Let's rotate Nose [0, 0, -1] around err_axis_ned by a small positive angle
    # and see if it moves towards North [1, 0, 0]!
    rot_err = R.from_rotvec(err_axis_ned * 0.1) # 0.1 rad rotation
    new_nose = rot_err.apply(np.array([0.0, 0.0, -1.0]))
    print(f"New Nose in NED: {new_nose}")
    
    print("\nIf New Nose has a POSITIVE North component (X > 0), then it's pitching North!")

if __name__ == "__main__":
    run_test()
