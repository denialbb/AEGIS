#!/usr/bin/env python3
import krpc
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

def run_test():
    print("Connecting to kRPC...")
    conn = krpc.connect(name="Attitude Test", address="172.22.80.1", rpc_port=50000, stream_port=50001)
    
    print("Loading save 'AEGIS MK2'...")
    conn.space_center.load("AEGIS MK2")
    time.sleep(1) # Let physics settle
    
    vessel = conn.space_center.active_vessel
    vessel.control.sas = True
    vessel.control.rcs = False
    
    # Construct NED frame just like main.py
    body = vessel.orbit.body
    pad_ecef = vessel.position(body.reference_frame)
    
    # We must import geometry to match AEGIS behavior
    import sys
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from src.common.geometry import ecef_to_ned
    
    R_ecef_to_ned, ned_quat, north_ecef, east_ecef = ecef_to_ned(np.array(pad_ecef))
    
    ned_frame = conn.space_center.ReferenceFrame.create_relative(
        body.reference_frame,
        rotation=tuple(ned_quat) # [x,y,z,w] -> KRPC requires this?
    )
    
    print("Vessel stable. Gathering attitude data in NED frame...")
    flight = vessel.flight(ned_frame)
    
    # KSP Euler Angles
    pitch = flight.pitch
    heading = flight.heading
    roll = flight.roll
    
    print(f"\n--- KSP Native Euler Angles ---")
    print(f"Pitch:   {pitch:.2f} deg (nose up/down)")
    print(f"Heading: {heading:.2f} deg (compass dir)")
    print(f"Roll:    {roll:.2f} deg")
    
    # KSP Quaternion
    ksp_quat = flight.rotation
    print(f"\n--- KSP Native Quaternion (flight.rotation) ---")
    print(f"kRPC Raw: {ksp_quat} (w, x, y, z or x, y, z, w?)")
    
    # AEGIS sensor_models.py interpretation
    # It assumes flight.rotation returns [x, y, z, w] or similar. 
    # Actually, kRPC docs say it returns (x, y, z, w).
    
    # Let's try what ui.py does for true_euler:
    q_krpc = np.array(ksp_quat)
    true_euler = R.from_quat(q_krpc).inv().as_euler("YXZ", degrees=True)
    print(f"\n--- AEGIS true_euler via UI.py ---")
    print(f"q used: {q_krpc}")
    print(f"true_euler (Pitch, Yaw, Roll): {true_euler[0]:.2f}, {true_euler[1]:.2f}, {true_euler[2]:.2f}")
    
    # Let's see what from_euler("YXZ", [heading, pitch, roll]).inv() gives
    ksp_euler_arr = [heading, pitch, roll]
    q_from_euler = R.from_euler("YXZ", ksp_euler_arr, degrees=True).inv().as_quat()
    print(f"\n--- AEGIS sensors.py fallback (from_euler YXZ .inv) ---")
    print(f"q_from_euler: {q_from_euler}")
    
    # Compare them
    diff = np.linalg.norm(q_krpc - q_from_euler)
    print(f"Diff between kRPC quat and fallback quat: {diff:.4f}")
    
    # What does Mahony output if we feed it q_krpc?
    # Mahony estimator is in src.estimation.mahony_estimator
    try:
        import sys
        import os
        sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
        from src.estimation.mahony_estimator import MahonyAttitudeEstimator
        mahony = MahonyAttitudeEstimator(kp=3.14, ki=0.06)
        mahony.quaternion = q_krpc.copy()
        est_euler = R.from_quat(mahony.quaternion).inv().as_euler("YXZ", degrees=True)
        print(f"\n--- AEGIS Mahony Output (using ui.py logic) ---")
        print(f"est_euler (Pitch, Yaw, Roll): {est_euler[0]:.2f}, {est_euler[1]:.2f}, {est_euler[2]:.2f}")
    except Exception as e:
        print(f"Failed to run Mahony test: {e}")
        
    print("\nTest completed.")

if __name__ == "__main__":
    run_test()
