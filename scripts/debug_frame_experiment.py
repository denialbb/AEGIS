#!/usr/bin/env python3
"""
Debug script to test EKF initialization in three different reference frames:
1. BODY_FRAME - Kerbin's body-fixed reference frame (rotating)
2. WORLD_FRAME - Kerbin's non-rotating reference frame (inertial)
3. REFERENCE_FRAME - Custom pad-relative frame (current implementation)

This script logs ground truth, sensor data, and EKF state for each frame
to diagnose the coordinate frame mismatch issue.
"""

import os
import sys
import time
import csv
import math
import numpy as np

sys.path.insert(0, '/mnt/c/Projects/AEGIS/src')
import krpc
import config

# ============================================================================
# Data structures
# ============================================================================

class FrameData:
    """Holds state data for one reference frame."""
    def __init__(self, name):
        self.name = name
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.alt = 0.0
        self.gravity = np.zeros(3)
        self.specific_force_ned = np.zeros(3)
        self.specific_force_body = np.zeros(3)
        self.attitude = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion [x,y,z,w]
        self.omega_body = np.zeros(3)
        self.mass = 0.0
        self.aero_body = np.zeros(3)
        
        # EKF state (if running)
        self.ekf_pos = np.zeros(3)
        self.ekf_vel = np.zeros(3)
        self.ekf_alt = 0.0  # dot(ekf_pos, up_vector)
        self.ekf_bg = np.zeros(3)  # gyro bias
        self.ekf_ba = np.zeros(3)  # accel bias

def get_body_radius(body, up_vector):
    """Compute body radius at the pad location."""
    pad_pos = np.array(body.surface_position(config.TARGET_LAT, config.TARGET_LON, body.reference_frame))
    return np.linalg.norm(pad_pos)

def compute_gravity_body_frame(vessel, body):
    """Compute gravity vector in body.reference_frame coordinates."""
    mu = body.gravitational_parameter
    pos_body = np.array(vessel.position(body.reference_frame))
    r = np.linalg.norm(pos_body)
    if r > 0:
        return - (mu / r**3) * pos_body
    return np.zeros(3)

def compute_gravity_nonrotating_frame(vessel, body):
    """Compute gravity vector in body.non_rotating_reference_frame coordinates."""
    mu = body.gravitational_parameter
    pos_nr = np.array(vessel.position(body.non_rotating_reference_frame))
    r = np.linalg.norm(pos_nr)
    if r > 0:
        return - (mu / r**3) * pos_nr
    return np.zeros(3)

def rotation_matrix_from_quat(q):
    """Convert quaternion [x,y,z,w] to rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*z*w,   2*x*z+2*y*w],
        [2*x*y+2*z*w,   1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w,   2*y*z+2*x*w,   1-2*x*x-2*y*y]
    ])

def rotate_vector(v, q, inverse=False):
    """Rotate vector v by quaternion q."""
    rot = rotation_matrix_from_quat(q)
    if inverse:
        rot = rot.T
    return rot @ v

# ============================================================================
# Simple EKF implementation (mirrors src/estimation/ekf.py)
# ============================================================================

class SimpleEKF:
    """Minimal EKF for debugging - mirrors the real EKF but with logging."""
    
    def __init__(self, initial_pos, initial_vel, up_vector, frame_name):
        self.pos = initial_pos.copy()
        self.vel = initial_vel.copy()
        self.bg = np.zeros(3)  # gyro bias
        self.ba = np.zeros(3)  # accel bias
        self.up_vector = up_vector / np.linalg.norm(up_vector)
        self.frame_name = frame_name
        
    def predict(self, f_body, omega_body, attitude, gravity_ned, dt):
        """Predict step - propagate state using IMU data."""
        if dt <= 0:
            return
        
        # Correct IMU with bias estimates
        omega_corr = omega_body - self.bg
        f_corr_body = f_body - self.ba
        
        # Rotate specific force to NED frame
        rot = rotation_matrix_from_quat(attitude)
        f_corr_ned = rot @ f_corr_body
        
        # Add gravity (NED frame)
        a_ned = f_corr_ned + gravity_ned
        
        # Propagate state
        self.pos = self.pos + self.vel * dt + 0.5 * a_ned * dt**2
        self.vel = self.vel + a_ned * dt
        
    def update(self, noisy_alt, noisy_vel):
        """Update step - simple measurement fusion (no full covariance for debugging)."""
        # Compute estimated altitude
        est_alt = np.dot(self.up_vector, self.pos)
        
        # Simple correction (gain-based, not full Kalman)
        alt_error = noisy_alt - est_alt
        vel_error = noisy_vel - self.vel
        
        # Apply corrections (simplified - real EKF uses Kalman gain)
        gain_pos = 0.1  # Position gain
        gain_vel = 0.1  # Velocity gain
        
        self.pos = self.pos + self.up_vector * alt_error * gain_pos
        self.vel = self.vel + vel_error * gain_vel
        
        return np.concatenate([self.pos, self.vel])
    
    def get_state(self):
        """Return 6-element state vector."""
        return np.concatenate([self.pos, self.vel])
    
    def get_altitude(self):
        """Compute altitude from position."""
        return np.dot(self.up_vector, self.pos)

# ============================================================================
# Main experiment
# ============================================================================

def main():
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    save_name = sys.argv[1] if len(sys.argv) > 1 else "quicksave"
    
    print(f"Connecting to KSP at {address}...")
    conn = krpc.connect(name="AEGIS_FrameExperiment", address=address)
    print(f"Connected.")
    
    print(f"Loading save '{save_name}'...")
    conn.space_center.load(save_name)
    print("Waiting for vessel to load...")
    time.sleep(2.0)
    
    vessel = conn.space_center.active_vessel
    if vessel is None:
        print("ERROR: No active vessel after loading save.")
        conn.close()
        return
    
    print(f"Active vessel: {vessel.name}")
    print(f"Vessel situation: {vessel.situation}")
    
    body = vessel.orbit.body
    print(f"Orbiting body: {body.name}")
    
    # Compute up_vector (local vertical at pad - always points away from center)
    pad_pos_body = np.array(body.surface_position(config.TARGET_LAT, config.TARGET_LON, body.reference_frame))
    # Ensure up_vector points away from center (radially outward)
    up_vector = pad_pos_body / np.linalg.norm(pad_pos_body)
    # Flip direction if pointing toward center (shouldn't happen, but just in case)
    if np.dot(pad_pos_body, up_vector) < 0:
        up_vector = -up_vector
    body_radius = np.linalg.norm(pad_pos_body)
    
    print(f"Target lat/lon: {config.TARGET_LAT}, {config.TARGET_LON}")
    print(f"Pad position (body frame): {pad_pos_body}")
    print(f"Up vector (body frame): {up_vector}")
    print(f"Body radius at pad: {body_radius:.1f} m")
    
    # Create reference frames
    body_frame = body.reference_frame
    nonrotating_frame = body.non_rotating_reference_frame
    custom_frame = conn.space_center.ReferenceFrame.create_relative(
        body_frame,
        position=pad_pos_body
    )
    
    # Store frame info for logging
    frame_info = {
        "BODY": body_frame,
        "NONROTATING": nonrotating_frame,
        "CUSTOM": custom_frame
    }
    
    # Create streams for each frame
    streams = {}
    for name, frame in frame_info.items():
        flight_data = vessel.flight(frame)
        streams[name] = {
            'pos': conn.add_stream(vessel.position, frame),
            'vel': conn.add_stream(vessel.velocity, frame),
            'alt': conn.add_stream(getattr, flight_data, 'surface_altitude'),
        }
    
    # Common streams
    ut_stream = conn.add_stream(getattr, conn.space_center, 'ut')
    mass_stream = conn.add_stream(getattr, vessel, 'mass')
    # Use flight.rotation for attitude (quaternion) - use body_frame flight
    flight_for_attitude = vessel.flight(body_frame)
    attitude_stream = conn.add_stream(getattr, flight_for_attitude, 'rotation')
    # Angular velocity in body frame - must pass reference_frame
    angular_velocity_stream = conn.add_stream(vessel.angular_velocity, body_frame)
    aero_stream = conn.add_stream(getattr, flight_for_attitude, 'aerodynamic_force')
    
    # Wait for streams to stabilize
    time.sleep(0.5)
    
    # Initialize three EKFs, one per frame
    ekfs = {}
    
    # Get initial data for each frame
    print("\n=== INITIAL STATE (t=0) ===")
    for frame_name in ["BODY", "NONROTATING", "CUSTOM"]:
        pos = np.array(streams[frame_name]['pos']())
        vel = np.array(streams[frame_name]['vel']())
        alt = float(streams[frame_name]['alt']())
        
        print(f"\n{frame_name}_FRAME:")
        print(f"  Position: {pos}")
        print(f"  Velocity: {vel}")
        print(f"  Altitude: {alt:.2f} m")
        print(f"  |Position|: {np.linalg.norm(pos):.1f} m")
        print(f"  dot(pos, up_vector): {np.dot(pos, up_vector):.1f} m")
        
        # Compute gravity in this frame
        if frame_name == "BODY":
            gravity = compute_gravity_body_frame(vessel, body)
        elif frame_name == "NONROTATING":
            gravity = compute_gravity_nonrotating_frame(vessel, body)
        else:  # CUSTOM
            gravity = compute_gravity_body_frame(vessel, body)  # Same as body frame (translated)
        
        print(f"  Gravity: {gravity}")
        print(f"  |Gravity|: {np.linalg.norm(gravity):.3f} m/s²")
        
        # Initialize EKF
        # Current implementation: pos = up_vector * alt (pad-relative)
        # Alternative: pos = actual position in frame
        if frame_name == "CUSTOM":
            # Current implementation uses pad-relative position
            ekf_initial_pos = up_vector * alt
        else:
            # For body/nonrotating, use actual position
            ekf_initial_pos = pos.copy()
        
        ekfs[frame_name] = SimpleEKF(ekf_initial_pos, vel.copy(), up_vector, frame_name)
        print(f"  EKF initial pos: {ekf_initial_pos}")
        print(f"  EKF initial alt: {np.dot(ekf_initial_pos, up_vector):.2f} m")
    
    # Setup CSV logging
    log_path = f"logs/frame_experiment_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    os.makedirs("logs", exist_ok=True)
    
    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Header
        header = ['tick', 'ut', 'frame', 
                  'gt_px', 'gt_py', 'gt_pz', 'gt_vx', 'gt_vy', 'gt_vz', 'gt_alt',
                  'ekf_px', 'ekf_py', 'ekf_pz', 'ekf_vx', 'ekf_vy', 'ekf_vz', 'ekf_alt',
                  'ekf_alt_err', 'ekf_vel_err',
                  'gravity_x', 'gravity_y', 'gravity_z',
                  'sf_body_x', 'sf_body_y', 'sf_body_z',
                  'omega_x', 'omega_y', 'omega_z',
                  'att_qx', 'att_qy', 'att_qz', 'att_qw']
        writer.writerow(header)
        
        print(f"\nLogging to {log_path}")
        print("Running for 5 seconds (250 ticks at 50Hz)...")
        print("-" * 80)
        
        # Main loop
        tick = 0
        last_ut = ut_stream()
        last_vel_body = None
        start_time = time.time()
        
        while time.time() - start_time < 5.0:
            current_ut = ut_stream()
            dt = current_ut - last_ut
            if dt <= 0 or dt > 0.1:  # Skip invalid dt
                time.sleep(0.01)
                continue
            
            tick += 1
            
            # Get common data
            mass = mass_stream()
            attitude_q = np.array(attitude_stream())  # [x,y,z,w]
            omega_body = np.array(angular_velocity_stream())
            aero_body = np.array(aero_stream())
            
            # Compute specific force in body frame
            # Use body frame velocity for acceleration computation
            vel_body = np.array(streams['BODY']['vel']())
            if last_vel_body is None:
                accel_body = np.zeros(3)
            else:
                # Acceleration in body frame (approximate, ignores rotation)
                accel_body = (vel_body - last_vel_body) / dt
            last_vel_body = vel_body
            
            # Gravity in body frame
            gravity_body = compute_gravity_body_frame(vessel, body)
            
            # Specific force = accel - gravity (in body frame)
            # But accel is in body frame, gravity is in body frame
            # Need to rotate gravity to body frame using attitude
            # For now, assume gravity is already aligned (small error for short runs)
            sf_body = accel_body - gravity_body
            
            # Process each frame
            for frame_name in ["BODY", "NONROTATING", "CUSTOM"]:
                pos = np.array(streams[frame_name]['pos']())
                vel = np.array(streams[frame_name]['vel']())
                alt = float(streams[frame_name]['alt']())
                
                # Get gravity in appropriate frame
                if frame_name == "BODY":
                    gravity = gravity_body
                elif frame_name == "NONROTATING":
                    gravity = compute_gravity_nonrotating_frame(vessel, body)
                else:
                    gravity = gravity_body  # Custom frame has same axes as body
                
                # Get EKF
                ekf = ekfs[frame_name]
                
                # EKF predict (use body-frame IMU data for all EKFs)
                ekf.predict(sf_body, omega_body, attitude_q, gravity_body, dt)
                
                # EKF update (use frame-specific measurements)
                # For altitude, use the frame's altitude
                # For velocity, use the frame's velocity
                noisy_alt = alt + np.random.normal(0, config.SIGMA_ALT)
                noisy_vel = vel + np.random.normal(0, config.SIGMA_VEL, 3)
                
                ekf.update(noisy_alt, noisy_vel)
                
                # Get EKF state
                ekf_state = ekf.get_state()
                ekf_pos = ekf_state[:3]
                ekf_vel = ekf_state[3:]
                ekf_alt = ekf.get_altitude()
                ekf_bg = ekf.bg  # gyro bias
                ekf_ba = ekf.ba  # accel bias
                
                # Compute errors
                ekf_alt_err = ekf_alt - alt
                ekf_vel_err = np.linalg.norm(ekf_vel - vel)
                
                # Write to CSV
                row = [tick, current_ut, frame_name,
                       pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], alt,
                       ekf_pos[0], ekf_pos[1], ekf_pos[2], ekf_vel[0], ekf_vel[1], ekf_vel[2], ekf_alt,
                       ekf_alt_err, ekf_vel_err,
                       gravity[0], gravity[1], gravity[2],
                       sf_body[0], sf_body[1], sf_body[2],
                       omega_body[0], omega_body[1], omega_body[2],
                       attitude_q[0], attitude_q[1], attitude_q[2], attitude_q[3]]
                writer.writerow(row)
                
                # Log every 10 ticks
                if tick % 10 == 0:
                    print(f"{frame_name}: tick={tick}, alt={alt:.1f}, ekf_alt={ekf_alt:.1f}, err={ekf_alt_err:.1f}")
            
            # Update last_ut
            last_ut = current_ut
            
            # Sleep to maintain 50Hz
            time.sleep(max(0, 0.02 - (time.time() - start_time)))
    
    print("\nExperiment complete. Results saved to:")
    print(f"  {log_path}")
    
    conn.close()

if __name__ == "__main__":
    main()