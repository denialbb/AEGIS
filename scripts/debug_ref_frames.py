#!/usr/bin/env python3
"""Debug script to query KRPC reference frames from the quicksave."""

import os
import sys
import krpc

# Add src to path
sys.path.insert(0, '/mnt/c/Projects/AEGIS/src')
import config

def main():
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    save_name = sys.argv[1] if len(sys.argv) > 1 else "AEGIS_test"
    
    print(f"Connecting to KSP at {address}...")
    conn = krpc.connect(name="AEGIS_DebugFrames", address=address)
    print(f"Connected to KSP")
    
    if save_name:
        print(f"Loading save '{save_name}'...")
        conn.space_center.load(save_name)
        print("Waiting for vessel to load...")
        import time
        time.sleep(2.0)
        vessel = conn.space_center.active_vessel
        if vessel is None:
            print("ERROR: No active vessel after loading save.")
            conn.close()
            return
    else:
        vessel = conn.space_center.active_vessel
    
    print(f"Active vessel: {vessel.name}")
    print(f"Vessel situation: {vessel.situation}")
    
    body = vessel.orbit.body
    print(f"\nOrbiting body: {body.name}")
    print(f"Body reference frame: {body.reference_frame}")
    print(f"Body non-rotating reference frame: {body.non_rotating_reference_frame}")
    print(f"Vessel reference frame: {vessel.reference_frame}")
    print(f"Vessel orbital reference frame: {vessel.orbital_reference_frame}")
    print(f"Vessel surface reference frame: {vessel.surface_reference_frame}")
    
    target_lat = config.TARGET_LAT
    target_lon = config.TARGET_LON
    print(f"\nTarget lat/lon: {target_lat}, {target_lon}")
    
    # Surface position in body frame
    surf_pos_body = body.surface_position(target_lat, target_lon, body.reference_frame)
    surf_pos_body_nr = body.surface_position(target_lat, target_lon, body.non_rotating_reference_frame)
    print(f"Surface pos (body frame): {surf_pos_body}")
    print(f"Surface pos (non-rotating): {surf_pos_body_nr}")
    
    pad_pos = np.array(surf_pos_body)
    up_vector = pad_pos / np.linalg.norm(pad_pos)
    print(f"Up vector (from pad_pos): {up_vector}")
    
    # Create custom reference frame
    ref_frame = conn.space_center.ReferenceFrame.create_relative(
        body.reference_frame,
        position=surf_pos_body,
    )
    print(f"Custom ref frame created")
    
    # Query vessel state in different frames
    print(f"\n--- Vessel state in body reference frame ---")
    pos_body = vessel.position(body.reference_frame)
    vel_body = vessel.flight(body.reference_frame).velocity
    alt_body = vessel.flight(body.reference_frame).surface_altitude
    print(f"Position: {pos_body}")
    print(f"Velocity: {vel_body}")
    print(f"Surface altitude: {alt_body}")
    print(f"Altitude dot up: {np.dot(np.array(pos_body), up_vector)}")
    
    print(f"\n--- Vessel state in body non-rotating reference frame ---")
    pos_nr = vessel.position(body.non_rotating_reference_frame)
    vel_nr = vessel.flight(body.non_rotating_reference_frame).velocity
    alt_nr = vessel.flight(body.non_rotating_reference_frame).surface_altitude
    print(f"Position: {pos_nr}")
    print(f"Velocity: {vel_nr}")
    print(f"Surface altitude: {alt_nr}")
    
    print(f"\n--- Vessel state in custom ref frame ---")
    pos_custom = vessel.position(ref_frame)
    vel_custom = vessel.flight(ref_frame).velocity
    alt_custom = vessel.flight(ref_frame).surface_altitude
    print(f"Position: {pos_custom}")
    print(f"Velocity: {vel_custom}")
    print(f"Surface altitude: {alt_custom}")
    print(f"Altitude dot up: {np.dot(np.array(pos_custom), up_vector)}")
    
    print(f"\n--- Vessel state in vessel surface reference frame ---")
    pos_vs = vessel.position(vessel.surface_reference_frame)
    vel_vs = vessel.flight(vessel.surface_reference_frame).velocity
    alt_vs = vessel.flight(vessel.surface_reference_frame).surface_altitude
    print(f"Position: {pos_vs}")
    print(f"Velocity: {vel_vs}")
    print(f"Surface altitude: {alt_vs}")
    
    # Check g_force
    g_force = vessel.flight(body.reference_frame).g_force
    print(f"\nG-force (body frame): {g_force}")
    
    # Check gravitational parameter
    print(f"\nBody gravitational parameter (mu): {body.gravitational_parameter}")
    print(f"Body mass: {body.mass}")
    print(f"Body radius: {body.equatorial_radius}")
    
    # Calculate gravity at current position
    pos_vec = np.array(vessel.position(body.reference_frame))
    r = np.linalg.norm(pos_vec)
    gravity_vec = - (body.gravitational_parameter / r**3) * pos_vec
    print(f"Calculated gravity vector: {gravity_vec}")
    print(f"Gravity magnitude: {np.linalg.norm(gravity_vec)}")
    print(f"Gravity dot up: {np.dot(gravity_vec, up_vector)}")
    
    conn.close()

if __name__ == "__main__":
    import numpy as np
    main()
