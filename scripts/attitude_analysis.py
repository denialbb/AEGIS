#!/usr/bin/env python3
"""Evaluate flight attitude control, especially during terminal phases."""

import pandas as pd
import numpy as np
import sys
import math

def analyze(telemetry_path: str = "logs/latest/telemetry.csv") -> None:
    df = pd.read_csv(telemetry_path)

    if "est_alt" in df.columns:
        descent = df[df["est_alt"] < 100.0].copy()
    else:
        descent = df[df["altitude"] < 100.0].copy()
        
    if descent.empty:
        print("No data below 100m — check telemetry path")
        sys.exit(1)

    has_attitude = "true_pitch" in df.columns
    
    print("=== Attitude Control Evaluation (below 100m) ===")
    if has_attitude:
        print(f"{'tick':>6} {'alt':>7} {'est_vz':>7} | {'True(P,Y,R)':>22} | {'Est(P,Y,R)':>22} | {'cmd_tilt':>9}")
    else:
        print(f"{'tick':>6} {'alt':>7} {'est_vz':>7} | {'cmd_tilt(deg)':>14} | {'gimbal_max(deg)':>15} | {'accel_x':>7} {'accel_y':>7}")
    print("-" * 90)
    
    max_tilt = 0.0
    max_gimbal = 0.0
    
    for i, (_, r) in enumerate(descent.iterrows()):
        # Commanded tilt from fb (force body)
        fb_x, fb_y, fb_z = r.get("fb_x", 0), r.get("fb_y", 0), r.get("fb_z", 0)
        fb_horiz = math.sqrt(fb_x**2 + fb_z**2)
        cmd_tilt_deg = math.degrees(math.atan2(fb_horiz, abs(fb_y))) if abs(fb_y) > 1e-3 else 0.0
        
        max_tilt = max(max_tilt, cmd_tilt_deg)
        
        # Max gimbal deflection
        g_max = 0.0
        for col in descent.columns:
            if col.startswith("gimbal_"):
                g_max = max(g_max, abs(r[col]))
                
        gimbal_deg = g_max * 4.5  
        max_gimbal = max(max_gimbal, gimbal_deg)
        
        if i % 10 == 0:
            alt = r.get('est_alt', r.get('altitude'))
            vz = r.get('vel_z', 0)
            
            if has_attitude:
                tp, ty, tr = r.get("true_pitch", 0), r.get("true_yaw", 0), r.get("true_roll", 0)
                ep, ey, er = r.get("est_pitch", 0), r.get("est_yaw", 0), r.get("est_roll", 0)
                print(f"{i:6.0f} {alt:7.1f} {vz:7.1f} | {tp:6.1f} {ty:6.1f} {tr:6.1f} | {ep:6.1f} {ey:6.1f} {er:6.1f} | {cmd_tilt_deg:9.1f}")
            else:
                ax = r.get('accel_x', 0)
                ay = r.get('accel_y', 0)
                print(f"{i:6.0f} {alt:7.1f} {vz:7.1f} | {cmd_tilt_deg:14.1f} | {gimbal_deg:15.2f} | {ax:7.2f} {ay:7.2f}")

    print("\n--- Summary ---")
    print(f"Max Commanded Tilt:    {max_tilt:.1f} degrees")
    print(f"Max Gimbal Deflection: {max_gimbal:.1f} degrees (estimated)")
    if max_tilt > 25.0:
        print("WARNING: Commanded tilt was extremely high (>25 deg), likely causing instability!")
    if max_gimbal >= 4.4:
        print("WARNING: Gimbals were saturated (max deflection reached)!")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "logs/latest/telemetry.csv"
    analyze(path)
