#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np

def diagnose_logs(log_dir: str = "logs/latest"):
    """Diagnose AEGIS telemetry logs for flight trajectory and attitude."""
    telemetry_path = os.path.join(log_dir, "telemetry.csv")
    if not os.path.exists(telemetry_path):
        print(f"Error: Could not find {telemetry_path}")
        return

    print(f"Reading logs from: {telemetry_path}\n")
    df = pd.read_csv(telemetry_path)
    
    # Analyze by state
    print("=== Phase Summaries ===")
    states = df['state'].unique()
    for state in states:
        state_df = df[df['state'] == state]
        print(f"\nState: {state} (Duration: {len(state_df)*0.02:.1f}s)")
        
        # Trajectory
        if 'pos_n' in state_df.columns and 'pos_e' in state_df.columns:
            start_n = state_df['pos_n'].iloc[0]
            start_e = state_df['pos_e'].iloc[0]
            end_n = state_df['pos_n'].iloc[-1]
            end_e = state_df['pos_e'].iloc[-1]
            print(f"  Trajectory (N, E): Start=({start_n:5.1f}, {start_e:5.1f}) -> End=({end_n:5.1f}, {end_e:5.1f})")
        
        if 'est_vz' in state_df.columns:
            vz_min, vz_max = state_df['est_vz'].min(), state_df['est_vz'].max()
            print(f"  Vertical Velocity: Min={vz_min:5.1f} m/s, Max={vz_max:5.1f} m/s")

        if 'ves_orientation' in state_df.columns:
            modes = state_df['ves_orientation'].unique()
            print(f"  SAS Modes Active: {', '.join(modes)}")

    print("\n=== Landing / Final Status ===")
    final_row = df.iloc[-1]
    print(f"End State: {final_row['state']}")
    
    if 'pos_n' in final_row and 'pos_e' in final_row:
        dist = np.sqrt(final_row['pos_n']**2 + final_row['pos_e']**2)
        print(f"Final Distance from Pad: {dist:.1f} meters")
        
    if 'vel_n' in final_row and 'vel_e' in final_row and 'est_vz' in final_row:
        h_vel = np.sqrt(final_row['vel_n']**2 + final_row['vel_e']**2)
        print(f"Final Horizontal Velocity: {h_vel:.1f} m/s")
        print(f"Final Vertical Velocity:   {final_row['est_vz']:.1f} m/s")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Diagnose AEGIS flight logs.")
    parser.add_argument("--log_dir", default="logs/latest", help="Directory containing telemetry.csv")
    args = parser.parse_args()
    
    diagnose_logs(args.log_dir)
