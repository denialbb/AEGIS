#!/usr/bin/env python3
"""
Replay simulation to calibrate FDI noise threshold (ISS-001).
Uses telemetry from a nominal run to compute residuals between
measured acceleration and expected acceleration from engine thrust.
Outputs suggested threshold based on residual standard deviation.
"""

import numpy as np
import os
import sys

def moving_average(x, w):
    """Return moving average of x with window w using cumulative sum."""
    if len(x) < w:
        return np.zeros_like(x)
    cumsum = np.cumsum(np.insert(x, 0, 0))
    ma = (cumsum[w:] - cumsum[:-w]) / float(w)
    # Pad to original length: prepend first value, append last value
    ma_padded = np.concatenate(([ma[0]], ma, [ma[-1]]))
    # Ensure same length
    if len(ma_padded) > len(x):
        ma_padded = ma_padded[:len(x)]
    elif len(ma_padded) < len(x):
        ma_padded = np.pad(ma_padded, (0, len(x)-len(ma_padded)), 'edge')
    return ma_padded

def load_telemetry(run_dir):
    """Load telemetry CSV from run directory."""
    telemetry_path = os.path.join(run_dir, 'telemetry.csv')
    if not os.path.exists(telemetry_path):
        raise FileNotFoundError(f"Telemetry file not found: {telemetry_path}")
    # Use numpy genfromtxt to load CSV with header
    data = np.genfromtxt(telemetry_path, delimiter=',', skip_header=1, filling_values=np.nan)
    # Get header to know column indices
    with open(telemetry_path, 'r') as f:
        header = f.readline().strip()
    col_names = header.split(',')
    return data, col_names

def compute_expected_accel(data, col_names):
    """
    Compute expected acceleration in body frame from engine thrust.
    Simplification: all engines thrust in +y direction (as per src/main.py).
    Throttle values are 0-1; if has_fuel == 0, throttle set to 0.
    Returns expected acceleration vector (ax, ay, az) for each timestamp.
    We compute a scale factor to best match measured acceleration in y direction.
    """
    # Build column index maps
    col_index = {name: i for i, name in enumerate(col_names)}
    
    # Extract throttle and fuel columns
    throttle_cols = [f'throttle_{i}' for i in range(5)]
    fuel_cols = [f'has_fuel_{i}' for i in range(5)]
    
    # Get columns
    throttle = np.column_stack([data[:, col_index[c]] for c in throttle_cols])
    fuel = np.column_stack([data[:, col_index[c]] for c in fuel_cols])
    
    # Effective throttle: zero if no fuel
    effective_throttle = throttle * fuel  # elementwise
    
    # Total thrust in y direction (normalized to N per engine unit, assuming max_thrust=1 N per engine)
    total_thrust_y = np.sum(effective_throttle, axis=1)  # shape (N,)
    
    # We will compute a scale factor such that expected_accel_y = scale * total_thrust_y
    # This scale factor effectively incorporates (real_max_thrust_per_engine / mass)
    # Determine scale using least squares on y axis (minimize squared error)
    # For simplicity, use ratio of means (assuming zero mean error)
    # Extract measured ay
    measured_ay = data[:, col_names.index('accel_y')]
    
    # Avoid division by zero
    if np.mean(total_thrust_y) == 0:
        scale = 0.0
    else:
        scale = np.mean(measured_ay) / np.mean(total_thrust_y)
    
    expected_accel = np.zeros((data.shape[0], 3))
    expected_accel[:, 1] = scale * total_thrust_y  # ay
    
    return expected_accel, scale

def compute_residuals(data, col_names, expected_accel):
    """Compute residual = measured - expected acceleration."""
    # Extract accel columns
    accel_cols = ['accel_x', 'accel_y', 'accel_z']
    col_index = {name: i for i, name in enumerate(col_names)}
    measured_accel = np.column_stack([data[:, col_index[c]] for c in accel_cols])
    
    residual = measured_accel - expected_accel
    return residual

def main():
    # Use the latest run with data (we'll pick a known good run)
    logs_base = 'logs/runs'
    # Find all run directories
    run_dirs = [os.path.join(logs_base, d) for d in os.listdir(logs_base)
                if os.path.isdir(os.path.join(logs_base, d))]
    # Sort by name (timestamp) descending
    run_dirs.sort(reverse=True)
    
    # Find first run with non-empty telemetry
    selected_run = None
    for rd in run_dirs:
        telemetry_path = os.path.join(rd, 'telemetry.csv')
        if os.path.exists(telemetry_path) and os.path.getsize(telemetry_path) > 0:
            selected_run = rd
            break
    if selected_run is None:
        print("Error: No run with telemetry data found.")
        sys.exit(1)
    
    print(f"Using telemetry from: {selected_run}")
    
    data, col_names = load_telemetry(selected_run)
    print(f"Loaded {data.shape[0]} telemetry records.")
    
    # Compute expected acceleration with scale factor
    expected_accel, scale = compute_expected_accel(data, col_names)
    print(f"Derived scale factor (throttle sum -> accel_y): {scale:.6f} m/s^2 per unit throttle")
    
    # Compute residuals
    residuals = compute_residuals(data, col_names, expected_accel)
    
    # Optionally skip initial transient (first 5 seconds)
    t_idx = col_names.index('timestamp')
    timestamps = data[:, t_idx]
    if len(timestamps) > 0:
        t0 = timestamps[0]
        time_sec = timestamps - t0
        # Skip first 5 seconds
        mask = time_sec >= 5.0
        if np.sum(mask) > 0:
            residuals_used = residuals[mask]
            print(f"After skipping first 5 sec, using {np.sum(mask)} samples.")
        else:
            residuals_used = residuals
            print("Warning: less than 5 seconds of data, using all.")
    else:
        residuals_used = residuals
    
    # Detrend residuals: remove low-frequency components using moving average
    # Window of 1 second (50 samples at 50Hz)
    window = 50
    if residuals_used.shape[0] > window:
        # Compute moving average per axis
        smoothed = np.zeros_like(residuals_used)
        for i in range(3):
            smoothed[:, i] = moving_average(residuals_used[:, i], window)
        residuals_detrended = residuals_used - smoothed
    else:
        residuals_detrended = residuals_used
        print("Warning: not enough data for detrending, using raw residuals.")
    
    # Compute statistics on detrended residuals
    mean_res = np.mean(residuals_detrended, axis=0)
    std_res = np.std(residuals_detrended, axis=0, ddof=1)
    
    # Compute residual magnitude (Euclidean norm) per sample
    res_mag = np.linalg.norm(residuals_detrended, axis=1)
    mean_mag = np.mean(res_mag)
    std_mag = np.std(res_mag, ddof=1)
    
    print("\nDetrended residual statistics (measured - expected):")
    print(f"  Mean (ax, ay, az): {mean_res[0]:.4f}, {mean_res[1]:.4f}, {mean_res[2]:.4f} m/s^2")
    print(f"  Std  (ax, ay, az): {std_res[0]:.4f}, {std_res[1]:.4f}, {std_res[2]:.4f} m/s^2")
    print(f"  Mean magnitude: {mean_mag:.4f} m/s^2")
    print(f"  Std magnitude:  {std_mag:.4f} m/s^2")
    
    # Suggest threshold based on 3-sigma of magnitude
    suggested_threshold = 3.0 * std_mag
    print(f"\nSuggested FDI threshold (3-sigma of residual magnitude): {suggested_threshold:.4f} m/s^2")
    print(f"  (Based on derived scale factor = {scale:.6f} and 1-sec detrending)")
    
    # Also output per-axis thresholds if desired
    print(f"\nPer-axis 3-sigma thresholds:")
    print(f"  ax: {3.0 * std_res[0]:.4f} m/s^2")
    print(f"  ay: {3.0 * std_res[1]:.4f} m/s^2")
    print(f"  az: {3.0 * std_res[2]:.4f} m/s^2")
    
    # Write suggestion to a file for easy retrieval
    out_file = 'fdi_threshold_suggestion.txt'
    with open(out_file, 'w') as f:
        f.write(f"# FDI threshold suggestion from replay simulation\n")
        f.write(f"# Run: {selected_run}\n")
        f.write(f"# Samples used: {len(residuals_detrended)}\n")
        f.write(f"# Derived scale factor: {scale:.6f}\n")
        f.write(f"# Detrending window: {window} samples ({window/50.0} sec)\n")
        f.write(f"suggested_threshold = {suggested_threshold:.6f}\n")
    print(f"\nWritten suggestion to {out_file}")

if __name__ == '__main__':
    main()
