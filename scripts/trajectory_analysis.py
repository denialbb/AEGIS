#!/usr/bin/env python3
"""Trace position/velocity evolution during HOVER phase to diagnose drift.

Usage:
    .venv/bin/python scripts/trajectory_analysis.py [telemetry.csv]

If the CSV has ``target_vel_x/y``, ``target_vel_y``, and ``torque_x/y/z``
columns (added by the diagnostic logging), the script also prints the
guidance's target horizontal velocity and the body-frame torque command
each row — these directly expose the velocity-target sign-flip pattern
and any RW saturation driving the close-vicinity oscillation.
"""

import pandas as pd
import numpy as np
import sys

def analyze(telemetry_path: str = "logs/latest/telemetry.csv") -> None:
    df = pd.read_csv(telemetry_path)

    if "est_alt" in df.columns:
        hover = df[df["est_alt"] < 100.0].copy()
    else:
        hover = df[df["altitude"] < 100.0].copy()
    if hover.empty:
        print("No data below 100m — check telemetry path")
        sys.exit(1)

    # Direction to pad is simply -pos since pad is at origin
    hover["to_pad_n"] = -hover["pos_n"]
    hover["to_pad_e"] = -hover["pos_e"]
    hover["dist"] = np.sqrt(hover["pos_n"] ** 2 + hover["pos_e"] ** 2)
    hover["vh"] = np.sqrt(hover["vel_x"] ** 2 + hover["vel_y"] ** 2)

    has_target = "target_vel_x" in hover.columns and "target_vel_y" in hover.columns
    has_torque = "torque_x" in hover.columns

    print("=== HOVER phase trajectory (pos_n, pos_e) ===")
    if has_target and has_torque:
        print(f"{'tick':>6} {'alt':>7} {'dist':>6} {'vh':>5} {'tgt_vh_x':>9} {'tgt_vh_y':>9} {'|torque|':>9}")
    else:
        print(f"{'tick':>6} {'alt':>7} {'pos_n':>8} {'pos_e':>8} {'→pad_n':>8} {'→pad_e':>8} {'dist':>7} {'vel_n':>7} {'vel_e':>7}")
    print("-" * 75)
    for _, r in hover.iterrows():
        if int(r.get("ticks", 0)) % 15 != 0:
            continue
        if has_target and has_torque:
            tvh_x = float(r.get("target_vel_x", 0.0))
            tvh_y = float(r.get("target_vel_y", 0.0))
            tq = np.sqrt(
                float(r.get("torque_x", 0.0)) ** 2
                + float(r.get("torque_y", 0.0)) ** 2
                + float(r.get("torque_z", 0.0)) ** 2
            )
            print(f"{r.get('ticks', -1):6.0f} {r['altitude']:7.1f} {r['dist']:6.1f} {r['vh']:5.1f} "
                  f"{tvh_x:9.2f} {tvh_y:9.2f} {tq:9.0f}")
        else:
            print(f"{r.get('ticks', -1):6.0f} {r['altitude']:7.1f} {r['pos_n']:8.2f} {r['pos_e']:8.2f} "
                  f"{r['to_pad_n']:8.2f} {r['to_pad_e']:8.2f} {r['dist']:7.2f} "
                  f"{r['vel_x']:7.2f} {r['vel_y']:7.2f}")

    print(f"\nStart: ({hover['pos_n'].iloc[0]:.1f}, {hover['pos_e'].iloc[0]:.1f})  dist={hover['dist'].iloc[0]:.1f}m")
    print(f"End:   ({hover['pos_n'].iloc[-1]:.1f}, {hover['pos_e'].iloc[-1]:.1f})  dist={hover['dist'].iloc[-1]:.1f}m")
    dn = hover["pos_n"].iloc[-1] - hover["pos_n"].iloc[0]
    de = hover["pos_e"].iloc[-1] - hover["pos_e"].iloc[0]
    trend = "IMPROVING" if hover["dist"].iloc[-1] < hover["dist"].iloc[0] else "WORSENING"
    print(f"Drift: dN={dn:.1f} {'N' if dn>0 else 'S'}, dE={de:.1f} {'E' if de>0 else 'W'}  |  {trend}")

    if has_target:
        # Count target_vel sign reversals on the dominant axis (proxy for hunting)
        tvh = np.sign(hover["target_vel_x"].fillna(0.0)) + 1j * np.sign(hover["target_vel_y"].fillna(0.0))
        flips = int(((tvh[1:] - tvh[:-1]) != 0).sum())
        print(f"Target-velocity quadrant flips during HOVER: {flips}")
        if flips > 0:
            print(f"  (Each flip = guidance commanded reversal. Many flips = hunting.)")

    if has_torque:
        torques = np.sqrt(
            hover["torque_x"].fillna(0.0) ** 2
            + hover["torque_y"].fillna(0.0) ** 2
            + hover["torque_z"].fillna(0.0) ** 2
        )
        print(f"Torque |.| max: {torques.max():.0f} Nm  (vs GUIDANCE_MAX_TORQUE={50000.0} Nm)")
        if torques.max() > 0.95 * 50000.0:
            print("  WARNING: torque saturating — RW authority may be insufficient")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "logs/latest/telemetry.csv"
    analyze(path)