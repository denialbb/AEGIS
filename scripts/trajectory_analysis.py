#!/usr/bin/env python3
"""Trace position/velocity evolution during HOVER phase to diagnose drift."""

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

    print("=== HOVER phase trajectory (pos_n, pos_e) ===")
    print(f"{'tick':>6} {'alt':>7} {'pos_n':>8} {'pos_e':>8} {'→pad_n':>8} {'→pad_e':>8} {'dist':>7} {'vel_n':>7} {'vel_e':>7}")
    print("-" * 75)
    for _, r in hover.iterrows():
        if int(r.get("ticks", 0)) % 15 == 0:
            print(f"{r.get('ticks', -1):6.0f} {r['altitude']:7.1f} {r['pos_n']:8.2f} {r['pos_e']:8.2f} "
                  f"{r['to_pad_n']:8.2f} {r['to_pad_e']:8.2f} {r['dist']:7.2f} "
                  f"{r['vel_x']:7.2f} {r['vel_y']:7.2f}")

    print(f"\nStart: ({hover['pos_n'].iloc[0]:.1f}, {hover['pos_e'].iloc[0]:.1f})  dist={hover['dist'].iloc[0]:.1f}m")
    print(f"End:   ({hover['pos_n'].iloc[-1]:.1f}, {hover['pos_e'].iloc[-1]:.1f})  dist={hover['dist'].iloc[-1]:.1f}m")
    dn = hover["pos_n"].iloc[-1] - hover["pos_n"].iloc[0]
    de = hover["pos_e"].iloc[-1] - hover["pos_e"].iloc[0]
    trend = "IMPROVING" if hover["dist"].iloc[-1] < hover["dist"].iloc[0] else "WORSENING"
    print(f"Drift: dN={dn:.1f} {'N' if dn>0 else 'S'}, dE={de:.1f} {'E' if de>0 else 'W'}  |  {trend}")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "logs/latest/telemetry.csv"
    analyze(path)