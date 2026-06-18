#!/usr/bin/env python3
"""Score a landing run from its telemetry CSV and log to tuning_log.csv.

Usage:
    .venv/bin/python scripts/score_landing.py logs/runs/<run_dir>/telemetry.csv

Score components (lower is better):
  - vz:       |vertical speed| at last tick (m/s)
  - vh:       sqrt(vx²+vy²) at last tick (m/s)
  - dist:     sqrt(pos_n²+pos_e²) at last tick (m)
  - fuel:     total integrated throttle across engines (arb. units)
  - time:     chrono duration (s)
  - TOTAL:    vz + vh + dist*5 + fuel/50 + time/100

Logs results + config params to scripts/tuning_log.csv for reference.
"""

import sys, math, csv, os, re
from pathlib import Path

LOG_PATH = Path(__file__).resolve().parent / "tuning_log.csv"
CONF_DIR  = Path(__file__).resolve().parent.parent / "src" / "config"

def _read_conf(key: str, default: str = "") -> str:
    """Read a value from a .conf file by key name."""
    for f in sorted(CONF_DIR.glob("*.conf")):
        text = f.read_text()
        m = re.search(rf"^{key}\s*=\s*(.+?)(?:\s*#.*)?$", text, re.MULTILINE)
        if m:
            return m.group(1).strip().strip('"').strip("'")
    return default


def score_file(csv_path: str) -> None:
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        print("No data.")
        return
    cols = list(rows[0].keys())
    num_eng = sum(1 for c in cols if c.startswith("throttle_"))
    has_pos = "pos_n" in cols

    run_dir = os.path.basename(os.path.dirname(csv_path))

    last = rows[-1]
    vx = float(last.get("vel_x", 0))
    vy = float(last.get("vel_y", 0))
    vz = float(last.get("vel_z", 0))
    alt = float(last.get("est_alt", last.get("altitude", 0)))
    vh = math.hypot(vx, vy)

    if has_pos:
        pn = float(last.get("pos_n", 0))
        pe = float(last.get("pos_e", 0))
        dist = math.hypot(pn, pe)
    else:
        dist = 0.0

    total_thr = 0.0
    for r in rows:
        for e in range(num_eng):
            v = float(r.get(f"throttle_{e}", 0))
            if v > 0.01:
                total_thr += v

    dur = float(rows[-1]["timestamp"]) - float(rows[0]["timestamp"])

    vz_p = abs(vz)
    vh_p = vh
    di_p = dist * 5.0
    fu_p = total_thr / 50.0
    ti_p = dur / 100.0
    total = vz_p + vh_p + di_p + fu_p + ti_p

    # ── Read current config params ──
    params = {
        "run_dir": run_dir,
        "vz": f"{vz:.1f}",
        "vh": f"{vh:.1f}",
        "dist": f"{dist:.1f}",
        "fuel": f"{total_thr:.0f}",
        "time": f"{dur:.0f}",
        "total_score": f"{total:.1f}",
        "ALT_POWERED_DESCENT": _read_conf("ALT_POWERED_DESCENT"),
        "ALT_HOVER": _read_conf("ALT_HOVER"),
        "ALT_TERMINAL": _read_conf("ALT_TERMINAL"),
        "RATE_PD": _read_conf("GLIDESLOPE_RATE_POWERED_DESCENT"),
        "RATE_HOVER": _read_conf("GLIDESLOPE_RATE_HOVER"),
        "RATE_TERMINAL": _read_conf("GLIDESLOPE_RATE_TERMINAL"),
        "TARGET_RATIO": _read_conf("GLIDESLOPE_TARGET_RATIO"),
        "PD_KP": _read_conf("PD_KP_POS_LATERAL"),
        "PD_KD": _read_conf("PD_KD_VEL_LATERAL"),
        "HOVER_KP": _read_conf("HOVER_KP_POS_LATERAL"),
        "HOVER_KD": _read_conf("HOVER_KD_VEL_LATERAL"),
        "HOVER_APP_K": _read_conf("HOVER_APPROACH_K"),
        "HOVER_APP_MAX": _read_conf("HOVER_APPROACH_MAX"),
        "TERM_KP": _read_conf("TERMINAL_KP_POS_LATERAL"),
        "TERM_KD": _read_conf("TERMINAL_KD_VEL_LATERAL"),
        "TERM_APP_K": _read_conf("TERMINAL_APPROACH_K"),
        "TERM_APP_MAX": _read_conf("TERMINAL_APPROACH_MAX"),
        "NOTES": "",
    }

    # ── Print results ──
    print(f"Run:       {os.path.dirname(csv_path)}")
    print(f"Rows:      {len(rows)}  Engines: {num_eng}")
    print(f"--- Last tick ---")
    print(f"vz (m/s):  {vz:.1f}")
    print(f"vh (m/s):  {vh:.1f}   (vx={vx:.1f}, vy={vy:.1f})")
    print(f"dist (m):  {dist:.1f}")
    print(f"alt (m):   {alt:.1f}")
    print(f"--- Resources ---")
    print(f"∫throttle: {total_thr:.0f}")
    print(f"duration:  {dur:.0f}s")
    print(f"--- Score ({os.path.basename(csv_path)}) ---")
    print(f"vz……{vz_p:.1f}   vh……{vh_p:.1f}   dist…{di_p:.1f}   fuel…{fu_p:.1f}   time…{ti_p:.1f}")
    print(f"TOTAL: {total:.1f}")

    # ── Append / update CSV log ──
    fieldnames = [
        "run_dir", "vz", "vh", "dist", "fuel", "time", "total_score",
        "ALT_POWERED_DESCENT", "ALT_HOVER", "ALT_TERMINAL",
        "RATE_PD", "RATE_HOVER", "RATE_TERMINAL", "TARGET_RATIO",
        "PD_KP", "PD_KD", "HOVER_KP", "HOVER_KD",
        "HOVER_APP_K", "HOVER_APP_MAX",
        "TERM_KP", "TERM_KD", "TERM_APP_K", "TERM_APP_MAX", "NOTES",
    ]

    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    rows_existing = []
    try:
        with open(LOG_PATH, newline="") as f:
            reader = csv.DictReader(f)
            for r in reader:
                row_clean = {k: v for k, v in r.items() if k is not None}
                rows_existing.append(row_clean)
    except FileNotFoundError:
        pass

    updated = False
    for r in rows_existing:
        if r.get("run_dir") == run_dir:
            r.update(params)
            updated = True
            break
    if not updated:
        rows_existing.append(params)

    with open(LOG_PATH, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows_existing)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: score_landing.py <telemetry.csv>")
        sys.exit(1)
    score_file(sys.argv[1])
