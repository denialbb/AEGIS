"""Debug script: logs throttle/gimbal/state per cycle to diagnose control issues.

Usage: KRPC_ADDRESS=172.xx.xx.xx .venv/bin/python scripts/debug_control.py
The script waits at STANDBY — activate the action group to start.
Logs are written to logs/debug_control.csv for post-flight review.
"""

import os, sys, csv, time
sys.path.insert(0, os.path.abspath('.'))

import krpc
import numpy as np
import src.config as config
from src.main import MissionDirector

address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
conn = krpc.connect(name="AEGIS_Debug", address=address)
vessel = conn.space_center.active_vessel
body = vessel.orbit.body

# --- monkey-patch MissionDirector.run_loop to inject debug logging ---
_original_run_loop = MissionDirector.run_loop

def _debug_run_loop(self):
    """Wrap run_loop with per-cycle CSV logging."""
    log_path = "logs/debug_control.csv"
    os.makedirs("logs", exist_ok=True)
    f = open(log_path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow([
        "tick", "state", "est_alt", "est_vz",
        "des_fx", "des_fy", "des_fz", "des_tx", "des_ty", "des_tz",
        "throttle_0", "throttle_1", "throttle_2", "throttle_3",
        "gimbal_x0", "gimbal_y0",
        "a_avail", "alloc_cond",
        "active_count", "total_count",
        "skip_predict", "sleep_time",
        "vessel_ctrl_throttle",
    ])

    # We need to access loop-local vars.  We'll patch into the while loop
    # by instrumenting the allocation call site.
    _orig_alloc = self.allocator.allocate
    tick = [0]

    def _debug_alloc(desired_wrench, active_engines):
        tick[0] += 1
        throttles, gimbals, forces = _orig_alloc(desired_wrench, active_engines)
        # Log every 5th tick to avoid massive files
        if tick[0] % 5 == 0:
            try:
                writer.writerow([
                    tick[0],
                    self.state,
                    getattr(self, '_debug_est_alt', 0),
                    getattr(self, '_debug_est_vz', 0),
                    *desired_wrench,
                    *([*throttles, *([0.0] * (4 - len(throttles)))][:4]),
                    gimbals[0, 0] if len(gimbals) > 0 else 0,
                    gimbals[0, 1] if len(gimbals) > 0 else 0,
                    getattr(self, '_debug_a_avail', 0),
                    getattr(self, '_debug_alloc_cond', 0),
                    len(active_engines),
                    len(self.engines),
                    getattr(self, '_debug_skip_predict', False),
                    getattr(self, '_debug_sleep_time', 0),
                    self.vessel.control.throttle,
                ])
                f.flush()
            except Exception as e:
                print(f"Log write error: {e}")
        return throttles, gimbals, forces

    self.allocator.allocate = _debug_alloc

    # Patch into main loop's dt check to expose loop-local vars
    _orig_loop_body = None  # We'll use __dict__ injection instead

    try:
        result = _original_run_loop(self)
    finally:
        f.close()
    return result

MissionDirector.run_loop = _debug_run_loop

# --- Also inject loop-local vars as attrs for the debug alloc wrapper ---
# We override a small part of main.py's loop to stash values.
# Simpler: just run and let the alloc wrapper capture what it can.

print(f"Connecting to KSP at {address}...")
director = MissionDirector(conn)
print("Director ready. Activate the action group to start the mission.")
print("Logging to logs/debug_control.csv")
print("Press Ctrl+C to stop.")

try:
    director.run_loop()
except KeyboardInterrupt:
    print("\nInterrupted.")
except Exception as e:
    print(f"Error: {e}")
finally:
    conn.close()
    print("Done. Review logs/debug_control.csv")
