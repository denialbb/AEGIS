"""
Comprehensive per-tick debug logger for AEGIS Mission Director.

Logs estimated state, raw sensors, desired wrench, per-engine throttles/gimbals,
FDI diagnostics, allocator health, and timing — all in one CSV per run.

Usage:
  KRPC_ADDRESS=172.xx.xx.xx .venv/bin/python scripts/debug_telemetry_detail.py

Waits at STANDBY — activate the action group (default AG-9) to start the mission.
Logs written to logs/debug_detail_<timestamp>.csv for post-flight review.
"""

import os
import sys
import csv
import time
import datetime
import numpy as np

sys.path.insert(0, os.path.abspath("."))

import krpc
import src.config as config
from src.main import MissionDirector

# ── helpers ──────────────────────────────────────────────────────────

def _ts() -> str:
    return datetime.datetime.now(datetime.UTC).strftime("%Y%m%d_%H%M%S")


def _maybe(val, default=0.0):
    """Return val or default if val is None/NaN."""
    if val is None:
        return default
    try:
        if np.isscalar(val):
            return float(val) if not np.isnan(val) else default
        return float(np.nan_to_num(val, nan=default))
    except (TypeError, ValueError):
        return default


def _vec(v, n=3):
    """Return length-n flattened list from array-like v."""
    arr = np.asarray(v, dtype=float).ravel()
    if len(arr) < n:
        arr = np.pad(arr, (0, n - len(arr)))
    return arr[:n].tolist()


# ── main ─────────────────────────────────────────────────────────────

def main():
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    conn = krpc.connect(name="AEGIS_DebugDetail", address=address)
    vessel = conn.space_center.active_vessel

    log_path = f"logs/debug_detail_{_ts()}.csv"
    os.makedirs("logs", exist_ok=True)
    f = open(log_path, "w", newline="")
    writer = csv.writer(f)
    header_written = False
    tick = [0]

    print(f"Connecting to KSP at {address}...")
    director = MissionDirector(conn)

    # ── 1. Patch the allocator to log allocation results ────────────
    _orig_alloc = director.allocator.allocate

    def _debug_alloc(desired_wrench, active_engines):
        nonlocal header_written, tick
        tick[0] += 1
        throttles, gimbals, forces_out = _orig_alloc(desired_wrench, active_engines)
        N = len(active_engines)

        # Gather stashed values (set by poll/guidance wrappers)
        sv = getattr(director, "_dbg_state_vector", np.zeros(6))
        ts = getattr(director, "_dbg_target_state", np.zeros(6))
        est_alt = float(np.dot(sv[:3], director.up_vector))
        est_vz = float(np.dot(sv[3:], director.up_vector))

        noisy_alt = getattr(director, "_dbg_noisy_alt", 0.0)
        noisy_vel = getattr(director, "_dbg_noisy_vel", np.zeros(3))
        sf_body = getattr(director, "_dbg_sf_body", np.zeros(3))
        omega_body = getattr(director, "_dbg_omega_body", np.zeros(3))
        gravity_world = getattr(director, "_dbg_gravity_world", np.zeros(3))
        attitude = getattr(director, "_dbg_attitude", np.array([0, 0, 0, 1]))
        aero_body = getattr(director, "_dbg_aero_body", np.zeros(3))
        mass = getattr(director, "_dbg_mass", 0.0)
        raw_gyro = getattr(director, "_dbg_raw_gyro", np.zeros(3))
        a_avail = getattr(director, "_dbg_a_avail", 1.0)

        skip_predict = getattr(director, "_dbg_skip_predict", False)
        actual_dt = getattr(director, "_dbg_actual_dt", 1.0 / config.TARGET_HZ)
        sleep_time = getattr(director, "_dbg_sleep_time", 0.0)
        landed_timer = getattr(director, "_landed_timer", 0.0)
        dt_spike_count = getattr(director, "_dt_spike_count", 0)

        # Engine state
        throttles_arr = np.zeros(max(len(director.engines), 1))
        gimbals_arr = np.zeros((max(len(director.engines), 1), 2))
        forces_arr = np.zeros((max(len(director.engines), 1), 3))
        positions_arr = np.zeros((max(len(director.engines), 1), 3))
        thrust_dirs_arr = np.zeros((max(len(director.engines), 1), 3))
        max_thrusts = np.zeros(max(len(director.engines), 1))
        active_flags = np.zeros(max(len(director.engines), 1), dtype=int)
        has_fuel_flags = np.zeros(max(len(director.engines), 1), dtype=int)
        expected_throttles = np.zeros(max(len(director.engines), 1))

        for i, eng in enumerate(director.engines):
            idx = eng.index
            positions_arr[idx] = eng.position
            thrust_dirs_arr[idx] = eng.thrust_direction
            max_thrusts[idx] = eng.max_thrust
            active_flags[idx] = 1 if eng.active else 0
            engine_obj = _safe_engine_access(eng.part)
            if engine_obj:
                has_fuel_flags[idx] = 1 if engine_obj.has_fuel else 0

        for i, eng in enumerate(active_engines):
            idx = eng.index
            throttles_arr[idx] = throttles[i]
            gimbals_arr[idx, 0] = gimbals[i, 0]
            gimbals_arr[idx, 1] = gimbals[i, 1]
            forces_arr[idx] = forces_out[i]

        # Expected throttles from EMA
        expected = getattr(director, "expected_throttles", None)
        if expected is not None and len(expected) > 0:
            for i, eng in enumerate(active_engines):
                if i < len(expected):
                    expected_throttles[eng.index] = expected[i]

        # FDI
        expected_accel = getattr(director, "expected_accel", np.zeros(3))
        alloc_cond = getattr(director, "_alloc_cond", 0.0)
        saturated_set = getattr(director, "_saturated_engines_set", set())

        # Building one row: tick, timing, state, estimated, raw sensors,
        # target, guidance, per-engine data, FDI, allocator

        row = [tick[0]]

        # ── timing ──────────────────────────────────────────────
        row += [actual_dt, sleep_time, skip_predict]

        # ── state ────────────────────────────────────────────────
        row += [director.state, dt_spike_count, landed_timer]

        # ── estimated state ─────────────────────────────────────
        row += _vec(sv, 6)        # state_vector: pos(3) + vel(3)
        row += _vec(attitude, 4)  # Mahony quaternion

        # ── raw sensors ─────────────────────────────────────────
        row += [noisy_alt]
        row += _vec(noisy_vel, 3)
        row += _vec(sf_body, 3)
        row += _vec(omega_body, 3)
        row += _vec(gravity_world, 3)
        row += _vec(raw_gyro, 3)
        row += [mass]
        row += _vec(aero_body, 3)

        # ── target state ─────────────────────────────────────────
        row += _vec(ts, 6)        # target_state: pos(3) + vel(3)

        # ── guidance ─────────────────────────────────────────────
        row += _vec(desired_wrench, 6)
        row += [a_avail, est_alt, est_vz]

        # ── per-engine (N engines × 9 values each) ──────────────
        # throttle, gimbal_x, gimbal_y, force_x, force_y, force_z,
        # max_thrust, active, has_fuel, expected_throttle,
        # pos_x, pos_y, pos_z, thrust_dir_x, thrust_dir_y, thrust_dir_z
        for i in range(len(director.engines)):
            row += [
                throttles_arr[i],
                gimbals_arr[i, 0],
                gimbals_arr[i, 1],
                forces_arr[i, 0],
                forces_arr[i, 1],
                forces_arr[i, 2],
            ]
        # ── engine static (once per engine) ──────────────────────
        for i in range(len(director.engines)):
            row += [
                max_thrusts[i],
                active_flags[i],
                has_fuel_flags[i],
                expected_throttles[i],
                positions_arr[i, 0],
                positions_arr[i, 1],
                positions_arr[i, 2],
                thrust_dirs_arr[i, 0],
                thrust_dirs_arr[i, 1],
                thrust_dirs_arr[i, 2],
            ]

        # ── FDI ──────────────────────────────────────────────────
        fdi_dev = (
            float(np.linalg.norm(expected_accel - sf_body))
            if np.linalg.norm(expected_accel) > 1e-6
            else 0.0
        )
        row += _vec(expected_accel, 3)
        row += [fdi_dev]
        row += [len(active_engines), len(director.engines)]

        # ── allocator ────────────────────────────────────────────
        row += [alloc_cond]
        row += [int(x) for x in sorted(saturated_set)]

        # Write header once
        if not header_written:
            _write_header(writer, len(director.engines))
            header_written = True

        writer.writerow(row)
        f.flush()

        return throttles, gimbals, forces_out

    director.allocator.allocate = _debug_alloc

    # ── 3. Patch sensors.poll to stash raw readings ────────────────
    def _debug_poll(*args, **kwargs):
        result = _orig_poll(*args, **kwargs)
        (
            director._dbg_noisy_alt,
            director._dbg_sf_body,
            director._dbg_attitude,
            director._dbg_mass,
            director._dbg_aero_body,
            _situation,  # not stored, captured from self.state
            director._dbg_omega_body,
            director._dbg_noisy_vel,
            director._dbg_gravity_world,
            director._dbg_raw_gyro,
        ) = result
        return result
    _orig_poll = director.sensors.poll
    director.sensors.poll = _debug_poll

    # ── 4. Patch compute_wrench to stash guidance inputs ───────────
    _orig_cw = director.guidance.compute_wrench
    def _debug_compute_wrench(current_state, current_attitude, mass,
                              target_state, up_vector, dt,
                              angular_velocity, max_a_avail):
        director._dbg_state_vector = np.asarray(current_state)
        director._dbg_target_state = np.asarray(target_state)
        director._dbg_a_avail = _maybe(max_a_avail, 1.0)
        return _orig_cw(current_state, current_attitude, mass,
                        target_state, up_vector, dt,
                        angular_velocity, max_a_avail)
    director.guidance.compute_wrench = _debug_compute_wrench

    # ── Run ────────────────────────────────────────────────────────
    print(f"Logging to {log_path}")
    print("Director ready. Activate the action group to start.")
    print("Press Ctrl+C to stop.")

    try:
        director.run_loop()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        f.close()
        conn.close()
        print(f"Done. {tick[0]} ticks logged to {log_path}")


def _safe_engine_access(part):
    if part is None:
        return None
    try:
        return part.engine
    except RuntimeError:
        return None


def _write_header(writer, N_engines):
    """Build the CSV header row dynamically based on N engines."""
    cols = []

    # timing
    cols += ["tick", "dt_actual", "sleep_time", "skip_predict"]

    # state
    cols += ["state", "dt_spike_count", "landed_timer"]

    # estimated state
    cols += ["est_px", "est_py", "est_pz", "est_vx", "est_vy", "est_vz"]
    cols += ["att_qx", "att_qy", "att_qz", "att_qw"]

    # raw sensors
    cols += ["noisy_alt"]
    cols += ["noisy_vx", "noisy_vy", "noisy_vz"]
    cols += ["sf_bx", "sf_by", "sf_bz"]
    cols += ["omega_bx", "omega_by", "omega_bz"]
    cols += ["grav_wx", "grav_wy", "grav_wz"]
    cols += ["raw_gyro_x", "raw_gyro_y", "raw_gyro_z"]
    cols += ["mass"]
    cols += ["aero_bx", "aero_by", "aero_bz"]

    # target state
    cols += ["tgt_px", "tgt_py", "tgt_pz", "tgt_vx", "tgt_vy", "tgt_vz"]

    # guidance
    cols += ["des_fx", "des_fy", "des_fz", "des_tx", "des_ty", "des_tz"]
    cols += ["a_avail", "est_alt", "est_vz"]

    # per-engine dynamic data (tick-varying)
    for i in range(N_engines):
        cols += [
            f"thr_{i}", f"gx_{i}", f"gy_{i}",
            f"fout_x{i}", f"fout_y{i}", f"fout_z{i}",
        ]

    # per-engine static data (constant or slow-varying)
    for i in range(N_engines):
        cols += [
            f"maxT_{i}", f"active_{i}", f"fuel_{i}",
            f"expThr_{i}",
            f"pos_x{i}", f"pos_y{i}", f"pos_z{i}",
            f"tdir_x{i}", f"tdir_y{i}", f"tdir_z{i}",
        ]

    # FDI
    cols += ["exp_acc_x", "exp_acc_y", "exp_acc_z", "fdi_deviation"]
    cols += ["active_count", "total_count"]

    # allocator health
    cols += ["alloc_cond"]  # saturated indices follow as variable-length
    for i in range(N_engines):
        cols += [f"sat_{i}"]

    writer.writerow(cols)


if __name__ == "__main__":
    main()
