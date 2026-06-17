"""
Flight recorder that uses kRPC Action Group 1 (configured in the game) to start and stop capturing telemetry.
The data are stored in a single ``.npz`` file, with arrays that are convenient for offline replay.

Survives vessel destruction and re-launch: if the vessel is lost during recording the current
file is saved and the script waits for a new active vessel before continuing.
"""

import krpc
import numpy as np
import os
import datetime
import time
import logging
import signal
import sys
from typing import Any

from src.common.geometry import ecef_to_ned

from src.telemetry.sensors import SensorModels
import src.config as config

from krpc.error import RPCError

# Directory that will contain recorded .npz files
RECORD_DIR = "recordings"
os.makedirs(RECORD_DIR, exist_ok=True)

# Action group used as a start/stop toggle for recording
RECORD_GROUP = 1

# Seconds between checks for a new active vessel after the current one is lost
VESSEL_POLL_INTERVAL = 0.5

# ── Graceful exit state (written by signal handler) ──────────────────
_exit_requested: bool = False


def _handle_exit(signum: int, frame: Any) -> None:
    """Set the exit flag — the main loop will save & quit on its next iteration."""
    global _exit_requested
    if _exit_requested:
        sys.exit(1)  # second Ctrl+C → force quit
    logging.getLogger(__name__).info(
        "Shutdown requested — saving recording and exiting …"
    )
    _exit_requested = True


# ── helpers ──────────────────────────────────────────────────────────


def _timestamp() -> str:
    """UTC timestamp string for filenames (no colons, no deprecation warnings)."""
    return datetime.datetime.now(datetime.UTC).strftime("%Y%m%d_%H%M%S")


def _wait_for_active_vessel(conn: Any) -> Any:
    """Block until kRPC reports an active vessel, then return it."""
    while True:
        v = conn.space_center.active_vessel
        if v is not None:
            return v
        time.sleep(VESSEL_POLL_INTERVAL)


def _init_sensors(
    conn: Any, vessel: Any
) -> tuple[Any, Any, np.ndarray, SensorModels]:
    """Build a true NED reference frame and SensorModels for *vessel*."""
    body = vessel.orbit.body
    target_lat = config.TARGET_LAT
    target_lon = config.TARGET_LON

    # ── Pad position in ECEF ──────────────────────────────────────────
    pad_ecef = np.array(
        body.surface_position(target_lat, target_lon, body.reference_frame),
        dtype=float,
    )

    _R, ned_quat, _north, _east = ecef_to_ned(pad_ecef)

    ned_frame = conn.space_center.ReferenceFrame.create_relative(
        body.reference_frame,
        position=tuple(float(v) for v in pad_ecef),
        rotation=tuple(float(v) for v in ned_quat),
    )
    up_vector: np.ndarray = np.array([0.0, 0.0, -1.0])

    sensors = SensorModels(conn, vessel, ned_frame, up_vector)
    return ned_frame, up_vector, sensors


def _save_recording(file_path: str, data: dict) -> None:
    """Convert lists to arrays and write the ``.npz`` file."""
    for k, v in data.items():
        data[k] = np.array(v)
    np.savez(file_path, **data)
    logging.info(f"Recording saved: {file_path}")


# ── main ─────────────────────────────────────────────────────────────


def main() -> None:
    global _exit_requested

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
    )
    signal.signal(signal.SIGINT, _handle_exit)
    signal.signal(signal.SIGTERM, _handle_exit)

    conn = krpc.connect(
        name=config.KRPC_CLIENT_NAME, address=config.KRPC_DEFAULT_ADDRESS
    )

    # Initial vessel acquisition (blocks until a vessel exists).
    vessel = _wait_for_active_vessel(conn)
    ned_frame, up_vector, sensors = _init_sensors(conn, vessel)
    logger = logging.getLogger(__name__)
    logger.info(f"Connected, tracking vessel {vessel.name}")

    running = False
    file_path: str | None = None
    data: dict = {
        "ut": [],
        "dt": [],
        "gt_pos": [],
        "gt_vel": [],
        "gt_att": [],
        "raw_gyro": [],
        "sf_body_noisy": [],
        "mahony_attitude": [],
        "noisy_alt": [],
        "noisy_vel": [],
        "gravity_ned": [],
        # -- new fields --
        "mass": [],
        "aero_body": [],
        "situation": [],
        "clean_alt": [],
        "clean_angular_vel": [],
        "throttle": [],
        "active_engine_count": [],
    }
    last_ut: float | None = None

    while True:
        # ── Graceful exit check ──────────────────────────────────────
        if _exit_requested:
            if running and file_path:
                _save_recording(file_path, data)
            logger.info("Exiting")
            break

        # ── Check Action Group toggle ────────────────────────────────
        try:
            group_on = vessel.control.get_action_group(RECORD_GROUP)
        except RPCError:
            # Vessel reference is stale (destroyed / reverted).  Save any
            # in‑progress recording, then wait for a new vessel.
            logger.warning("Vessel lost while reading action group")
            if running and file_path:
                _save_recording(file_path, data)
            running = False
            file_path = None

            vessel = _wait_for_active_vessel(conn)
            ned_frame, up_vector, sensors = _init_sensors(conn, vessel)
            logger.info(f"Re‑acquired vessel {vessel.name}")
            time.sleep(VESSEL_POLL_INTERVAL)
            continue

        if group_on and not running:
            # ── Start recording ──────────────────────────────────────
            ts = _timestamp()
            file_path = os.path.join(RECORD_DIR, f"flight_{ts}.npz")
            for key in data:
                data[key] = []
            last_ut = None
            running = True
            logger.info(f"Recording started: {file_path}")

        elif not group_on and running:
            # ── Stop recording and save ──────────────────────────────
            if file_path:
                _save_recording(file_path, data)
            running = False
            file_path = None

        if not running:
            time.sleep(0.1)
            continue

        # ── Record a single telemetry frame ──────────────────────────
        try:
            flight = vessel.flight(ned_frame)
            ut = conn.space_center.ut
            dt = 0.0 if last_ut is None else ut - last_ut
            last_ut = ut

            gt_pos = (
                np.array(flight.position)
                if hasattr(flight, "position")
                else np.array(vessel.position(ned_frame))
            )
            gt_vel = np.array(flight.velocity)
            gt_att = sensors._read_krpc_quaternion()

            (
                noisy_alt,
                sf_body_noisy,
                mahony_attitude,
                mass,
                aero_body,
                situation,
                omega_body,
                noisy_vel,
                gravity_ned,
                raw_gyro,
            ) = sensors.poll()

        except RPCError:
            logger.warning("Vessel lost during telemetry poll")
            if file_path:
                _save_recording(file_path, data)
            running = False
            file_path = None

            vessel = _wait_for_active_vessel(conn)
            ned_frame, up_vector, sensors = _init_sensors(conn, vessel)
            logger.info(f"Re‑acquired vessel {vessel.name}")
            time.sleep(VESSEL_POLL_INTERVAL)
            continue

        # ── Clean sensor values (direct kRPC reads, no noise) ──────
        clean_alt = float(flight.surface_altitude)
        av_raw = sensors.gyro_sensor.angular_velocity_stream()
        if hasattr(av_raw, "x"):
            clean_angular_vel = np.array([av_raw.x, av_raw.y, av_raw.z])
        else:
            clean_angular_vel = np.array(av_raw, dtype=float)

        # ── Engine status ───────────────────────────────────────────
        throttle = float(vessel.control.throttle)
        active_engine_count = 0
        try:
            tagged_parts = vessel.parts.with_tag("AegisEngine")
            for p in tagged_parts:
                try:
                    if p.engine is not None and p.engine.active:
                        active_engine_count += 1
                except RPCError:
                    continue
        except RPCError:
            pass

        # ── Store ────────────────────────────────────────────────────
        data["ut"].append(ut)
        data["dt"].append(dt)
        data["gt_pos"].append(gt_pos.tolist())
        data["gt_vel"].append(gt_vel.tolist())
        data["gt_att"].append(gt_att.tolist())
        data["raw_gyro"].append(raw_gyro.tolist())
        data["sf_body_noisy"].append(sf_body_noisy.tolist())
        data["mahony_attitude"].append(mahony_attitude.tolist())
        data["noisy_alt"].append(noisy_alt)
        data["noisy_vel"].append(noisy_vel.tolist())
        data["gravity_ned"].append(gravity_ned.tolist())
        data["mass"].append(mass)
        data["aero_body"].append(aero_body.tolist())
        data["situation"].append(situation)
        data["clean_alt"].append(clean_alt)
        data["clean_angular_vel"].append(clean_angular_vel.tolist())
        data["throttle"].append(throttle)
        data["active_engine_count"].append(active_engine_count)


if __name__ == "__main__":
    main()
