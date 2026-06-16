"""
Flight recorder that uses kRPC Action Group 1 (configured in the game) to start and stop capturing telemetry.
The data are stored in a single ``.npz`` file, with arrays that are convenient for offline replay.
"""

import krpc
import numpy as np
import os
import datetime
import time
import logging

from src.telemetry.sensors import SensorModels
import src.config as config

# Directory that will contain recorded .npz files
RECORD_DIR = "recordings"
os.makedirs(RECORD_DIR, exist_ok=True)

# Action group used as a start/stop toggle for recording
RECORD_GROUP = 1


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
    )
    conn = krpc.connect(
        name=config.KRPC_CLIENT_NAME, address=config.KRPC_DEFAULT_ADDRESS
    )
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    target_lat = config.TARGET_LAT
    target_lon = config.TARGET_LON
    # Reference frame centered at the pad location
    ref_frame = conn.space_center.ReferenceFrame.create_relative(
        body.reference_frame,
        position=body.surface_position(
            target_lat, target_lon, body.reference_frame
        ),
    )
    pin = np.array(
        body.surface_position(target_lat, target_lon, body.reference_frame)
    )
    up_vector = pin / np.linalg.norm(pin)

    # Initialise sensor models (the same used by the main director)
    sensors = SensorModels(conn, vessel, ref_frame, up_vector)

    running = False
    file_path: str | None = None
    data = {
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
        "gravity_world": [],
    }
    last_ut: float | None = None

    while True:
        group_on = vessel.control.get_action_group(RECORD_GROUP)
        if group_on and not running:
            # Start a new recording
            timestamp = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
            file_path = os.path.join(RECORD_DIR, f"flight_{timestamp}.npz")
            for key in data:
                data[key] = []
            last_ut = None
            running = True
            logging.info(f"Recording started: {file_path}")

        elif not group_on and running:
            # Stop recording and write file
            if file_path:
                # Convert lists to numpy arrays
                for k, v in data.items():
                    data[k] = np.array(v)
                np.savez(file_path, **data)
                logging.info(f"Recording finished and saved: {file_path}")
            running = False
            file_path = None

        if not running:
            time.sleep(0.1)
            continue

        # Ground‑truth samples
        flight = vessel.flight(ref_frame)
        ut = conn.space_center.ut
        dt = 0.0 if last_ut is None else ut - last_ut
        last_ut = ut

        gt_pos = (
            np.array(flight.position)
            if hasattr(flight, "position")
            else np.array(vessel.position(ref_frame))
        )
        gt_vel = np.array(flight.velocity)
        gt_att = sensors._read_krpc_quaternion()

        # Poll sensor data
        (
            noisy_alt,
            sf_body_noisy,
            mahony_attitude,
            _mass,
            _aero_body,
            _situation,
            omega_body,
            noisy_vel,
            gravity_world,
            raw_gyro,
        ) = sensors.poll()

        # Store telemetry
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
        data["gravity_world"].append(gravity_world.tolist())

        # Loop continuously; the check for group toggling is at the top of the loop.


if __name__ == "__main__":
    main()
