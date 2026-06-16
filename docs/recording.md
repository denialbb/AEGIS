# Flight Recording Workflow

This document describes how to capture a KSP flight session for offline replay.

## Start recording
1. Press the **Action Group 1** button in the game, or issue
   ``vessel.control.set_action_group(1, True)`` via kRPC.
2. The script `scripts/flight_recorder.py` will automatically start writing data to a new
   ``.npz`` file inside the `recordings/` directory.

## Stop recording
1. Toggle Action Group 1 **OFF** (or issue
   ``vessel.control.set_action_group(1, False)``).  The script writes out the
   file and exits the recording state.

## File format
The ``.npz`` file contains the following numpy arrays:

| Key | Shape | Description |
|-----|-------|------------|
| `ut` | (N,) | Universal time of each tick |
| `dt` | (N,) | Inter‑tick duration |
| `gt_pos` | (N,3) | Ground‑truth position in the custom reference frame |
| `gt_vel` | (N,3) | Ground‑truth velocity |
| `gt_att` | (N,4) | Ground‑truth quaternion (x,y,z,w) |
| `raw_gyro` | (N,3) | Raw gyroscope measurement (rad/s) – noise added but bias not corrected |
| `sf_body_noisy` | (N,3) | Noisy body‑frame specific force from the accelerometer |
| `mahony_attitude` | (N,4) | Mahony complementary‑filter attitude estimate |
| `noisy_alt` | (N,) | Altimeter reading |
| `noisy_vel` | (N,3) | Velocity measurement |
| `gravity_world` | (N,3) | Gravitational acceleration vector |

The file can be loaded with `np.load()` and fed into the
``FlightReplayer`` class for automatic replay.

## Offline replay
``FlightReplayer`` (see :mod:`src.simulation.flights`) summarizes a
recording and evaluates an ``ErrorStateEKF`` against the ground truth.  It
returns RMSE for position and velocity and the mean Normalised Innovation
Squared, which can be combined into a single score for optimisation.

---

Use the sample script `scripts/tune_estimator_optuna.py` to optimise the
Kalman filter hyper‑parameters with Optuna.  The script is configured to run
with parallel workers (`n_jobs=-1`), maximising the use of available CPU cores.
"""