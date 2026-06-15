import numpy as np
from src.estimation.estimator import StateEstimator
import src.config as config


def test_adaptive_process_noise_scaling():
    """Ensure the estimator scales the velocity‑noise block of Q with thrust.

    The test creates a StateEstimator with a known base Q matrix, runs a predict
    step with a high‑magnitude acceleration, and checks that the filter's Q
    has been increased according to config.PROCESS_NOISE_THRUST_COEF.
    """
    init_state = np.zeros(6)
    P0 = np.eye(6)
    base_Q = np.eye(6) * 0.01
    R0 = np.eye(1) * 0.1

    est = StateEstimator(init_state, P0, base_Q, R0)
    # Record the original velocity‑noise sub‑matrix (rows/cols 3:6)
    original_block = base_Q[3:6, 3:6].copy()

    # High proper acceleration in body frame (positive Z)
    noisy_accel_body = np.array([0.0, 0.0, 20.0])
    # No rotation quaternion (x, y, z, w)
    attitude = np.array([0.0, 0.0, 0.0, 1.0])
    dt = 0.1

    est.predict(noisy_accel_body, attitude, dt)

    scaled_block = est.kf.Q[3:6, 3:6]
    # The scaled block should be larger than the original (strictly greater)
    assert np.all(scaled_block > original_block)
    # Verify the scaling factor is roughly as expected (within 10% tolerance)
    accel_norm = np.linalg.norm(
        # kinematic acceleration = proper + gravity (gravity = -9.81 in Z)
        noisy_accel_body + np.array([0.0, 0.0, -9.81])
    )
    expected_scale = 1.0 + config.PROCESS_NOISE_THRUST_COEF * (accel_norm ** 2)
    # Compare average of diagonal entries
    actual_scale = scaled_block[0, 0] / original_block[0, 0]
    assert np.isclose(actual_scale, expected_scale, rtol=0.1)
