import numpy as np
from src.estimation.estimator import StateEstimator


def test_estimator_initialization_with_vel_noise():
    """Ensure the estimator accepts alt and vel noise matrices."""
    init_state = np.zeros(6)
    P0 = np.eye(6)
    base_Q = np.eye(6) * 0.01
    R_alt = np.eye(1) * 0.1
    R_vel = np.eye(3) * 0.5

    est = StateEstimator(init_state, P0, base_Q, R_alt, R_vel)

    assert est.kf.R.shape == (4, 4)
    np.testing.assert_allclose(est.kf.R[0, 0], 0.1)
    np.testing.assert_allclose(est.kf.R[1, 1], 0.5)
    np.testing.assert_allclose(est.kf.R[2, 2], 0.5)
    np.testing.assert_allclose(est.kf.R[3, 3], 0.5)


def test_predict_uses_base_Q():
    """Verify that predict() sets kf.Q based on the base Q matrix after scaling."""
    init_state = np.zeros(6)
    P0 = np.eye(6)
    base_Q = np.eye(6) * 0.01
    R_alt = np.eye(1) * 0.1
    R_vel = np.eye(3) * 0.5

    est = StateEstimator(init_state, P0, base_Q, R_alt, R_vel)
    dt = 0.1
    noisy_accel = np.array([0.0, 0.0, 0.0])
    identity_att = np.array([0.0, 0.0, 0.0, 1.0])
    gravity_world = np.array([0.0, 0.0, 0.0])
    est.predict(noisy_accel, identity_att, dt, gravity_world)

    # Q should be different from base_Q due to adaptive scaling
    assert np.any(est.kf.Q != base_Q)


def test_update_fuses_alt_and_vel():
    """Verify that update() fuses both alt and vel measurements."""
    init_state = np.zeros(6)
    init_state[2] = 100.0
    init_state[5] = -10.0
    P0 = np.eye(6)
    base_Q = np.eye(6) * 0.01
    R_alt = np.eye(1) * 0.1
    R_vel = np.eye(3) * 0.1

    est = StateEstimator(init_state, P0, base_Q, R_alt, R_vel)
    dt = 0.1
    noisy_accel = np.array([0.0, 0.0, 9.81])
    identity_att = np.array([0.0, 0.0, 0.0, 1.0])
    gravity_world = np.array([0.0, 0.0, -9.81])

    est.predict(noisy_accel, identity_att, dt, gravity_world)

    noisy_alt = 95.0
    noisy_vel = np.array([1.0, -0.5, -12.0])
    updated_state = est.update(noisy_alt, noisy_vel)

    assert updated_state.shape == (6,)
    # Altitude should be corrected towards measurement
    assert abs(updated_state[2] - 95.0) < 5.0
    # Velocity should also be corrected
    assert abs(updated_state[5] - (-12.0)) < 5.0