import numpy as np
from src.estimation.estimator import StateEstimator


def test_estimator_initialization():
    initial_state = np.zeros(6)
    initial_covariance = np.eye(6)
    process_noise = np.eye(6) * 0.1
    measurement_noise_alt = np.eye(1) * 0.5
    measurement_noise_vel = np.eye(3) * 0.1

    estimator = StateEstimator(
        initial_state,
        initial_covariance,
        process_noise,
        measurement_noise_alt,
        measurement_noise_vel,
    )
    state = estimator.get_state()
    assert state.shape == (6,)
    np.testing.assert_array_equal(state, initial_state)


def test_estimator_update():
    initial_state = np.zeros(6)
    initial_covariance = np.eye(6)
    process_noise = np.eye(6) * 0.01
    measurement_noise_alt = np.eye(1) * 0.1
    measurement_noise_vel = np.eye(3) * 0.1

    estimator = StateEstimator(
        initial_state,
        initial_covariance,
        process_noise,
        measurement_noise_alt,
        measurement_noise_vel,
    )

    noisy_alt = 10.0
    noisy_vel = np.array([0.0, 0.0, 0.0])
    noisy_accel = np.array([0.0, 0.0, 9.81])
    dummy_attitude = np.array([0.0, 0.0, 0.0, 1.0])
    dt = 0.1
    gravity_world = np.array([0.0, 0.0, -9.81])

    estimator.predict(noisy_accel, dummy_attitude, dt, gravity_world)

    # Before update, the state should not have jumped to 10m
    pre_update_state = estimator.get_state()
    assert abs(pre_update_state[2]) < 1.0

    updated_state = estimator.update(noisy_alt, noisy_vel)

    assert updated_state.shape == (6,)

    assert updated_state[2] > 0.0
    assert updated_state[2] < 10.0

    # With zero kinematic acceleration (proper=[0,0,9.81], gravity=[0,0,-9.81]),
    # the innovation for acceleration is 0, so velocity correction comes
    # only from the altimeter cross-coupling.
    assert abs(updated_state[5]) < 5.0


def test_estimator_synthetic_fall():
    initial_state = np.zeros(6)
    initial_state[2] = 100.0
    initial_state[5] = -10.0
    initial_covariance = np.eye(6)
    process_noise = np.eye(6) * 0.01
    measurement_noise_alt = np.eye(1) * 0.1
    measurement_noise_vel = np.eye(3) * 0.1

    estimator = StateEstimator(
        initial_state,
        initial_covariance,
        process_noise,
        measurement_noise_alt,
        measurement_noise_vel,
    )

    dt = 0.1
    steps = 50
    identity_att = np.array([0.0, 0.0, 0.0, 1.0])

    true_z = 100.0
    true_vz = -10.0
    gravity_z = -9.81
    gravity_world = np.array([0.0, 0.0, gravity_z])

    for _ in range(steps):
        true_vz += gravity_z * dt
        true_z += true_vz * dt

        noisy_accel = np.array([0.0, 0.0, np.random.normal(0, 0.1)])
        noisy_alt = true_z + np.random.normal(0, np.sqrt(0.1))
        noisy_vel = np.array([0.0, 0.0, true_vz])

        estimator.predict(noisy_accel, identity_att, dt, gravity_world)
        estimator.update(noisy_alt, noisy_vel)

    state = estimator.get_state()
    assert abs(state[2] - true_z) < 10.0
    assert abs(state[5] - true_vz) < 5.0


def test_estimator_noisy_update():
    initial_state = np.zeros(6)
    initial_state[2] = 100.0
    initial_covariance = np.eye(6) * 1.0
    process_noise = np.eye(6) * 0.01

    sigma_alt = 2.0
    sigma_vel = 1.0
    sigma_accel = 0.1
    measurement_noise_alt = np.eye(1) * (sigma_alt ** 2)
    measurement_noise_vel = np.eye(3) * (sigma_vel ** 2)

    estimator = StateEstimator(
        initial_state,
        initial_covariance,
        process_noise,
        measurement_noise_alt,
        measurement_noise_vel,
    )

    dt = 0.1
    np.random.seed(42)

    true_z = 100.0
    true_vz = 0.0

    errors = []
    identity_att = np.array([0.0, 0.0, 0.0, 1.0])
    gravity_world = np.array([0.0, 0.0, -9.81])

    for _ in range(50):
        true_z += true_vz * dt

        noisy_accel = np.array([0.0, 0.0, 9.81 + np.random.normal(0, sigma_accel)])
        noisy_alt = true_z + np.random.normal(0, sigma_alt)
        noisy_vel = np.array([0.0, 0.0, true_vz + np.random.normal(0, sigma_vel)])

        estimator.predict(noisy_accel, identity_att, dt, gravity_world)
        estimator.update(noisy_alt, noisy_vel)

        estimated_z = estimator.get_state()[2]
        errors.append(estimated_z - true_z)

    rms_error = np.sqrt(np.mean(np.square(errors)))

    assert rms_error < sigma_alt
