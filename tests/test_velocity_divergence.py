import numpy as np
from src.estimation.estimator import StateEstimator


def test_velocity_divergence_free_fall_then_burn():
    """
    Verify the estimator does not diverge during a realistic descent profile:
    Phase 1 — Free-fall: gravity acts, IMU reads ~1G, altimeter drifts.
    Phase 2 — Powered braking: engines fire, IMU reads high proper acceleration.

    The Kalman gain must properly weigh IMU acceleration against altimeter data
    so that velocity error stays bounded.
    """
    np.random.seed(42)
    dt = 0.1
    sigma_alt = 2.0
    sigma_vel = 0.5
    sigma_accel = 0.5

    initial_state = np.zeros(6)
    initial_state[2] = 1000.0
    initial_state[5] = -50.0
    P0 = np.eye(6)
    base_Q = np.eye(6) * 0.01
    R_alt = np.eye(1) * (sigma_alt ** 2)
    R_vel = np.eye(3) * (sigma_vel ** 2)

    est = StateEstimator(initial_state, P0, base_Q, R_alt, R_vel)

    true_pos = np.array([0.0, 0.0, 1000.0])
    true_vel = np.array([0.0, 0.0, -50.0])
    gravity_world = np.array([0.0, 0.0, -9.81])
    up = np.array([0.0, 0.0, 1.0])
    identity_att = np.array([0.0, 0.0, 0.0, 1.0])

    vel_errors: list[float] = []
    alt_errors: list[float] = []

    for step in range(200):
        true_alt = float(np.dot(true_pos, up))
        thrust_on = (step >= 80) and (step < 160)
        if thrust_on:
            thrust_accel = np.array([0.0, 0.0, 9.0])
        else:
            thrust_accel = np.array([0.0, 0.0, 0.0])

        net_accel = thrust_accel + gravity_world
        true_vel = true_vel + net_accel * dt
        true_pos = true_pos + true_vel * dt

        proper_accel_world = net_accel - gravity_world
        noisy_accel_body = proper_accel_world + np.random.normal(0, sigma_accel, 3)
        noisy_alt = true_alt + np.random.normal(0, sigma_alt)
        noisy_vel = true_vel + np.random.normal(0, sigma_vel, 3)

        est.predict(noisy_accel_body, identity_att, dt)
        est.update(noisy_alt, noisy_vel)

        state = est.get_state()
        est_alt = float(np.dot(state[:3], up))
        est_vz = float(np.dot(state[3:], up))

        vel_errors.append(abs(est_vz - true_vel[2]))
        alt_errors.append(abs(est_alt - true_alt))

    max_vel_error = max(vel_errors)
    final_vel_error = vel_errors[-1]

    assert max_vel_error < 15.0, (
        f"Velocity diverged: max error={max_vel_error:.2f} m/s during descent. "
        f"Filter is not properly fusing IMU data."
    )
    assert final_vel_error < 5.0, (
        f"Final velocity error={final_vel_error:.2f} m/s. "
        f"Filter did not converge after burn."
    )

    rms_alt_error = np.sqrt(np.mean([e ** 2 for e in alt_errors]))
    assert rms_alt_error < 20.0, (
        f"Altitude RMS error={rms_alt_error:.2f} m. "
        f"Filter is not tracking altitude properly."
    )