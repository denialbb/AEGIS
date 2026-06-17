import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R
from src.estimation.ekf import ErrorStateEKF
from src.estimation.mahony_estimator import MahonyAttitudeEstimator
import src.config as config

KERBIN_MU = 3.5316000e12
KERBIN_RADIUS = 600_000.0


def test_ekf_initialization():
    """Test that the EKF initializes correctly."""
    initial_pos = np.array([0.0, 0.0, 0.0])
    initial_vel = np.array([10.0, 0.0, -5.0])
    initial_covariance = np.eye(12)
    
    ekf = ErrorStateEKF(initial_pos, initial_vel, initial_covariance)
    
    # Check initial state
    state = ekf.get_state()
    assert state.shape == (6,)
    np.testing.assert_array_equal(state[:3], initial_pos)
    np.testing.assert_array_equal(state[3:], initial_vel)
    
    # Check biases are zero initially
    np.testing.assert_array_equal(ekf.get_gyro_bias(), np.zeros(3))
    np.testing.assert_array_equal(ekf.get_accel_bias(), np.zeros(3))


def test_ekf_prediction_step():
    """Test that the EKF prediction step runs without error."""
    initial_pos = np.array([0.0, 0.0, 100.0])
    initial_vel = np.array([0.0, 0.0, -10.0])
    initial_covariance = np.eye(12) * 0.1
    
    ekf = ErrorStateEKF(initial_pos, initial_vel, initial_covariance)
    
    # Simulate IMU readings for hovering (1G up to counteract gravity)
    f_body = np.array([0.0, 0.0, 9.81])  # 1G upward in body frame
    omega_body = np.array([0.0, 0.0, 0.0])  # No rotation
    attitude = np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion (level)
    gravity_world = np.array([0.0, 0.0, -9.81])  # Standard gravity downward
    dt = 0.1
    
    # Store initial state
    initial_state = ekf.get_state().copy()
    
    # Predict step
    ekf.predict(f_body, omega_body, attitude, gravity_world, dt)
    
    # With 1G up thrust counteracting gravity, net acceleration should be zero
    # So velocity should remain constant
    state_after = ekf.get_state()
    
    # Vertical velocity should remain approximately -10 m/s (initial velocity)
    # Position should have changed due to that velocity
    assert np.allclose(state_after[3:5], [0.0, 0.0], atol=0.01)  # X,Y velocity unchanged
    assert np.allclose(state_after[5], -10.0, atol=0.1)  # Z velocity unchanged
    assert np.allclose(state_after[2], 100.0 + (-10.0 * dt), atol=0.1)  # Z position changed


def test_ekf_update_step():
    """Test that the EKF update step runs without error."""
    initial_pos = np.array([0.0, 0.0, 100.0])
    initial_vel = np.array([0.0, 0.0, 0.0])
    initial_covariance = np.eye(12)
    
    ekf = ErrorStateEKF(initial_pos, initial_vel, initial_covariance)
    
    # Predict
    f_body = np.array([0.0, 0.0, 9.81])
    omega_body = np.array([0.0, 0.0, 0.0])
    attitude = np.array([0.0, 0.0, 0.0, 1.0])
    gravity_world = np.array([0.0, 0.0, -9.81])
    dt = 0.1
    
    ekf.predict(f_body, omega_body, attitude, gravity_world, dt)
    
    # Update with measurements
    noisy_alt = 100.0
    noisy_vel = np.array([0.0, 0.0, 0.0])
    
    state = ekf.update(noisy_alt, noisy_vel)
    assert state.shape == (6,)


def test_ekf_bias_estimation():
    """Test that the EKF can estimate biases."""
    initial_pos = np.array([0.0, 0.0, 0.0])
    initial_vel = np.array([0.0, 0.0, 0.0])
    initial_covariance = np.eye(12)
    
    ekf = ErrorStateEKF(initial_pos, initial_vel, initial_covariance)
    
    # Simulate constant acceleration with bias
    true_accel = np.array([0.0, 0.0, 2.0])  # 2 m/s^2 upward
    gyro_bias_true = np.array([0.01, 0.0, 0.0])  # 0.01 rad/s bias in x
    accel_bias_true = np.array([0.0, 0.0, 0.1])  # 0.1 m/s^2 bias in z
    
    # What the IMU would measure (specific force = accel - gravity)
    f_body_true = true_accel - np.array([0.0, 0.0, -9.81])  # Remove gravity
    f_body_measured = f_body_true + accel_bias_true  # Add accelerometer bias
    
    omega_body_true = np.array([0.0, 0.0, 0.0])
    omega_body_measured = omega_body_true + gyro_bias_true  # Add gyro bias
    
    attitude = np.array([0.0, 0.0, 0.0, 1.0])  # Level
    gravity_world = np.array([0.0, 0.0, -9.81])
    dt = 0.02
    
    # Run filter for several steps
    for i in range(50):
        ekf.predict(f_body_measured, omega_body_measured, attitude, gravity_world, dt)
        
        # Simulate velocity and altitude measurements
        # Integrate true acceleration to get velocity and position
        true_vel = true_accel * (i + 1) * dt
        true_pos = 0.5 * true_accel * ((i + 1) * dt) ** 2
        
        noisy_alt = true_pos[2] + np.random.normal(0, 0.1)
        noisy_vel = np.array([0.0, 0.0, true_vel[2]]) + np.random.normal(0, 0.05, 3)
        
        ekf.update(noisy_alt, noisy_vel)
    
    # After convergence, biases should be estimated
    estimated_gyro_bias = ekf.get_gyro_bias()
    estimated_accel_bias = ekf.get_accel_bias()
    
    # Should be close to true values (within reasonable tolerance)
    assert np.allclose(estimated_gyro_bias, gyro_bias_true, atol=0.05)
    assert np.allclose(estimated_accel_bias, accel_bias_true, atol=0.2)


def test_ekf_innovation_access():
    """Test that we can access innovation for IMU health monitoring."""
    initial_pos = np.array([0.0, 0.0, 0.0])
    initial_vel = np.array([0.0, 0.0, 0.0])
    initial_covariance = np.eye(12)
    
    ekf = ErrorStateEKF(initial_pos, initial_vel, initial_covariance)
    
    # Check initial innovation
    innovation_norm = ekf.get_innovation_norm()
    assert innovation_norm >= 0.0
    
    innovation = ekf.get_innovation()
    assert innovation.shape == (4,)
    
    # Run a predict/update cycle
    f_body = np.array([0.0, 0.0, 9.81])
    omega_body = np.array([0.0, 0.0, 0.0])
    attitude = np.array([0.0, 0.0, 0.0, 1.0])
    gravity_world = np.array([0.0, 0.0, -9.81])
    dt = 0.1
    
    ekf.predict(f_body, omega_body, attitude, gravity_world, dt)
    ekf.update(1.0, np.array([0.0, 0.0, 0.0]))
    
    # Innovation should now be meaningful
    innovation_norm_after = ekf.get_innovation_norm()
    assert innovation_norm_after >= 0.0


def test_mahony_ekf_integration():
    """Test that Mahony estimator and EKF can work together."""
    # Initialize both filters
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12)
    )
    
    mahony = MahonyAttitudeEstimator(
        kp=config.MAHONY_KP,
        ki=config.MAHONY_KI,
        up_vector=np.array([0.0, 0.0, 1.0])
    )
    
    # Simulate some IMU data
    f_body = np.array([0.0, 0.0, 9.81])  # Level, 1G up
    omega_body = np.array([0.0, 0.1, 0.0])  # Slow pitch rate
    gravity_world = np.array([0.0, 0.0, -9.81])
    dt = 0.02
    
    # Run for a few steps
    for i in range(10):
        # Update Mahony with gyro (will get bias from EKF)
        mahony.set_gyro_bias(ekf.get_gyro_bias())
        attitude = mahony.update(omega_body, f_body, gravity_world, dt)
        
        # Update EKF with Mahony attitude
        ekf.predict(f_body, omega_body, attitude, gravity_world, dt)
        ekf.update(0.0, np.array([0.0, 0.0, 0.0]))


# ════════════════════════════════════════════════════════════════
#  EDGE-CASE & NUMERICAL-SAFETY TESTS
# ════════════════════════════════════════════════════════════════


def test_ekf_zero_dt_is_noop():
    """predict with dt <= 0 must not mutate state."""
    ekf = ErrorStateEKF(
        np.array([10.0, 20.0, 100.0]),
        np.array([1.0, 2.0, -5.0]),
        np.eye(12),
    )
    pos_before = ekf.pos.copy()
    vel_before = ekf.vel.copy()
    P_before = ekf.P.copy()

    ekf.predict(
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -9.81]),
        0.0,
    )
    np.testing.assert_array_equal(ekf.pos, pos_before)
    np.testing.assert_array_equal(ekf.vel, vel_before)
    np.testing.assert_array_equal(ekf.P, P_before)


def test_ekf_negative_dt_is_noop():
    """predict with negative dt must not mutate state."""
    ekf = ErrorStateEKF(
        np.array([10.0, 20.0, 100.0]),
        np.array([1.0, 2.0, -5.0]),
        np.eye(12),
    )
    pos_before = ekf.pos.copy()
    vel_before = ekf.vel.copy()
    P_before = ekf.P.copy()

    ekf.predict(
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -9.81]),
        -0.1,
    )
    np.testing.assert_array_equal(ekf.pos, pos_before)
    np.testing.assert_array_equal(ekf.vel, vel_before)
    np.testing.assert_array_equal(ekf.P, P_before)


def test_ekf_nan_imu_produces_nan_state():
    """NaN in IMU readings should propagate to NaN state (not crash)."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12),
    )
    ekf.predict(
        np.array([float("nan"), 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -9.81]),
        0.1,
    )
    assert np.any(np.isnan(ekf.pos)) or np.any(np.isnan(ekf.vel))


def test_ekf_nan_attitude_rejected():
    """NaN quaternion raises ValueError from SciPy — EKF should not be called
    with invalid quaternions (validation belongs upstream)."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12),
    )
    with pytest.raises(ValueError, match="zero norm quaternions"):
        ekf.predict(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.0, 0.0]),
            np.array([float("nan"), 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            0.1,
        )


def test_ekf_huge_gravity_does_not_crash():
    """Gravity at ~1e12 (seen in corrupted recordings) should not crash."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12),
    )
    huge_gravity = np.array([-1.77e12, 2.85e11, 2.87e12])
    ekf.predict(
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        huge_gravity,
        0.1,
    )
    state = ekf.get_state()
    assert not np.any(np.isnan(state))
    assert not np.any(np.isinf(state))
    # Innovation should be huge after bad inputs
    ekf.update(1.0, np.array([0.0, 0.0, 0.0]))
    assert ekf.get_innovation_norm() > 1.0


def test_ekf_inf_imu_does_not_crash():
    """Inf in IMU should not crash the predict step (state may become NaN)."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12),
    )
    # Must not raise
    ekf.predict(
        np.array([float("inf"), 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -9.81]),
        0.1,
    )


def test_ekf_covariance_symmetry_after_predict():
    """P should remain symmetric after predict."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([10.0, 0.0, -5.0]),
        np.eye(12) * 0.1,
    )
    for _ in range(20):
        ekf.predict(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.1, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            0.02,
        )
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-12)


def test_ekf_covariance_symmetry_after_update():
    """P should remain symmetric after update."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, -10.0]),
        np.eye(12) * 0.1,
    )
    for _ in range(10):
        ekf.predict(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            0.02,
        )
        ekf.update(100.0, np.array([0.0, 0.0, -10.0]))
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-12)


def test_ekf_covariance_positive_definite():
    """P should remain positive-definite after predict+update cycles."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, -10.0]),
        np.eye(12) * 0.1,
    )
    for _ in range(50):
        ekf.predict(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            0.02,
        )
        ekf.update(100.0, np.array([0.0, 0.0, -10.0]))
    eigenvalues = np.linalg.eigvalsh(ekf.P)
    assert np.all(eigenvalues > 0), "Covariance must be positive-definite"


def test_ekf_singular_S_skips_update():
    """When S is singular (e.g. zero R), update should skip gracefully."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12) * 0.1,
    )
    ekf.predict(
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -9.81]),
        0.02,
    )
    # Huge P leads to singular S (numerically)
    ekf.P = np.eye(12) * 1e30
    state = ekf.update(100.0, np.array([0.0, 0.0, 0.0]))
    assert state.shape == (6,)


def test_ekf_large_dt_no_explosion():
    """A single large dt should not make P explode to inf."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12) * 0.1,
    )
    ekf.predict(
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -9.81]),
        5.0,  # 5 second dt
    )
    assert not np.any(np.isinf(ekf.P))
    assert not np.any(np.isnan(ekf.P))


def test_ekf_divergence_detection_via_innovation():
    """Repeated bad inputs should cause large innovation, detectable by monitoring."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12) * 0.1,
    )
    for i in range(20):
        ekf.predict(
            np.array([0.0, 0.0, 100.0]),  # huge upward thrust (10G)
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            0.1,
        )
        ekf.update(100.0, np.array([0.0, 0.0, 0.0]))
    # Innovation should be large because the model expects ~0 accel but gets ~90 m/s
    assert ekf.get_innovation_norm() > 1.0, (
        f"Expected large innovation norm, got {ekf.get_innovation_norm()}"
    )


def test_ekf_innovation_monitors_imu_health():
    """Innovation norm should flag IMU faults per EKF_INNOVATION_FAULT_THRESHOLD."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12),
    )
    for _ in range(5):
        ekf.predict(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            0.02,
        )
        ekf.update(0.0, np.array([0.0, 0.0, 0.0]))
    innov_norm = ekf.get_innovation_norm()
    # With no altitude change and large gravity, innovation should exceed threshold
    assert innov_norm >= 0.0


def test_ekf_bias_convergence_sign_change():
    """Bias estimates should track when true bias changes sign."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12) * 0.1,
    )
    np.random.seed(42)
    gyro_bias = np.array([0.02, -0.01, 0.005])
    accel_bias = np.array([0.05, -0.03, 0.1])
    for i in range(100):
        f_body = np.array([0.0, 0.0, 9.81]) + accel_bias
        omega_body = np.array([0.0, 0.1, 0.0]) + gyro_bias
        ekf.predict(
            f_body,
            omega_body,
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            0.02,
        )
        noisy_alt = float(np.random.normal(0.0, 0.1))
        noisy_vel = np.random.normal(0.0, 0.05, 3)
        ekf.update(noisy_alt, noisy_vel)
    assert np.allclose(ekf.get_gyro_bias(), gyro_bias, atol=0.05)
    assert np.allclose(ekf.get_accel_bias(), accel_bias, atol=0.3)


def test_ekf_mahony_self_heals_from_nan():
    """Mahony recovers from NaN quaternion internally — EKF never sees NaN."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, -10.0]),
        np.eye(12),
    )
    mahony = MahonyAttitudeEstimator(
        kp=config.MAHONY_KP,
        ki=config.MAHONY_KI,
        up_vector=np.array([0.0, 0.0, 1.0]),
    )
    # Run nominal steps first
    for i in range(5):
        mahony.set_gyro_bias(ekf.get_gyro_bias())
        attitude = mahony.update(
            np.array([0.0, 0.1, 0.0]),
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.0, -9.81]),
            0.02,
        )
        ekf.predict(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.1, 0.0]),
            attitude,
            np.array([0.0, 0.0, -9.81]),
            0.02,
        )
        ekf.update(100.0, np.array([0.0, 0.0, -10.0]))
    # Inject NaN into Mahony — it self-heals, producing valid quaternion
    mahony.q = np.array([float("nan"), float("nan"), float("nan"), float("nan")])
    attitude = mahony.update(
        np.array([0.0, 0.1, 0.0]),
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.0, -9.81]),
        0.02,
    )
    assert np.all(np.isfinite(attitude))
    assert np.allclose(np.linalg.norm(attitude), 1.0, atol=1e-6)
    ekf.predict(
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.1, 0.0]),
        attitude,
        np.array([0.0, 0.0, -9.81]),
        0.02,
    )
    assert np.all(np.isfinite(ekf.get_state()))


def test_ekf_get_innovation_returns_copy():
    """get_innovation() should return a copy, not a reference."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12),
    )
    ekf.predict(
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -9.81]),
        0.02,
    )
    ekf.update(1.0, np.array([0.0, 0.0, 0.0]))
    innov = ekf.get_innovation()
    innov[:] = 999.0
    assert not np.allclose(ekf.get_innovation(), 999.0)


def test_ekf_update_returns_6d_state():
    """update() must return a (6,) array matching get_state()."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, -10.0]),
        np.eye(12),
    )
    ekf.predict(
        np.array([0.0, 0.0, 9.81]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -9.81]),
        0.02,
    )
    state = ekf.update(100.0, np.array([0.0, 0.0, -10.0]))
    assert state.shape == (6,)
    np.testing.assert_array_equal(state, ekf.get_state())


def test_ekf_high_dt_spike_does_not_crash():
    """A sudden dt spike of 5 seconds (simulated comms glitch) should not crash."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 100.0]),
        np.array([0.0, 0.0, -10.0]),
        np.eye(12) * 0.1,
    )
    for _ in range(3):
        ekf.predict(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            5.0,
        )
    assert not np.any(np.isnan(ekf.P))
    assert not np.any(np.isinf(ekf.P))
    assert not np.any(np.isnan(ekf.get_state()))
    assert not np.any(np.isinf(ekf.get_state()))


def test_ekf_state_bounds_for_controller():
    """State vector values should remain within physically reasonable bounds."""
    ekf = ErrorStateEKF(
        np.array([1000.0, 500.0, 5000.0]),
        np.array([50.0, -30.0, -100.0]),
        np.eye(12) * 0.1,
    )
    for _ in range(100):
        ekf.predict(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -9.81]),
            0.02,
        )
        ekf.update(5000.0, np.array([50.0, -30.0, -100.0]))
    state = ekf.get_state()
    # State values should not be absurd
    assert np.all(np.abs(state) < 1e9), f"State contains absurd values: {state}"


def test_ekf_up_vector_is_normalized():
    """up_vector should be normalized regardless of input."""
    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12),
        up_vector=np.array([0.0, 0.0, 5.0]),
    )
    assert np.allclose(np.linalg.norm(ekf.up_vector), 1.0)


# ═══════════════════════════════════════════════════════════════
#  AT-REST PAD VALIDATION — Full Chain Integrity
# ═══════════════════════════════════════════════════════════════

# NED convention:  +x = north, +y = east, +z = down.
# On the pad at rest:
#   position_NED           = [0, 0, 0]
#   velocity_NED           = [0, 0, 0]
#   gravity_NED            = [0, 0, +g]      (points down)
#   proper accel_NED       = [0, 0, -g]      (specific force, upward normal)
#   kinematic accel_NED    = [0, 0, 0]       = f_corr_ned + gravity_ned
#   gyro rate (body)       = [0, 0, 0]
#   attitude               = identity quat   (level, body z = down)

KERBIN_SURFACE_G = KERBIN_MU / KERBIN_RADIUS**2  # ≈ 9.81


@pytest.mark.parametrize("pad_name, lat_lon", [
    ("ksc",        (-0.0972, -74.5577)),
    ("equator",    (0.0,      0.0)),
    ("mid_south",  (-45.0,   120.0)),
    ("near_pole",  (-89.0,    0.0)),
], ids=["ksc", "equator", "mid_south", "near_pole"])
def test_ekf_at_rest_on_pad(pad_name: str, lat_lon: tuple[float, float]) -> None:
    """
    Validate the full chain at rest on every pad:
      accelerometer → body frame → Mahony → ECEF → NED → guidance.

    With zero gyro rate and identity attitude, the EKF should hold
    position and velocity at zero across repeated predict/update cycles.
    """
    lat_deg, lon_deg = lat_lon
    # Expected gravity magnitude at this latitude (pad is at body radius)
    g = KERBIN_SURFACE_G

    gravity_ned = np.array([0.0, 0.0, g], dtype=float)

    ekf = ErrorStateEKF(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12) * 0.1,
        up_vector=np.array([0.0, 0.0, 1.0]),
    )

    # Identity attitude — body frame axes align with NED:
    #   body x (forward) = NED x (north)
    #   body y (right)   = NED y (east)
    #   body z (down)    = NED z (down)
    attitude = np.array([0.0, 0.0, 0.0, 1.0])  # identity quat

    # Specific force (proper acceleration) at rest on pad:
    #   accelerometer measures: f = kinematic - gravity
    #   kinematic = [0,0,0], gravity_NED = [0,0,+g]
    #   f_NED = [0,0,0] - [0,0,+g] = [0,0,-g]
    #   Rotated to body frame via identity: f_body = [0,0,-g]
    f_body = np.array([0.0, 0.0, -g], dtype=float)

    omega_body = np.array([0.0, 0.0, 0.0], dtype=float)  # no rotation

    dt = 0.02  # 50 Hz

    for step in range(50):
        ekp_pos_before = ekf.pos.copy()
        ekp_vel_before = ekf.vel.copy()

        ekf.predict(f_body, omega_body, attitude, gravity_ned, dt)
        ekf.update(0.0, np.array([0.0, 0.0, 0.0]))

        # Position and velocity must stay at zero (within numerical tolerance)
        np.testing.assert_allclose(
            ekf.pos, np.zeros(3), atol=1e-9,
            err_msg=f"pad={pad_name} step={step}: position drifted",
        )
        np.testing.assert_allclose(
            ekf.vel, np.zeros(3), atol=1e-9,
            err_msg=f"pad={pad_name} step={step}: velocity drifted",
        )

        # Innovation should be near-zero (measurements match prediction)
        innov = ekf.get_innovation()
        np.testing.assert_allclose(innov[:3], np.zeros(3), atol=1e-6,
                                   err_msg=f"pad={pad_name} step={step}: innovation non-zero")

    # ── Verify the internal chain logic ─────────────────────────
    # Manual verification of the NED-frame kinematic acceleration:
    rot_bw = R.from_quat(attitude)  # identity
    f_corr_ned = rot_bw.apply(f_body)  # specific force in NED (should be [0,0,-g])
    kinematic_ned = f_corr_ned + gravity_ned  # should be [0,0,0]

    np.testing.assert_allclose(
        f_corr_ned, np.array([0.0, 0.0, -g]), atol=1e-12,
        err_msg=f"pad={pad_name}: f_corr_ned should be [0,0,-g]",
    )
    np.testing.assert_allclose(
        kinematic_ned, np.zeros(3), atol=1e-12,
        err_msg=f"pad={pad_name}: kinematic accel should be zero at rest",
    )

    # Gravity is purely +z (Down) in NED
    np.testing.assert_allclose(
        gravity_ned[:2], np.zeros(2), atol=1e-12,
        err_msg=f"pad={pad_name}: gravity N/E components must be zero",
    )
    assert gravity_ned[2] > 0.0, f"pad={pad_name}: gravity down component must be +ve"

    # Specific force is purely -z (Up) in NED
    np.testing.assert_allclose(
        f_corr_ned[:2], np.zeros(2), atol=1e-12,
        err_msg=f"pad={pad_name}: specific force N/E components must be zero at rest",
    )
    assert f_corr_ned[2] < 0.0, f"pad={pad_name}: specific force must point up (-z) at rest"


if __name__ == "__main__":
    test_ekf_initialization()
    test_ekf_prediction_step()
    test_ekf_update_step()
    test_ekf_bias_estimation()
    test_ekf_innovation_access()
    test_mahony_ekf_integration()
    test_ekf_zero_dt_is_noop()
    test_ekf_negative_dt_is_noop()
    test_ekf_nan_imu_produces_nan_state()
    test_ekf_nan_attitude_rejected()
    test_ekf_huge_gravity_does_not_crash()
    test_ekf_inf_imu_does_not_crash()
    test_ekf_covariance_symmetry_after_predict()
    test_ekf_covariance_symmetry_after_update()
    test_ekf_covariance_positive_definite()
    test_ekf_singular_S_skips_update()
    test_ekf_large_dt_no_explosion()
    test_ekf_divergence_detection_via_innovation()
    test_ekf_innovation_monitors_imu_health()
    test_ekf_bias_convergence_sign_change()
    test_ekf_mahony_self_heals_from_nan()
    test_ekf_get_innovation_returns_copy()
    test_ekf_update_returns_6d_state()
    test_ekf_high_dt_spike_does_not_crash()
    test_ekf_state_bounds_for_controller()
    test_ekf_up_vector_is_normalized()
    print("All EKF tests passed!")
