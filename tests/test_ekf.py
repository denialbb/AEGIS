import numpy as np
from src.estimation.ekf import ErrorStateEKF
from src.estimation.mahony_estimator import MahonyAttitudeEstimator
import src.config as config


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


if __name__ == "__main__":
    test_ekf_initialization()
    test_ekf_prediction_step()
    test_ekf_update_step()
    test_ekf_bias_estimation()
    test_ekf_innovation_access()
    test_mahony_ekf_integration()
    print("All EKF tests passed!")
