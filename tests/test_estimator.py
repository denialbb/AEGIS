import numpy as np
from src.estimation.estimator import StateEstimator

def test_estimator_initialization():
    initial_state = np.zeros(6)
    initial_covariance = np.eye(6)
    process_noise = np.eye(6) * 0.1
    measurement_noise = np.eye(1) * 0.5
    
    estimator = StateEstimator(initial_state, initial_covariance, process_noise, measurement_noise)
    state = estimator.get_state()
    assert state.shape == (6,)
    np.testing.assert_array_equal(state, initial_state)

def test_estimator_update():
    initial_state = np.zeros(6)
    initial_covariance = np.eye(6)
    process_noise = np.eye(6) * 0.01
    measurement_noise = np.eye(1) * 0.1
    
    estimator = StateEstimator(initial_state, initial_covariance, process_noise, measurement_noise)
    
    noisy_alt = 10.0
    noisy_accel = np.array([0.0, 0.0, 9.81])
    # Use correct quaternion for no rotation: [x, y, z, w] = [0, 0, 0, 1]
    dummy_attitude = np.array([0.0, 0.0, 0.0, 1.0])
    dt = 0.1
    
    estimator.predict(noisy_accel, dummy_attitude, dt)
    updated_state = estimator.update(noisy_alt)
    
    # Check shape
    assert updated_state.shape == (6,)
    
    # With alt measurement of 10.0, Z should move towards 10
    assert updated_state[2] > 0.0
    assert updated_state[2] < 10.0  # Should not overshoot significantly
    
    # With accelerometer reading [0,0,9.81] and no rotation:
    # proper_accel_world = [0,0,9.81] 
    # gravity_world = [0,0,-9.81]
    # kinematic_accel_world = [0,0,0] (no net acceleration)
    # The velocity change comes from Kalman filter coupling due to position residual
    # With a 10m position residual after predict step, some velocity adjustment is expected
    # but it should be reasonable (not huge)
    assert abs(updated_state[5]) < 5.0  # Reasonable bound for velocity change

def test_estimator_synthetic_fall():
    initial_state = np.zeros(6)
    initial_state[2] = 100.0  # Start at Z=100m
    initial_state[5] = -10.0  # Initial downward velocity of -10 m/s
    initial_covariance = np.eye(6)
    process_noise = np.eye(6) * 0.01  # Small process noise
    measurement_noise = np.eye(1) * 0.1  # Altimeter noise variance
    
    estimator = StateEstimator(initial_state, initial_covariance, process_noise, measurement_noise)
    
    # Simulate constant velocity motion (zero acceleration)
    # During constant velocity flight:
    # Kinematic acceleration = [0,0,0]
    # Proper acceleration = kinematic - gravity = [0,0,0] - [0,0,-9.81] = [0,0,9.81]
    # So accelerometer should read [0,0,9.81] (1G upward)
    dt = 0.1
    steps = 50  # 5 seconds
    
    for _ in range(steps):
        # Constant velocity motion: accelerometer reads 1G upward
        noisy_accel = np.array([0.0, 0.0, 9.81])
        # Correct quaternion for no rotation
        dummy_attitude = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Predict with zero acceleration input
        estimator.predict(noisy_accel, dummy_attitude, dt)
        # Update with noisy altitude measurement
        true_altitude = estimator.get_state()[2]  # Get predicted altitude
        noisy_alt = true_altitude + np.random.normal(0, np.sqrt(0.1))
        estimator.update(noisy_alt)
    
    state = estimator.get_state()
    # After 5 seconds of constant -10 m/s velocity, position should change by -10 * 5 = -50m
    # Starting at 100m, should be around 50m
    # Allow some tolerance due to noise and filtering
    assert state[2] > 40.0 and state[2] < 60.0
    # Velocity should remain around -10 m/s
    assert abs(state[5] - (-10.0)) < 2.0

def test_estimator_noisy_update():
    initial_state = np.zeros(6)
    initial_state[2] = 100.0
    initial_covariance = np.eye(6) * 1.0
    process_noise = np.eye(6) * 0.01
    
    # Let's say altimeter sigma is 2.0m -> variance is 4.0
    sigma_alt = 2.0
    measurement_noise = np.eye(1) * (sigma_alt ** 2)
    
    estimator = StateEstimator(initial_state, initial_covariance, process_noise, measurement_noise)
    
    dt = 0.1
    np.random.seed(42)
    
    true_z = 100.0
    true_vz = 0.0
    
    errors = []
    
    # Run for 5 seconds (50 steps) hover
    for _ in range(50):
        # True dynamics: hover (zero acceleration)
        true_vz += 0.0 * dt
        true_z += true_vz * dt
        
        # During hover: true acceleration = 0
        # Accelerometer measures proper acceleration = kinematic - gravity
        # = [0,0,0] - [0,0,-9.81] = [0,0,9.81] plus noise
        noisy_accel = np.array([0.0, 0.0, 9.81 + np.random.normal(0, 0.1)]) 
        # Correct quaternion for no rotation
        dummy_attitude = np.array([0.0, 0.0, 0.0, 1.0])
        noisy_alt = true_z + np.random.normal(0, sigma_alt) # Altimeter noise
        
        estimator.predict(noisy_accel, dummy_attitude, dt)
        estimator.update(noisy_alt)
        
        estimated_z = estimator.get_state()[2]
        errors.append(estimated_z - true_z)
    
    rms_error = np.sqrt(np.mean(np.square(errors)))
    
    # RMS error of the estimate should be lower than the raw sensor noise (sigma_alt)
    assert rms_error < sigma_alt
