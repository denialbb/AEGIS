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
    dt = 0.1
    
    estimator.predict(noisy_accel, dt)
    updated_state = estimator.update(noisy_alt)
    
    # Check shape
    assert updated_state.shape == (6,)
    
    # With alt measurement of 10.0, Z should move towards 10
    assert updated_state[2] > 0.0
    
    # Velocity Z should be updated by acceleration prediction
    assert updated_state[5] > 0.0

def test_estimator_synthetic_fall():
    initial_state = np.zeros(6)
    initial_state[2] = 100.0  # Z = 100m
    initial_covariance = np.eye(6)
    process_noise = np.eye(6) * 0.01
    measurement_noise = np.eye(1) * 0.1
    
    estimator = StateEstimator(initial_state, initial_covariance, process_noise, measurement_noise)
    
    # Simulate a fall at -9.81 m/s^2 for 1 second
    dt = 0.1
    for _ in range(10):
        # Accelerometer measures proper acceleration
        noisy_accel = np.array([0.0, 0.0, -9.81]) 
        # Z should decrease by v*dt + 0.5*a*dt^2 roughly
        noisy_alt = estimator.get_state()[2] + estimator.get_state()[5]*dt + 0.5*(-9.81)*dt**2
        
        estimator.predict(noisy_accel, dt)
        estimator.update(noisy_alt)
        
    state = estimator.get_state()
    # After 1s, velocity should be roughly -9.81
    assert np.isclose(state[5], -9.81, atol=0.5)
    # Z should be roughly 100 - 0.5*9.81*1^2 = 95.095
    assert np.isclose(state[2], 95.095, atol=0.5)

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
        # True dynamics: hover
        true_vz += 0.0 * dt
        true_z += true_vz * dt
        
        noisy_accel = np.array([0.0, 0.0, np.random.normal(0, 0.1)]) # Small accel noise
        noisy_alt = true_z + np.random.normal(0, sigma_alt) # Altimeter noise
        
        estimator.predict(noisy_accel, dt)
        estimator.update(noisy_alt)
        
        estimated_z = estimator.get_state()[2]
        errors.append(estimated_z - true_z)
        
    rms_error = np.sqrt(np.mean(np.square(errors)))
    
    # RMS error of the estimate should be lower than the raw sensor noise (sigma_alt)
    assert rms_error < sigma_alt
