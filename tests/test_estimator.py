import numpy as np
from src.estimation.estimator import StateEstimator

def test_estimator_initialization():
    initial_state = np.zeros(7)
    initial_covariance = np.eye(7)
    process_noise = np.eye(7) * 0.1
    measurement_noise = np.eye(4) * 0.5
    
    estimator = StateEstimator(initial_state, initial_covariance, process_noise, measurement_noise)
    state = estimator.get_state()
    assert state.shape == (7,)
    np.testing.assert_array_equal(state, initial_state)

def test_estimator_update():
    initial_state = np.zeros(7)
    initial_state[6] = 1000.0  # Mass
    initial_covariance = np.eye(7)
    process_noise = np.eye(7) * 0.01
    measurement_noise = np.eye(4) * 0.1
    
    estimator = StateEstimator(initial_state, initial_covariance, process_noise, measurement_noise)
    
    noisy_alt = 10.0
    noisy_accel = np.array([0.0, 0.0, 9.81])
    dt = 0.1
    
    updated_state = estimator.update(noisy_alt, noisy_accel, dt)
    
    # Check shape
    assert updated_state.shape == (7,)
    
    # With alt measurement of 10.0, Z should move towards 10
    assert updated_state[2] > 0.0
    
    # Velocity Z should be updated by acceleration
    assert updated_state[5] > 0.0
    
    # Mass should remain unchanged
    assert np.isclose(updated_state[6], 1000.0)

def test_estimator_synthetic_fall():
    initial_state = np.zeros(7)
    initial_state[2] = 100.0  # Z = 100m
    initial_state[6] = 1000.0 # Mass
    initial_covariance = np.eye(7)
    process_noise = np.eye(7) * 0.01
    measurement_noise = np.eye(4) * 0.1
    
    estimator = StateEstimator(initial_state, initial_covariance, process_noise, measurement_noise)
    
    # Simulate a fall at -9.81 m/s^2 for 1 second
    dt = 0.1
    for _ in range(10):
        # Accelerometer measures proper acceleration, but let's assume it outputs -9.81 for this test
        noisy_accel = np.array([0.0, 0.0, -9.81]) 
        # Z should decrease by v*dt + 0.5*a*dt^2 roughly
        noisy_alt = estimator.get_state()[2] + estimator.get_state()[5]*dt + 0.5*(-9.81)*dt**2
        
        estimator.update(noisy_alt, noisy_accel, dt)
        
    state = estimator.get_state()
    # After 1s, velocity should be roughly -9.81
    assert np.isclose(state[5], -9.81, atol=0.5)
    # Z should be roughly 100 - 0.5*9.81*1^2 = 95.095
    assert np.isclose(state[2], 95.095, atol=0.5)
