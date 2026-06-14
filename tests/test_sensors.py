import pytest
import numpy as np
from unittest.mock import MagicMock
from src.telemetry.sensors import SensorModels
import src.config as config

def test_sensor_models_noise_statistics():
    # Mock kRPC connection and streams
    conn = MagicMock()
    vessel = MagicMock()
    ref_frame = MagicMock()
    flight = MagicMock()
    
    vessel.flight.return_value = flight
    
    # Define up_vector
    up_vector = np.array([0.0, 0.0, 1.0])
    
    # Mock the streams to return constant perfect values
    altitude_mock = MagicMock(return_value=1000.0)
    # Falling velocity decreasing by 9.81 each second
    ut_values = [0.0]
    vel_values = [np.array([0.0, 0.0, 0.0])]
    def velocity_mock_func():
        return vel_values[0]
    def ut_mock_func():
        return ut_values[0]
        
    velocity_mock = MagicMock(side_effect=velocity_mock_func)
    ut_mock = MagicMock(side_effect=ut_mock_func)
    attitude_mock = MagicMock(return_value=(1.0, 0.0, 0.0, 0.0))
    mass_mock = MagicMock(return_value=5000.0)
    angular_vel_mock = MagicMock(return_value=(0.0, 0.0, 0.0))
    aero_mock = MagicMock(return_value=(0.0, 0.0, 0.0))
    situation_mock = MagicMock()
    situation_mock.name = "flying"
    situation_stream = MagicMock(return_value=situation_mock)
    
    # Simulate conn.add_stream returning callable mock streams
    def add_stream_side_effect(func, obj, attr):
        if attr == 'surface_altitude':
            return altitude_mock
        elif attr == 'velocity':
            return velocity_mock
        elif attr == 'ut':
            return ut_mock
        elif attr == 'rotation':
            return attitude_mock
        elif attr == 'mass':
            return mass_mock
        elif attr == 'angular_velocity':
            return angular_vel_mock
        elif attr == 'aerodynamic_force':
            return aero_mock
        elif attr == 'situation':
            return situation_stream
        return MagicMock()
        
    conn.add_stream.side_effect = add_stream_side_effect
    
    # Instantiate the sensor models
    sensors = SensorModels(conn, vessel, ref_frame, up_vector)
    
    # Run a statistical test by polling multiple times
    n_samples = 10000
    alt_samples = []
    accel_samples = []
    
    # First poll initializes last_vel and last_ut
    sensors.poll()
    
    for _ in range(n_samples):
        # Advance time by 0.02s
        ut_values[0] += 0.02
        # Velocity changes due to gravity (falling straight down)
        vel_values[0] = vel_values[0] + np.array([0.0, 0.0, -9.81 * 0.02])
        
        result = sensors.poll()
        noisy_alt, noisy_accel, attitude, mass = result[0], result[1], result[2], result[3]
        alt_samples.append(noisy_alt)
        accel_samples.append(noisy_accel)
        
        # Verify perfect values are passed through untouched where expected
        np.testing.assert_array_equal(attitude, np.array([1.0, 0.0, 0.0, 0.0]))
        assert mass == 5000.0
        
        # Verify angular velocity is returned correctly
        np.testing.assert_array_equal(result[6], np.array([0.0, 0.0, 0.0]))
        
    alt_samples = np.array(alt_samples)
    accel_samples = np.array(accel_samples)
    
    # Verify the mean is roughly the perfect value (zero-mean noise)
    assert np.isclose(np.mean(alt_samples), 1000.0, atol=0.1)
    
    # Accel should be close to 0,0,0 since we are in freefall and proper accel is 0
    assert np.isclose(np.mean(accel_samples[:, 0]), 0.0, atol=0.05)
    assert np.isclose(np.mean(accel_samples[:, 1]), 0.0, atol=0.05)
    assert np.isclose(np.mean(accel_samples[:, 2]), 0.0, atol=0.05)
    
    # Verify the standard deviation matches the config (within 5% relative tolerance)
    assert np.isclose(np.std(alt_samples), config.SIGMA_ALT, rtol=0.05)
    assert np.isclose(np.std(accel_samples[:, 0]), config.SIGMA_ACCEL, rtol=0.05)
    assert np.isclose(np.std(accel_samples[:, 1]), config.SIGMA_ACCEL, rtol=0.05)
    assert np.isclose(np.std(accel_samples[:, 2]), config.SIGMA_ACCEL, rtol=0.05)
