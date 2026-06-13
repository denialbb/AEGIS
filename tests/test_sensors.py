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
    
    # Mock the streams to return constant perfect values
    altitude_mock = MagicMock(return_value=1000.0)
    accel_mock = MagicMock(return_value=(0.0, 9.8, 0.0))
    attitude_mock = MagicMock(return_value=(1.0, 0.0, 0.0, 0.0))
    mass_mock = MagicMock(return_value=5000.0)
    
    # Simulate conn.add_stream returning callable mock streams
    def add_stream_side_effect(func, obj, attr):
        if attr == 'surface_altitude':
            return altitude_mock
        elif attr == 'acceleration':
            return accel_mock
        elif attr == 'rotation':
            return attitude_mock
        elif attr == 'mass':
            return mass_mock
        return MagicMock()
        
    conn.add_stream.side_effect = add_stream_side_effect
    
    # Instantiate the sensor models
    sensors = SensorModels(conn, vessel, ref_frame)
    
    # Run a statistical test by polling multiple times
    n_samples = 10000
    alt_samples = []
    accel_samples = []
    
    for _ in range(n_samples):
        noisy_alt, noisy_accel, attitude, mass = sensors.poll()
        alt_samples.append(noisy_alt)
        accel_samples.append(noisy_accel)
        
        # Verify perfect values are passed through untouched where expected
        np.testing.assert_array_equal(attitude, np.array([1.0, 0.0, 0.0, 0.0]))
        assert mass == 5000.0
        
    alt_samples = np.array(alt_samples)
    accel_samples = np.array(accel_samples)
    
    # Verify the mean is roughly the perfect value (zero-mean noise)
    assert np.isclose(np.mean(alt_samples), 1000.0, atol=0.1)
    assert np.isclose(np.mean(accel_samples[:, 0]), 0.0, atol=0.05)
    assert np.isclose(np.mean(accel_samples[:, 1]), 9.8, atol=0.05)
    assert np.isclose(np.mean(accel_samples[:, 2]), 0.0, atol=0.05)
    
    # Verify the standard deviation matches the config (within 5% relative tolerance)
    assert np.isclose(np.std(alt_samples), config.SIGMA_ALT, rtol=0.05)
    assert np.isclose(np.std(accel_samples[:, 0]), config.SIGMA_ACCEL, rtol=0.05)
    assert np.isclose(np.std(accel_samples[:, 1]), config.SIGMA_ACCEL, rtol=0.05)
    assert np.isclose(np.std(accel_samples[:, 2]), config.SIGMA_ACCEL, rtol=0.05)
