import numpy as np
import pytest
from src.guidance.controller import GuidanceController

def test_translation_pd_control():
    """Test that pure translation differences produce correct force in body frame without rotation."""
    controller = GuidanceController(
        kp_pos=[1.0, 1.0, 1.0],
        kd_vel=[2.0, 2.0, 2.0],
        kp_att=[0.0, 0.0, 0.0],
        kd_att=[0.0, 0.0, 0.0]
    )
    
    current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target_state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    current_attitude = np.array([1.0, 0.0, 0.0, 0.0]) # upright
    mass = 1000.0
    
    wrench = controller.compute_wrench(current_state, current_attitude, mass, target_state)
    
    # expected force = mass * (kp * (target - current) + kd * (target - current))
    # force_x = 1000 * (1.0 * 10.0 + 2.0 * 0.0) = 10000
    assert np.allclose(wrench[:3], [10000.0, 0.0, 0.0])
    assert np.allclose(wrench[3:], [0.0, 0.0, 0.0])

def test_attitude_pd_control():
    """Test that pure attitude differences produce correct torque in body frame."""
    controller = GuidanceController(
        kp_pos=[0.0, 0.0, 0.0],
        kd_vel=[0.0, 0.0, 0.0],
        kp_att=[10.0, 10.0, 10.0],
        kd_att=[5.0, 5.0, 5.0]
    )
    
    current_state = np.zeros(6)
    target_state = np.zeros(6)
    mass = 1000.0
    
    # Rotate 90 degrees around X axis: q = [cos(45), sin(45), 0, 0]
    # scalar first: [0.70710678, 0.70710678, 0.0, 0.0]
    # To return to upright [1, 0, 0, 0], we need to rotate -90 degrees around X.
    # The error axis should be roughly [-0.70710678, 0, 0]
    current_attitude = np.array([np.cos(np.pi/4), np.sin(np.pi/4), 0.0, 0.0])
    
    wrench = controller.compute_wrench(current_state, current_attitude, mass, target_state, dt=0.1)
    
    assert np.allclose(wrench[:3], [0.0, 0.0, 0.0])
    
    # For torque, the error axis is [-sin(45), 0, 0].
    # prev error was 0, so d_err = [-sin(45)/0.1, 0, 0]
    # torque = Kp * [-sin(45)] - Kd * [-sin(45)/0.1]
    err_x = -np.sin(np.pi/4)
    expected_torque_x = 10.0 * err_x - 5.0 * (err_x / 0.1)
    
    assert np.isclose(wrench[3], expected_torque_x)
    assert np.allclose(wrench[4:], [0.0, 0.0])

def test_gravity_compensation():
    """Test that feed-forward gravity compensation works."""
    controller = GuidanceController(
        kp_pos=[1.0, 1.0, 1.0],
        kd_vel=[2.0, 2.0, 2.0],
        kp_att=[0.0, 0.0, 0.0],
        kd_att=[0.0, 0.0, 0.0],
        gravity=[0.0, 0.0, -9.81]
    )
    
    current_state = np.zeros(6)
    target_state = np.zeros(6)
    current_attitude = np.array([1.0, 0.0, 0.0, 0.0])
    mass = 1000.0
    
    wrench = controller.compute_wrench(current_state, current_attitude, mass, target_state)
    
    # Force should just counteract gravity
    # a_cmd_world = -gravity = [0, 0, 9.81]
    # force = 1000 * 9.81 = 9810.0
    assert np.allclose(wrench[:3], [0.0, 0.0, 9810.0])

def test_rotation_to_body_frame():
    """Test that world-frame acceleration commands are properly rotated to the body frame."""
    controller = GuidanceController(
        kp_pos=[1.0, 1.0, 1.0],
        kd_vel=[2.0, 2.0, 2.0],
        kp_att=[0.0, 0.0, 0.0],
        kd_att=[0.0, 0.0, 0.0]
    )
    
    current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target_state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # Target is in +X in world frame.
    
    # Vessel is rotated 90 degrees around Z axis: +X body is +Y world, +Y body is -X world
    # So world +X corresponds to body -Y.
    # Quaternion for 90 deg around Z: [cos(45), 0, 0, sin(45)]
    current_attitude = np.array([np.cos(np.pi/4), 0.0, 0.0, np.sin(np.pi/4)])
    mass = 1000.0
    
    wrench = controller.compute_wrench(current_state, current_attitude, mass, target_state)
    
    # World force is [10000, 0, 0]
    # rot maps from body to world.
    # body = [0, -1, 0] -> world = [1, 0, 0]
    # So world [10000, 0, 0] -> body [0, -10000, 0]
    assert np.allclose(wrench[:3], [0.0, -10000.0, 0.0], atol=1e-5)
    assert np.allclose(wrench[3:], [0.0, 0.0, 0.0])

def test_reset():
    """Test that reset() correctly clears the derivative state."""
    controller = GuidanceController(
        kp_pos=[0.0, 0.0, 0.0],
        kd_vel=[0.0, 0.0, 0.0],
        kp_att=[10.0, 10.0, 10.0],
        kd_att=[5.0, 5.0, 5.0]
    )
    
    current_state = np.zeros(6)
    target_state = np.zeros(6)
    mass = 1000.0
    
    # Induce an error
    current_attitude = np.array([np.cos(np.pi/4), np.sin(np.pi/4), 0.0, 0.0])
    controller.compute_wrench(current_state, current_attitude, mass, target_state, dt=0.1)
    
    assert not np.allclose(controller.last_att_error, np.zeros(3))
    
    controller.reset()
    assert np.allclose(controller.last_att_error, np.zeros(3))
