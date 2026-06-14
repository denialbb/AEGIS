import numpy as np
import pytest
from src.guidance.controller import GuidanceController


def test_translation_pd_control():
    """Test that pure translation differences produce correct force in body frame without rotation."""
    controller = GuidanceController(
        kp_pos_lateral=1.0,
        kp_pos_vertical=1.0,
        kd_vel_lateral=2.0,
        kd_vel_vertical=2.0,
        kp_att=np.array([0.0, 0.0, 0.0]),
        kd_att=np.array([0.0, 0.0, 0.0]),
    )

    current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target_state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # Identity quaternion [x,y,z,w] = [0,0,0,1] for upright
    current_attitude = np.array([0.0, 0.0, 0.0, 1.0])
    mass = 1000.0
    up = np.array([0.0, 0.0, 1.0])

    wrench = controller.compute_wrench(current_state, current_attitude, mass, target_state, up)

    # force_x = 1000 * (1.0 * 10.0 + 2.0 * 0.0) = 10000
    assert np.allclose(wrench[:3], [10000.0, 0.0, 0.0])
    assert np.allclose(wrench[3:], [0.0, 0.0, 0.0])


def test_attitude_pd_control():
    """Test that pure attitude differences produce correct torque in body frame."""
    controller = GuidanceController(
        kp_pos_lateral=0.0,
        kp_pos_vertical=0.0,
        kd_vel_lateral=0.0,
        kd_vel_vertical=0.0,
        kp_att=np.array([10.0, 10.0, 10.0]),
        kd_att=np.array([5.0, 5.0, 5.0]),
    )

    current_state = np.zeros(6)
    target_state = np.zeros(6)
    mass = 1000.0
    up = np.array([0.0, 0.0, 1.0])

    # Rotate 45 degrees around X axis: q = [sin(22.5), 0, 0, cos(22.5)]
    theta_deg = 45.0
    theta_rad = np.radians(theta_deg)
    current_attitude = np.array([np.sin(theta_rad / 2), 0.0, 0.0, np.cos(theta_rad / 2)])

    # Legacy PD (no inertia tensor): torque = Kp * err - Kd * ang_vel
    wrench = controller.compute_wrench(
        current_state, current_attitude, mass, target_state, up,
        dt=0.02, angular_velocity=np.zeros(3),
    )

    assert np.allclose(wrench[:3], [0.0, 0.0, 0.0])

    # For 45deg pitch (X rotation), target_up_body = R_inv * [0,0,1]
    # R_x(-45): [0, sin(45), cos(45)] = [0, 0.707, 0.707]
    # err_axis = cross([0,1,0], [0, 0.707, 0.707]) = [0.707, 0, 0]
    # torque_x = 10.0 * 0.707 = 7.07
    expected_err_x = np.sin(theta_rad)  # sin(45) = 0.707
    expected_torque_x = 10.0 * expected_err_x
    assert np.isclose(wrench[3], expected_torque_x, atol=1e-5)
    assert np.allclose(wrench[4:], [0.0, 0.0])


def test_gravity_compensation():
    """Test that feed-forward gravity compensation works."""
    controller = GuidanceController(
        kp_pos_lateral=1.0,
        kp_pos_vertical=1.0,
        kd_vel_lateral=2.0,
        kd_vel_vertical=2.0,
        kp_att=np.array([0.0, 0.0, 0.0]),
        kd_att=np.array([0.0, 0.0, 0.0]),
        gravity=np.array([0.0, 0.0, -9.81]),
    )

    # Identity quaternion [x,y,z,w] = [0,0,0,1]
    current_state = np.zeros(6)
    target_state = np.zeros(6)
    current_attitude = np.array([0.0, 0.0, 0.0, 1.0])
    mass = 1000.0
    up = np.array([0.0, 0.0, 1.0])

    wrench = controller.compute_wrench(current_state, current_attitude, mass, target_state, up)

    # -gravity = -[0,0,-9.81] = [0,0,9.81] (upward)
    # With identity quaternion: a_body = a_world = [0,0,9.81]
    # force = 1000 * [0,0,9.81] = [0,0,9810]
    assert np.allclose(wrench[:3], [0.0, 0.0, 9810.0])


def test_rotation_to_body_frame():
    """Test that world-frame acceleration commands are properly rotated to the body frame."""
    controller = GuidanceController(
        kp_pos_lateral=1.0,
        kp_pos_vertical=1.0,
        kd_vel_lateral=2.0,
        kd_vel_vertical=2.0,
        kp_att=np.array([0.0, 0.0, 0.0]),
        kd_att=np.array([0.0, 0.0, 0.0]),
    )

    current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target_state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # 90 deg around Z axis via [x,y,z,w]: q = [0, 0, sin(45°), cos(45°)]
    # For 90° Z rotation: body X → world Y, body Y → world -X
    current_attitude = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    mass = 1000.0
    up = np.array([0.0, 0.0, 1.0])

    wrench = controller.compute_wrench(current_state, current_attitude, mass, target_state, up)

    # World force is [10000, 0, 0]; rotated to body: world +X ≈ body -Y
    assert np.allclose(wrench[:3], [0.0, -10000.0, 0.0], atol=1e-5)
    assert np.allclose(wrench[3:], [0.0, 0.0, 0.0])


def test_reset():
    """Test that reset() does not throw and clears ADRC state if present."""
    controller = GuidanceController(
        kp_pos_lateral=0.0,
        kp_pos_vertical=0.0,
        kd_vel_lateral=0.0,
        kd_vel_vertical=0.0,
        kp_att=np.array([10.0, 10.0, 10.0]),
        kd_att=np.array([5.0, 5.0, 5.0]),
    )
    # Just assert reset does not throw
    controller.reset()


def test_controller_no_adrc_backward_compat():
    """Test that controller without ADRC still works."""
    c1 = GuidanceController(
        kp_pos_lateral=1.0, kp_pos_vertical=1.0,
        kd_vel_lateral=2.0, kd_vel_vertical=2.0,
        kp_att=np.array([10.0, 10.0, 10.0]),
        kd_att=np.array([5.0, 5.0, 5.0]),
    )
    c2 = GuidanceController(
        kp_pos_lateral=1.0, kp_pos_vertical=1.0,
        kd_vel_lateral=2.0, kd_vel_vertical=2.0,
        kp_att=np.array([10.0, 10.0, 10.0]),
        kd_att=np.array([5.0, 5.0, 5.0]),
        adrc=None,
    )
    state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    att = np.array([0.0, 0.0, 0.0, 1.0])
    up = np.array([0.0, 0.0, 1.0])
    w1 = c1.compute_wrench(state, att, 1000.0, np.zeros(6), up)
    w2 = c2.compute_wrench(state, att, 1000.0, np.zeros(6), up)
    assert np.allclose(w1, w2)


def test_controller_with_adrc():
    """Test controller with ADRC produces valid wrench."""
    from src.guidance.adrc import ADRCController
    adrc = ADRCController(kp=np.array([10.0, 10.0, 10.0]),
                          kd=np.array([5.0, 5.0, 5.0]))
    controller = GuidanceController(
        kp_pos_lateral=1.0, kp_pos_vertical=1.0,
        kd_vel_lateral=2.0, kd_vel_vertical=2.0,
        kp_att=np.array([10.0, 10.0, 10.0]),
        kd_att=np.array([5.0, 5.0, 5.0]),
        adrc=adrc,
    )
    wrench = controller.compute_wrench(
        current_state=np.zeros(6),
        current_attitude=np.array([0.0, 0.0, 0.0, 1.0]),
        mass=1000.0,
        target_state=np.zeros(6),
        up_vector=np.array([0.0, 0.0, 1.0]),
        dt=0.02,
        angular_velocity=np.zeros(3),
    )
    assert wrench.shape == (6,)
    assert np.all(np.isfinite(wrench))
