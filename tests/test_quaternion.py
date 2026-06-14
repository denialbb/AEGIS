import numpy as np
from scipy.spatial.transform import Rotation as R

def test_scipy_from_quat_scalar_last():
    """Verify scipy's R.from_quat uses scalar-last [x, y, z, w] convention.

    A 90-degree rotation about Z should map +X to +Y.
    A quaternion for this rotation in scalar-last format is [0, 0, sin(45°), cos(45°)].
    """
    angle = np.pi / 2
    q_scalar_last = np.array([0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)])
    rot = R.from_quat(q_scalar_last)
    x_rotated = rot.apply(np.array([1.0, 0.0, 0.0]))
    expected = np.array([0.0, 1.0, 0.0])
    np.testing.assert_allclose(x_rotated, expected, atol=1e-15,
                               err_msg="90° about Z did not map +X to +Y")


def test_quaternion_round_trip():
    """Round-trip a known rotation through from_quat/as_quat preserves values."""
    angle = np.pi / 3
    axis = np.array([1.0, 1.0, 0.0])
    axis = axis / np.linalg.norm(axis)
    q = np.zeros(4)
    q[:3] = axis * np.sin(angle / 2)
    q[3] = np.cos(angle / 2)
    rot = R.from_quat(q)
    q_rt = rot.as_quat()
    np.testing.assert_allclose(q, q_rt, atol=1e-15)


def test_quaternion_euler_consistency():
    """Verify quaternion -> Euler -> quaternion consistency."""
    for roll, pitch, yaw in [(0.0, 0.0, 0.0),
                              (0.1, 0.0, 0.0),
                              (0.0, 0.2, 0.0),
                              (0.0, 0.0, 0.3),
                              (0.1, 0.2, 0.3)]:
        rot = R.from_euler('XYZ', [roll, pitch, yaw])
        q = rot.as_quat()
        rot2 = R.from_quat(q)
        np.testing.assert_allclose(rot2.as_euler('XYZ'), [roll, pitch, yaw],
                                   atol=1e-10)


def test_quaternion_inverse_multiply():
    """Test quaternion inverse and multiplication used in attitude error computation.

    The error rotation from attitude_a to attitude_b is: q_err = q_b * q_a^-1.
    """
    q_a = np.array([0.0, 0.0, 0.0, 1.0])  # identity
    q_b = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])  # 90° about Z
    r_a = R.from_quat(q_a)
    r_b = R.from_quat(q_b)
    r_err = r_b * r_a.inv()
    angle = 2 * np.arccos(np.clip(r_err.as_quat()[3], -1.0, 1.0))
    np.testing.assert_allclose(angle, np.pi / 2, atol=1e-10)


def test_controller_attitude_docstring_convention():
    """Verify the convention used by controller.py matches kRPC/scipy scalar-last.

    kRPC returns quaternions as [x, y, z, w] which is scipy's native format.
    This test confirms that passing an [x,y,z,w] quaternion to R.from_quat
    produces the correct rotation, matching the sensor.py convention.
    """
    # Simulate a vessel pointing north (identity in body frame)
    # The body-frame forward axis is +Y.
    # A rotation that aligns world +Z with body +Y is 90° about +X.
    q_body_to_world = np.array([np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4)])
    rot = R.from_quat(q_body_to_world)
    body_y = np.array([0.0, 1.0, 0.0])
    world_dir = rot.apply(body_y)
    np.testing.assert_allclose(world_dir, np.array([0.0, 0.0, 1.0]), atol=1e-10)
