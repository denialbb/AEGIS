import numpy as np
from src.guidance.controller import GuidanceController


def test_controller_initialization_with_inertia():
    controller = GuidanceController(
        kp_pos_lateral=1.0, kp_pos_vertical=1.0,
        kd_vel_lateral=10.0, kd_vel_vertical=40.0,
        kp_att=np.array([9.0, 9.0, 9.0]),
        kd_att=np.array([6.0, 6.0, 6.0]),
        gravity_ned=np.array([0.0, 0.0, -9.81]),
        inertia_tensor=np.eye(3)
    )
    assert controller.inertia_tensor is not None
    assert controller.inertia_tensor.shape == (3, 3)


def test_controller_without_inertia():
    controller = GuidanceController(
        kp_pos_lateral=1.0, kp_pos_vertical=1.0,
        kd_vel_lateral=10.0, kd_vel_vertical=40.0,
        kp_att=np.array([10.0, 10.0, 10.0]),
        kd_att=np.array([20.0, 20.0, 20.0]),
        gravity_ned=np.array([0.0, 0.0, -9.81]),
    )
    assert controller.inertia_tensor is None


def test_controller_inertia_torque_computation():
    controller = GuidanceController(
        kp_pos_lateral=1.0, kp_pos_vertical=1.0,
        kd_vel_lateral=10.0, kd_vel_vertical=40.0,
        kp_att=np.array([9.0, 9.0, 9.0]),
        kd_att=np.array([6.0, 6.0, 6.0]),
        gravity_ned=np.array([0.0, 0.0, -9.81]),
        inertia_tensor=np.diag([1000.0, 500.0, 800.0])
    )

    current_state = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
    attitude = np.array([0.0, 0.0, 0.0, 1.0])
    mass = 5000.0
    target_state = np.array([0.0, 0.0, 100.0, 0.0, 0.0, -10.0])
    up_vector = np.array([0.0, 0.0, 1.0])
    dt = 0.02
    angular_velocity = np.array([0.01, 0.02, 0.005])

    wrench = controller.compute_wrench(
        current_state=current_state,
        current_attitude=attitude,
        mass=mass,
        target_state=target_state,
        up_vector=up_vector,
        dt=dt,
        angular_velocity=angular_velocity
    )

    assert wrench.shape == (6,)
    # With identity attitude and downward velocity target -10 m/s:
    # vel_err_z = -10, kd_vel_vertical = 40 => a_cmd_vel_z = -400
    # a_cmd_world = a_cmd_vel - gravity = [0,0,-400] - [0,0,-9.81] = [0,0,-390.19]
    # force = mass * a_cmd_world = 5000 * [0,0,-390.19] = [0,0,-1950950]
    expected_force_z = 5000.0 * (-40.0 * 10.0 + 9.81)
    np.testing.assert_allclose(wrench[:3], np.array([0.0, 0.0, expected_force_z]), atol=1.0)
    assert wrench.shape == (6,)


def test_controller_gyroscopic_term():
    controller = GuidanceController(
        kp_pos_lateral=1.0, kp_pos_vertical=1.0,
        kd_vel_lateral=10.0, kd_vel_vertical=40.0,
        kp_att=np.array([9.0, 9.0, 9.0]),
        kd_att=np.array([6.0, 6.0, 6.0]),
        gravity_ned=np.array([0.0, 0.0, -9.81]),
        inertia_tensor=np.diag([1000.0, 500.0, 800.0]),
        max_torque=np.array([3200.0, 3200.0, 3200.0])
    )

    # Match target to current so pos_err and vel_err are zero -> a_cmd = -gravity
    # Use attitude that points nose (+Y) toward up (+Z): 90 deg about X.
    # In this orientation, body +Y aligns with world +Z, so err_axis = 0.
    # Torque should then be: -J*Kd*omega + omega x J*omega
    q_nose_up = np.array([np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4)])
    current_state = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
    attitude = q_nose_up
    mass = 5000.0
    target_state = current_state.copy()
    up_vector = np.array([0.0, 0.0, 1.0])
    dt = 0.02

    angular_velocity = np.array([0.5, 0.3, 0.1])

    wrench = controller.compute_wrench(
        current_state=current_state,
        current_attitude=attitude,
        mass=mass,
        target_state=target_state,
        up_vector=up_vector,
        dt=dt,
        angular_velocity=angular_velocity
    )

    # With zero pos/vel error, a_cmd_world = -gravity = [0,0,9.81]
    # target_up_world = normalize([0,0,9.81]) = [0,0,1] = up_vector
    # In body frame with nose-up attitude: target_up_body = [0,1,0]
    # err_axis = cross([0,1,0], [0,1,0]) = [0,0,0]
    # torque = -J*Kd*omega + omega x (J*omega)
    J = np.diag([1000.0, 500.0, 800.0])
    tau_pd = -(J @ np.diag([6.0, 6.0, 6.0])) @ angular_velocity
    h = J @ angular_velocity
    tau_gyro = np.cross(angular_velocity, h)
    expected_torque = tau_pd + tau_gyro
    np.testing.assert_allclose(wrench[3:], expected_torque, atol=1e-10)


def test_controller_requires_angular_velocity_with_inertia():
    controller = GuidanceController(
        kp_pos_lateral=1.0, kp_pos_vertical=1.0,
        kd_vel_lateral=10.0, kd_vel_vertical=40.0,
        kp_att=np.array([9.0, 9.0, 9.0]),
        kd_att=np.array([6.0, 6.0, 6.0]),
        gravity_ned=np.array([0.0, 0.0, -9.81]),
        inertia_tensor=np.eye(3)
    )

    current_state = np.zeros(6)
    attitude = np.array([0.0, 0.0, 0.0, 1.0])
    target_state = np.zeros(6)
    up_vector = np.array([0.0, 0.0, 1.0])

    import pytest
    with pytest.raises(ValueError, match="angular_velocity is required"):
        controller.compute_wrench(
            current_state=current_state,
            current_attitude=attitude,
            mass=5000.0,
            target_state=target_state,
            up_vector=up_vector,
            dt=0.02,
            angular_velocity=None
        )


def test_invalid_inertia_tensor_shape():
    import pytest
    with pytest.raises(ValueError, match="inertia_tensor must have shape"):
        GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=10.0, kd_vel_vertical=40.0,
            kp_att=np.array([9.0, 9.0, 9.0]),
            kd_att=np.array([6.0, 6.0, 6.0]),
            gravity_ned=np.array([0.0, 0.0, -9.81]),
            inertia_tensor=np.eye(4)
        )
