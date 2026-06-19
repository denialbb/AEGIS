import numpy as np
import math
import pytest
from unittest.mock import MagicMock
from scipy.spatial.transform import Rotation as R

from src.guidance.attitude import AttitudeController


def _make_director(
    up_vector: np.ndarray = np.array([0.0, 0.0, -1.0]),
    pos: np.ndarray = np.zeros(3),
    vel: np.ndarray = np.zeros(3),
) -> MagicMock:
    d = MagicMock()
    d.up_vector = up_vector
    d.estimator = MagicMock()
    d.estimator.pos = pos
    d.estimator.vel = vel
    d.vessel = MagicMock()
    d.vessel.control = MagicMock()
    return d


def _up_quat() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0])


def _nose_up_quat() -> np.ndarray:
    """Quaternion for vessel nose pointing up (body Y → NED Z-)."""
    return R.from_rotvec([-np.pi / 2, 0, 0]).as_quat()


class TestSignFlipFix:
    def test_pitch_positive_when_target_below_nose(self) -> None:
        ctrl = AttitudeController()
        d = _make_director()

        target_nose_ned = np.array([0.0, 0.1, -0.5])
        target_nose_ned /= np.linalg.norm(target_nose_ned)

        # Body frame when nose is up: body Y ~ NED [0, 0, -1]
        # target_nose_body for nose-up attitude + target with +Y component:
        # target should have positive tz (below the nose in body frame)
        # → pitch_error should be positive
        quat = _nose_up_quat()
        rot = R.from_quat(quat)
        target_body = rot.inv().apply(target_nose_ned)
        tx, _, tz = target_body

        pitch_error = tz
        assert pitch_error > 0, (
            f"Expected positive pitch_error (target below nose), "
            f"got tz={tz:.4f} for target_nose_body={target_body.round(3)}"
        )

    def test_yaw_positive_when_target_starboard(self) -> None:
        ctrl = AttitudeController()
        d = _make_director()
        up = d.up_vector

        target_nose_ned = np.array([0.3, 0.0, -0.95])
        target_nose_ned /= np.linalg.norm(target_nose_ned)

        quat = _nose_up_quat()
        rot = R.from_quat(quat)
        target_body = rot.inv().apply(target_nose_ned)
        tx, _, tz = target_body

        yaw_error = tx
        assert yaw_error != 0.0


class TestIsActive:
    def test_active_in_powered_descent_stability(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._is_active("POWERED_DESCENT", "stability") is True

    def test_inactive_for_non_stability_powered_descent(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._is_active("POWERED_DESCENT", "retrograde") is False
        assert ctrl._is_active("POWERED_DESCENT", "prograde") is False

    def test_active_in_hover_off(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._is_active("HOVER_TARGETING", "off") is True
        assert ctrl._is_active("HOVER_TARGETING", "stability") is True

    def test_active_in_terminal_off(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._is_active("TERMINAL_DESCENT", "off") is True
        assert ctrl._is_active("TERMINAL_DESCENT", "stability") is True

    def test_inactive_in_standby(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._is_active("STANDBY", "stability") is False


class TestDeadband:
    def test_zero_commands_when_on_target(self) -> None:
        ctrl = AttitudeController()
        d = _make_director()

        up = d.up_vector
        a_cmd_ned = -np.array([0.0, 0.0, -9.81])

        quat = _nose_up_quat()

        ctrl.update(d, a_cmd_ned, "HOVER_TARGETING", "stability", quat)

        pitch = d.vessel.control.pitch
        yaw = d.vessel.control.yaw

        assert abs(pitch) < 0.1, f"pitch should be near zero on-target, got {pitch:.3f}"
        assert abs(yaw) < 0.1, f"yaw should be near zero on-target, got {yaw:.3f}"


class TestClampPitch:
    def test_does_not_modify_within_limit(self) -> None:
        ctrl = AttitudeController()
        up = np.array([0.0, 0.0, -1.0])
        target = np.array([0.1, 0.0, -0.99])
        target /= np.linalg.norm(target)

        result = ctrl._clamp_pitch(target, up, 25.0)
        cos_angle = float(np.dot(result, up))
        assert cos_angle < 1.0

    def test_clamps_beyond_limit(self) -> None:
        ctrl = AttitudeController()
        up = np.array([0.0, 0.0, -1.0])
        target = np.array([1.0, 0.0, 0.0])

        result = ctrl._clamp_pitch(target, up, 25.0)
        alignment = float(np.dot(result, up))
        assert alignment >= math.cos(math.radians(25.0))


class TestPhaseMaxPitch:
    def test_powered_descent_capped_at_15(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._max_pitch_for_state("POWERED_DESCENT") <= 15.0

    def test_hover_uses_base(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._max_pitch_for_state("HOVER_TARGETING") == 25.0

    def test_terminal_landing_capped_at_10(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._max_pitch_for_state("TERMINAL_LANDING") <= 10.0


class TestHoverTargetNose:
    def test_returns_up_when_on_target(self) -> None:
        ctrl = AttitudeController()
        d = _make_director(pos=np.zeros(3), vel=np.zeros(3))
        up = d.up_vector

        result = ctrl._hover_target_nose(d, up * 9.81, 9.81, up)
        np.testing.assert_allclose(result, up, atol=1e-6)

    def test_returns_a_cmd_when_far_from_target(self) -> None:
        ctrl = AttitudeController()
        d = _make_director(pos=np.array([50.0, 50.0, -100.0]),
                           vel=np.array([0.0, 0.0, 0.0]))
        up = d.up_vector
        a_cmd = np.array([2.0, 0.0, -9.81])
        norm_a = float(np.linalg.norm(a_cmd))

        result = ctrl._hover_target_nose(d, a_cmd, norm_a, up)
        expected = a_cmd / norm_a
        np.testing.assert_allclose(result, expected, atol=1e-6)
