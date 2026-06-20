import numpy as np
import pytest
from unittest.mock import MagicMock

from src.guidance.attitude import AttitudeController


def _make_director() -> MagicMock:
    d = MagicMock()
    d.vessel = MagicMock()
    d.vessel.control = MagicMock()
    d.vessel.control.pitch = 0.0
    d.vessel.control.yaw = 0.0
    d.vessel.control.roll = 0.0
    return d


class TestIsActive:
    def test_active_in_powered_descent_stability(self) -> None:
        ctrl = AttitudeController()
        # POWERED_DESCENT is not active for trim anymore in the new unified controller
        # Wait, the unified controller is active in HOVER, TERMINAL_DESCENT, TERMINAL_LANDING
        assert ctrl._is_active("POWERED_DESCENT", "stability") is False

    def test_active_in_hover_off_or_stability(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._is_active("HOVER_TARGETING", "off") is True
        assert ctrl._is_active("HOVER_TARGETING", "stability") is True

    def test_active_in_terminal(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._is_active("TERMINAL_DESCENT", "off") is True
        assert ctrl._is_active("TERMINAL_DESCENT", "stability") is True

    def test_inactive_in_standby(self) -> None:
        ctrl = AttitudeController()
        assert ctrl._is_active("STANDBY", "stability") is False


class TestAttitudeMapping:
    def test_maps_torque_to_joystick(self) -> None:
        ctrl = AttitudeController()
        d = _make_director()
        
        # Test torque: [10.0, -20.0, 30.0]
        torque_body = np.array([10.0, -20.0, 30.0])
        ctrl.update(d, torque_body, "HOVER_TARGETING", "stability")
        
        # We mapped pitch = torque_body[0]/100.0 = 0.1
        # yaw = -torque_body[2]/100.0 = -0.3 -> clipped to -0.15
        # roll = torque_body[1]/100.0 = -0.2 -> clipped to -0.15
        # plus smoothing alpha = 0.6 -> first tick is 0.6 * cmd
        
        expected_pitch = 0.6 * (10.0 / 100.0)
        expected_yaw = 0.6 * (-30.0 / 100.0)
        expected_yaw_clipped = 0.6 * -0.15
        expected_roll = 0.6 * (-20.0 / 100.0)
        expected_roll_clipped = 0.6 * -0.15
        
        assert np.isclose(d.vessel.control.pitch, expected_pitch)
        assert np.isclose(d.vessel.control.yaw, expected_yaw_clipped)
        assert np.isclose(d.vessel.control.roll, expected_roll_clipped)

    def test_zeroes_controls_when_inactive(self) -> None:
        ctrl = AttitudeController()
        d = _make_director()
        
        torque_body = np.array([100.0, 100.0, 100.0])
        ctrl.update(d, torque_body, "STANDBY", "stability")
        
        assert d.vessel.control.pitch == 0.0
        assert d.vessel.control.yaw == 0.0
        assert d.vessel.control.roll == 0.0
