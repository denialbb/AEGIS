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
        
        # torque_scale=200.0, max_trim=0.15:
        # pitch = clip(10.0/200, ±0.15) = 0.05
        # yaw   = clip(-30.0/200, ±0.15) = -0.15
        # roll  = clip(-(-20.0)/200, ±0.15) = 0.10
        # smoothing alpha=0.6 → first tick is 0.6 * cmd

        expected_pitch = 0.6 * 0.05
        expected_yaw = 0.6 * -0.15
        expected_roll = 0.6 * 0.10

        assert np.isclose(d.vessel.control.pitch, expected_pitch)
        assert np.isclose(d.vessel.control.yaw, expected_yaw)
        assert np.isclose(d.vessel.control.roll, expected_roll)

    def test_zeroes_controls_when_inactive(self) -> None:
        ctrl = AttitudeController()
        d = _make_director()
        
        torque_body = np.array([100.0, 100.0, 100.0])
        ctrl.update(d, torque_body, "STANDBY", "stability")
        
        assert d.vessel.control.pitch == 0.0
        assert d.vessel.control.yaw == 0.0
        assert d.vessel.control.roll == 0.0
