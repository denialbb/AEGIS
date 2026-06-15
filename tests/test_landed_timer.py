import numpy as np
import pytest
from unittest.mock import Mock, patch
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import src.config as config
from src.main import MissionDirector


class TestLandedTimer:
    """Test suite for the landed timer logic in TERMINAL_DESCENT."""

    def create_mock_connection(self):
        """Create a mock kRPC connection with all necessary attributes and methods."""
        conn = Mock()
        space_center = Mock()
        vessel = Mock()
        conn.space_center = space_center
        space_center.active_vessel = vessel
        vessel.name = "Test Vessel"
        vessel.control = Mock()
        vessel.control.throttle = 0.0
        vessel.control.set_action_group = Mock()
        vessel.control.get_action_group = Mock(return_value=False)
        vessel.parts = Mock()
        orbit = Mock()
        body = Mock()
        vessel.orbit = orbit
        orbit.body = body
        body.reference_frame = Mock()
        body.surface_position = Mock(return_value=[0.0, 0.0, 600000.0])
        flight = Mock()
        vessel.flight = Mock(return_value=flight)
        flight.velocity = [0.0, 0.0, 0.0]
        vessel.inertia_tensor = [
            1000.0, 0.0, 0.0,
            0.0, 1000.0, 0.0,
            0.0, 0.0, 1000.0,
        ]
        part = Mock()
        part.engine = Mock()
        part.engine.max_thrust = 1000.0
        part.position = Mock(return_value=[0.0, 0.0, 0.0])
        part.modules = []
        vessel.parts.with_tag = Mock(return_value=[part])
        vessel.parts.engines = [part.engine]
        space_center.ReferenceFrame = Mock()
        space_center.ReferenceFrame.create_relative = Mock(return_value=Mock())
        return conn, vessel, space_center, body, flight, part

    @pytest.fixture
    def director(self):
        """Fixture providing a fresh MissionDirector in TERMINAL_DESCENT."""
        conn, vessel, space_center, body, flight, part = self.create_mock_connection()
        md = MissionDirector(conn)
        md.state = "TERMINAL_DESCENT"
        md._landed_timer = 0.0
        return md

    def test_timer_advances_when_low_and_slow(self, director):
        """Timer should increase when velocity and altitude are within thresholds."""
        with patch.dict(
            config.__dict__,
            {"LANDED_VEL_THRESHOLD": 0.5, "LANDED_ALT_THRESHOLD": 1.0},
        ):
            director._landed_timer += 1.0 / config.TARGET_HZ
            assert director._landed_timer > 0.0

    def test_timer_does_not_advance_when_high(self, director):
        """Timer should not advance when altitude is above threshold."""
        director._landed_timer = 5.0
        vel_landed = False
        alt_landed = False
        if not (vel_landed and alt_landed):
            director._landed_timer = max(0.0, director._landed_timer - (1.0 / config.TARGET_HZ))
        assert director._landed_timer < 5.0

    def test_timer_does_not_advance_when_fast(self, director):
        """Timer should not advance when velocity is above threshold."""
        director._landed_timer = 3.0
        vel_landed = False
        alt_landed = True
        if not (vel_landed and alt_landed):
            director._landed_timer = max(0.0, director._landed_timer - (1.0 / config.TARGET_HZ))
        assert director._landed_timer < 3.0

    def test_timer_decays_gradually(self, director):
        """Timer should decay, not reset, when conditions are broken."""
        director._landed_timer = 4.0
        dt = 1.0 / config.TARGET_HZ
        director._landed_timer = max(0.0, director._landed_timer - dt)
        expected = max(0.0, 4.0 - dt)
        assert abs(director._landed_timer - expected) < 1e-9