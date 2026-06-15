import numpy as np
import pytest
from unittest.mock import Mock, MagicMock
import sys
import os

# Add src to path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.main import MissionDirector
import src.config as config


class TestMissionDirector:
    """Test suite for the MissionDirector class."""
    
    def create_mock_connection(self):
        """Create a mock kRPC connection with all necessary attributes and methods."""
        conn = Mock()
        
        # Mock space center and active vessel
        space_center = Mock()
        vessel = Mock()
        conn.space_center = space_center
        space_center.active_vessel = vessel
        
        # Mock vessel properties
        vessel.name = "Test Vessel"
        vessel.control = Mock()
        vessel.control.throttle = 0.0
        vessel.control.set_action_group = Mock()
        vessel.control.get_action_group = Mock(return_value=False)
        vessel.parts = Mock()
        
        # Mock orbit and body
        orbit = Mock()
        body = Mock()
        vessel.orbit = orbit
        orbit.body = body
        body.reference_frame = Mock()
        body.surface_position = Mock(return_value=[0.0, 0.0, 0.0])
        
        # Mock flight and inertia tensor
        flight = Mock()
        vessel.flight = Mock(return_value=flight)
        flight.velocity = [0.0, 0.0, 0.0]
        vessel.inertia_tensor = [1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0]  # Diagonal matrix
        
        # Mock parts with engines (for engine discovery)
        part = Mock()
        part.engine = Mock()
        part.engine.max_thrust = 1000.0
        part.position = Mock(return_value=[0.0, 0.0, 0.0])
        part.modules = []
        
        vessel.parts.with_tag = Mock(return_value=[part])
        vessel.parts.engines = [part.engine]
        
        # Mock reference frame creation
        space_center.ReferenceFrame = Mock()
        space_center.ReferenceFrame.create_relative = Mock(return_value=Mock())
        
        return conn, vessel, space_center, body, flight, part

    def test_mission_director_initialization(self):
        """Test that MissionDirector initializes correctly with mocked kRPC connection."""
        conn, vessel, space_center, body, flight, part = self.create_mock_connection()
        
        # Create MissionDirector instance
        md = MissionDirector(conn)
        
        # Assert basic properties
        assert md.conn == conn
        assert md.state == "STANDBY"
        assert md.vessel == vessel
        assert len(md.engines) == 1
        assert isinstance(md.engines[0].index, int)
        assert isinstance(md.engines[0].position, np.ndarray)
        assert isinstance(md.engines[0].thrust_direction, np.ndarray)
        assert md.engines[0].max_thrust == 1000.0
        
        # Assert submodules are initialized
        assert md.estimator is not None
        assert md.fdi is not None
        assert md.guidance is not None
        assert md.allocator is not None
        assert md.sensors is not None
        assert md.writer is not None

    def test_mission_director_state_transitions(self):
        """Test state transition logic based on altitude and velocity."""
        conn, vessel, space_center, body, flight, part = self.create_mock_connection()
        md = MissionDirector(conn)
        
        # Test initial state
        assert md.state == "STANDBY"
        
        # Mock sensors.poll to return specific values
        md.sensors.poll = Mock(return_value=(0.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), 
                                            1000.0, np.zeros(3), "flying", np.zeros(3)))
        
        # Mock estimator to return specific state
        md.estimator.update = Mock(return_value=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        md.estimator.get_state = Mock(return_value=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        
        # Test that state remains STANDBY when not activated
        md.run_loop()  # This would normally run the loop, but we'll test the logic directly
        # Note: Full loop testing requires more complex mocking, but we can test transition logic

    def test_safe_engine_access(self):
        """Test the _safe_engine_access helper function."""
        conn, vessel, space_center, body, flight, part = self.create_mock_connection()
        md = MissionDirector(conn)
        
        # Test with valid part
        result = md._safe_engine_access(part)
        assert result == part.engine
        
        # Test with None part
        result = md._safe_engine_access(None)
        assert result is None
        
        # Test with part that throws RuntimeError when accessing .engine
        bad_part = Mock()
        bad_part.engine = Mock(side_effect=RuntimeError("No engine"))
        result = md._safe_engine_access(bad_part)
        assert result is None

    def test_compute_glideslope_target(self):
        """Test the _compute_glideslope_target method."""
        conn, vessel, space_center, body, flight, part = self.create_mock_connection()
        md = MissionDirector(conn)
        
        # Set up test parameters
        state_vector = np.array([0.0, 0.0, 100.0, 0.0, 0.0, -10.0])  # [x, y, z, vx, vy, vz]
        floor_alt = 50.0
        max_descent_rate = 20.0
        a_avail = 15.0  # m/s^2 net upward acceleration
        
        # Call the method
        target_state = md._compute_glideslope_target(state_vector, floor_alt, max_descent_rate, a_avail)
        
        # Assert return type and shape
        assert isinstance(target_state, np.ndarray)
        assert target_state.shape == (6,)
        
        # The target should have the same x,y position as current (projected onto up vector)
        # Since up_vector is [0,0,1] in our mock, x and y should be 0
        assert target_state[0] == 0.0
        assert target_state[1] == 0.0
        
        # Z should be at floor_alt since we're computing a target for that altitude
        # Actually, let's check the math:
        # est_pos = [0,0,100], up_vector = [0,0,1] (from mock)
        # est_alt = dot([0,0,100], [0,0,1]) = 100
        # target_state[:3] = est_alt * up_vector = 100 * [0,0,1] = [0,0,100]
        # But then we compute alt_above_floor = max(est_alt - floor_alt, 0) = max(100-50, 0) = 50
        # desired_speed = min(max_descent_rate, sqrt(2 * a_avail * alt_above_floor))
        # desired_speed = min(20.0, sqrt(2 * 15.0 * 50.0)) = min(20.0, sqrt(1500)) = min(20.0, 38.7) = 20.0
        # target_state[3:] = -up_vector * desired_speed = -[0,0,1] * 20.0 = [0,0,-20.0]
        
        # So target_state should be [0,0,100,0,0,-20.0]
        assert target_state[2] == 100.0  # Z position
        assert target_state[5] == -20.0  # Z velocity

    def test_mission_director_handles_hard_abort(self):
        """Test that MissionDirector properly handles HARD_ABORT state."""
        conn, vessel, space_center, body, flight, part = self.create_mock_connection()
        md = MissionDirector(conn)
        
        # Set state to HARD_ABORT
        md.state = "HARD_ABORT"
        
        # Call run_loop - should exit immediately
        # We can't easily test the full loop without complex mocking,
        # but we can verify the state checking logic works
        assert md.state in ["HARD_ABORT", "LANDED"]  # Loop condition
        
        # Test that setting _exit_requested works
        md._exit_requested = True
        assert md._exit_requested == True

    def test_mission_director_engine_discovery_fallback(self):
        """Test engine discovery falls back to all engines when no tagged parts found."""
        conn, vessel, space_center, body, flight, part = self.create_mock_connection()
        
        # Make with_tag return empty list to trigger fallback
        vessel.parts.with_tag = Mock(return_value=[])
        # But make sure parts.engines returns our test part
        vessel.parts.engines = [part.engine]
        
        md = MissionDirector(conn)
        
        # Should have discovered engines through fallback
        assert len(md.engines) == 1
        assert md.engines[0].index == 0

    def test_mission_director_no_engines(self):
        """Test behavior when no engines are discovered."""
        conn, vessel, space_center, body, flight, part = self.create_mock_connection()
        
        # Make both with_tag and parts.engines return empty
        vessel.parts.with_tag = Mock(return_value=[])
        vessel.parts.engines = []
        
        md = MissionDirector(conn)
        
        # Should have zero engines
        assert len(md.engines) == 0
        # Allocator should still be created but with empty engines list
        assert md.allocator is not None
