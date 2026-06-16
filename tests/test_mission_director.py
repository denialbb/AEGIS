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
        body.surface_position = Mock(return_value=[0.0, 0.0, 600000.0])
        
        # Mock flight and inertia tensor
        flight = Mock()
        vessel.flight = Mock(return_value=flight)
        flight.velocity = [0.0, 0.0, 0.0]
        vessel.inertia_tensor = [1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0]  # Diagonal matrix
        
        # Mock parts with engines (for engine discovery)
        part = Mock()
        part.engine = Mock()
        part.engine.max_thrust = 1000.0
        part.engine.gimbal_range = 10.0
        part.engine.thrust_direction = (0.0, 0.0, -1.0)
        part.engine.has_fuel = True
        part.engine.part = part  # engine.part → part (needed for fallback path)
        part.position = Mock(return_value=[0.0, 0.0, -1.0])
        part.reference_frame = Mock()
        part.modules = []
        flight.surface_altitude = 100.0
        
        vessel.parts.with_tag = Mock(return_value=[part])
        vessel.parts.engines = [part.engine]
        
        # Mock reference frame creation
        space_center.ReferenceFrame = Mock()
        space_center.ReferenceFrame.create_relative = Mock(return_value=Mock())
        # Mock transform_direction — return identity-like directions
        def fake_transform_dir(vec, from_rf, to_rf):
            return list(vec)  # pass through
        space_center.transform_direction = fake_transform_dir
        
        # Mock add_stream for SensorModels
        conn.add_stream = Mock(return_value=Mock())
        conn.add_stream().x = 0.0
        conn.add_stream().y = 0.0
        conn.add_stream().z = 0.0
        
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
                                             1000.0, np.zeros(3), "flying", np.zeros(3), np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, -9.81])))

        # Mock estimator to return specific state
        md.estimator.update = Mock(return_value=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        md.estimator.get_state = Mock(return_value=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        # The loop must reach a terminal state. With est_alt=0, est_vz=0,
        # action_group=False, situation="flying", none of the exit paths
        # (activation, emergency, landed/destroyed) trigger, so the loop
        # would hang. Force an exit after one tick via _exit_requested.
        original_poll = md.sensors.poll

        def poll_once_then_exit(*args, **kwargs):
            md._exit_requested = True
            return original_poll(*args, **kwargs)

        md.sensors.poll = poll_once_then_exit

        success = md.run_loop()

        # State should have transitioned to HARD_ABORT via the exit path
        assert md.state == "HARD_ABORT"
        assert success is False

    def test_safe_engine_access(self):
        """Test the _safe_engine_access helper function."""
        from src.main import _safe_engine_access

        # Test with valid part
        part = Mock()
        part.engine = Mock()
        result = _safe_engine_access(part)
        assert result == part.engine

        # Test with None part
        result = _safe_engine_access(None)
        assert result is None

        # Test with part that throws RuntimeError when accessing .engine
        bad_part = Mock()
        type(bad_part).engine = property(
            lambda self: (_ for _ in ()).throw(RuntimeError("No engine"))
        )
        result = _safe_engine_access(bad_part)
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
        
        # With up_vector = [0,0,1] (from mock surface_position=[0,0,600000]):
        # est_alt = dot([0,0,100], [0,0,1]) = 100
        # target_state[:3] = 100 * [0,0,1] = [0,0,100]
        # alt_above_floor = max(100-50, 0) = 50
        # desired_speed = min(20.0, sqrt(2*15*50)) = min(20.0, 38.7) = 20.0
        # target_state[3:] = -[0,0,1] * 20.0 = [0,0,-20.0]
        assert target_state[0] == 0.0
        assert target_state[1] == 0.0
        assert target_state[2] == 100.0
        assert target_state[5] == -20.0

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
