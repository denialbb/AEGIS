import numpy as np
import pytest
import logging
from src.simulation.physics import DigitalTwin, PhysicsState
from src.simulation.environment import VacuumEnvironment
from src.simulation.vessel import VesselModel
from src.common.engine import Engine
from src.simulation.mock_vessel import SimpleTestVessel

def test_commanded_throttles_clamping_and_spooling(caplog):
    env = VacuumEnvironment(g=9.80665)
    vessel = SimpleTestVessel()
    
    initial_state = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -100.0]),
        vel=np.zeros(3),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.zeros(3),
        fuel_mass=10.0,
        throttles=np.array([0.0])
    )
    
    dt = DigitalTwin(env, vessel, initial_state)
    
    # Send out-of-bounds throttles: [1.5], which should clamp to [1.0]
    with caplog.at_level(logging.WARNING):
        state = dt.step(commanded_throttles=np.array([1.5]), commanded_gimbals=np.array([[0.0, 0.0]]), dt=0.01)
        
    # Verify that a warning was logged
    assert any("Commanded throttles out of physical bounds" in record.message for record in caplog.records)
    
    # Verify that actual throttles spooled up (since target clamped to 1.0)
    assert state.throttles[0] > 0.0
    assert state.throttles[0] < 1.0
    assert state.time == 0.01

def test_engine_failure_spooldown():
    env = VacuumEnvironment(g=9.80665)
    
    # 2 engine vessel
    class TwoEngineVessel(SimpleTestVessel):
        def __init__(self):
            super().__init__()
            self._engines = [
                Engine(index=0, position=np.array([1.0, 0.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=100.0),
                Engine(index=1, position=np.array([-1.0, 0.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=100.0)
            ]
            
    vessel = TwoEngineVessel()
    initial_state = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -100.0]),
        vel=np.zeros(3),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.zeros(3),
        fuel_mass=10.0,
        throttles=np.array([0.5, 0.5])
    )
    
    dt = DigitalTwin(env, vessel, initial_state)
    dt.kill_engine(0)
    
    state = dt.step(commanded_throttles=np.array([1.0, 1.0]), commanded_gimbals=np.zeros((2, 2)), dt=0.05)
    
    # Engine 0 should spool down from 0.5 towards 0.0
    assert state.throttles[0] < 0.5
    # Engine 1 should spool up from 0.5 towards 1.0
    assert state.throttles[1] > 0.5

def test_fuel_consumption_and_exhaustion():
    env = VacuumEnvironment(g=9.80665)
    vessel = SimpleTestVessel()
    
    # Fuel burn rate is 1.0 * throttle. So at throttle=1.0, fuel burns at 1.0 kg/s.
    initial_state = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -100.0]),
        vel=np.zeros(3),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.zeros(3),
        fuel_mass=1.0,  # Only 1.0 kg of fuel
        throttles=np.array([1.0])
    )
    
    dt = DigitalTwin(env, vessel, initial_state)
    
    # Step forward by 0.5 seconds at full throttle command
    state = dt.step(commanded_throttles=np.array([1.0]), commanded_gimbals=np.zeros((1, 2)), dt=0.5)
    
    # Fuel should have depleted
    assert state.fuel_mass < 1.0
    assert state.fuel_mass > 0.0
    
    # Total mass should match dry mass (10.0) + remaining fuel mass
    expected_mass = vessel.total_mass(state.fuel_mass)
    assert np.isclose(expected_mass, 10.0 + state.fuel_mass)
    
    # Step forward by 2.0 seconds to completely exhaust fuel
    state = dt.step(commanded_throttles=np.array([1.0]), commanded_gimbals=np.zeros((1, 2)), dt=2.0)
    
    # Fuel should be exactly 0.0, not negative
    assert state.fuel_mass == 0.0
    
    # Check that acceleration is now only due to gravity (since fuel is exhausted)
    # The derivatives computed at fuel_mass=0 should have 0 thrust force.
    derivs = dt._compute_derivatives(state, cmd_throttles=np.array([1.0]), cmd_gimbals=np.zeros((1, 2)))
    # Since gravity is 9.80665 downwards (NED Z), acceleration should be purely 9.80665 downwards
    np.testing.assert_allclose(derivs["vel"], np.array([0.0, 0.0, 9.80665]))

def test_translational_physics():
    env = VacuumEnvironment(g=9.80665)
    
    # Drag vessel
    class DragVessel(SimpleTestVessel):
        def get_drag_force(self, velocity_ned: np.ndarray, density: float) -> np.ndarray:
            return -0.5 * density * velocity_ned
            
    vessel_drag = DragVessel()
    
    # Dense environment
    class CustomEnv(VacuumEnvironment):
        def air_density(self, altitude: float) -> float:
            return 1.225
            
    env_dense = CustomEnv(g=9.80665)
    
    state_drag = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -100.0]),
        vel=np.array([10.0, 0.0, 0.0]),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.zeros(3),
        fuel_mass=10.0,
        throttles=np.array([0.0])
    )
    
    dt_drag = DigitalTwin(env_dense, vessel_drag, state_drag)
    derivs = dt_drag._compute_derivatives(state_drag, cmd_throttles=np.array([0.0]), cmd_gimbals=np.zeros((1, 2)))
    
    expected_accel = np.array([-0.30625, 0.0, 9.80665])
    np.testing.assert_allclose(derivs["vel"], expected_accel)
    
    # Test thrust force rotation
    q_90y = np.array([0.0, np.sin(np.pi / 4), 0.0, np.cos(np.pi / 4)])
    state_rot = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -100.0]),
        vel=np.zeros(3),
        q=q_90y,
        omega=np.zeros(3),
        fuel_mass=10.0,
        throttles=np.array([1.0])
    )
    
    env_zero_g = VacuumEnvironment(g=0.0)
    dt_rot = DigitalTwin(env_zero_g, vessel_drag, state_rot)
    derivs_rot = dt_rot._compute_derivatives(state_rot, cmd_throttles=np.array([1.0]), cmd_gimbals=np.zeros((1, 2)))
    
    expected_rot_accel = np.array([-5.0, 0.0, 0.0])
    np.testing.assert_allclose(derivs_rot["vel"], expected_rot_accel, atol=1e-12)

def test_rotational_physics():
    class OffcenterEngineVessel(SimpleTestVessel):
        def __init__(self):
            super().__init__()
            self._engines = [
                Engine(index=0, position=np.array([1.0, 0.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=100.0, max_gimbal_deg=5.0)
            ]
            
        def inertia_tensor(self, fuel_mass_kg: float) -> np.ndarray:
            return np.diag([1.0, 2.0, 3.0])
            
    vessel = OffcenterEngineVessel()
    env = VacuumEnvironment(g=0.0)
    
    state = PhysicsState(
        time=0.0,
        pos=np.zeros(3),
        vel=np.zeros(3),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.zeros(3),
        fuel_mass=10.0,
        throttles=np.array([1.0])
    )
    
    dt = DigitalTwin(env, vessel, state)
    
    derivs_no_gimbal = dt._compute_derivatives(state, cmd_throttles=np.array([1.0]), cmd_gimbals=np.zeros((1, 2)))
    np.testing.assert_allclose(derivs_no_gimbal["omega"], np.array([0.0, 50.0, 0.0]))
    
    max_rad = np.deg2rad(5.0)
    derivs_clamped = dt._compute_derivatives(state, cmd_throttles=np.array([1.0]), cmd_gimbals=np.array([[0.5, 0.0]]))
    derivs_exact = dt._compute_derivatives(state, cmd_throttles=np.array([1.0]), cmd_gimbals=np.array([[max_rad, 0.0]]))
    np.testing.assert_allclose(derivs_clamped["omega"], derivs_exact["omega"])
    
    state_spin = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -100.0]),
        vel=np.zeros(3),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.array([0.0, 1.0, 0.0]),
        fuel_mass=10.0,
        throttles=np.array([0.0])
    )
    
    dt_spin = DigitalTwin(env, vessel, state_spin)
    for _ in range(10):
        new_state = dt_spin.step(commanded_throttles=np.array([0.0]), commanded_gimbals=np.zeros((1, 2)), dt=0.02)
        
    np.testing.assert_allclose(np.linalg.norm(new_state.q), 1.0, rtol=1e-12)
    np.testing.assert_allclose(new_state.omega, np.array([0.0, 1.0, 0.0]))
    assert np.isclose(new_state.time, 0.2)

def test_ground_interaction():
    env = VacuumEnvironment(g=9.80665)
    vessel = SimpleTestVessel()
    
    initial_state = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -0.5]),
        vel=np.array([0.0, 0.0, 5.0]),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.array([0.1, 0.2, 0.3]),
        fuel_mass=10.0,
        throttles=np.array([0.0])
    )
    
    dt = DigitalTwin(env, vessel, initial_state)
    
    state = dt.step(commanded_throttles=np.array([0.0]), commanded_gimbals=np.zeros((1, 2)), dt=0.2)
    
    assert dt.landed is True
    assert state.pos[2] == 0.0
    np.testing.assert_allclose(state.vel, np.zeros(3))
    np.testing.assert_allclose(state.omega, np.zeros(3))
    
    state_after = dt.step(commanded_throttles=np.array([1.0]), commanded_gimbals=np.zeros((1, 2)), dt=1.0)
    assert dt.landed is True
    assert state_after.pos[2] == 0.0
    assert state_after.time == state.time

def test_determinism_and_equilibrium():
    env = VacuumEnvironment(g=9.80665)
    
    class HoverVessel(SimpleTestVessel):
        def __init__(self):
            super().__init__()
            self._engines = [
                Engine(index=0, position=np.array([1.0, 0.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=100.0),
                Engine(index=1, position=np.array([-1.0, 0.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=100.0)
            ]
            
        def get_fuel_burn_rate(self, actual_throttles: np.ndarray) -> float:
            return 0.0
            
    vessel = HoverVessel()
    hover_throttle = (20.0 * 9.80665) / 200.0
    
    initial_state = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -100.0]),
        vel=np.zeros(3),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.zeros(3),
        fuel_mass=10.0,
        throttles=np.array([hover_throttle, hover_throttle])
    )
    
    dt1 = DigitalTwin(env, vessel, initial_state)
    state1 = dt1.step(commanded_throttles=np.array([hover_throttle, hover_throttle]), commanded_gimbals=np.zeros((2, 2)), dt=0.05)
    
    np.testing.assert_allclose(state1.vel, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(state1.pos, np.array([0.0, 0.0, -100.0]), atol=1e-12)
    
    dt2 = DigitalTwin(env, vessel, initial_state)
    state2 = dt2.step(commanded_throttles=np.array([hover_throttle, hover_throttle]), commanded_gimbals=np.zeros((2, 2)), dt=0.05)
    
    np.testing.assert_equal(state1.pos, state2.pos)
    np.testing.assert_equal(state1.vel, state2.vel)
    np.testing.assert_equal(state1.q, state2.q)
    np.testing.assert_equal(state1.omega, state2.omega)
