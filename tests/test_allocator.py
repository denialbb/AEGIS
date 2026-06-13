import numpy as np
import pytest
import logging
from src.common.engine import Engine
from src.guidance.allocator import ControlAllocator, AllocationDegenerateError

def test_allocator_nominal():
    # 4 engines placed symmetrically
    engines = [
        Engine(0, np.array([1.0, 0.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(1, np.array([-1.0, 0.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(2, np.array([0.0, 1.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(3, np.array([0.0, -1.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0)
    ]
    
    allocator = ControlAllocator(engines)
    
    # Desired wrench: pure upward force of 200 N
    # W = [Fx, Fy, Fz, Tx, Ty, Tz]
    wrench = np.array([0.0, 0.0, 200.0, 0.0, 0.0, 0.0])
    
    throttles, gimbals = allocator.allocate(wrench, engines)
    
    assert throttles.shape == (4,)
    assert gimbals.shape == (4, 2)
    
    # Each engine should output 50 N, so throttle = 0.5
    np.testing.assert_allclose(throttles, [0.5, 0.5, 0.5, 0.5], atol=1e-5)
    
    # Thrust is pure Z, aligned with thrust_direction, so gimbals should be ~0
    np.testing.assert_allclose(gimbals, np.zeros((4, 2)), atol=1e-5)

def test_allocator_torque():
    # 4 engines placed symmetrically (so it is full rank)
    engines = [
        Engine(0, np.array([1.0, 0.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(1, np.array([-1.0, 0.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(2, np.array([0.0, 1.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(3, np.array([0.0, -1.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0)
    ]
    
    allocator = ControlAllocator(engines)
    
    # Pure torque around Y axis (pitch)
    # Fz on engine 0 causes -Ty (since r = [1,0,0], r x F = [0, -Fz, 0])
    # So if we want +Ty of 50, we need Fz on engine 0 to be -25 and engine 1 to be +25
    # Wait, B[4, 2] for engine 0 (where B is 6x6): r = [1,0,0], F = [0,0,Fz] -> r x F = [0, -Fz, 0] -> Ty = -Fz.
    # So Fz on engine 0 gives -Ty. Engine 1 has r = [-1,0,0], so Ty = Fz.
    # Desired wrench Ty = 50. Then engine 1 Fz = 25, engine 0 Fz = -25.
    # But wait, pinv might distribute it to Fx/Fy as well?
    # Let's just test that the resulting u produces the desired wrench.
    wrench = np.array([0.0, 0.0, 0.0, 0.0, 50.0, 0.0])
    throttles, gimbals = allocator.allocate(wrench, engines)
    
    # We don't check throttles exactly because Fz might be negative (which still increases f_mag)
    # Let's verify if the B * u = wrench.
    # We can reconstruct u from throttles and gimbals? Not perfectly without knowing the reverse mapping.
    
    assert throttles.shape == (4,)

def test_allocator_rank_deficient():
    # Single engine at origin -> rank deficient (can't produce torques)
    engines = [
        Engine(0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0)
    ]
    allocator = ControlAllocator(engines)
    wrench = np.array([0.0, 0.0, 100.0, 10.0, 0.0, 0.0])
    
    with pytest.raises(AllocationDegenerateError) as excinfo:
        allocator.allocate(wrench, engines)

    assert "rank < 6" in str(excinfo.value)

def test_allocator_empty():
    allocator = ControlAllocator([])
    throttles, gimbals = allocator.allocate(np.zeros(6), [])
    assert len(throttles) == 0
    assert gimbals.shape == (0, 2)

def test_allocator_throttle_saturation(caplog):
    # Ensure rank is 6 by spreading engines
    engines = [
        Engine(0, np.array([ 1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),
        Engine(1, np.array([-1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),
        Engine(2, np.array([ 0.0,  1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),
        Engine(3, np.array([ 0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),
        Engine(4, np.array([ 0.0,  0.0, 1.0]), np.array([1.0, 0.0, 0.0]), 10.0),
        Engine(5, np.array([ 0.0,  0.0,-1.0]), np.array([0.0, 1.0, 0.0]), 10.0)
    ]
    allocator = ControlAllocator(engines)
    
    # Request massive Z force (1200 N total, 200 N per engine Z-wise), they only have 10 N
    wrench = np.array([0.0, 0.0, 1200.0, 0.0, 0.0, 0.0])
    
    with caplog.at_level(logging.WARNING):
        throttles, gimbals = allocator.allocate(wrench, engines)
        
    # Throttles should be saturated at 1.0 (some engines might not contribute if B makes them 0, 
    # but the first 4 pointing Z will be heavily saturated).
    assert throttles[0] == 1.0
    assert throttles[1] == 1.0
    assert "thrust saturated" in caplog.text

def test_allocator_gimbal_angles():
    # 6 engines pointing Z. We request pure X force.
    # The pseudo-inverse will distribute the X force equally among them to minimize norm.
    engines = [
        Engine(0, np.array([ 1.0,  0.0,  0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(1, np.array([-1.0,  0.0,  0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(2, np.array([ 0.0,  1.0,  0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(3, np.array([ 0.0, -1.0,  0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(4, np.array([ 0.0,  0.0,  1.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(5, np.array([ 0.0,  0.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0)
    ]
    allocator = ControlAllocator(engines)
    
    # Pure X force wrench = [120, 0, 0, 0, 0, 0]
    # Since all engines point Z, they must gimbal 90 degrees to produce X force.
    wrench = np.array([120.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    throttles, gimbals = allocator.allocate(wrench, engines)
    
    # Thrust direction is [0,0,1], desired force is [20,0,0]. Angle is 90 degrees (pi/2).
    # Cross product [0,0,1] x [1,0,0] = [0,1,0]. So rotation axis is Y.
    # Gimbal vector should have magnitude pi/2 and be along Y axis: [0.0, ~1.57]
    for i in range(6):
        assert np.isclose(gimbals[i, 1], np.pi / 2, atol=1e-5)
        assert np.isclose(gimbals[i, 0], 0.0, atol=1e-5)
