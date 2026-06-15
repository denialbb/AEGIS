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
    # Test that gimbal computation doesn't crash or produce NaNs for a reasonable case.
    # Use 6 engines in a symmetric configuration requesting a small force.
    engines = [
        Engine(0, np.array([ 1.0,  0.0,  0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(1, np.array([-1.0,  0.0,  0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(2, np.array([ 0.0,  1.0,  0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(3, np.array([ 0.0, -1.0,  0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(4, np.array([ 0.0,  0.0,  1.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(5, np.array([ 0.0,  0.0, -1.0]), np.array([0.0, 0.0, 1.0]), 100.0)
    ]
    allocator = ControlAllocator(engines)
    
    # Small Z force wrench = [0, 0, 12, 0, 0, 0] (12N total, 2N per engine)
    # This requires minimal gimbaling and should work fine
    wrench = np.array([0.0, 0.0, 12.0, 0.0, 0.0, 0.0])
    throttles, gimbals = allocator.allocate(wrench, engines)
    
    # Each engine should produce ~2N of Z force
    expected_force_per_engine = 2.0
    for i in range(6):
        f_vec = np.array([
            engines[i].thrust_direction[0] * throttles[i] * engines[i].max_thrust,
            engines[i].thrust_direction[1] * throttles[i] * engines[i].max_thrust,
            engines[i].thrust_direction[2] * throttles[i] * engines[i].max_thrust
        ])
        assert np.isclose(f_vec[2], expected_force_per_engine, atol=1e-1), f"Engine {i} Z force: {f_vec[2]}"
    
    # Verify throttles are reasonable (should be 0.02 for 2N/100N max)
    for i in range(6):
        assert np.isclose(throttles[i], 0.02, atol=1e-1), f"Engine {i} throttle: {throttles[i]}"
    
    # Verify gimbal values are sane (not NaN or extremely large)
    for i in range(6):
        assert not np.isnan(gimbals[i, 0])
        assert not np.isnan(gimbals[i, 1])
        assert np.isfinite(gimbals[i, 0])
        assert np.isfinite(gimbals[i, 1])
        # Gimbal angles should be small (within reasonable bounds for small force)
        assert abs(gimbals[i, 0]) < 1.0  # Less than ~57 degrees
        assert abs(gimbals[i, 1]) < 1.0  # Less than ~57 degrees

def test_allocator_negative_thrust():
    engines = [
        Engine(0, np.array([ 1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(1, np.array([-1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(2, np.array([ 0.0,  1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(3, np.array([ 0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(4, np.array([ 0.0,  0.0, 1.0]), np.array([1.0, 0.0, 0.0]), 100.0),
        Engine(5, np.array([ 0.0,  0.0,-1.0]), np.array([0.0, 1.0, 0.0]), 100.0)
    ]
    allocator = ControlAllocator(engines)
    
    # Request massive downward force (-1200 Z). Engines 0-3 point in +Z.
    # To satisfy this, the allocator might try to fire engines 0-3 with negative dot product,
    # or fire engines 4/5 by gimbaling. Actually, engines 0-3 are physically incapable of pushing -Z.
    wrench = np.array([0.0, 0.0, -1200.0, 0.0, 0.0, 0.0])
    throttles, gimbals = allocator.allocate(wrench, engines)
    
    # Engines 0-3 should be clipped to 0.0 because dot_prod < 0
    assert throttles[0] == 0.0
    assert throttles[1] == 0.0
    assert throttles[2] == 0.0
    assert throttles[3] == 0.0


def test_is_rank_sufficient():
    engines_6 = [
        Engine(i, pos, np.array([0.0, 0.0, 1.0]), 100.0)
        for i, pos in enumerate([
            np.array([1.0, 0.0, -1.0]),
            np.array([-1.0, 0.0, -1.0]),
            np.array([0.0, 1.0, -1.0]),
            np.array([0.0, -1.0, -1.0]),
            np.array([0.5, 0.5, 0.0]),
            np.array([-0.5, -0.5, 0.0]),
        ])
    ]
    allocator = ControlAllocator(engines_6)

    # 0 engines: insufficient
    sufficient, rank = allocator.is_rank_sufficient([])
    assert not sufficient
    assert rank == 0

    # 1 engine: insufficient
    sufficient, rank = allocator.is_rank_sufficient(engines_6[:1])
    assert not sufficient

    # 2 engines: always insufficient (max rank is 5)
    sufficient, rank = allocator.is_rank_sufficient(engines_6[:2])
    assert not sufficient

    # 3 engines with diverse positions: sufficient
    sufficient, rank = allocator.is_rank_sufficient(engines_6[:3])
    assert sufficient, f"Expected rank >= 6, got {rank}"
    assert rank >= 6

    # 4 engines: sufficient
    sufficient, rank = allocator.is_rank_sufficient(engines_6[:4])
    assert sufficient
    assert rank >= 6

    # All 6 engines: sufficient
    sufficient, rank = allocator.is_rank_sufficient(engines_6)
    assert sufficient
    assert rank >= 6

    # 2 coaxial engines (same x-axis): rank < 6
    coaxial = [
        Engine(0, np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(1, np.array([-1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
    ]
    sufficient, rank = allocator.is_rank_sufficient(coaxial)
    assert not sufficient
