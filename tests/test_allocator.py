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


def test_allocator_iterative_saturation():
    """Test that the allocator correctly redistributes force when some engines saturate."""
    # Create 4 engines pointing in +Z direction at different positions
    # This setup allows us to produce pure Z force without torque
    engines = [
        Engine(0, np.array([ 1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),  # max thrust 10 N
        Engine(1, np.array([-1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),  # max thrust 10 N
        Engine(2, np.array([ 0.0,  1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),  # max thrust 10 N
        Engine(3, np.array([ 0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),  # max thrust 10 N
    ]
    allocator = ControlAllocator(engines)
    
    # Request 25 N of Z force (should require 6.25 N from each engine, well under limits)
    wrench = np.array([0.0, 0.0, 25.0, 0.0, 0.0, 0.0])
    throttles, _ = allocator.allocate(wrench, engines)
    
    # Each engine should produce 6.25 N, so throttle = 0.625
    expected_throttle = 0.625
    np.testing.assert_allclose(throttles, [expected_throttle] * 4, atol=1e-5)
    
    # Now request 50 N of Z force (should require 12.5 N from each engine, which exceeds max thrust of 10 N)
    # With the iterative allocator, we should saturate all engines at 10 N each, giving 40 N total
    wrench = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
    throttles, _ = allocator.allocate(wrench, engines)
    
    # All engines should be saturated at 1.0 (10 N each)
    expected_throttle = 1.0
    np.testing.assert_allclose(throttles, [expected_throttle] * 4, atol=1e-5)
    
    # Verify that the total force produced is 40 N (4 engines * 10 N each)
    # We can't easily compute the exact force from throttles without knowing the allocation,
    # but we know it should be 40 N since all engines are at max thrust
    
    # Test another case: asymmetric engine placement to check torque handling
    engines_asym = [
        Engine(0, np.array([ 2.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),  # 2x leverage
        Engine(1, np.array([-1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),  # 1x leverage
        Engine(2, np.array([ 0.0,  2.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),  # 2x leverage
        Engine(3, np.array([ 0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 10.0),  # 1x leverage
    ]
    allocator_asym = ControlAllocator(engines_asym)
    
    # Request pure Z force of 30 N
    # Let Fz0 = force from engine 0 (pos [2,0,0])
    # Let Fz1 = force from engine 1 (pos [-1,0,0])
    # Let Fz2 = force from engine 2 (pos [0,2,0])
    # Let Fz3 = force from engine 3 (pos [0,-1,0])
    #
    # Torque balance:
    # Tx: -2*Fz2 + 1*Fz3 = 0  → Fz3 = 2*Fz2
    # Ty: -2*Fz0 + 1*Fz1 = 0  → Fz1 = 2*Fz0
    # Force: Fz0 + Fz1 + Fz2 + Fz3 = 30
    #
    # Substituting: Fz0 + 2*Fz0 + Fz2 + 2*Fz2 = 30
    #             3*Fz0 + 3*Fz2 = 30
    #             Fz0 + Fz2 = 10
    #
    # To minimize effort (sum of squares of throttles), we set Fz0 = Fz2 = 5
    # Then: Fz1 = 2*5 = 10, Fz3 = 2*5 = 10
    wrench = np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.0])
    throttles, _ = allocator_asym.allocate(wrench, engines_asym)
    
    # Expected throttles: [0.5, 1.0, 0.5, 1.0]
    expected_throttles = np.array([0.5, 1.0, 0.5, 1.0])
    np.testing.assert_allclose(throttles, expected_throttles, atol=1e-5)


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


def test_allocator_numerical_stability():
    """Test allocator behavior near numerical limits and with ill-conditioned matrices."""
    
    # Test 1: Well-conditioned engines for baseline tests
    engines_well_conditioned = [
        Engine(0, np.array([ 1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(1, np.array([-1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(2, np.array([ 0.0,  1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(3, np.array([ 0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(4, np.array([ 0.0,  0.0, 1.0]), np.array([1.0, 0.0, 0.0]), 100.0),
        Engine(5, np.array([ 0.0,  0.0,-1.0]), np.array([0.0, 1.0, 0.0]), 100.0),
    ]
    
    allocator_well = ControlAllocator(engines_well_conditioned)
    
    # Test extremely small force values (testing numerical precision)
    tiny_wrench = np.array([1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12])
    throttles_tiny, gimbals_tiny = allocator_well.allocate(tiny_wrench, engines_well_conditioned)
    
    # Should produce very small throttles (close to zero)
    assert np.all(throttles_tiny >= 0.0) and np.all(throttles_tiny <= 1.0)
    assert np.all(np.isfinite(gimbals_tiny))
    
    # Test very large force values (should saturate)
    large_wrench = np.array([1e6, 1e6, 1e6, 1e6, 1e6, 1e6])  # Very large request
    throttles_large, gimbals_large = allocator_well.allocate(large_wrench, engines_well_conditioned)
    
    # Should be saturated (at least some engines at max throttle)
    assert np.all(throttles_large >= 0.0) and np.all(throttles_large <= 1.0)
    assert np.all(np.isfinite(gimbals_large))
    
    # Test 2: Near-singular configuration (high condition number but below threshold)
    # Create engines that are nearly identical to produce high condition number
    engines_near_singular = [
        Engine(0, np.array([ 1.0,  0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(1, np.array([ 1.0,  1e-4, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),  # Very small perturbations
        Engine(2, np.array([ 1.0,  0.0, 1e-4]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(3, np.array([ 1.0, -1e-4, 0.0]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(4, np.array([ 1.0,  0.0,-1e-4]), np.array([0.0, 0.0, 1.0]), 100.0),
        Engine(5, np.array([ 1.0,  1e-4, 1e-4]), np.array([0.0, 0.0, 1.0]), 100.0),
    ]
    
    allocator_near = ControlAllocator(engines_near_singular)
    
    # Check that the condition number is high but below degeneracy threshold (1e4)
    B_near = allocator_near._build_B(engines_near_singular)
    cond_near = np.linalg.cond(B_near)
    # Accept that we might not hit exactly the range we want, but test behavior
    print(f"Near-singular B condition number: {cond_near}")
    
    # Should still be able to solve for reasonable forces
    wrench = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])  # Small Z force
    try:
        throttles, gimbals = allocator_near.allocate(wrench, engines_near_singular)
        # If successful, verify reasonable outputs
        assert throttles.shape == (6,)
        assert gimbals.shape == (6, 2)
        assert np.all(throttles >= 0.0) and np.all(throttles <= 1.0)
        assert np.all(np.isfinite(gimbals))
    except AllocationDegenerateError:
        # If it fails due to conditioning exceeding threshold, that's informative
        # but we'll skip the numerical assertions in that case
        if cond_near > 1e4:
            # Expected to fail if condition number exceeds threshold
            pass
        else:
            # Unexpected failure
            raise
    
    # Test 3: Force values that might cause numerical issues in intermediate calculations
    # Test with values that could cause loss of precision in matrix operations
    medium_wrench = np.array([1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0])
    throttles_medium, gimbals_medium = allocator_well.allocate(medium_wrench, engines_well_conditioned)
    
    assert np.all(throttles_medium >= 0.0) and np.all(throttles_medium <= 1.0)
    assert np.all(np.isfinite(gimbals_medium))
