import numpy as np
import pytest
from src.common.engine import Engine
from src.fdi.fdi import FaultDetectionIsolation

def test_detect_fault():
    fdi = FaultDetectionIsolation(threshold=0.5)
    
    expected = np.array([0.0, 10.0, 0.0])
    measured_nominal = np.array([0.0, 9.8, 0.0]) # Diff is 0.2 < 0.5
    measured_fault = np.array([0.0, 8.0, 0.0])   # Diff is 2.0 > 0.5
    
    assert not fdi.detect_fault(expected, measured_nominal)
    assert fdi.detect_fault(expected, measured_fault)

def test_isolate_fault():
    fdi = FaultDetectionIsolation(threshold=0.5)
    
    e0 = Engine(index=0, position=np.array([ 1.0, 0.0,  0.0]), thrust_direction=np.array([ 0.1, 0.99, 0.0]), max_thrust=100.0)
    e1 = Engine(index=1, position=np.array([-1.0, 0.0,  0.0]), thrust_direction=np.array([-0.1, 0.99, 0.0]), max_thrust=100.0)
    e2 = Engine(index=2, position=np.array([ 0.0, 0.0,  1.0]), thrust_direction=np.array([ 0.0, 0.99, 0.1]), max_thrust=100.0)
    e3 = Engine(index=3, position=np.array([ 0.0, 0.0, -1.0]), thrust_direction=np.array([ 0.0, 0.99,-0.1]), max_thrust=100.0)
    
    # Normalize thrust directions just to be safe
    e0.thrust_direction /= np.linalg.norm(e0.thrust_direction)
    e1.thrust_direction /= np.linalg.norm(e1.thrust_direction)
    e2.thrust_direction /= np.linalg.norm(e2.thrust_direction)
    e3.thrust_direction /= np.linalg.norm(e3.thrust_direction)
    
    active_engines = [e0, e1, e2, e3]
    mass = 10.0
    
    # All engines firing at 100%
    expected_throttles = np.array([1.0, 1.0, 1.0, 1.0])
    
    expected_force = (e0.thrust_direction + e1.thrust_direction + e2.thrust_direction + e3.thrust_direction) * 100.0
    expected_accel = expected_force / mass
    
    # 1) Nominal case:
    faults = fdi.isolate_fault(active_engines, expected_throttles, expected_accel, mass)
    assert len(faults) == 0
    
    # 2) Engine 2 fails (index 2)
    measured_force_e2_fail = (e0.thrust_direction + e1.thrust_direction + e3.thrust_direction) * 100.0
    measured_accel_e2_fail = measured_force_e2_fail / mass
    
    faults = fdi.isolate_fault(active_engines, expected_throttles, measured_accel_e2_fail, mass)
    assert faults == [2]

    # 3) Engine 0 fails (index 0)
    measured_force_e0_fail = (e1.thrust_direction + e2.thrust_direction + e3.thrust_direction) * 100.0
    measured_accel_e0_fail = measured_force_e0_fail / mass
    
    faults = fdi.isolate_fault(active_engines, expected_throttles, measured_accel_e0_fail, mass)
    assert faults == [0]
