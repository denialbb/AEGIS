import krpc
import time
import numpy as np

conn = krpc.connect(address='172.22.80.1')
vessel = conn.space_center.active_vessel

def print_gimbal_effect():
    eng_part = vessel.parts.with_name('liquidEngineMini.v2')[0]
    eng = eng_part.engine
    
    mod = None
    for m in eng_part.modules:
        if m.name == "ModuleGimbalTrim":
            mod = m
            
    if not mod:
        print("No ModuleGimbalTrim found!")
        return

    # Baseline
    mod.set_field_float("Gimbal X", 0.0)
    mod.set_field_float("Gimbal Y", 0.0)
    time.sleep(0.1)
    d0 = np.array(eng.thrust_direction(eng_part.reference_frame))
    
    # X = 10
    mod.set_field_float("Gimbal X", 10.0)
    time.sleep(0.1)
    dx = np.array(eng.thrust_direction(eng_part.reference_frame))
    
    # Y = 10
    mod.set_field_float("Gimbal X", 0.0)
    mod.set_field_float("Gimbal Y", 10.0)
    time.sleep(0.1)
    dy = np.array(eng.thrust_direction(eng_part.reference_frame))
    
    print(f"Base: {d0}")
    print(f"X=10: {dx} (diff: {dx-d0})")
    print(f"Y=10: {dy} (diff: {dy-d0})")

print_gimbal_effect()
