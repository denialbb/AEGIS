import krpc
import time
import numpy as np

conn = krpc.connect(address='172.22.80.1')
print("Loading savefile...")
conn.space_center.load("AEGIS MK2")
time.sleep(2.0)

vessel = conn.space_center.active_vessel
ref_frame = vessel.reference_frame

if conn.krpc.paused:
    conn.krpc.paused = False
    
print("Waiting 3s for physics ease-in...")
time.sleep(3.0)

print("Releasing clamps and activating engines...")
vessel.control.activate_next_stage()
time.sleep(0.5)
vessel.control.activate_next_stage()
time.sleep(1.0)

for part in vessel.parts.engines:
    part.active = True
    part.thrust_limit = 1.0
    
vessel.control.sas = False
vessel.control.rcs = False

print("Setting vessel throttle to 1.0...")
vessel.control.throttle = 1.0
time.sleep(1.0)

def set_gimbal(gx, gy):
    for part in vessel.parts.with_tag('AegisEngine'):
        for module in part.modules:
            if module.name == "ModuleGimbalTrim":
                module.set_field_float("Gimbal X", float(gx))
                module.set_field_float("Gimbal Y", float(gy))

def run_test(name, gx, gy):
    print(f"Running {name}...")
    set_gimbal(gx, gy)
    time.sleep(1.0) # Let physics catch up
    
    # Take baseline angular velocity
    v0 = np.array(vessel.angular_velocity(ref_frame))
    
    # Wait another second of physics
    time.sleep(1.0)
    
    # Take post angular velocity
    v1 = np.array(vessel.angular_velocity(ref_frame))
    
    dv = v1 - v0
    print(f"Test {name}: dv = {dv}")
    
run_test("Gimbal X = +10.0", 10.0, 0.0)
run_test("Gimbal Y = +10.0", 0.0, 10.0)
run_test("Gimbal X = -10.0", -10.0, 0.0)
run_test("Gimbal Y = -10.0", 0.0, -10.0)

vessel.control.throttle = 0.0
set_gimbal(0.0, 0.0)
