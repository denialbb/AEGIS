import krpc
import math

address = "172.22.80.1"
conn = krpc.connect(name="CalcBurnHeight", address=address)
conn.space_center.load("AEGIS MK2")
vessel = conn.space_center.active_vessel

import time
time.sleep(2)

mass = vessel.mass
# Find Aegis engines
engines = []
for part in vessel.parts.all:
    if "AegisEngine" in part.tag and part.engine is not None:
        engines.append(part.engine)

if not engines:
    engines = vessel.parts.engines

max_thrust = sum(e.max_thrust for e in engines)

gravity = 9.81
twr = max_thrust / (mass * gravity)
a_avail = (max_thrust / mass) - gravity

v0 = 250.0
stopping_distance = (v0**2) / (2 * a_avail) if a_avail > 0 else float('inf')

print(f"Mass: {mass:.2f} kg")
print(f"Max Thrust: {max_thrust:.2f} N")
print(f"TWR: {twr:.2f}")
print(f"Available Accel: {a_avail:.2f} m/s^2")
print(f"Theoretical stopping distance from 250m/s: {stopping_distance:.2f} m")

burn_height = stopping_distance * 1.2 + 50.0
print(f"Recommended ALT_POWERED_DESCENT: {burn_height:.2f} m")
