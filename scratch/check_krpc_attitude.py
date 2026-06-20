import krpc
import time

conn = krpc.connect(name="Attitude Check")
vessel = conn.space_center.active_vessel
ref_frame = vessel.surface_reference_frame

print(f"SAS: {vessel.control.sas}")
print(f"SAS Mode: {vessel.control.sas_mode}")
print(f"Pitch: {vessel.flight(ref_frame).pitch}")
print(f"Heading: {vessel.flight(ref_frame).heading}")
print(f"Roll: {vessel.flight(ref_frame).roll}")

vessel.control.pitch = 0.5
print("Set pitch joystick to 0.5")
time.sleep(1)
vessel.control.pitch = 0.0

print(f"AutoPilot active? {vessel.auto_pilot.active}")
