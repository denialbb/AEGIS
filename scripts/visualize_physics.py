import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from scipy.spatial.transform import Rotation as R
import pyray as pr

from src.simulation.physics import DigitalTwin, PhysicsState
from src.simulation.environment import VacuumEnvironment
from src.simulation.mock_vessel import MockVessel

def ned_to_raylib(pos_ned: np.ndarray) -> pr.Vector3:
    """Transforms NED (N, E, D) to Raylib Y-up Right-Handed (N, -D, E)."""
    return pr.Vector3(float(pos_ned[0]), float(-pos_ned[2]), float(pos_ned[1]))

def draw_vessel(state: PhysicsState, vessel: MockVessel):
    # Vessel attitude
    rot = R.from_quat(state.q)
    
    current_com = vessel.get_com_position(state.fuel_mass)
    
    # Offset cylinder by -current_com to shift the geometric body relative to the CoM
    body_bottom_ned = rot.apply(np.array([0.0, 0.0, 0.0]) - current_com) + state.pos
    body_top_ned = rot.apply(np.array([0.0, 0.0, -8.0]) - current_com) + state.pos
    
    start_pos = ned_to_raylib(body_bottom_ned)
    end_pos = ned_to_raylib(body_top_ned)
    
    # Draw yellow sphere at exact CoM (state.pos) FIRST so it isn't blocked by the cylinder's depth buffer
    pr.draw_sphere(ned_to_raylib(state.pos), 0.3, pr.YELLOW)

    vessel_color = pr.Color(200, 200, 200, int(255 * 0.5)) # 50% opacity for better visibility
    pr.draw_cylinder_ex(start_pos, end_pos, 1.0, 1.0, 16, vessel_color)
    
    # Draw engines and thrust
    for i, engine in enumerate(vessel.engines):
        eng_pos_body = engine.position - current_com
        thrust_dir_body = engine.thrust_direction
        
        eng_pos_ned = rot.apply(eng_pos_body) + state.pos
        eng_pos_rl = ned_to_raylib(eng_pos_ned)
        
        pr.draw_sphere(eng_pos_rl, 0.2, pr.RED if state.throttles[i] > 0 else pr.DARKGRAY)
        
        throttle = state.throttles[i]
        if throttle > 0:
            # Plume goes in opposite direction of thrust
            thrust_end_body = eng_pos_body - thrust_dir_body * throttle * 5.0
            thrust_end_ned = rot.apply(thrust_end_body) + state.pos
            thrust_end_rl = ned_to_raylib(thrust_end_ned)
            pr.draw_line_3d(eng_pos_rl, thrust_end_rl, pr.ORANGE)

def draw_overlay(state: PhysicsState, dt: DigitalTwin):
    # Top-right corner overlay
    screen_w = pr.get_screen_width()
    x = screen_w - 300
    
    pr.draw_text("AEGIS Telemetry", x, 10, 20, pr.GREEN)
    pr.draw_text(f"Time: {state.time:.2f} s", x, 40, 20, pr.GREEN)
    
    alt = -state.pos[2]
    pr.draw_text(f"Altitude: {alt:.2f} m", x, 70, 20, pr.GREEN)
    
    vel_z = -state.vel[2]
    pr.draw_text(f"Speed(Z): {vel_z:.2f} m/s", x, 100, 20, pr.GREEN)
    
    # Attitude
    rot = R.from_quat(state.q)
    euler = rot.as_euler('xyz', degrees=True)
    pr.draw_text(f"Att: [{euler[0]:.0f}, {euler[1]:.0f}, {euler[2]:.0f}]", x, 130, 20, pr.GREEN)
    
    tot_thrust = np.sum(state.throttles) * 100.0
    pr.draw_text(f"Total Thrust: {tot_thrust:.1f} kN", x, 160, 20, pr.GREEN)
    
    y = 190
    for i, throttle in enumerate(state.throttles):
        status = "FAILED" if i in dt.failed_engines else f"{throttle:.2f}"
        pr.draw_text(f"Eng {i}: {status}", x, y, 20, pr.GREEN if i not in dt.failed_engines else pr.RED)
        y += 30

def main():
    pr.init_window(1280, 720, "AEGIS Physics Visualizer")
    pr.set_target_fps(60)
    
    env = VacuumEnvironment(g=9.80665)
    vessel = MockVessel()
    
    initial_state = PhysicsState(
        time=0.0,
        pos=np.array([0.0, 0.0, -100.0]),
        vel=np.zeros(3),
        q=np.array([0.0, 0.0, 0.0, 1.0]),
        omega=np.zeros(3),
        fuel_mass=40.0,
        throttles=np.array([(80.0 * 9.80665) / 2000.0] * 4) # pre-spooled to hover thrust
    )
    
    twin = DigitalTwin(env, vessel, initial_state)
    
    cam = pr.Camera3D()
    cam.position = pr.Vector3(20.0, 20.0, 20.0)
    cam.target = pr.Vector3(0.0, 0.0, 0.0)
    cam.up = pr.Vector3(0.0, 1.0, 0.0)
    cam.fovy = 45.0
    cam.projection = pr.CAMERA_PERSPECTIVE
    
    physics_dt = 0.02
    accumulator = 0.0
    
    cmd_gimbals = np.zeros((4, 2))
    
    while not pr.window_should_close():
        frame_time = pr.get_frame_time()
        accumulator += frame_time
        
        while accumulator >= physics_dt:
            # Dynamically adjust hover throttle as fuel burns off
            current_mass = vessel.total_mass(twin.state.fuel_mass)
            hover_throttle = (current_mass * env.g) / 2000.0
            
            # Simple proportional velocity controller
            target_vel_z = 0.0 if twin.state.time < 3.0 else 5.0 # 5 m/s descent (NED Z is down)
            vel_error = target_vel_z - twin.state.vel[2]
            
            # If we need to go down (positive error), reduce throttle.
            throttle_adj = -0.02 * vel_error 
            
            cmd_throttles = np.array([hover_throttle + throttle_adj] * 4)
            
            twin.step(cmd_throttles, cmd_gimbals, physics_dt)
            accumulator -= physics_dt
            
        cam_offset = pr.Vector3(20.0, 20.0, 20.0)
        vessel_pos_rl = ned_to_raylib(twin.state.pos)
        cam.target = vessel_pos_rl
        cam.position = pr.Vector3(vessel_pos_rl.x + cam_offset.x, vessel_pos_rl.y + cam_offset.y, vessel_pos_rl.z + cam_offset.z)
        
        pr.begin_drawing()
        pr.clear_background(pr.BLACK)
        
        pr.begin_mode_3d(cam)
        pr.draw_grid(20, 10.0)
        
        # Draw the landing pad at (0,0,0) NED -> (0,0,0) Raylib
        # The vessel has radius 1.0, so the pad has radius 3.0. Thin cylinder from Y=0.0 to Y=-0.1.
        pr.draw_cylinder_ex(pr.Vector3(0.0, 0.0, 0.0), pr.Vector3(0.0, -0.1, 0.0), 3.0, 3.0, 32, pr.GREEN)
        
        draw_vessel(twin.state, vessel)
        pr.end_mode_3d()
        
        draw_overlay(twin.state, twin)
        
        pr.end_drawing()
        
    pr.close_window()

if __name__ == "__main__":
    main()
