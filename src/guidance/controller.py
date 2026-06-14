import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore

class GuidanceController:
    """
    Guidance Controller for computing the required 6-DOF wrench.
    Uses Proportional-Derivative (PD) control for both translation and attitude.
    """
    def __init__(self, 
                 kp_pos: np.ndarray, 
                 kd_vel: np.ndarray, 
                 kp_att: np.ndarray, 
                 kd_att: np.ndarray,
                 gravity: np.ndarray = np.zeros(3)):
        """
        Initializes the Guidance Controller with tunable gains.
        
        Args:
            kp_pos: (3,) Proportional gains for position error.
            kd_vel: (3,) Derivative gains for velocity error.
            kp_att: (3,) Proportional gains for attitude error.
            kd_att: (3,) Derivative gains for attitude error (angular velocity damping).
            gravity: (3,) Gravity vector in world frame (e.g. [0, 0, -9.81]). 
                     Used for feed-forward gravity compensation.
        """
        self.kp_pos = np.array(kp_pos, dtype=float)
        self.kd_vel = np.array(kd_vel, dtype=float)
        self.kp_att = np.array(kp_att, dtype=float)
        self.kd_att = np.array(kd_att, dtype=float)
        self.gravity = np.array(gravity, dtype=float)
        
        # State to store previous attitude error for numerical differentiation
        self.last_att_error = np.zeros(3)

    def reset(self) -> None:
        """Resets the internal state of the controller, specifically the derivative term."""
        self.last_att_error = np.zeros(3)

    def compute_wrench(self, 
                       current_state: np.ndarray, 
                       current_attitude: np.ndarray, 
                       mass: float, 
                       target_state: np.ndarray,
                       dt: float = 0.02) -> np.ndarray:
        """
        Computes the 6-DOF wrench required to move towards the target state 
        and an upright attitude.
        
        Args:
            current_state: (6,) array [x, y, z, vx, vy, vz] in world frame.
            current_attitude: (4,) array [w, x, y, z] scalar-first quaternion.
            mass: Current vessel mass in kg.
            target_state: (6,) array [x, y, z, vx, vy, vz] in world frame.
            dt: Time step for numerical differentiation of attitude error.
            
        Returns:
            wrench: (6,) array [Fx, Fy, Fz, Tx, Ty, Tz] in the body frame.
        """
        assert current_state.shape == (6,), f"current_state must have shape (6,), got {current_state.shape}"
        assert target_state.shape == (6,), f"target_state must have shape (6,), got {target_state.shape}"
        
        if dt <= 0.0:
            dt = 1e-6
            
        # ---------------------------------------------------------
        # 1. TRANSLATION CONTROL (World Frame)
        # ---------------------------------------------------------
        pos_err = target_state[:3] - current_state[:3]
        vel_err = target_state[3:] - current_state[3:]
        
        # Commanded Acceleration Equation:
        # a_cmd = Kp_pos * e_pos + Kd_vel * e_vel - g
        # By subtracting the gravity vector (which typically points downwards, e.g. [0, 0, -9.81]),
        # we add an upward feed-forward acceleration to counteract it.
        a_cmd_world = (self.kp_pos * pos_err) + (self.kd_vel * vel_err) - self.gravity
        
        # ---------------------------------------------------------
        # 2. FRAME ROTATION (World -> Body)
        # ---------------------------------------------------------
        # Convert scalar-first [w, x, y, z] to scalar-last [x, y, z, w] for SciPy Rotation
        quat_scalar_last = np.array([
            current_attitude[1], 
            current_attitude[2], 
            current_attitude[3], 
            current_attitude[0]
        ])
        
        # rot represents the rotation FROM body TO world frame
        rot = R.from_quat(quat_scalar_last)
        
        # We need the commanded acceleration in the BODY frame.
        # Since rot maps body -> world, rot.inv() maps world -> body.
        a_cmd_body = rot.inv().apply(a_cmd_world)
        
        # Newton's Second Law: F = m * a
        force_body = mass * a_cmd_body
        
        # ---------------------------------------------------------
        # 3. ATTITUDE CONTROL (Body Frame)
        # ---------------------------------------------------------
        # Our target attitude is assumed to be upright, which corresponds to the identity rotation.
        # The rotation error R_err from the current body frame to the target frame is simply:
        # R_err = R_current^-1 * R_target = R_current^-1 * I = R_current^-1
        # Thus, the quaternion of R_err is simply the inverse of the current rotation quaternion.
        q_err = rot.inv().as_quat()
        
        # A quaternion q = [x, y, z, w] represents a rotation of angle theta around axis v:
        # q = [v_x*sin(theta/2), v_y*sin(theta/2), v_z*sin(theta/2), cos(theta/2)]
        # We enforce the shortest path by ensuring the scalar part (w) is positive.
        if q_err[3] < 0:
            q_err = -q_err
            
        # The vector part provides a proportional error term since it scales with sin(theta/2).
        # For small angles, sin(theta/2) ≈ theta/2.
        err_axis = q_err[:3]
        
        # We estimate the body angular velocity by numerically differentiating the error axis.
        d_err_axis = (err_axis - self.last_att_error) / dt
        self.last_att_error = err_axis
        
        # Torque Equation (PD Control):
        # tau = Kp_att * e_axis - Kd_att * d_e_axis
        torque_body = (self.kp_att * err_axis) - (self.kd_att * d_err_axis)
        
        # ---------------------------------------------------------
        # 4. ASSEMBLE WRENCH
        # ---------------------------------------------------------
        wrench = np.zeros(6)
        wrench[:3] = force_body
        wrench[3:] = torque_body
        
        return wrench
