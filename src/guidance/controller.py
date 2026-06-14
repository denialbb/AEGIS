import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore

class GuidanceController:
    """
    Guidance Controller for computing the required 6-DOF wrench.
    Uses Proportional-Derivative (PD) control for both translation and attitude.
    """
    def __init__(self, 
                 kp_pos_lateral: float,
                 kp_pos_vertical: float,
                 kd_vel_lateral: float,
                 kd_vel_vertical: float,
                 kp_att: np.ndarray, 
                 kd_att: np.ndarray,
                 gravity: np.ndarray = np.zeros(3)):
        """
        Initializes the Guidance Controller with tunable gains.
        
        Args:
            kp_pos_lateral: Proportional gain for lateral position error.
            kp_pos_vertical: Proportional gain for vertical position error.
            kd_vel_lateral: Derivative gain for lateral velocity error.
            kd_vel_vertical: Derivative gain for vertical velocity error.
            kp_att: (3,) Proportional gains for attitude error.
            kd_att: (3,) Derivative gains for attitude error (angular velocity damping).
            gravity: (3,) Gravity vector in world frame (e.g. [0, 0, -9.81]). 
                     Used for feed-forward gravity compensation.
        """
        self.kp_pos_lateral = float(kp_pos_lateral)
        self.kp_pos_vertical = float(kp_pos_vertical)
        self.kd_vel_lateral = float(kd_vel_lateral)
        self.kd_vel_vertical = float(kd_vel_vertical)
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
                       up_vector: np.ndarray,
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
        if current_state.shape != (6,):
            raise ValueError(f"current_state must have shape (6,), got {current_state.shape}")
        if target_state.shape != (6,):
            raise ValueError(f"target_state must have shape (6,), got {target_state.shape}")
        
        if dt <= 0.0:
            dt = 1e-6
            
        # ---------------------------------------------------------
        # 1. TRANSLATION CONTROL (World Frame)
        # ---------------------------------------------------------
        pos_err = target_state[:3] - current_state[:3]
        vel_err = target_state[3:] - current_state[3:]
        
        # Decompose errors into vertical (along up_vector) and lateral components
        pos_err_vert = np.dot(pos_err, up_vector) * up_vector
        pos_err_lat = pos_err - pos_err_vert
        
        vel_err_vert = np.dot(vel_err, up_vector) * up_vector
        vel_err_lat = vel_err - vel_err_vert
        
        # Commanded Acceleration Equation:
        # a_cmd = Kp_pos * e_pos + Kd_vel * e_vel - g
        # By subtracting the gravity vector (which typically points downwards, e.g. [0, 0, -9.81]),
        # we add an upward feed-forward acceleration to counteract it.
        a_cmd_pos = (self.kp_pos_lateral * pos_err_lat) + (self.kp_pos_vertical * pos_err_vert)
        a_cmd_vel = (self.kd_vel_lateral * vel_err_lat) + (self.kd_vel_vertical * vel_err_vert)
        
        a_cmd_world = a_cmd_pos + a_cmd_vel - self.gravity
        
        # ---------------------------------------------------------
        # 2. FRAME ROTATION (World -> Body)
        # ---------------------------------------------------------
        # kRPC provides the rotation quaternion in [x, y, z, w] format, which matches SciPy natively.
        rot = R.from_quat(current_attitude)
        
        # We need the commanded acceleration in the BODY frame.
        # Since rot maps body -> world, rot.inv() maps world -> body.
        a_cmd_body = rot.inv().apply(a_cmd_world)
        
        # Newton's Second Law: F = m * a
        force_body = mass * a_cmd_body
        
        # ---------------------------------------------------------
        # 3. ATTITUDE CONTROL (Body Frame)
        # ---------------------------------------------------------
        # We want the vessel's Y-axis (nose) to align with the world `up_vector`.
        # Map the world up_vector into the body frame:
        target_up_body = rot.inv().apply(up_vector)
        
        # The cross product between our current forward axis [0, 1, 0] and the target 
        # up vector gives the rotation axis and magnitude required to align them.
        err_axis = np.cross(np.array([0.0, 1.0, 0.0]), target_up_body)
        
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
