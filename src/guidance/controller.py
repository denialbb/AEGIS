import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore
from src.guidance.adrc import ADRCController, CTMCalculator
from src.guidance.nn import NNFeedforward
import src.config as config
import logging

logger = logging.getLogger(__name__)

class GuidanceController:
    """
    Guidance Controller for computing the required 6-DOF wrench.
    Uses Proportional-Derivative (PD) control for both translation and attitude.

    Phase 1 (ADR-028): torque law upgraded to inertia-scaled PD +
    gyroscopic cross-coupling feedforward using the full 3x3 inertia tensor.
    Kp_att / Kd_att gains are computed externally from natural frequency
    and damping ratio: Kp = omega_n^2, Kd = 2*zeta*omega_n.
    """
    def __init__(self,
                 kp_pos_lateral: float,
                 kp_pos_vertical: float,
                 kd_vel_lateral: float,
                 kd_vel_vertical: float,
                 kp_att: np.ndarray,
                 kd_att: np.ndarray,
                 gravity_ned: np.ndarray = np.zeros(3),
                 inertia_tensor: np.ndarray | None = None,
                 adrc: ADRCController | None = None,
                 ctm_calculator: CTMCalculator | None = None,
                 nn_model: NNFeedforward | None = None,
                 accel_clamp_factor: float = 1.5,
                 max_torque: np.ndarray | None = None,
                 torque_filter_alpha: float = 1.0):
        """
        Initializes the Guidance Controller with tunable gains.

        Args:
            kp_pos_lateral: Proportional gain for lateral position error.
            kp_pos_vertical: Proportional gain for vertical position error.
            kd_vel_lateral: Derivative gain for lateral velocity error.
            kd_vel_vertical: Derivative gain for vertical velocity error.
            kp_att: (3,) Proportional gains for attitude error (pseudo-acceleration, rad/s^2).
            kd_att: (3,) Derivative gains for angular velocity damping (1/s).
            gravity_ned: (3,) Gravity vector in NED frame (e.g. [0, 0, -9.81]).
            inertia_tensor: (3,3) Full inertia tensor in body frame. When provided,
                            torque = J*(Kp*e + Kd*omega_err) + omega x J*omega.
                            When None, falls back to direct PD (no inertia scaling).
            adrc: Optional ADRCController for attitude control (Phase 2).
                  When provided, replaces the PD attitude torque with the ADRC
                  WSEF control law. Gyroscopic feedforward is still applied
                  when inertia_tensor is available.
            ctm_calculator: Optional CTMCalculator for CTM-ADRC feedforward
                            (Phase 3). When provided alongside adrc, the CTM
                            feedforward torque is added to the ADRC output.
                            Requires inertia_tensor to be set.
            nn_model: Optional NNFeedforward for learned disturbance compensation
                      (Phase 4). The NN correction is added to the CTM feedforward
                      in torque space. Requires adrc and inertia_tensor.
            accel_clamp_factor: Multiplier on max_a_avail used to cap
                a_cmd_ned before force and attitude target computation.
                Default 1.5 (see config.ACCEL_CLAMP_FACTOR).
        """
        self.kp_pos_lateral = float(kp_pos_lateral)
        self.kp_pos_vertical = float(kp_pos_vertical)
        self.kd_vel_lateral = float(kd_vel_lateral)
        self.kd_vel_vertical = float(kd_vel_vertical)
        self.kp_att = np.array(kp_att, dtype=float)
        self.kd_att = np.array(kd_att, dtype=float)
        self.gravity_ned = np.array(gravity_ned, dtype=float)
        self.accel_clamp_factor = float(accel_clamp_factor)
        if max_torque is not None:
            self.max_torque = np.array(max_torque, dtype=float)
        else:
            self.max_torque = np.array(config.GUIDANCE_MAX_TORQUE, dtype=float)

        self.inertia_tensor: np.ndarray | None = None
        if inertia_tensor is not None:
            self.inertia_tensor = np.array(inertia_tensor, dtype=float)
            if self.inertia_tensor.shape != (3, 3):
                raise ValueError(f"inertia_tensor must have shape (3,3), got {self.inertia_tensor.shape}")

        self.adrc: ADRCController | None = adrc
        self.ctm_calculator: CTMCalculator | None = ctm_calculator
        if ctm_calculator is not None and inertia_tensor is None:
            raise ValueError("CTMCalculator requires inertia_tensor to be set")

        self.nn_model: NNFeedforward | None = nn_model
        if nn_model is not None and adrc is None:
            raise ValueError("NNFeedforward requires ADRC to be active (adrc must be set)")
        if nn_model is not None and inertia_tensor is None:
            raise ValueError("NNFeedforward requires inertia_tensor to be set")

        self.torque_filter_alpha = torque_filter_alpha
        self._filtered_torque: np.ndarray = np.zeros(3)

    def set_phase_gains(
        self,
        kp_pos_lateral: float,
        kd_vel_lateral: float,
        kp_pos_vertical: float | None = None,
        kd_vel_vertical: float | None = None,
    ) -> None:
        """Update translation PD gains mid-flight for phase-specific behavior.

        Args:
            kp_pos_lateral: Proportional gain for lateral position error.
            kd_vel_lateral: Derivative gain for lateral velocity error.
            kp_pos_vertical: Optional vertical position gain (None = keep current).
            kd_vel_vertical: Optional vertical velocity gain (None = keep current).
        """
        self.kp_pos_lateral = float(kp_pos_lateral)
        self.kd_vel_lateral = float(kd_vel_lateral)
        if kp_pos_vertical is not None:
            self.kp_pos_vertical = float(kp_pos_vertical)
        if kd_vel_vertical is not None:
            self.kd_vel_vertical = float(kd_vel_vertical)

    def reset(self) -> None:
        """Resets the internal state of the controller and ADRC if active."""
        if self.adrc is not None:
            self.adrc.reset()
        self._filtered_torque = np.zeros(3)

    def compute_wrench(self,
                       current_state: np.ndarray,
                       current_attitude: np.ndarray,
                       mass: float,
                       target_state: np.ndarray,
                       up_vector: np.ndarray,
                       dt: float = 0.02,
                       angular_velocity: np.ndarray | None = None,
                       max_a_avail: float | None = None) -> np.ndarray:
        """
        Computes the 6-DOF wrench required to move towards the target state
        and an upright attitude.

        Args:
            current_state: (6,) array [x, y, z, vx, vy, vz] in NED frame.
            current_attitude: (4,) array [x, y, z, w] scalar-last quaternion (kRPC/scipy convention).
                              kRPC returns [x, y, z, w] matching scipy's R.from_quat.
            mass: Current vessel mass in kg.
            target_state: (6,) array [x, y, z, vx, vy, vz] in NED frame.
            dt: Time step for numerical differentiation of attitude error.
            angular_velocity: (3,) body-frame angular velocity (rad/s). Required when
                              inertia_tensor is set; falls back to numerical differentiation
                              of error axis if None.

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
        # 1. TRANSLATION CONTROL (NED Frame)
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
        a_cmd_pos = (self.kp_pos_lateral * pos_err_lat) + (self.kp_pos_vertical * pos_err_vert)
        a_cmd_vel = (self.kd_vel_lateral * vel_err_lat) + (self.kd_vel_vertical * vel_err_vert)

        logger.debug(f"[GUIDANCE] up_vector={up_vector} current_v={current_state[3:]} target_v={target_state[3:]}")
        logger.debug(f"[GUIDANCE] current_pos={current_state[:3]} target_pos={target_state[:3]} pos_err={pos_err}")
        logger.debug(f"[GUIDANCE] est_alt={np.dot(current_state[:3], up_vector):.2f}m state_vector={current_state}")
        logger.debug(f"[GUIDANCE] dot(gravity_ned, up_vector)={np.dot(self.gravity_ned, up_vector):.4f} gravity_ned={self.gravity_ned}")
        logger.debug(f"[GUIDANCE] a_cmd_pos={a_cmd_pos} a_cmd_vel={a_cmd_vel}")
        logger.debug(f"[GUIDANCE] pos_err_lat={np.linalg.norm(pos_err_lat):.1f}m pos_err_vert={np.dot(pos_err_vert, up_vector):.1f}m")
        logger.debug(f"[GUIDANCE] vel_err_lat={np.linalg.norm(vel_err_lat):.1f}m/s vel_err_vert={np.dot(vel_err_vert, up_vector):.1f}m/s")

        a_cmd_ned = a_cmd_pos + a_cmd_vel - self.gravity_ned
        
        logger.debug(f"[GUIDANCE] a_cmd_ned={a_cmd_ned} max_a_avail={max_a_avail}")

        # Clamp a_cmd_ned magnitude to prevent the guidance from commanding
        # accelerations far beyond the vehicle's physical capability. This keeps
        # the attitude target from flipping wildly and lets the
        # allocator's existing saturation handling do its job without the
        # attitude-thrashing side effect.
        if max_a_avail is not None and max_a_avail > 0.0:
            a_norm = np.linalg.norm(a_cmd_ned)
            limit = self.accel_clamp_factor * max_a_avail
            if a_norm > limit:
                a_cmd_ned = (a_cmd_ned / a_norm) * limit

        # ---------------------------------------------------------
        # 2. FRAME ROTATION (NED -> Body)
        # ---------------------------------------------------------
        rot = R.from_quat(current_attitude)
        a_cmd_body = rot.inv().apply(a_cmd_ned)
        
        logger.debug(f"[GUIDANCE] attitude_q={current_attitude} a_cmd_ned={a_cmd_ned} a_cmd_body={a_cmd_body}")

        # Newton's Second Law: F = m * a
        force_body = mass * a_cmd_body

        # ---------------------------------------------------------
        # 3. ATTITUDE CONTROL (Body Frame)
        # ---------------------------------------------------------
        a_cmd_norm = np.linalg.norm(a_cmd_ned)
        if a_cmd_norm > 1e-6:
            target_up_ned = a_cmd_ned / a_cmd_norm
        else:
            target_up_ned = up_vector

        target_up_body = rot.inv().apply(target_up_ned)

            # Cross product between current nose [0, 1, 0] and target up (NED) vector
        # gives the rotation axis and magnitude (sin(theta)) to align them.
        err_axis = np.cross(np.array([0.0, 1.0, 0.0]), target_up_body)

        if self.adrc is not None:
            # ---- ADRC attitude control (Phase 2, ADR-027) ----
            # Optionally augmented with CTM feedforward (Phase 3)
            # and/or NN learned correction (Phase 4)
            feedforward_torque = np.zeros(3)

            if self.ctm_calculator is not None:
                if angular_velocity is None:
                    raise ValueError("angular_velocity is required for CTM-ADRC")
                feedforward_torque = self.ctm_calculator.compute_feedforward(
                    err_axis, angular_velocity,
                )

            if self.nn_model is not None and self.nn_model.is_trained:
                if angular_velocity is None:
                    raise ValueError("angular_velocity is required with NN feedforward")
                z3_current = np.array([eso.z3 for eso in self.adrc.eso])
                nn_state = np.concatenate([err_axis, angular_velocity, z3_current])
                nn_correction = self.nn_model.predict(nn_state)
                feedforward_torque += self.inertia_tensor @ nn_correction

            if self.ctm_calculator is not None or (
                self.nn_model is not None and self.nn_model.is_trained
            ):
                torque_body = self.adrc.compute_torque(
                    err_axis, angular_velocity, ctm_feedforward=feedforward_torque,
                )
            elif angular_velocity is not None:
                torque_body = self.adrc.compute_torque(err_axis, angular_velocity)
            else:
                torque_body = self.adrc.compute_torque(err_axis)
        elif self.inertia_tensor is not None:
            # ---- Inertia-scaled PD torque + gyroscopic feedforward (ADR-028) ----
            if angular_velocity is None:
                raise ValueError("angular_velocity is required when inertia_tensor is set")

            # PD torque: tau_pd = J * (Kp * e - Kd * omega)
            # Negative Kd * omega provides damping opposing rotation.
            tau_pd = self.inertia_tensor @ (self.kp_att * err_axis - self.kd_att * angular_velocity)

            # Gyroscopic cross-coupling feedforward: tau_gyro = omega x (J * omega)
            tau_gyro = np.cross(angular_velocity, self.inertia_tensor @ angular_velocity)

            torque_body = tau_pd + tau_gyro
        else:
            # ---- Legacy direct PD (no inertia scaling) ----
            if angular_velocity is not None:
                d_err_axis = angular_velocity
            else:
                d_err_axis = np.zeros(3)
            torque_body = (self.kp_att * err_axis) - (self.kd_att * d_err_axis)

        # ---- Torque clamping to prevent asymmetric thrust ----
        torque_body = np.clip(
            torque_body,
            -self.max_torque,
            self.max_torque,
        )

        # ---- Low-pass filter to smooth torque commands ----
        torque_body = self.torque_filter_alpha * torque_body + (1.0 - self.torque_filter_alpha) * self._filtered_torque
        self._filtered_torque = torque_body

        # ---------------------------------------------------------
        # 4. ASSEMBLE WRENCH
        # ---------------------------------------------------------
        wrench = np.zeros(6)
        wrench[:3] = force_body
        wrench[3:] = torque_body

        return wrench
