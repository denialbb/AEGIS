import numpy as np
from typing import Any
import math
import logging
from scipy.spatial.transform import Rotation as R

import src.config as config

logger = logging.getLogger(__name__)


class AttitudeController:
    """
    Manual attitude controller for KSP that tracks a desired NED acceleration
    vector or a specific nose orientation by outputting joystick commands
    (pitch, yaw).  SAS remains in ``stability_assist`` mode for damping and
    roll hold.  When joystick commands are zeroed SAS holds the current
    attitude — so we must *continuously* command toward the target and
    zero commands only when on-target to let SAS freeze the attitude.

    Body frame convention (kRPC vessel reference frame):
        X = starboard,  Y = forward (nose),  Z = belly (down)

    KSP joystick convention:
        Positive pitch  → nose toward belly (positive rotation about body X)
        Positive yaw    → nose toward starboard (negative rotation about body Z)
        Positive roll   → right wing down

    Sign derivation for error → command:
        cross([0,1,0], target_body) = [tz, 0, -tx]
        pitch command (body X rotation)  = +tz   (tz>0 → target below nose → pitch down)
        yaw   command (body Z rotation)  = +tx   (tx>0 → target on starboard → yaw right)
    """

    ERROR_DEADBAND: float = 0.005

    def __init__(self) -> None:
        self._prev_pitch_cmd: float = 0.0
        self._prev_yaw_cmd: float = 0.0
        self._cmd_alpha: float = 0.6

    def update(
        self,
        director: Any,
        a_cmd_ned: np.ndarray,
        state: str,
        ves_orientation: str,
        current_attitude: np.ndarray,
        est_alt: float = 0.0,
        angular_velocity: np.ndarray | None = None,
    ) -> None:
        if not self._is_active(state, ves_orientation):
            self._zero_controls(director)
            return

        target_nose_ned = self._get_target_nose_ned(
            director, a_cmd_ned, state
        )
        if target_nose_ned is None:
            return

        max_pitch = self._max_pitch_for_state(state)
        target_nose_ned = self._clamp_pitch(
            target_nose_ned, director.up_vector, max_pitch
        )
        self._apply_control(director, state, target_nose_ned, current_attitude, angular_velocity)

    def _is_active(self, state: str, ves_orientation: str) -> bool:
        if state not in (
            "HOVER_TARGETING",
            "TERMINAL_DESCENT",
            "TERMINAL_LANDING",
        ):
            return False
        if state in ("HOVER_TARGETING", "TERMINAL_DESCENT", "TERMINAL_LANDING"):
            return ves_orientation in ("stability", "off")
        return False

    def _zero_controls(self, director: Any) -> None:
        director.vessel.control.pitch = 0.0
        director.vessel.control.yaw = 0.0
        director.vessel.control.roll = 0.0

    def _max_pitch_for_state(self, state: str) -> float:
        base = config.ATTITUDE_MAX_PITCH_DEG
        if state == "POWERED_DESCENT":
            return min(base, 20.0)
        if state == "HOVER_TARGETING":
            return min(base, 22.0)
        if state == "TERMINAL_DESCENT":
            return min(base, 18.0)
        if state == "TERMINAL_LANDING":
            return min(base, 10.0)
        return base

    def _get_target_nose_ned(
        self,
        director: Any,
        a_cmd_ned: np.ndarray,
        state: str,
    ) -> np.ndarray | None:
        norm_a = float(np.linalg.norm(a_cmd_ned))
        up = director.up_vector

        if state == "POWERED_DESCENT":
            if norm_a > 1e-6:
                return a_cmd_ned / norm_a
            return up

        if state == "HOVER_TARGETING":
            return self._hover_target_nose(director, a_cmd_ned, norm_a, up)

        if state in ("TERMINAL_DESCENT", "TERMINAL_LANDING"):
            return self._terminal_target_nose(director, a_cmd_ned, norm_a, up, state)

        return None

    def _hover_target_nose(
        self,
        director: Any,
        a_cmd_ned: np.ndarray,
        norm_a: float,
        up: np.ndarray,
    ) -> np.ndarray:
        horiz_vel = director.estimator.vel - np.dot(director.estimator.vel, up) * up
        horiz_pos = director.estimator.pos[:2]
        dist = float(np.linalg.norm(horiz_pos))
        vh = float(np.linalg.norm(horiz_vel))

        if dist < 5.0 and vh < 2.0:
            return up

        if norm_a > 1e-6:
            raw_target = a_cmd_ned / norm_a
            return self._clamp_pitch(raw_target, up, max_pitch_deg=5.0)
        return up

    def _terminal_target_nose(
        self,
        director: Any,
        a_cmd_ned: np.ndarray,
        norm_a: float,
        up: np.ndarray,
        state: str,
    ) -> np.ndarray:
        horiz_vel = director.estimator.vel - np.dot(director.estimator.vel, up) * up
        horiz_pos = director.estimator.pos[:2]
        dist = float(np.linalg.norm(horiz_pos))
        vh = float(np.linalg.norm(horiz_vel))

        brake_threshold = 3.0
        if state == "TERMINAL_LANDING":
            brake_threshold = 1.0

        if dist < brake_threshold and vh < 0.5:
            return up

        if norm_a > 1e-6:
            raw_target = a_cmd_ned / norm_a
            return self._clamp_pitch(raw_target, up, max_pitch_deg=3.0)
        return up

    def _clamp_pitch(
        self,
        target_nose_ned: np.ndarray,
        up_vector: np.ndarray,
        max_pitch_deg: float,
    ) -> np.ndarray:
        vertical_comp = float(np.dot(target_nose_ned, up_vector))
        horizontal_comp = float(
            np.linalg.norm(target_nose_ned - vertical_comp * up_vector)
        )

        pitch_rad = math.atan2(vertical_comp, horizontal_comp)
        min_pitch_rad = math.radians(90.0 - max_pitch_deg)

        if pitch_rad < min_pitch_rad and horizontal_comp > 1e-6:
            horizontal_dir = (
                target_nose_ned - vertical_comp * up_vector
            ) / horizontal_comp
            new_vertical = math.sin(min_pitch_rad)
            new_horizontal = math.cos(min_pitch_rad)
            target = new_horizontal * horizontal_dir + new_vertical * up_vector
            return target / np.linalg.norm(target)

        return target_nose_ned

    def _apply_control(
        self,
        director: Any,
        state: str,
        target_nose_ned: np.ndarray,
        current_attitude: np.ndarray,
        angular_velocity: np.ndarray | None = None,
    ) -> None:
        rot_body_to_ned = R.from_quat(current_attitude)

        current_nose_ned = rot_body_to_ned.apply([0, 1, 0])

        target_nose_body = rot_body_to_ned.inv().apply(target_nose_ned)

        # cross([0,1,0], target_body) = [tz, 0, -tx]
        tx, _, tz = target_nose_body

        # Pitch: negative tz → target is towards Belly → pitch down (positive KSP pitch)
        # Yaw: positive tx → target is to the right → yaw right (positive KSP yaw)
        pitch_error = -tz
        yaw_error = tx
        error_mag = math.sqrt(pitch_error ** 2 + yaw_error ** 2)
        if error_mag < self.ERROR_DEADBAND:
            self._smooth_set(director, 0.0, 0.0, 0.0)
            logger.debug(
                f"[ATTITUDE] state={state} err_mag={error_mag:.4f} < deadband → hold"
            )
            return

        kp = config.ATTITUDE_KP
        kd = getattr(config, "ATTITUDE_KD", 0.0)
        
        raw_pitch = kp * pitch_error
        raw_yaw = kp * yaw_error
        
        if angular_velocity is not None:
            raw_pitch += kd * angular_velocity[0]
            raw_yaw += kd * angular_velocity[2]

        if error_mag > 1.0:
            scale = 1.0 / error_mag
            pitch_cmd = float(np.clip(raw_pitch * scale, -1.0, 1.0))
            yaw_cmd = float(np.clip(raw_yaw * scale, -1.0, 1.0))
        else:
            pitch_cmd = float(np.clip(raw_pitch, -1.0, 1.0))
            yaw_cmd = float(np.clip(raw_yaw, -1.0, 1.0))

        self._smooth_set(director, pitch_cmd, yaw_cmd)

        lateral_ned = target_nose_ned - np.dot(target_nose_ned, director.up_vector) * director.up_vector
        lateral_err = float(np.linalg.norm(lateral_ned))
        horiz_vel = director.estimator.vel - np.dot(director.estimator.vel, director.up_vector) * director.up_vector
        vh = float(np.linalg.norm(horiz_vel))
        horiz_pos = director.estimator.pos[:2]
        dist = float(np.linalg.norm(horiz_pos))

        logger.debug(
            f"[ATTITUDE] state={state} nose_body={target_nose_body.round(3)} "
            f"pitch_e={pitch_error:.3f} yaw_e={yaw_error:.3f} "
            f"cmd_p={pitch_cmd:.3f} cmd_y={yaw_cmd:.3f} "
            f"lateral_err={lateral_err:.3f} dist={dist:.1f} vh={vh:.1f}"
        )

    def _smooth_set(
        self, director: Any, pitch_cmd: float, yaw_cmd: float, roll_cmd: float = 0.0
    ) -> None:
        a = self._cmd_alpha
        self._prev_pitch_cmd = a * pitch_cmd + (1.0 - a) * self._prev_pitch_cmd
        self._prev_yaw_cmd = a * yaw_cmd + (1.0 - a) * self._prev_yaw_cmd
        director.vessel.control.pitch = float(self._prev_pitch_cmd)
        director.vessel.control.yaw = float(self._prev_yaw_cmd)
        director.vessel.control.roll = float(roll_cmd)
