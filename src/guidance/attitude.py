import numpy as np
from typing import Any
import math
import logging

import src.config as config

logger = logging.getLogger(__name__)


class AttitudeController:
    """
    Unified attitude controller that maps the ADRC torque demand directly to 
    KSP joystick commands. This ensures that both the engine gimbals and the 
    reaction wheels actuate simultaneously based on the exact same error signal,
    preventing the two systems from fighting each other.

    Body frame convention (kRPC vessel reference frame):
        X = starboard,  Y = forward (nose),  Z = belly (down)

    KSP joystick convention:
        Positive pitch  → nose toward belly (positive rotation about body X)
        Positive yaw    → nose toward starboard (negative rotation about body Z)
        Positive roll   → right wing down
    """

    def __init__(self) -> None:
        self._prev_pitch_cmd: float = 0.0
        self._prev_yaw_cmd: float = 0.0
        self._prev_roll_cmd: float = 0.0
        self._cmd_alpha: float = 0.6

    def update(
        self,
        director: Any,
        torque_body: np.ndarray,
        state: str,
        ves_orientation: str,
    ) -> None:
        if not self._is_active(state, ves_orientation):
            self._zero_controls(director)
            return

        # Map required torque (N*m) to joystick commands [-1.0, 1.0]
        # We scale by an estimated RW torque authority. If the required torque is huge, 
        # the joystick will max out and the gimbals will provide the rest.
        torque_scale = 100.0  # Assumed reaction wheel authority factor
        
        # torque_body[0] is X-axis torque (pitch). Positive = pitch down (KSP positive)
        # torque_body[2] is Z-axis torque (yaw). Positive = nose left. KSP positive yaw = nose right.
        # torque_body[1] is Y-axis torque (roll). Positive = roll right (KSP positive)
        
        raw_pitch = float(torque_body[0] / torque_scale)
        raw_yaw = float(-torque_body[2] / torque_scale)
        raw_roll = float(torque_body[1] / torque_scale)

        # We strictly limit the reaction wheels to act as gentle "steering trims"
        # to prevent fighting SAS and causing massive overshoot.
        max_trim = 0.15
        
        pitch_cmd = float(np.clip(raw_pitch, -max_trim, max_trim))
        yaw_cmd = float(np.clip(raw_yaw, -max_trim, max_trim))
        roll_cmd = float(np.clip(raw_roll, -max_trim, max_trim))

        self._smooth_set(director, pitch_cmd, yaw_cmd, roll_cmd)

    def _is_active(self, state: str, ves_orientation: str) -> bool:
        if state not in (
            "HOVER_TARGETING",
            "TERMINAL_DESCENT",
            "TERMINAL_LANDING",
        ):
            return False
        if state in ("HOVER_TARGETING", "TERMINAL_DESCENT", "TERMINAL_LANDING"):
            # We allow trims even when stability assist is on!
            return ves_orientation in ("stability", "off")
        return False

    def _zero_controls(self, director: Any) -> None:
        director.vessel.control.pitch = 0.0
        director.vessel.control.yaw = 0.0
        director.vessel.control.roll = 0.0

    def _smooth_set(
        self, director: Any, pitch_cmd: float, yaw_cmd: float, roll_cmd: float
    ) -> None:
        a = self._cmd_alpha
        self._prev_pitch_cmd = a * pitch_cmd + (1.0 - a) * self._prev_pitch_cmd
        self._prev_yaw_cmd = a * yaw_cmd + (1.0 - a) * self._prev_yaw_cmd
        self._prev_roll_cmd = a * roll_cmd + (1.0 - a) * self._prev_roll_cmd
        
        director.vessel.control.pitch = float(self._prev_pitch_cmd)
        director.vessel.control.yaw = float(self._prev_yaw_cmd)
        director.vessel.control.roll = float(self._prev_roll_cmd)
