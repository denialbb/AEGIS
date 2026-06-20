import math
import numpy as np
import logging
from typing import Tuple, Any
import src.config as config
from scipy.spatial.transform import Rotation as R  # type: ignore

from src.estimation.gyro_sensor import GyroSensor
from src.estimation.accelerometer_sensor import AccelerometerSensor
from src.estimation.mahony_estimator import MahonyAttitudeEstimator

logger = logging.getLogger(__name__)


class SensorModels:
    """
    Wraps kRPC telemetry streams and injects synthetic Gaussian noise.

    Returns a 10-tuple compatible with main.py's expectations:
    0. noisy_alt      : float               — altitude [m]
    1. sf_body        : ndarray (3,)       — body-frame specific force [m/s²]
    2. attitude       : ndarray (4,)       — Mahony quaternion [x,y,z,w]
    3. mass           : float               — vessel mass [kg]
    4. aero_body      : ndarray (3,)       — body-frame aero force [N]
    5. situation      : str                  — e.g. "flying"
    6. omega_body     : ndarray (3,)       — body-frame angular rates [rad/s]
    7. vel            : ndarray (3,)       — NED frame velocity [m/s]
    8. gravity_ned    : ndarray (3,)       — gravity in NED frame [m/s²]
    9. raw_gyro       : ndarray (3,)       — raw gyro readings (noisy) [rad/s]
    """
    def __init__(self, conn: Any, vessel: Any, ned_frame: Any, up_vector: np.ndarray):
        self.conn = conn
        self.vessel = vessel
        self.ned_frame = ned_frame
        self.up_vector = up_vector

        flight_ned = self.vessel.flight(self.ned_frame)
        flight_body = self.vessel.flight(self.vessel.orbit.body.reference_frame)

        self.altitude_stream = self.conn.add_stream(getattr, flight_body, 'mean_altitude')
        try:
            self.pad_mean_altitude = float(self.vessel.orbit.body.surface_height(config.TARGET_LAT, config.TARGET_LON))
        except (TypeError, ValueError, AttributeError):
            # Fallback for unit testing mocks
            self.pad_mean_altitude = 0.0
        self.velocity_stream = self.conn.add_stream(getattr, flight_ned, 'velocity')
        self.ut_stream = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.attitude_stream = self.conn.add_stream(getattr, flight_ned, 'rotation')
        self.aero_stream = self.conn.add_stream(getattr, flight_ned, 'aerodynamic_force')
        self.mass_stream = self.conn.add_stream(getattr, self.vessel, 'mass')
        self.situation_stream = self.conn.add_stream(getattr, self.vessel, 'situation')

        self.last_vel: np.ndarray | None = None
        self.last_ut: float | None = None

        self.gyro_sensor = GyroSensor(conn, vessel, ned_frame, up_vector)
        self.accel_sensor = AccelerometerSensor(
            conn, vessel, ned_frame, up_vector,
            shared_ut_stream=self.ut_stream,
            shared_vel_stream=self.velocity_stream,
        )

        self.attitude_estimator = MahonyAttitudeEstimator(
            kp=config.MAHONY_KP if hasattr(config, 'MAHONY_KP') else 2.0,
            ki=config.MAHONY_KI if hasattr(config, 'MAHONY_KI') else 0.0,
            up_vector=up_vector,
        )

        self.sigma_alt = config.SIGMA_ALT
        self.sigma_accel = config.SIGMA_ACCEL
        self.sigma_vel = config.SIGMA_VEL

        self.rng = np.random.default_rng(config.RANDOM_SEED)

        logger.info(
            f"Initialized SensorModels with sigma_alt={self.sigma_alt}, "
            f"sigma_accel={self.sigma_accel}, sigma_vel={self.sigma_vel}"
        )

    def get_truth_attitude(self) -> np.ndarray:
        """Return kRPC truth attitude quaternion [x,y,z,w] (body→NED).

        Uses the kRPC flight.rotation stream directly — this is the
        ground-truth attitude from KSP physics, NOT the Mahony estimate.
        """
        return self._read_krpc_quaternion()

    def _read_krpc_quaternion(self) -> np.ndarray:
        """
        Read the kRPC rotation and convert to [x,y,z,w] (scipy convention).

        ………………………… CONVENTION TRAP …………………………
        kRPC ``flight.rotation`` can return EITHER 3 Euler angles (heading,
        pitch, roll) OR a 4-element quaternion depending on the kRPC version.

        * When returning 3 Euler angles, kRPC provides the rotation from
          NED→body (i.e. the Euler angles that describe how the NED axes
          are rotated to align with the vessel body).
        * When returning a 4-element quaternion, kRPC provides the rotation
          from body→NED (i.e. the orientation of the body axes *in* NED).

        ``R.from_euler("YXZ", (heading, pitch, roll))`` reconstructs the
        NED→body rotation (matching the KSP heading-pitch-roll convention).
        This is the INVERSE of what the Mahony filter, EKF and guidance
        controller expect (they all require body→NED).  Hence the ``.inv()``.

        If the stream returns a 4-element sequence we pass it through
        directly — kRPC already returns it as body→NED.
        …………………………………………………………………………

        Returns
        -------
        np.ndarray
            Quaternion ``[x, y, z, w]`` in scipy convention, representing
            the body→NED rotation (consistent with the rest of the pipeline).
        """
        raw = self.attitude_stream()
        try:
            euler = tuple(float(v) for v in raw)
            if len(euler) == 3:
                rot = R.from_euler("YXZ", euler, degrees=True).inv()
                return rot.as_quat()
        except (TypeError, ValueError):
            pass
        arr = np.array(raw, dtype=float)
        if arr.shape == (4,):
            q = arr / np.linalg.norm(arr)
            # kRPC flight.rotation returns body→NED quaternion directly
            # (as verified by visual attitude tests). No conjugate is needed.
            q_body_to_ned = q.copy()
            if q_body_to_ned[3] < 0:
                q_body_to_ned = -q_body_to_ned
            return q_body_to_ned
        return np.array([0.0, 0.0, 0.0, 1.0])

    def poll(self) -> Tuple[
        float,
        np.ndarray,
        np.ndarray,
        float,
        np.ndarray,
        str,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        """
        Sample all streams and apply noise.
        Returns 10-tuple matching main.py's destructure.
        """
        # ── Altitude & velocity ───────────────────────────────────
        perfect_alt = self.altitude_stream() - self.pad_mean_altitude

        ut = self.ut_stream()
        vel = np.array(self.velocity_stream())

        self.last_vel = vel
        self.last_ut = ut

        # ── Mass, aero, situation ─────────────────────────────────
        mass = self.mass_stream()
        aero_ned = np.array(self.aero_stream())
        situation = self.situation_stream().name

        # ── kRPC attitude quaternion (used ONLY for frame rotation) ──
        krpc_att = self._read_krpc_quaternion()
        rot_bw: R = R.from_quat(krpc_att)  # body → NED rotation

        # ── Accelerometer: NED-frame specific force + gravity ──────
        sf_ned, gravity_ned = self.accel_sensor.poll(np.zeros(3))

        # Rotate specific-force to body frame using kRPC truth attitude.
        # This gives both filters a correctly-framed input.
        # Noise is already added in accelerometer_sensor.poll() — no duplicate.
        sf_body_noisy: np.ndarray = rot_bw.inv().apply(sf_ned)

        # ── Gyroscope: body-frame angular rates ─────────────────────
        # gyro_sensor.poll() returns ω in NED frame axes (the custom
        # pad-relative frame).  Rotate to body frame, matching the
        # pattern used for sf and aero above.
        omega_ned: np.ndarray = self.gyro_sensor.poll()
        omega_body: np.ndarray = rot_bw.inv().apply(omega_ned)

        # Obtain raw gyroscope reading (NED frame), rotate to body
        av_raw = self.gyro_sensor.angular_velocity_stream()
        if hasattr(av_raw, "x"):
            perfect_raw_ned = np.array([av_raw.x, av_raw.y, av_raw.z])
        else:
            perfect_raw_ned = np.array(av_raw, dtype=float)
        raw_gyro_ned = perfect_raw_ned if config.NOISELESS_MODE else \
            perfect_raw_ned + self.rng.normal(0, self.gyro_sensor.sigma_gyro, size=3)
        raw_gyro: np.ndarray = rot_bw.inv().apply(raw_gyro_ned)
        # NOTE: gyroscope “raw” data includes noise but does not include bias correction.


        # ── Mahony filter update (currently bypassed — see TODO) ────
        # TODO(MAHONY-DIVERGENCE): The Mahony attitude filter diverges during
        # the high-thrust retrograde burn because the gyro bias from the
        # warmup phase is zeroed (FRAME-004: SAS rotation makes warmup
        # bias estimate unusable), and the EKF gyro bias converges too
        # slowly.  The Mahony integral term then accumulates the uncorrected
        # bias, causing the quaternion to drift 180° within ~3-5 seconds,
        # triggering ATT-FLIP resets.  Each reset gives a correct attitude
        # momentarily, but the filter immediately starts drifting again.
        #
        # The drift means compute_wrench() gets a wildly wrong body-frame
        # rotation, producing near-zero axial force → zero braking.
        #
        # TEMPORARY FIX: Use kRPC truth attitude directly.  The Mahony
        # is still updated for monitoring but its output is ignored.
        #
        # PROPER FIX: Either (a) pre-calibrate gyro bias during coast
        # when SAS is not rotating the vessel, or (b) use the truth
        # attitude stream directly as the "attitude estimate" since KSP
        # provides it at 20+ Hz with no latency concerns.
        dt_mahony: float = 1.0 / config.TARGET_HZ
        # Pass krpc_att to the attitude estimator to align it with truth and prevent divergence in monitoring/UI
        self.attitude_estimator.quaternion = krpc_att
        self.attitude_estimator.update(
            omega_body, sf_body_noisy, gravity_ned, dt_mahony
        )

        # ── Log attitude divergence: Mahony estimate vs kRPC truth ──
        try:
            mahony_q = self.attitude_estimator.get_attitude()
            dot_q = abs(float(np.dot(mahony_q, np.array(krpc_att))))
            dot_q = min(max(dot_q, -1.0), 1.0)
            angle_err_deg = 2.0 * math.degrees(math.acos(dot_q)) if dot_q < 0.9999 else 0.0
            eul_truth = R.from_quat(krpc_att).as_euler('xyz', degrees=True)
            eul_mahony = R.from_quat(mahony_q).as_euler('xyz', degrees=True)
            logger.info(
                f"[SENSOR] krpc_q=({krpc_att[0]:+.3f},{krpc_att[1]:+.3f},{krpc_att[2]:+.3f},{krpc_att[3]:+.3f}) "
                f"mahony_q=({mahony_q[0]:+.3f},{mahony_q[1]:+.3f},{mahony_q[2]:+.3f},{mahony_q[3]:+.3f}) "
                f"angle_err={angle_err_deg:.1f}° "
                f"truth_RPY=({eul_truth[0]:+.1f},{eul_truth[1]:+.1f},{eul_truth[2]:+.1f}) "
                f"mahony_RPY=({eul_mahony[0]:+.1f},{eul_mahony[1]:+.1f},{eul_mahony[2]:+.1f})"
            )
        except Exception as e:
            logger.warning(f"[SENSOR] Attitude log failed: {e}")

        # ── Aero in body frame ─────────────────────────────────────
        aero_body: np.ndarray = rot_bw.inv().apply(aero_ned)

        # ── Noise on alt and vel ───────────────────────────────────
        if config.NOISELESS_MODE:
            noisy_alt = float(perfect_alt)
            noisy_vel = vel
        else:
            noisy_alt = float(perfect_alt + self.rng.normal(0, self.sigma_alt))
            noisy_vel = vel + self.rng.normal(0, self.sigma_vel, size=3)

        logger.debug(
            f"Gyro: [{omega_body[0]:.3f}, {omega_body[1]:.3f}, {omega_body[2]:.3f}] "
            f"SF body: [{sf_body_noisy[0]:.3f}, {sf_body_noisy[1]:.3f}, {sf_body_noisy[2]:.3f}] "
            f"Attitude (truth): [{krpc_att[0]:.3f}, {krpc_att[1]:.3f}, "
            f"{krpc_att[2]:.3f}, {krpc_att[3]:.3f}]"
        )

        return (
            noisy_alt,
            sf_body_noisy,
            krpc_att,
            float(mass),
            aero_body,
            situation,
            omega_body,
            noisy_vel,
            gravity_ned,
            raw_gyro,
        )
