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

    Returns a 9-tuple compatible with main.py's expectations:
    0. noisy_alt      : float               — altitude [m]
    1. sf_body        : ndarray (3,)       — body-frame specific force [m/s²]
    2. attitude       : ndarray (4,)       — Mahony quaternion [x,y,z,w]
    3. mass           : float               — vessel mass [kg]
    4. aero_body      : ndarray (3,)       — body-frame aero force [N]
    5. situation      : str                  — e.g. "flying"
    6. omega_body     : ndarray (3,)       — body-frame angular rates [rad/s]
    7. vel            : ndarray (3,)       — world-frame velocity [m/s]
    8. gravity_world  : ndarray (3,)       — gravity in world frame [m/s²]
    """
    def __init__(self, conn: Any, vessel: Any, ref_frame: Any, up_vector: np.ndarray):
        self.conn = conn
        self.vessel = vessel
        self.ref_frame = ref_frame
        self.up_vector = up_vector

        flight_world = self.vessel.flight(self.ref_frame)

        self.altitude_stream = self.conn.add_stream(getattr, flight_world, 'surface_altitude')
        self.velocity_stream = self.conn.add_stream(getattr, flight_world, 'velocity')
        self.ut_stream = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.attitude_stream = self.conn.add_stream(getattr, flight_world, 'rotation')
        self.aero_stream = self.conn.add_stream(getattr, flight_world, 'aerodynamic_force')
        self.mass_stream = self.conn.add_stream(getattr, self.vessel, 'mass')
        self.situation_stream = self.conn.add_stream(getattr, self.vessel, 'situation')

        self.last_vel: np.ndarray | None = None
        self.last_ut: float | None = None

        self.gyro_sensor = GyroSensor(conn, vessel, ref_frame, up_vector)
        self.accel_sensor = AccelerometerSensor(conn, vessel, ref_frame, up_vector)

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
    ]:
        """
        Sample all streams and apply noise.
        Returns 9-tuple matching main.py's destructure.
        """
        # ── Altitude & velocity ───────────────────────────────────
        perfect_alt = self.altitude_stream()

        ut = self.ut_stream()
        vel = np.array(self.velocity_stream())

        self.last_vel = vel
        self.last_ut = ut

        # ── Mass, aero, situation ─────────────────────────────────
        mass = self.mass_stream()
        aero_world = np.array(self.aero_stream())
        situation = self.situation_stream().name

        # ── kRPC attitude quaternion (used ONLY for frame rotation) ──
        krpc_att = self._read_krpc_quaternion()
        rot_bw: R = R.from_quat(krpc_att)

        # ── Accelerometer: world-frame specific force + gravity ────
        sf_world, gravity_world = self.accel_sensor.poll(np.zeros(3))

        # Rotate specific-force to body frame using kRPC truth attitude.
        # This gives both filters a noisy but correctly-framed input.
        sf_body_raw: np.ndarray = rot_bw.inv().apply(sf_world)
        sf_body_noisy: np.ndarray = sf_body_raw + self.rng.normal(0, self.sigma_accel, size=3)

        # ── Gyroscope: body-frame angular rates ─────────────────────
        omega_body: np.ndarray = self.gyro_sensor.poll()

        # ── Mahony filter update (attitude for EKF + guidance) ───────
        dt_mahony: float = 1.0 / config.TARGET_HZ
        mahony_attitude: np.ndarray = self.attitude_estimator.update(
            omega_body, sf_body_noisy, gravity_world, dt_mahony
        )

        # ── Aero in body frame ─────────────────────────────────────
        aero_body: np.ndarray = rot_bw.inv().apply(aero_world)

        # ── Noise on alt and vel ───────────────────────────────────
        noisy_alt: float = float(perfect_alt + self.rng.normal(0, self.sigma_alt))
        noisy_vel: np.ndarray = vel + self.rng.normal(0, self.sigma_vel, size=3)

        logger.debug(
            f"Gyro: [{omega_body[0]:.3f}, {omega_body[1]:.3f}, {omega_body[2]:.3f}] "
            f"SF body: [{sf_body_noisy[0]:.3f}, {sf_body_noisy[1]:.3f}, {sf_body_noisy[2]:.3f}] "
            f"Attitude: [{mahony_attitude[0]:.3f}, {mahony_attitude[1]:.3f}, "
            f"{mahony_attitude[2]:.3f}, {mahony_attitude[3]:.3f}]"
        )
        logger.debug(
            f"Gravity world: [{gravity_world[0]:.3f}, {gravity_world[1]:.3f}, {gravity_world[2]:.3f}]"
        )

        return (
            noisy_alt,
            sf_body_noisy,
            mahony_attitude,
            float(mass),
            aero_body,
            situation,
            omega_body,
            noisy_vel,
            gravity_world,
        )

    def _read_krpc_quaternion(self) -> np.ndarray:
        """
        Read the kRPC rotation and convert to [x,y,z,w] (scipy convention).

        kRPC ``flight.rotation`` returns a Euler-angle triplet, NOT a
        quaternion.  We reconstruct a quaternion via
        ``R.from_euler('YXZ', (y, x, z))`` which matches the KSP
        heading-pitch-roll convention.  If the stream returns a
        4-element sequence we fall back to a direct cast.
        """
        raw = self.attitude_stream()
        try:
            euler = tuple(float(v) for v in raw)
            if len(euler) == 3:
                rot = R.from_euler("YXZ", euler, degrees=True)
                return rot.as_quat()
        except (TypeError, ValueError):
            pass
        arr = np.array(raw, dtype=float)
        if arr.shape == (4,):
            q = arr / np.linalg.norm(arr)
            if q[3] < 0:
                q = -q
            return q
        return np.array([0.0, 0.0, 0.0, 1.0])
