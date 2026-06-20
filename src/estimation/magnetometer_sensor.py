"""
Magnetometer sensor — placeholder for future integration.

======================================================================
 WHY A MAGNETOMETER?
======================================================================

The current attitude estimator (Mahony filter, mahony_estimator.py) uses only:

  • Gyroscope     — short-term integration (angular rate × dt)
  • Accelerometer — gravity-reference correction (gated off during burns)

This leaves roll AND yaw unobservable during powered descent when the
accelerometer correction gate closes (|f| >> |g| because of engine thrust):

  1. ROLL is unobservable from the cross(body_Y, target_up) error computed
     in controller.py:260 — that cross product has zero Y-component, so
     roll is entirely handled by the geometric projection hack
     ROLL_CORRECTION_KP (line 274).

  2. YAW drifts freely when only gyro integration feeds the Mahony filter
     during the ~30 s retrograde burn.

  3. The current workaround (sensors.py:191-210) bypasses Mahony entirely
     and uses kRPC truth attitude — a cheat that prevents autonomous
     real-hardware deployment.

A magnetometer measures the local magnetic field vector **B** in body
frame [Bx, By, Bz].  Given a known reference field **B_ned** (from a
planetary magnetic model), the measured vs. expected field provides a
vector observation that:

  • Breaks the roll/yaw symmetry — the horizontal (N/E) component of
    **B_ned** gives observability in ALL THREE axes.
  • Is valid DURING powered burns — magnetic field is not corrupted by
    engine thrust (unlike the accelerometer).
  • Converges globally — the magnetic field direction has no sign-flip
    ambiguity (unlike the gravity vector which is symmetric under 180°
    roll).

======================================================================
 WHAT WOULD CHANGE
======================================================================

  1. CONFIG (sensors.conf)
     ──────────────────────
     Add Kerbin's local magnetic field at the KSC pad and sensor noise:

         # Kerbin surface field at KSC ≈ 25 µT, inclination ≈ -65°
         # NED: [B_north, B_east, B_down] — CRS or IGRF model.
         # Rough estimate from Kerbin magnetic equator (vanilla KSP):
         #   B_horizontal ≈ 25 µT pointing north
         #   B_vertical   ≈ -50 µT (pointing up — south-hemisphere)
         #   => NED: [25e-6, 0, 50e-6]  (north, east, down)
         MAG_LOCAL_NED = [2.5e-5, 0.0, 5.0e-5]
         SIGMA_MAG = 1.0e-6   # 1 µT RMS noise (typical MEMS)

  2. SENSOR MODEL (this file)
     ─────────────────────────
     MagnetometerSensor class following the GyroSensor / AccelerometerSensor
     pattern in src/estimation/.  Key method:

         def poll(self, truth_attitude: np.ndarray) -> np.ndarray:
             # Compute true field in body frame: B_body = q^-1 @ B_ned @ q
             # Add Gaussian noise (SIGMA_MAG)
             # Return B_body (3,)

     kRPC does NOT expose a magnetic field stream natively — we simulate it
     synthetically from the known quaternion.  This is correct for testing:
     real hardware would use an actual magnetometer IC.

  3. SENSOR MODELS WRAPPER (sensors.py)
     ───────────────────────────────────
     Add to SensorModels.__init__():

         self.mag_sensor = MagnetometerSensor(...)

     Add mag measurement to poll() return tuple (element 10):

         mag_body = self.mag_sensor.poll(krpc_att)

  4. MAHONY FILTER (mahony_estimator.py)
     ────────────────────────────────────
     Extend the correction-error computation.  The standard Mahony+Mag
     fusion uses a weighted sum of gravity and magnetic errors:

         # Gravity correction (gated as before)
         error_acc = cross(g_expected_body, g_measured_body)

         # Magnetic correction (always active)
         B_expected_body = q^-1 @ B_ned @ q
         B_measured_body = mag_body / ||mag_body||
         error_mag = cross(B_expected_body, B_measured_body)

         # TRIAD: project magnetic error orthogonal to gravity reference
         # to avoid cross-coupling:
         error_mag_ortho = (
             error_mag
             - dot(error_mag, g_expected_body) * g_expected_body
         )
         error_total = k_acc * error_acc + k_mag * error_mag_ortho

     Where k_acc = kp (from existing config) and k_mag ≈ 0.1-0.5 × kp.
     The orthogonal projection prevents the magnetic correction from
     corrupting the pitch/roll estimates already fixed by gravity.

  5. CONTROL LOOP (loop.py)
     ───────────────────────
     Pass mag_body to the Mahony update:

         self.attitude_estimator.update(
             omega_body, sf_body_noisy, gravity_ned, dt_mahony,
             mag_body=mag_body,
             mag_local_ned=config.MAG_LOCAL_NED,
         )

  6. EVENTUALLY REMOVE THE TRUTH-ATTITUDE HACK
     ──────────────────────────────────────────
     Once Mag+Mahony converges during powered burns, sensors.py:191-210
     can revert to using the Mahony estimate instead of kRPC truth.
     This makes the full pipeline autonomous (no kRPC cheating).

======================================================================
 RISKS & CAVEATS
======================================================================

  • Hard to test without KSP access — rely on synthetic field from truth.
  • KSP has no magnetometer part model in stock — fine for simulation
    (we generate synthetic measurements), but on real hardware verify
    the magnetometer's alignment with the vessel frame.
  • Kerbin's magnetic field model is extremely simple (axial dipole ≈
    25 µT at equator).  For other bodies (Mun, Minmus) the surface field
    may be negligible — the magnetometer only helps on bodies with a
    significant magnetic field aligned differently from the gravity
    vector (i.e., not perfectly axial).
  • The TRIAD orthogonal projection assumes gravity is the dominant
    reference; if both references disagree by >90° the filter can
    diverge — add a consistency check (dot(B_expected, B_measured) > 0).
"""

import numpy as np
import logging

import src.config as config

logger = logging.getLogger(__name__)


class MagnetometerSensor:
    """
    Simulated magnetometer sensor.

    Generates synthetic magnetic field measurements in the body frame
    from the truth attitude and a known local magnetic field vector.
    This mimics a real 3-axis magnetometer IC (e.g., RM3100, LIS3MDL)
    for attitude estimation.

    kRPC does not expose native magnetic field data — synthetic generation
    is the only option in simulation.  Replace with real driver on hardware.
    """

    def __init__(self, local_mag_ned: np.ndarray | None = None,
                 sigma_mag: float | None = None) -> None:
        """
        Args:
            local_mag_ned: (3,) Magnetic field vector in NED frame [T].
                           Default from config.MAG_LOCAL_NED if available,
                           else [2.5e-5, 0, 5e-5] (Kerbin KSC estimate).
            sigma_mag:    Standard deviation of Gaussian noise [T].
                           Default from config.SIGMA_MAG if available,
                           else 1e-6.
        """
        if local_mag_ned is not None:
            self.local_mag_ned = np.asarray(local_mag_ned, dtype=float)
        elif hasattr(config, 'MAG_LOCAL_NED'):
            self.local_mag_ned = np.asarray(config.MAG_LOCAL_NED, dtype=float)
        else:
            self.local_mag_ned = np.array([2.5e-5, 0.0, 5.0e-5], dtype=float)
            logger.warning("MAG_LOCAL_NED not configured, using Kerbin KSC default.")

        if sigma_mag is not None:
            self.sigma_mag = float(sigma_mag)
        elif hasattr(config, 'SIGMA_MAG'):
            self.sigma_mag = float(config.SIGMA_MAG)
        else:
            self.sigma_mag = 1e-6
            logger.warning("SIGMA_MAG not configured, using 1 uT default.")

        self.rng = np.random.default_rng(
            config.RANDOM_SEED if hasattr(config, 'RANDOM_SEED') else 42
        )

        logger.info(
            f"MagnetometerSensor: local_mag={self.local_mag_ned}, "
            f"sigma={self.sigma_mag:.2e}"
        )

    def poll(self, truth_attitude: np.ndarray) -> np.ndarray:
        """
        Simulate a magnetometer reading.

        Args:
            truth_attitude: (4,) quaternion [x,y,z,w] (body->NED rotation).

        Returns:
            mag_body: (3,) noisy magnetic field vector in body frame [T].
        """
        from scipy.spatial.transform import Rotation as R

        # Rotate local field from NED to body frame using the inverse
        # of the body->NED quaternion.
        rot = R.from_quat(truth_attitude)
        mag_truth_body = rot.inv().apply(self.local_mag_ned)

        # Add white Gaussian noise
        noise = self.rng.normal(0.0, self.sigma_mag, size=3)
        mag_body = mag_truth_body + noise

        return mag_body

    def get_local_mag_ned(self) -> np.ndarray:
        """Return the local magnetic field vector in NED frame."""
        return self.local_mag_ned.copy()
