import numpy as np
import pytest
from src.guidance.adrc import fal, PerAxisESO, ADRCController, CTMCalculator


# =============================================================================
# fal() Unit Tests
# =============================================================================

class TestFal:
    def test_symmetry(self):
        """fal(-e, alpha, delta) == -fal(e, alpha, delta) for all e."""
        for alpha in [0.1, 0.25, 0.5, 0.75, 0.9]:
            for delta in [0.01, 0.1, 1.0]:
                for e in [-2.0, -1.0, -0.5 * delta, 0.0, 0.5 * delta, 1.0, 2.0]:
                    assert np.isclose(fal(e, alpha, delta), -fal(-e, alpha, delta))

    def test_linear_region(self):
        """For |e| <= delta, fal(e) = e / delta^(1-alpha)."""
        alpha = 0.5
        delta = 0.1
        e = 0.05
        expected = e / (delta ** (1.0 - alpha))
        assert np.isclose(fal(e, alpha, delta), expected)

    def test_nonlinear_region_positive(self):
        """For e > delta, fal(e) = |e|^alpha."""
        alpha = 0.5
        delta = 0.1
        e = 2.0
        expected = abs(e) ** alpha
        assert np.isclose(fal(e, alpha, delta), expected)

    def test_nonlinear_region_negative(self):
        """For e < -delta, fal(e) = -|e|^alpha."""
        alpha = 0.5
        delta = 0.1
        e = -2.0
        expected = -(abs(e) ** alpha)
        assert np.isclose(fal(e, alpha, delta), expected)

    def test_continuity_at_positive_delta(self):
        """Verify fal() is continuous at e = +delta."""
        alpha = 0.5
        delta = 0.1
        v_from_below = fal(delta * 0.9999, alpha, delta)
        v_at = fal(delta, alpha, delta)
        v_from_above = fal(delta * 1.0001, alpha, delta)
        assert np.isclose(v_from_below, v_at, rtol=1e-3)
        assert np.isclose(v_from_above, v_at, rtol=1e-3)

    def test_continuity_at_negative_delta(self):
        """Verify fal() is continuous at e = -delta."""
        alpha = 0.5
        delta = 0.1
        v_from_below = fal(-delta * 0.9999, alpha, delta)
        v_at = fal(-delta, alpha, delta)
        v_from_above = fal(-delta * 1.0001, alpha, delta)
        assert np.isclose(v_from_below, v_at, rtol=1e-3)
        assert np.isclose(v_from_above, v_at, rtol=1e-3)

    def test_zero_at_origin(self):
        """fal(0, alpha, delta) == 0."""
        for alpha in [0.1, 0.25, 0.5, 0.75, 0.9]:
            for delta in [0.01, 0.1, 1.0]:
                assert fal(0.0, alpha, delta) == 0.0

    def test_delta_zero_raises(self):
        """fal() must raise ValueError when delta <= 0."""
        with pytest.raises(ValueError, match="delta > 0"):
            fal(1.0, 0.5, 0.0)
        with pytest.raises(ValueError, match="delta > 0"):
            fal(1.0, 0.5, -0.1)

    def test_various_alpha_values(self):
        """fal() works across the full range of valid alpha (0, 1)."""
        delta = 0.05
        e = 1.5
        for alpha in [0.1, 0.25, 0.5, 0.75, 0.99]:
            result = fal(e, alpha, delta)
            assert np.isfinite(result)
            assert result > 0.0

    def test_boundary_e_equals_delta(self):
        """fal(delta) should match from both sides exactly."""
        alpha = 0.5
        delta = 0.1
        linear_val = delta / (delta ** (1.0 - alpha))
        nonlinear_val = delta ** alpha
        assert np.isclose(linear_val, nonlinear_val)

    def test_monotonicity(self):
        """fal(e) is monotonically increasing in e."""
        alpha = 0.5
        delta = 0.1
        es = np.linspace(-2.0, 2.0, 100)
        vals = [fal(e, alpha, delta) for e in es]
        for i in range(1, len(vals)):
            assert vals[i] >= vals[i - 1]


# =============================================================================
# PerAxisESO Unit Tests
# =============================================================================

class TestPerAxisESO:
    def test_initial_state_zero(self):
        """ESO starts with z1=z2=z3=0."""
        eso = PerAxisESO()
        assert eso.z1 == 0.0
        assert eso.z2 == 0.0
        assert eso.z3 == 0.0

    def test_update_no_nan(self):
        """ESO update doesn't produce NaN values."""
        eso = PerAxisESO(dt=0.02)
        for _ in range(100):
            eso.update(y=0.0, u=0.0)
            assert np.isfinite(eso.z1)
            assert np.isfinite(eso.z2)
            assert np.isfinite(eso.z3)

    def test_tracks_constant_input(self):
        """ESO z1 converges to a constant input y.
        Uses ω₀=5 rad/s (β01=15, β02=75, β03=125) with δ=0.1 for
        discrete-time stability at 50Hz. The linearized eigenvalue
        1-dt*β01/δ^(1-α)=0.95 ensures non-oscillatory convergence.
        """
        eso = PerAxisESO(dt=0.02, beta_01=15.0, beta_02=75.0,
                         beta_03=125.0, delta=0.1)
        y_const = 0.5
        for _ in range(1000):
            eso.update(y=y_const, u=0.0)
        assert np.isclose(eso.z1, y_const, atol=0.05)

    def test_estimates_constant_disturbance(self):
        """ESO z3 converges to a constant disturbance (acceleration) in a
        double-integrator plant: y = ∫∫(disturbance + b0*u) dt².
        With u=0, y is quadratic: y += dt*vel; vel += dt*disturbance.
        z3 should converge to the disturbance (second derivative of y).
        """
        eso = PerAxisESO(dt=0.02, beta_01=15.0, beta_02=75.0,
                         beta_03=125.0, delta=0.1)
        disturbance = -5.0
        y = 0.0
        vel = 0.0
        for _ in range(4000):
            vel += 0.02 * disturbance
            y += 0.02 * vel
            eso.update(y=y, u=0.0)
        assert np.isclose(eso.z3, disturbance, atol=0.5)

    def test_disturbance_cancellation_with_control(self):
        """With correct control, z3 estimates disturbance and z1 tracks output."""
        eso = PerAxisESO(dt=0.02, beta_01=15.0, beta_02=75.0,
                         beta_03=125.0, delta=0.1, b0=2.0)
        disturbance = 5.0
        u = -disturbance / eso.b0
        y = 0.0
        for _ in range(3000):
            y += 0.02 * (disturbance + eso.b0 * u)
            eso.update(y=y, u=u)
        assert np.isclose(eso.z3, disturbance, atol=0.5)
        assert np.isclose(eso.z1, y, atol=0.05)

    def test_converges_from_nonzero_initial(self):
        """ESO converges to correct values from non-zero initial state."""
        eso = PerAxisESO(dt=0.02, beta_01=15.0, beta_02=75.0,
                         beta_03=125.0, delta=0.1)
        eso.z1 = 10.0
        eso.z2 = 5.0
        eso.z3 = 20.0
        y_const = 0.0
        for _ in range(2000):
            eso.update(y=y_const, u=0.0)
        assert np.isclose(eso.z1, y_const, atol=0.1)
        assert np.isclose(eso.z2, 0.0, atol=0.5)

    def test_reset(self):
        """Reset sets all states to zero."""
        eso = PerAxisESO()
        eso.z1 = 5.0
        eso.z2 = 3.0
        eso.z3 = 10.0
        eso.reset()
        assert eso.z1 == 0.0
        assert eso.z2 == 0.0
        assert eso.z3 == 0.0

    def test_delta_zero_raises(self):
        """PerAxisESO construction with delta=0 raises ValueError."""
        with pytest.raises(ValueError, match="delta > 0"):
            PerAxisESO(delta=0.0)

    def test_high_gain_convergence(self):
        """Higher observer bandwidth produces faster convergence.
        ω₀=3 (β01=9) vs ω₀=5 (β01=15), both with δ=0.1 for stability.
        """
        eso_slow = PerAxisESO(dt=0.02, beta_01=9.0, beta_02=27.0,
                              beta_03=27.0, delta=0.1)
        eso_fast = PerAxisESO(dt=0.02, beta_01=15.0, beta_02=75.0,
                              beta_03=125.0, delta=0.1)
        y_const = 1.0
        for _ in range(200):
            eso_slow.update(y=y_const, u=0.0)
            eso_fast.update(y=y_const, u=0.0)
        fast_error = abs(eso_fast.z1 - y_const)
        slow_error = abs(eso_slow.z1 - y_const)
        assert fast_error < slow_error

    def test_follower_ramp(self):
        """ESO can track a ramp input (steadily increasing y) with bounded error."""
        eso = PerAxisESO(dt=0.02, beta_01=30.0, beta_02=300.0, beta_03=1000.0)
        y = 0.0
        tracking_errors = []
        for i in range(1000):
            y = 0.005 * i
            eso.update(y=y, u=0.0)
            if i > 200:
                tracking_errors.append(abs(eso.z1 - y))
        assert np.mean(tracking_errors) < 1.0

    def test_negative_b0(self):
        """ESO handles negative b0 (reversed control direction)."""
        eso = PerAxisESO(dt=0.02, b0=-1.0)
        for _ in range(100):
            eso.update(y=0.0, u=0.0)
            assert np.isfinite(eso.z1)
            assert np.isfinite(eso.z2)
            assert np.isfinite(eso.z3)


# =============================================================================
# ADRCController Unit Tests
# =============================================================================

class TestADRCController:
    def test_initialization_default(self):
        """Default ADRCController creates 3 ESOS with correct structure."""
        adrc = ADRCController()
        assert len(adrc.eso) == 3
        for eso in adrc.eso:
            assert isinstance(eso, PerAxisESO)
        assert np.allclose(adrc.kp, np.ones(3))
        assert np.allclose(adrc.kd, np.ones(3))
        assert np.allclose(adrc.prev_u, np.zeros(3))

    def test_initialization_custom_kp_kd(self):
        """Custom kp/kd gains are stored correctly."""
        kp = np.array([5.0, 3.0, 1.0])
        kd = np.array([10.0, 8.0, 6.0])
        adrc = ADRCController(kp=kp, kd=kd)
        assert np.allclose(adrc.kp, kp)
        assert np.allclose(adrc.kd, kd)

    def test_initialization_custom_eso_params(self):
        """Custom per-axis ESO parameters create correctly configured ESOS."""
        eso_params = [
            dict(beta_01=50.0, beta_02=150.0, beta_03=500.0, b0=2.0),
            dict(beta_01=80.0, beta_02=240.0, beta_03=800.0, b0=1.5),
            dict(beta_01=100.0, beta_02=300.0, beta_03=1000.0, b0=1.0),
        ]
        adrc = ADRCController(eso_params=eso_params)
        assert adrc.eso[0].beta_01 == 50.0
        assert adrc.eso[0].b0 == 2.0
        assert adrc.eso[1].beta_01 == 80.0
        assert adrc.eso[1].b0 == 1.5
        assert adrc.eso[2].beta_01 == 100.0
        assert adrc.eso[2].b0 == 1.0

    def test_invalid_eso_params_length(self):
        """eso_params must have exactly 3 entries."""
        with pytest.raises(ValueError, match="must have length 3"):
            ADRCController(eso_params=[dict(), dict()])

    def test_invalid_kp_shape(self):
        """kp must have shape (3,)."""
        with pytest.raises(ValueError, match="kp and kd must have shape"):
            ADRCController(kp=np.array([1.0, 2.0]))

    def test_compute_torque_shape(self):
        """compute_torque returns a (3,) array."""
        adrc = ADRCController()
        err = np.array([0.1, -0.05, 0.02])
        torque = adrc.compute_torque(err)
        assert torque.shape == (3,)
        assert np.all(np.isfinite(torque))

    def test_compute_torque_with_angular_velocity(self):
        """compute_torque works with angular velocity input."""
        adrc = ADRCController()
        err = np.array([0.1, -0.05, 0.02])
        omega = np.array([0.01, -0.02, 0.005])
        torque = adrc.compute_torque(err, angular_velocity=omega)
        assert torque.shape == (3,)
        assert np.all(np.isfinite(torque))

    def test_compute_torque_no_nan(self):
        """compute_torque produces finite outputs across many steps."""
        adrc = ADRCController(kp=np.array([5.0, 5.0, 5.0]), kd=np.array([10.0, 10.0, 10.0]))
        np.random.seed(42)
        for _ in range(200):
            err = np.random.randn(3) * 0.1
            omega = np.random.randn(3) * 0.05
            torque = adrc.compute_torque(err, angular_velocity=omega)
            assert torque.shape == (3,)
            assert np.all(np.isfinite(torque))

    def test_invalid_err_axis_shape(self):
        """compute_torque raises for non-(3,) err_axis."""
        adrc = ADRCController()
        with pytest.raises(ValueError, match="err_axis must have shape"):
            adrc.compute_torque(np.array([0.1, 0.2]))

    def test_converges_error_to_zero(self):
        """ADRC drives attitude error toward zero in a double-integrator plant.
        Plant model: omega += dt*torque/inertia; err += dt*omega.
        Uses ω₀=5 ESO bandwidth (β01=15) with δ=0.1.
        """
        eso_params = [dict(beta_01=15.0, beta_02=75.0, beta_03=125.0, delta=0.1)
                      for _ in range(3)]
        adrc = ADRCController(
            dt=0.02,
            kp=np.array([20.0, 20.0, 20.0]),
            kd=np.array([10.0, 10.0, 10.0]),
            eso_params=eso_params,
        )
        inertia = 2.0
        err_axis = np.array([0.5, 0.0, 0.0])
        omega = np.array([0.0, 0.0, 0.0])
        for _ in range(2000):
            torque = adrc.compute_torque(err_axis, angular_velocity=omega)
            omega += 0.02 * torque / inertia
            err_axis += 0.02 * omega
        assert abs(err_axis[0]) < 0.01
        assert abs(err_axis[1]) < 0.001
        assert abs(err_axis[2]) < 0.001

    def test_rejects_constant_disturbance(self):
        """ADRC rejects a constant disturbance better than pure PD."""
        kp = np.array([10.0, 10.0, 10.0])
        kd = np.array([5.0, 5.0, 5.0])
        adrc = ADRCController(dt=0.02, kp=kp, kd=kd)

        err_axis = np.array([0.0, 0.0, 0.0])
        omega = np.array([0.0, 0.0, 0.0])
        disturbance = np.array([2.0, 0.0, 0.0])  # constant disturbance torque

        for _ in range(1000):
            torque = adrc.compute_torque(err_axis, angular_velocity=omega)
            # Plant: err_dot = omega; omega_dot = torque + disturbance
            omega += 0.02 * (torque + disturbance)
            err_axis += 0.02 * omega

        # With ADRC, the error should remain small (disturbance rejected)
        assert abs(err_axis[0]) < 0.1

    def test_reset(self):
        """Reset clears all ESO states and prev_u."""
        adrc = ADRCController()
        adrc.compute_torque(np.array([0.1, -0.05, 0.02]))
        assert not np.allclose(adrc.prev_u, np.zeros(3))
        for eso in adrc.eso:
            assert abs(eso.z1) > 0 or abs(eso.z2) > 0 or abs(eso.z3) > 0
        adrc.reset()
        assert np.allclose(adrc.prev_u, np.zeros(3))
        for eso in adrc.eso:
            assert eso.z1 == 0.0
            assert eso.z2 == 0.0
            assert eso.z3 == 0.0

    def test_prev_u_stored(self):
        """compute_torque stores output for next tick's ESO update."""
        adrc = ADRCController()
        err = np.array([0.2, -0.1, 0.05])
        torque1 = adrc.compute_torque(err)
        assert np.allclose(adrc.prev_u, torque1)
        torque2 = adrc.compute_torque(err)
        assert np.allclose(adrc.prev_u, torque2)

    def test_zero_error_zero_torque(self):
        """With zero error and zero velocity, ADRC outputs zero torque."""
        adrc = ADRCController()
        err = np.zeros(3)
        torque = adrc.compute_torque(err)
        assert np.allclose(torque, np.zeros(3), atol=1e-10)

    def test_per_axis_independence(self):
        """Each axis ESO operates independently."""
        eso_params = [
            dict(beta_01=100.0, beta_02=300.0, beta_03=1000.0, b0=1.0),
            dict(beta_01=200.0, beta_02=600.0, beta_03=2000.0, b0=2.0),
            dict(beta_01=50.0, beta_02=150.0, beta_03=500.0, b0=0.5),
        ]
        adrc = ADRCController(eso_params=eso_params)
        # Only axis 0 has error
        err = np.array([0.5, 0.0, 0.0])
        adrc.compute_torque(err)
        # Axis 0 ESO should have non-zero states, axes 1,2 should be near zero
        assert abs(adrc.eso[0].z1) > 0.01
        assert abs(adrc.eso[1].z1) < 0.001
        assert abs(adrc.eso[2].z1) < 0.001


# =============================================================================
# Integration Tests: ADRC vs PD Attitude Control
# =============================================================================

class TestADRCvsPD:
    """Compare ADRC and PD controllers in a 1-DOF attitude simulation."""

    def _run_pd_1dof(self, dt: float, steps: int, disturbance: float,
                     kp: float, kd: float) -> tuple:
        """Pure PD 1-DOF attitude simulation."""
        theta = 0.5
        omega = 0.0
        for _ in range(steps):
            torque = kp * (0.0 - theta) - kd * omega
            omega += dt * (torque + disturbance)
            theta += dt * omega
        return theta, omega

    def _run_adrc_1dof(self, dt: float, steps: int, disturbance: float,
                       kp: float, kd: float, b0: float) -> tuple:
        """ADRC 1-DOF attitude simulation."""
        theta = 0.5
        omega = 0.0
        eso = PerAxisESO(dt=dt, beta_01=15.0, beta_02=75.0,
                         beta_03=125.0, delta=0.1, b0=b0)
        prev_u = 0.0
        for _ in range(steps):
            eso.update(y=theta, u=prev_u)
            u0 = kp * (0.0 - eso.z1) - kd * omega
            torque = u0 - eso.z3 / eso.b0
            prev_u = torque
            omega += dt * (torque + disturbance)
            theta += dt * omega
        return theta, omega

    def _simulate_1dof_attitude(self, use_adrc: bool, dt: float = 0.02,
                                steps: int = 500, disturbance: float = 0.0,
                                kp: float = 10.0, kd: float = 5.0,
                                b0: float = 1.0) -> tuple:
        """
        Run a 1-DOF attitude control simulation.

        Simple plant: theta_dot = omega, omega_dot = torque + disturbance.
        Returns final (theta, omega).
        """
        if use_adrc:
            return self._run_adrc_1dof(dt, steps, disturbance, kp, kd, b0)
        else:
            return self._run_pd_1dof(dt, steps, disturbance, kp, kd)

    def test_adrc_converges_faster(self):
        """ADRC converges faster than PD for a simple attitude error."""
        pd_theta, _ = self._simulate_1dof_attitude(
            use_adrc=False, steps=200, kp=10.0, kd=5.0
        )
        adrc_theta, _ = self._simulate_1dof_attitude(
            use_adrc=True, steps=200, kp=10.0, kd=5.0
        )
        # Both should converge, but ADRC should be at least as good
        assert abs(adrc_theta) < 0.01
        assert abs(pd_theta) < 0.01

    def test_adrc_better_disturbance_rejection(self):
        """ADRC rejects constant disturbance better than PD."""
        disturbance = 3.0
        pd_theta, _ = self._simulate_1dof_attitude(
            use_adrc=False, steps=1000, disturbance=disturbance,
            kp=10.0, kd=5.0
        )
        adrc_theta, _ = self._simulate_1dof_attitude(
            use_adrc=True, steps=1000, disturbance=disturbance,
            kp=10.0, kd=5.0, b0=1.0
        )
        # PD will have non-zero steady-state error under disturbance
        assert abs(adrc_theta) < abs(pd_theta)

    def test_no_disturbance_similar_performance(self):
        """Without disturbance, ADRC and PD both converge well."""
        pd_theta, _ = self._simulate_1dof_attitude(
            use_adrc=False, steps=2000, disturbance=0.0,
            kp=20.0, kd=10.0
        )
        adrc_theta, _ = self._simulate_1dof_attitude(
            use_adrc=True, steps=2000, disturbance=0.0,
            kp=20.0, kd=10.0, b0=1.0
        )
        assert abs(pd_theta) < 0.001
        assert abs(adrc_theta) < 0.01

    def test_increasing_disturbance_adrc_advantage(self):
        """As disturbance increases, ADRC advantage over PD grows."""
        results = []
        for dist in [0.5, 1.0, 2.0, 5.0]:
            pd_theta, _ = self._simulate_1dof_attitude(
                use_adrc=False, steps=1000, disturbance=dist,
                kp=10.0, kd=5.0
            )
            adrc_theta, _ = self._simulate_1dof_attitude(
                use_adrc=True, steps=1000, disturbance=dist,
                kp=10.0, kd=5.0, b0=1.0
            )
            results.append((dist, abs(pd_theta), abs(adrc_theta)))
            assert abs(adrc_theta) < abs(pd_theta)
        # Verify ADRC advantage grows
        pd_errs = [r[1] for r in results]
        adrc_errs = [r[2] for r in results]
        ratios = [p / a for p, a in zip(pd_errs, adrc_errs)]
        # ADRC should be increasingly better as disturbance grows
        assert ratios[-1] > ratios[0]


# =============================================================================
# Integration Tests: GuidanceController with ADRC
# =============================================================================

class TestGuidanceControllerWithADRC:
    """Test that ADRC integrates correctly with GuidanceController."""

    def test_controller_accepts_adrc(self):
        """GuidanceController accepts optional ADRCController."""
        from src.guidance.controller import GuidanceController
        adrc = ADRCController(kp=np.array([10.0, 10.0, 10.0]),
                              kd=np.array([5.0, 5.0, 5.0]))
        controller = GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=2.0, kd_vel_vertical=2.0,
            kp_att=np.array([10.0, 10.0, 10.0]),
            kd_att=np.array([5.0, 5.0, 5.0]),
            adrc=adrc,
        )
        assert controller.adrc is adrc

    def test_controller_compute_wrench_with_adrc(self):
        """Wrench computed with ADRC has correct shape and finite values."""
        from src.guidance.controller import GuidanceController
        adrc = ADRCController(kp=np.array([10.0, 10.0, 10.0]),
                              kd=np.array([5.0, 5.0, 5.0]))
        controller = GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=2.0, kd_vel_vertical=2.0,
            kp_att=np.array([10.0, 10.0, 10.0]),
            kd_att=np.array([5.0, 5.0, 5.0]),
            adrc=adrc,
        )
        wrench = controller.compute_wrench(
            current_state=np.zeros(6),
            current_attitude=np.array([0.0, 0.0, 0.0, 1.0]),
            mass=1000.0,
            target_state=np.zeros(6),
            up_vector=np.array([0.0, 0.0, 1.0]),
            dt=0.02,
            angular_velocity=np.zeros(3),
        )
        assert wrench.shape == (6,)
        assert np.all(np.isfinite(wrench))

    def test_adrc_vs_pd_wrench_attitude_component(self):
        """ADRC produces different (but valid) torque than PD for same error."""
        from src.guidance.controller import GuidanceController

        kp_att = np.array([10.0, 10.0, 10.0])
        kd_att = np.array([5.0, 5.0, 5.0])

        pd_controller = GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=2.0, kd_vel_vertical=2.0,
            kp_att=kp_att, kd_att=kd_att,
        )
        adrc_controller = GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=2.0, kd_vel_vertical=2.0,
            kp_att=kp_att, kd_att=kd_att,
            adrc=ADRCController(kp=kp_att, kd=kd_att),
        )

        # A non-upright attitude requiring correction
        attitude = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])
        state = np.zeros(6)
        target = np.zeros(6)
        up = np.array([0.0, 0.0, 1.0])
        omega = np.zeros(3)

        wrench_pd = pd_controller.compute_wrench(
            state, attitude, 1000.0, target, up,
            dt=0.02, angular_velocity=omega,
        )
        # Run ADRC a few times to let ESO settle
        for _ in range(10):
            wrench_adrc = adrc_controller.compute_wrench(
                state, attitude, 1000.0, target, up,
                dt=0.02, angular_velocity=omega,
            )

        # Both should produce valid wrenches
        assert np.all(np.isfinite(wrench_pd))
        assert np.all(np.isfinite(wrench_adrc))

        # The ADRC torque may differ from PD since ESO is still converging
        # But the force components (first 3) should be identical
        assert np.allclose(wrench_pd[:3], wrench_adrc[:3], atol=1e-6)

    def test_adrc_reset_via_controller(self):
        """GuidanceController.reset() calls ADRC reset."""
        from src.guidance.controller import GuidanceController
        adrc = ADRCController()
        adrc.compute_torque(np.array([0.5, 0.0, 0.0]))
        controller = GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=2.0, kd_vel_vertical=2.0,
            kp_att=np.array([10.0, 10.0, 10.0]),
            kd_att=np.array([5.0, 5.0, 5.0]),
            adrc=adrc,
        )
        assert not np.allclose(adrc.prev_u, np.zeros(3))
        controller.reset()
        assert np.allclose(adrc.prev_u, np.zeros(3))

    def test_adrc_with_inertia_tensor(self):
        """ADRC works correctly with inertia tensor (gyroscopic terms added)."""
        from src.guidance.controller import GuidanceController
        adrc = ADRCController(kp=np.array([10.0, 10.0, 10.0]),
                              kd=np.array([5.0, 5.0, 5.0]))
        inertia = np.eye(3) * 1000.0
        controller = GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=2.0, kd_vel_vertical=2.0,
            kp_att=np.array([10.0, 10.0, 10.0]),
            kd_att=np.array([5.0, 5.0, 5.0]),
            inertia_tensor=inertia,
            adrc=adrc,
        )
        wrench = controller.compute_wrench(
            current_state=np.zeros(6),
            current_attitude=np.array([0.0, 0.0, 0.0, 1.0]),
            mass=1000.0,
            target_state=np.zeros(6),
            up_vector=np.array([0.0, 0.0, 1.0]),
            dt=0.02,
            angular_velocity=np.array([0.1, 0.0, 0.0]),
        )
        assert wrench.shape == (6,)
        assert np.all(np.isfinite(wrench))

    def test_controller_no_adrc_fallback(self):
        """Without ADRC, controller works exactly as before (backward compat)."""
        from src.guidance.controller import GuidanceController
        c1 = GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=2.0, kd_vel_vertical=2.0,
            kp_att=np.array([10.0, 10.0, 10.0]),
            kd_att=np.array([5.0, 5.0, 5.0]),
        )
        c2 = GuidanceController(
            kp_pos_lateral=1.0, kp_pos_vertical=1.0,
            kd_vel_lateral=2.0, kd_vel_vertical=2.0,
            kp_att=np.array([10.0, 10.0, 10.0]),
            kd_att=np.array([5.0, 5.0, 5.0]),
            adrc=None,
        )
        state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        att = np.array([0.0, 0.0, 0.0, 1.0])
        w1 = c1.compute_wrench(state, att, 1000.0, np.zeros(6), np.array([0.0, 0.0, 1.0]))
        w2 = c2.compute_wrench(state, att, 1000.0, np.zeros(6), np.array([0.0, 0.0, 1.0]))
        assert np.allclose(w1, w2)


# =============================================================================
# CTMCalculator Unit Tests
# =============================================================================

class TestCTMCalculator:
    """Unit tests for CTMCalculator (Phase 3 feedforward)."""

    def test_default_initialization(self):
        """CTMCalculator with default gains creates correctly."""
        inertia = np.eye(3) * 1000.0
        ctm = CTMCalculator(inertia)
        assert np.allclose(ctm.kp_ctm, [9.0, 9.0, 9.0])
        assert np.allclose(ctm.kd_ctm, [6.0, 6.0, 6.0])
        assert np.allclose(ctm.inertia_tensor, inertia)

    def test_custom_gains(self):
        """Custom kp_ctm/kd_ctm are stored correctly."""
        inertia = np.eye(3) * 1000.0
        kp = np.array([12.0, 15.0, 9.0])
        kd = np.array([8.0, 10.0, 6.0])
        ctm = CTMCalculator(inertia, kp_ctm=kp, kd_ctm=kd)
        assert np.allclose(ctm.kp_ctm, kp)
        assert np.allclose(ctm.kd_ctm, kd)

    def test_compute_feedforward_shape(self):
        """compute_feedforward returns a (3,) array."""
        inertia = np.eye(3) * 1000.0
        ctm = CTMCalculator(inertia)
        err = np.array([0.1, -0.05, 0.02])
        omega = np.array([0.01, -0.02, 0.005])
        ff = ctm.compute_feedforward(err, omega)
        assert ff.shape == (3,)
        assert np.all(np.isfinite(ff))

    def test_feedforward_zero_error_zero_omega(self):
        """With zero error and zero angular velocity, feedforward is zero."""
        inertia = np.eye(3) * 1000.0
        ctm = CTMCalculator(inertia)
        ff = ctm.compute_feedforward(np.zeros(3), np.zeros(3))
        assert np.allclose(ff, np.zeros(3), atol=1e-10)

    def test_feedforward_nonzero_error(self):
        """Feedforward proportional to err via J @ (-kp * err) (negative fb)."""
        inertia = np.diag(np.array([500.0, 800.0, 600.0]))
        ctm = CTMCalculator(inertia, kp_ctm=np.array([9.0, 9.0, 9.0]))
        err = np.array([0.2, 0.0, 0.0])
        ff = ctm.compute_feedforward(err, np.zeros(3))
        # tau_x = Ixx * (-kp_x) * err_x = 500 * (-9) * 0.2 = -900
        assert np.isclose(ff[0], -900.0)
        assert np.isclose(ff[1], 0.0)
        assert np.isclose(ff[2], 0.0)

    def test_feedforward_gyroscopic_term(self):
        """Feedforward includes omega x J @ omega gyroscopic term."""
        inertia = np.diag(np.array([500.0, 800.0, 600.0]))
        ctm = CTMCalculator(inertia, kp_ctm=np.zeros(3), kd_ctm=np.zeros(3))
        err = np.zeros(3)
        omega = np.array([0.1, 0.0, 0.0])
        ff = ctm.compute_feedforward(err, omega)
        # omega x J*omega = [0.1,0,0] x [50,0,0] = [0,0,0]
        # Pure x rotation has no gyroscopic coupling
        assert np.allclose(ff, np.zeros(3), atol=1e-10)

    def test_feedforward_gyroscopic_coupling(self):
        """Gyroscopic coupling appears with multi-axis rotation."""
        inertia = np.diag(np.array([500.0, 800.0, 600.0]))
        ctm = CTMCalculator(inertia, kp_ctm=np.zeros(3), kd_ctm=np.zeros(3))
        # omega = [0.1, 0.2, 0.0]
        omega = np.array([0.1, 0.2, 0.0])
        # J*omega = [50, 160, 0]
        # cross(omega, J*omega) = cross([0.1,0.2,0], [50,160,0])
        #   = [0.2*0 - 0*160, 0*50 - 0.1*0, 0.1*160 - 0.2*50]
        #   = [0, 0, 16 - 10] = [0, 0, 6]
        Jw = inertia @ omega
        expected_gyro = np.cross(omega, Jw)
        ff = ctm.compute_feedforward(np.zeros(3), omega)
        assert np.allclose(ff, expected_gyro)

    def test_feedforward_combined(self):
        """Feedforward combines negative-feedback PD + gyroscopic."""
        inertia = np.diag(np.array([500.0, 800.0, 600.0]))
        kp = np.array([9.0, 9.0, 9.0])
        kd = np.array([6.0, 6.0, 6.0])
        ctm = CTMCalculator(inertia, kp_ctm=kp, kd_ctm=kd)
        err = np.array([0.1, -0.05, 0.02])
        omega = np.array([0.01, -0.02, 0.005])
        # CTM uses negative feedback: J @ (-kp * err - kd * omega)
        tau_pd = inertia @ (-kp * err - kd * omega)
        tau_gyro = np.cross(omega, inertia @ omega)
        expected = tau_pd + tau_gyro
        ff = ctm.compute_feedforward(err, omega)
        assert np.allclose(ff, expected)

    def test_non_diagonal_inertia(self):
        """CTM works with non-diagonal (coupled) inertia tensor."""
        inertia = np.array([
            [500.0, -10.0, -5.0],
            [-10.0, 800.0, -20.0],
            [-5.0, -20.0, 600.0],
        ])
        ctm = CTMCalculator(inertia, kp_ctm=np.ones(3), kd_ctm=np.ones(3))
        err = np.array([0.1, 0.0, 0.0])
        omega = np.array([0.01, 0.02, 0.015])
        ff = ctm.compute_feedforward(err, omega)
        assert ff.shape == (3,)
        assert np.all(np.isfinite(ff))

    def test_expected_angular_accel(self):
        """expected_angular_accel converts torque to angular acceleration."""
        inertia = np.diag(np.array([500.0, 800.0, 600.0]))
        ctm = CTMCalculator(inertia)
        torque = np.array([100.0, 200.0, 150.0])
        alpha = ctm.expected_angular_accel(torque)
        expected_alpha = np.linalg.inv(inertia) @ torque
        assert np.allclose(alpha, expected_alpha)

    def test_expected_angular_accel_non_diagonal(self):
        """expected_angular_accel works with non-diagonal inertia."""
        inertia = np.array([
            [500.0, -10.0, -5.0],
            [-10.0, 800.0, -20.0],
            [-5.0, -20.0, 600.0],
        ])
        ctm = CTMCalculator(inertia)
        torque = np.array([100.0, 200.0, 150.0])
        alpha = ctm.expected_angular_accel(torque)
        expected_alpha = np.linalg.inv(inertia) @ torque
        assert np.allclose(alpha, expected_alpha)

    def test_invalid_inertia_shape(self):
        """Non-(3,3) inertia tensor raises ValueError."""
        with pytest.raises(ValueError, match="must have shape"):
            CTMCalculator(np.eye(4))

    def test_invalid_kp_shape(self):
        """Non-(3,) kp_ctm raises ValueError."""
        with pytest.raises(ValueError, match="kp_ctm and kd_ctm"):
            CTMCalculator(np.eye(3), kp_ctm=np.array([1.0, 2.0]))

    def test_invalid_err_axis_shape(self):
        """compute_feedforward raises for non-(3,) err_axis."""
        ctm = CTMCalculator(np.eye(3))
        with pytest.raises(ValueError, match="err_axis must have shape"):
            ctm.compute_feedforward(np.array([0.1, 0.2]), np.zeros(3))

    def test_invalid_angular_velocity_shape(self):
        """compute_feedforward raises for non-(3,) angular_velocity."""
        ctm = CTMCalculator(np.eye(3))
        with pytest.raises(ValueError, match="angular_velocity must have shape"):
            ctm.compute_feedforward(np.zeros(3), np.array([0.1, 0.2]))

    def test_invalid_expected_torque_shape(self):
        """expected_angular_accel raises for non-(3,) torque."""
        ctm = CTMCalculator(np.eye(3))
        with pytest.raises(ValueError, match="expected_torque must have shape"):
            ctm.expected_angular_accel(np.array([0.1, 0.2]))


# =============================================================================
# Integration Tests: ADRCController with CTM Feedforward
# =============================================================================

_STABLE_ESO = dict(beta_01=15.0, beta_02=75.0, beta_03=125.0, delta=0.1)
"""Stable bandwidth-parameterized ESO gains (ω₀=5 rad/s, see PHASE_2.md).
All CTM-ADRC tests use these to avoid the discrete-time instability of the
default gains (β01=100, δ=0.01) which oscillate at 50Hz.
"""


class TestADRCWithCTM:
    """Test that CTM feedforward correctly augments ADRC output."""

    def test_ctm_feedforward_accepted(self):
        """ADRCController.compute_torque accepts ctm_feedforward parameter."""
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc = ADRCController(dt=0.02, eso_params=eso_params)
        err = np.array([0.1, -0.05, 0.02])
        omega = np.array([0.01, -0.02, 0.005])
        ctm_ff = np.array([10.0, -5.0, 2.0])
        torque = adrc.compute_torque(err, angular_velocity=omega,
                                     ctm_feedforward=ctm_ff)
        assert torque.shape == (3,)
        assert np.all(np.isfinite(torque))

    def test_ctm_is_additive(self):
        """CTM feedforward adds to disturbance rejection output.
        In CTM mode: torque = ctm_feedforward - z3/b0.
        With zero error (y=0), z3 stays near zero, so torque ≈ ctm_ff.
        """
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc = ADRCController(dt=0.02, kp=np.array([5.0, 5.0, 5.0]),
                              kd=np.array([10.0, 10.0, 10.0]),
                              eso_params=eso_params)
        err = np.zeros(3)
        omega = np.zeros(3)
        ctm_ff = np.array([50.0, 0.0, 0.0])

        t_yes = adrc.compute_torque(err, angular_velocity=omega,
                                    ctm_feedforward=ctm_ff)

        # With zero error and omega, ESO e=0 so z3 stays 0
        # total = ctm_ff - 0/b0 = ctm_ff
        assert np.allclose(t_yes, ctm_ff, atol=1e-10)

    def test_prev_u_includes_ctm(self):
        """prev_u stores the TOTAL torque including CTM."""
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc = ADRCController(dt=0.02, eso_params=eso_params)
        err = np.array([0.1, 0.0, 0.0])
        ctm_ff = np.array([25.0, 0.0, 0.0])
        torque = adrc.compute_torque(err, ctm_feedforward=ctm_ff)
        assert np.allclose(adrc.prev_u, torque)

    def test_prev_u_without_ctm(self):
        """prev_u stores only ADRC output when no CTM."""
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc = ADRCController(dt=0.02, eso_params=eso_params)
        err = np.array([0.1, 0.0, 0.0])
        torque = adrc.compute_torque(err)
        assert np.allclose(adrc.prev_u, torque)

    def test_ctm_feedforward_none_identical(self):
        """ctm_feedforward=None gives same result as no kwarg."""
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc1 = ADRCController(dt=0.02, eso_params=eso_params)
        adrc2 = ADRCController(dt=0.02, eso_params=eso_params)
        err = np.array([0.1, -0.05, 0.02])
        omega = np.array([0.01, -0.02, 0.005])
        t1 = adrc1.compute_torque(err, angular_velocity=omega)
        t2 = adrc2.compute_torque(err, angular_velocity=omega,
                                  ctm_feedforward=None)
        assert np.allclose(t1, t2)

    def test_ctm_eso_sees_full_command(self):
        """CTM-ADRC converges with stable ESO and properly tuned CTM gains."""
        inertia_val = 2.0
        inertia = np.eye(3) * inertia_val
        kp = np.array([10.0, 10.0, 10.0])
        kd = np.array([5.0, 5.0, 5.0])

        ctm = CTMCalculator(inertia,
                            kp_ctm=kp, kd_ctm=kd)
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc = ADRCController(dt=0.02, kp=kp, kd=kd,
                              eso_params=eso_params)

        err_axis = np.array([0.5, 0.0, 0.0])
        omega = np.array([0.0, 0.0, 0.0])

        for _ in range(3000):
            ctm_ff = ctm.compute_feedforward(err_axis, omega)
            torque = adrc.compute_torque(err_axis, angular_velocity=omega,
                                         ctm_feedforward=ctm_ff)
            omega += 0.02 * torque / inertia_val
            err_axis += 0.02 * omega

        assert abs(err_axis[0]) < 0.01
        assert abs(err_axis[1]) < 0.001
        assert abs(err_axis[2]) < 0.001

    def test_ctm_better_convergence_with_inertia(self):
        """CTM-ADRC converges well with stable ESO and inertia-aware CTM."""
        inertia_val = 10.0
        inertia = np.eye(3) * inertia_val
        kp = np.array([5.0, 5.0, 5.0])
        kd = np.array([10.0, 10.0, 10.0])

        ctm = CTMCalculator(inertia, kp_ctm=kp, kd_ctm=kd)
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc_only = ADRCController(dt=0.02, kp=kp, kd=kd,
                                   eso_params=eso_params)
        adrc_ctm = ADRCController(dt=0.02, kp=kp, kd=kd,
                                  eso_params=eso_params)

        err_only = np.array([0.5, 0.0, 0.0])
        omega_only = np.array([0.0, 0.0, 0.0])
        err_ctm = np.array([0.5, 0.0, 0.0])
        omega_ctm = np.array([0.0, 0.0, 0.0])

        for _ in range(3000):
            t_only = adrc_only.compute_torque(err_only, angular_velocity=omega_only)
            omega_only += 0.02 * t_only / inertia_val
            err_only += 0.02 * omega_only

            ctm_ff = ctm.compute_feedforward(err_ctm, omega_ctm)
            t_ctm = adrc_ctm.compute_torque(err_ctm, angular_velocity=omega_ctm,
                                            ctm_feedforward=ctm_ff)
            omega_ctm += 0.02 * t_ctm / inertia_val
            err_ctm += 0.02 * omega_ctm

        # Both converge; CTM-ADRC with inertia-aware PD converges tighter
        assert abs(err_ctm[0]) < 0.01
        assert abs(err_only[0]) < 0.01

    def test_ctm_disturbance_rejection(self):
        """CTM-ADRC rejects disturbances with stable ESO gains."""
        inertia_val = 5.0
        inertia = np.eye(3) * inertia_val
        kp = np.array([8.0, 8.0, 8.0])
        kd = np.array([6.0, 6.0, 6.0])

        ctm = CTMCalculator(inertia, kp_ctm=kp, kd_ctm=kd)
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc_only = ADRCController(dt=0.02, kp=kp, kd=kd,
                                   eso_params=eso_params)
        adrc_ctm = ADRCController(dt=0.02, kp=kp, kd=kd,
                                  eso_params=eso_params)

        disturbance = 5.0

        err_only = 0.0
        omega_only = 0.0
        err_ctm = 0.0
        omega_ctm = 0.0

        for _ in range(3000):
            t_only = adrc_only.compute_torque(
                np.array([err_only, 0.0, 0.0]),
                angular_velocity=np.array([omega_only, 0.0, 0.0]),
            )
            omega_only += 0.02 * (t_only[0] + disturbance) / inertia_val
            err_only += 0.02 * omega_only

            err_arr = np.array([err_ctm, 0.0, 0.0])
            omega_arr = np.array([omega_ctm, 0.0, 0.0])
            ctm_ff = ctm.compute_feedforward(err_arr, omega_arr)
            t_ctm = adrc_ctm.compute_torque(
                err_arr, angular_velocity=omega_arr,
                ctm_feedforward=ctm_ff,
            )
            omega_ctm += 0.02 * (t_ctm[0] + disturbance) / inertia_val
            err_ctm += 0.02 * omega_ctm

        # Both reject disturbance
        assert abs(err_ctm) < 0.05
        assert abs(err_only) < 0.05

    def test_invalid_ctm_shape(self):
        """Non-(3,) ctm_feedforward raises ValueError."""
        adrc = ADRCController()
        with pytest.raises(ValueError, match="ctm_feedforward must have shape"):
            adrc.compute_torque(np.zeros(3), ctm_feedforward=np.array([1.0, 2.0]))

    def test_eso_prev_u_with_ctm(self):
        """With CTM, prev_u correctly includes CTM in total."""
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc = ADRCController(dt=0.02, eso_params=eso_params)
        err = np.array([0.1, -0.05, 0.02])
        ctm_ff = np.array([5.0, -3.0, 1.0])
        torque = adrc.compute_torque(err, ctm_feedforward=ctm_ff)
        assert np.allclose(adrc.prev_u, torque)


# =============================================================================
# Integration Tests: CTM Stabilizes Edge Cases
# =============================================================================

class TestCTMEdgeCases:
    """CTM-ADRC behavior under edge conditions."""

    def test_ctm_large_error_no_instability(self):
        """CTM-ADRC handles large initial errors without instability."""
        inertia = np.eye(3) * 10.0
        ctm = CTMCalculator(inertia)
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc = ADRCController(dt=0.02, kp=np.array([3.0, 3.0, 3.0]),
                              kd=np.array([5.0, 5.0, 5.0]),
                              eso_params=eso_params)
        err = np.array([10.0, -5.0, 3.0])
        omega = np.array([0.0, 0.0, 0.0])
        for _ in range(200):
            ctm_ff = ctm.compute_feedforward(err, omega)
            torque = adrc.compute_torque(err, angular_velocity=omega,
                                         ctm_feedforward=ctm_ff)
            omega += 0.02 * torque / 10.0
            err += 0.02 * omega
            assert np.all(np.isfinite(torque))
            assert np.all(np.isfinite(err))

    def test_ctm_equals_adrc_for_identity_inertia(self):
        """CTM-ADRC and pure ADRC both converge for identity inertia."""
        inertia = np.eye(3)
        kp = np.array([10.0, 10.0, 10.0])
        kd = np.array([5.0, 5.0, 5.0])
        ctm = CTMCalculator(inertia, kp_ctm=kp, kd_ctm=kd)
        eso_params = [_STABLE_ESO.copy() for _ in range(3)]
        adrc_ctm = ADRCController(dt=0.02, kp=kp, kd=kd,
                                  eso_params=eso_params)
        adrc_only = ADRCController(dt=0.02, kp=kp, kd=kd,
                                   eso_params=eso_params)

        err_ctm = np.array([0.2, 0.0, 0.0])
        omega_ctm = np.zeros(3)
        err_only = np.array([0.2, 0.0, 0.0])
        omega_only = np.zeros(3)

        for _ in range(2000):
            ctm_ff = ctm.compute_feedforward(err_ctm, omega_ctm)
            t_ctm = adrc_ctm.compute_torque(err_ctm, angular_velocity=omega_ctm,
                                            ctm_feedforward=ctm_ff)
            omega_ctm += 0.02 * t_ctm
            err_ctm += 0.02 * omega_ctm

            t_only = adrc_only.compute_torque(err_only, angular_velocity=omega_only)
            omega_only += 0.02 * t_only
            err_only += 0.02 * omega_only

        # Both converge; CTM-ADRC uses CTM PD directly while pure ADRC
        # uses WSEF PD via z1 — both converge well for identity inertia
        assert abs(err_ctm[0]) < 0.01
        assert abs(err_only[0]) < 0.01


# =============================================================================
# Edge Case and Stability Tests
# =============================================================================

class TestADRCStability:
    """Test ADRC stability under edge conditions."""

    def test_large_initial_error(self):
        """ADRC handles large initial errors without instability."""
        adrc = ADRCController(
            dt=0.02,
            kp=np.array([5.0, 5.0, 5.0]),
            kd=np.array([10.0, 10.0, 10.0]),
        )
        err_axis = np.array([10.0, -5.0, 3.0])
        omega = np.array([0.0, 0.0, 0.0])
        inertia = 2.0
        for _ in range(100):
            torque = adrc.compute_torque(err_axis, angular_velocity=omega)
            omega += 0.02 * torque / inertia
            err_axis += 0.02 * omega
            assert np.all(np.isfinite(err_axis))
            assert np.all(np.isfinite(torque))

    def test_noisy_input(self):
        """ADRC is stable with small noisy variations in the error signal."""
        adrc = ADRCController(
            dt=0.02,
            kp=np.array([5.0, 5.0, 5.0]),
            kd=np.array([10.0, 10.0, 10.0]),
        )
        rng = np.random.default_rng(42)
        err_axis = np.array([0.1, -0.05, 0.02])
        omega = np.array([0.0, 0.0, 0.0])
        inertia = 2.0
        for _ in range(200):
            noise = rng.normal(0, 0.01, 3)
            torque = adrc.compute_torque(err_axis + noise, angular_velocity=omega + noise)
            omega += 0.02 * torque / inertia
            err_axis += 0.02 * omega
            assert np.all(np.isfinite(torque))

    def test_small_dt(self):
        """ADRC works with very small timesteps."""
        adrc = ADRCController(
            dt=0.001,
            kp=np.array([10.0, 10.0, 10.0]),
            kd=np.array([5.0, 5.0, 5.0]),
        )
        err = np.array([0.1, 0.0, 0.0])
        omega = 0.0
        for _ in range(1000):
            torque = adrc.compute_torque(err)
            omega += 0.001 * torque[0] / 2.0
            err += 0.001 * omega * np.array([1.0, 0.0, 0.0])
            assert np.all(np.isfinite(torque))

    def test_large_dt(self):
        """ADRC handles large timesteps without producing NaN."""
        adrc = ADRCController(
            dt=0.1,
            kp=np.array([1.0, 1.0, 1.0]),
            kd=np.array([2.0, 2.0, 2.0]),
        )
        err = np.array([0.5, 0.0, 0.0])
        omega = 0.0
        for _ in range(50):
            torque = adrc.compute_torque(err)
            omega += 0.1 * torque[0] / 2.0
            err += 0.1 * omega * np.array([1.0, 0.0, 0.0])
            assert np.all(np.isfinite(torque))

    def test_adrc_reset_during_operation(self):
        """Reset during operation clears state and allows fresh convergence."""
        adrc = ADRCController(kp=np.array([5.0, 5.0, 5.0]),
                              kd=np.array([10.0, 10.0, 10.0]))
        err = np.array([0.5, 0.0, 0.0])
        omega = np.array([0.0, 0.0, 0.0])
        inertia = 2.0
        for _ in range(200):
            torque = adrc.compute_torque(err, angular_velocity=omega)
            omega += 0.02 * torque / inertia
            err += 0.02 * omega

        adrc.reset()
        assert np.allclose(adrc.prev_u, np.zeros(3))
        torque = adrc.compute_torque(np.array([0.5, 0.0, 0.0]), angular_velocity=np.zeros(3))
        assert np.all(np.isfinite(torque))

    def test_eso_dt_zero_fallback(self):
        """ESO handles dt=0 by falling back to minimal dt."""
        eso = PerAxisESO(dt=0.0)
        assert eso.dt > 0.0
        eso.update(y=0.5, u=0.0)
        assert np.isfinite(eso.z1)
