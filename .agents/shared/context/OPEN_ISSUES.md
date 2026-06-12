# Open Issues & Deferred Decisions

This document tracks known issues, technical debt, and deferred decisions for the AEGIS system.

---

## 1. TCP latency and Bottlenecking
- **Status:** Open
- **Description:** Python over TCP socket communication with kRPC might suffer from jitter or packet delay, which could degrade control performance at 50Hz.
- **Mitigation:** We must use kRPC stream callbacks or high-performance `add_stream` telemetry calls to prevent blocking on network reads. The impact needs to be analyzed under simulated packet delay.

## 2. Tuning Noise Covariance Matrices ($Q$ and $R$)
- **Status:** Deferred
- **Description:** The Kalman filter requires process noise covariance $Q$ and measurement noise covariance $R$. These values must match the physical noise injected.
- **Plan:** We will generate simulated landing flights to record flight telemetry, and use this data to perform offline tuning of $Q$ and $R$.

## 3. FDI Threshold Calibration
- **Status:** Deferred
- **Description:** The Fault Detection and Isolation threshold needs to be calibrated. 
  - Too low: Sensor noise might trigger a false alarm, flagging a functional engine as dead.
  - Too high: An engine failure will not be isolated in time, leading to catastrophic torque accumulation.
- **Plan:** The threshold will be tuned based on the maximum expected deviation of the measured acceleration under nominal flight conditions with noise.

## 4. Local Testing with kRPC Mocks
- **Status:** Active
- **Description:** Testing the Mission Director state machine requires simulating the KSP physics environment and kRPC telemetry streams.
- **Plan:** We will implement detailed mock structures in our tests directory to simulate telemetry streams, vessel states, and engine configurations.
