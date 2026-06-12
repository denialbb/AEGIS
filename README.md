# AEGIS: Autonomous Estimation & Guidance Integrated System

AEGIS is an advanced, fault-tolerant autonomous mission control system for Kerbal Space Program. It is designed to survive catastrophic asymmetric engine failures during powered descent by utilizing aerospace-grade state estimation, fault detection, and dynamic control allocation.

## Architecture
Built entirely in Python using the **kRPC** framework, AEGIS breaks away from traditional "perfect physics" KSP scripts.
- **Mission Director**: High-level hierarchical state machine for contingencies.
- **State Estimation**: Fuses deliberately noisy radar and IMU data to estimate true state.
- **Fault Detection (FDI)**: Monitors deviations between commanded thrust and measured acceleration to detect dead engines.
- **Control Allocator**: Uses pseudo-inverse matrix math to dynamically map a 6-DOF control wrench to surviving engine vectors, preventing catastrophic torque.
