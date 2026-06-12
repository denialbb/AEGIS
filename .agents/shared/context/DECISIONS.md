# Architecture Decision Records (ADRs)

This document tracks the key architectural decisions made for the AEGIS system, including the rationale, alternatives considered, and implications.

---

## ADR-001: kRPC Integration with Python

### Context
Kerbal Space Program's native VM runs KerboScript (kOS). However, implementing linear algebra solvers (such as pseudo-inverses for Control Allocation) and complex state estimators (Kalman Filters) is extremely challenging in kOS due to the lack of library support, lack of type systems, and CPU cycle limits per physics tick.

### Decision
We use the **kRPC** mod to expose a TCP socket and control the vessel using a local Python script running inside WSL (`Arch` distribution).

### Rationale
- Python offers mature scientific libraries (`numpy`, `scipy`, `filterpy`) that make complex control and estimation implementation straightforward.
- A localhost TCP stream can update at 50Hz (20ms/tick) with latency under 2ms, leaving plenty of headroom.
- Python supports static type hints which can be checked using `mypy` to prevent dynamic runtime type errors.

---

## ADR-002: Discrete-Time Kalman Filter for State Estimation

### Context
KSP provides perfect state information (exact position, velocity, and acceleration vectors). To simulate a real aerospace environment, we must deliberately obscure this telemetry and reconstruct the state.

### Decision
We will wrap kRPC streams in a noise-generating function that adds Gaussian white noise to telemetry (e.g. radar altitude, accelerometer). We will use a Discrete-Time Kalman Filter to reconstruct the state vector:
$$\mathbf{x} = \begin{bmatrix} X & Y & Z & V_x & V_y & V_z & Mass \end{bmatrix}^T$$

### Rationale
- The Kalman Filter provides an optimal estimator under linear systems and Gaussian noise.
- Since descent is approximately linear or locally linear over short intervals, a linear KF or EKF provides high fidelity without the computational overhead of particle filters.

---

## ADR-003: Pseudo-Inverse Control Allocation (Wrench Mapping)

### Context
When an engine fails, it creates an asymmetric thrust vector, generating a destabilizing torque. Direct mapping of throttle commands from a standard autopilot would spin the vessel out of control.

### Decision
We command a desired 6-DOF Wrench:
$$\mathbf{W} = \begin{bmatrix} F_x & F_y & F_z & \tau_x & \tau_y & \tau_z \end{bmatrix}^T$$
And use a control effectiveness matrix $\mathbf{B}$ to map the individual engine thrusts $\mathbf{u}$ to the wrench:
$$\mathbf{W} = \mathbf{B} \mathbf{u}$$
Since we have redundant engines, we solve for $\mathbf{u}$ using the Moore-Penrose pseudo-inverse of $\mathbf{B}$:
$$\mathbf{u} = \mathbf{B}^\dagger \mathbf{W}$$
where $\mathbf{B}^\dagger = \mathbf{B}^T (\mathbf{B} \mathbf{B}^T)^{-1}$ is computed via `numpy.linalg.pinv`.

### Rationale
- Pseudo-inverse control allocation naturally handles redundant actuators.
- When an engine fails, the FDI flags it as inactive. We remove the corresponding column from $\mathbf{B}$ and recompute the pseudo-inverse. The allocator automatically redistributes the thrust to surviving engines, balancing torque to prevent spins.

---

## ADR-004: Arch WSL Execution Environment with `uv`

### Context
To run the code, we need a consistent development and execution environment. 

### Decision
We execute all python code, tests, and static analysis inside the Arch WSL distribution using `uv` to manage dependencies.

### Rationale
- Arch WSL provides a fast, standardized Linux environment.
- `uv` is a blazingly fast, modern Python package manager that handles virtual environments and dependency resolution reliably.
