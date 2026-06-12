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

---

## ADR-005: 3D Force Control Allocation for Gimbaled Engines

### Context
The vessel's main engines are gimbaled, meaning they can vector their thrust. The allocator needs to compute both the throttle settings and the 2-axis gimbal angles for all active engines to control both net force and net torque (attitude).

### Decision
We treat each engine's control input as a 3D thrust force vector $\mathbf{f}_i = [f_{x,i}, f_{y,i}, f_{z,i}]^T \in \mathbb{R}^3$. For $N$ engines, the control vector is $\mathbf{u} \in \mathbb{R}^{3N}$. The control effectiveness matrix $\mathbf{B}$ has shape $6 \times 3N$, where each engine's block is:
$$\mathbf{B}_i = \begin{bmatrix} \mathbf{I}_{3\times 3} \\ [\mathbf{r}_i]_\times \end{bmatrix}$$
We solve for $\mathbf{u}$ using `numpy.linalg.pinv` and then map the 3D force vector $\mathbf{f}_i$ for each engine back to physical throttle and gimbal angles:
- Throttle: $T_i = \|\mathbf{f}_i\| / F_{max,i}$
- Gimbal X: $\theta_{x,i} = \arcsin(-f_{y,i}/\|\mathbf{f}_i\|)$
- Gimbal Y: $\theta_{y,i} = \arcsin(f_{x,i}/\|\mathbf{f}_i\|)$

### Rationale
This formulation keeps the mapping linear and allows us to use the standard pseudo-inverse control allocation algorithm directly, without needing non-linear optimization solvers.

---

## ADR-006: Target-Relative Local Cartesian Tangent Plane

### Context
Guidance calculations and trajectory tracking are mathematically simpler in a local flat-plane coordinate system compared to planetary spherical or body-centric Cartesian systems.

### Decision
We define the landing target on the surface as the origin $(0, 0, 0)$. Position and velocity vectors are represented in a local Cartesian tangent plane (e.g. North-East-Down or target-relative frame) fixed to the rotating surface of the celestial body.

### Rationale
Using a target-relative local frame simplifies the state estimator and control equations, as the target position is always constant at the origin.

---

## ADR-007: Local Gravity Vector Querying

### Context
Gravity changes as the vessel descends. The State Estimator needs to know the gravity vector to propagate acceleration correctly.

### Decision
We query the exact local gravity vector dynamically from the kRPC API based on the vessel's current position relative to the celestial body. We use this gravity vector in the state transition step of the Kalman Filter.

### Rationale
This ensures high accuracy without needing to model the complex celestial body physics or gravity fields locally in our code.

