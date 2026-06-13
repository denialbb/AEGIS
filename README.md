# AEGIS: Autonomous Estimation & Guidance Integrated System

AEGIS (Autonomous Estimation & Guidance Integrated System) is a fault-tolerant landing system for Kerbal Space Program.

## 1. Project Objective

Most Kerbal Space Program automation scripts treat guidance as an isolated solver operating in a perfect, deterministic physics environment. This project actively rejects that premise.

The objective is to build an autonomous mission architecture that deliberately blinds itself with artificial noise and suffers from catastrophic asymmetric engine failures during powered descent. The system must utilize aerospace-grade State Estimation, Fault Detection, and Control Allocation to dynamically adapt and survive, guided by a robust, high-level contingency state machine.

## 2. Real-World Inspiration

To effectively test the control allocation algorithms, AEGIS expects a redundant, multi-engine vehicle layout. This design mimics real-world spacecraft that rely on differential throttling and gimbaling to maintain control after engine failures.

![SpaceX Crew Dragon](/c:/Projects/AEGIS/docs/images/crew_dragon.jpg)
*SpaceX Crew Dragon with its 8x SuperDraco engine layout.*

![Blue Moon Lander](/c:/Projects/AEGIS/docs/images/blue_moon.png)
*Blue Origin Blue Moon MK2 Lunar Lander concept.*

## 3. System Architecture

The architecture is strictly decoupled into four primary domains to ensure robust systems engineering. The system breaks out of the native KerboScript (kOS) virtual machine and utilizes the kRPC mod to stream game data over TCP/IP to a local Python server. Python provides access to the powerful `numpy` and `filterpy` libraries, which are essential for matrix math and state estimation.

### The Mission Director (Hierarchical State Machine)
The overarching logic controller manages nominal mission phases and handles contingency branching based on the severity and timing of a detected fault. For instance, an engine failure during high-altitude powered descent might cause the Director to shift the landing target to a closer safe zone. Conversely, a failure during terminal descent (< 50m) triggers a "Hard Abort" contingency, maximizing vertical thrust to survive impact regardless of lateral drift.

### State Estimation Module (Navigation)
To simulate real-world sensor imperfections, the kRPC telemetry streams are deliberately obscured. The system injects continuous Gaussian noise into the radar altimeter and accelerometer readings. A Discrete-Time Kalman Filter then fuses these noisy measurements to produce a clean, probabilistic estimation of the true state vector.

### Fault Detection & Isolation Module (FDI)
This module acts as the diagnostic nervous system. It continuously calculates the expected acceleration vector based on commanded throttle and vessel mass. By comparing this expected acceleration against the measured acceleration from the State Estimator, the FDI can detect deviations. Once a deviation exceeds the noise tolerance threshold, the FDI isolates the failing engine, flags it as "Dead", and alerts the Mission Director.

### Control Allocation Module (Guidance & Control)
When an off-center engine dies, simply throttling up the remaining engines induces catastrophic torque. The guidance algorithm solves this by commanding a desired 6-DOF Wrench (Forces and Torques) instead of individual engines. A pseudo-inverse matrix solver maps this desired Wrench to the surviving engines. It automatically throttles down engines opposite the failure to kill the torque, while adjusting adjacent engines to maintain the required vertical thrust.

## 4. Setup and Execution

The execution environment for AEGIS is strictly contained within a Linux environment using WSL (Windows Subsystem for Linux) with the Arch distribution. Dependency management and virtual environments are handled by `uv`.

### Prerequisites
- Kerbal Space Program with the **kRPC** mod installed and server running.
- Windows Subsystem for Linux (WSL) running the **Arch** distribution.
- `uv` installed in WSL or `.\bin\uv`.

### Installation
Clone the repository and set up the virtual environment:
```bash
wsl -d Arch
uv venv
# Ensure dependencies are installed
uv pip install -r requirements.txt
```

### Running the System
Execution, type-checking, and tests must be run using the `.venv` inside WSL:

**Run the Mission Director:**
```bash
wsl -d Arch .venv/bin/python src/main.py
```

**Run Static Analysis:**
```bash
wsl -d Arch .venv/bin/mypy .
```

**Run Tests:**
```bash
wsl -d Arch .venv/bin/pytest
```

## 5. The Gremlin (Live Testing)

To properly test the system's resilience, we use a lightweight background script known as "The Gremlin". This script runs either in kOS or as a separate Python thread during flight tests. Its sole purpose is to act as an unpredictable adversary—randomly selecting an engine part module and forcing its thrust limit to zero or shutting it down entirely. This forced failure triggers the FDI module in real-time, allowing us to observe the Control Allocator and Mission Director reacting to sudden catastrophic events.

## 6. Documentation

For a deeper dive into the architectural design and the specifics of the AEGIS Test Vehicle (ATV), refer to the following documents:
- [[Architecture Design]](file:///c:/Projects/AEGIS/docs/architecture_design.md)
- [[Vessel Design]](file:///c:/Projects/AEGIS/docs/vessel_design.md)
