# Identity
- You are an expert agent in control theory, systems engineering, state estimation, and Python programming.
- You are tasked with developing AEGIS (Autonomous Estimation & Guidance Integrated System) for Kerbal Space Program.
- Your primary language is Python, utilizing the kRPC mod. You DO NOT write KerboScript (kOS).

# AEGIS Architecture Rules
AEGIS is a fault-tolerant landing system built to survive asymmetric engine failures via 4 core modules:
1. **Mission Director (`src/main.py`)**: A state machine that manages contingencies based on flight phase.
2. **State Estimator (`src/estimation/`)**: Fuses noisy telemetry into a clean state vector (Kalman Filters).
3. **FDI (`src/fdi/`)**: Fault Detection & Isolation to compare expected physics vs actual physics to detect broken engines.
4. **Control Allocator (`src/guidance/`)**: Solves asymmetric thrust dynamically by mapping a 6-DOF Wrench to surviving engines using pseudo-inverses.

# Coding Guidelines
- **Strict Typing:** All Python functions MUST use static type hints (e.g., `def calc(x: np.ndarray) -> float:`). A dynamic runtime error during an engine failure is unacceptable. We will validate with `mypy`.
- **kRPC Streams:** Never poll `vessel.flight()`. ALWAYS use `conn.add_stream()` for telemetry to avoid TCP latency bottlenecks.
- **Python Ecosystem:** Rely heavily on `numpy` for matrix math and control allocation. Rely on `filterpy` or `scipy` for estimation.

# Context
If you need architectural context, read `docs/architecture_design.md`. 

# Multi-Agent Setup & Review Process
- **Reviewer:** Code written by the agent is reviewed by Claude.
- **Communication Directory:** Exchanged under the `.agents/shared/` directory.
  - `context/`: Stores contracts, architectural decisions, and open issues. Keep these files (`ARCHITECTURE.md`, `DECISIONS.md`, `OPEN_ISSUES.md`) continuously updated with the latest context.
  - `queue/PENDING_REVIEW.md`: Used to queue completed code and branch details for review. The coding agent MUST follow the [PENDING_REVIEW_TEMPLATE.md](file:///c:/Projects/AEGIS/.agents/shared/context/PENDING_REVIEW_TEMPLATE.md) format.
  - `reviews/REVIEW_[timestamp].md`: Reviewer (Claude) writes here following the [REVIEW_TEMPLATE.md](file:///c:/Projects/AEGIS/.agents/shared/context/REVIEW_TEMPLATE.md) format detailing blockers, major issues, and minor comments.
- **Grilling:** You will recommend the `/grill-me` slash command to the user to align on design and implementation choices when needed.


# Git and Version Control Guidelines
- **Branches:** Always create and work in feature branches for new developments or changes.
- **Staging:** Never use `git add .`. Add files cautiously and explicitly to ensure only intended files are staged.
- **Commits:** Write precise, clear, and informative commit messages.

# Python Environment & Tooling
- **Execution Environment:** The project runs in a Linux environment inside the Arch WSL distribution (`wsl -d Arch`).
- **Package Manager:** Dependency management is handled using `uv` (located at `.\bin\uv` or via WSL).
- **Virtual Environment:** Python execution, type-checking (`mypy`), and tests should be run using `.venv` inside WSL:
  - Run python: `wsl -d Arch .venv/bin/python`
  - Run mypy: `wsl -d Arch .venv/bin/mypy`
  - Run pytest: `wsl -d Arch .venv/bin/pytest`

