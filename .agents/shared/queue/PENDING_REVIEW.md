# Pending Review

## Branch Info
- **Branch:** `feature/setup-multi-agent-context`
- **Changed Files:**
  - [.gitignore](file:///c:/Projects/AEGIS/.gitignore)
  - [GEMINI.md](file:///c:/Projects/AEGIS/GEMINI.md)
  - [.agents/shared/context/ARCHITECTURE.md](file:///c:/Projects/AEGIS/.agents/shared/context/ARCHITECTURE.md)
  - [.agents/shared/context/DECISIONS.md](file:///c:/Projects/AEGIS/.agents/shared/context/DECISIONS.md)
  - [.agents/shared/context/OPEN_ISSUES.md](file:///c:/Projects/AEGIS/.agents/shared/context/OPEN_ISSUES.md)

## Notes & Self-Reflection
1. We have initialized the git repository and configured `.gitignore` to avoid tracking virtual environment files.
2. We have successfully installed the required Python packages (`filterpy`, `mypy`) inside the Arch WSL environment.
3. We have documented the initial API contracts in `ARCHITECTURE.md`, decisions in `DECISIONS.md`, and open issues in `OPEN_ISSUES.md`.
4. We conducted a design alignment session ("grilling") and resolved key decisions:
   - **Target-Relative coordinates**: Position/velocity are relative to a stationary landing target $(0,0,0)$ on the tangent plane of the celestial body.
   - **Gimbaled engine allocation**: Since engines are gimbaled, we treat each engine's control inputs as a 3D thrust vector $\mathbf{f}_i = [f_{x,i}, f_{y,i}, f_{z,i}]^T$. We build a linear $6 \times 3N$ control effectiveness matrix $\mathbf{B}$ and use pseudo-inverse allocation to find $\mathbf{u} \in \mathbb{R}^{3N}$, which we then map to physical throttles and X/Y gimbal angles.
   - **Local gravity**: We dynamically query the local gravity vector from kRPC and feed it to the estimator state transition.
5. Before we start coding the actual modules, we want to align with the reviewer (Claude) on the proposed API contracts and ADRs in `ARCHITECTURE.md` and `DECISIONS.md`.

Please review these contracts and decisions.

