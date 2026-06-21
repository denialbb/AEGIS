---
name: run-tests
description: Runs AEGIS unit tests (pytest) and static type checking (mypy) across platforms (Windows, Linux, macOS, WSL2).
---

# `run-tests` Skill

This skill automates the execution of static type checking and unit tests for the AEGIS project. It is designed to work across multiple operating systems and AI agent environments (including OpenCode).

## When to Use This Skill
Use this skill when you want to verify code correctness, validate type safety after code changes, or ensure that refactored code has not broken existing unit tests.

## Instructions

When asked to run tests, perform type checking, or run static analysis:

1. **Detect Operating System and Platform:**
   Determine the current operating environment. Look for active indicators (e.g. file paths using `/` vs `\`, shell types, or env variables).

2. **Select Virtual Environment Executable Paths:**
   Locate the correct Python environment binaries:
   - **Linux / macOS / WSL:** Use `.venv/bin/` paths (e.g. `.venv/bin/mypy` and `.venv/bin/pytest`).
   - **Windows (Native Command Prompt or PowerShell):** Use `.venv\Scripts\` paths (e.g. `.venv\Scripts\mypy` and `.venv\Scripts\pytest`).

3. **Check for WSL Environment Rules:**
   If the repository has project-level rules (like [AGENTS.md](file:///c:/Projects/AEGIS/AGENTS.md)) specifying that the code must be run inside a specific WSL distribution (e.g., `wsl -d Arch`), prepend command executions with `wsl -d Arch` (e.g., `wsl -d Arch .venv/bin/mypy .`).

4. **Execute Static Type Checking (mypy):**
   Run mypy on the codebase to check type annotations:
   - Linux/macOS/WSL: `.venv/bin/mypy .`
   - Windows: `.venv\Scripts\mypy .`
   Note any type failures or implicit `Any` errors.

5. **Execute Unit Tests (pytest):**
   - **CRITICAL:** Always specify a test file or directory (e.g., `tests/`). Never run `pytest` from the root folder without arguments.
   - Linux/macOS/WSL: `.venv/bin/pytest tests/` (or specify a specific file: `.venv/bin/pytest tests/test_allocator.py`)
   - Windows: `.venv\Scripts\pytest tests\` (or specify a specific file: `.venv\Scripts\pytest tests\test_allocator.py`)

6. **Report Results:**
   Summarize the output of both checks. List any failing test cases, type checking violations, and suggest fixes for any issues found.
