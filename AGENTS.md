# Repository Guidelines

## Project Structure & Module Organization
- Core Python package: `src/claudia/`.
- Brain and safety pipeline: `src/claudia/brain/` (for example `production_brain.py`, `safety_compiler.py`, `action_registry.py`).
- Robot/device integration: `src/claudia/robot_controller/`.
- Entry points: `production_commander.py` (interactive), `start_production_brain.sh` (launcher).
- Tests: `test/` with `unit/`, `integration/`, and `hardware/` subfolders.
- Supporting assets: `models/`, `scripts/`, `docs/`, and runtime logs under `logs/`.

## Build, Test, and Development Commands
- Install editable package: `pip install -e .`
- Run commander (simulation): `python3 production_commander.py`
- Run commander (hardware): `python3 production_commander.py --hardware`
- Recommended launcher: `./start_production_brain.sh`
- Run all test suites via wrapper: `python3 test/run_tests.py`
- Run targeted tests:
  - `python3 test/run_tests.py --type unit`
  - `python3 -m pytest -q test/unit/test_safety_regression.py`
- Format/lint/type-check:
  - `black src test`
  - `flake8 src test`
  - `mypy src`

## Coding Style & Naming Conventions
- Python 3.8+; 4-space indentation.
- Follow Black formatting (`line-length = 88` in `pyproject.toml`).
- Use type hints for new/changed code; prefer explicit return types for async methods.
- Naming: `snake_case` for variables/functions, `PascalCase` for classes, `UPPER_SNAKE_CASE` for constants.
- Keep safety logic centralized (route all executable actions through `SafetyCompiler`).

## Testing Guidelines
- Framework: `pytest` (+ `pytest-asyncio`).
- Test files: `test_*.py`; test classes: `Test*`; test functions: `test_*`.
- Add/adjust unit tests for each behavior change, then run related integration tests.
- For hardware-facing changes, include at least one hardware smoke result or explain why not run.

## Commit & Pull Request Guidelines
- Follow Conventional Commit style used in history, e.g.:
  - `feat(brain): ...`
  - `fix(safety): ...`
  - `docs: ...`
  - `chore: ...`
- PRs should include:
  - Scope and motivation
  - Key files changed
  - Test evidence (commands + pass/fail summary)
  - Hardware impact/risk notes (if applicable)
  - Rollback plan for runtime-critical changes

## Security & Configuration Tips
- Do not commit credentials or device-specific secrets; use `.env.example` as template.
- Validate `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and network settings before hardware runs.
- Treat emergency-stop and safety-gate changes as high-risk; require regression coverage before merge.
