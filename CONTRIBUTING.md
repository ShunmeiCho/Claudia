# Contributing to Claudia

Thank you for your interest in contributing to Claudia! This guide will help you
get started.

Claudia is an LLM-brained AI system for the Unitree Go2 quadruped robot. Since
this project involves **physical hardware control**, contributions require extra
care around safety and testing.

> **No robot? No problem.** All core development can be done in simulation mode
> without access to a physical Go2 robot.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Environment](#development-environment)
- [Code Style](#code-style)
- [Testing](#testing)
- [Safety Guidelines](#safety-guidelines)
- [Pull Request Process](#pull-request-process)
- [Issue Guidelines](#issue-guidelines)
- [Community](#community)

## Code of Conduct

This project follows the [Contributor Covenant v2.1](CODE_OF_CONDUCT.md). By
participating, you are expected to uphold this code. Please report unacceptable
behavior to **dev@claudia-robot.com**.

## Getting Started

### Prerequisites

| Component | Requirement | Notes |
|-----------|-------------|-------|
| Python | 3.8+ | Target: 3.8.10 (Jetson default) |
| OS | Ubuntu 20.04 / macOS / Linux | Production: aarch64 on Jetson |
| Ollama | Latest | For local LLM inference |
| Git | 2.25+ | For version control |

**Optional (hardware development only):**

| Component | Requirement |
|-----------|-------------|
| Robot | Unitree Go2 (R&D Plus recommended) |
| Compute | NVIDIA Jetson Orin NX |
| ROS2 | Foxy + CycloneDDS |
| Network | Ethernet to robot (`192.168.123.x`) |
| Microphone | USB mic (e.g., AT2020USB-XP) |

### Installation

```bash
# Clone the repository
git clone https://github.com/nvidiasamp/Claudia.git
cd Claudia

# Install in development mode
pip install -e ".[dev]"

# Set up Ollama (for LLM inference)
# See https://ollama.ai/ for installation
ollama pull qwen2.5:7b
```

### Verify Your Setup

```bash
# Run unit tests (no hardware needed)
python3 test/run_tests.py --type unit

# Start in simulation mode
python3 production_commander.py
```

## Development Environment

### Python 3.8 Compatibility

Claudia targets **Python 3.8** (Jetson Orin NX default). Please ensure your
code is compatible:

```python
# Do NOT use (Python 3.8 incompatible):
match command:        # Structural pattern matching (3.10+)
    case "stop": ...

x := value            # Walrus operator in some contexts

await asyncio.to_thread(fn)  # 3.9+

# Use instead:
loop = asyncio.get_event_loop()
await loop.run_in_executor(None, fn)
```

### Environment Variables

For simulation development, no special environment variables are needed. For
hardware development:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH
```

See `CLAUDE.md` for the full list of environment variables.

### Project Structure

```
Claudia/
  src/claudia/
    brain/              # Core pipeline: cache -> router -> safety -> execution
    robot_controller/   # Hardware abstraction (LED, state monitor)
    audio/              # Voice pipeline (capture, ASR, wake word)
    ai_components/      # LLM service layer
  test/
    unit/               # Unit tests (no hardware needed)
    integration/        # Integration tests
    hardware/           # Hardware-specific tests (requires Go2)
  scripts/              # Setup, testing, and maintenance scripts
  docs/                 # Documentation and guides
  models/               # LLM model definitions (Ollama Modelfiles)
```

## Code Style

### Formatting

- **Formatter**: [Black](https://github.com/psf/black), line-length 88
- **Linter**: flake8
- **Type checker**: mypy

```bash
# Format
black --line-length 88 src/

# Lint
flake8 src/

# Type check
mypy src/
```

### Conventions

| Item | Convention |
|------|-----------|
| Formatter | Black, line-length 88 |
| Type hints | Required for all functions |
| Docstrings / comments | Written in **Chinese** |
| User-facing text | **Japanese** (robot responses must contain hiragana/katakana/kanji) |
| Logging | stdlib `logging` with emoji prefixes (in brain module) |
| Target Python | 3.8 (no walrus operator, no `asyncio.to_thread`) |

### Naming Conventions

```python
# Modules and files: snake_case
safety_compiler.py
action_registry.py

# Classes: PascalCase
class SafetyCompiler:
class ProductionBrain:

# Functions and variables: snake_case
def process_and_execute(self, command: str) -> BrainOutput:

# Constants: UPPER_SNAKE_CASE
VALID_API_CODES = {1001, 1002, ...}
EMERGENCY_COMMANDS = {"止まれ": 1003, ...}
```

## Testing

### Test Categories

| Type | Location | Hardware Required | Description |
|------|----------|:-:|-------------|
| Unit | `test/unit/` | No | Individual functions, modules |
| Integration | `test/integration/` | No | Cross-module interactions, LLM pipeline |
| Hardware | `test/hardware/` | Yes | Real robot communication, sensors |

### Running Tests

```bash
# All tests
python3 test/run_tests.py

# By category
python3 test/run_tests.py --type unit
python3 test/run_tests.py --type integration
python3 test/run_tests.py --type hardware

# Via pytest
pytest test/ -v
pytest test/unit/ -v
```

### Testing Requirements

- All PRs must pass **unit tests** before merge
- New features must include corresponding tests
- Hardware tests are run by maintainers on physical robot before release
- Simulation mode (`MockSportClient`) is available for testing action execution
  without hardware

### Writing Tests

```python
# test/unit/test_safety_compiler.py

def test_battery_gating_blocks_high_energy():
    """低电量时高能耗动作应被阻止"""
    compiler = SafetyCompiler()
    result = compiler.compile(
        api_code=1030,  # FrontFlip
        battery_level=15
    )
    assert result.blocked is True
    assert "battery" in result.reason
```

## Safety Guidelines

Claudia controls a physical robot. **Safety is non-negotiable.**

### SafetyCompiler Rules

The `SafetyCompiler` is the single safety gate for all action execution. These
rules must never be violated:

1. **All action paths go through `SafetyCompiler.compile()`** — no bypass is
   allowed, regardless of the command source (cache, LLM, or direct API)
2. **Battery gating cannot be disabled** — the 3-tier battery system
   (<=10% / <=20% / <=30%) protects against hardware damage
3. **Standing prerequisites are enforced** — actions requiring standing state
   must have automatic `StandUp` prepended
4. **Whitelist enforcement** — only registered, enabled actions can execute

### What This Means for Contributors

- Never add a code path that executes robot actions without going through
  `SafetyCompiler`
- Never modify battery thresholds without explicit maintainer approval
- New actions must be registered in `action_registry.py` with correct
  `standing_required` and risk flags
- High-risk actions (FrontFlip, FrontJump, FrontPounce) are disabled by
  default and must stay that way

### Emergency Stop

The emergency stop system (`EMERGENCY_COMMANDS`) is hardcoded and bypasses all
other processing. Never modify emergency stop behavior without a dedicated
safety review.

## Pull Request Process

### Branch Naming

```
feat/description      # New features
fix/description       # Bug fixes
refactor/description  # Code refactoring
docs/description      # Documentation
test/description      # Test additions/fixes
```

### Commit Messages

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
feat: add voice wake word detection
fix: correct battery threshold for Dance actions
refactor: extract action validation into SafetyCompiler
docs: update ASR configuration guide
test: add integration tests for dual-channel routing
```

### PR Checklist

Before submitting your PR, ensure:

- [ ] Code follows the project style guide (Black formatted, type hints added)
- [ ] Unit tests pass: `python3 test/run_tests.py --type unit`
- [ ] New features include tests
- [ ] No hardcoded secrets, API keys, or credentials
- [ ] SafetyCompiler is not bypassed or weakened
- [ ] New actions are registered in `action_registry.py`
- [ ] Python 3.8 compatibility verified (no 3.9+ features)
- [ ] Documentation updated if needed

### Review Process

1. Open a PR with a clear description of changes
2. At least one maintainer review is required
3. All CI checks must pass
4. Hardware-affecting changes require testing on physical robot by a maintainer
5. Safety-critical changes require explicit approval from the project lead

### PR Description Template

Your PR description should include:
- What changed and why
- How it was tested (simulation / hardware)
- Any safety implications
- Related issues (e.g., `Closes #42`)

## Issue Guidelines

We use three issue templates:

- **Bug Report** — Something isn't working correctly
- **Feature Request** — Suggest a new feature or enhancement
- **Hardware Issue** — Problems specific to Go2 / Jetson / sensors

When reporting bugs, please include:
- Your running mode (keyboard/voice, simulation/hardware)
- Python version and OS
- Relevant log output from `logs/` directory
- Steps to reproduce

## Community

- **Languages**: The project community is multilingual. Issues and PRs can be
  written in English, Japanese, or Chinese.
- **Questions**: Open a Discussion or Issue for questions about the project
- **Security**: For security vulnerabilities, please see [SECURITY.md](SECURITY.md)
  instead of opening a public issue

Thank you for helping make Claudia better!
