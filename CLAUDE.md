# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Claudia** is an intelligent AI system for the **Unitree Go2** quadruped robot that uses LLM as the robot's brain. The system understands natural language commands in Japanese, Chinese, and English, translating semantic concepts (like "可愛い" → heart gesture) directly into robot API calls without keyword mapping.

### Key Technologies
- **Hardware**: Unitree Go2 R&D Plus on NVIDIA Jetson Orin NX
- **OS**: Ubuntu 20.04.5 LTS (aarch64, Tegra R35.3.1)
- **ROS**: ROS2 Foxy + CycloneDDS
- **LLM**: Qwen2.5-3B/7B via Ollama (local inference)
- **SDK**: unitree_sdk2py + unitree_ros2
- **Python**: 3.8.10
- **CUDA**: 11.4

---

## Claudia Robot Commands

### Quick Start
```bash
# Launch Claudia brain (interactive Japanese/Chinese/English commander)
./start_production_brain.sh
# Options: 1) Simulation mode (safe testing), 2) Real hardware mode

# Alternative: Direct launch
python3 production_commander.py           # Simulation mode
python3 production_commander.py --hardware # Real hardware mode
```

### Production Deployment
For deploying optimized models or GPU acceleration:
```bash
# Deploy optimized LLM models
bash scripts/setup/deploy_claudia_llm.sh

# TensorRT-LLM (GPU acceleration) - experimental
# See docker/tensorrt-llm/ for details
```

### Environment Setup
```bash
# Verify complete environment (OS, Python, CUDA, ROS2, GPU)
bash scripts/setup/verify_environment.sh
bash scripts/setup/verify_environment.sh --quick  # Fast check

# Validate ROS2 Foxy installation
bash scripts/setup/validate_ros2.sh

# Setup CycloneDDS workspace
bash scripts/setup/setup_cyclonedx_workspace.sh

# Setup ROS2 environment integration with Claudia
bash scripts/setup/setup_ros2_environment.sh
source .env.ros2  # Load ROS2 environment variables

# CycloneDDS configuration (critical for Go2 communication)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

### Testing

#### Run All Tests
```bash
python3 test/run_tests.py
python3 test/run_tests.py --type hardware    # Hardware tests only
python3 test/run_tests.py --type unit        # Unit tests only
python3 test/run_tests.py --type integration # Integration tests only
```

#### Specific Hardware Tests
```bash
# Communication performance (Task 3.7)
python3 test/hardware/test_communication_performance.py

# LLM performance evaluation (Task 11)
python3 test/test_claudia_llm_performance.py --model claudia-v3.2:3b
python3 test/test_claudia_llm_performance.py --compare claudia-v3.1:3b claudia-v3.2:3b

# Basic connection tests
python3 test/hardware/test_unitree_connection.py
python3 test/hardware/test_sportclient_connection.py
python3 test/hardware/test_robot_state_reading.py
python3 test/hardware/test_basic_control_commands.py
```

#### Sensor Validation
```bash
# Camera validation
bash scripts/validation/camera/run_front_camera_validation.sh

# IMU validation
bash scripts/validation/imu/run_imu_validation.sh

# Foot force sensors
bash scripts/validation/foot_force/run_static_validation.sh

# Audio system
bash scripts/validation/audio/run_audio_validation.sh
```

#### Using pytest (if available)
```bash
pytest test/ -v
pytest test/hardware/ --hardware -v
```

### Build and Package Management
```bash
# Install Python dependencies
pip install -e .                    # Development install
pip install -e ".[dev]"            # With dev dependencies
pip install -e ".[tensorrt]"       # With TensorRT support
pip install -e ".[realsense]"      # With RealSense support

# Build ROS2 workspace
cd cyclonedx_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash

# Clean build
rm -rf cyclonedx_ws/build/ cyclonedx_ws/install/ cyclonedx_ws/log/
```

### Maintenance
```bash
# Daily cleanup (removes .pyc, __pycache__, temp files, old logs)
bash scripts/maintenance/daily_cleanup.sh

# Test cleanup
find test/ -name ".pytest_cache" -type d -exec rm -rf {} + 2>/dev/null
find test/ -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null
```

---

## Architecture Overview

### LLM Brain System
The core brain (`src/claudia/brain/production_brain.py`) uses a hybrid 3B/7B model approach:

```python
# User: "可愛い動作して" (do something cute)
#   ↓
# 🧠 LLM: "可愛い" → Heart gesture (API 1036)
#   ↓
# 🎯 JSON: {"response":"ハートします","api_code":1036}
#   ↓
# 🤖 Execute: client.Heart() → Robot performs heart
```

**Key Components**:
- **ProductionBrain**: LLM core with smart caching, 3B for common commands, 7B for complex sequences
- **MockSportClient**: Safe simulation for testing
- **State Machine**: Auto-handles dependencies (e.g., auto-stand before hello when sitting)

### Directory Structure
```
src/claudia/
├── brain/                          # LLM brain system
│   ├── production_brain.py         # Main LLM brain (3B/7B hybrid)
│   └── mock_sport_client.py        # Safe simulation client
├── robot_controller/               # Hardware control
│   ├── unified_led_controller.py   # LED system interface
│   ├── led_state_machine.py        # LED state management
│   ├── led_patterns.py             # LED visual patterns
│   └── unitree_messages.py         # Protocol definitions
├── ai_components/                  # AI services
│   ├── llm_service/                # LLM integration
│   └── deployment/                 # Deployment configs
├── common/                         # Shared utilities
│   ├── config.py                   # Configuration management
│   ├── logger.py                   # Logging system
│   ├── ros2_manager.py             # ROS2 integration
│   └── utils.py                    # Common utilities
├── sensors/                        # Sensor interfaces
├── vision/                         # Computer vision
├── navigation/                     # Path planning
└── audio/                          # Audio processing

test/
├── run_tests.py                    # Test runner
├── unit/                           # Unit tests
├── integration/                    # Integration tests
└── hardware/                       # Hardware tests
```

### Supported Robot Actions
**15 Validated Actions**:
- **Basic (8)**: Sit(1009), Stand(1004), Down(1005), Stop(1003), Damp(1001), Balance(1002), Recovery(1006), RiseSit(1010)
- **Performance (4)**: Hello(1016), Stretch(1017), Heart(1036), Scrape(1029)
- **Advanced (3)**: FrontFlip(1030), FrontJump(1031), FrontPounce(1032)

### LED Control System
The robot features a sophisticated LED control system for visual feedback:

```python
from claudia.robot_controller.unified_led_controller import UnifiedLEDController

led = UnifiedLEDController()
led.set_mode('thinking')    # Blue pulsing during LLM processing
led.set_mode('success')     # Green for successful action
led.set_mode('error')       # Red for errors
led.set_mode('listening')   # Cyan when waiting for input
```

**Key Components**:
- `unified_led_controller.py` - Main LED interface
- `led_state_machine.py` - State-based LED transitions
- `led_patterns.py` - Predefined visual patterns
- `led_controller.py` - Low-level LED hardware control

### ROS2 Integration
```python
from claudia.common.ros2_manager import ROS2Manager

ros_mgr = ROS2Manager(auto_setup=True)
node = ros_mgr.create_node('claudia_control')
```

**Key Topics**: `/sportmodestate`, `/lowstate`, `/lowcmd`, `/wirelesscontroller`

---

## Development Guidelines

### Testing Workflow
1. **Always test in simulation first**: Use mode 1 or `--simulation`
2. **Hardware testing**: Clear space, emergency stop ready, monitor battery
3. **Test creation**: Add to `test/unit/`, `test/integration/`, or `test/hardware/`

### Adding New Actions
1. Test with raw SDK: `client.ActionName()` in `unitree_sdk2_python/`
2. Add to `ProductionBrain.hot_cache` if commonly used
3. Update LLM training data for new semantic concepts
4. Add test case in `test/hardware/`

### Code Style
- **Formatting**: Black (line-length=88)
- **Type hints**: Python 3.8+ annotations required
- **Async**: Use `asyncio` for LLM calls and long operations

### Configuration
- `config/default.yaml` - Main config with ROS2 settings
- `.env.ros2` - Auto-generated ROS2 environment (source before running)
- `pyproject.toml` - Python package definition

---

## Debugging Commands

### Check LLM Service
```bash
# Verify Ollama is running
curl http://localhost:11434/api/tags
ollama list | grep claudia
ollama ps                     # Show running models

# Test LLM manually
ollama run claudia-go2-3b:v11.2 "こんにちは"
```

### Check Robot Connection
```bash
# Verify network connectivity
ping 192.168.123.161
ip addr show eth0

# Check ROS2 topics
ros2 topic list
ros2 topic echo /sportmodestate --once
ros2 node list

# Monitor DDS communication
bash scripts/setup/test_dds_communication.sh
```

### Monitor System
```bash
# GPU status (Jetson)
tegrastats

# System resources
htop
nvidia-smi  # If available

# Check logs
tail -f logs/$(date '+%Y%m')/claudia_*.log
```

---

## Common Issues

### API Error 3103 (Sport Mode Occupied)
Close Unitree app and restart robot, then reconnect.

### DDS Connection Failed
```bash
echo $RMW_IMPLEMENTATION  # Should be: rmw_cyclonedds_cpp
ip addr show eth0 | grep 192.168.123  # Verify network
ros2 topic list  # Should see /sportmodestate
```

### LLM Response Timeout
```bash
curl http://localhost:11434/api/tags  # Check Ollama
ollama list | grep claudia-go2        # Verify models
ollama pull claudia-go2-3b:v11.2      # Pull if missing
```

### Import Errors (unitree_sdk2py)
```bash
ls unitree_sdk2_python/  # Verify SDK location
# Path auto-added in scripts, but for manual use:
export PYTHONPATH=/home/m1ng/claudia/unitree_sdk2_python:$PYTHONPATH
```

---

## Key Documentation

- `docs/FINAL_BREAKTHROUGH_SUMMARY.md` - Technical breakthrough
- `docs/SDK_LIMITATION_ANALYSIS.md` - SDK analysis
- `docs/GO2_SUPPORTED_ACTIONS.md` - Complete action list
- `docs/CORRECT_LLM_ARCHITECTURE.md` - LLM brain architecture
- `docs/guides/LED_USAGE_GUIDE.md` - LED system documentation
- `docs/guides/LED_SYSTEM_VERIFICATION_REPORT.md` - LED validation
- `test/README.md` - Testing framework
- `README_PERFORMANCE_OPTIMIZATION.md` - Performance tuning
- `PRODUCTION_READY.md` - Production deployment guide
- `.cursor/rules/project_management.mdc` - Project rules

---

## Network Configuration

**Robot Network** (Unitree Go2):
- Robot IP: `192.168.123.161`
- Interface: `eth0` (required)

```bash
ping 192.168.123.161                    # Test connection
ip addr show eth0 | grep 192.168.123    # Verify interface
```

---

## Safety and Best Practices

### Hardware Safety
1. Clear 2m radius around robot
2. Keep remote controller handy for emergency stop
3. Monitor battery (robot stops at ~10%)
4. Test progression: Simulation → Basic → Complex
5. Always verify robot state before commands
6. **LED Indicators**: Watch LED colors for system status
   - Blue pulsing = LLM thinking
   - Green = Action successful
   - Red = Error occurred
   - Cyan = Listening for input

### Code Safety
1. Use simulation mode for development/testing
2. Validate inputs before hardware commands
3. Timeout all hardware calls (default 5s)
4. Log all actions for review
5. Never assume robot state

### Project Management
From `.cursor/rules/project_management.mdc`:
- Clean after operations
- Standardized timestamps: `date '+%Y%m%d_%H%M%S'`
- Automated backups before overwrites
- Structured logging: `logs/YYYYMM/`
- Check git status before major changes

---

# Task Master AI Integration

## Essential TaskMaster Commands

### Core Workflow
```bash
# Project Setup
task-master init
task-master parse-prd .taskmaster/docs/prd.txt
task-master models --setup

# Daily Development
task-master list                                   # Show all tasks
task-master next                                   # Get next task
task-master show <id>                             # View task details
task-master set-status --id=<id> --status=done    # Mark complete

# Task Management
task-master add-task --prompt="description" --research
task-master expand --id=<id> --research --force
task-master update-subtask --id=<id> --prompt="notes"

# Analysis
task-master analyze-complexity --research
task-master complexity-report
```

### MCP Integration
Configure in `.mcp.json`:
```json
{
  "mcpServers": {
    "task-master-ai": {
      "command": "npx",
      "args": ["-y", "--package=task-master-ai", "task-master-ai"],
      "env": {
        "ANTHROPIC_API_KEY": "your_key_here",
        "PERPLEXITY_API_KEY": "your_key_here"
      }
    }
  }
}
```

**MCP Tools**: `help`, `initialize_project`, `parse_prd`, `get_tasks`, `next_task`, `get_task`, `set_task_status`, `add_task`, `expand_task`, `update_task`, `update_subtask`, `analyze_project_complexity`, `complexity_report`

### TaskMaster Files
- `.taskmaster/tasks/tasks.json` - Main task database (auto-managed)
- `.taskmaster/config.json` - AI model config (use `task-master models`)
- `.taskmaster/tasks/*.txt` - Individual task files (auto-generated)
- `.env` - API keys

### Task Structure
- **IDs**: Main tasks (1, 2, 3), Subtasks (1.1, 1.2), Sub-subtasks (1.1.1)
- **Status**: pending, in-progress, done, deferred, cancelled, blocked
- **Current tasks**: 17+ tasks in `.taskmaster/tasks/tasks.json` covering environment setup, hardware validation, LLM integration

### Best Practices
1. Use `/clear` between tasks for focus
2. `task-master show <id>` to pull specific task context
3. Log implementation: `task-master update-subtask --id=<id> --prompt="notes"`
4. For PRD updates: `task-master parse-prd --append`
5. Reference tasks in commits: `git commit -m "feat: implement X (task 1.2)"`

### Troubleshooting TaskMaster
```bash
cat .env                              # Check API keys
task-master models                    # Verify config
task-master generate                  # Regenerate task files
task-master fix-dependencies          # Fix dependency issues
```

**Note**: Never manually edit `tasks.json` or `.taskmaster/config.json` - use commands instead.

---

**Last Updated**: 2025-10-02
**Status**: Production Ready ✅
**Platform**: NVIDIA Jetson Orin NX + Unitree Go2 R&D Plus
