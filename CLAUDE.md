# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Claudia** is an LLM-brained AI system for the **Unitree Go2** quadruped robot. It understands natural language commands in Japanese, Chinese, and English, translating semantic concepts (e.g., "可愛い" → Heart gesture API 1036) directly into robot API calls via local LLM inference (Qwen2.5-7B on Ollama).

**Platform**: NVIDIA Jetson Orin NX, Ubuntu 20.04 (aarch64), Python 3.8.10, ROS2 Foxy, CycloneDDS, CUDA 11.4

## Commands

### Run
```bash
./start_production_brain.sh              # Interactive launcher (keyboard/voice x sim/hardware)
python3 production_commander.py          # Keyboard REPL, simulation mode
python3 production_commander.py --hardware  # Keyboard REPL, real robot

# Voice mode (Phase 2: USB mic → ASR → LLM → robot)
python3 voice_commander.py              # Voice, simulation mode
python3 voice_commander.py --hardware   # Voice, real robot
python3 voice_commander.py --asr-mock   # Voice, mock ASR (no mic needed)
ASR_MOCK=1 python3 voice_commander.py   # Env-based mock ASR
```

### Test
```bash
python3 test/run_tests.py                # All tests
python3 test/run_tests.py --type unit    # Unit only
python3 test/run_tests.py --type hardware  # Hardware only
python3 test/run_tests.py --type integration

# Single test file
python3 test/hardware/test_unitree_connection.py

# pytest (if installed)
pytest test/ -v
pytest test/hardware/ --hardware -v
```

### Install
```bash
pip install -e .                # Dev install
pip install -e ".[dev]"         # With pytest, black, flake8, mypy
```

### Lint/Format
```bash
black --line-length 88 src/     # Format (configured in pyproject.toml)
flake8 src/
mypy src/
```

### LLM Model Management
```bash
ollama list | grep claudia      # Check models
ollama ps                       # Running models
curl http://localhost:11434/api/tags  # Ollama health check

# Model is auto-created from modelfile if missing
# Override model: BRAIN_MODEL_7B=model-name python3 production_commander.py
```

### ROS2 Workspace Build
```bash
cd cyclonedx_ws && source /opt/ros/foxy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

### Environment Variables
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Required for Go2 DDS
export CYCLONEDDS_URI='...'                     # eth0 network interface config
export PYTHONPATH=~/claudia/unitree_sdk2_python:$PYTHONPATH  # SDK imports
export BRAIN_MODEL_7B=claudia-7b:v2.0  # Override LLM model

# PR2: Dual-channel routing
export BRAIN_ROUTER_MODE=dual     # "dual" (default) | "legacy" | "shadow"
export BRAIN_MODEL_ACTION=claudia-action-v3  # Action channel model name
```

## Architecture

### Command Processing Pipeline (`src/claudia/brain/production_brain.py`)

`ProductionBrain.process_and_execute()` is the atomic entry point (PR2). Commands flow through `process_command()` — a **5-stage pipeline** with early-exit at each stage:

```
User Input → process_and_execute()
  ↓
1. Emergency Bypass    — hardcoded stop/halt commands, ~0ms, bypasses everything
  ↓
2. Hot Cache           — ~60+ cached command→API mappings + KANA_ALIASES auto-expansion
                         + Japanese suffix stripping (です/ます/ください → base form match)
  ↓
3. Conversational Detection — pattern-matching for greetings/questions, returns text-only (no API)
  ↓
4. LLM Inference (BRAIN_ROUTER_MODE controls routing):
   - dual (default): Action channel (action-only model, ~30 tokens) → template response
                     a=null → template conversational response (no 7B needed)
   - legacy: 7B model via Ollama → JSON {"response":"...", "api_code":N}
   - shadow: Legacy wins, Action runs sequentially for A/B comparison logging (single GPU)
  ↓
Post-processing:
  - SafetyCompiler: validates ALL router modes (Invariant 1 — never bypassed)
  - State tracking: updates last_posture_standing based on executed API
  - Audit logging: route, model, timing, safety verdict, shadow comparison (if any)
  ↓
execute_action() → SportClient RPC (or mock)
  ↓
execution_status: "success" | "unknown" | "failed" | "skipped"
```

### Key Design Decisions

- **Atomic entry point**: `process_and_execute()` handles process + execute + status in one call. Direct `process_command()` calls emit deprecation warning via `contextvars.ContextVar`
- **Dual-channel routing** (PR2): `ChannelRouter` in `channel_router.py` is **decision-only** — never executes or calls SafetyCompiler. Three modes via `BRAIN_ROUTER_MODE` env var (default: `dual`)
- **Single 7B model** (legacy): `claudia-7b:v2.0`, configurable via `BRAIN_MODEL_7B` env var
- **Action model** (dual/shadow): `claudia-action-v3`, `num_predict=30`, `temperature=0.0`, outputs only `{"a":N}` or `{"s":[...]}` or `{"a":null}`
- **Ollama JSON mode**: `format='json'` with `temperature=0.0` for deterministic output
- **Graceful fallback chain**: Real SportClient → MockSportClient → simulation mode. Error 3103 (sport mode occupied by Unitree app) auto-falls back to mock.
- **LRU cache on `_call_ollama`**: subprocess-based fallback path caches results
- **Async LLM calls**: `_call_ollama_v2` uses `asyncio.wait_for` + thread executor (Python 3.8 compatible, no `asyncio.to_thread`)
- **Response sanitization**: `_sanitize_response()` rejects non-Japanese LLM output (checks for hiragana/katakana/kanji) + word-boundary regex for nonsense filtering
- **Shadow mode**: Legacy result returned to user; dual result logged for comparison. Sequential execution (single GPU), request_id tracking, high_risk_divergence detection
- **Action channel timeout**: 10s (Jetson GPU inference 3-5s + context switch overhead)

### Robot Actions (API Codes)

15 validated actions communicated as integer codes:
- **Safety**: Damp(1001), Balance(1002), Stop(1003)
- **Posture**: Stand(1004), Down(1005), Recovery(1006), Sit(1009), RiseSit(1010)
- **Performance**: Hello(1016), Stretch(1017), Dance(1022/1023), Scrape(1029), Heart(1036)
- **Advanced**: FrontFlip(1030), FrontJump(1031), FrontPounce(1032) — require standing, battery-gated

Actions requiring standing state: 1016, 1017, 1022, 1023, 1029, 1030, 1031, 1032, 1036

### Module Responsibilities

| Module | Role |
|--------|------|
| `brain/production_brain.py` | Core pipeline: cache → router → safety → execution |
| `brain/channel_router.py` | PR2: Dual-channel LLM router (legacy/dual/shadow modes) |
| `brain/action_registry.py` | Action definitions, VALID_API_CODES, template responses |
| `brain/safety_compiler.py` | SafetyCompiler: action validation, standing deps, battery gates |
| `brain/mock_sport_client.py` | Simulates SportClient with realistic delays and return codes |
| `brain/audit_logger.py` | Structured audit trail to `logs/audit/` with PR2 extensions |
| `brain/audit_routes.py` | Route constants (all _log_audit route= values) |
| `robot_controller/system_state_monitor.py` | ROS2-based battery/posture monitoring at 5Hz |
| `robot_controller/unified_led_controller.py` | LED mode API (thinking/success/error/listening) |
| `robot_controller/led_state_machine.py` | State-based LED transitions |
| `ai_components/llm_service/` | LLM service layer (cache, streaming, prompt optimization, API server) |
| `common/ros2_manager.py` | ROS2 node lifecycle management |
| `production_commander.py` | Interactive REPL entry point, delegates to ProductionBrain |
| `voice_commander.py` | Voice mode entry point: ASR subprocess + AudioCapture + ASRBridge |
| `audio/pcm_utils.py` | PCM resampling utility (44100→16000, numpy index-based) |
| `audio/audio_capture.py` | USB mic capture: arecord subprocess → resample → UDS audio.sock |
| `audio/asr_bridge.py` | ASR result consumer: result.sock → dedup/filter → Queue → Brain |
| `audio/asr_service/` | ASR server: faster-whisper + silero-vad + 3-way UDS (Phase 1) |

### Voice Pipeline (Phase 2)

```
USB Mic (AT2020USB-XP, auto-detect card, 44100Hz)
  │ arecord subprocess → resample → 16kHz 960byte frames
  ▼
AudioCapture ──→ /tmp/claudia_audio.sock ──→ ASR Server (subprocess)
                                                ├── silero-vad + emergency
                                                ├── faster-whisper base (ja, beam=1, CPU int8)
                                                ▼
ASRBridge ←── /tmp/claudia_asr_result.sock ←─── JSON Lines
  ├── emergency → queue flush + cooldown → brain call (bypass lock)
  ├── transcript → confidence ≥0.55 filter → dedup → Queue(3)
  └── command worker → brain.process_and_execute(text)

ASR env overrides:
  CLAUDIA_ASR_MODEL=small          # base(default)/small/medium
  CLAUDIA_ASR_BEAM_SIZE=3          # 1(default,greedy)/3+(beam search)
  CLAUDIA_ASR_DEVICE=cuda          # cpu(default)/cuda
```

### Hardware Communication

SportClient initialization in `_init_sport_client()`:
1. Sets `CYCLONEDDS_HOME`, `LD_LIBRARY_PATH`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`
2. Calls `ChannelFactoryInitialize(0, "eth0")` for DDS
3. Creates `SportClient()`, sets timeout, calls `Init()`
4. Tests with `RecoveryStand()` — return code 0=OK, 3103=app occupied, 3203=unsupported

Robot IP: `192.168.123.161` via `eth0`

## Code Style

- **Formatter**: Black, line-length 88 (`pyproject.toml`)
- **Type hints**: Python 3.8+ annotations required
- **Target Python**: 3.8 (no walrus operator, no `asyncio.to_thread`, use `loop.run_in_executor`)
- **Logging**: Uses stdlib `logging` with emoji prefixes, not loguru in brain module
- **Comments/docstrings**: Written in Chinese
- **User-facing text**: Japanese (robot responses must contain hiragana/katakana/kanji)

## Common Issues

| Problem | Cause | Fix |
|---------|-------|-----|
| Error 3103 | Unitree app occupying sport mode | Close app, restart robot |
| DDS connection failed | Wrong network config | Check `eth0` has `192.168.123.x`, verify `RMW_IMPLEMENTATION` |
| LLM timeout | Model not loaded or Ollama down | `ollama list`, `curl localhost:11434/api/tags` |
| Action channel 10s timeout | Jetson GPU cold start or long input | First command after idle; model re-warms automatically |
| Import errors (unitree_sdk2py) | Missing PYTHONPATH | `export PYTHONPATH=~/claudia/unitree_sdk2_python:$PYTHONPATH` |
| Non-Japanese LLM output | Model hallucination | Handled by `_sanitize_response()`, falls back to default Japanese |
| `(聴取中)` but no recognition | Mic mute/low gain/VAD not triggering | Check mic gain, test: `arecord -D hw:X,0 -d 3 /tmp/t.raw` then check RMS |
| Hot cache miss on `かわいいです` | Suffix not stripped | Fixed: suffix stripping layer (です/ます/ください) now active |
| ASR slow (>5s/utterance) | whisper-small on CPU | Use base (default) or set `CLAUDIA_ASR_MODEL=base` |

## Project Conventions

- **Timestamps**: `date '+%Y%m%d_%H%M%S'` for filenames, ISO format for logs
- **Log directory**: `logs/YYYYMM/`
- **Audit logs**: `logs/audit/`
- **Cleanup**: `bash scripts/maintenance/daily_cleanup.sh`
- **Test structure**: `test/unit/`, `test/integration/`, `test/hardware/` — each with `__init__.py`
- **Config**: `config/default.yaml`, `.env.ros2` (auto-generated)
- **Model files**: `models/ClaudiaIntelligent_7B_v2.0` — Ollama Modelfile for creating the brain model (no extension, per Ollama convention)
