[English](README.md) | [日本語](README.ja.md) | [中文](README.zh.md)

<p align="center">
  <img src="docs/images/cover.jpg" alt="Claudia — Unitree Go2 with LLM Brain" width="800">
  <br>
  <sub>Image credit: <a href="https://www.unitree.com/go2">Unitree Robotics</a></sub>
</p>

# Claudia — LLM-Brained Robot Intelligence

[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20NX-green.svg)](https://developer.nvidia.com/embedded/jetson-orin)
[![Robot](https://img.shields.io/badge/Robot-Unitree%20Go2-orange.svg)](https://www.unitree.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Claudia** is an LLM-brained AI system for the **Unitree Go2** quadruped robot. It translates natural language commands in Japanese, Chinese, and English into robot actions through local LLM inference (Qwen2.5-7B on Ollama), running entirely on-device on NVIDIA Jetson Orin NX.

> *"The LLM is the robot's brain"* — semantic understanding, not keyword matching.

---

## Demo

<!-- TODO: Replace with actual video links after recording -->

> Videos coming soon — see [Demo Video Plan](docs/DEMO_VIDEO_PLAN.md) for the recording checklist.

| Demo | Description | Status |
|------|-------------|--------|
| **Hero Demo** | End-to-end: Japanese commands → LLM semantic understanding → robot action | Planned |
| **Voice Pipeline** | USB mic → ASR → LLM → SafetyCompiler → execution chain | Planned |
| **Multilingual** | Same robot controlled in Japanese, Chinese, and English | Planned |
| **Safety System** | Battery gating, standing prerequisites, emergency stop | Planned |
| **Action Showcase** | All 18 performance actions in quick-cut montage | Planned |

---

## Key Features

### LLM Brain Architecture
- **Semantic Understanding**: Maps abstract concepts to robot actions (e.g., "可愛い" (cute) → Heart gesture, "疲れた" (tired) → Sit down)
- **Direct API Output**: LLM outputs structured JSON with action codes — no intermediate mapping layer
- **Single Source of Truth**: All 27 action definitions live in `action_registry.py`; downstream sets (whitelist, standing requirements, method map) are auto-derived
- **Deterministic Inference**: `temperature=0.0` with JSON mode for consistent behavior

### Safety System (SafetyCompiler)
- **Unified Safety Pipeline**: All action paths go through `SafetyCompiler.compile()` — no bypass possible
- **3-Tier Battery Gating**: <=10%: safe actions only | <=20%: no high-energy | <=30%: downgrade flips to dance
- **Standing Prerequisites**: Auto-prepends StandUp when needed (e.g., Hello requires standing)
- **Virtual Posture Tracking**: Tracks posture changes within action sequences for correct prerequisite insertion
- **Whitelist Enforcement**: Only registered, enabled actions can execute

### Hardware Control
- **18 Validated Actions**: 8 basic postures + 7 performance + 3 advanced (see [Supported Actions](#supported-actions))
- **Real-Time Control**: 1ms (cached) to ~5s (LLM on Jetson) response time
- **State-Aware Sequencing**: Automatic action dependency resolution
- **Graceful Fallback**: Real hardware → Mock simulation, with structured error codes

### Multilingual Interaction
- **Japanese-Primary**: Natural Japanese conversation with the robot
- **Chinese Support**: Full Chinese command recognition
- **English Compatible**: Core English commands supported
- **ASR Kana Normalization**: Built-in KANA_ALIASES pipeline for speech recognition output cleanup

---

## Quick Start

### Prerequisites

| Component | Requirement |
|-----------|-------------|
| Robot | Unitree Go2 (R&D Plus recommended) |
| Compute | NVIDIA Jetson Orin NX |
| OS | Ubuntu 20.04 (aarch64) |
| Python | 3.8+ |
| LLM Runtime | [Ollama](https://ollama.ai/) |
| Middleware | ROS2 Foxy + CycloneDDS |
| Network | Ethernet to robot (`192.168.123.x`) |

### Installation

```bash
git clone https://github.com/ShunmeiCho/Claudia.git
cd claudia
pip install -e .

# Set up environment
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH
```

### Launch

```bash
# Interactive launcher with config panel (recommended)
./start_production_brain.sh
# → Select mode, configure settings (wake word, model, routing, etc.)
# → Option 'c' opens config panel, 't' launches in tmux background

# Keyboard mode:
python3 production_commander.py              # Simulation mode
python3 production_commander.py --hardware   # Real robot

# Voice mode (Phase 2: USB mic → ASR → LLM → robot)
python3 voice_commander.py                   # Voice, simulation
python3 voice_commander.py --hardware        # Voice, real robot
python3 voice_commander.py --asr-mock        # Voice, mock ASR (no mic)
python3 voice_commander.py --daemon          # Background mode (for tmux)

# Direct launch (skip menu):
./start_production_brain.sh --voice          # Voice + simulation
./start_production_brain.sh --voice-hw       # Voice + real robot
```

---

## Usage Examples

```
Claudia> こんにちは          → Hello gesture (1016)
Claudia> 座って              → Sit down (1009)
Claudia> 可愛いね            → Heart gesture (1036) — semantic: "cute" triggers affection
Claudia> 立ってから挨拶して  → Sequence: StandUp(1004) → Hello(1016)
Claudia> 疲れた              → Sit down (1009) — semantic: "tired" triggers rest
Claudia> dance               → Random Dance1(1022) or Dance2(1023)
Claudia> 量子力学について教えて → Conversational response (no action)
```

---

## Supported Actions

### Basic Posture (8 actions)

| API Code | Method | Japanese | Chinese | English | Standing Required |
|----------|--------|----------|---------|---------|:-:|
| 1001 | Damp | ダンプモード | 阻尼模式 | Damp | - |
| 1002 | BalanceStand | バランス | 平衡站立 | Balance | - |
| 1003 | StopMove | 止まる | 停止 | Stop | - |
| 1004 | StandUp | 立つ | 站立 | Stand Up | - |
| 1005 | StandDown | 伏せる | 趴下 | Stand Down | Yes |
| 1006 | RecoveryStand | 回復 | 恢复站立 | Recovery | - |
| 1009 | Sit | 座る | 坐下 | Sit | Yes |
| 1010 | RiseSit | 起き上がる | 起立 | Rise Sit | - |

### Performance (7 actions)

| API Code | Method | Japanese | Chinese | English | Standing Required |
|----------|--------|----------|---------|---------|:-:|
| 1016 | Hello | 挨拶 | 打招呼 | Hello | Yes |
| 1017 | Stretch | 伸び | 伸懒腰 | Stretch | Yes |
| 1022 | Dance1 | ダンス1 | 舞蹈1 | Dance 1 | Yes |
| 1023 | Dance2 | ダンス2 | 舞蹈2 | Dance 2 | Yes |
| 1029 | Scrape | 刮る | 刮擦 | Scrape | Yes |
| 1033 | WiggleHips | 腰振り | 摇臀 | Wiggle Hips | Yes |
| 1036 | Heart | ハート | 比心 | Heart | Yes |

### Advanced / High-Risk (3 actions)

| API Code | Method | Japanese | Chinese | English | Risk |
|----------|--------|----------|---------|---------|------|
| 1030 | FrontFlip | 前転 | 前空翻 | Front Flip | High |
| 1031 | FrontJump | ジャンプ | 前跳 | Front Jump | High |
| 1032 | FrontPounce | 飛びかかる | 前扑 | Front Pounce | High |

> High-risk actions are battery-gated and require standing state. Disabled by default (`allow_high_risk=False`).

---

## Architecture

### Hardware Platform

<p align="center">
  <img src="docs/images/go2-hardware.jpg" alt="Unitree Go2 Hardware Overview" width="800">
  <br>
  <sub>Image credit: <a href="https://www.unitree.com/go2">Unitree Robotics</a></sub>
</p>

Claudia runs on a **Unitree Go2** quadruped robot with an external **NVIDIA Jetson Orin NX** as the AI compute module:

| Component | Specification |
|-----------|---------------|
| **Robot** | Unitree Go2 (12-DOF, 8000mAh battery, 4D LiDAR L2) |
| **AI Module** | NVIDIA Jetson Orin NX 16GB (Developer Kit) |
| **GPU** | 1024-core NVIDIA Ampere, 612MHz max, CUDA 11.4 |
| **CPU** | 8-core Arm Cortex-A78AE (aarch64) |
| **RAM / Storage** | 16GB LPDDR5 / 512GB NVMe SSD |
| **OS** | Ubuntu 20.04, L4T R35.3.1 (JetPack 5.1.1), Python 3.8.10 |
| **LLM Runtime** | Ollama + Qwen2.5-7B (Q4_K_M, ~4.7GB VRAM) |
| **ASR** | faster-whisper base (CPU int8, ~1.6s/utterance) |
| **Microphone** | Audio-Technica AT2020USB-XP (44.1kHz → 16kHz resample) |
| **Network** | Ethernet (eth0, 192.168.123.x) for robot DDS communication |

### Command Processing Pipeline

```
User Input (JA/ZH/EN)
  |
  v
1. Emergency Bypass ........... hardcoded stop commands, ~0ms
  |
  v
2. Hot Cache .................. 80+ cached command→API mappings, ~1ms
  |                            (cultural terms, kana aliases, suffix stripping)
  v
3. Conversational Detection ... greetings/questions → text-only response
  |
  v
4. LLM Inference (BRAIN_ROUTER_MODE controls routing):
  |  - dual (default): Action model (~30 tokens, 3-5s on Jetson)
  |  - legacy: 7B model (full response + action code)
  |  - shadow: both channels, A/B comparison logging
  v
SafetyCompiler.compile() ...... whitelist → battery gate → standing prereq
  |
  v
Execute ....................... SportClient RPC via CycloneDDS/DDS
```

### Module Overview

| Module | Responsibility |
|--------|---------------|
| `brain/production_brain.py` | Core pipeline: cache → router → safety → execution |
| `brain/channel_router.py` | Dual-channel LLM router (legacy/dual/shadow modes) |
| `brain/action_registry.py` | Single source of truth for all action definitions |
| `brain/safety_compiler.py` | Unified safety pipeline (battery, standing, whitelist) |
| `brain/audit_logger.py` | Structured audit trail (`logs/audit/`) |
| `brain/audit_routes.py` | Canonical route names for audit logging |
| `brain/sdk_state_provider.py` | Direct SDK state queries (alternative to ROS2 monitor) |
| `brain/mock_sport_client.py` | Simulates SportClient for testing |
| `robot_controller/system_state_monitor.py` | ROS2-based battery/posture monitoring at 5Hz |
| `robot_controller/unified_led_controller.py` | LED mode API (thinking/success/error/listening) |
| `production_commander.py` | Keyboard REPL entry point |
| `voice_commander.py` | Voice mode entry point: ASR subprocess + AudioCapture + ASRBridge |
| `audio/audio_capture.py` | USB mic capture: arecord subprocess → resample → UDS |
| `audio/asr_bridge.py` | ASR result consumer: dedup/filter → Queue → Brain |
| `audio/wake_word.py` | Wake word matcher + gate: exact prefix matching + listening window |
| `audio/asr_service/` | ASR server: faster-whisper + silero-vad + UDS |

---

## Speech Recognition (ASR)

> Status: **Phase 2 Operational** — Full voice pipeline running on Jetson with USB microphone.

### Voice Pipeline

```
USB Mic (AT2020USB-XP, auto-detect card, 44100Hz)
  │ arecord subprocess → resample → 16kHz 960byte frames
  v
AudioCapture ──→ /tmp/claudia_audio.sock ──→ ASR Server (subprocess)
                                                ├── silero-vad + emergency detection
                                                ├── faster-whisper base (ja, beam=1, CPU int8)
                                                v
ASRBridge ←── /tmp/claudia_asr_result.sock ←─── JSON Lines
  ├── emergency → queue flush + cooldown → brain call (bypass lock)
  ├── transcript → confidence ≥0.35 filter → dedup → Queue(3)
  └── command worker → brain.process_and_execute(text)
```

### Process Resilience

- **SIGHUP Handling**: Both commanders ignore SIGHUP — SSH disconnection does not kill the process
- **ASR Auto-Restart**: If the ASR subprocess crashes, VoiceCommander automatically restarts the full pipeline (Bridge → Capture → ASR → rebuild). Up to 3 attempts before entering degraded mode (keyboard-only)
- **tmux Integration**: `start_production_brain.sh` option `t` launches in a tmux session with full environment forwarding. Survives SSH disconnection
- **Ollama GPU Cleanup**: On shutdown, models are explicitly unloaded from GPU memory (`keep_alive=0`) instead of occupying VRAM for 30 minutes

### ASR Environment Overrides

| Variable | Default | Options |
|----------|---------|---------|
| `CLAUDIA_ASR_MODEL` | `base` | `base` / `small` / `medium` |
| `CLAUDIA_ASR_BEAM_SIZE` | `1` (greedy) | `1` / `3`+ (beam search) |
| `CLAUDIA_ASR_DEVICE` | `cpu` | `cpu` / `cuda` |
| `CLAUDIA_WAKE_WORD_ENABLED` | `0` (off) | `0` / `1` |
| `CLAUDIA_WAKE_WORD_TIMEOUT` | `5` (seconds) | Listening window after standalone wake word |

### KANA Normalization

- **KANA_ALIASES Pipeline**: Integrated into the hot cache layer. Normalizes common ASR kana outputs to their kanji equivalents (e.g., `おすわり` → `お座り`, `おて` → `お手`, `はーと` → `ハート`). Eliminates the #1 source of ASR mismatches with Japanese voice commands.
- **Japanese Suffix Stripping**: Polite suffixes (です/ます/ください) are automatically stripped for hot cache matching (e.g., `かわいいです` → `かわいい`).
- **Emergency Command Kana Variants**: `EMERGENCY_COMMANDS` dictionary includes kana-only variants (`とまれ`, `とめて`, `ていし`) ensuring emergency stops work even with imperfect ASR transcription.

---

## Text-to-Speech (TTS)

> Status: **Echo Gating Implemented** — TTS echo gating (tts_start/tts_end via ctrl socket) is implemented in asr_server.py; TTS provider integration pending.

### Current State

Responses are currently displayed as text in the REPL. The robot responds in Japanese (enforced by `_sanitize_response()` which validates hiragana/katakana/kanji presence). The ASR server implements echo gating to mute recognition during TTS playback.

### Planned Architecture (PR3)

```
ProductionBrain
  |
  v
BrainOutput.response (Japanese text)
  |
  v
Commander._speak_nonblocking()
  |  - TTSProvider (pluggable)
  |    - VOICEVOX (Japanese, high quality)
  |    - Google TTS
  |    - gTTS (lightweight)
  |  - ThreadPoolExecutor (non-blocking)
  |  - generation_id cancellation
  v
Audio Output (speaker)
```

**Key design principle**: The brain **never** touches TTS. `ProductionBrain` produces `BrainOutput` (text + action code); TTS playback is managed entirely in the commander layer. This ensures:
- Brain tests don't need TTS mocks
- TTS failures never block action execution
- New commands automatically cancel in-progress speech

---

## Tech Stack

| Component | Technology |
|-----------|-----------|
| LLM | Qwen2.5-7B via [Ollama](https://ollama.ai/) |
| Robot | Unitree Go2 + unitree_sdk2_python |
| Communication | CycloneDDS + ROS2 Foxy |
| Platform | NVIDIA Jetson Orin NX (aarch64) |
| Language | Python 3.8.10 |
| OS | Ubuntu 20.04 |
| GPU | CUDA 11.4 |

---

## Development

### Install (development mode)

```bash
pip install -e ".[dev]"    # Includes pytest, black, flake8, mypy
```

### Test

```bash
python3 test/run_tests.py                    # All tests
python3 test/run_tests.py --type unit        # Unit only
python3 test/run_tests.py --type hardware    # Hardware only
pytest test/ -v                               # Via pytest
```

### Lint / Format

```bash
black --line-length 88 src/
flake8 src/
mypy src/
```

### LLM Model Management

```bash
ollama list | grep claudia                   # Check models
ollama ps                                    # Running models
curl http://localhost:11434/api/tags         # Ollama health check
```

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| Error 3103 | Unitree app occupying sport mode | Close the app and restart the robot |
| DDS connection failed | Wrong network config | Verify `eth0` has `192.168.123.x`, check `RMW_IMPLEMENTATION` |
| LLM timeout | Model not loaded | Run `ollama list`, check `curl localhost:11434/api/tags` |
| Import error (unitree_sdk2py) | Missing PYTHONPATH | `export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH` |
| Error 3104 | RPC timeout (async action) | Robot may still be executing; check connectivity |
| Action channel 10s timeout | Jetson GPU cold start | First command after idle; model re-warms automatically |
| `(聴取中)` but no recognition | Mic mute/low gain | Check mic gain, test: `arecord -D hw:X,0 -d 3 /tmp/t.raw` |
| ASR slow (>5s/utterance) | whisper-small on CPU | Use base (default): `CLAUDIA_ASR_MODEL=base` |

---

## Roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| PR1 | SafetyCompiler + action_registry + P0 safety fixes | Done |
| PR2 | Dual-channel LLM routing (action + voice separation) | Done |
| PR3 | ASR/TTS integration | Phase 2 Done (ASR), TTS Pending |
| P2 | Parameterized actions (Move, Euler, SpeedLevel) | Future |
| P2 | 3B action-channel A/B testing | Future |

---

## License

MIT License — see [LICENSE](LICENSE) for details.
