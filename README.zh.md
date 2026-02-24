[English](README.md) | [日本語](README.ja.md) | [中文](README.zh.md)

<p align="center">
  <img src="docs/images/cover.jpg" alt="Claudia — Unitree Go2 LLM 大脑机器人" width="800">
</p>

# Claudia — LLM 大脑机器人智能

[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20NX-green.svg)](https://developer.nvidia.com/embedded/jetson-orin)
[![Robot](https://img.shields.io/badge/Robot-Unitree%20Go2-orange.svg)](https://www.unitree.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Claudia** 是面向 **Unitree Go2** 四足机器人的 LLM 大脑 AI 系统。它通过本地 LLM 推理（Ollama 上的 Qwen2.5-7B），将日语、中文和英语的自然语言命令转化为机器人动作。完全在 NVIDIA Jetson Orin NX 上本地运行。

> *"LLM 就是机器人的大脑"* —— 语义理解，而非关键词匹配。

---

## 演示

<!-- TODO: 录制完成后替换为实际视频链接 -->

> 视频准备中 —— 录制计划详见 [Demo Video Plan](docs/DEMO_VIDEO_PLAN.md)。

| 演示 | 内容 | 状态 |
|------|------|------|
| **核心演示** | 端到端：日语命令 → LLM 语义理解 → 机器人动作 | 计划中 |
| **语音管线** | USB 麦克风 → ASR → LLM → SafetyCompiler → 执行链路 | 计划中 |
| **多语言支持** | 同一机器人用日语、中文、英语控制 | 计划中 |
| **安全系统** | 电量门控、站立前置条件、紧急停止 | 计划中 |
| **动作集锦** | 全部 18 个表演动作快剪辑 | 计划中 |

---

## 核心特性

### LLM 大脑架构
- **语义理解**：将抽象概念映射到机器人动作（例如："可愛い"（可爱）→ 比心手势，"疲れた"（累了）→ 坐下）
- **直接 API 输出**：LLM 输出带动作码的结构化 JSON —— 无需中间映射层
- **单一真源**：全部 27 个动作定义集中在 `action_registry.py`；白名单、站立需求、方法映射等下游集合自动派生
- **确定性推理**：`temperature=0.0` 的 JSON 模式确保行为一致

### 安全系统 (SafetyCompiler)
- **统一安全管线**：所有动作路径都经过 `SafetyCompiler.compile()` —— 无法绕过
- **三级电量门控**：<=10%：仅安全动作 | <=20%：禁止高能耗 | <=30%：翻转降级为舞蹈
- **站立前置条件**：需要时自动前插 StandUp（例如：Hello 需要站立状态）
- **虚拟姿态追踪**：在动作序列内追踪姿态变化，确保前置条件插入正确
- **白名单强制**：只有已注册、已启用的动作才能执行

### 硬件控制
- **18 个验证动作**：8 个基础姿态 + 7 个表演 + 3 个高级（参见[支持的动作](#支持的动作)）
- **实时控制**：1ms（缓存命中）到约 5 秒（Jetson 上 LLM 推理）的响应时间
- **状态感知序列**：自动解决动作依赖关系
- **优雅降级**：真实硬件 → 模拟仿真，带结构化错误码

### 多语言交互
- **日语优先**：与机器人的自然日语对话
- **中文支持**：完整的中文命令识别
- **英语兼容**：支持基本英语命令
- **ASR 假名正规化**：内置 KANA_ALIASES 管线，清理语音识别输出

---

## 快速开始

### 环境要求

| 组件 | 要求 |
|:---:|---|
| 机器人 | Unitree Go2（推荐 R&D Plus 版本） |
| 计算平台 | NVIDIA Jetson Orin NX |
| 操作系统 | Ubuntu 20.04 (aarch64) |
| Python | 3.8 以上 |
| LLM 运行时 | [Ollama](https://ollama.ai/) |
| 中间件 | ROS2 Foxy + CycloneDDS |
| 网络 | 以太网连接机器人 (`192.168.123.x`) |

### 安装

```bash
git clone https://github.com/ShunmeiCho/Claudia.git
cd claudia
pip install -e .

# 环境配置
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH
```

### 启动

```bash
# 交互式启动器 + 配置面板（推荐）
./start_production_brain.sh
# → 选择运行模式，配置设置（唤醒词、模型、路由等）
# → 输入 'c' 进入配置面板，'t' 以 tmux 后台模式启动

# 键盘模式：
python3 production_commander.py              # 模拟模式
python3 production_commander.py --hardware   # 真实硬件模式

# 语音模式（Phase 2：USB 麦克风 → ASR → LLM → 机器人）
python3 voice_commander.py                   # 语音，模拟模式
python3 voice_commander.py --hardware        # 语音，真实硬件
python3 voice_commander.py --asr-mock        # 语音，模拟 ASR（无需麦克风）
python3 voice_commander.py --daemon          # 后台模式（tmux 用）

# 直接启动（跳过菜单）：
./start_production_brain.sh --voice          # 语音 + 模拟
./start_production_brain.sh --voice-hw       # 语音 + 真实硬件
```

---

## 使用示例

```
Claudia> こんにちは          → 挥手打招呼 (1016)
Claudia> 座って              → 坐下 (1009)
Claudia> 可愛いね            → 比心手势 (1036) — 语义理解："可爱" 触发爱意表达
Claudia> 立ってから挨拶して  → 序列：StandUp(1004) → Hello(1016)
Claudia> 疲れた              → 坐下 (1009) — 语义理解："累了" 触发休息
Claudia> dance               → 随机 Dance1(1022) 或 Dance2(1023)
Claudia> 量子力学について教えて → 对话回复（无动作）
```

---

## 支持的动作

### 基础姿态（8 个动作）

| API 码 | 方法名 | 日语 | 中文 | 英语 | 需站立 |
|:-:|--------|--------|---------|---------|:-:|
| 1001 | Damp | ダンプモード | 阻尼模式 | Damp | - |
| 1002 | BalanceStand | バランス | 平衡站立 | Balance | - |
| 1003 | StopMove | 止まる | 停止 | Stop | - |
| 1004 | StandUp | 立つ | 站立 | Stand Up | - |
| 1005 | StandDown | 伏せる | 趴下 | Stand Down | Yes |
| 1006 | RecoveryStand | 回復 | 恢复站立 | Recovery | - |
| 1009 | Sit | 座る | 坐下 | Sit | Yes |
| 1010 | RiseSit | 起き上がる | 起立 | Rise Sit | - |

### 表演动作（7 个动作）

| API 码 | 方法名 | 日语 | 中文 | 英语 | 需站立 |
|:-:|--------|--------|---------|---------|:-:|
| 1016 | Hello | 挨拶 | 打招呼 | Hello | Yes |
| 1017 | Stretch | 伸び | 伸懒腰 | Stretch | Yes |
| 1022 | Dance1 | ダンス1 | 舞蹈1 | Dance 1 | Yes |
| 1023 | Dance2 | ダンス2 | 舞蹈2 | Dance 2 | Yes |
| 1029 | Scrape | 刮る | 刮擦 | Scrape | Yes |
| 1033 | WiggleHips | 腰振り | 摇臀 | Wiggle Hips | Yes |
| 1036 | Heart | ハート | 比心 | Heart | Yes |

### 高级 / 高风险（3 个动作）

| API 码 | 方法名 | 日语 | 中文 | 英语 | 风险 |
|:-:|--------|--------|---------|---------|------|
| 1030 | FrontFlip | 前転 | 前空翻 | Front Flip | 高 |
| 1031 | FrontJump | ジャンプ | 前跳 | Front Jump | 高 |
| 1032 | FrontPounce | 飛びかかる | 前扑 | Front Pounce | 高 |

> 高风险动作受电量门控约束，且需要站立状态。默认禁用（`allow_high_risk=False`）。

---

## 架构

### 硬件平台

<p align="center">
  <img src="docs/images/go2-hardware.jpg" alt="Unitree Go2 硬件构造" width="800">
</p>

Claudia 运行在 **Unitree Go2** 四足机器人上，外接 **NVIDIA Jetson Orin NX** 作为 AI 计算模块：

| 组件 | 规格 |
|------|------|
| **机器人** | Unitree Go2（12 自由度、8000mAh 电池、4D LiDAR L2） |
| **AI 模块** | NVIDIA Jetson Orin NX 16GB（Developer Kit） |
| **GPU** | 1024 核 NVIDIA Ampere，612MHz max，CUDA 11.4 |
| **CPU** | 8 核 Arm Cortex-A78AE（aarch64） |
| **内存 / 存储** | 16GB LPDDR5 / 512GB NVMe SSD |
| **操作系统** | Ubuntu 20.04，L4T R35.3.1（JetPack 5.1.1），Python 3.8.10 |
| **LLM 运行时** | Ollama + Qwen2.5-7B（Q4_K_M，VRAM ~4.7GB） |
| **ASR** | faster-whisper base（CPU int8，~1.6 秒/句） |
| **麦克风** | Audio-Technica AT2020USB-XP（44.1kHz → 16kHz 重采样） |
| **网络** | 以太网（eth0，192.168.123.x）用于机器人 DDS 通信 |

### 命令处理管线

```
用户输入（日/中/英）
  |
  v
1. 紧急绕过 ................. 硬编码停止命令，~0ms
  |
  v
2. 热缓存 .................. 80+ 条缓存命令→API 映射，~1ms
  |                          （文化表达、假名别名、后缀剥离）
  v
3. 对话检测 ................. 问候/提问 → 纯文本回复
  |
  v
4. LLM 推理（BRAIN_ROUTER_MODE 控制路由）：
  |  - dual（默认）：Action 模型（~30 tokens，Jetson 上 3-5 秒）
  |  - legacy：7B 模型（完整回复 + 动作码）
  |  - shadow：双通道，A/B 对比日志
  v
SafetyCompiler.compile() ..... 白名单→电量门控→站立前置
  |
  v
执行 ......................... SportClient RPC via CycloneDDS/DDS
```

### 模块概览

| 模块 | 职责 |
|--------|------|
| `brain/production_brain.py` | 核心管线：缓存→路由→安全→执行 |
| `brain/channel_router.py` | 双通道 LLM 路由器（legacy/dual/shadow 模式） |
| `brain/action_registry.py` | 所有动作定义的单一真源 |
| `brain/safety_compiler.py` | 统一安全管线（电量、站立、白名单） |
| `brain/audit_logger.py` | 结构化审计日志 (`logs/audit/`) |
| `brain/audit_routes.py` | 审计日志规范路由名 |
| `brain/sdk_state_provider.py` | 直接 SDK 状态查询（ROS2 监控器的替代方案） |
| `brain/mock_sport_client.py` | 测试用 SportClient 模拟器 |
| `robot_controller/system_state_monitor.py` | 基于 ROS2 的电量/姿态监控（5Hz） |
| `robot_controller/unified_led_controller.py` | LED 模式 API（思考中/成功/错误/监听） |
| `production_commander.py` | 键盘 REPL 入口 |
| `voice_commander.py` | 语音模式入口：ASR 子进程 + AudioCapture + ASRBridge |
| `audio/audio_capture.py` | USB 麦克风采集：arecord 子进程 → 重采样 → UDS |
| `audio/asr_bridge.py` | ASR 结果消费者：去重/过滤 → Queue → Brain |
| `audio/wake_word.py` | 唤醒词匹配器 + 门控：精确前缀匹配 + 监听窗口 |
| `audio/asr_service/` | ASR 服务器：faster-whisper + silero-vad + UDS |

---

## 语音识别 (ASR)

> 状态：**Phase 2 已上线** —— 完整语音管线已在 Jetson 上通过 USB 麦克风运行。

### 语音管线

```
USB 麦克风 (AT2020USB-XP, 自动检测声卡, 44100Hz)
  │ arecord 子进程 → 重采样 → 16kHz 960byte 帧
  v
AudioCapture ──→ /tmp/claudia_audio.sock ──→ ASR Server（子进程）
                                                ├── silero-vad + 紧急检测
                                                ├── faster-whisper base (ja, beam=1, CPU int8)
                                                v
ASRBridge ←── /tmp/claudia_asr_result.sock ←─── JSON Lines
  ├── emergency → 队列清空 + 冷却 → brain 调用（绕过锁）
  ├── transcript → 置信度 ≥0.35 过滤 → 去重 → Queue(3)
  └── command worker → brain.process_and_execute(text)
```

### 进程守护

- **SIGHUP 处理**：两个命令器均忽略 SIGHUP —— SSH 断连不会杀死进程
- **ASR 自动重启**：ASR 子进程崩溃时，VoiceCommander 自动重启完整管线（Bridge → Capture → ASR → 重建）。最多重试 3 次，超限后进入降级模式（仅键盘）
- **tmux 集成**：`start_production_brain.sh` 选项 `t` 在 tmux 会话中启动，完整转发环境变量，抗 SSH 断连
- **Ollama GPU 清理**：退出时通过 `keep_alive=0` 立即释放 GPU 显存，避免模型空占 30 分钟

### ASR 环境变量

| 变量 | 默认值 | 选项 |
|------|--------|------|
| `CLAUDIA_ASR_MODEL` | `base` | `base` / `small` / `medium` |
| `CLAUDIA_ASR_BEAM_SIZE` | `1`（贪心解码） | `1` / `3`+（束搜索） |
| `CLAUDIA_ASR_DEVICE` | `cpu` | `cpu` / `cuda` |
| `CLAUDIA_WAKE_WORD_ENABLED` | `0`（关闭） | `0` / `1` |
| `CLAUDIA_WAKE_WORD_TIMEOUT` | `5`（秒） | 独立唤醒词后的监听窗口 |

### 假名正规化

- **KANA_ALIASES 管线**：已集成到热缓存层。将常见的 ASR 假名输出正规化为汉字（例如：`おすわり` → `お座り`、`おて` → `お手`、`はーと` → `ハート`）。消除了日语语音命令中 ASR 不匹配的首要原因。
- **日语后缀剥离**：敬语后缀（です/ます/ください）在热缓存匹配时自动剥离（例如：`かわいいです` → `かわいい`）。
- **紧急命令假名变体**：`EMERGENCY_COMMANDS` 字典包含纯假名变体（`とまれ`、`とめて`、`ていし`），确保即使 ASR 转写不完美也能可靠执行紧急停止。

---

## 文本转语音 (TTS)

> 状态：**回声门控已实现** —— ASR 服务器中已实现 TTS 回声门控（通过 ctrl socket 的 tts_start/tts_end）；TTS Provider 集成待完成。

### 当前状态

回复目前在 REPL 中以文本形式显示。机器人用日语回复（由 `_sanitize_response()` 验证平假名/片假名/汉字的存在来确保）。ASR 服务器实现了回声门控，在 TTS 播放期间静音识别。

### 规划架构 (PR3)

```
ProductionBrain
  |
  v
BrainOutput.response（日语文本）
  |
  v
Commander._speak_nonblocking()
  |  - TTS Provider（可插拔）
  |    - VOICEVOX（日语，高品质）
  |    - Google TTS
  |    - gTTS（轻量级）
  |  - ThreadPoolExecutor（非阻塞）
  |  - generation_id 取消机制
  v
音频输出（扬声器）
```

**核心设计原则**：大脑**绝不**触碰 TTS。`ProductionBrain` 仅产出 `BrainOutput`（文本 + 动作码），TTS 播放完全由命令器层管理。这确保了：
- 大脑的测试无需 TTS mock
- TTS 故障不会阻塞动作执行
- 新命令自动取消正在播放的语音

---

## 技术栈

| 组件 | 技术 |
|:---:|---|
| LLM | Qwen2.5-7B via [Ollama](https://ollama.ai/) |
| 机器人 | Unitree Go2 + unitree_sdk2_python |
| 通信 | CycloneDDS + ROS2 Foxy |
| 平台 | NVIDIA Jetson Orin NX (aarch64) |
| 语言 | Python 3.8.10 |
| 操作系统 | Ubuntu 20.04 |
| GPU | CUDA 11.4 |

---

## 开发

### 安装（开发模式）

```bash
pip install -e ".[dev]"    # 包含 pytest, black, flake8, mypy
```

### 测试

```bash
python3 test/run_tests.py                    # 全部测试
python3 test/run_tests.py --type unit        # 仅单元测试
python3 test/run_tests.py --type hardware    # 仅硬件测试
pytest test/ -v                               # 通过 pytest
```

### 代码检查/格式化

```bash
black --line-length 88 src/
flake8 src/
mypy src/
```

---

## 故障排除

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 错误 3103 | Unitree APP 占用 sport mode | 关闭 APP 并重启机器人 |
| DDS 连接失败 | 网络配置错误 | 确认 `eth0` 的 IP 为 `192.168.123.x`，检查 `RMW_IMPLEMENTATION` |
| LLM 超时 | 模型未加载 | 运行 `ollama list`，检查 `curl localhost:11434/api/tags` |
| 导入错误 | 缺少 PYTHONPATH | `export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH` |
| 错误 3104 | RPC 超时（异步动作） | 机器人可能仍在执行中，请检查连通性 |
| Action 通道 10 秒超时 | Jetson GPU 冷启动 | 空闲后首条命令正常现象；模型会自动预热 |
| `(聴取中)` 但无识别 | 麦克风静音/增益过低 | 检查麦克风增益，测试：`arecord -D hw:X,0 -d 3 /tmp/t.raw` |
| ASR 慢（>5 秒/句） | CPU 上使用 whisper-small | 使用 base（默认）：`CLAUDIA_ASR_MODEL=base` |

---

## 路线图

| 阶段 | 内容 | 状态 |
|:---:|---|:---:|
| PR1 | SafetyCompiler + action_registry + P0 安全修复 | 完成 |
| PR2 | 双通道 LLM 路由（动作+语音分离） | 完成 |
| PR3 | ASR/TTS 集成 | Phase 2 完成（ASR），TTS 待实现 |
| P2 | 参数化动作（Move, Euler, SpeedLevel） | 未来 |
| P2 | 3B 动作通道 A/B 测试 | 未来 |

---

## 许可证

MIT License — 详见 [LICENSE](LICENSE)。

*最后更新：2026-02-20*
