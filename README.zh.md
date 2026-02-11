[English](README.md) | [日本語](README.ja.md) | [中文](README.zh.md)

# Claudia — LLM 大脑机器人智能

[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20NX-green.svg)](https://developer.nvidia.com/embedded/jetson-orin)
[![Robot](https://img.shields.io/badge/Robot-Unitree%20Go2-orange.svg)](https://www.unitree.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Claudia** 是面向 **Unitree Go2** 四足机器人的 LLM 大脑 AI 系统。它通过本地 LLM 推理（Ollama 上的 Qwen2.5-7B），将日语、中文和英语的自然语言命令转化为机器人动作。完全在 NVIDIA Jetson Orin NX 上本地运行。

> *"LLM 就是机器人的大脑"* —— 语义理解，而非关键词匹配。

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
- **15 个验证动作**：8 个基础姿态 + 7 个表演 + 3 个高级（参见[支持的动作](#支持的动作)）
- **实时控制**：0ms（缓存命中）到约 3 秒（LLM 推理）的响应时间
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
# 交互式启动器（推荐）
./start_production_brain.sh

# 或直接启动：
python3 production_commander.py              # 模拟模式
python3 production_commander.py --hardware   # 真实硬件模式
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
| 1005 | StandDown | 伏せる | 趴下 | Stand Down | - |
| 1006 | RecoveryStand | 回復 | 恢复站立 | Recovery | - |
| 1009 | Sit | 座る | 坐下 | Sit | Yes |
| 1010 | RiseSit | 起き上がる | 起立 | Rise Sit | - |

### 表演动作（7 个动作）

| API 码 | 方法名 | 日语 | 中文 | 英语 | 需站立 |
|:-:|--------|--------|---------|---------|:-:|
| 1016 | Hello | 挨拶 | 打招呼 | Hello | Yes |
| 1017 | Stretch | 伸び | 伸懒腰 | Stretch | Yes |
| 1021 | Wallow | 転がる | 翻滚 | Wallow | - |
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

### 命令处理管线

```
用户输入（日/中/英）
  |
  v
1. 紧急绕过 ................. 硬编码停止命令，~0ms
  |
  v
2. 热缓存 .................. ~55 条缓存命令→API 映射，~1ms
  |                          （文化表达、假名别名、核心动作）
  v
3. 对话检测 ................. 问候/提问 → 纯文本回复
  |
  v
4. LLM 推理 ................. Qwen2.5-7B via Ollama，JSON 输出，~2-3 秒
  |
  v
5. SafetyCompiler.compile() .. 白名单→电量门控→站立前置
  |
  v
6. 执行 ..................... SportClient RPC via CycloneDDS/DDS
```

### 模块概览

| 模块 | 职责 |
|--------|------|
| `brain/production_brain.py` | 核心管线：缓存→LLM→安全→执行 |
| `brain/action_registry.py` | 所有动作定义的单一真源 |
| `brain/safety_compiler.py` | 统一安全管线（电量、站立、白名单） |
| `brain/audit_logger.py` | 结构化审计日志 (`logs/audit/`) |
| `brain/audit_routes.py` | 审计日志规范路由名 |
| `brain/sdk_state_provider.py` | 直接 SDK 状态查询（ROS2 监控器的替代方案） |
| `brain/mock_sport_client.py` | 测试用 SportClient 模拟器 |
| `robot_controller/system_state_monitor.py` | 基于 ROS2 的电量/姿态监控（5Hz） |
| `robot_controller/unified_led_controller.py` | LED 模式 API（思考中/成功/错误/监听） |
| `production_commander.py` | 交互式 REPL 入口 |

---

## 语音识别 (ASR)

> 状态：**基础就绪** —— 假名正规化管线已集成，Provider 协议已设计。

### 当前状态

输入目前通过交互式 REPL（`production_commander.py`）进行文本输入。不过，语音识别的基础已经就位：

- **KANA_ALIASES 管线**：已集成到热缓存层。将常见的 ASR 假名输出正规化为汉字（例如：`おすわり` → `お座り`、`おて` → `お手`、`はーと` → `ハート`）。消除了日语语音命令中 ASR 不匹配的首要原因。
- **紧急命令假名变体**：`EMERGENCY_COMMANDS` 字典包含纯假名变体（`とまれ`、`とめて`、`ていし`），确保即使 ASR 转写不完美也能可靠执行紧急停止。

### 规划架构 (PR3)

```
麦克风
  |
  v
唤醒词检测 (pvporcupine)
  |
  v
ASR Provider（可插拔）
  |  - Google Speech-to-Text
  |  - OpenAI Whisper（本地，Jetson 优化）
  |  - VOSK（完全离线）
  v
假名正规化（已集成）
  |
  v
ProductionBrain.process_command()
```

`ASRProvider` 抽象基类定义了标准接口，可以在不修改大脑层或命令器层的情况下更换 ASR 引擎。ASR 运行在**命令器层**，将转写文本输入大脑。

---

## 文本转语音 (TTS)

> 状态：**已设计** —— 架构已定义，计划在 PR3 实现。

### 当前状态

回复目前在 REPL 中以文本形式显示。机器人用日语回复（由 `_sanitize_response()` 验证平假名/片假名/汉字的存在来确保）。

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

---

## 路线图

| 阶段 | 内容 | 状态 |
|:---:|---|:---:|
| PR1 | SafetyCompiler + action_registry + P0 安全修复 | 完成 |
| PR2 | 双通道 LLM（动作+语音分离） | 计划中 |
| PR3 | ASR/TTS 集成（Provider 协议） | 已设计 |
| P2 | 参数化动作（Move, Euler, SpeedLevel） | 未来 |
| P2 | 3B 动作通道 A/B 测试 | 未来 |

---

## 许可证

MIT License — 详见 [LICENSE](LICENSE)。

## 贡献者

- **ShunmeiCho** — 项目愿景与核心洞察
- **Claude AI** — 技术实现与架构设计

---

*最后更新：2026-02-11*
