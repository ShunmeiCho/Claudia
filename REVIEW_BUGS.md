# Claudia 项目代码审查报告 v2

**审查日期**: 2026-02-16（第二次审查）
**审查范围**: main 分支 + feat/pr3-asr 分支全量（33 文件变更，+5302 / -329 行）
**新增模块**: ASR 语音服务、Action-primary 双通道架构、v2.0/v2.1/v3.0 Modelfile

---

## 总结

| 严重度 | 数量 | 说明 |
|--------|------|------|
| **CRITICAL** | 5 | 命令注入(遗留)、json 缺失导入、asyncio+threading 锁混用、readexactly 资源泄漏、信号处理器不安全 |
| **HIGH** | 7 | asyncio.Lock 初始化、审计单例竞态(遗留)、_result_writer 竞态、帧数据无校验、TTS 门控计时器泄漏、紧急关键词匹配不一致、硬编码路径 |
| **MEDIUM** | 9 | 紧急命令标点(遗留)、lru_cache 实例方法(遗留)、JSON 结构无校验、环形缓冲区越界、重采样精度、VAD+Ring 死锁风险、P50 计算偏差、setup 脚本静默失败、exec() 使用 |
| **LOW** | 5 | asyncio API 废弃、格式风格混用、文档不同步、魔法数 300、测试 mock 签名不一致 |

### 上次审查修复状态

| 编号 | 问题 | 状态 |
|------|------|------|
| C1 | shell=True 命令注入 | **❌ 未修复** |
| C2 | ROS2 Node 类未守护 | **✅ 已修复** — `system_state_monitor.py:49` 添加 `Node = object` 占位 |
| C3 | asyncio.Lock 在同步 __init__ | **❌ 未修复**（brain 中），ASR 模块中同样存在 |
| C4 | 审计日志单例无线程锁 | **❌ 未修复** |
| H5 | 启动脚本硬编码路径 | **⚠️ 部分修复** — Python 入口已改为相对路径，bash 脚本仍硬编码 |

---

## CRITICAL — 必须立即修复

### C1. [遗留] 命令注入漏洞 (`shell=True`)

**文件**: `src/claudia/brain/production_brain.py:725-744`

```python
check_cmd = f"ollama list | grep {model.split(':')[0]}"
subprocess.run(check_cmd, shell=True, ...)

cmd = f'echo "{command}" | timeout {timeout} ollama run {model}'
subprocess.run(cmd, shell=True, ...)
```

**问题**: 用户输入 `command` 直接拼接到 shell 命令中。攻击者可通过输入 `"; rm -rf / #` 执行任意系统命令。`_call_ollama` 虽有 `@lru_cache` 且新代码主要走 `_call_ollama_v2`，但该方法仍可被调用。

**修复**: 改用 `subprocess.run([...], shell=False)` 或统一删除此方法，全部走 `_call_ollama_v2`。

---

### C2. [新] `production_commander.py` 缺少 `import json`

**文件**: `production_commander.py:125`

```python
# 第 1-13 行: 无 import json
# 第 125 行:
payload = json.dumps({...}).encode("utf-8")  # NameError: name 'json' is not defined
```

**问题**: HTTP 兜底预热路径 `_sync_warmup_http()` 使用了 `json.dumps()`，但文件头未导入 `json` 模块。当 Python `ollama` 包不可用时（启动脚本已明确处理此场景），调用此函数会立即抛出 `NameError`，导致模型预热完全失败。

**影响**: 生产环境中 Jetson 若缺少 `ollama` Python 包，**每次启动都会预热失败**，首条命令延迟 10-30 秒。

**修复**: 在文件头添加 `import json`。

---

### C3. [新] asyncio.Lock 与 threading.Lock 混用导致事件循环阻塞

**文件**: `src/claudia/audio/asr_service/asr_server.py:295,452` + `ring_buffer.py:39`

```python
# ring_buffer.py:39 — 阻塞锁
self._lock = threading.Lock()

# asr_server.py:295 — 异步锁
self._result_lock = asyncio.Lock()

# asr_server.py:452 — 在 async 上下文直接调用阻塞操作
self._ring.write(data)  # 内部获取 threading.Lock，阻塞事件循环
```

**问题**: `RingBuffer.write()` 内部获取 `threading.Lock()`（阻塞锁），但被直接从 async handler（`_handle_audio_connection`）调用，**未使用 `run_in_executor`**。这会阻塞整个 asyncio 事件循环，冻结心跳、结果发送、控制消息处理等所有并发操作。

**影响**: 音频处理期间整个 ASR 服务无响应，心跳超时，客户端误判服务崩溃。

**修复**: 将 `self._ring.write(data)` 和 VAD `process_frame()` 调用包裹在 `await loop.run_in_executor(None, ...)` 中。

---

### C4. [新] `readexactly()` 异常路径资源泄漏

**文件**: `src/claudia/audio/asr_service/asr_server.py:447-469`

```python
data = await reader.readexactly(FRAME_BYTES)  # 447
if not data:   # 448 — 死代码，readexactly 永不返回空
    break

except asyncio.IncompleteReadError:  # 464
    logger.info("🎙️ Audio 流结束 (不完整帧)")
    # ❌ 未关闭 writer — 对比 finally 块（468-469）只打印日志
except (asyncio.CancelledError, ConnectionError):  # 466
    pass
finally:  # 468
    logger.info("🎙️ Audio socket 客户端断开")
    # ❌ writer 从未关闭！
```

**问题**:
1. `readexactly()` 要么返回恰好 N 字节，要么抛 `IncompleteReadError`，**永不返回空**。第 448 行是死代码。
2. `finally` 块仅打印日志，**未关闭 writer**。连接断开后 socket 资源泄漏，长时间运行后文件描述符耗尽。

**修复**: 在 `finally` 中添加 `writer.close(); await writer.wait_closed()`。

---

### C5. [新] 信号处理器中不安全的 `asyncio.ensure_future()`

**文件**: `src/claudia/audio/asr_service/asr_server.py:683`

```python
for sig in (signal.SIGTERM, signal.SIGINT):
    loop.add_signal_handler(sig, lambda: asyncio.ensure_future(server.shutdown()))
```

**问题**: 信号处理器中调用 `asyncio.ensure_future()` 创建协程，但信号处理器的执行上下文特殊——可能中断正在执行的协程。此外 lambda 在循环中捕获 `server` 引用，若有变量重绑定风险。若 `shutdown()` 协程抛出异常，异常被静默丢弃（fire-and-forget）。

**修复**: 改用 `loop.call_soon_threadsafe(lambda: asyncio.ensure_future(server.shutdown()))` 或直接设置 `self._running = False` 让主循环优雅退出。

---

## HIGH — 应在发布前修复

### H1. [遗留] asyncio.Lock() 在同步 `__init__` 中创建

**文件**: `production_brain.py:286`, `asr_server.py:295`

两处 `asyncio.Lock()` 都在同步 `__init__` 中创建。Python 3.10+ 严格要求在运行的事件循环中创建，否则 `RuntimeError`。

---

### H2. [遗留] 审计日志单例无线程安全保护

**文件**: `audit_logger.py:182-191`

多线程并发调用 `get_audit_logger()` 可创建多个实例，后者覆盖前者。

---

### H3. [新] `_result_writer` 赋值无锁保护

**文件**: `asr_server.py:411`

```python
self._result_writer = writer  # 无锁
await self._emit_result({...})  # _emit_result 内部获取 _result_lock
```

赋值与 `_emit_result` 之间存在竞态窗口。其他协程可能在赋值后、首次 emit 前读取到部分初始化的 writer。

---

### H4. [新] VAD 帧数据无大小校验

**文件**: `vad_processor.py:233, 436`

```python
frame_ms = len(frame) // BYTES_PER_MS  # 不校验 frame 大小
n_samples = len(frame) // 2  # 假设偶数长度
samples = struct.unpack(f"<{n_samples}h", frame[:n_samples * 2])
```

奇数长度 frame 或超大 frame 会导致计算错误或内存耗尽。

---

### H5. [新] TTS 门控计时器泄漏

**文件**: `asr_server.py:517-526`

快速连续收到多个 `tts_start` 消息时，虽然旧计时器被 cancel，但 `cancel()` 失败（如回调已入队）不会报错，新旧计时器可能同时触发。

---

### H6. [新] 紧急关键词匹配大小写不一致

**文件**: `vad_processor.py:370-377`

输入文本被 `.lower()` 正规化，但 `_emergency_keywords` 列表中的关键词未正规化。日语关键词不受影响，但英文关键词（如大写 "STOP"）匹配逻辑依赖于列表中恰好是小写形式。

---

### H7. [部分修复] 启动脚本硬编码路径

**文件**: `start_production_brain.sh:10,21,28` + `scripts/setup_asr_venv.sh:18,131`

Python 入口 `production_commander.py:16` 已改为相对路径（✅），但 bash 脚本仍硬编码 `/home/m1ng/claudia`。

---

## MEDIUM — 影响健壮性

### M1. [遗留] 紧急命令不处理标点变体

**文件**: `production_brain.py:649` — "止まって！" 漏过紧急通道。

### M2. [遗留] `@lru_cache` 用在实例方法上

**文件**: `production_brain.py:720` — 实例被强引用，无法 GC。

### M3. [新] JSON 控制消息无结构校验

**文件**: `asr_server.py:486-493` — `json.loads()` 结果可能不是 dict，`msg.get()` 会 AttributeError。

### M4. [新] RingBuffer `_read_tail` 无越界校验

**文件**: `ring_buffer.py:147` — `nbytes > capacity` 时返回错误数据。

### M5. [新] 重采样索引精度丢失

**文件**: `asr_server.py:77-78` — 整数除法精度丢失 + 未校验 `src_rate > 0`。

### M6. [新] VAD + RingBuffer 在 async 上下文中死锁风险

**文件**: `vad_processor.py:244,353` — `read_last()` 获取 threading.Lock，与 C3 同源。

### M7. [遗留] P50 中位数计算偏差

**文件**: `audit_logger.py:174` — 偶数列表中位数应取两值平均。

### M8. [新] `setup_asr_venv.sh` pip 安装静默失败

**文件**: `scripts/setup_asr_venv.sh:53,57,61,67` — `--quiet` 隐藏错误输出。

### M9. [新] `offline_route_comparison.py` 使用 `exec()` 加载脚本

**文件**: `scripts/offline_route_comparison.py:54-58` — 应改用 `importlib.util`。

---

## LOW — 可择期改善

### L1. `asyncio.ensure_future()` 废弃

**文件**: `asr_server.py:351,542,683` — 应改用 `asyncio.create_task()`。

### L2. 日志格式混用

**文件**: `production_brain.py` 全文 — `.format()` 和 f-string 混用。

### L3. CLAUDE.md 文档与代码不同步

CLAUDE.md 的 standing 列表仍缺 `1009 (Sit)` 和 `1033 (WiggleHips)`。

### L4. VAD 能量检测魔法数

**文件**: `vad_processor.py:439` — 硬编码阈值 `300` 应提取为常量。

### L5. 测试 mock 签名不一致

**文件**: `test/unit/test_channel_router.py:745-808` — 部分用显式参数，部分用 `**kwargs`。

---

## 新增 ASR 模块整体评估

### 架构设计 ✅ 合理

- 3 路 UDS (Audio / Control / Result) 分离清晰
- VAD → ASR → 紧急关键词检测管道逻辑通顺
- TTS 回声门控设计防止自激
- IPC 协议有版本号和 handshake 机制
- Mock 模式支持完善

### 主要风险

1. **async + threading 混用**是最大架构风险——RingBuffer 用 `threading.Lock`，但被 asyncio 事件循环直接调用。这不是某个方法的 bug，而是整个音频管道的设计问题。应该要么全部 async 化，要么将音频处理整体放入独立线程（通过 `run_in_executor`）。

2. **Socket 资源管理不完整**——`_handle_audio_connection` 和 `_handle_ctrl_connection` 的 `finally` 块都未关闭 writer，长时间运行会泄漏文件描述符。

3. **紧急关键词有两个数据源**——`emergency_keywords.py`（定义）和 `vad_processor.py:207`（硬编码兜底列表），可能不同步。

---

## Action-primary 架构评估

### 路由逻辑 ✅ 基本正确

- SafetyCompiler **不会被绕过**（Invariant 1 维持）：所有路径最终都经过 `safety_compiler.compile()`
- `a + s` 同时出现时 sequence 优先（正确）
- 序列校验先过滤无效码再截断（逻辑正确）

### 注意事项

1. **Dual fallback 审计路由标记**（`channel_router.py:211-214`）：Action channel 失败时 fallback 返回 `ROUTE_ACTION_FALLBACK`，审计链可能混淆。
2. **Shadow 超时语义不一致**：内层 30s Ollama 超时 vs 外层 45s asyncio 超时，`_action_status` 不区分超时来源。

---

## 修复优先级建议

| 优先级 | 编号 | 工作量 | 说明 |
|--------|------|--------|------|
| **立即** | C2 | 极小 | 加一行 `import json` |
| **立即** | C3 | 中 | 将 ring.write + VAD 包裹在 run_in_executor |
| **立即** | C4 | 小 | finally 中关闭 writer |
| **立即** | C1 | 小 | 删除 `_call_ollama` 或移除 shell=True |
| **立即** | C5 | 小 | 修改信号处理器模式 |
| **发布前** | H1-H7 | 中 | 逐个修复 |
| **迭代中** | M1-M9 | 中 | 结合功能迭代处理 |
| **择期** | L1-L5 | 小 | 代码质量改善 |
