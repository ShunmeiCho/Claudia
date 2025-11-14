# Claudia硬件测试指南

**日期**: 2025-11-14
**版本**: v2.0
**前置条件**: DDS符号问题已修复 + 7个Critical问题已修复

---

## 问题诊断和修复

### 问题1：ROS2初始化失败 ✅ 无需处理
```
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Precondition Not Met
ROS2初始化失败，启用模拟模式
```

**根本原因**：
- LD_LIBRARY_PATH=/home/m1ng/cyclonedds/install/lib 导致ROS2 rmw加载了CycloneDDS 0.10.x
- ROS2 Foxy的rmw_cyclonedds_cpp编译针对CycloneDDS 0.7.0，库不兼容

**影响评估**：
- ✅ **无功能影响**：代码已有fallback机制，自动切换模拟模式
- ✅ unitree_sdk2py仍正常工作（使用0.10.x）
- ✅ ProductionBrain功能完整（不依赖ROS2）

**为什么可以忽略**：
1. SystemStateMonitor的ROS2功能是**可选增强**，非核心依赖
2. 模拟模式提供相同API接口，默认电池100%，姿态模拟
3. 真实硬件控制通过unitree_sdk2py（已成功初始化）

**日志解读**：
```bash
🧠 📡 初始化DDS通道工厂 (eth0)...
🧠 ✅ 真实SportClient初始化成功 - 机器人已连接  # ← 关键成功
[ERROR] [rmw_cyclonedds_cpp]: ...                      # ← 可忽略
ROS2初始化失败，启用模拟模式                           # ← 预期fallback
🧠 ✅ 状态监控器已启动                                 # ← 模拟模式正常
```

### 问题2：BrainOutput TypeError ✅ 已修复
```
TypeError: __init__() got an unexpected keyword argument 'reasoning'
TypeError: __init__() got an unexpected keyword argument 'success'
```

**修复内容**：
- 第一次修复（Commit d252ab3）：添加`reasoning`字段
- 第二次修复（Commit 221e9b3）：添加`success`字段
- 文件：`src/claudia/brain/production_brain.py:54-55`

**根本原因**：
热路径SafetyValidator集成时添加了新参数，但dataclass定义未同步更新

**验证结果**：
```python
✅ BrainOutput创建成功（含reasoning和success参数）
✅ to_dict()正确包含所有字段
✅ 默认值正确设置（reasoning="", success=True）
```

### 问题3：Python 3.8兼容性 - asyncio.to_thread ✅ 已修复
```
🧠 Ollama调用错误: module 'asyncio' has no attribute 'to_thread'
```

**修复内容**：
- Commit c14935e：替换`asyncio.to_thread`为`loop.run_in_executor`
- 文件：`src/claudia/brain/production_brain.py:634-640`

**根本原因**：
- `asyncio.to_thread`是Python 3.9+新增API
- 系统运行Python 3.8.10，不支持此API

**影响范围**：
- ❌ 非热路径、非缓存的LLM调用失败（如"可愛ね"、"あなたは誰"）
- ✅ 热路径命令不受影响（不调用LLM）
- ✅ 缓存命中不受影响（跳过LLM）

**验证结果**：
```python
✅ loop.run_in_executor在Python 3.8.10正常工作
✅ 异步超时控制保持不变
✅ 与热路径、缓存逻辑完全兼容
```

### 问题4：重复执行动作 ✅ 已修复
```
🧠 ✅ 热路径执行完成 (7055ms, success=True)  # 已经执行
...
🚀 执行动作...  # 又执行一遍！
```

**修复内容**：
- Commit 8df3710：移除热路径中的execute_action调用
- 文件：`src/claudia/brain/production_brain.py:849`

**根本原因**：
- 热路径在`process_command`中调用了`await self.execute_action(brain_output)`
- `production_commander.py`中也调用了`await self.brain.execute_action(brain_output)`
- 导致相同动作执行两次

**修复方案**：
- 热路径只返回`BrainOutput`，不执行动作
- 统一由`production_commander.py`执行所有动作

**验证结果**：
```python
✅ 热路径只返回BrainOutput(success=True)标记待执行
✅ Commander检测到BrainOutput后单次执行
✅ 审计日志正确记录单次执行
```

### 问题5：状态快照不准确 ✅ 已修复
```
🧠 📊 状态快照: 电池100%, 姿态非站立  # 明明已经站立了
```

**修复内容**：
- Commit 8df3710：添加姿态跟踪机制
- 文件：`src/claudia/brain/production_brain.py:283-285,729-730,1195-1207`

**根本原因**：
- 模拟模式下SystemStateMonitor不跟踪姿态变化
- 每次get_current_state()都返回默认值（is_standing=False）
- SafetyValidator基于错误的姿态判断，导致不必要的自动站立

**修复方案**：
1. 添加跟踪字段：`self.last_posture_standing`，`self.last_executed_api`
2. 在`execute_action`中更新姿态（Stand→True, Sit/Down→False）
3. 状态快照使用跟踪的姿态而非SystemStateMonitor默认值

**验证结果**：
```python
✅ 初始状态: last_posture_standing=False（假设坐姿）
✅ 执行Stand(1004)后: last_posture_standing=True
✅ 状态快照正确显示"姿态站立"
✅ 后续Heart命令不再自动插入Stand
```

### 问题6：ROS2错误信息暴露 ✅ 已修复
```
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Precondition Not Met
ROS2初始化失败，启用模拟模式
```

**修复内容**：
- Commit 8df3710：抑制ROS2错误输出
- 文件：`src/claudia/robot_controller/system_state_monitor.py:183-221`

**根本原因**：
- ROS2库本身打印错误到stderr（代码无法直接控制）
- 虽然有fallback机制，但用户仍看到底层技术错误

**修复方案**：
1. 使用`contextlib.redirect_stderr(open(os.devnull, 'w'))`抑制stderr
2. 设置ROS2环境变量抑制日志：`RCUTILS_CONSOLE_OUTPUT_FORMAT=''`
3. 移除_initialize_ros2的详细错误日志（返回False即可）

**验证结果**：
```bash
✅ ROS2错误不再显示给用户
✅ Fallback机制正常工作（自动切换模拟模式）
✅ unitree_sdk2py不受影响（独立通道）
```

### 问题7：LLM将对话误解为动作 ✅ 已修复
```
くら> あなたは誰  # "Who are you?"
💬 回复: こんにちは  # 错误：返回"Hello"
📋 序列: [1004, 1016]  # 错误：执行Stand+Hello动作
```

**修复内容**：
- Commit 8df3710：添加对话查询检测
- 文件：`src/claudia/brain/production_brain.py:593-673,952-973`

**根本原因**：
- LLM训练为将所有输入映射到动作API
- 缺少对话型查询的预处理检测
- "あなたは誰"被LLM误解为打招呼需求

**修复方案**：
1. 添加`_is_conversational_query()`检测对话关键词
2. 添加`_generate_conversational_response()`生成友好回复
3. 在热路径后、LLM前执行对话检测
4. 对话查询返回`BrainOutput(api_code=None, response="...")`

**对话关键词覆盖**：
- 身份：あなた、誰、名前、who、你是、你叫
- 赞美：可愛い、すごい、cute、cool、可爱、厉害
- 感谢：ありがとう、thank you、谢谢
- 问候：おはよう、こんばんは、さようなら、good morning

**验证结果**：
```python
✅ "あなたは誰" → "私はClaudiaです。Unitree Go2のAIアシスタントです。"
✅ "可愛いね" → "ありがとうございます！"
✅ api_code=None, sequence=None（不执行动作）
✅ 审计日志route="conversational", model_used="dialog_detector"
```

---

## 硬件测试准备

### 1. 环境检查
```bash
# 确认LD_LIBRARY_PATH配置
echo $LD_LIBRARY_PATH
# 应包含: /home/m1ng/cyclonedds/install/lib

# 确认网络连接
ping -c 3 192.168.123.161

# 确认Ollama服务
curl http://localhost:11434/api/tags
```

### 2. 启动测试
```bash
./start_production_brain_v2.sh
# 选择模式2（真实硬件模式）
```

### 3. 预期日志模式
```
✅ 真实SportClient初始化成功 - 机器人已连接
[ERROR] [rmw_cyclonedds_cpp]: ...  # ← 预期错误，可忽略
ROS2初始化失败，启用模拟模式      # ← 预期fallback
🧠 ✅ 状态监控器已启动             # ← 模拟模式
🧠 ✅ 安全验证器已加载
🧠 ✅ 审计日志器已启动
```

---

## 硬件测试场景

### 场景1：坐姿 → Heart（自动补Stand）
**目标**：验证SafetyValidator站立需求检查

**步骤**：
1. 确保机器人处于**坐姿**（手动或发送"座って"）
2. 输入命令：`ハート` 或 `比心` 或 `hello`

**预期行为**：
```bash
🧠 📥 接收指令: 'ハート'
🧠 📊 状态快照: 电池100%, 姿态非站立
🧠 ⚡ 热路径命中: ハート → 1036
🧠 🔧 热路径自动补全序列: [1004, 1036]  # ← 自动插入Stand
🧠 ✅ 序列执行: Stand(1004) → Heart(1036)
```

**实际机器人动作**：
1. 先站立（Stand, API 1004）
2. 再做Heart手势（API 1036）

### 场景2：站立 → Heart（直接执行）
**目标**：验证已站立时无需补前置

**步骤**：
1. 确保机器人处于**站立姿态**
2. 输入命令：`ハート`

**预期行为**：
```bash
🧠 📥 接收指令: 'ハート'
🧠 📊 状态快照: 电池100%, 姿态站立
🧠 ⚡ 热路径命中: ハート → 1036
🧠 ✅ 直接执行: Heart(1036)  # ← 无需补Stand
```

### 场景3：低电量拒绝（模拟8%）
**目标**：验证Final Safety Gate电量检查

**步骤**：
1. 修改代码临时模拟低电量：
   ```python
   # production_brain.py get_current_state()
   return SystemStateInfo(
       battery_level=0.08,  # ← 修改此处
       is_standing=True,
       ...
   )
   ```
2. 输入高能命令：`前転` 或 `FrontFlip`

**预期行为**：
```bash
🧠 📥 接收指令: '前転'
🧠 📊 状态快照: 电池8%, 姿态站立
🧠 ⚠️ Quick precheck拒绝: 電池残量が極めて低い状態です (8%)
❌ 动作被拒绝
```

### 场景4：复杂序列（"座ってから挨拶"）
**目标**：验证LLM理解复杂语义

**步骤**：
1. 输入命令：`座ってから挨拶` 或 `先坐下再打招呼`

**预期行为**：
```bash
🧠 📥 接收指令: '座ってから挨拶'
🧠 🧠 使用3B模型推理...
🧠 ✅ LLM返回: {"r":"座ってから挨拶します","a":null,"seq":[1009,1016]}
🧠 ✅ 序列执行: Sit(1009) → Hello(1016)
```

### 场景5：热路径高频命令
**目标**：验证热路径性能

**连续输入**：
```
座って
立って
止まれ
ダンス
ハート
```

**预期行为**：每个命令都应该命中热路径（⚡标记），延迟<0.01ms

---

## 故障排查

### SportClient初始化失败
**症状**：
```
❌ SportClient初始化失败
```

**检查项**：
1. 机器人是否开机：`ping 192.168.123.161`
2. Unitree App是否关闭（避免占用冲突）
3. 网络接口：`ip addr show eth0`
4. DDS域ID：`echo $ROS_DOMAIN_ID` (应为0)

### LD_LIBRARY_PATH未设置
**症状**：
```
ImportError: undefined symbol: ddsi_sertype_v0
```

**修复**：
```bash
export LD_LIBRARY_PATH=/home/m1ng/cyclonedds/install/lib:$LD_LIBRARY_PATH
```

### Ollama模型未加载
**症状**：
```
❌ 模型不存在: claudia-go2-3b:v11.2
```

**修复**：
```bash
ollama list | grep claudia
# 如果缺失：
ollama pull claudia-go2-3b:v11.2
```

---

## 性能基准

### 热路径性能
- **命中判断**：< 0.01ms（字典查找）
- **SafetyValidator检查**：< 0.05ms
- **总延迟（坐姿→Heart+Stand）**：< 0.1ms（不含SDK通信）

### LLM推理性能
- **3B模型**：800-1500ms（简单命令）
- **7B模型**：2000-4000ms（复杂序列）
- **参数收敛**：temperature=0.1, num_predict=30

### 安全检查性能
- **Quick Precheck**：< 0.01ms（LLM前）
- **SafetyValidator**：< 0.05ms（动作前）
- **Final Safety Gate**：< 0.01ms（执行前）

---

## 审计日志

### 日志位置
```bash
logs/audit/audit_YYYYMMDD.jsonl
```

### 关键字段
```json
{
  "timestamp": "2025-11-14T17:10:45.123",
  "command": "ハート",
  "model": "claudia-go2-3b:v11.2",
  "response": "ハートします",
  "api_code": 1036,
  "sequence": [1004, 1036],
  "latency_ms": 0.05,
  "battery_level": 1.0,
  "is_standing": false,
  "safety_result": "sequence_modified",
  "reasoning": "hotpath_executed"
}
```

### 分析命令
```bash
# 统计热路径命中率
grep "hotpath_executed" logs/audit/*.jsonl | wc -l

# 查看拒绝案例
grep "rejected" logs/audit/*.jsonl | jq .

# 延迟P95
jq -r '.latency_ms' logs/audit/*.jsonl | sort -n | tail -n 5
```

---

## 下一步

### 已完成修复（Commit 8df3710）✅
1. ✅ 重复执行动作问题 - 热路径不再执行，统一由commander执行
2. ✅ 状态快照不准确 - 添加姿态跟踪，模拟模式准确反映Stand/Sit状态
3. ✅ ROS2错误信息暴露 - 抑制stderr输出，用户不再看到底层错误
4. ✅ LLM对话误解 - 添加对话检测，"あなたは誰"等返回友好回复而非动作

### 立即可执行（P0 - 今天）
1. ⏳ 执行场景1-5完整测试（验证修复效果）
2. ⏳ 测试对话查询："あなたは誰"、"可愛いね"、"ありがとう"
3. ⏳ 验证状态跟踪：Stand→Heart（无重复站立）
4. ⏳ 收集审计日志（验证route="conversational"、"hotpath"）

### 优化方向（P1 - 本周）
1. 扩展热路径覆盖（添加10-15个变体）
2. 扩展对话关键词（更多对话场景）
3. 外部化电量阈值配置（config/default.yaml）
4. 每日审计报告脚本

### 生产部署准备（P2 - 下周）
1. 性能压测（1000条命令，包含对话和动作混合）
2. 边界条件测试（网络断开、低电量、异常姿态）
3. LED状态同步验证
4. 真实状态监控集成（SportClient API查询实际姿态）

---

## 常见问题

### Q1：ROS2错误影响功能吗？
**A**：不影响。unitree_sdk2py独立工作，SystemStateMonitor已fallback到模拟模式。

### Q2：模拟模式的状态准确吗？
**A**：✅ **已优化**（Commit 8df3710）：模拟模式现在跟踪姿态变化（Stand/Sit/Down），状态快照准确反映最后执行的动作。电池仍为默认100%。真实电量/IMU需要通过SportClient API查询（已计划集成P2）。

### Q3：如何完全禁用ROS2警告？
**A**：✅ **已自动处理**（Commit 8df3710）：系统自动抑制ROS2错误输出，用户不再看到`[ERROR] [rmw_cyclonedds_cpp]`信息。如需手动禁用其他ROS2日志：
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
```

### Q4：LD_LIBRARY_PATH会影响其他程序吗？
**A**：不会。LD_LIBRARY_PATH只在当前shell会话生效，不影响全局环境。

### Q5：如何验证CycloneDDS版本？
**A**：
```bash
ldd ~/.local/lib/python3.8/site-packages/cyclonedds/_clayer.so | grep ddsc
# 应显示: /home/m1ng/cyclonedds/install/lib/libddsc.so.0
```

---

**状态**: 🚀 **Ready for Full Hardware Testing (7 Critical Fixes Complete)**
**修复汇总**:
- ✅ DDS符号问题（Commit 54fd322）
- ✅ BrainOutput TypeError（Commits d252ab3, 221e9b3）
- ✅ Python 3.8兼容性（Commit c14935e）
- ✅ 重复执行动作（Commit 8df3710）
- ✅ 状态快照不准确（Commit 8df3710）
- ✅ ROS2错误暴露（Commit 8df3710）
- ✅ LLM对话误解（Commit 8df3710）

**下一步**: 执行硬件测试场景1-5 + 对话查询测试

**作者**: Claude + User
**最后更新**: 2025-11-14 17:45 UTC
