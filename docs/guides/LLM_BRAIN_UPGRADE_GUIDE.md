# 🧠 Claudia LLM大脑升级方案 - Track A+B实施指南

## 📋 项目背景

**当前问题诊断**：
- 现有LLM系统本质上是"关键词提取器"，而非真正的语义理解
- Modelfile包含175行硬编码关键词映射（座って→1009, 立って→1004...）
- 缺少状态感知和安全验证层
- 无法理解上下文和语义相似词（如"疲れた"→休息→Sit）

**升级目标**：
实现真正的"去词表化"智能大脑，达到Unitree App "笨笨狗"级别的离线语义理解

---

## 🎯 双轨并行方案

### Track A：保守式改进（兼容现有模型）
**目标**：在不改变模型的前提下，提升系统安全性和稳定性

#### 已实现改进：

1. **异步调用优化** ([production_brain.py:454-503](src/claudia/brain/production_brain.py#L454-L503))
   - 使用`asyncio.to_thread`包装同步LLM调用
   - 防止事件循环阻塞
   - 添加超时保护（默认10秒）

2. **状态感知集成** ([production_brain.py:229-254](src/claudia/brain/production_brain.py#L229-L254))
   - 集成`SystemStateMonitor`实时读取IMU/电量/姿态
   - 每次命令执行前检查机器人当前状态
   - 电量单位归一化（自动处理0-1和百分比两种格式）

3. **代码层安全验证** ([safety_validator.py](src/claudia/brain/safety_validator.py))
   - **规则1**：紧急状态仅允许Stop命令
   - **规则2**：电量<5%仅允许Stop和Sit
   - **规则3**：电量<20%禁止高能动作（Flip/Jump/Pounce）
   - **规则4**：自动补全前置动作（需要站立时自动插入StandUp）
   - **规则5**：错误状态拒绝执行

4. **增强启动脚本** ([start_production_brain_v2.sh](start_production_brain_v2.sh))
   - ROS2 Foxy环境显式source
   - CycloneDDS配置验证（eth0, Domain 0）
   - Ollama服务自动检查和启动
   - 模型可用性预检
   - LLM预热（首次调用延迟优化）
   - 网络连通性测试（ping 192.168.123.161）

5. **紧急旁路系统** ([production_brain.py:520-532](src/claudia/brain/production_brain.py#L520-L532))
   - "緊急停止"/"stop"等命令跳过LLM直接执行
   - 降低停止命令延迟（<50ms）

#### 关键修复：

| 问题 | 修复方案 | 代码位置 |
|------|---------|---------|
| 事件循环阻塞 | `asyncio.to_thread(ollama.chat)` | production_brain.py:467 |
| 未定义计时变量 | `start_time`移至函数入口 | production_brain.py:506 |
| 电量单位不一致 | 归一化检查`>1.0则/100` | safety_validator.py:90-92 |
| 序列/单步二义性 | `should_use_sequence_only`标志 | safety_validator.py:126 |
| 高风险动作默认启用 | `enable_high_risk_actions=False` | production_brain.py:244 |

---

### Track B：激进式重构（去词表化）
**目标**：彻底移除关键词映射，使用能力描述实现真正语义理解

#### 新Modelfile设计 ([ClaudiaIntelligent_Qwen7B](ClaudiaIntelligent_Qwen7B))

**核心变化**：

```diff
- OLD (v11.2):
  sit→{"r":"座ります","a":1009}
  stand→{"r":"立ちます","a":1004}
  hello→{"r":"こんにちは","a":1016}
  ...（175行关键词表）

+ NEW (v1):
  【利用可能な動作】
  - Sit(1009): 座る。疲れた時や休息したい時に使用
  - Stand(1004): 立ち上がる。動作開始前の基本姿勢
  - Hello(1016): 手を振って挨拶する
  ...（能力描述）

  【Few-shot例】
  Q: 疲れた
  A: {"intent":"休息したい","action":"Sit","api_code":1009}

  Q: 可愛い動作して
  A: {"intent":"可愛らしいジェスチャー","action":"Heart","api_code":1036}
```

**新特性**：
1. **语义推理**：通过动作描述理解意图（疲れた→休息→Sit）
2. **上下文理解**：可解释为何选择该动作（"intent"字段）
3. **物理约束内嵌**：Modelfile中声明电量/姿态限制
4. **Few-shot学习**：提供典型语义理解案例

#### 部署工具 ([deploy_track_b.sh](deploy_track_b.sh))

```bash
# 自动化部署流程
./deploy_track_b.sh
# 1. 拉取qwen2.5:7b基础模型
# 2. 创建claudia-intelligent-7b:v1
# 3. 运行sanity测试
```

#### A/B测试脚本 ([test_ab_quick.py](test_ab_quick.py))

**测试矩阵**（20个核心场景）：
- **基础精确匹配**（5条）：座って、立って、ハート、ダンス、stop
- **语义理解**（10条）：疲れた→Sit、可愛い→Heart、元気→Dance...
- **多语言**（3条）：sit、cute、坐下
- **异常处理**（2条）：空输入、无意义输入

**验收标准**：
- 准确率≥旧模型-5%（允许小幅下降）
- 平均延迟≤2000ms
- JSON合规率100%

---

## 🚀 实施步骤

### Phase 1：环境准备

```bash
cd ~/claudia

# 1. 安装Python ollama库
pip3 install ollama

# 2. 验证Ollama服务
curl http://localhost:11434/api/tags
ollama list | grep claudia

# 3. 验证ROS2环境
source /opt/ros/foxy/setup.bash
ros2 topic list | grep sportmodestate

# 4. 网络连通性
ping -c 1 192.168.123.161
```

### Phase 2：Track A验证（模拟模式）

```bash
# 1. 使用新启动脚本
./start_production_brain_v2.sh
# 选择: 1) 模拟模式

# 2. 测试核心功能
输入: 座って
预期: ✅ {"response":"座ります","api_code":1009}

输入: 立って
预期: ✅ {"response":"立ちます","api_code":1004}

输入: 緊急停止
预期: ✅ 快速响应（<100ms），无LLM调用

# 3. 测试安全验证
# （需要手动修改state_monitor返回值测试）
模拟电量5%: 仅允许Stop/Sit
模拟坐姿: Hello命令自动插入StandUp
```

### Phase 3：Track B部署与测试

```bash
# 1. 部署新模型
./deploy_track_b.sh
# 预期输出:
# ✅ qwen2.5:7b已存在
# ✅ 创建claudia-intelligent-7b:v1成功
# ✅ 基础测试通过

# 2. 运行A/B对比
python3 test_ab_quick.py
# 预期输出:
# 【Baseline】claudia-go2-3b:v11.2
#   准确率: ~85% (17/20)
#   JSON合规率: 100%
#   平均延迟: ~800ms
#
# 【新模型】claudia-intelligent-7b:v1
#   准确率: ≥80% (目标)
#   延迟: ≤2000ms

# 3. 检查关键语义理解案例
期待改进:
  "疲れた" → 1009 (Sit) ✅
  "可愛い" → 1036 (Heart) ✅
  "元気" → 1023 (Dance) ✅
```

### Phase 4：真实硬件验证（谨慎）

```bash
# ⚠️ 仅在模拟模式测试通过后执行

# 1. 硬件准备
- 机器人电量>50%
- 清理2m半径活动空间
- 关闭Unitree App
- 准备遥控器紧急停止

# 2. 启动硬件模式
./start_production_brain_v2.sh
# 选择: 2) 真实硬件模式

# 3. 基础动作验证
座って → 观察机器人坐下
立って → 观察机器人站立
止まって → 立即停止

# 4. 语义理解验证
疲れた → 应坐下（而非错误）
挨拶して → 应Hello（而非无响应）
```

---

## 📊 验收标准

### Track A（必须100%通过）

| 项目 | 标准 | 验证方法 |
|------|------|---------|
| 异步调用 | 无阻塞，超时生效 | 长时间命令不卡死界面 |
| 状态读取 | 正确读取IMU/电量 | 打印当前状态日志 |
| 安全规则 | 低电量拒绝高能动作 | 模拟5%电量测试 |
| 紧急旁路 | "stop"响应<100ms | 计时器验证 |
| 姿态自动补全 | 坐姿Hello自动站立 | 观察动作序列 |

### Track B（目标）

| 指标 | 基线(v11.2) | 新模型(v1) | 状态 |
|------|------------|-----------|------|
| 基础命令准确率 | 100% (5/5) | ≥95% | 待测 |
| 语义理解准确率 | ~40% (4/10) | ≥70% | 待测 |
| 多语言准确率 | 66% (2/3) | ≥80% | 待测 |
| 平均延迟 | ~800ms | ≤2000ms | 待测 |
| JSON合规率 | 100% | 100% | 待测 |

---

## 🔄 渐进式上线策略

### Stage 1：灰度测试（0% → 30%）
```python
# production_brain.py中添加流量切分
import random
if random.random() < 0.3:
    result = await self._call_ollama_v2("claudia-intelligent-7b:v1", command)
else:
    result = await self._call_ollama_v2("claudia-go2-3b:v11.2", command)
```

**监控指标**：
- 准确率变化
- 延迟P95/P99
- 错误率

**决策点**：
- 如果准确率提升>10% → 扩大到70%
- 如果延迟P95>3000ms → 回滚

### Stage 2：扩大灰度（30% → 70%）
- 持续监控1周
- 收集用户反馈（如果有）
- 分析失败案例

### Stage 3：全量上线（70% → 100%）
- 最终决策前review所有metrics
- 保留v11.2作为降级方案

---

## 🐛 已知问题与TODO

### Track A
- [x] 事件循环阻塞 ✅
- [x] 电量单位归一化 ✅
- [x] 序列执行二义性 ✅
- [ ] 测试用例需要实际state注入（当前用文本标记）
- [ ] 高风险动作的动态开关（当前硬编码False）

### Track B
- [ ] Few-shot案例可能需要扩充（目前仅2个）
- [ ] 7B模型首次加载延迟较高（~3-5秒）
- [ ] 如果语义理解不理想，可能需要LoRA微调
- [ ] 多语言支持待验证（英文/中文）

---

## 📁 关键文件清单

### 新增文件
```
~/claudia/
├── start_production_brain_v2.sh      # Track A增强启动脚本
├── ClaudiaIntelligent_Qwen7B         # Track B新Modelfile
├── deploy_track_b.sh                 # Track B自动部署
├── test_ab_quick.py                  # A/B快速对比测试
└── LLM_BRAIN_UPGRADE_GUIDE.md        # 本文档
```

### 修改文件
```
src/claudia/brain/
├── production_brain.py               # 异步调用+状态集成+安全验证
└── safety_validator.py               # 新增：代码层安全规则
```

### 配置文件
```
.env.ros2                              # ROS2环境变量（需source）
cyclonedds.xml                         # DDS配置（eth0）
```

---

## 🎓 技术细节

### Ollama Python库使用
```python
import ollama

# 检查模型存在性
ollama.show("claudia-go2-3b:v11.2")

# 结构化输出
response = ollama.chat(
    model="claudia-go2-3b:v11.2",
    messages=[{'role': 'user', 'content': '座って'}],
    format='json',  # 强制JSON输出
    options={
        'temperature': 0.1,   # 低温度保证稳定性
        'num_predict': 30,    # 限制token数
        'top_p': 0.7
    }
)
data = json.loads(response['message']['content'])
```

### 状态监控器集成
```python
from claudia.robot_controller.system_state_monitor import (
    create_system_state_monitor, SystemStateInfo, SystemState
)

monitor = create_system_state_monitor(
    node_name="claudia_brain_monitor",
    update_rate=5.0  # 5Hz更新频率
)
monitor.initialize()
monitor.start_monitoring()

# 读取当前状态
state = monitor.get_current_state()
print(f"Battery: {state.battery_level*100:.1f}%")
print(f"Standing: {state.is_standing}")
print(f"System: {state.state.name}")
```

### 安全验证器用法
```python
from claudia.brain.safety_validator import get_safety_validator

validator = get_safety_validator(enable_high_risk=False)
result = validator.validate_action(api_code=1030, state=current_state)

if not result.is_safe:
    print(f"拒绝: {result.reason}")
elif result.modified_sequence:
    print(f"自动补全: {result.modified_sequence}")
    if result.should_use_sequence_only:
        api_code = None  # 只执行序列，避免重复
```

---

## 📞 故障排除

### Ollama服务无法连接
```bash
curl http://localhost:11434/api/tags
# 如果失败:
ps aux | grep ollama
killall ollama
nohup ollama serve > /tmp/ollama.log 2>&1 &
```

### 模型不存在
```bash
ollama list | grep claudia
# 如果缺失:
ollama pull claudia-go2-3b:v11.2
ollama pull qwen2.5:7b
```

### ROS2 Topic不可见
```bash
echo $RMW_IMPLEMENTATION  # 应为rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
source .env.ros2
ros2 topic list
```

### 安全验证误拦截
```bash
# 临时禁用高风险检查
# 在production_brain.py:244修改
self.safety_validator = get_safety_validator(enable_high_risk=True)
```

---

## 🎯 下一步行动

### 立即执行
1. ✅ 代码实施完成（Track A+B）
2. ⏳ 安装ollama库：`pip3 install ollama`
3. ⏳ Track A模拟测试
4. ⏳ Track B部署与A/B测试

### 短期（1-2天）
- 收集A/B测试数据
- 决策是否上线Track B
- 修复发现的问题

### 中期（1周）
- 真实硬件验证
- 性能调优（如果延迟过高）
- 考虑LoRA微调（如果语义理解不足）

---

## 📚 参考文档

- [Ollama Python Library](https://github.com/ollama/ollama-python)
- [Qwen2.5模型卡片](https://huggingface.co/Qwen/Qwen2.5-7B-Instruct)
- `docs/CORRECT_LLM_ARCHITECTURE.md` - 原架构设计
- `docs/SDK_LIMITATION_ANALYSIS.md` - SDK限制分析
- `test/README.md` - 测试框架说明

---

**最后更新**: 2025-11-13
**实施者**: Claude Code
**状态**: ✅ 代码完成，待测试验证
