# ⚡ Claudia LLM大脑升级 - 快速启动清单

## 📋 实施状态

### ✅ 已完成（Track A + Track B）

- [x] **异步调用修复**：`production_brain.py` 使用 `asyncio.to_thread`
- [x] **状态监控集成**：实时读取IMU/电量/姿态
- [x] **安全验证器**：`safety_validator.py` 代码层强制规则
- [x] **增强启动脚本**：`start_production_brain_v2.sh` 全面环境检查
- [x] **新Modelfile**：`ClaudiaIntelligent_Qwen7B` 去词表化设计
- [x] **部署脚本**：`deploy_track_b.sh` 自动化创建新模型
- [x] **A/B测试脚本**：`test_ab_quick.py` 20个核心场景对比

---

## 🚀 10分钟验证流程

### Step 1: 环境准备（2分钟）
```bash
cd ~/claudia

# 安装ollama Python库
pip3 install ollama

# 验证Ollama服务
curl http://localhost:11434/api/tags
# 如果失败则启动: nohup ollama serve > /tmp/ollama.log 2>&1 &

# 检查现有模型
ollama list | grep claudia
# 应该看到: claudia-go2-3b:v11.2, claudia-go2-7b:v7
```

### Step 2: Track A验证 - 模拟模式（3分钟）
```bash
# 启动增强版脚本
./start_production_brain_v2.sh
# 选择: 1

# 测试基础命令
输入: 座って
预期: ✅ 返回JSON，api_code=1009

输入: 緊急停止
预期: ✅ 快速响应（<100ms），直接执行Stop

输入: quit
# 退出测试
```

**验证点**：
- [ ] 脚本能正确加载ROS2和Ollama环境
- [ ] 模拟模式能正常响应命令
- [ ] 紧急停止走旁路通道（无LLM调用）
- [ ] 无异步阻塞报错

### Step 3: Track B部署（3分钟）
```bash
# 自动部署新模型
./deploy_track_b.sh

# 预期输出:
# ✅ qwen2.5:7b已存在（或自动拉取）
# ✅ 创建claudia-intelligent-7b:v1成功
# ✅ 测试: "座って" → 1009
# ✅ 测试: "立って" → 1004

# 验证模型已创建
ollama list | grep claudia-intelligent
# 应该看到: claudia-intelligent-7b:v1
```

### Step 4: A/B对比测试（2分钟）
```bash
# 运行快速对比
python3 test_ab_quick.py

# 查看输出关键指标:
# 【Baseline】claudia-go2-3b:v11.2
#   准确率: X%
#   延迟: Yms
#
# 【新模型】claudia-intelligent-7b:v1
#   准确率: X%
#   延迟: Yms
#
# 📈 对比结果:
#   准确率变化: +/- Z%
#   延迟变化: +/- Wms
```

**决策标准**：
- ✅ 通过：准确率 ≥ 基线-5%，延迟 ≤ 2000ms
- ⚠️  待优化：准确率下降>5% 或 延迟>2000ms
- ❌ 失败：JSON合规率<100% 或 大量错误

---

## 🎯 关键测试用例

### 基础命令（必须100%）
```
座って → 1009 (Sit)
立って → 1004 (Stand)
止まって → 1003 (Stop)
```

### 语义理解（Track B核心优势）
```
疲れた → 1009 (Sit)     # "累了"应理解为休息→坐下
可愛い → 1036 (Heart)    # "可爱"应理解为心形手势
元気 → 1023 (Dance)      # "有活力"应理解为跳舞
挨拶して → 1016 (Hello)  # "打招呼"应理解为Hello动作
```

### 异常处理
```
空输入 → api_code=None或0
无意义输入 → api_code=None或0
```

---

## 📊 预期结果对比

### Track A（保守改进）
| 指标 | 改进前 | 改进后 | 说明 |
|------|-------|-------|------|
| 准确率 | 85% | 85% | 模型未变，准确率不变 |
| 首次延迟 | ~3000ms | ~500ms | LLM预热 |
| 紧急停止延迟 | ~800ms | <100ms | 旁路通道 |
| 低电量保护 | ❌ | ✅ | 代码层验证 |
| 姿态自适应 | ❌ | ✅ | 自动补全前置动作 |

### Track B（激进重构）
| 指标 | v11.2基线 | v1目标 | 期望 |
|------|----------|-------|------|
| 基础命令 | 100% | ≥95% | 轻微容差 |
| 语义理解 | ~40% | ≥70% | **核心提升** |
| 多语言 | 66% | ≥80% | 支持英文/中文 |
| 平均延迟 | 800ms | ≤2000ms | 7B模型略慢 |

---

## 🔧 常见问题速查

### Q1: pip3 install ollama失败
```bash
# 使用国内镜像
pip3 install ollama -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### Q2: Ollama服务无响应
```bash
# 检查进程
ps aux | grep ollama
# 重启服务
killall ollama
nohup ollama serve > /tmp/ollama.log 2>&1 &
sleep 5
curl http://localhost:11434/api/tags
```

### Q3: ROS2 Topic不可见
```bash
# 确认环境变量
echo $RMW_IMPLEMENTATION  # 应为rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
source .env.ros2
ros2 topic list
```

### Q4: 模型加载超时
```bash
# 首次加载7B模型较慢（~5秒），属正常
# 可选：预加载
ollama run claudia-intelligent-7b:v1 "test" >/dev/null 2>&1 &
```

### Q5: A/B测试报错
```bash
# 确认两个模型都存在
ollama list | grep -E "(claudia-go2-3b:v11.2|claudia-intelligent-7b:v1)"

# 如果v11.2缺失
ollama pull claudia-go2-3b:v11.2

# 如果v1缺失，重新部署
./deploy_track_b.sh
```

---

## 📁 文件索引

### 核心代码
- [src/claudia/brain/production_brain.py](src/claudia/brain/production_brain.py) - 主大脑逻辑
- [src/claudia/brain/safety_validator.py](src/claudia/brain/safety_validator.py) - 安全验证器

### 启动脚本
- [start_production_brain_v2.sh](start_production_brain_v2.sh) - 增强启动脚本（推荐）
- [start_production_brain.sh](start_production_brain.sh) - 旧版启动脚本

### Track B文件
- [ClaudiaIntelligent_Qwen7B](ClaudiaIntelligent_Qwen7B) - 新Modelfile定义
- [deploy_track_b.sh](deploy_track_b.sh) - 自动部署脚本
- [test_ab_quick.py](test_ab_quick.py) - A/B快速测试

### 文档
- [LLM_BRAIN_UPGRADE_GUIDE.md](LLM_BRAIN_UPGRADE_GUIDE.md) - 完整技术方案
- [QUICK_START_CHECKLIST.md](QUICK_START_CHECKLIST.md) - 本文档
- [README_FINAL_SOLUTION.md](README_FINAL_SOLUTION.md) - SDK互斥问题方案

---

## 🚦 下一步行动

### 立即执行（今天）
1. ⏳ 执行Step 1-4完整验证（10分钟）
2. ⏳ 查看A/B测试结果，决策是否上线Track B
3. ⏳ 如果Track B通过：考虑30%灰度测试

### 短期（1-2天）
- 如果语义理解提升明显（>20%）→ 扩大灰度到70%
- 如果延迟过高（>2000ms）→ 考虑优化或回退到3B
- 如果语义理解提升有限（<10%）→ 考虑Few-shot扩充或LoRA微调

### 中期（1周）
- 真实硬件验证（⚠️ 谨慎，确保周围安全）
- 收集实际使用数据
- 持续监控错误率和性能

---

## ✅ 验证通过标志

```
[✅] ollama Python库已安装
[✅] Ollama服务运行中
[✅] Track A模拟模式测试通过
[✅] Track B模型创建成功
[✅] A/B测试完成，准确率≥目标
[✅] 延迟符合预期（≤2000ms）
[✅] 无阻塞/崩溃报错

→ 可以进入真实硬件测试阶段
```

---

**创建时间**: 2025-11-13
**预计执行时间**: 10分钟
**前置条件**: Jetson Orin NX + ROS2 Foxy + Ollama 0.9.5+
