# Unitree Go2 足端力传感器 ABCD 验证流程

## 概述

本文档说明如何运行Unitree Go2足端力传感器的完整ABCD验证流程。

## 测试阶段

### 阶段A: 数据读取框架验证 ✅
- **目标**: 验证足端力数据读取能力
- **组件**: `FootForceConfig`, `DataCollector`
- **测试内容**: 传感器连接、数据采样率、基本数据格式

### 阶段B: 静态力分布验证 ✅
- **目标**: 验证静态条件下的力分布准确性
- **组件**: `StaticFootForceTester`
- **测试内容**: 零负载测试、静态站立测试、重量分布分析

### 阶段C: 动态响应测试 🆕
- **目标**: 验证动态条件下的传感器响应
- **组件**: `DynamicFootForceTester`
- **测试内容**: 缓慢行走、正常行走、冲击测试

### 阶段D: 综合可视化和文档 🆕
- **目标**: 生成综合报告和可视化
- **组件**: `ComprehensiveFootForceDashboard`
- **输出**: JSON报告、HTML报告、可视化图表

## 运行方式

### 1. 快速组件测试

首先运行快速测试验证所有组件是否正常工作：

```bash
cd /home/m1ng/claudia
python3 scripts/validation/foot_force/run_quick_abcd_test.py
```

**功能**:
- 测试所有ABCD组件导入
- 运行模拟数据测试
- 生成测试报告
- 不需要连接真实机器人

### 2. 完整ABCD验证流程

在确认组件测试通过后，运行完整验证：

```bash
cd /home/m1ng/claudia
python3 scripts/validation/foot_force/run_complete_validation.py
```

**功能**:
- 连接真实Unitree Go2机器人
- 依次执行ABCD四个阶段
- 生成完整验证报告
- 需要机器人网络连接 (192.168.123.161)

## 环境要求

### 系统环境
```bash
# ROS2 Foxy + CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
source cyclonedx_ws/install/setup.bash
```

### Python依赖
- numpy
- matplotlib
- pandas (可选)
- unitree_sdk2py

### 网络连接
- 机器人IP: 192.168.123.161
- 网络接口: eth0

## 输出文件

所有测试结果保存在: `scripts/validation/foot_force/foot_force_validation/output/`

### 目录结构
```
output/
├── logs/                           # 测试日志
│   └── complete_validation_*.log
├── mock_test_report_*.json         # 快速测试报告
├── final_validation_report_*.json  # 完整验证报告
├── dynamic_test_results_*.json     # 动态测试结果
└── comprehensive_report_*.html     # 综合HTML报告
```

## 故障排除

### 1. 组件导入失败
```bash
# 检查Python路径
export PYTHONPATH="/home/m1ng/claudia:$PYTHONPATH"
```

### 2. 机器人连接失败
```bash
# 检查网络连接
ping 192.168.123.161

# 检查CycloneDDS环境
ros2 topic list
```

### 3. 数据收集失败
- 确认机器人已开机并连接
- 验证DDS通信正常
- 检查传感器状态

## 测试流程

### 完整验证流程
1. **预检查**: 网络连接、环境变量
2. **阶段A**: 数据读取框架验证 (5秒)
3. **阶段B**: 静态验证测试 (10秒)
4. **阶段C**: 动态测试套件 (135秒)
   - 缓慢行走 (60秒)
   - 正常行走 (45秒)
   - 冲击测试 (30秒)
5. **阶段D**: 报告生成和可视化
6. **后处理**: 清理资源、生成最终报告

### 预期运行时间
- 快速测试: < 30秒
- 完整验证: 5-10分钟

## 评分标准

### 阶段评分
- **阶段A**: 数据采样率、连接稳定性
- **阶段B**: 零点准确性、力分布平衡 (静态60%权重)
- **阶段C**: 步态一致性、动态响应 (动态40%权重)  
- **阶段D**: 报告生成成功率

### 总体评分
- **A级**: ≥90分 - 优秀
- **B级**: 80-89分 - 良好
- **C级**: 70-79分 - 合格
- **D级**: 60-69分 - 需要改进
- **F级**: <60分 - 不合格

## 重要提醒

⚠️ **安全注意事项**:
- 动态测试期间确保机器人周围有足够空间
- 遵循机器人操作安全规范
- 测试过程中保持人员安全距离

📊 **数据处理**:
- 所有数据自动保存，无需手动备份
- 报告包含时间戳，便于历史对比
- 支持多次运行，结果会累积保存

🔧 **维护建议**:
- 定期运行快速测试验证系统状态
- 在重要测试前运行完整验证
- 保存历史测试数据以进行趋势分析 