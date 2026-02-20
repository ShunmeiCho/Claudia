# Claudia机器人测试框架

## 📁 测试结构

```
test/
├── __init__.py              # Python包初始化
├── README.md                # 测试说明文档（本文件）
├── conftest.py              # pytest配置
├── run_tests.py             # 统一测试运行器
├── unit/                    # 单元测试
│   ├── __init__.py
│   └── test_*.py            # 单元测试文件
├── integration/             # 集成测试
│   ├── __init__.py
│   └── test_*.py            # 集成测试文件
├── hardware/                # 硬件测试
│   ├── __init__.py
│   ├── test_unitree_connection.py      # Unitree机器人连接测试
│   ├── test_sportclient_connection.py  # 运动客户端连接测试
│   ├── test_robot_state_reading.py     # 机器人状态读取测试
│   ├── test_basic_control_commands.py  # 基础控制命令测试
│   ├── test_communication_performance.py # 通信性能测试 (任务3.7)
│   └── test_*.py            # 其他硬件测试文件
├── test_claudia_llm_performance.py     # LLM性能测试 (任务11)
└── utils/                   # 测试工具
    ├── __init__.py
    └── test_helpers.py      # 测试辅助函数
```

## 🚀 快速开始

### 运行所有测试
```bash
python3 test/run_tests.py
```

### 运行特定类型的测试
```bash
python3 test/run_tests.py --type hardware
python3 test/run_tests.py --type unit
python3 test/run_tests.py --type integration
```

### 运行单个测试文件
```bash
python3 test/hardware/test_unitree_connection.py
python3 test/test_claudia_llm_performance.py --model claudia-v3.2:3b
```

## 🤖 硬件测试详细说明

### 任务3.7: 通信性能测试
**文件**: `test/hardware/test_communication_performance.py`
**目标**: 验证控制命令延迟consistently <50ms，评估整体系统响应性

**运行方式**:
```bash
# 设置正确的环境变量
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 运行测试
python3 test/hardware/test_communication_performance.py
```

**测试内容**:
- 📊 测量控制命令的延迟（毫秒级精度）
- 📈 进行100次测试获得统计数据
- 🎯 验证95%的命令延迟<50ms
- 💾 生成详细的性能报告
- 📁 自动保存测试结果到logs/目录

**安全特性**:
- 使用最小化的安全命令进行测试
- 机器人保持静止状态，仅测试通信延迟
- 不执行可能导致机器人移动的动作

### 任务11: LLM性能测试
**文件**: `test/test_claudia_llm_performance.py`
**目标**: 评估Claudia机器人日语命令处理LLM模型的准确率和响应性能

**运行方式**:
```bash
# 单模型测试
python3 test/test_claudia_llm_performance.py --model claudia-v3.2:3b

# 多模型对比测试
python3 test/test_claudia_llm_performance.py --compare claudia-v3.1:3b claudia-v3.2:3b
```

**测试内容**:
- 🎯 **准确率测试**: 19种日语命令的识别准确率
- ⚡ **响应时间测试**: 各类命令的处理延迟统计
- 🔍 **边界情况测试**: 空输入、英语、复合命令等处理
- 📊 **分类性能分析**: 基本动作、紧急命令、表演动作、状态查询
- 💾 **详细报告生成**: JSON格式的完整测试结果

**测试类别**:
- **basic**: 基本动作 (座る、立つ、歩く、回る)
- **emergency**: 紧急命令 (停止、ストップ、ダンプ)
- **performance**: 表演动作 (ダンス、挨拶、ストレッチ)
- **status**: 状态查询 (状態、バランス)
- **variant**: 表达变体 (お座り、起立、前進、ハロー)

**期望性能指标**:
- 总体准确率: ≥90%
- 平均响应时间: ≤2秒
- 紧急命令准确率: 100%

### 其他硬件测试

#### 基础连接测试
```bash
# Unitree连接测试
python3 test/hardware/test_unitree_connection.py

# SportClient连接测试  
python3 test/hardware/test_sportclient_connection.py
```

#### 功能测试
```bash
# 机器人状态读取测试
python3 test/hardware/test_robot_state_reading.py

# 基础控制命令测试
python3 test/hardware/test_basic_control_commands.py
```

## 🔧 环境要求

### 必需环境变量
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Python依赖
- Python 3.8+
- unitree_sdk2py (已安装在 `~/claudia/unitree_sdk2_python`)
- 标准库: time, statistics, datetime, typing

### 网络要求
- Unitree Go2机器人已连接
- 网络接口: eth0 (默认)
- 正确的DDS配置

## 📊 使用pytest运行测试（可选）

如果系统已安装pytest：
```bash
# 运行所有测试
pytest test/ -v

# 运行硬件测试
pytest test/hardware/ --hardware -v

# 运行特定测试
pytest test/hardware/test_communication_performance.py -v
```

## 📝 测试报告

测试结果将自动保存到以下位置：
- **性能测试结果**: `logs/performance_test_YYYYMMDD_HHMMSS.txt`
- **其他测试日志**: `logs/tests/YYYY MM/`

## ⚠️ 安全注意事项

1. **机器人安全**: 确保机器人周围有足够的安全空间
2. **网络连接**: 确认机器人网络连接稳定
3. **紧急停止**: 随时准备使用遥控器急停按钮
4. **电池状态**: 确保机器人电池电量充足

## 🆘 故障排除

### 常见问题

1. **模块导入错误**
   ```bash
   # 确认SDK路径正确
   ls ~/claudia/unitree_sdk2_python
   ```

2. **DDS连接失败**
   ```bash
   # 检查环境变量
   echo $RMW_IMPLEMENTATION
   
   # 确认应该是: rmw_cyclonedds_cpp
   ```

3. **网络连接问题**
   ```bash
   # 检查网络接口
   ip addr show eth0
   
   # 检查机器人连接
   ping <robot_ip>
   ```

## 📚 更多信息

- 详细的环境配置请参考项目根目录的安装文档
- 硬件问题排除请参考 `Docs/ENVIRONMENT_TROUBLESHOOTING.md`
- 项目管理规则请参考 `.cursor/rules/project_management.mdc` 