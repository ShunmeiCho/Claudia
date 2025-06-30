# Unitree Go2 IMU验证系统

## 概述

本系统为Unitree Go2机器人的IMU（惯性测量单元）提供全面的验证和测试功能。通过脚本驱动的数据采集、实时可视化和全面文档记录，确保IMU传感器系统的准确性和可靠性。

## 功能特性

### 🔧 核心功能
- **自动化IMU初始化和配置**
- **实时数据采集和处理**
- **多维度传感器验证**
- **交互式可视化界面**
- **全面的校准分析**
- **详细的验证报告**

### 📊 测试模块
1. **静态稳定性测试** - 验证静止状态下的传感器稳定性
2. **动态响应测试** - 测试运动状态下的响应性能
3. **校准验证分析** - 分析传感器校准质量
4. **实时可视化** - 3D姿态和时序图显示
5. **综合评估报告** - 生成详细的验证文档

## 系统要求

### 硬件要求
- Unitree Go2机器人（带IMU传感器）
- 开发主机（Ubuntu 20.04+）
- 稳定的网络连接

### 软件依赖
```bash
# Python包依赖
numpy>=1.20.0
matplotlib>=3.5.0
scipy>=1.7.0
unitree_sdk2py>=1.0.0

# 系统依赖
sudo apt update
sudo apt install python3-dev python3-pip
pip3 install numpy matplotlib scipy
```

## 快速开始

### 1. 环境准备
```bash
# 导航到IMU验证目录
cd scripts/validation/imu/imu_validation

# 检查Python环境
python3 --version  # 需要Python 3.8+

# 安装依赖
pip3 install -r requirements.txt  # 如果有requirements文件
```

### 2. 配置文件设置
```bash
# 检查配置文件
cat validation_config.json

# 根据需要修改配置
vim validation_config.json
```

### 3. 运行完整验证
```bash
# 执行完整的IMU验证流程
python3 main_validation_script.py

# 或者指定配置文件
python3 main_validation_script.py --config custom_config.json
```

### 4. 查看结果
```bash
# 验证完成后，结果保存在output目录
ls output/imu_validation/

# 查看最新的测试报告
cat output/imu_validation/*/imu_validation_report.txt
```

## 详细使用说明

### 命令行选项
```bash
python3 main_validation_script.py [选项]

选项:
  --config, -c     配置文件路径
  --test-type, -t  测试类型 (full|static|dynamic|calibration|visualization)
  --output, -o     输出目录
  --verbose, -v    详细输出
  --help, -h       显示帮助信息
```

### 测试类型说明

#### 完整验证 (--test-type full)
运行所有验证模块，包括：
- IMU初始化检查
- 静态稳定性测试（60秒）
- 动态响应测试（pitch/roll/yaw测试）
- 校准验证分析（多姿态数据收集）
- 实时可视化功能验证

#### 静态测试 (--test-type static)
专注于静态条件下的传感器稳定性：
- 加速度计稳定性分析
- 陀螺仪偏置测量
- 重力精度验证
- 噪声水平评估

#### 动态测试 (--test-type dynamic)
测试动态条件下的响应性能：
- 响应时间测量
- 跟踪精度分析
- 动态范围评估
- 频率响应特性

#### 校准分析 (--test-type calibration)
深入的校准质量分析：
- 比例因子分析
- 交叉轴耦合测量
- 温度补偿验证
- 姿态融合质量评估

#### 可视化验证 (--test-type visualization)
验证可视化功能：
- 实时3D姿态显示
- 多传感器时序图
- 数据导出功能

## 配置说明

### 主要配置参数

```json
{
  "imu_config": {
    "sampling_rate_hz": 100,        // IMU采样频率
    "test_duration_seconds": 30,    // 默认测试持续时间
    "timeout_seconds": 10,          // 连接超时
    "network_interface": "eth0"     // 网络接口
  },
  "test_parameters": {
    "static_test": {
      "duration_seconds": 60,       // 静态测试时长
      "stability_threshold": {
        "accelerometer_std_max": 0.05,  // 加速度计稳定性阈值
        "gyroscope_std_max": 0.1,       // 陀螺仪稳定性阈值
        "quaternion_drift_max": 0.01    // 四元数漂移阈值
      }
    },
    "dynamic_test": {
      "duration_seconds": 120,      // 动态测试时长
      "response_tests": [           // 动态测试项目
        "pitch_test",
        "roll_test", 
        "yaw_test",
        "translation_test"
      ],
      "response_threshold_ms": 50   // 响应时间阈值(毫秒)
    }
  },
  "visualization": {
    "real_time_plots": true,        // 启用实时绘图
    "plot_update_interval_ms": 100, // 绘图更新间隔
    "max_plot_points": 500,         // 最大绘图点数
    "enable_3d_orientation": true   // 启用3D姿态显示
  },
  "quality_thresholds": {
    "accuracy": {
      "gravity_error_max_percent": 2.0  // 重力测量最大误差百分比
    },
    "noise_levels": {
      "accelerometer_noise_max": 0.02,  // 加速度计最大噪声
      "gyroscope_noise_max": 0.01       // 陀螺仪最大噪声
    }
  }
}
```

## 测试流程详解

### 阶段1: 系统初始化
1. **IMU连接验证** - 检查与Unitree Go2的连接
2. **传感器规格确认** - 验证采样率、量程等参数
3. **数据流测试** - 确认能够正常接收IMU数据
4. **基础功能验证** - 测试数据解析和处理

### 阶段2: 静态稳定性测试
1. **环境准备**
   ```
   请确保机器人处于静止状态
   避免振动和外部干扰
   测试持续时间: 60秒
   ```

2. **数据采集**
   - 连续采集IMU数据
   - 实时监控数据质量
   - 自动检测异常值

3. **稳定性分析**
   - 加速度计各轴标准差
   - 陀螺仪偏置和噪声
   - 四元数一致性检查
   - 重力精度验证

### 阶段3: 动态响应测试
1. **Pitch测试（俯仰）**
   ```
   操作指导: 请缓慢前后倾斜机器人（pitch轴旋转），然后快速回到水平位置
   测试指标: 响应时间、跟踪精度、超调量
   ```

2. **Roll测试（翻滚）**
   ```
   操作指导: 请缓慢左右倾斜机器人（roll轴旋转），然后快速回到水平位置
   测试指标: 响应时间、动态范围、线性度
   ```

3. **Yaw测试（偏航）**
   ```
   操作指导: 请缓慢左右转动机器人（yaw轴旋转），然后快速回到原始方向
   测试指标: 角速度响应、积分漂移、频率响应
   ```

4. **Translation测试（平移）**
   ```
   操作指导: 请平稳地前后、左右移动机器人，避免旋转
   测试指标: 加速度响应、噪声抑制、信号质量
   ```

### 阶段4: 校准验证分析
1. **多姿态数据收集**
   ```
   位置1: 水平静置（正常姿态）
   位置2: 左侧倾斜90度
   位置3: 右侧倾斜90度
   位置4: 前倾90度
   位置5: 后倾90度
   位置6: 倒置180度
   ```

2. **校准质量分析**
   - 重力矢量一致性
   - 比例因子误差
   - 交叉轴耦合
   - 温度系数（如适用）

### 阶段5: 可视化验证
1. **实时绘图测试**
   - 启动多窗口可视化
   - 验证数据更新
   - 测试3D姿态显示
   - 检查绘图性能

2. **数据导出测试**
   - PNG格式图像保存
   - 数据文件导出
   - 可视化统计生成

## 结果解释

### 测试状态代码
- **PASS** - 所有指标符合要求
- **WARNING** - 次要问题，可接受使用
- **FAIL** - 严重问题，需要修复
- **ERROR** - 测试执行错误
- **UNKNOWN** - 状态未知

### 关键指标解释

#### 静态测试指标
- **加速度计稳定性** - 标准差 < 0.05 m/s²
- **陀螺仪偏置** - 偏置量级 < 0.1 rad/s
- **重力精度** - 误差 < 2%
- **四元数一致性** - 幅值误差 < 0.01

#### 动态测试指标
- **响应时间** - < 50ms
- **跟踪精度** - 相关性 > 0.8
- **动态范围** - > 10度角度变化
- **超调量** - < 10%

#### 校准质量指标
- **重力校准** - 误差 < 2%
- **偏置稳定性** - 变化 < 0.05 rad/s
- **交叉轴耦合** - < 10%
- **总体质量分数** - > 0.7

## 故障排除

### 常见问题

#### 1. IMU初始化失败
```
现象: "IMU未初始化，无法进行测试"
解决方案:
- 检查网络连接
- 确认机器人电源状态
- 验证SDK版本兼容性
- 重启网络接口
```

#### 2. 数据采集超时
```
现象: "数据采集超时，强制停止"
解决方案:
- 检查网络延迟
- 增加timeout_seconds配置
- 确认机器人响应正常
- 减少采集持续时间
```

#### 3. 可视化显示失败
```
现象: "可视化启动失败"
解决方案:
- 安装matplotlib依赖
- 配置X11转发（如SSH使用）
- 检查显示环境变量
- 禁用real_time_plots配置
```

#### 4. 测试精度不足
```
现象: 多个测试项目显示FAIL
解决方案:
- 确保测试环境稳定
- 检查机械振动干扰
- 重新校准IMU传感器
- 调整质量阈值配置
```

### 日志分析
```bash
# 查看详细日志
tail -f logs/imu_validation/imu_validation_*.log

# 搜索错误信息
grep "ERROR\|FAIL" logs/imu_validation/imu_validation_*.log

# 分析网络连接问题
grep "timeout\|connection" logs/imu_validation/imu_validation_*.log
```

## 输出文件说明

### 结果目录结构
```
output/imu_validation/YYYYMMDD_HHMMSS/
├── imu_validation_results.json      # 完整JSON结果
├── imu_validation_report.txt        # 简化文本报告
├── imu_timeseries_YYYYMMDD_HHMMSS.png    # 时序图
├── imu_3d_orientation_YYYYMMDD_HHMMSS.png  # 3D姿态图
└── data_exports/                     # 原始数据导出
    ├── static_test_data.json
    ├── dynamic_test_data.json
    └── calibration_data.json
```

### JSON结果格式
```json
{
  "test_info": {
    "start_time": "ISO时间戳",
    "test_version": "1.0.0",
    "robot_model": "Unitree Go2",
    "test_operator": "用户名"
  },
  "initialization": {
    "status": "SUCCESS|FAILED",
    "imu_specs": { ... }
  },
  "static_test": {
    "test_status": "PASS|WARNING|FAIL",
    "accelerometer_stability": { ... },
    "gyroscope_stability": { ... },
    "recommendations": [ ... ]
  },
  "dynamic_test": {
    "pitch_test": { ... },
    "roll_test": { ... },
    "overall": { ... }
  },
  "calibration_analysis": { ... },
  "visualization_test": { ... },
  "overall_assessment": {
    "status": "PASS|WARNING|FAIL",
    "pass_rate_percent": 85.5,
    "overall_conclusion": "结论文本"
  }
}
```

## 最佳实践

### 测试环境
1. **稳定的物理环境**
   - 避免振动和冲击
   - 控制温度变化
   - 减少电磁干扰

2. **网络环境**
   - 使用有线连接
   - 确保低延迟
   - 避免网络拥塞

### 测试操作
1. **准备阶段**
   - 预热机器人系统
   - 检查电池电量
   - 确认传感器状态

2. **执行阶段**
   - 按照提示进行操作
   - 保持动作平稳
   - 避免突然移动

3. **结果分析**
   - 仔细阅读建议
   - 关注关键指标
   - 保存测试记录

## 技术支持

### 联系信息
- 项目仓库: [GitHub链接]
- 技术文档: [文档链接] 
- 问题反馈: [Issue链接]

### 版本历史
- v1.0.0 (2025-06-27) - 初始版本发布
  - 完整的IMU验证功能
  - 实时可视化支持
  - 全面的校准分析

---

**注意**: 本验证系统设计用于Unitree Go2机器人。使用其他机器人型号前请确认兼容性。 