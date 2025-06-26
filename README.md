# Claudia智能四足机器人系统

<div align="center">

![Claudia Logo](assets/claudia-logo.png)

**基于Unitree Go2 R&D Plus平台的智能四足机器人系统**  
*通过深度集成大语言模型技术实现高级自然语言交互功能*

[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://python.org)
[![ROS2 Version](https://img.shields.io/badge/ROS2-Foxy-green.svg)](https://docs.ros.org/en/foxy/)
[![Platform](https://img.shields.io/badge/platform-NVIDIA%20Jetson%20Orin%20NX-brightgreen.svg)](https://developer.nvidia.com/embedded/jetson-orin)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

</div>

## 🎯 **项目概述**

Claudia（日语读音：くら）是一个革命性的智能四足机器人系统，专为大学开放日等公共活动设计。系统集成了最新的AI技术，包括大语言模型、语音识别、计算机视觉和自主导航，为用户提供自然流畅的日语交互体验。

### **✨ 核心特性**

- 🤖 **自然语言交互**: 基于Qwen2.5-7B LLM的日语对话系统
- 🎙️ **智能语音识别**: NVIDIA Parakeet-TDT日语ASR，准确率95%+
- 🔊 **自然语音合成**: 日语TTS系统，支持情感表达
- 👁️ **双摄像头视觉**: Go2主摄+RealSense D435i深度相机
- 🧭 **自主导航**: 无监督SLAM+语义路径规划
- 💡 **智能LED交互**: 12颗可编程LED状态指示
- 🚶 **丰富动作库**: 15+预设动作，支持动态序列组合## 🏗️ **技术架构**

### **硬件平台**
- **机器人**: Unitree Go2 R&D Plus (足端力传感器+深度相机)
- **计算平台**: NVIDIA Jetson Orin NX (Ubuntu 20.04.5 LTS)
- **传感器**: 4D LiDAR L1, 双摄像头, IMU, 足端力传感器
- **通信**: CycloneDDS + ROS2 Foxy原生通信

### **软件架构**
```
┌─────────────────────────────────────────────────────────────┐
│                    Claudia 系统架构                          │
├─────────────────┬─────────────────┬─────────────────────────┤
│   AI组件层      │   感知层        │      控制层             │
│                 │                 │                         │
│ ┌─────────────┐ │ ┌─────────────┐ │ ┌─────────────────────┐ │
│ │ Qwen2.5 LLM │ │ │ 4D LiDAR L1 │ │ │   Unitree SDK2      │ │
│ └─────────────┘ │ └─────────────┘ │ └─────────────────────┘ │
│ ┌─────────────┐ │ ┌─────────────┐ │ ┌─────────────────────┐ │
│ │ Parakeet ASR│ │ │Go2 主摄像头  │ │ │   动作控制器        │ │
│ └─────────────┘ │ └─────────────┘ │ └─────────────────────┘ │
│ ┌─────────────┐ │ ┌─────────────┐ │ ┌─────────────────────┐ │
│ │ Japanese TTS│ │ │RealSense D435i│ │ │   LED控制器        │ │
│ └─────────────┘ │ └─────────────┘ │ └─────────────────────┘ │
│ ┌─────────────┐ │ ┌─────────────┐ │                         │
│ │Porcupine唤醒│ │ │ IMU+Force   │ │                         │
│ └─────────────┘ │ └─────────────┘ │                         │
└─────────────────┴─────────────────┴─────────────────────────┘
                         │
                    ROS2 Foxy + CycloneDDS
```## 📁 **项目结构**

```
claudia/
├── src/claudia/                     # 核心Python包 (消除重复结构)
│   ├── ai_components/               # AI组件 (LLM, ASR, TTS, Wake Word)
│   ├── robot_controller/            # 机器人控制 (动作, LED)
│   ├── sensors/                     # 传感器处理 (LiDAR, Camera, IMU)
│   ├── vision/                      # 视觉处理 (YOLO, 深度, 双摄)
│   ├── navigation/                  # 导航系统 (SLAM, 路径规划)
│   ├── audio/                       # 音频处理 (语音交互)
│   ├── common/                      # 通用工具 (配置, 日志)
│   └── main.py                      # 系统入口点
├── cyclonedds_ws/                   # Unitree ROS2工作空间 (官方标准)
│   ├── src/                         # ROS2包源码
│   │   └── unitree/                 # Unitree消息定义包
│   ├── build/                       # 构建输出
│   ├── install/                     # 安装文件
│   └── log/                         # 构建日志
├── config/                          # 统一配置管理
│   ├── default.yaml                 # 默认配置
│   ├── models/                      # AI模型配置
│   ├── sensors/                     # 传感器校准
│   ├── network/                     # 网络配置
│   └── ros2/                        # ROS2专用配置
├── scripts/                         # 脚本工具
│   ├── setup/                       # 安装配置脚本
│   └── deployment/                  # 部署脚本
├── tools/                           # 开发工具集中管理
│   ├── .cursor/                     # Cursor IDE配置
│   ├── .taskmaster/                 # TaskMaster项目管理
│   └── .serena/                     # Serena MCP工具
├── test/                            # 测试套件 (重新组织)
│   ├── unit/                        # 单元测试
│   ├── integration/                 # 集成测试
│   ├── hardware/                    # 硬件测试 (包含Unitree连接测试)
│   ├── utils/                       # 测试工具和辅助函数
│   ├── run_tests.py                 # 统一测试运行器
│   ├── conftest.py                  # pytest配置
│   └── README.md                    # 测试文档
├── data/                            # 数据存储
│   ├── maps/                        # SLAM地图
│   ├── recordings/                  # 音频录制
│   └── calibration/                 # 校准数据
├── launch/                          # ROS2启动文件
├── models/                          # AI模型文件
├── assets/                          # 资源文件
│   ├── models/                      # 预训练模型
│   ├── audio/                       # 音频资源
│   └── wake_words/                  # 唤醒词模型
└── docs/                            # 项目文档 (统一小写命名)
```## 🚀 **开发路线图**

### **第一阶段：基础环境与AI集成** 🔄 进行中
- [x] 项目结构创建和配置管理
- [x] ROS2 Foxy环境优化 (Task 1) ✅
- [x] 基础系统配置 (Task 2) ✅
- [x] **Unitree SDK安装配置 (Task 3)** ✅ **完成!**
- [ ] 传感器系统验证 (Task 4) 🔄 **当前任务**
- [ ] 音频输入输出验证 (Task 5)
- [ ] 网络配置优化 (Task 6)
- [ ] AI组件部署 (Task 7-10)
- [ ] 端到端语音交互 (Task 12)

### **第二阶段：视觉与导航系统**
- [ ] YOLOv8s目标检测 (Task 13)
- [ ] 多模态传感器融合 (Task 15)
- [ ] 无监督SLAM建图 (Task 16)
- [ ] 语义导航系统 (Task 17)

### **第三阶段：系统优化与部署**
- [ ] 性能优化和稳定性测试 (Task 18)
- [ ] 用户界面开发
- [ ] 部署和维护工具

## 🏃‍♂️ **快速开始**

### **环境要求**
- Ubuntu 20.04.5 LTS (aarch64)
- ROS2 Foxy
- Python 3.8+
- NVIDIA Jetson Orin NX
- Unitree Go2 R&D Plus

### **安装步骤**

1. **克隆项目**
   ```bash
   git clone https://github.com/your-org/claudia.git
   cd claudia
   ```

2. **环境配置** ⭐
   ```bash
   # 关键步骤：正确设置DDS环境
   source scripts/setup/setup_environment.sh
   ```

3. **验证安装**
   ```bash
   # 运行基础连接测试
   python3 test/hardware/test_unitree_connection.py
   
   # 运行通信性能测试
   python3 test/hardware/test_communication_performance.py
   ```

4. **查看完整文档**
   ```bash
   # 查看详细配置指南
   cat docs/guides/environment_setup.md
   
   # 查看任务进度
   cat docs/tasks/README.md
   ```

### **重要提醒** ⚠️

每次运行Unitree相关测试前，必须执行：
```bash
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

或使用自动化脚本：
```bash
source scripts/setup/setup_environment.sh
```

## 📖 **完整文档**

### **核心文档**
- [📚 文档主页](docs/README.md) - 完整文档导航
- [🛠️ 环境配置指南](docs/guides/environment_setup.md) - 详细安装配置步骤
- [📋 任务进度](docs/tasks/README.md) - 所有任务状态和说明
- [🧪 测试框架](test/README.md) - 测试运行指南

### **当前状态** (2025-06-26)
- ✅ **任务3完成**: Unitree SDK2集成和通信测试
- 🔄 **下一个任务**: 任务4 - 硬件传感器验证
- 📊 **进度**: 3/18 任务完成 (17%)

### **快速链接**
- [任务3完成报告](docs/tasks/task-3-completed.md) - 详细技术成果
- [通信性能基准](docs/tasks/task-3-completed.md#性能测试结果) - 延迟测试结果
- [环境设置脚本](scripts/setup/setup_environment.sh) - 一键环境配置