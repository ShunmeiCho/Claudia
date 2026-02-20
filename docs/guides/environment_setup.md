# Claudia机器人环境配置指南

## 🎯 概述

本指南详细说明了Claudia机器人项目的完整环境配置过程，特别是关键的DDS通信环境设置。

## 📋 系统要求

### 硬件要求
- **计算平台**: NVIDIA Jetson Orin NX
- **机器人**: Unitree Go2 R&D Plus
- **内存**: 至少8GB RAM
- **存储**: 至少64GB可用空间

### 软件要求
- **操作系统**: Ubuntu 20.04.5 LTS (aarch64)
- **ROS版本**: ROS2 Foxy
- **Python版本**: Python 3.8+
- **DDS实现**: CycloneDDS

## 🛠️ 安装步骤

### 1. 基础环境准备

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础依赖
sudo apt install -y \
    curl \
    wget \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-dev
```

### 2. ROS2 Foxy安装

```bash
# 设置ROS2源
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2 Foxy
sudo apt update
sudo apt install -y ros-foxy-desktop python3-argcomplete
```

### 3. CycloneDDS工作空间配置 ⭐

这是**最关键**的配置步骤，确保正确的DDS通信环境：

```bash
# 进入项目根目录
cd ~/claudia

# 创建CycloneDDS工作空间（如果不存在的话，已存在则跳过）
mkdir -p cyclonedds_ws/src
cd cyclonedds_ws

# 注意：实际项目中CycloneDDS已经通过其他方式安装
# 这里记录的是工作空间结构
```

### 4. Unitree SDK2 Python安装

```bash
# 克隆SDK
cd ~/claudia
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git

# 安装依赖
cd unitree_sdk2_python
pip3 install -e .
```

## 🚀 **关键环境配置** ⭐

### 正确的DDS环境设置

**每次运行Unitree相关测试前，必须按以下顺序执行：**

```bash
# 1. 首先source CycloneDDS工作空间
source cyclonedds_ws/install/setup.bash

# 2. 然后设置RMW实现
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3. 验证环境变量
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
```

**⚠️ 重要说明：**
- 必须按照上述顺序执行
- **不是** `rmw_cyclonedx_cpp`，而是 `rmw_cyclonedds_cpp`
- 每个新终端都需要重新设置
- 设置错误会导致DDS库加载失败

### 自动化环境设置脚本

创建便捷的环境设置脚本：

```bash
# 创建环境设置脚本
cat > scripts/setup/setup_environment.sh << 'EOF'
#!/bin/bash
# Claudia机器人环境设置脚本
# Generated: 2025-06-26 18:40:00

echo "🔧 设置Claudia机器人环境..."

# 检查项目根目录
if [ ! -f "pyproject.toml" ]; then
    echo "❌ 请在项目根目录运行此脚本"
    exit 1
fi

# 设置ROS2环境
source /opt/ros/foxy/setup.bash
echo "✅ ROS2 Foxy环境已加载"

# 设置CycloneDDS工作空间
if [ -f "cyclonedds_ws/install/setup.bash" ]; then
    source cyclonedds_ws/install/setup.bash
    echo "✅ CycloneDDS工作空间已加载"
else
    echo "⚠️ CycloneDDS工作空间未找到，请先构建"
fi

# 设置RMW实现
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "✅ RMW_IMPLEMENTATION设置为: $RMW_IMPLEMENTATION"

# 设置Python路径
export PYTHONPATH=$PYTHONPATH:~/claudia/unitree_sdk2_python
echo "✅ Python路径已设置"

echo "🎉 环境设置完成！可以运行Unitree测试了"
EOF

chmod +x scripts/setup/setup_environment.sh
```

### 使用环境设置脚本

```bash
# 在项目根目录运行
source scripts/setup/setup_environment.sh
```

## 🧪 环境验证

### 验证DDS通信

```bash
# 设置环境
source scripts/setup/setup_environment.sh

# 运行基础连接测试
python3 test/hardware/test_unitree_connection.py

# 运行通信性能测试
python3 test/hardware/test_communication_performance.py
```

### 预期输出示例

正确配置后应看到：
```
✅ 成功导入所有必需的模块
✅ 环境变量已设置: rmw_cyclonedds_cpp
🤖 正在初始化Sport客户端...
✅ Sport客户端初始化成功
📊 通信性能测试开始...
```

## ❌ 常见问题

### 问题1: DDS库加载失败
```
OSError: libddsc.so.0: cannot open shared object file
```

**解决方案:**
```bash
# 确保按正确顺序设置环境
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 问题2: 环境变量名称错误
```
设置了 rmw_cyclonedx_cpp 但仍然失败
```

**解决方案:**
```bash
# 使用正确的环境变量名称
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 注意是 cyclonedds 不是 cyclonedx
```

### 问题3: 工作空间未构建
```
cyclonedds_ws/install/setup.bash: No such file or directory
```

**解决方案:**
```bash
cd cyclonedds_ws
colcon build --symlink-install
```

## 📊 性能基准

### 通信延迟基准 (任务3.7结果)

- **轻量级命令 (Sit)**: 平均36.64ms, 97%<50ms ✅
- **复杂动作命令 (StandUp)**: 平均640.87ms
- **中等复杂命令 (Damp)**: 平均214.72ms

## 🔄 环境重置

如果环境出现问题，可以重置：

```bash
# 清理构建文件
rm -rf cyclonedds_ws/build cyclonedds_ws/install cyclonedds_ws/log

# 重新构建
cd cyclonedds_ws
colcon build --symlink-install

# 重新设置环境
source scripts/setup/setup_environment.sh
```

## 📝 配置检查清单

使用此检查清单验证环境配置：

- [ ] Ubuntu 20.04 已安装
- [ ] ROS2 Foxy 已安装
- [ ] CycloneDDS工作空间已构建
- [ ] Unitree SDK2 Python已安装
- [ ] 环境设置脚本可以运行
- [ ] 基础连接测试通过
- [ ] 通信性能测试通过

---

**文档更新时间**: 2025-06-26 18:40:00  
**适用版本**: Claudia v0.1.0  
**测试平台**: NVIDIA Jetson Orin NX + Unitree Go2 