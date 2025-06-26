# 第三方依赖获取指南

## 🎯 概述

为了保持GitHub仓库的轻量化，Claudia项目的大型第三方依赖不包含在版本控制中。本指南说明如何获取和设置这些必需的依赖。

## 📋 需要手动获取的依赖

### 1. CycloneDDS
**大小**: ~101MB  
**用途**: DDS通信中间件

```bash
# 克隆CycloneDDS
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

### 2. Unitree SDK2 Python
**大小**: ~2.3MB  
**用途**: Unitree机器人Python SDK

```bash
# 克隆Unitree SDK2 Python
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```

### 3. CycloneDDS工作空间
**大小**: ~413MB  
**用途**: ROS2消息定义和编译产物

```bash
# 创建cyclonedds_ws工作空间
mkdir -p cyclonedds_ws/src
cd cyclonedds_ws/src

# 克隆Unitree ROS2包
git clone https://github.com/unitreerobotics/unitree_ros2.git
git clone https://github.com/unitreerobotics/unitree_sdk2.git

# 编译工作空间
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## 🚀 自动化设置脚本

项目提供了自动化设置脚本来简化依赖获取过程：

```bash
# 使用项目提供的环境设置脚本
source scripts/setup/setup_environment.sh

# 或分别运行各个安装脚本
bash scripts/setup/install_cyclonedds_deps.sh
bash scripts/setup/install_unitree_sdks.sh
bash scripts/setup/setup_cyclonedds_workspace.sh
```

## ⚠️ 重要说明

1. **存储位置**: 所有依赖应放置在项目根目录下
2. **环境变量**: 运行脚本会自动设置必要的环境变量
3. **版本兼容性**: 确保使用指定的版本分支以保证兼容性
4. **磁盘空间**: 总计需要约500MB的磁盘空间

## 🔧 验证安装

安装完成后，可以使用以下命令验证：

```bash
# 验证环境配置
source scripts/setup/setup_environment.sh

# 运行连接测试
python3 test/hardware/test_unitree_connection.py

# 运行通信性能测试
python3 test/hardware/test_communication_performance.py
```

## 📞 故障排除

如果遇到问题，请参考：
- [故障排除指南](../troubleshooting/README.md)
- [环境配置指南](environment_setup.md)
- [任务3完成报告](../tasks/task-3-completed.md) - 包含详细的安装验证过程 