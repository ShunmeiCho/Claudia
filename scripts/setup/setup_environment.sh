#!/bin/bash
# Claudia机器人环境设置脚本
# Generated: 2025-06-26 18:40:00
# Purpose: 自动设置Claudia机器人项目的运行环境

set -e

echo "🔧 设置Claudia机器人环境..."

# 检查项目根目录
if [ ! -f "pyproject.toml" ]; then
    echo "❌ 请在项目根目录运行此脚本"
    echo "📍 当前目录: $(pwd)"
    echo "💡 请切换到包含pyproject.toml的目录"
    exit 1
fi

echo "📍 项目根目录: $(pwd)"

# 设置ROS2环境
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "✅ ROS2 Foxy环境已加载"
else
    echo "⚠️ ROS2 Foxy未找到，请确保已正确安装"
fi

# 设置CycloneDDS工作空间
if [ -f "cyclonedds_ws/install/setup.bash" ]; then
    source cyclonedds_ws/install/setup.bash
    echo "✅ CycloneDDS工作空间已加载"
else
    echo "⚠️ CycloneDDS工作空间未找到 (cyclonedds_ws/install/setup.bash)"
    echo "💡 如果首次运行，这是正常的，请先构建工作空间"
fi

# 设置RMW实现 - 这是关键！
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "✅ RMW_IMPLEMENTATION设置为: $RMW_IMPLEMENTATION"

# 设置Python路径
if [ -d "unitree_sdk2_python" ]; then
    export PYTHONPATH=$PYTHONPATH:$(pwd)/unitree_sdk2_python
    echo "✅ Python路径已设置: $(pwd)/unitree_sdk2_python"
else
    echo "⚠️ unitree_sdk2_python目录未找到"
fi

# 验证关键环境变量
echo ""
echo "🔍 环境变量验证:"
echo "   ROS_DISTRO: ${ROS_DISTRO:-未设置}"
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-未设置}"
echo "   PYTHONPATH: ${PYTHONPATH:-未设置}"

# 检查关键文件
echo ""
echo "📁 关键文件检查:"
[ -f "cyclonedds_ws/install/setup.bash" ] && echo "   ✅ CycloneDDS setup.bash" || echo "   ❌ CycloneDDS setup.bash"
[ -d "unitree_sdk2_python" ] && echo "   ✅ Unitree SDK2 Python" || echo "   ❌ Unitree SDK2 Python"
[ -d "test/hardware" ] && echo "   ✅ 硬件测试目录" || echo "   ❌ 硬件测试目录"

echo ""
echo "🎉 环境设置完成！"
echo ""
echo "📋 接下来可以运行："
echo "   python3 test/hardware/test_unitree_connection.py           # 基础连接测试"
echo "   python3 test/hardware/test_basic_control_commands.py       # 基础控制命令测试"
echo "   python3 test/hardware/test_communication_performance.py    # 通信性能测试"
echo "   python3 test/run_tests.py --type hardware                  # 运行所有硬件测试" 