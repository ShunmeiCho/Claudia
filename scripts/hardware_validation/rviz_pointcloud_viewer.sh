#!/bin/bash
# RViz2点云可视化脚本
# 用途: 在SSH环境下查看Unitree Go2 LiDAR点云数据
# 生成时间: $(date '+%Y-%m-%d %H:%M:%S')

echo "🤖 Unitree Go2 LiDAR点云可视化器"
echo "========================================"

# 设置环境
source /opt/ros/foxy/setup.bash
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "✅ ROS2环境已加载"

# 检查X11转发
if [ -z "$DISPLAY" ]; then
    echo "⚠️ 警告: 未检测到X11转发"
    echo "💡 建议: 使用 'ssh -X' 重新连接以启用图形界面"
    echo ""
    echo "🔄 如果无法使用X11，请选择其他方案:"
    echo "1. 生成静态点云图像 (方案3)"
    echo "2. 保存点云文件 (方案4)"
    echo "3. 使用Web可视化 (方案5)"
    echo ""
    read -p "是否继续尝试启动RViz2? (y/n): " choice
    if [ "$choice" != "y" ]; then
        echo "❌ 用户取消操作"
        exit 1
    fi
fi

echo "🚀 启动RViz2..."
echo "📋 配置说明:"
echo "  1. 添加PointCloud2显示类型"
echo "  2. Topic设置为: /utlidar/cloud"  
echo "  3. Fixed Frame设置为: utlidar_lidar"
echo "  4. 可调整点云大小和颜色"
echo ""

# 启动RViz2
ros2 run rviz2 rviz2

echo "✅ RViz2已退出" 