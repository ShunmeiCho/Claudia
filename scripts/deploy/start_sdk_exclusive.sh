#!/bin/bash
# SDK独占模式启动器 - 确保APP不会干扰

echo "===================================="
echo "🔒 Claudia SDK独占模式启动器"
echo "===================================="
echo ""
echo "⚠️  重要提示："
echo "SDK和APP不能同时控制机器人！"
echo "这是Unitree的安全设计，不是故障。"
echo ""
echo "📋 启动前检查清单："
echo "□ 1. Unitree Go APP已完全关闭"
echo "□ 2. 机器人已重启（清除APP连接）"
echo "□ 3. 网络连接正常(192.168.123.x)"
echo ""

read -p "确认以上条件都满足? [y/N]: " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo ""
    echo "请按以下步骤操作："
    echo "1. 关闭手机上的Unitree Go APP"
    echo "2. 按住机器人电源键重启"
    echo "3. 等待30秒后再运行此脚本"
    exit 1
fi

echo ""
echo "🔧 配置SDK环境..."

# 设置CycloneDDS环境
export CYCLONEDDS_HOME="$HOME/claudia/cyclonedds/install"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:$LD_LIBRARY_PATH"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# 使用内联XML配置（官方推荐）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'

echo "✅ 环境配置完成"
echo ""

# 检查网络
echo "📡 检查网络连接..."
if ping -c 1 192.168.123.161 > /dev/null 2>&1; then
    echo "✅ 机器人网络可达 (192.168.123.161)"
else
    echo "⚠️  无法连接机器人，请检查网络"
    echo "   提示：确保在同一网段192.168.123.x"
    exit 1
fi

echo ""
echo "🚀 启动SDK独占控制..."
echo "----------------------------------------"
echo "提示：如果返回3103错误，说明APP仍在占用"
echo "      需要重启机器人并确保APP已关闭"
echo "----------------------------------------"
echo ""

# 启动生产大脑
cd $HOME/claudia
python3 production_commander.py

echo ""
echo "🧹 会话结束"
echo ""
echo "如需切换到APP控制："
echo "1. 先停止此程序"
echo "2. 重启机器人"
echo "3. 打开APP连接"
