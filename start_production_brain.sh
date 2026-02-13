#!/bin/bash
# Claudia Production Brain 启动脚本

echo "=================================="
echo "🤖 Claudia Production Brain Launcher"
echo "=================================="
echo ""

# 设置环境
cd /home/m1ng/claudia

# 配置CycloneDDS环境 - 关键！
export CYCLONEDDS_HOME=/home/m1ng/claudia/cyclonedds/install
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 配置网络连接 - 使用官方推荐的内联配置方式
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

echo "🔧 网络配置:"
echo "   本机IP: $(ip addr show eth0 | grep '192.168.123' | awk '{print $2}' | cut -d'/' -f1)"
echo "   机器人IP: 192.168.123.161 (Go2)"
echo "   DDS配置: 使用eth0网络接口"
echo ""

# 注意: 不再 source setup_cyclonedds.sh（它使用 $HOME/cyclonedds/install，
# 可能覆盖上面设置的 CYCLONEDDS_HOME，造成路径不一致）
# setup_cyclonedds.sh 仅用于首次安装 CycloneDDS，不在运行时加载

# PR2: 检查并创建 Action 模型（仅在非 legacy 模式下需要）
ROUTER_MODE="${BRAIN_ROUTER_MODE:-legacy}"
if [ "$ROUTER_MODE" != "legacy" ]; then
    ACTION_MODEL="${BRAIN_MODEL_ACTION:-claudia-action-v1}"
    if ! ollama list 2>/dev/null | grep -q "$ACTION_MODEL"; then
        echo "📦 创建 Action 模型: $ACTION_MODEL ..."
        ollama create "$ACTION_MODEL" -f models/ClaudiaAction_v1.0
        echo "✅ Action 模型已创建"
    else
        echo "✅ Action 模型已存在: $ACTION_MODEL"
    fi
    echo "   路由模式: $ROUTER_MODE"
    echo ""
fi

# 询问模式
echo "请选择运行模式:"
echo "1) 模拟模式 (安全测试)"
echo "2) 真实硬件模式 (连接机器人)"
echo ""
read -p "选择 [1/2]: " choice

case $choice in
    1)
        echo ""
        echo "✅ 启动模拟模式..."
        echo ""
        python3 production_commander.py
        ;;
    2)
        echo ""
        echo "⚠️  真实硬件模式 - 请确保机器人已连接"
        read -p "确认继续? [y/N]: " confirm
        if [[ $confirm == [yY] ]]; then
            echo ""
            echo "✅ 启动真实硬件模式..."
            echo ""
            python3 production_commander.py --hardware
        else
            echo "已取消"
        fi
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
