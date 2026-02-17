#!/bin/bash
# Claudia Production Brain 启动脚本

echo "=================================="
echo "🤖 Claudia Production Brain Launcher"
echo "=================================="
echo ""

# 设置环境（从脚本位置推导项目根目录，避免硬编码）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 统一 Python 解释器（避免 conda/base 环境缺依赖导致误回退到 Mock）
# 可通过 CLAUDIA_PYTHON_BIN 覆盖
PYTHON_BIN="${CLAUDIA_PYTHON_BIN:-/usr/bin/python3}"
if [ ! -x "$PYTHON_BIN" ]; then
    echo "❌ Python解释器不可用: $PYTHON_BIN"
    exit 1
fi

# 注入 Unitree SDK2 Python 源码路径（兼容两个常见安装位置）
for SDK_PATH in "$SCRIPT_DIR/unitree_sdk2_python" "$HOME/unitree_sdk2_python"; do
    if [ -d "$SDK_PATH" ] && [[ ":${PYTHONPATH:-}:" != *":$SDK_PATH:"* ]]; then
        export PYTHONPATH="$SDK_PATH${PYTHONPATH:+:$PYTHONPATH}"
    fi
done

# 配置CycloneDDS环境 - 关键！
export CYCLONEDDS_HOME="$SCRIPT_DIR/cyclonedds/install"
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
echo "   Python: $PYTHON_BIN ($($PYTHON_BIN -V 2>&1))"
echo ""

# 注意: 不再 source setup_cyclonedds.sh（它使用 $HOME/cyclonedds/install，
# 可能覆盖上面设置的 CYCLONEDDS_HOME，造成路径不一致）
# setup_cyclonedds.sh 仅用于首次安装 CycloneDDS，不在运行时加载

# PR2: 检查并创建 Action 模型（仅在非 legacy 模式下需要）
ROUTER_MODE="${BRAIN_ROUTER_MODE:-legacy}"
if [ "$ROUTER_MODE" != "legacy" ]; then
    ACTION_MODEL="${BRAIN_MODEL_ACTION:-claudia-action-v3}"
    if ! ollama list 2>/dev/null | grep -q "$ACTION_MODEL"; then
        echo "📦 创建 Action 模型: $ACTION_MODEL ..."
        ollama create "$ACTION_MODEL" -f models/ClaudiaAction_v3.0
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
        # 软检查：ollama Python 包缺失时，Commander 会使用 HTTP API 兜底预热
        if ! "$PYTHON_BIN" -c "import ollama" >/dev/null 2>&1; then
            echo "⚠️ 未检测到Python ollama包，预热将走HTTP兜底"
        fi
        "$PYTHON_BIN" production_commander.py
        ;;
    2)
        echo ""
        echo "⚠️  真实硬件模式 - 请确保机器人已连接"
        read -p "确认继续? [y/N]: " confirm
        if [[ $confirm == [yY] ]]; then
            # 硬件模式强依赖：缺失时直接失败，避免“名义硬件实际Mock”的误导状态
            if ! "$PYTHON_BIN" -c "import unitree_sdk2py, cyclonedds.idl" >/dev/null 2>&1; then
                echo ""
                echo "❌ 硬件依赖导入失败（unitree_sdk2py / cyclonedds.idl）"
                echo "   当前Python: $PYTHON_BIN"
                echo "   PYTHONPATH: ${PYTHONPATH:-<empty>}"
                echo ""
                echo "建议修复步骤:"
                echo "1) export PYTHONPATH=$SCRIPT_DIR/unitree_sdk2_python:\$PYTHONPATH"
                echo "2) 检查CycloneDDS库路径: $CYCLONEDDS_HOME/lib"
                echo "3) 重试导入: $PYTHON_BIN -c 'import unitree_sdk2py, cyclonedds.idl; print(\"OK\")'"
                echo ""
                echo "已停止启动，避免错误回退到 MockSportClient。"
                exit 1
            fi

            # 唤醒动画（起立→伸懒腰）— 默认关闭，需显式启用
            if [ -z "$COMMANDER_WAKEUP_ANIMATION" ]; then
                read -p "启用唤醒动画（起立→伸懒腰）? [y/N]: " wakeup
                if [[ $wakeup == [yY] ]]; then
                    export COMMANDER_WAKEUP_ANIMATION=1
                fi
            fi
            echo ""
            echo "✅ 启动真实硬件模式..."
            echo ""
            if ! "$PYTHON_BIN" -c "import ollama" >/dev/null 2>&1; then
                echo "⚠️ 未检测到Python ollama包，预热将走HTTP兜底"
            fi
            "$PYTHON_BIN" production_commander.py --hardware
        else
            echo "已取消"
        fi
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
