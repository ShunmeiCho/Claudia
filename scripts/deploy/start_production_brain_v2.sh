#!/bin/bash
# Claudia Production Brain 启动脚本 v2.0（Track A优化版）
# 修复：显式source ROS2、Ollama预检、预热、DDS验证

set -e

echo "=================================="
echo "🤖 Claudia Production Brain v2.0"
echo "=================================="
echo ""

# 工作目录
cd $HOME/claudia

# ========================
# 1. ROS2 Foxy 环境
# ========================
echo "🔧 加载ROS2 Foxy环境..."
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "   ✅ ROS2 Foxy已加载"
else
    echo "   ⚠️  ROS2 Foxy未找到（将使用模拟模式）"
fi

# 项目ROS2配置
if [ -f ".env.ros2" ]; then
    source .env.ros2
    echo "   ✅ 项目ROS2配置已加载"
fi

# ========================
# 2. CycloneDDS配置（修复DDS符号问题）
# ========================
echo "🔧 配置CycloneDDS（rmw_cyclonedds_cpp + eth0）..."
# 使用独立编译的CycloneDDS 0.10.x（解决符号不匹配问题）
export CYCLONEDDS_HOME=$HOME/cyclonedds/install
export LD_LIBRARY_PATH=$HOME/cyclonedds/install/lib:$LD_LIBRARY_PATH

# ROS2配置（仍使用ROS2 Foxy的CycloneDDS RMW）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# 内联DDS配置（eth0固定）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

echo "   ✅ DDS配置完成（eth0, Domain 0）"

# ========================
# 3. 网络验证
# ========================
echo "🔧 网络验证..."
LOCAL_IP=$(ip addr show eth0 2>/dev/null | grep '192.168.123' | awk '{print $2}' | cut -d'/' -f1 || echo "未配置")
echo "   本机IP: $LOCAL_IP"
echo "   机器人IP: 192.168.123.161"
echo "   ROS域: $ROS_DOMAIN_ID"

if ping -c 1 -W 1 192.168.123.161 >/dev/null 2>&1; then
    echo "   ✅ 机器人网络可达"
else
    echo "   ⚠️  无法ping通机器人（可能未连接）"
fi

# ========================
# 4. Ollama服务与模型
# ========================
echo "🔧 检查Ollama服务..."
if ! curl -s http://localhost:11434/api/tags >/dev/null 2>&1; then
    echo "   ⚠️  Ollama未运行，尝试启动..."
    nohup ollama serve >/dev/null 2>&1 &
    sleep 3

    if curl -s http://localhost:11434/api/tags >/dev/null 2>&1; then
        echo "   ✅ Ollama已启动"
    else
        echo "   ❌ 无法启动Ollama"
        exit 1
    fi
else
    echo "   ✅ Ollama运行中"
fi

# 检查模型
echo "🔧 检查LLM模型..."
for model in "claudia-go2-3b:v11.2" "claudia-go2-7b:v7"; do
    if ollama list | grep -q "$model"; then
        echo "   ✅ $model"
    else
        echo "   ⚠️  $model 不存在，尝试拉取..."
        ollama pull "$model" 2>/dev/null || echo "   ⚠️  拉取失败，跳过"
    fi
done

# LLM预热（避免首次高延迟）
echo "🔧 预热LLM..."
python3 - <<'PYWARMUP' 2>/dev/null || echo "   ℹ️  跳过预热（requests库不可用）"
try:
    import requests, json
    resp = requests.post(
        "http://localhost:11434/api/generate",
        json={"model":"claudia-go2-3b:v11.2","prompt":"test","stream":False},
        timeout=10
    )
    if resp.status_code == 200:
        print("   ✅ LLM预热完成")
except:
    pass
PYWARMUP

# ========================
# 5. Python依赖
# ========================
echo "🔧 检查Python依赖..."
python3 -c "import ollama" 2>/dev/null && echo "   ✅ ollama库" || {
    echo "   ⚠️  ollama库未安装，尝试安装..."
    pip3 install ollama --quiet 2>/dev/null || echo "   ⚠️  安装失败"
}

# ========================
# 6. 启动
# ========================
echo ""
echo "=================================="
echo "请选择运行模式:"
echo "1) 模拟模式（安全测试）"
echo "2) 真实硬件模式（连接Go2）"
echo "=================================="
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
        echo "⚠️  真实硬件模式 - 请确认："
        echo "   1. 机器人已开机并连接（IP:192.168.123.161）"
        echo "   2. 周围有≥2m活动空间"
        echo "   3. Unitree App已关闭（避免占用冲突）"
        echo ""
        read -p "确认继续? [y/N]: " confirm
        if [[ $confirm =~ ^[yY]$ ]]; then
            echo ""
            echo "✅ 启动真实硬件模式..."
            echo ""
            python3 production_commander.py --hardware
        else
            echo "❌ 已取消"
            exit 0
        fi
        ;;
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac
