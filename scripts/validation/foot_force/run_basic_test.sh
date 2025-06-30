#!/bin/bash
# scripts/validation/foot_force/run_basic_test.sh
# Generated: 2025-06-27 14:10:00 CST
# Purpose: 运行足端力传感器基础测试

set -e

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../" && pwd)"
VALIDATION_DIR="$SCRIPT_DIR/foot_force_validation"

echo "🔧 足端力传感器基础测试启动脚本"
echo "⏰ 启动时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "📁 项目根目录: $PROJECT_ROOT"
echo "📁 验证目录: $VALIDATION_DIR"

# 检查当前目录
cd "$PROJECT_ROOT"
echo "📁 当前工作目录: $(pwd)"

# 环境检查
echo ""
echo "🔍 环境检查..."

# 检查Python环境
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3未找到"
    exit 1
fi
echo "✅ Python3: $(python3 --version)"

# 检查必要的Python包
echo "🔍 检查Python依赖..."
python3 -c "
import sys
required_packages = ['numpy', 'json', 'threading', 'pathlib']
missing_packages = []

for package in required_packages:
    try:
        __import__(package)
        print(f'✅ {package}')
    except ImportError:
        missing_packages.append(package)
        print(f'❌ {package}')

if missing_packages:
    print(f'缺少包: {missing_packages}')
    sys.exit(1)
else:
    print('✅ 所有基础依赖都已安装')
"

# 检查Unitree SDK
echo "🔍 检查Unitree SDK..."
python3 -c "
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    print('✅ Unitree SDK2 导入成功')
except ImportError as e:
    print(f'❌ Unitree SDK2 导入失败: {e}')
    print('请确保已正确安装和配置 unitree_sdk2py')
    exit(1)
"

# 检查验证目录和文件
echo "🔍 检查验证文件..."
required_files=(
    "$VALIDATION_DIR/foot_force_config.py"
    "$VALIDATION_DIR/data_collector.py"
    "$VALIDATION_DIR/validation_config.json"
    "$VALIDATION_DIR/basic_test.py"
)

for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ $(basename "$file")"
    else
        echo "❌ 文件不存在: $file"
        exit 1
    fi
done

# 创建必要的目录
echo ""
echo "📁 创建输出目录..."
mkdir -p "$VALIDATION_DIR/logs"
mkdir -p "$VALIDATION_DIR/output"
echo "✅ 输出目录已创建"

# 设置环境变量
echo ""
echo "🌍 设置环境变量..."
export PYTHONPATH="$VALIDATION_DIR:$PYTHONPATH"
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

echo "✅ PYTHONPATH: $PYTHONPATH"
echo "✅ RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# 检查网络接口
echo ""
echo "🌐 检查网络接口..."
if ip link show eth0 &> /dev/null; then
    echo "✅ eth0 网络接口存在"
    ip addr show eth0 | grep -E "inet " || echo "⚠️ eth0 未配置IP地址"
else
    echo "⚠️ eth0 网络接口不存在，将使用默认配置"
fi

# 询问测试参数
echo ""
echo "📋 测试配置..."

# 默认测试时长
DEFAULT_DURATION=10
read -p "🕐 请输入测试持续时间（秒，默认${DEFAULT_DURATION}）: " duration
duration=${duration:-$DEFAULT_DURATION}

# 是否启用调试模式
read -p "🐛 是否启用调试模式？(y/n，默认n): " debug_mode
debug_mode=${debug_mode:-n}

# 构建命令参数
cmd_args="--duration $duration"
if [[ "$debug_mode" =~ ^[Yy]$ ]]; then
    cmd_args="$cmd_args --debug"
fi

# 显示最终配置
echo ""
echo "🎯 测试配置确认:"
echo "   测试时长: ${duration}秒"
echo "   调试模式: $([ "$debug_mode" = 'y' ] && echo '启用' || echo '禁用')"
echo "   输出目录: $VALIDATION_DIR/output"
echo "   日志目录: $VALIDATION_DIR/logs"

# 确认执行
echo ""
read -p "▶️ 是否开始执行测试？(y/n): " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo "❌ 测试已取消"
    exit 0
fi

# 执行测试
echo ""
echo "🚀 开始执行足端力传感器基础测试..."
echo "=========================================="

cd "$VALIDATION_DIR"

# 运行Python测试脚本
if python3 basic_test.py $cmd_args; then
    echo ""
    echo "=========================================="
    echo "✅ 足端力传感器基础测试完成"
    echo "📊 测试结果已保存到: $VALIDATION_DIR/output/"
    echo "📋 日志文件: $VALIDATION_DIR/logs/"
    echo ""
    
    # 显示生成的文件
    if [ -d "$VALIDATION_DIR/output" ]; then
        echo "📄 生成的文件:"
        find "$VALIDATION_DIR/output" -type f -name "*$(date '+%Y%m%d')*" 2>/dev/null | head -10 | while read file; do
            echo "   - $(basename "$file")"
        done
    fi
    
    echo ""
    echo "🎉 测试成功完成！"
    
else
    echo ""
    echo "=========================================="
    echo "❌ 足端力传感器基础测试失败"
    echo "📋 请检查日志文件: $VALIDATION_DIR/logs/"
    echo ""
    echo "🔧 常见问题排查:"
    echo "   1. 检查机器人连接状态"
    echo "   2. 确认网络接口配置"
    echo "   3. 验证Unitree SDK安装"
    echo "   4. 查看详细错误日志"
    echo ""
    exit 1
fi

echo ""
echo "⏰ 测试结束时间: $(date '+%Y-%m-%d %H:%M:%S')" 