#!/bin/bash
# Generated: 2025-01-27 11:55:00
# Purpose: Install dependencies for IMU validation system
# Platform: Jetson Xavier NX Ubuntu 18.04

set -e

echo "🔧 IMU验证系统依赖安装脚本"
echo "=========================================="

# 检查运行环境
echo "⏰ 当前时间: $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo "📁 当前目录: $(pwd)"
echo "💾 磁盘使用: $(df . | tail -1 | awk '{print $5}')"
echo "🧠 内存使用: $(free | grep Mem | awk '{printf "%.0f%%", $3/$2 * 100.0}')"

# 错误处理函数
handle_error() {
    local exit_code=$?
    local line_number=$1
    echo "❌ 错误发生在第 $line_number 行"
    echo "退出码: $exit_code"
    echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
    exit $exit_code
}

trap 'handle_error $LINENO' ERR

echo ""
echo "📦 检查和安装Python依赖..."

# 确保pip可用
if ! command -v pip3 &> /dev/null; then
    echo "安装pip3..."
    sudo apt update
    sudo apt install -y python3-pip
fi

# 检查并安装必要的Python包
REQUIRED_PACKAGES=(
    "numpy>=1.19.0"
    "matplotlib>=3.3.0"
    "scipy>=1.5.0"
    "pyserial>=3.4"
    "pyyaml>=5.3.0"
    "jsonschema>=3.2.0"
)

echo "检查Python包依赖..."
for package in "${REQUIRED_PACKAGES[@]}"; do
    package_name=$(echo $package | cut -d'>' -f1 | cut -d'=' -f1)
    echo -n "  检查 $package_name ... "
    
    if python3 -c "import $package_name" 2>/dev/null; then
        echo "✅ 已安装"
    else
        echo "❌ 未安装，正在安装..."
        pip3 install "$package" --user
        echo "✅ 安装完成"
    fi
done

echo ""
echo "🤖 检查Unitree SDK2 Python依赖..."

# 检查unitree_sdk2py
echo -n "  检查 unitree_sdk2py ... "
if python3 -c "import unitree_sdk2py" 2>/dev/null; then
    echo "✅ 已安装"
else
    echo "❌ 未安装"
    echo "    请参考以下步骤手动安装 unitree_sdk2py:"
    echo "    1. git clone https://github.com/unitreerobotics/unitree_sdk2_python.git"
    echo "    2. cd unitree_sdk2_python"
    echo "    3. pip3 install -e ."
fi

echo ""
echo "🔍 检查系统依赖..."

# 检查必要的系统工具
SYSTEM_TOOLS=("git" "wget" "curl")
for tool in "${SYSTEM_TOOLS[@]}"; do
    echo -n "  检查 $tool ... "
    if command -v "$tool" &> /dev/null; then
        echo "✅ 可用"
    else
        echo "❌ 未找到，正在安装..."
        sudo apt install -y "$tool"
        echo "✅ 安装完成"
    fi
done

echo ""
echo "📊 验证安装结果..."

# 创建验证脚本
cat > /tmp/verify_imu_deps.py << 'EOF'
#!/usr/bin/env python3
import sys

def check_import(module_name, package_name=None):
    try:
        __import__(module_name)
        print(f"✅ {package_name or module_name}")
        return True
    except ImportError as e:
        print(f"❌ {package_name or module_name}: {e}")
        return False

print("验证Python依赖:")
all_ok = True
all_ok &= check_import("numpy", "NumPy")
all_ok &= check_import("matplotlib", "Matplotlib")
all_ok &= check_import("scipy", "SciPy")
all_ok &= check_import("serial", "PySerial")
all_ok &= check_import("yaml", "PyYAML")
all_ok &= check_import("jsonschema", "JsonSchema")

print("\n验证Unitree SDK:")
all_ok &= check_import("unitree_sdk2py", "Unitree SDK2 Python")

if all_ok:
    print("\n🎉 所有依赖安装成功!")
    sys.exit(0)
else:
    print("\n⚠️ 部分依赖缺失，请检查安装")
    sys.exit(1)
EOF

python3 /tmp/verify_imu_deps.py
VERIFICATION_RESULT=$?

# 清理临时文件
rm -f /tmp/verify_imu_deps.py

echo ""
echo "📋 安装总结："
echo "  - Python依赖: 已检查和安装"
echo "  - 系统工具: 已检查和安装"
echo "  - Unitree SDK: 请手动确认安装"

if [ $VERIFICATION_RESULT -eq 0 ]; then
    echo ""
    echo "✅ IMU验证系统依赖安装完成!"
    echo "现在可以运行: ./run_imu_validation.sh"
else
    echo ""
    echo "⚠️ 部分依赖可能需要手动处理"
    echo "请检查上述错误信息并手动安装缺失的依赖"
fi

echo ""
echo "🕐 安装完成时间: $(date '+%Y-%m-%d %H:%M:%S %Z')" 