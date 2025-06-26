#!/bin/bash

#1.1
# 2025-06-26
# Environment Verification Script for Claudia Robot System
# 基于TaskMaster研究的Jetson Orin NX环境验证最佳实践
# 验证Ubuntu 20.04.5 LTS + Python 3.8.10 + CUDA 11.4环境

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✅ PASS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[⚠️  WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[❌ FAIL]${NC} $1"
}

# 检查结果计数
TOTAL_CHECKS=0
PASSED_CHECKS=0
FAILED_CHECKS=0

run_check() {
    local check_name="$1"
    local check_function="$2"
    
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    log_info "检查: $check_name"
    
    if $check_function; then
        log_success "$check_name"
        PASSED_CHECKS=$((PASSED_CHECKS + 1))
        return 0
    else
        log_error "$check_name"
        FAILED_CHECKS=$((FAILED_CHECKS + 1))
        return 1
    fi
}

# 系统基础信息检查
check_system_info() {
    echo "=== 系统基础信息 ==="
    uname -a
    if [ -f /etc/nv_tegra_release ]; then
        echo "Tegra版本:"
        cat /etc/nv_tegra_release
    fi
    echo
    return 0
}

# Ubuntu版本检查
check_ubuntu_version() {
    local expected_version="20.04"
    local actual_version=$(lsb_release -rs 2>/dev/null || echo "unknown")
    
    if [[ "$actual_version" == "$expected_version"* ]]; then
        echo "Ubuntu版本: $actual_version ✓"
        return 0
    else
        echo "Ubuntu版本不匹配: 期望=$expected_version, 实际=$actual_version"
        return 1
    fi
}

# Python版本检查
check_python_version() {
    local expected_version="3.8.10"
    local actual_version=$(python3 --version 2>/dev/null | cut -d' ' -f2 || echo "unknown")
    
    if [[ "$actual_version" == "$expected_version"* ]]; then
        echo "Python版本: $actual_version ✓"
        return 0
    else
        echo "Python版本不匹配: 期望=$expected_version, 实际=$actual_version"
        return 1
    fi
}

# CUDA工具链检查
check_cuda_toolkit() {
    local expected_version="11.4"
    
    if command -v nvcc >/dev/null 2>&1; then
        local actual_version=$(nvcc --version | grep "release" | sed -n 's/.*release \([0-9]\+\.[0-9]\+\).*/\1/p')
        if [[ "$actual_version" == "$expected_version"* ]]; then
            echo "CUDA版本: $actual_version ✓"
            return 0
        else
            echo "CUDA版本不匹配: 期望=$expected_version, 实际=$actual_version"
            return 1
        fi
    else
        echo "nvcc命令未找到"
        return 1
    fi
}

# GPU检查
check_gpu_info() {
    if command -v nvidia-smi >/dev/null 2>&1; then
        nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader,nounits
        return 0
    else
        # Jetson上可能没有nvidia-smi，尝试其他方法
        if [ -f /proc/driver/nvidia/version ]; then
            echo "NVIDIA驱动信息:"
            cat /proc/driver/nvidia/version
            return 0
        else
            echo "无法获取GPU信息（这在Jetson上是正常的）"
            return 0
        fi
    fi
}

# CUDA示例运行测试
check_cuda_samples() {
    local samples_dir="/usr/local/cuda/samples/1_Utilities/deviceQuery"
    
    if [ -d "$samples_dir" ]; then
        log_info "编译并运行CUDA deviceQuery示例..."
        cd "$samples_dir"
        
        if [ ! -f "./deviceQuery" ]; then
            if sudo make >/dev/null 2>&1; then
                echo "CUDA示例编译成功 ✓"
            else
                echo "CUDA示例编译失败"
                return 1
            fi
        fi
        
        if ./deviceQuery | grep -q "Result = PASS"; then
            echo "CUDA设备查询成功 ✓"
            return 0
        else
            echo "CUDA设备查询失败"
            return 1
        fi
    else
        echo "CUDA示例目录未找到: $samples_dir"
        return 1
    fi
}

# 内存检查
check_memory() {
    local total_mem=$(free -h | awk '/^Mem:/{print $2}')
    local available_mem=$(free -h | awk '/^Mem:/{print $7}')
    local swap_mem=$(free -h | awk '/^Swap:/{print $2}')
    
    echo "总内存: $total_mem"
    echo "可用内存: $available_mem"
    echo "交换空间: $swap_mem"
    
    # 检查是否有足够内存（至少8GB）
    local total_mem_gb=$(free -g | awk '/^Mem:/{print $2}')
    if [ "$total_mem_gb" -ge 8 ]; then
        echo "内存容量充足 ✓"
        return 0
    else
        echo "内存可能不足，建议至少8GB"
        return 1
    fi
}

# ROS2环境检查
check_ros2_environment() {
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        echo "ROS2 Foxy安装检查 ✓"
        
        # 临时源ROS2环境
        source /opt/ros/foxy/setup.bash >/dev/null 2>&1
        
        if command -v ros2 >/dev/null 2>&1; then
            echo "ROS2命令可用 ✓"
            return 0
        else
            echo "ROS2命令不可用"
            return 1
        fi
    else
        echo "ROS2 Foxy未安装"
        return 1
    fi
}

# 网络配置检查
check_network_config() {
    local eth_interface=$(ip route | grep default | awk '{print $5}' | head -1)
    local ip_address=$(ip addr show "$eth_interface" 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)
    
    if [ -n "$ip_address" ]; then
        echo "网络接口: $eth_interface"
        echo "IP地址: $ip_address"
        
        # 检查是否在预期的网段
        if [[ "$ip_address" =~ ^192\.168\. ]]; then
            echo "网络配置正常 ✓"
            return 0
        else
            echo "网络配置可能需要调整"
            return 1
        fi
    else
        echo "无法获取网络信息"
        return 1
    fi
}

# 关键目录检查
check_project_structure() {
    local project_dirs=("src/claudia" "config" "scripts" "cyclonedds_ws")
    
    for dir in "${project_dirs[@]}"; do
        if [ -d "$dir" ]; then
            echo "目录存在: $dir ✓"
        else
            echo "目录缺失: $dir"
            return 1
        fi
    done
    
    return 0
}

# 主执行函数
main() {
    echo "🚀 Claudia机器人系统环境验证脚本"
    echo "==============================================="
    echo "验证目标: Ubuntu 20.04.5 LTS + Python 3.8.10 + CUDA 11.4"
    echo
    
    # 执行所有检查
    run_check "系统基础信息" check_system_info
    run_check "Ubuntu版本检查" check_ubuntu_version
    run_check "Python版本检查" check_python_version
    run_check "CUDA工具链检查" check_cuda_toolkit
    run_check "GPU信息检查" check_gpu_info
    run_check "内存配置检查" check_memory
    run_check "ROS2环境检查" check_ros2_environment
    run_check "网络配置检查" check_network_config
    run_check "项目结构检查" check_project_structure
    
    # 可选：CUDA示例测试（耗时较长）
    if [ "${1:-}" != "--quick" ]; then
        run_check "CUDA示例测试" check_cuda_samples
    fi
    
    echo
    echo "==============================================="
    echo "📊 验证结果汇总:"
    echo "总检查项: $TOTAL_CHECKS"
    echo "通过: $PASSED_CHECKS"
    echo "失败: $FAILED_CHECKS"
    
    if [ "$FAILED_CHECKS" -eq 0 ]; then
        log_success "✅ 所有环境检查通过！系统已准备就绪"
        return 0
    else
        log_error "❌ 发现 $FAILED_CHECKS 项环境问题，请检查上述输出"
        return 1
    fi
}

# 脚本帮助
if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  --quick    快速检查（跳过CUDA示例测试）"
    echo "  --help     显示帮助信息"
    exit 0
fi

# 执行主函数
main "$@" 