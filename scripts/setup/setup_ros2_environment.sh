#!/bin/bash

#1.4
# ROS2环境设置脚本 - Claudia机器人系统
# Generated: 2025-06-26
# Purpose: 设置和测试ROS2与Claudia项目集成

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目路径
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CYCLONEDDS_WS="$PROJECT_ROOT/cyclonedds_ws"

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✅ SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[⚠️  WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[❌ ERROR]${NC} $1"
}

# 设置CUDA环境变量
setup_cuda_environment() {
    log_info "设置CUDA环境变量..."
    
    export PATH=/usr/local/cuda/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
    export CUDA_HOME=/usr/local/cuda
    
    if command -v nvcc >/dev/null 2>&1; then
        log_success "CUDA环境设置完成: $(nvcc --version | grep release | sed 's/.*release //' | sed 's/,.*//')"
    else
        log_error "CUDA环境设置失败"
        return 1
    fi
}

# 设置ROS2环境
setup_ros2_environment() {
    log_info "设置ROS2环境..."
    
    # 源ROS2 Foxy
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        log_success "ROS2 Foxy环境已加载"
    else
        log_error "ROS2 Foxy setup文件未找到"
        return 1
    fi
    
    # 设置ROS2环境变量
    export ROS_VERSION=2
    export ROS_DISTRO=foxy
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=0
    
    # 源工作空间
    if [ -f "$CYCLONEDDS_WS/install/setup.bash" ]; then
        source "$CYCLONEDDS_WS/install/setup.bash"
        log_success "cyclonedds_ws工作空间已加载"
    else
        log_warning "工作空间setup文件未找到，尝试构建..."
        if build_workspace; then
            source "$CYCLONEDDS_WS/install/setup.bash"
            log_success "工作空间构建并加载成功"
        else
            log_error "工作空间构建失败"
            return 1
        fi
    fi
    
    # 设置Python路径
    PYTHON_PATH="$CYCLONEDDS_WS/install/lib/python3.8/site-packages"
    if [ -d "$PYTHON_PATH" ]; then
        export PYTHONPATH="$PYTHONPATH:$PYTHON_PATH"
        log_success "Python路径已设置"
    fi
}

# 构建工作空间
build_workspace() {
    log_info "构建cyclonedds_ws工作空间..."
    
    cd "$CYCLONEDDS_WS"
    
    # 确保源目录存在
    if [ ! -d "src" ]; then
        log_error "工作空间src目录不存在"
        return 1
    fi
    
    # 清理并构建
    if colcon build --symlink-install --event-handlers console_direct+; then
        log_success "工作空间构建成功"
        return 0
    else
        log_error "工作空间构建失败"
        return 1
    fi
}

# 验证ROS2安装
verify_ros2_installation() {
    log_info "验证ROS2安装..."
    
    # 检查ROS2命令
    if ! command -v ros2 >/dev/null 2>&1; then
        log_error "ros2命令不可用"
        return 1
    fi
    
    # 检查ROS2包
    local package_count=$(ros2 pkg list | wc -l)
    log_success "ROS2包检查通过 ($package_count 个包)"
    
    # 检查Unitree包
    if ros2 pkg list | grep -q "unitree"; then
        log_success "Unitree ROS2包已安装"
        ros2 pkg list | grep unitree | while read -r pkg; do
            echo "  - $pkg"
        done
    else
        log_warning "Unitree ROS2包未找到"
    fi
}

# 测试Python集成
test_python_integration() {
    log_info "测试Python集成..."
    
    cd "$PROJECT_ROOT"
    
    # 测试基础ROS2包导入
    python3 -c "
import sys
try:
    import rclpy
    import std_msgs.msg
    import geometry_msgs.msg
    print('✅ 基础ROS2 Python包导入成功')
except ImportError as e:
    print(f'❌ ROS2 Python包导入失败: {e}')
    sys.exit(1)

# 测试Unitree包导入
try:
    import unitree_go.msg
    import unitree_api.msg
    print('✅ Unitree Python包导入成功')
except ImportError as e:
    print(f'⚠️  Unitree Python包导入失败: {e}')

# 测试Claudia ROS2管理器
try:
    sys.path.insert(0, 'src')
    from claudia.common.ros2_manager import ROS2Manager
    manager = ROS2Manager()
    print('✅ Claudia ROS2管理器导入成功')
    print(f'  项目根目录: {manager.project_root}')
    print(f'  工作空间: {manager.cyclonedds_ws}')
except ImportError as e:
    print(f'❌ Claudia ROS2管理器导入失败: {e}')
    sys.exit(1)
except Exception as e:
    print(f'❌ ROS2管理器初始化失败: {e}')
    sys.exit(1)
"
    
    if [ $? -eq 0 ]; then
        log_success "Python集成测试通过"
    else
        log_error "Python集成测试失败"
        return 1
    fi
}

# 测试ROS2通信
test_ros2_communication() {
    log_info "测试ROS2通信..."
    
    # 检查可用话题
    local topics=$(ros2 topic list 2>/dev/null | wc -l)
    if [ "$topics" -gt 0 ]; then
        log_success "ROS2话题发现成功 ($topics 个话题)"
        
        # 显示关键话题
        echo "关键话题:"
        ros2 topic list | grep -E "(sportmode|state|cmd)" | while read -r topic; do
            echo "  - $topic"
        done
    else
        log_warning "未发现ROS2话题（可能机器人未连接）"
    fi
    
    # 检查可用服务
    local services=$(ros2 service list 2>/dev/null | wc -l)
    if [ "$services" -gt 0 ]; then
        log_success "ROS2服务发现成功 ($services 个服务)"
    else
        log_warning "未发现ROS2服务"
    fi
}

# 创建环境变量文件
create_env_file() {
    log_info "创建环境变量文件..."
    
    local env_file="$PROJECT_ROOT/.env.ros2"
    
    cat > "$env_file" << EOF
# ROS2环境变量 - Claudia机器人系统

# CUDA环境
export PATH=/usr/local/cuda/bin:\$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda

# ROS2环境
export ROS_VERSION=2
export ROS_DISTRO=foxy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# 工作空间
source /opt/ros/foxy/setup.bash
source $CYCLONEDDS_WS/install/setup.bash

# Python路径
export PYTHONPATH=$CYCLONEDDS_WS/install/lib/python3.8/site-packages:\$PYTHONPATH
EOF
    
    chmod +x "$env_file"
    log_success "环境变量文件已创建: $env_file"
    
    echo "使用方法: source $env_file"
}

# 主函数
main() {
    echo "🚀 Claudia机器人ROS2环境设置脚本"
    echo "=============================================="
    echo "项目根目录: $PROJECT_ROOT"
    echo "工作空间: $CYCLONEDDS_WS"
    echo
    
    # 设置环境
    setup_cuda_environment || exit 1
    setup_ros2_environment || exit 1
    
    # 验证安装
    verify_ros2_installation || exit 1
    
    # 测试集成
    test_python_integration || exit 1
    test_ros2_communication
    
    # 创建环境文件
    create_env_file
    
    echo
    echo "=============================================="
    log_success "🎉 ROS2环境设置完成！"
    echo
    echo "下一步:"
    echo "1. source $PROJECT_ROOT/.env.ros2"
    echo "2. 测试与Unitree Go2机器人的连接"
    echo "3. 运行Claudia AI组件"
    
    return 0
}

# 脚本帮助
if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  --help     显示帮助信息"
    echo "  --build    强制重新构建工作空间"
    exit 0
fi

# 强制构建选项
if [ "${1:-}" = "--build" ]; then
    log_info "强制重新构建工作空间..."
    cd "$CYCLONEDDS_WS"
    rm -rf build/ install/ log/
    build_workspace || exit 1
fi

# 执行主函数
main "$@" 