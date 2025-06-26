#!/bin/bash

#1.3
# CycloneDX工作空间设置脚本 - Claudia机器人系统 
# Generated: 2025-06-26
# Purpose: 任务1.3 - 设置cyclonedds_ws工作空间和编译Unitree ROS2包
# Platform: aarch64 Ubuntu 20.04.5 LTS

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

# 错误处理
cleanup_on_failure() {
    local operation_name="$1"
    local failure_reason="$2"
    
    log_error "$operation_name 失败: $failure_reason"
    
    # 记录失败信息
    FAILURE_LOG="logs/failures/$(date '+%Y%m%d_%H%M%S')_${operation_name}_failure.log"
    mkdir -p "$(dirname "$FAILURE_LOG")"
    
    cat > "$FAILURE_LOG" << EOF
Operation: $operation_name
Time: $(date '+%Y-%m-%d %H:%M:%S %Z')
Reason: $failure_reason
Platform: $(uname -a)
Working Directory: $(pwd)
Git Status: $(git status --porcelain 2>/dev/null || echo "Not a git repository")
Disk Space: $(df -h .)
Memory: $(free -h)
EOF
    
    echo "🗂️ 失败信息已记录: $FAILURE_LOG"
}

# 设置错误处理
trap 'cleanup_on_failure "cyclonedds_workspace_setup" "命令执行失败"' ERR

# 开始设置
echo "🚀 开始设置CycloneDX工作空间 - $(date '+%Y-%m-%d %H:%M:%S')"
echo "📁 项目根目录: $PROJECT_ROOT"
echo "🔧 工作空间目录: $CYCLONEDDS_WS"

# 1. 检查前提条件
log_info "检查前提条件..."

# 检查ROS2 Foxy
if ! command -v ros2 > /dev/null 2>&1; then
    log_error "ROS2 Foxy 未安装或未在PATH中"
    exit 1
fi

# 检查colcon
if ! command -v colcon > /dev/null 2>&1; then
    log_error "colcon 构建工具未安装"
    log_info "请运行: sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# 检查网络连接
if ! ping -c 1 github.com > /dev/null 2>&1; then
    log_warning "网络连接检查失败，可能影响仓库克隆"
fi

log_success "前提条件检查完成"

# 2. 创建工作空间目录结构
log_info "创建工作空间目录结构..."

if [ -d "$CYCLONEDDS_WS" ]; then
    log_warning "工作空间目录已存在: $CYCLONEDDS_WS"
    read -p "是否重新创建工作空间? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log_info "备份现有工作空间..."
        BACKUP_DIR="${CYCLONEDDS_WS}.backup_$(date '+%Y%m%d_%H%M%S')"
        mv "$CYCLONEDDS_WS" "$BACKUP_DIR"
        log_info "备份完成: $BACKUP_DIR"
    else
        log_info "使用现有工作空间目录"
    fi
fi

# 创建工作空间结构
mkdir -p "$CYCLONEDDS_WS/src"
cd "$CYCLONEDDS_WS"

log_success "工作空间目录结构创建完成"

# 3. 克隆Unitree仓库
log_info "克隆Unitree仓库..."

cd src

# 克隆unitree_ros2
if [ ! -d "unitree_ros2" ]; then
    log_info "克隆unitree_ros2仓库..."
    git clone https://github.com/unitreerobotics/unitree_ros2.git
    log_success "unitree_ros2 克隆完成"
else
    log_info "unitree_ros2 已存在，跳过克隆"
fi

# 克隆unitree_sdk2  
if [ ! -d "unitree_sdk2" ]; then
    log_info "克隆unitree_sdk2仓库..."
    git clone https://github.com/unitreerobotics/unitree_sdk2.git
    log_success "unitree_sdk2 克隆完成"
else
    log_info "unitree_sdk2 已存在，跳过克隆"
fi

# 检查克隆结果
log_info "验证克隆的仓库..."
for repo in unitree_ros2 unitree_sdk2; do
    if [ -d "$repo" ] && [ "$(ls -A $repo)" ]; then
        log_success "$repo 克隆验证成功"
    else
        log_error "$repo 克隆失败或目录为空"
        exit 1
    fi
done

cd ..

# 4. 设置ROS2环境
log_info "设置ROS2环境..."

# 确保使用正确的RMW实现名称
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# 源ROS2环境
source /opt/ros/foxy/setup.bash

log_success "ROS2环境设置完成 (RMW: $RMW_IMPLEMENTATION)"

# 5. 构建工作空间
log_info "开始构建工作空间..."

BUILD_START_TIME=$(date '+%s')

# 构建所有包
if colcon build --symlink-install --event-handlers console_direct+; then
    BUILD_END_TIME=$(date '+%s')
    BUILD_DURATION=$((BUILD_END_TIME - BUILD_START_TIME))
    log_success "工作空间构建完成 (耗时: ${BUILD_DURATION}秒)"
else
    log_error "工作空间构建失败"
    exit 1
fi

# 6. 验证构建结果
log_info "验证构建结果..."

# 检查install目录
if [ -d "install" ] && [ "$(ls -A install)" ]; then
    log_success "install目录创建成功"
else
    log_error "install目录未创建或为空"
    exit 1
fi

# 源工作空间环境
source install/setup.bash

# 检查ROS2包
log_info "检查已安装的ROS2包..."
UNITREE_PACKAGES=$(ros2 pkg list | grep unitree | wc -l)
if [ "$UNITREE_PACKAGES" -gt 0 ]; then
    log_success "发现 $UNITREE_PACKAGES 个Unitree ROS2包"
    ros2 pkg list | grep unitree
else
    log_warning "未发现Unitree ROS2包"
fi

# 7. 测试Python导入
log_info "测试Python包导入..."

python3 -c "
try:
    import unitree_go.msg
    import unitree_api.msg
    print('✅ Python包导入成功')
except ImportError as e:
    print(f'❌ Python包导入失败: {e}')
    exit(1)
"

# 8. 创建环境设置脚本
log_info "创建环境设置脚本..."

ENV_SCRIPT="$CYCLONEDDS_WS/setup_env.sh"
cat > "$ENV_SCRIPT" << 'EOF'
#!/bin/bash
# CycloneDX工作空间环境设置
# 注意：使用正确的RMW实现名称

# ROS2基础环境
source /opt/ros/foxy/setup.bash

# 工作空间环境
source "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash"

# DDS配置 - 使用正确的rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# CycloneDDS网络配置
export CYCLONEDDS_URI='<CycloneDX><Domain><General><Interfaces>
 <NetworkInterface name="eth0" priority="default" multicast="default" />
 </Interfaces></General></Domain></CycloneDX>'

echo "✅ CycloneDX工作空间环境已加载"
echo "   RMW实现: $RMW_IMPLEMENTATION"
echo "   ROS域ID: $ROS_DOMAIN_ID"
EOF

chmod +x "$ENV_SCRIPT"
log_success "环境设置脚本创建完成: $ENV_SCRIPT"

# 9. 生成状态报告
log_info "生成工作空间状态报告..."

REPORT_FILE="$CYCLONEDDS_WS/workspace_status.txt"
cat > "$REPORT_FILE" << EOF
CycloneDX工作空间状态报告
Generated: $(date '+%Y-%m-%d %H:%M:%S %Z')
====================================

工作空间路径: $CYCLONEDDS_WS
构建时间: ${BUILD_DURATION}秒
RMW实现: rmw_cyclonedds_cpp (已修正)

目录结构:
$(tree -L 2 2>/dev/null || find . -maxdepth 2 -type d | sort)

ROS2包列表:
$(ros2 pkg list | grep unitree)

Python包验证:
$(python3 -c "import unitree_go.msg, unitree_api.msg; print('✅ Python导入成功')" 2>/dev/null || echo "❌ Python导入失败")

磁盘使用:
$(du -sh .)

环境脚本: $ENV_SCRIPT
报告文件: $REPORT_FILE
EOF

log_success "状态报告生成完成: $REPORT_FILE"

# 10. 清理临时文件
log_info "清理临时文件..."
find /tmp -name "claudia*" -mtime +1 -type f -delete 2>/dev/null || true
find . -name "*.tmp" -delete 2>/dev/null || true

# 最终成功信息
echo ""
echo "🎉 =========================="
echo "🎉 CycloneDX工作空间设置完成！"
echo "🎉 =========================="
echo ""
echo "📋 使用说明:"
echo "   1. 加载环境: source $CYCLONEDDS_WS/setup_env.sh"
echo "   2. 查看状态: cat $CYCLONEDDS_WS/workspace_status.txt"
echo "   3. 重新构建: cd $CYCLONEDDS_WS && colcon build"
echo ""
echo "✅ 任务1.3完成 - $(date '+%Y-%m-%d %H:%M:%S')"

# 返回项目根目录
cd "$PROJECT_ROOT" 