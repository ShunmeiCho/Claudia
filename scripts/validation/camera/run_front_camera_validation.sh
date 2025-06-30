#!/bin/bash
# scripts/validation/camera/run_front_camera_validation.sh
# Generated: 2024-12-26 16:30:00
# Purpose: Unitree Go2前置摄像头验证快速启动脚本

set -e

# 脚本信息
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATION_DIR="${SCRIPT_DIR}/front_camera_validation"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

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
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示帮助信息
show_help() {
    cat << EOF
Unitree Go2前置摄像头验证快速启动脚本

用法:
    $0 [选项]

选项:
    -h, --help          显示此帮助信息
    -q, --quick         快速验证模式（仅基础测试）
    -f, --full          完整验证模式（包含压力测试）
    -c, --config FILE   指定配置文件
    -o, --output DIR    指定输出目录
    -v, --verbose       详细输出模式
    --dry-run          预运行模式（不执行实际测试）

示例:
    $0                  # 默认完整验证
    $0 -q               # 快速验证
    $0 -c my_config.json -o ./my_results -v  # 自定义配置和输出

EOF
}

# 检查环境
check_environment() {
    log_info "检查运行环境..."
    
    # 检查Python
    if ! command -v python3 &> /dev/null; then
        log_error "Python3未找到，请安装Python 3.7+"
        exit 1
    fi
    
    local python_version=$(python3 --version | cut -d' ' -f2)
    log_info "Python版本: $python_version"
    
    # 检查必要的Python包
    local required_packages=("cv2" "numpy" "skimage")
    for package in "${required_packages[@]}"; do
        if ! python3 -c "import $package" &> /dev/null; then
            log_warning "Python包 $package 未找到，可能影响功能"
        fi
    done
    
    # 检查摄像头设备
    if ls /dev/video* &> /dev/null; then
        log_info "找到摄像头设备: $(ls /dev/video*)"
    else
        log_warning "未找到摄像头设备，验证可能失败"
    fi
    
    # 检查权限
    if groups | grep -q video; then
        log_info "用户已在video组中"
    else
        log_warning "用户不在video组中，可能需要摄像头权限"
    fi
    
    log_success "环境检查完成"
}

# 验证脚本存在
check_scripts() {
    log_info "检查验证脚本..."
    
    if [ ! -d "$VALIDATION_DIR" ]; then
        log_error "验证目录不存在: $VALIDATION_DIR"
        exit 1
    fi
    
    local required_files=(
        "main_validation_script.py"
        "camera_config.py"
        "performance_tester.py"
        "image_quality_analyzer.py"
        "validation_config.json"
    )
    
    for file in "${required_files[@]}"; do
        if [ ! -f "$VALIDATION_DIR/$file" ]; then
            log_error "缺少必要文件: $file"
            exit 1
        fi
    done
    
    log_success "验证脚本检查完成"
}

# 创建输出目录
setup_output_dir() {
    local output_dir="$1"
    if [ -z "$output_dir" ]; then
        output_dir="logs/camera_validation"
    fi
    
    mkdir -p "$output_dir"
    log_info "输出目录: $output_dir"
}

# 运行验证
run_validation() {
    local mode="$1"
    local config_file="$2"
    local output_dir="$3"
    local verbose="$4"
    local dry_run="$5"
    
    log_info "开始前置摄像头验证..."
    log_info "模式: $mode"
    log_info "时间: $TIMESTAMP"
    
    # 构建命令
    local cmd="python3 $VALIDATION_DIR/main_validation_script.py"
    
    if [ -n "$config_file" ]; then
        cmd="$cmd --config $config_file"
    fi
    
    if [ -n "$output_dir" ]; then
        cmd="$cmd --output $output_dir"
    fi
    
    if [ "$verbose" = "true" ]; then
        cmd="$cmd --verbose"
    fi
    
    log_info "执行命令: $cmd"
    
    if [ "$dry_run" = "true" ]; then
        log_warning "预运行模式 - 不执行实际测试"
        return 0
    fi
    
    # 切换到验证目录
    cd "$VALIDATION_DIR"
    
    # 执行验证
    if eval "$cmd"; then
        log_success "验证完成！"
        return 0
    else
        log_error "验证失败！"
        return 1
    fi
}

# 主函数
main() {
    # 默认参数
    local mode="full"
    local config_file=""
    local output_dir=""
    local verbose="false"
    local dry_run="false"
    
    # 解析参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -q|--quick)
                mode="quick"
                shift
                ;;
            -f|--full)
                mode="full"
                shift
                ;;
            -c|--config)
                config_file="$2"
                shift 2
                ;;
            -o|--output)
                output_dir="$2"
                shift 2
                ;;
            -v|--verbose)
                verbose="true"
                shift
                ;;
            --dry-run)
                dry_run="true"
                shift
                ;;
            *)
                log_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 显示欢迎信息
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}Unitree Go2前置摄像头验证系统${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    
    # 执行预检查
    check_environment
    check_scripts
    setup_output_dir "$output_dir"
    
    echo ""
    log_info "准备开始验证..."
    
    # 运行验证
    if run_validation "$mode" "$config_file" "$output_dir" "$verbose" "$dry_run"; then
        echo ""
        log_success "前置摄像头验证成功完成！"
        echo -e "${GREEN}查看结果: ${output_dir:-logs/camera_validation}${NC}"
        exit 0
    else
        echo ""
        log_error "前置摄像头验证失败！"
        log_info "请检查日志文件获取详细信息"
        exit 1
    fi
}

# 执行主函数
main "$@" 