#!/bin/bash
# scripts/validation/camera/install_camera_validation_deps.sh
# Generated: 2025-06-27 14:10:00
# Purpose: 安装前置摄像头验证系统的依赖包

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
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否为root用户
check_root() {
    if [ "$EUID" -eq 0 ]; then
        log_warning "建议使用普通用户运行此脚本"
        log_warning "系统包将使用sudo安装"
    fi
}

# 更新系统包
update_system_packages() {
    log_info "更新系统包列表..."
    
    if command -v apt-get &> /dev/null; then
        sudo apt-get update
        log_success "APT包列表更新完成"
    elif command -v yum &> /dev/null; then
        sudo yum update
        log_success "YUM包列表更新完成"
    else
        log_warning "未识别的包管理器，跳过系统更新"
    fi
}

# 安装系统依赖
install_system_deps() {
    log_info "安装系统依赖包..."
    
    if command -v apt-get &> /dev/null; then
        # Ubuntu/Debian系统
        local packages=(
            "python3"
            "python3-pip"
            "python3-dev"
            "build-essential"
            "cmake"
            "pkg-config"
            "libjpeg-dev"
            "libtiff5-dev"
            "libpng-dev"
            "libavcodec-dev"
            "libavformat-dev"
            "libswscale-dev"
            "libv4l-dev"
            "libxvidcore-dev"
            "libx264-dev"
            "libgtk-3-dev"
            "libatlas-base-dev"
            "gfortran"
        )
        
        log_info "安装APT包: ${packages[*]}"
        sudo apt-get install -y "${packages[@]}"
        
    elif command -v yum &> /dev/null; then
        # CentOS/RHEL系统
        local packages=(
            "python3"
            "python3-pip"
            "python3-devel"
            "gcc"
            "gcc-c++"
            "cmake"
            "pkgconfig"
            "libjpeg-turbo-devel"
            "libpng-devel"
            "libtiff-devel"
            "opencv-devel"
        )
        
        log_info "安装YUM包: ${packages[*]}"
        sudo yum install -y "${packages[@]}"
        
    else
        log_error "未识别的Linux发行版，请手动安装依赖"
        return 1
    fi
    
    log_success "系统依赖安装完成"
}

# 安装Python包
install_python_deps() {
    log_info "安装Python依赖包..."
    
    # 升级pip
    python3 -m pip install --upgrade pip
    
    # 核心依赖包
    local python_packages=(
        "opencv-python>=4.0.0"
        "numpy>=1.18.0"
        "scikit-image>=0.17.0"
        "matplotlib>=3.0.0"
        "Pillow>=7.0.0"
    )
    
    log_info "安装Python包: ${python_packages[*]}"
    
    # 使用用户模式安装，避免权限问题
    python3 -m pip install --user "${python_packages[@]}"
    
    log_success "Python依赖安装完成"
}

# 验证安装
verify_installation() {
    log_info "验证安装结果..."
    
    # 检查Python包
    local packages_to_verify=("cv2" "numpy" "skimage" "matplotlib" "PIL")
    
    for package in "${packages_to_verify[@]}"; do
        if python3 -c "import $package" &> /dev/null; then
            log_success "✓ $package 安装成功"
        else
            log_error "✗ $package 安装失败"
        fi
    done
    
    # 检查摄像头设备
    if ls /dev/video* &> /dev/null; then
        log_success "✓ 找到摄像头设备: $(ls /dev/video* | tr '\n' ' ')"
    else
        log_warning "✗ 未找到摄像头设备"
    fi
    
    # 检查用户权限
    if groups | grep -q video; then
        log_success "✓ 用户在video组中"
    else
        log_warning "✗ 用户不在video组中"
        log_info "运行以下命令添加video组权限:"
        log_info "sudo usermod -a -G video $USER"
        log_info "然后重新登录"
    fi
    
    log_success "安装验证完成"
}

# 安装完成后的配置
post_install_config() {
    log_info "执行安装后配置..."
    
    # 检查是否需要添加用户到video组
    if ! groups | grep -q video; then
        log_info "添加用户到video组..."
        sudo usermod -a -G video "$USER"
        log_warning "需要重新登录以使组权限生效"
    fi
    
    # 创建必要的目录
    mkdir -p logs/camera_validation
    mkdir -p tmp/downloads
    
    log_success "安装后配置完成"
}

# 显示安装完成信息
show_completion_info() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}前置摄像头验证系统依赖安装完成！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    
    log_info "下一步操作:"
    echo "1. 如果添加了video组权限，请重新登录"
    echo "2. 运行验证脚本:"
    echo "   ./scripts/validation/camera/run_front_camera_validation.sh --dry-run"
    echo "3. 进行实际验证:"
    echo "   ./scripts/validation/camera/run_front_camera_validation.sh"
    echo ""
    
    log_info "如果遇到问题，请查看:"
    echo "- scripts/validation/camera/front_camera_validation/README_front_camera_validation.md"
    echo "- 日志文件: front_camera_validation.log"
}

# 主函数
main() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}前置摄像头验证系统依赖安装${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    
    check_root
    
    log_info "开始安装依赖..."
    
    # 检查是否有网络连接
    if ! ping -c 1 google.com &> /dev/null && ! ping -c 1 baidu.com &> /dev/null; then
        log_error "网络连接失败，无法下载依赖包"
        exit 1
    fi
    
    # 执行安装步骤
    update_system_packages
    install_system_deps
    install_python_deps
    verify_installation
    post_install_config
    show_completion_info
    
    log_success "依赖安装脚本执行完成！"
}

# 脚本入口
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 