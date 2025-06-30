#!/bin/bash
# Unitree Go2 音频I/O系统验证启动脚本
# Generated: 2025-06-30 13:06:45
# Platform: Ubuntu 20.04 - aarch64

set -e

# 脚本配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
VALIDATION_SCRIPT="$SCRIPT_DIR/audio_validation_main.py"

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
🎵 Unitree Go2 音频I/O系统验证

用法: $0 [选项]

选项:
  -p, --phases PHASES     要执行的验证阶段 (A,B,C,D,E)，默认: A,B
  -c, --config CONFIG     配置文件路径
  -sr, --sample-rate RATE 采样率 (默认: 44100)
  -ch, --channels NUM     音频通道数 (默认: 2)
  -d, --duration SECONDS  测试持续时间 (默认: 5.0)
  -i, --install           安装依赖库
  -h, --help              显示此帮助信息

阶段说明:
  Phase A: 硬件连接与基础采集验证
  Phase B: 麦克风阵列全方位测试
  Phase C: 扬声器校准与音质评估 (待实现)
  Phase D: ROS2音频话题集成验证 (待实现)
  Phase E: 综合可视化与性能报告生成 (待实现)

示例:
  $0                                    # 运行默认验证 (Phase A,B)
  $0 -p A B C                          # 运行指定阶段
  $0 -sr 48000 -ch 2 -d 10.0          # 自定义音频参数
  $0 -i                                 # 安装依赖
  $0 -c custom_config.json             # 使用自定义配置

EOF
}

# 检查依赖
check_dependencies() {
    log_info "检查Python依赖..."
    
    local missing_deps=()
    
    # 检查Python库
    for dep in sounddevice scipy librosa matplotlib numpy; do
        if ! python3 -c "import $dep" 2>/dev/null; then
            missing_deps+=("$dep")
        fi
    done
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_warning "以下依赖缺失: ${missing_deps[*]}"
        log_info "运行 '$0 -i' 来安装依赖"
        return 1
    fi
    
    log_success "所有依赖已满足"
    return 0
}

# 安装依赖
install_dependencies() {
    log_info "开始安装音频验证依赖..."
    
    # 更新包列表
    log_info "更新系统包列表..."
    sudo apt update
    
    # 安装系统依赖
    log_info "安装系统音频依赖..."
    sudo apt install -y \
        portaudio19-dev \
        libasound2-dev \
        libsndfile1-dev \
        libfftw3-dev \
        python3-pip \
        python3-dev
    
    # 安装Python依赖
    log_info "安装Python音频处理库..."
    pip3 install --user \
        sounddevice \
        scipy \
        librosa \
        matplotlib \
        numpy \
        audio-common-msgs || log_warning "audio-common-msgs安装失败，ROS2集成将不可用"
    
    log_success "依赖安装完成!"
}

# 检查音频设备
check_audio_devices() {
    log_info "检查音频设备..."
    
    # 检查ALSA设备
    if command -v aplay &> /dev/null; then
        log_info "可用音频播放设备:"
        aplay -l | grep -E "^card" || log_warning "未找到音频播放设备"
    fi
    
    if command -v arecord &> /dev/null; then
        log_info "可用音频录制设备:"
        arecord -l | grep -E "^card" || log_warning "未找到音频录制设备"
    fi
    
    # 检查PulseAudio
    if command -v pactl &> /dev/null; then
        log_info "PulseAudio源设备:"
        pactl list short sources 2>/dev/null || log_warning "PulseAudio未运行"
        
        log_info "PulseAudio汇设备:"
        pactl list short sinks 2>/dev/null || log_warning "PulseAudio未运行"
    fi
}

# 预验证环境
pre_validation_check() {
    log_info "执行预验证检查..."
    
    # 检查Python版本
    python_version=$(python3 --version 2>&1 | awk '{print $2}')
    log_info "Python版本: $python_version"
    
    # 检查是否在Unitree环境中
    if [ -f "$PROJECT_ROOT/cyclonedx_ws/install/setup.bash" ]; then
        log_info "检测到Unitree工作空间"
        source "$PROJECT_ROOT/cyclonedx_ws/install/setup.bash" 2>/dev/null || true
    fi
    
    # 检查ROS2环境
    if command -v ros2 &> /dev/null; then
        log_info "检测到ROS2环境"
        export ROS2_AVAILABLE=1
    else
        log_warning "未检测到ROS2环境，将跳过ROS2集成测试"
        export ROS2_AVAILABLE=0
    fi
    
    # 检查权限
    if ! groups | grep -q audio; then
        log_warning "当前用户不在audio组，可能会遇到音频设备权限问题"
        log_info "可运行: sudo usermod -a -G audio \$USER"
    fi
    
    check_audio_devices
}

# 主验证函数
run_validation() {
    local phases="$1"
    local config="$2"
    local sample_rate="$3"
    local channels="$4"
    local duration="$5"
    
    log_info "启动音频I/O系统验证..."
    log_info "阶段: $phases"
    log_info "采样率: ${sample_rate}Hz"
    log_info "通道数: $channels"
    log_info "测试时长: ${duration}s"
    
    # 构建命令参数
    local cmd_args=()
    
    if [ -n "$phases" ]; then
        IFS=',' read -ra PHASE_ARRAY <<< "$phases"
        cmd_args+=("--phases" "${PHASE_ARRAY[@]}")
    fi
    
    if [ -n "$config" ]; then
        cmd_args+=("--config" "$config")
    fi
    
    if [ -n "$sample_rate" ]; then
        cmd_args+=("--sample-rate" "$sample_rate")
    fi
    
    if [ -n "$channels" ]; then
        cmd_args+=("--channels" "$channels")
    fi
    
    if [ -n "$duration" ]; then
        cmd_args+=("--duration" "$duration")
    fi
    
    # 切换到项目根目录
    cd "$PROJECT_ROOT"
    
    # 运行验证脚本
    log_info "执行验证脚本..."
    python3 "$VALIDATION_SCRIPT" "${cmd_args[@]}"
    
    local exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        log_success "音频验证完成!"
    else
        log_error "音频验证失败 (退出码: $exit_code)"
        return $exit_code
    fi
}

# 主函数
main() {
    local phases=""
    local config=""
    local sample_rate="44100"
    local channels="2"
    local duration="5.0"
    local install_deps=false
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -p|--phases)
                phases="$2"
                shift 2
                ;;
            -c|--config)
                config="$2"
                shift 2
                ;;
            -sr|--sample-rate)
                sample_rate="$2"
                shift 2
                ;;
            -ch|--channels)
                channels="$2"
                shift 2
                ;;
            -d|--duration)
                duration="$2"
                shift 2
                ;;
            -i|--install)
                install_deps=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 显示启动信息
    echo "🎵 Unitree Go2 音频I/O系统验证"
    echo "=========================================="
    echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "平台: $(uname -a)"
    echo "项目: $PROJECT_ROOT"
    echo ""
    
    # 安装依赖模式
    if [ "$install_deps" = true ]; then
        install_dependencies
        exit 0
    fi
    
    # 检查验证脚本是否存在
    if [ ! -f "$VALIDATION_SCRIPT" ]; then
        log_error "验证脚本不存在: $VALIDATION_SCRIPT"
        exit 1
    fi
    
    # 检查依赖
    if ! check_dependencies; then
        log_error "依赖检查失败，请先安装依赖"
        exit 1
    fi
    
    # 预验证检查
    pre_validation_check
    
    # 运行验证
    run_validation "$phases" "$config" "$sample_rate" "$channels" "$duration"
}

# 脚本入口
main "$@" 