#!/bin/bash
# Generated: 2025-01-27 11:55:00
# Purpose: Execute IMU validation system with various options
# Platform: Jetson Xavier NX Ubuntu 18.04

set -e

# 脚本目录和路径设置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATION_DIR="$SCRIPT_DIR/imu_validation"
MAIN_SCRIPT="$VALIDATION_DIR/main_validation_script.py"

echo "🤖 Unitree Go2 IMU验证系统"
echo "=========================================="

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

# 显示帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助信息"
    echo "  -c, --config <file>     使用自定义配置文件"
    echo "  -t, --test              运行快速测试模式(5秒)"
    echo "  -v, --verbose           详细输出模式"
    echo "  -r, --report-only       仅生成报告，不进行新的数据采集"
    echo "  --install-deps          安装必要依赖"
    echo "  --check-env             检查环境和依赖"
    echo "  --interactive           交互式模式选择验证项目"
    echo ""
    echo "示例:"
    echo "  $0                              # 运行完整验证"
    echo "  $0 --test                       # 快速测试"
    echo "  $0 -c custom_config.json        # 使用自定义配置"
    echo "  $0 --interactive                # 交互式选择验证项目"
    echo "  $0 --check-env                  # 检查环境状态"
    echo ""
}

# 检查环境和依赖
check_environment() {
    echo "🔍 检查运行环境..."
    echo "⏰ 当前时间: $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo "📁 工作目录: $(pwd)"
    echo "💾 磁盘使用: $(df . | tail -1 | awk '{print $5}')"
    echo "🧠 内存使用: $(free | grep Mem | awk '{printf "%.0f%%", $3/$2 * 100.0}')"
    
    # 检查Python
    echo -n "检查Python3... "
    if command -v python3 &> /dev/null; then
        echo "✅ $(python3 --version)"
    else
        echo "❌ Python3未找到"
        exit 1
    fi
    
    # 检查验证脚本
    echo -n "检查验证脚本... "
    if [ -f "$MAIN_SCRIPT" ]; then
        echo "✅ 找到主验证脚本"
    else
        echo "❌ 未找到主验证脚本: $MAIN_SCRIPT"
        exit 1
    fi
    
    # 检查Python依赖
    echo "检查Python依赖:"
    local deps_ok=true
    local required_modules=("numpy" "matplotlib" "scipy" "yaml" "jsonschema")
    
    for module in "${required_modules[@]}"; do
        echo -n "  $module... "
        if python3 -c "import $module" 2>/dev/null; then
            echo "✅"
        else
            echo "❌"
            deps_ok=false
        fi
    done
    
    # 检查Unitree SDK
    echo -n "  unitree_sdk2py... "
    if python3 -c "import unitree_sdk2py" 2>/dev/null; then
        echo "✅"
    else
        echo "❌ (需要手动安装)"
        echo "    请运行: $0 --install-deps"
        deps_ok=false
    fi
    
    if [ "$deps_ok" = false ]; then
        echo ""
        echo "⚠️ 部分依赖缺失，建议运行: $0 --install-deps"
        if [ "$1" = "--strict" ]; then
            exit 1
        fi
    else
        echo "✅ 所有依赖检查通过"
    fi
    
    echo ""
}

# 交互式模式
interactive_mode() {
    echo "🎛️ 交互式验证模式"
    echo "请选择要执行的验证项目:"
    echo ""
    echo "1) 完整验证流程 (包含所有测试)"
    echo "2) 静态稳定性测试"
    echo "3) 动态响应测试"
    echo "4) 校准精度分析"
    echo "5) 实时数据可视化"
    echo "6) 快速测试模式 (5秒)"
    echo "7) 环境检查"
    echo "8) 安装依赖"
    echo ""
    
    read -p "请输入选项 (1-8): " choice
    
    case $choice in
        1)
            echo "🚀 启动完整验证流程..."
            run_validation
            ;;
        2)
            echo "🔍 启动静态稳定性测试..."
            python3 "$MAIN_SCRIPT" --mode static
            ;;
        3)
            echo "🏃 启动动态响应测试..."
            python3 "$MAIN_SCRIPT" --mode dynamic
            ;;
        4)
            echo "📊 启动校准精度分析..."
            python3 "$MAIN_SCRIPT" --mode calibration
            ;;
        5)
            echo "📈 启动实时数据可视化..."
            python3 "$MAIN_SCRIPT" --mode visualize
            ;;
        6)
            echo "⚡ 启动快速测试模式..."
            python3 "$MAIN_SCRIPT" --test-duration 5
            ;;
        7)
            check_environment --strict
            ;;
        8)
            "$SCRIPT_DIR/install_imu_validation_deps.sh"
            ;;
        *)
            echo "❌ 无效选项: $choice"
            exit 1
            ;;
    esac
}

# 运行验证的核心函数
run_validation() {
    local config_file=""
    local test_mode=false
    local verbose=false
    local report_only=false
    local extra_args=()
    
    # 处理参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--config)
                config_file="$2"
                shift 2
                ;;
            -t|--test)
                test_mode=true
                shift
                ;;
            -v|--verbose)
                verbose=true
                shift
                ;;
            -r|--report-only)
                report_only=true
                shift
                ;;
            *)
                extra_args+=("$1")
                shift
                ;;
        esac
    done
    
    # 构建命令
    local cmd="python3 \"$MAIN_SCRIPT\""
    
    if [ -n "$config_file" ]; then
        if [ -f "$config_file" ]; then
            cmd="$cmd --config \"$config_file\""
        else
            echo "❌ 配置文件不存在: $config_file"
            exit 1
        fi
    fi
    
    if [ "$test_mode" = true ]; then
        cmd="$cmd --test-duration 5"
    fi
    
    if [ "$verbose" = true ]; then
        cmd="$cmd --verbose"
    fi
    
    if [ "$report_only" = true ]; then
        cmd="$cmd --report-only"
    fi
    
    # 添加额外参数
    for arg in "${extra_args[@]}"; do
        cmd="$cmd \"$arg\""
    done
    
    echo "🚀 启动IMU验证..."
    echo "命令: $cmd"
    echo ""
    
    # 切换到验证目录并执行
    cd "$VALIDATION_DIR"
    eval $cmd
    
    local exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        echo ""
        echo "✅ IMU验证完成!"
        echo "📄 请查看生成的报告文件"
    else
        echo ""
        echo "❌ IMU验证失败 (退出码: $exit_code)"
        echo "请检查错误信息和日志文件"
    fi
    
    return $exit_code
}

# 主程序逻辑
main() {
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            --install-deps)
                "$SCRIPT_DIR/install_imu_validation_deps.sh"
                exit $?
                ;;
            --check-env)
                check_environment --strict
                exit $?
                ;;
            --interactive)
                check_environment
                interactive_mode
                exit $?
                ;;
            *)
                # 其他参数传递给验证函数
                check_environment
                run_validation "$@"
                exit $?
                ;;
        esac
    done
    
    # 如果没有参数，运行默认验证
    check_environment
    echo "启动默认验证模式..."
    echo "提示: 使用 --interactive 进入交互式模式"
    echo "     使用 --help 查看所有选项"
    echo ""
    
    run_validation
}

# 执行主程序
main "$@" 