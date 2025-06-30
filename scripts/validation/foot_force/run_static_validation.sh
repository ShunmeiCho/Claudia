#!/bin/bash
# scripts/validation/foot_force/run_static_validation.sh
# Generated: 2025-06-27 14:30:00 CST
# Purpose: Unitree Go2 足端力传感器静态验证启动脚本

set -e

# 脚本常量
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATION_DIR="${SCRIPT_DIR}/foot_force_validation"
MAIN_SCRIPT="${VALIDATION_DIR}/static_validation.py"
CONFIG_FILE="${VALIDATION_DIR}/validation_config.json"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
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

log_header() {
    echo -e "${PURPLE}$1${NC}"
}

# 显示标题
show_header() {
    clear
    echo -e "${CYAN}"
    echo "================================================================================"
    echo "               🦾 Unitree Go2 足端力传感器静态验证系统"
    echo "================================================================================"
    echo "版本: 1.0.0"
    echo "生成时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "脚本位置: ${SCRIPT_DIR}"
    echo "================================================================================"
    echo -e "${NC}"
}

# 检查系统环境
check_environment() {
    log_header "🔍 系统环境检查"
    
    local all_ok=true
    
    # 检查Python版本
    if command -v python3 &> /dev/null; then
        local python_version=$(python3 --version | cut -d' ' -f2)
        log_success "Python3 版本: ${python_version}"
    else
        log_error "Python3 未安装"
        all_ok=false
    fi
    
    # 检查必要的Python模块
    local required_modules=("numpy" "matplotlib" "scipy" "pandas")
    for module in "${required_modules[@]}"; do
        if python3 -c "import ${module}" &> /dev/null; then
            log_success "Python模块 ${module} 可用"
        else
            log_error "Python模块 ${module} 未安装"
            all_ok=false
        fi
    done
    
    # 检查Unitree SDK
    if python3 -c "from unitree_sdk2py.core.channel import ChannelSubscriber" &> /dev/null; then
        log_success "Unitree SDK2 Python 可用"
    else
        log_warning "Unitree SDK2 Python 不可用，将使用模拟模式"
    fi
    
    # 检查必要文件
    if [[ -f "${MAIN_SCRIPT}" ]]; then
        log_success "主脚本文件存在: ${MAIN_SCRIPT}"
    else
        log_error "主脚本文件不存在: ${MAIN_SCRIPT}"
        all_ok=false
    fi
    
    if [[ -f "${CONFIG_FILE}" ]]; then
        log_success "配置文件存在: ${CONFIG_FILE}"
    else
        log_warning "配置文件不存在: ${CONFIG_FILE}"
    fi
    
    # 检查目录权限
    if [[ -w "${VALIDATION_DIR}" ]]; then
        log_success "验证目录可写"
    else
        log_error "验证目录不可写: ${VALIDATION_DIR}"
        all_ok=false
    fi
    
    # 检查磁盘空间
    local available_space=$(df "${VALIDATION_DIR}" | tail -1 | awk '{print $4}')
    local available_mb=$((available_space / 1024))
    
    if [[ ${available_mb} -gt 1000 ]]; then
        log_success "磁盘空间充足: ${available_mb}MB"
    else
        log_warning "磁盘空间不足: ${available_mb}MB (建议至少1GB)"
    fi
    
    echo ""
    
    if [[ "${all_ok}" == true ]]; then
        log_success "✅ 环境检查通过！"
        return 0
    else
        log_error "❌ 环境检查失败！"
        return 1
    fi
}

# 显示菜单
show_menu() {
    echo ""
    log_header "📋 静态验证选项菜单"
    echo ""
    echo "1. 🔧 完整静态验证 (推荐)"
    echo "2. ⚡ 快速测试模式"
    echo "3. 🎨 仅数据分析和可视化"
    echo "4. ⚙️ 自定义参数验证"
    echo "5. 📊 查看历史报告"
    echo "6. 🧹 清理输出文件"
    echo "7. ❓ 显示帮助信息"
    echo "8. 🚪 退出"
    echo ""
}

# 获取用户选择
get_user_choice() {
    local choice
    while true; do
        read -p "请选择操作 [1-8]: " choice
        case $choice in
            [1-8])
                echo $choice
                return 0
                ;;
            *)
                log_warning "无效选择，请输入 1-8"
                ;;
        esac
    done
}

# 确认用户操作
confirm_action() {
    local prompt="$1"
    local response
    
    while true; do
        read -p "${prompt} [y/N]: " response
        case $response in
            [Yy]|[Yy][Ee][Ss])
                return 0
                ;;
            [Nn]|[Nn][Oo]|"")
                return 1
                ;;
            *)
                log_warning "请输入 y(是) 或 n(否)"
                ;;
        esac
    done
}

# 运行完整静态验证
run_full_validation() {
    log_header "🔧 运行完整静态验证"
    
    echo ""
    echo "完整静态验证包括以下测试项目："
    echo "  • 零负载测试 (机器人悬空状态)"
    echo "  • 静态站立测试 (机器人正常站立)"
    echo "  • 零点漂移分析 (长时间稳定性)"
    echo "  • 综合数据分析"
    echo "  • 可视化图表生成"
    echo ""
    
    local estimated_time=15
    log_info "预计总耗时: ${estimated_time} 分钟"
    
    if confirm_action "确认开始完整静态验证？"; then
        log_info "启动完整静态验证..."
        cd "${VALIDATION_DIR}"
        python3 static_validation.py --config validation_config.json --log-level INFO
    else
        log_info "操作已取消"
    fi
}

# 运行快速测试
run_quick_test() {
    log_header "⚡ 运行快速测试模式"
    
    echo ""
    echo "快速测试模式特点："
    echo "  • 缩短测试时间 (零负载: 10秒, 站立: 20秒, 漂移: 60秒)"
    echo "  • 包含完整的测试流程"
    echo "  • 适合系统调试和初步验证"
    echo ""
    
    local estimated_time=3
    log_info "预计总耗时: ${estimated_time} 分钟"
    
    if confirm_action "确认开始快速测试？"; then
        log_info "启动快速测试模式..."
        cd "${VALIDATION_DIR}"
        python3 static_validation.py --config validation_config.json --test-mode --log-level INFO
    else
        log_info "操作已取消"
    fi
}

# 仅运行分析和可视化
run_analysis_only() {
    log_header "🎨 仅运行数据分析和可视化"
    
    echo ""
    echo "此选项将："
    echo "  • 跳过数据采集过程"
    echo "  • 使用已有数据进行分析"
    echo "  • 生成可视化图表"
    echo "  • 生成分析报告"
    echo ""
    
    log_warning "注意: 需要有之前采集的数据文件"
    
    if confirm_action "确认仅运行分析和可视化？"; then
        log_info "启动分析和可视化..."
        cd "${VALIDATION_DIR}"
        python3 static_validation.py --config validation_config.json --skip-data-collection --log-level INFO
    else
        log_info "操作已取消"
    fi
}

# 自定义参数验证
run_custom_validation() {
    log_header "⚙️ 自定义参数验证"
    
    echo ""
    echo "请配置测试参数："
    
    # 零负载测试时间
    local zero_load_duration
    while true; do
        read -p "零负载测试时间 (秒) [默认: 30]: " zero_load_duration
        zero_load_duration=${zero_load_duration:-30}
        if [[ "$zero_load_duration" =~ ^[0-9]+$ ]] && [[ "$zero_load_duration" -ge 5 ]] && [[ "$zero_load_duration" -le 300 ]]; then
            break
        else
            log_warning "请输入5-300之间的整数"
        fi
    done
    
    # 静态站立测试时间
    local standing_duration
    while true; do
        read -p "静态站立测试时间 (秒) [默认: 60]: " standing_duration
        standing_duration=${standing_duration:-60}
        if [[ "$standing_duration" =~ ^[0-9]+$ ]] && [[ "$standing_duration" -ge 10 ]] && [[ "$standing_duration" -le 600 ]]; then
            break
        else
            log_warning "请输入10-600之间的整数"
        fi
    done
    
    # 零点漂移分析时间
    local drift_duration
    while true; do
        read -p "零点漂移分析时间 (秒) [默认: 300]: " drift_duration
        drift_duration=${drift_duration:-300}
        if [[ "$drift_duration" =~ ^[0-9]+$ ]] && [[ "$drift_duration" -ge 60 ]] && [[ "$drift_duration" -le 1800 ]]; then
            break
        else
            log_warning "请输入60-1800之间的整数"
        fi
    done
    
    # 日志级别
    local log_level
    echo ""
    echo "选择日志级别："
    echo "1. DEBUG (详细调试信息)"
    echo "2. INFO (标准信息)"
    echo "3. WARNING (仅警告和错误)"
    echo "4. ERROR (仅错误信息)"
    
    while true; do
        read -p "日志级别 [1-4, 默认: 2]: " log_choice
        log_choice=${log_choice:-2}
        case $log_choice in
            1) log_level="DEBUG"; break ;;
            2) log_level="INFO"; break ;;
            3) log_level="WARNING"; break ;;
            4) log_level="ERROR"; break ;;
            *) log_warning "请输入1-4" ;;
        esac
    done
    
    # 是否跳过可视化
    local skip_viz=""
    if confirm_action "跳过可视化生成以加快速度？"; then
        skip_viz="--skip-visualization"
    fi
    
    echo ""
    log_info "配置总结："
    log_info "  零负载测试: ${zero_load_duration}秒"
    log_info "  静态站立测试: ${standing_duration}秒"
    log_info "  零点漂移分析: ${drift_duration}秒"
    log_info "  日志级别: ${log_level}"
    log_info "  跳过可视化: $([ -n "$skip_viz" ] && echo "是" || echo "否")"
    
    local total_time=$((zero_load_duration + standing_duration + drift_duration / 60 + 2))
    log_info "  预计总耗时: ${total_time} 分钟"
    
    if confirm_action "确认开始自定义验证？"; then
        log_info "启动自定义验证..."
        
        # 创建临时配置文件
        local temp_config="/tmp/custom_validation_config.json"
        cp "${CONFIG_FILE}" "${temp_config}"
        
        # 修改配置参数
        python3 -c "
import json
with open('${temp_config}', 'r') as f:
    config = json.load(f)
config['static_validation']['zero_load_test_duration'] = ${zero_load_duration}
config['static_validation']['static_standing_duration'] = ${standing_duration}
config['static_validation']['zero_drift_duration'] = ${drift_duration}
with open('${temp_config}', 'w') as f:
    json.dump(config, f, indent=2)
"
        
        cd "${VALIDATION_DIR}"
        python3 static_validation.py --config "${temp_config}" --log-level "${log_level}" ${skip_viz}
        
        # 清理临时文件
        rm -f "${temp_config}"
    else
        log_info "操作已取消"
    fi
}

# 查看历史报告
view_reports() {
    log_header "📊 查看历史报告"
    
    local output_dir="${VALIDATION_DIR}/output"
    
    if [[ ! -d "${output_dir}" ]]; then
        log_warning "输出目录不存在: ${output_dir}"
        return
    fi
    
    # 查找报告文件
    local reports=($(find "${output_dir}" -name "*final_report*.json" -type f | sort -r))
    
    if [[ ${#reports[@]} -eq 0 ]]; then
        log_warning "未找到历史报告文件"
        return
    fi
    
    echo ""
    echo "找到 ${#reports[@]} 个历史报告："
    echo ""
    
    for i in "${!reports[@]}"; do
        local report="${reports[$i]}"
        local basename=$(basename "${report}")
        local timestamp=$(stat -c %y "${report}" | cut -d' ' -f1-2)
        local size=$(du -h "${report}" | cut -f1)
        
        echo "$((i+1)). ${basename}"
        echo "   时间: ${timestamp}"
        echo "   大小: ${size}"
        echo ""
    done
    
    while true; do
        read -p "选择报告编号 [1-${#reports[@]}, 0=返回]: " report_choice
        
        if [[ "$report_choice" == "0" ]]; then
            return
        elif [[ "$report_choice" =~ ^[0-9]+$ ]] && [[ "$report_choice" -ge 1 ]] && [[ "$report_choice" -le ${#reports[@]} ]]; then
            local selected_report="${reports[$((report_choice-1))]}"
            log_info "显示报告: $(basename "${selected_report}")"
            
            # 使用jq格式化显示（如果可用）
            if command -v jq &> /dev/null; then
                cat "${selected_report}" | jq '.'
            else
                cat "${selected_report}"
            fi
            
            echo ""
            read -p "按Enter继续..."
            break
        else
            log_warning "无效选择"
        fi
    done
}

# 清理输出文件
cleanup_outputs() {
    log_header "🧹 清理输出文件"
    
    local output_dir="${VALIDATION_DIR}/output"
    local log_dir="${VALIDATION_DIR}/logs"
    
    echo ""
    echo "可清理的内容："
    echo "1. 输出目录中的所有文件 (${output_dir})"
    echo "2. 日志目录中的所有文件 (${log_dir})"
    echo "3. 临时文件和缓存"
    echo "4. 全部清理"
    echo "5. 返回主菜单"
    echo ""
    
    while true; do
        read -p "选择清理内容 [1-5]: " cleanup_choice
        
        case $cleanup_choice in
            1)
                if [[ -d "${output_dir}" ]]; then
                    local file_count=$(find "${output_dir}" -type f | wc -l)
                    if [[ $file_count -gt 0 ]]; then
                        log_info "输出目录包含 ${file_count} 个文件"
                        if confirm_action "确认清理输出目录？"; then
                            rm -rf "${output_dir}"/*
                            log_success "输出目录已清理"
                        fi
                    else
                        log_info "输出目录已经为空"
                    fi
                else
                    log_info "输出目录不存在"
                fi
                break
                ;;
            2)
                if [[ -d "${log_dir}" ]]; then
                    local file_count=$(find "${log_dir}" -type f | wc -l)
                    if [[ $file_count -gt 0 ]]; then
                        log_info "日志目录包含 ${file_count} 个文件"
                        if confirm_action "确认清理日志目录？"; then
                            rm -rf "${log_dir}"/*
                            log_success "日志目录已清理"
                        fi
                    else
                        log_info "日志目录已经为空"
                    fi
                else
                    log_info "日志目录不存在"
                fi
                break
                ;;
            3)
                log_info "清理临时文件和缓存..."
                find "${VALIDATION_DIR}" -name "*.pyc" -delete 2>/dev/null || true
                find "${VALIDATION_DIR}" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
                find "${VALIDATION_DIR}" -name "*.tmp" -delete 2>/dev/null || true
                rm -f /tmp/custom_validation_config.json 2>/dev/null || true
                log_success "临时文件已清理"
                break
                ;;
            4)
                log_warning "这将删除所有输出文件、日志和临时文件"
                if confirm_action "确认全部清理？"; then
                    [[ -d "${output_dir}" ]] && rm -rf "${output_dir}"/*
                    [[ -d "${log_dir}" ]] && rm -rf "${log_dir}"/*
                    find "${VALIDATION_DIR}" -name "*.pyc" -delete 2>/dev/null || true
                    find "${VALIDATION_DIR}" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
                    find "${VALIDATION_DIR}" -name "*.tmp" -delete 2>/dev/null || true
                    rm -f /tmp/custom_validation_config.json 2>/dev/null || true
                    log_success "全部文件已清理"
                fi
                break
                ;;
            5)
                return
                ;;
            *)
                log_warning "无效选择，请输入1-5"
                ;;
        esac
    done
}

# 显示帮助信息
show_help() {
    log_header "❓ 帮助信息"
    
    echo ""
    echo "🦾 Unitree Go2 足端力传感器静态验证系统帮助"
    echo ""
    echo "📖 系统概述："
    echo "  本系统用于验证Unitree Go2机器人足端力传感器的准确性、稳定性和一致性。"
    echo "  通过多种测试确保传感器在静态条件下的可靠性。"
    echo ""
    echo "🔧 测试项目："
    echo "  1. 零负载测试   - 机器人悬空时的零点验证"
    echo "  2. 静态站立测试 - 正常站立时的力分布验证"
    echo "  3. 零点漂移分析 - 长时间稳定性监控"
    echo "  4. 综合数据分析 - 统计分析、频域分析、异常检测"
    echo "  5. 可视化报告   - 图表和仪表板生成"
    echo ""
    echo "⚙️ 系统要求："
    echo "  • Python 3.6+"
    echo "  • NumPy, SciPy, Matplotlib, Pandas"
    echo "  • Unitree SDK2 Python (可选，有模拟模式)"
    echo "  • 至少1GB磁盘空间"
    echo ""
    echo "🤖 机器人准备："
    echo "  • 零负载测试: 机器人完全悬空，足端不接触任何表面"
    echo "  • 静态站立测试: 机器人正常站立，四足平稳接触地面"
    echo "  • 保持测试环境安静，避免振动干扰"
    echo ""
    echo "📊 输出文件："
    echo "  • 验证报告 (JSON格式)"
    echo "  • 原始数据 (CSV格式)"
    echo "  • 可视化图表 (PNG格式)"
    echo "  • 日志文件"
    echo ""
    echo "🔍 故障排除："
    echo "  • 检查Python模块安装: pip3 install numpy scipy matplotlib pandas"
    echo "  • 确认Unitree SDK连接正常"
    echo "  • 检查磁盘空间是否充足"
    echo "  • 查看日志文件获取详细错误信息"
    echo ""
    echo "📞 技术支持："
    echo "  • 查看README.md获取详细文档"
    echo "  • 检查logs/目录获取诊断信息"
    echo "  • 运行环境检查确认系统状态"
    echo ""
    
    read -p "按Enter返回主菜单..."
}

# 主循环
main_loop() {
    while true; do
        show_menu
        local choice=$(get_user_choice)
        
        echo ""
        case $choice in
            1)
                run_full_validation
                ;;
            2)
                run_quick_test
                ;;
            3)
                run_analysis_only
                ;;
            4)
                run_custom_validation
                ;;
            5)
                view_reports
                ;;
            6)
                cleanup_outputs
                ;;
            7)
                show_help
                ;;
            8)
                log_info "感谢使用！再见！"
                exit 0
                ;;
        esac
        
        echo ""
        read -p "按Enter继续..."
    done
}

# 主函数
main() {
    # 设置信号处理
    trap 'echo ""; log_warning "操作被中断"; exit 130' INT TERM
    
    # 显示标题
    show_header
    
    # 检查环境
    if ! check_environment; then
        echo ""
        log_error "环境检查失败，请解决上述问题后重试"
        
        if confirm_action "是否查看故障排除帮助？"; then
            show_help
        fi
        
        exit 1
    fi
    
    # 进入主循环
    main_loop
}

# 检查是否直接运行
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 