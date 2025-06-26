#!/bin/bash
# Claudia机器人项目日常清理脚本
# Generated: 2025-06-26 18:43:27
# Purpose: 自动清理临时文件、日志文件、构建缓存等

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
    echo -e "${GREEN}[✅ SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[⚠️  WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[❌ ERROR]${NC} $1"
}

# 检查系统状态
check_system_status() {
    log_info "检查系统状态..."
    
    # 磁盘空间检查
    local disk_usage=$(df . | tail -1 | awk '{print $5}' | sed 's/%//')
    if [ "$disk_usage" -gt 90 ]; then
        log_warning "磁盘空间不足: ${disk_usage}%"
        echo "开始紧急清理..."
    else
        log_info "磁盘使用率: ${disk_usage}%"
    fi
    
    # 内存检查
    local mem_usage=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100.0}')
    if [ "$mem_usage" -gt 90 ]; then
        log_warning "内存使用率过高: ${mem_usage}%"
    else
        log_info "内存使用率: ${mem_usage}%"
    fi
}

# 清理构建文件
cleanup_build_files() {
    log_info "清理构建文件..."
    local cleaned_count=0
    
    # Python缓存文件
    if [ -n "$(find . -name "*.pyc" -type f 2>/dev/null)" ]; then
        find . -name "*.pyc" -delete
        cleaned_count=$((cleaned_count + $(find . -name "*.pyc" -type f 2>/dev/null | wc -l)))
    fi
    
    # __pycache__ 目录
    if [ -n "$(find . -name "__pycache__" -type d 2>/dev/null)" ]; then
        find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
        cleaned_count=$((cleaned_count + 1))
    fi
    
    # pytest 缓存
    if [ -n "$(find . -name ".pytest_cache" -type d 2>/dev/null)" ]; then
        find . -name ".pytest_cache" -type d -exec rm -rf {} + 2>/dev/null || true
        cleaned_count=$((cleaned_count + 1))
    fi
    
    # ROS2构建缓存 (但保留install目录)
    if [ -d "cyclonedds_ws/build" ]; then
        local build_size=$(du -sh cyclonedds_ws/build 2>/dev/null | cut -f1)
        log_info "ROS2构建缓存大小: $build_size"
        # 只清理超过1GB的构建缓存
        local build_size_mb=$(du -sm cyclonedds_ws/build 2>/dev/null | cut -f1)
        if [ "$build_size_mb" -gt 1024 ]; then
            log_warning "构建缓存过大(${build_size}MB)，建议手动清理"
        fi
    fi
    
    log_success "构建文件清理完成 (清理了 $cleaned_count 项)"
}

# 清理编辑器临时文件
cleanup_editor_temp() {
    log_info "清理编辑器临时文件..."
    local cleaned_count=0
    
    # Vim/Emacs临时文件
    for pattern in "*~" ".#*" "#*#" ".*.swp" ".*.swo"; do
        if [ -n "$(find . -name "$pattern" -type f 2>/dev/null)" ]; then
            local count=$(find . -name "$pattern" -type f -delete -print 2>/dev/null | wc -l)
            cleaned_count=$((cleaned_count + count))
        fi
    done
    
    # VS Code临时文件
    if [ -d ".vscode" ]; then
        find .vscode -name "*.log" -mtime +7 -delete 2>/dev/null || true
    fi
    
    log_success "编辑器临时文件清理完成 (清理了 $cleaned_count 个文件)"
}

# 清理日志文件
cleanup_logs() {
    log_info "清理日志文件..."
    
    if [ ! -d "logs" ]; then
        log_info "日志目录不存在，跳过日志清理"
        return 0
    fi
    
    local total_logs=$(find logs/ -name "*.log" 2>/dev/null | wc -l)
    local old_logs=0
    local compressed_logs=0
    
    # 删除超过30天的日志
    if [ -n "$(find logs/ -name "*.log" -mtime +30 2>/dev/null)" ]; then
        old_logs=$(find logs/ -name "*.log" -mtime +30 -delete -print 2>/dev/null | wc -l)
    fi
    
    # 压缩7天前的日志
    if [ -n "$(find logs/ -name "*.log" -mtime +7 2>/dev/null)" ]; then
        find logs/ -name "*.log" -mtime +7 -exec gzip {} \; 2>/dev/null || true
        compressed_logs=$(find logs/ -name "*.log.gz" -mtime +7 2>/dev/null | wc -l)
    fi
    
    # 显示日志目录大小
    if [ -d "logs" ]; then
        local logs_size=$(du -sh logs/ 2>/dev/null | cut -f1)
        log_info "日志目录大小: $logs_size"
    fi
    
    log_success "日志清理完成 (总日志: $total_logs, 删除旧日志: $old_logs, 压缩日志: $compressed_logs)"
}

# 清理临时下载文件
cleanup_downloads() {
    log_info "清理临时下载文件..."
    
    if [ ! -d "tmp" ]; then
        log_info "临时目录不存在，跳过下载文件清理"
        return 0
    fi
    
    local cleaned_count=0
    
    # 清理失败的下载
    if [ -n "$(find tmp/ -name "*.tmp" 2>/dev/null)" ]; then
        cleaned_count=$(find tmp/ -name "*.tmp" -delete -print 2>/dev/null | wc -l)
    fi
    
    # 清理空文件
    if [ -n "$(find tmp/ -size 0 2>/dev/null)" ]; then
        local empty_count=$(find tmp/ -size 0 -delete -print 2>/dev/null | wc -l)
        cleaned_count=$((cleaned_count + empty_count))
    fi
    
    # 清理超过7天的下载文件
    if [ -n "$(find tmp/downloads/ -type f -mtime +7 2>/dev/null)" ]; then
        local old_downloads=$(find tmp/downloads/ -type f -mtime +7 -delete -print 2>/dev/null | wc -l)
        cleaned_count=$((cleaned_count + old_downloads))
    fi
    
    log_success "下载文件清理完成 (清理了 $cleaned_count 个文件)"
}

# 清理空目录
cleanup_empty_dirs() {
    log_info "清理空目录..."
    
    # 清理空目录，但排除.git目录
    local empty_dirs=$(find . -type d -empty -not -path "./.git*" 2>/dev/null | wc -l)
    if [ "$empty_dirs" -gt 0 ]; then
        find . -type d -empty -not -path "./.git*" -delete 2>/dev/null || true
        log_success "清理了 $empty_dirs 个空目录"
    else
        log_info "没有发现空目录"
    fi
}

# Git状态检查
check_git_status() {
    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        log_info "不在Git仓库中，跳过Git状态检查"
        return 0
    fi
    
    log_info "检查Git状态..."
    
    # 检查未跟踪文件
    local untracked=$(git ls-files --others --exclude-standard | wc -l)
    if [ "$untracked" -gt 0 ]; then
        log_warning "发现 $untracked 个未跟踪文件"
        echo "未跟踪文件列表："
        git ls-files --others --exclude-standard | head -10
        if [ "$untracked" -gt 10 ]; then
            echo "... (还有 $((untracked - 10)) 个文件)"
        fi
    fi
    
    # 检查未提交更改
    if ! git diff --quiet; then
        log_warning "有未提交的更改"
    else
        log_info "工作目录干净"
    fi
}

# 主清理函数
main_cleanup() {
    local start_time=$(date '+%s')
    local cleanup_date=$(date '+%Y-%m-%d %H:%M:%S %Z')
    
    echo "=================================================="
    echo "🧹 Claudia机器人项目日常清理"
    echo "开始时间: $cleanup_date"
    echo "=================================================="
    
    # 系统状态检查
    check_system_status
    
    echo ""
    echo "🔄 开始清理操作..."
    
    # 执行各种清理
    cleanup_build_files
    cleanup_editor_temp
    cleanup_logs
    cleanup_downloads
    cleanup_empty_dirs
    
    echo ""
    echo "🔍 最终状态检查..."
    
    # Git状态检查
    check_git_status
    
    # 最终系统状态
    check_system_status
    
    # 计算耗时
    local end_time=$(date '+%s')
    local duration=$((end_time - start_time))
    
    echo ""
    echo "=================================================="
    echo "✅ 清理完成!"
    echo "结束时间: $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo "总耗时: ${duration}秒"
    echo "=================================================="
    
    # 记录到清理日志
    mkdir -p logs/maintenance
    echo "$(date '+%Y-%m-%d %H:%M:%S') | daily_cleanup | ${duration}s | success" >> logs/maintenance/cleanup.log
}

# 错误处理
handle_error() {
    local exit_code=$?
    local line_number=$1
    
    log_error "清理脚本在第 $line_number 行出错 (退出码: $exit_code)"
    
    # 记录错误
    mkdir -p logs/maintenance
    echo "$(date '+%Y-%m-%d %H:%M:%S') | daily_cleanup | error | line:$line_number | exit:$exit_code" >> logs/maintenance/cleanup.log
    
    exit $exit_code
}

# 设置错误处理
trap 'handle_error $LINENO' ERR

# 帮助信息
show_help() {
    echo "Claudia机器人项目日常清理脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help     显示帮助信息"
    echo "  -q, --quiet    静默模式，只显示重要信息"
    echo "  -v, --verbose  详细模式，显示更多信息"
    echo ""
    echo "示例:"
    echo "  $0              # 正常清理"
    echo "  $0 --quiet      # 静默清理"
    echo "  $0 --verbose    # 详细清理"
}

# 参数处理
QUIET_MODE=false
VERBOSE_MODE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -q|--quiet)
            QUIET_MODE=true
            shift
            ;;
        -v|--verbose)
            VERBOSE_MODE=true
            shift
            ;;
        *)
            log_error "未知参数: $1"
            echo "使用 $0 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 根据模式调整输出
if [ "$QUIET_MODE" = true ]; then
    # 静默模式：重定向大部分输出
    main_cleanup 2>&1 | grep -E "(SUCCESS|ERROR|WARNING|清理完成)" || true
elif [ "$VERBOSE_MODE" = true ]; then
    # 详细模式：显示调试信息
    set -x
    main_cleanup
else
    # 正常模式
    main_cleanup
fi 