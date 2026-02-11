#!/bin/bash
# Claudia LLM Deployment & Management Script
# 自动化部署和管理Claudia机器人的优化LLM模型
# Generated: $(date '+%Y-%m-%d %H:%M:%S')

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目路径
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODELFILE_PATH="$PROJECT_ROOT/ClaudiaOptimizedModelfile_v2_3B"
LOG_DIR="$PROJECT_ROOT/logs/llm"
TEST_SCRIPT="$PROJECT_ROOT/test/test_claudia_llm_performance.py"

# 创建日志目录
mkdir -p "$LOG_DIR"

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [INFO] $1" >> "$LOG_DIR/deployment.log"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [SUCCESS] $1" >> "$LOG_DIR/deployment.log"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [WARNING] $1" >> "$LOG_DIR/deployment.log"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [ERROR] $1" >> "$LOG_DIR/deployment.log"
}

# 检查依赖
check_dependencies() {
    log_info "检查依赖项..."
    
    if ! command -v ollama &> /dev/null; then
        log_error "Ollama未安装。请先安装Ollama。"
        exit 1
    fi
    
    if ! command -v python3 &> /dev/null; then
        log_error "Python3未安装。"
        exit 1
    fi
    
    if [ ! -f "$MODELFILE_PATH" ]; then
        log_error "模型文件不存在: $MODELFILE_PATH"
        exit 1
    fi
    
    log_success "依赖检查通过"
}

# 检查系统资源
check_system_resources() {
    log_info "检查系统资源..."
    
    # 检查内存
    local mem_total=$(free -m | awk '/^Mem:/{print $2}')
    local mem_available=$(free -m | awk '/^Mem:/{print $7}')
    
    if [ "$mem_available" -lt 2048 ]; then
        log_warning "可用内存不足2GB，可能影响模型性能"
    fi
    
    # 检查磁盘空间
    local disk_usage=$(df "$PROJECT_ROOT" | tail -1 | awk '{print $5}' | sed 's/%//')
    if [ "$disk_usage" -gt 90 ]; then
        log_warning "磁盘使用率过高: ${disk_usage}%"
    fi
    
    log_info "内存: ${mem_available}MB可用/${mem_total}MB总计"
    log_info "磁盘使用率: ${disk_usage}%"
}

# 部署模型
deploy_model() {
    local model_name="${1:-claudia-v3.2:3b}"
    
    log_info "部署模型: $model_name"
    
    # 检查是否已存在
    if ollama list | grep -q "$model_name"; then
        log_warning "模型 $model_name 已存在，将重新创建..."
        ollama rm "$model_name" 2>/dev/null || true
    fi
    
    # 创建模型
    log_info "创建模型中..."
    if ollama create "$model_name" -f "$MODELFILE_PATH"; then
        log_success "模型 $model_name 创建成功"
    else
        log_error "模型创建失败"
        exit 1
    fi
}

# 运行性能测试
run_performance_test() {
    local model_name="${1:-claudia-v3.2:3b}"
    
    log_info "运行性能测试: $model_name"
    
    if [ ! -f "$TEST_SCRIPT" ]; then
        log_error "测试脚本不存在: $TEST_SCRIPT"
        return 1
    fi
    
    # 运行测试并保存结果
    local test_output="$LOG_DIR/test_$(date '+%Y%m%d_%H%M%S').log"
    if python3 "$TEST_SCRIPT" --model "$model_name" > "$test_output" 2>&1; then
        log_success "性能测试完成，结果保存到: $test_output"
        
        # 提取准确率
        local accuracy=$(grep "总体准确率" "$test_output" | grep -o '[0-9.]*%' || echo "未知")
        log_info "模型准确率: $accuracy"
    else
        log_error "性能测试失败"
        return 1
    fi
}

# 快速测试
quick_test() {
    local model_name="${1:-claudia-v3.2:3b}"
    
    log_info "运行快速功能测试..."
    
    local test_commands=("座って" "停止" "ダンス" "バランス")
    local expected_outputs=("座ります" "緊急停止" "踊ります" "バランス調整")
    
    local passed=0
    local total=${#test_commands[@]}
    
    for i in "${!test_commands[@]}"; do
        local cmd="${test_commands[$i]}"
        local expected="${expected_outputs[$i]}"
        
        log_info "测试: '$cmd'"
        local output=$(timeout 10 ollama run "$model_name" "$cmd" 2>/dev/null | grep -v "^$" | tail -1)
        
        if [[ "$output" == *"$expected"* ]]; then
            log_success "✅ '$cmd' → '$output'"
            ((passed++))
        else
            log_error "❌ '$cmd' → '$output' (期望: $expected)"
        fi
    done
    
    local accuracy=$((passed * 100 / total))
    log_info "快速测试准确率: $accuracy% ($passed/$total)"
    
    if [ "$accuracy" -ge 75 ]; then
        log_success "快速测试通过"
        return 0
    else
        log_error "快速测试失败"
        return 1
    fi
}

# 监控模型性能
monitor_model() {
    local model_name="${1:-claudia-v3.2:3b}"
    local duration="${2:-60}"
    
    log_info "监控模型性能 $duration 秒..."
    
    local start_time=$(date +%s)
    local end_time=$((start_time + duration))
    local test_count=0
    local success_count=0
    
    while [ $(date +%s) -lt $end_time ]; do
        local test_cmd="座って"
        local expected="座ります"
        
        local start_response=$(date +%s.%3N)
        local output=$(ollama run "$model_name" "$test_cmd" 2>/dev/null | tail -1)
        local end_response=$(date +%s.%3N)
        
        local response_time=$(echo "$end_response - $start_response" | bc)
        
        ((test_count++))
        
        if [[ "$output" == *"$expected"* ]]; then
            ((success_count++))
            log_info "测试 $test_count: ✅ ${response_time}s"
        else
            log_warning "测试 $test_count: ❌ ${response_time}s"
        fi
        
        sleep 2
    done
    
    local success_rate=$((success_count * 100 / test_count))
    log_info "监控结果: $success_rate% 成功率 ($success_count/$test_count)"
}

# 列出可用模型
list_models() {
    log_info "可用的Claudia模型:"
    ollama list | grep -E "(claudia|kura)" || log_warning "未找到Claudia相关模型"
}

# 切换模型
switch_model() {
    local new_model="$1"
    
    if [ -z "$new_model" ]; then
        log_error "请指定要切换的模型名称"
        return 1
    fi
    
    if ! ollama list | grep -q "$new_model"; then
        log_error "模型 $new_model 不存在"
        return 1
    fi
    
    # 更新配置文件（如果存在）
    local config_file="$PROJECT_ROOT/.env"
    if [ -f "$config_file" ]; then
        if grep -q "CLAUDIA_MODEL" "$config_file"; then
            sed -i "s/CLAUDIA_MODEL=.*/CLAUDIA_MODEL=$new_model/" "$config_file"
        else
            echo "CLAUDIA_MODEL=$new_model" >> "$config_file"
        fi
        log_success "已切换到模型: $new_model"
    else
        log_warning "配置文件不存在，请手动配置模型: $new_model"
    fi
}

# 备份模型
backup_model() {
    local model_name="${1:-claudia-v3.2:3b}"
    local backup_name="${model_name}-backup-$(date '+%Y%m%d_%H%M%S')"
    
    log_info "备份模型 $model_name 为 $backup_name"
    
    # 创建备份（通过重新创建）
    if ollama create "$backup_name" -f "$MODELFILE_PATH"; then
        log_success "模型备份成功: $backup_name"
    else
        log_error "模型备份失败"
        return 1
    fi
}

# 清理旧模型
cleanup_old_models() {
    log_info "清理旧的Claudia模型..."
    
    # 列出所有claudia模型（除了最新的）
    local models=$(ollama list | grep -E "claudia.*:3b" | awk '{print $1}' | head -n -1)
    
    if [ -z "$models" ]; then
        log_info "没有需要清理的旧模型"
        return 0
    fi
    
    for model in $models; do
        if [[ "$model" == *"backup"* ]]; then
            continue  # 保留备份模型
        fi
        
        log_warning "删除旧模型: $model"
        ollama rm "$model" 2>/dev/null || true
    done
    
    log_success "旧模型清理完成"
}

# 显示帮助
show_help() {
    cat << 'EOF'
Claudia LLM Deployment & Management Script

用法:
  ./deploy_claudia_llm.sh [命令] [选项]

命令:
  deploy [模型名]    - 部署指定模型 (默认: claudia-v3.2:3b)
  test [模型名]      - 运行完整性能测试  
  quick [模型名]     - 运行快速功能测试
  monitor [模型名] [秒数] - 监控模型性能
  list              - 列出所有可用模型
  switch [模型名]    - 切换当前使用的模型
  backup [模型名]    - 备份指定模型
  cleanup           - 清理旧模型
  full [模型名]      - 完整部署流程 (部署+测试+监控)

示例:
  ./deploy_claudia_llm.sh deploy              # 部署默认模型
  ./deploy_claudia_llm.sh test claudia-v3.2:3b # 测试指定模型
  ./deploy_claudia_llm.sh quick               # 快速测试
  ./deploy_claudia_llm.sh monitor claudia-v3.2:3b 120 # 监控2分钟
  ./deploy_claudia_llm.sh full                # 完整流程

日志位置: logs/llm/deployment.log
EOF
}

# 完整部署流程
full_deployment() {
    local model_name="${1:-claudia-v3.2:3b}"
    
    log_info "开始完整部署流程: $model_name"
    
    # 1. 检查环境
    check_dependencies
    check_system_resources
    
    # 2. 部署模型
    deploy_model "$model_name"
    
    # 3. 快速测试
    if quick_test "$model_name"; then
        log_success "快速测试通过，继续完整测试..."
        
        # 4. 完整性能测试
        run_performance_test "$model_name"
        
        # 5. 短期监控
        monitor_model "$model_name" 30
        
        log_success "完整部署流程完成！"
        log_info "模型 $model_name 已就绪，可以开始使用"
    else
        log_error "快速测试失败，请检查模型配置"
        exit 1
    fi
}

# 主函数
main() {
    local command="${1:-help}"
    shift || true
    
    case "$command" in
        deploy)
            check_dependencies
            deploy_model "$@"
            ;;
        test)
            run_performance_test "$@"
            ;;
        quick)
            quick_test "$@"
            ;;
        monitor)
            monitor_model "$@"
            ;;
        list)
            list_models
            ;;
        switch)
            switch_model "$@"
            ;;
        backup)
            backup_model "$@"
            ;;
        cleanup)
            cleanup_old_models
            ;;
        full)
            full_deployment "$@"
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            log_error "未知命令: $command"
            show_help
            exit 1
            ;;
    esac
}

# 执行主函数
main "$@" 