#!/bin/bash

# Claudia Robot AI Assistant - Quick Start Script
# Created: $(date '+%Y-%m-%d %H:%M:%S')
# Purpose: 便捷启动和测试Claudia专属AI模型

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示横幅
show_banner() {
    echo -e "${BLUE}"
    echo "================================================="
    echo "    クラウディア (Claudia) Robot AI Assistant    "
    echo "================================================="
    echo "   Japanese Robotics Environment AI Assistant    "
    echo "   Optimized for ROS2 Foxy + Jetson Orin NX     "
    echo "================================================="
    echo -e "${NC}"
}

# 检查Ollama服务状态
check_ollama_service() {
    print_status "Checking Ollama service status..."
    
    if ! command -v ollama &> /dev/null; then
        print_error "Ollama not found! Please install Ollama first."
        exit 1
    fi
    
    if ! pgrep -x "ollama" > /dev/null; then
        print_warning "Ollama service not running. Starting service..."
        ollama serve &
        sleep 3
    fi
    
    print_success "Ollama service is running"
}

# 检查Claudia模型是否存在
check_claudia_model() {
    print_status "Checking Claudia model availability..."
    
    if ollama list | grep -q "claudia-optimized"; then
        print_success "Claudia model found: claudia-optimized"
    else
        print_error "Claudia model not found!"
        print_status "Please create the model first using:"
        echo "ollama create claudia-optimized -f ClaudiaOptimizedModelfile"
        exit 1
    fi
}

# 运行快速测试
run_quick_tests() {
    print_status "Running quick functionality tests..."
    
    echo -e "\n${YELLOW}Testing Control Mode:${NC}"
    echo "Input: 前に進む"
    echo "Output:" 
    echo "前に進む" | ollama run claudia-optimized
    
    echo -e "\n${YELLOW}Testing LED Control:${NC}"
    echo "Input: LED点灯"
    echo "Output:"
    echo "LED点灯" | ollama run claudia-optimized
    
    echo -e "\n${YELLOW}Testing Emergency Mode:${NC}"
    echo "Input: 緊急停止"
    echo "Output:"
    echo "緊急停止" | ollama run claudia-optimized
    
    print_success "All tests completed successfully!"
}

# 交互模式
interactive_mode() {
    print_status "Starting interactive mode..."
    print_status "Type 'exit' or 'quit' to end session"
    print_status "Type 'test' to run quick tests"
    echo ""
    
    while true; do
        echo -n -e "${GREEN}Claudia> ${NC}"
        read -r user_input
        
        case $user_input in
            "exit"|"quit")
                print_success "Session ended. Goodbye!"
                break
                ;;
            "test")
                run_quick_tests
                ;;
            "")
                continue
                ;;
            *)
                echo -e "${BLUE}Claudia:${NC}"
                echo "$user_input" | ollama run claudia-optimized
                echo ""
                ;;
        esac
    done
}

# 显示使用帮助
show_help() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  -h, --help      Show this help message"
    echo "  -t, --test      Run quick functionality tests"
    echo "  -i, --interactive Enter interactive chat mode"
    echo "  -s, --status    Show system status"
    echo "  -v, --version   Show model information"
    echo ""
    echo "Examples:"
    echo "  $0 -t           # Run tests"
    echo "  $0 -i           # Start interactive session"
    echo "  echo '前に進む' | $0   # Quick command"
}

# 显示系统状态
show_status() {
    print_status "System Status Report"
    echo "===================="
    
    echo "Date: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "Ollama Service: $(pgrep -x ollama >/dev/null && echo 'Running' || echo 'Stopped')"
    echo "Available Models:"
    ollama list | grep -E "(claudia|qwen)"
    echo ""
    echo "Memory Usage:"
    free -h | head -2
    echo ""
    echo "CPU Usage:"
    top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1"%"}'
}

# 显示模型版本信息
show_version() {
    print_status "Claudia Model Information"
    echo "========================="
    ollama show claudia-optimized | head -20
}

# 主函数
main() {
    show_banner
    
    # 解析命令行参数
    case "${1:-}" in
        -h|--help)
            show_help
            exit 0
            ;;
        -t|--test)
            check_ollama_service
            check_claudia_model
            run_quick_tests
            exit 0
            ;;
        -i|--interactive)
            check_ollama_service
            check_claudia_model
            interactive_mode
            exit 0
            ;;
        -s|--status)
            show_status
            exit 0
            ;;
        -v|--version)
            show_version
            exit 0
            ;;
        "")
            # 无参数时的默认行为
            check_ollama_service
            check_claudia_model
            
            # 如果有管道输入，直接处理
            if [ ! -t 0 ]; then
                while IFS= read -r line; do
                    echo "$line" | ollama run claudia-optimized
                done
            else
                # 否则进入交互模式
                interactive_mode
            fi
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
}

# 执行主函数
main "$@" 