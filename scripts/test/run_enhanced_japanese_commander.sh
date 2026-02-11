#!/bin/bash
# Enhanced Interactive Japanese Commander Startup Script
# 增强版交互式日语指令界面启动脚本
# Generated: 2025-07-10 
# Purpose: 启动集成LLM的智能日语机器人控制界面

set -e

# 颜色定义
RED='\033[91m'
GREEN='\033[92m'
YELLOW='\033[93m'
BLUE='\033[94m'
PURPLE='\033[95m'
CYAN='\033[96m'
WHITE='\033[97m'
BOLD='\033[1m'
END='\033[0m'

SCRIPT_NAME="Enhanced Japanese Commander v3.2"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

echo -e "${BOLD}${CYAN}🤖 $SCRIPT_NAME 启动脚本${END}"
echo -e "${CYAN}时间: $TIMESTAMP${END}"
echo -e "${CYAN}======================================${END}"

# 预启动检查函数
pre_startup_check() {
    echo -e "\n${BLUE}📋 预启动检查...${END}"
    
    # 检查当前时间和系统状态
    echo -e "⏰ 当前时间: $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo -e "📁 当前目录: $(pwd)"
    echo -e "💾 磁盘使用: $(df . | tail -1 | awk '{print $5}')"
    echo -e "🧠 内存使用: $(free | grep Mem | awk '{printf "%.0f%%", $3/$2 * 100.0}')"
    
    # 检查项目根目录
    if [ ! -f "README.md" ] || [ ! -d "src/claudia" ]; then
        echo -e "${RED}❌ 错误: 请在项目根目录运行此脚本${END}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ 基础环境检查通过${END}"
}

# Python环境检查
check_python_env() {
    echo -e "\n${BLUE}🐍 Python环境检查...${END}"
    
    # 检查Python版本
    if ! command -v python3 &> /dev/null; then
        echo -e "${RED}❌ Python3未安装${END}"
        exit 1
    fi
    
    PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)
    echo -e "Python版本: $PYTHON_VERSION"
    
    # 检查必要的Python模块
    REQUIRED_MODULES=("asyncio" "requests" "json" "pathlib")
    for module in "${REQUIRED_MODULES[@]}"; do
        if python3 -c "import $module" 2>/dev/null; then
            echo -e "✅ $module"
        else
            echo -e "${RED}❌ $module 模块缺失${END}"
            exit 1
        fi
    done
    
    echo -e "${GREEN}✅ Python环境检查通过${END}"
}

# LLM服务检查
check_llm_service() {
    echo -e "\n${BLUE}🧠 LLM服务检查...${END}"
    
    # 检查Ollama服务
    if curl -s http://127.0.0.1:11434/api/tags > /dev/null 2>&1; then
        echo -e "${GREEN}✅ Ollama服务运行正常${END}"
        
        # 检查可用模型
        MODELS=$(curl -s http://127.0.0.1:11434/api/tags | python3 -c "
import json, sys
try:
    data = json.load(sys.stdin)
    models = [model['name'] for model in data.get('models', [])]
    print('可用模型:', ', '.join(models) if models else '无')
    
    # 检查是否有claudia-v3.2:3b模型
    claudia_models = [m for m in models if 'claudia-v3.2:3b' in m]
    if claudia_models:
        print('✅ Claudia优化模型已就绪:', ', '.join(claudia_models))
    else:
        print('⚠️ Claudia优化模型(claudia-v3.2:3b)未找到')
except:
    print('解析模型列表失败')
")
        echo -e "📋 $MODELS"
        
    else
        echo -e "${YELLOW}⚠️ Ollama服务未运行或不可达${END}"
        echo -e "${YELLOW}   LLM功能可能受限，但程序将继续启动${END}"
    fi
}

# CycloneDDS环境检查
check_cyclonedds_env() {
    echo -e "\n${BLUE}🔧 CycloneDDS环境检查...${END}"
    
    # 检查工作空间
    if [ -d "cyclonedds_ws" ]; then
        echo -e "✅ CycloneDDS工作空间存在"
        
        # 检查安装目录
        if [ -d "cyclonedds_ws/install" ]; then
            echo -e "✅ CycloneDDS已构建"
            
            # 设置环境变量
            echo -e "${CYAN}📡 设置CycloneDDS环境...${END}"
            source /opt/ros/foxy/setup.bash
            source cyclonedds_ws/install/setup.bash
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            
            echo -e "✅ RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
        else
            echo -e "${YELLOW}⚠️ CycloneDDS未构建，机器人控制功能可能受限${END}"
        fi
    else
        echo -e "${YELLOW}⚠️ CycloneDDS工作空间不存在${END}"
    fi
}

# 机器人SDK检查
check_robot_sdk() {
    echo -e "\n${BLUE}🤖 机器人SDK检查...${END}"
    
    if [ -d "unitree_sdk2_python" ]; then
        echo -e "✅ Unitree SDK2 Python存在"
        
        # 检查SDK导入
        if python3 -c "
import sys
sys.path.append('unitree_sdk2_python')
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    print('SDK导入成功')
except ImportError as e:
    print(f'SDK导入失败: {e}')
" 2>/dev/null | grep -q "SDK导入成功"; then
            echo -e "✅ SDK导入测试通过"
        else
            echo -e "${YELLOW}⚠️ SDK导入失败，真实机器人控制不可用${END}"
        fi
    else
        echo -e "${YELLOW}⚠️ Unitree SDK2未找到${END}"
    fi
}

# 项目文件检查
check_project_files() {
    echo -e "\n${BLUE}📁 项目文件检查...${END}"
    
    REQUIRED_FILES=(
        "src/claudia/interactive_japanese_commander_enhanced.py"
        "src/claudia/robot_controller/action_mapping_engine_real.py"
        "scripts/llm/claudia_llm_interface.py"
    )
    
    for file in "${REQUIRED_FILES[@]}"; do
        if [ -f "$file" ]; then
            echo -e "✅ $file"
        else
            echo -e "${RED}❌ $file 缺失${END}"
            exit 1
        fi
    done
    
    echo -e "${GREEN}✅ 核心文件检查通过${END}"
}

# 启动界面
start_interface() {
    echo -e "\n${BOLD}${GREEN}🚀 启动增强版日语指令界面...${END}"
    
    # 确保日志目录存在
    mkdir -p logs
    
    # 启动界面
    echo -e "${CYAN}正在启动...${END}"
    python3 src/claudia/interactive_japanese_commander_enhanced.py
}

# 清理函数
cleanup_on_exit() {
    echo -e "\n${YELLOW}🧹 清理退出...${END}"
    
    # 清理临时文件
    find /tmp -name "claudia*" -mtime +1 -delete 2>/dev/null || true
    
    echo -e "${GREEN}✅ 清理完成${END}"
}

# 错误处理
handle_error() {
    local exit_code=$?
    local line_number=$1
    
    echo -e "\n${RED}❌ 错误发生在第 $line_number 行${END}"
    echo -e "${RED}退出码: $exit_code${END}"
    echo -e "${RED}时间: $(date '+%Y-%m-%d %H:%M:%S')${END}"
    
    # 记录错误
    ERROR_LOG="logs/errors/$(date '+%Y%m%d_%H%M%S')_enhanced_japanese_commander_error.log"
    mkdir -p "$(dirname "$ERROR_LOG")"
    
    {
        echo "Enhanced Japanese Commander Error Report"
        echo "Time: $(date '+%Y-%m-%d %H:%M:%S %Z')"
        echo "Exit Code: $exit_code"
        echo "Line: $line_number"
        echo "Working Directory: $(pwd)"
        echo "Environment:"
        env | grep -E "(ROS|CYCLONE|OLLAMA|UNITREE)" || true
    } > "$ERROR_LOG"
    
    cleanup_on_exit
    exit $exit_code
}

# 设置错误处理
set -e
trap 'handle_error $LINENO' ERR
trap cleanup_on_exit EXIT

# 主执行流程
main() {
    echo -e "${BOLD}开始启动检查序列...${END}"
    
    # 执行所有检查
    pre_startup_check
    check_python_env
    check_llm_service
    check_cyclonedds_env
    check_robot_sdk
    check_project_files
    
    echo -e "\n${BOLD}${GREEN}🎉 所有检查完成，系统就绪！${END}"
    
    # 显示系统摘要
    echo -e "\n${BOLD}${PURPLE}📊 系统摘要${END}"
    echo -e "Python: $(python3 --version | cut -d' ' -f2)"
    echo -e "LLM服务: $(curl -s http://127.0.0.1:11434/api/tags > /dev/null 2>&1 && echo '🟢 运行中' || echo '🟡 离线')"
    echo -e "目标模型: claudia-v3.2:3b"
    echo -e "CycloneDDS: $([ -d cyclonedds_ws/install ] && echo '🟢 已构建' || echo '🟡 未构建')"
    echo -e "机器人SDK: $([ -d unitree_sdk2_python ] && echo '🟢 可用' || echo '🟡 不可用')"
    
    # 启动确认
    echo -e "\n${YELLOW}准备启动增强版日语指令界面...${END}"
    read -p "按Enter继续，或Ctrl+C取消: "
    
    # 启动
    start_interface
}

# 执行主函数
main

echo -e "\n${GREEN}✅ Enhanced Japanese Commander 启动脚本执行完成${END}" 