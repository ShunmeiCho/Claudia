#!/bin/bash

# 1.6
# Generated: 2025-06-26
# Purpose: Validate Tool Chain Integration (Cursor MCP + TaskMaster + ROS2)
# Platform: aarch64 Ubuntu 20.04.5 LTS
# Note: Serena tool validation removed as per project decision

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory and paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$PROJECT_ROOT/logs/$(date '+%Y%m')"
LOG_FILE="$LOG_DIR/$(date '+%Y%m%d_%H%M%S')_toolchain_integration.log"

# Ensure log directory exists
mkdir -p "$LOG_DIR"

# Logging function
log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${timestamp} [$level] $message" | tee -a "$LOG_FILE"
}

print_header() {
    echo -e "${BLUE}================================================${NC}"
    echo -e "${BLUE}    Tool Chain Integration Validation${NC}"
    echo -e "${BLUE}    Cursor MCP + TaskMaster + ROS2${NC}"
    echo -e "${BLUE}    Generated: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${BLUE}================================================${NC}"
    log "INFO" "Starting Tool Chain Integration Validation"
}

print_status() {
    local status="$1"
    local message="$2"
    case "$status" in
        "PASS")
            echo -e "${GREEN}✅ $message${NC}"
            log "PASS" "$message"
            ;;
        "FAIL")
            echo -e "${RED}❌ $message${NC}"
            log "FAIL" "$message"
            ;;
        "WARN")
            echo -e "${YELLOW}⚠️  $message${NC}"
            log "WARN" "$message"
            ;;
        "INFO")
            echo -e "${BLUE}ℹ️  $message${NC}"
            log "INFO" "$message"
            ;;
    esac
}

# Test counter
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0

run_test() {
    local test_name="$1"
    local test_command="$2"
    
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    echo -e "\n${BLUE}--- Test $TESTS_TOTAL: $test_name ---${NC}"
    
    if eval "$test_command"; then
        print_status "PASS" "$test_name"
        TESTS_PASSED=$((TESTS_PASSED + 1))
        return 0
    else
        print_status "FAIL" "$test_name"
        TESTS_FAILED=$((TESTS_FAILED + 1))
        return 1
    fi
}

# Test functions
test_project_structure() {
    # Verify Claudia project structure
    local required_dirs=(
        "$PROJECT_ROOT/.taskmaster"
        "$PROJECT_ROOT/.cursor" 
        "$PROJECT_ROOT/cyclonedds_ws"
        "$PROJECT_ROOT/src/claudia"
        "$PROJECT_ROOT/scripts"
        "$PROJECT_ROOT/config"
    )
    
    for dir in "${required_dirs[@]}"; do
        if [[ ! -d "$dir" ]]; then
            print_status "FAIL" "Missing required directory: $dir"
            return 1
        fi
    done
    
    return 0
}

test_cursor_mcp_configuration() {
    # Check .cursor/mcp.json configuration
    local mcp_config="$PROJECT_ROOT/.cursor/mcp.json"
    
    if [[ ! -f "$mcp_config" ]]; then
        return 1
    fi
    
    # Check for required MCP servers
    if ! grep -q "taskmaster-ai" "$mcp_config" 2>/dev/null; then
        print_status "WARN" "TaskMaster MCP server not found in configuration"
        return 1
    fi
    
    if ! grep -q "desktop-commander" "$mcp_config" 2>/dev/null; then
        print_status "WARN" "Desktop Commander MCP server not found in configuration"
        return 1
    fi
    
    return 0
}

test_taskmaster_functionality() {
    # Test TaskMaster installation and basic functionality
    if ! command -v npx >/dev/null 2>&1; then
        return 1
    fi
    
    # Check if .taskmaster directory structure exists
    if [[ ! -d "$PROJECT_ROOT/.taskmaster" ]]; then
        return 1
    fi
    
    # Check for tasks.json
    if [[ ! -f "$PROJECT_ROOT/.taskmaster/tasks/tasks.json" ]]; then
        return 1
    fi
    
    # Verify tasks.json is valid JSON
    if ! python3 -m json.tool "$PROJECT_ROOT/.taskmaster/tasks/tasks.json" >/dev/null 2>&1; then
        return 1
    fi
    
    return 0
}

test_ros2_integration() {
    # Test ROS2 environment with project structure
    if ! source /opt/ros/foxy/setup.bash 2>/dev/null; then
        return 1
    fi
    
    if [[ -f "$PROJECT_ROOT/cyclonedds_ws/install/setup.bash" ]]; then
        if ! source "$PROJECT_ROOT/cyclonedds_ws/install/setup.bash" 2>/dev/null; then
            return 1
        fi
    fi
    
    # Check environment variables are set
    if [[ -z "$ROS_DISTRO" ]] || [[ "$ROS_DISTRO" != "foxy" ]]; then
        return 1
    fi
    
    if [[ -z "$ROS_VERSION" ]] || [[ "$ROS_VERSION" != "2" ]]; then
        return 1
    fi
    
    # Test basic ROS2 functionality
    if ! ros2 --help >/dev/null 2>&1; then
        return 1
    fi
    
    # Check if ROS2 can list topics
    if ! timeout 10s ros2 topic list >/dev/null 2>&1; then
        return 1
    fi
    
    return 0
}

test_claudia_python_integration() {
    # Test Claudia Python package integration
    cd "$PROJECT_ROOT"
    
    # Test if Python can import Claudia modules
    if ! python3 -c "import sys; sys.path.append('src'); import claudia" 2>/dev/null; then
        return 1
    fi
    
    # Test configuration loading
    if ! python3 -c "
import sys; sys.path.append('src')
from claudia.common.config import Config
try:
    config = Config()
    print('Config loading successful')
except Exception as e:
    print(f'Config loading failed: {e}')
    exit(1)
" 2>/dev/null; then
        return 1
    fi
    
    return 0
}

test_environment_variables() {
    # Check important environment variables
    local ros_vars_ok=true
    
    # Source environments
    source /opt/ros/foxy/setup.bash 2>/dev/null
    if [[ -f "$PROJECT_ROOT/cyclonedds_ws/install/setup.bash" ]]; then
        source "$PROJECT_ROOT/cyclonedds_ws/install/setup.bash" 2>/dev/null
    fi
    
    if [[ -z "$ROS_DISTRO" ]] || [[ "$ROS_DISTRO" != "foxy" ]]; then
        ros_vars_ok=false
    fi
    
    if [[ -z "$ROS_VERSION" ]] || [[ "$ROS_VERSION" != "2" ]]; then
        ros_vars_ok=false
    fi
    
    if [[ "$ros_vars_ok" == "false" ]]; then
        return 1
    fi
    
    return 0
}

test_workspace_functionality() {
    # Test cyclonedds_ws workspace functionality
    if [[ ! -d "$PROJECT_ROOT/cyclonedds_ws" ]]; then
        return 1
    fi
    
    cd "$PROJECT_ROOT/cyclonedds_ws"
    
    # Check if workspace is built
    if [[ ! -d "install" ]] || [[ ! -d "build" ]]; then
        return 1
    fi
    
    # Source workspace
    source /opt/ros/foxy/setup.bash 2>/dev/null
    source install/setup.bash 2>/dev/null
    
    # Check if Unitree packages are available
    if ! ros2 pkg list | grep -q unitree 2>/dev/null; then
        return 1
    fi
    
    return 0
}

test_mcp_tool_availability() {
    # Test if MCP tools are working (basic check)
    # This would normally be tested within Cursor, but we can check configuration
    
    if [[ ! -f "$PROJECT_ROOT/.cursor/mcp.json" ]]; then
        return 1
    fi
    
    # Check if required API keys are configured (without exposing them)
    local mcp_config="$PROJECT_ROOT/.cursor/mcp.json"
    
    if ! grep -q "ANTHROPIC_API_KEY" "$mcp_config" 2>/dev/null; then
        print_status "WARN" "ANTHROPIC_API_KEY not found in MCP configuration"
    fi
    
    if ! grep -q "taskmaster-ai" "$mcp_config" 2>/dev/null; then
        return 1
    fi
    
    return 0
}

test_development_workflow() {
    # Test if the development workflow components are in place
    local workflow_files=(
        "$PROJECT_ROOT/scripts/setup/verify_environment.sh"
        "$PROJECT_ROOT/scripts/setup/validate_ros2.sh"
        "$PROJECT_ROOT/scripts/setup/setup_cyclonedds_workspace.sh"
        "$PROJECT_ROOT/scripts/setup/setup_ros2_environment.sh"
        "$PROJECT_ROOT/scripts/setup/test_dds_communication.sh"
    )
    
    for file in "${workflow_files[@]}"; do
        if [[ ! -x "$file" ]]; then
            print_status "WARN" "Missing or non-executable workflow file: $file"
            return 1
        fi
    done
    
    return 0
}

# Main test execution
main() {
    print_header
    
    # System information
    print_status "INFO" "Platform: $(uname -a)"
    print_status "INFO" "ROS2 Distro: ${ROS_DISTRO:-'Not set'}"
    print_status "INFO" "Python: $(python3 --version)"
    print_status "INFO" "Project Root: $PROJECT_ROOT"
    print_status "INFO" "Tool Chain: Cursor MCP + TaskMaster (Serena excluded)"
    
    # Run tests
    run_test "Project Structure Validation" "test_project_structure"
    run_test "Cursor MCP Configuration" "test_cursor_mcp_configuration"
    run_test "TaskMaster Functionality" "test_taskmaster_functionality"
    run_test "ROS2 Integration" "test_ros2_integration"
    run_test "Claudia Python Integration" "test_claudia_python_integration"
    run_test "Environment Variables" "test_environment_variables"
    run_test "Workspace Functionality" "test_workspace_functionality"
    run_test "MCP Tool Availability" "test_mcp_tool_availability"
    run_test "Development Workflow" "test_development_workflow"
    
    # Test summary
    echo -e "\n${BLUE}================================================${NC}"
    echo -e "${BLUE}           Test Summary${NC}"
    echo -e "${BLUE}================================================${NC}"
    print_status "INFO" "Total Tests: $TESTS_TOTAL"
    print_status "INFO" "Passed: $TESTS_PASSED"
    print_status "INFO" "Failed: $TESTS_FAILED"
    
    local success_rate=$(( TESTS_PASSED * 100 / TESTS_TOTAL ))
    print_status "INFO" "Success Rate: $success_rate%"
    
    if [[ $TESTS_FAILED -eq 0 ]]; then
        print_status "PASS" "All tool chain integration tests completed successfully!"
        echo -e "\n${GREEN}🎉 Tool Chain Integration is ready for Claudia development!${NC}"
        log "PASS" "All tool chain integration tests completed successfully"
        return 0
    else
        print_status "FAIL" "Some tests failed. Check the logs for details."
        echo -e "\n${RED}❌ Tool Chain Integration needs attention before proceeding.${NC}"
        log "FAIL" "Some tool chain integration tests failed"
        return 1
    fi
}

# Cleanup function
cleanup_on_exit() {
    local exit_code=$?
    log "INFO" "Tool Chain Integration Test completed with exit code: $exit_code"
    exit $exit_code
}

# Set up cleanup trap
trap cleanup_on_exit EXIT

# Run main function
main "$@"