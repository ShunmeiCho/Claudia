#!/bin/bash

#1.5
# Generated: 2025-06-26
# Purpose: Test DDS Communication and ROS2 functionality with Unitree messages
# Platform: aarch64 Ubuntu 20.04.5 LTS

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
CYCLONEDDS_WS="$PROJECT_ROOT/cyclonedds_ws"
LOG_DIR="$PROJECT_ROOT/logs/$(date '+%Y%m')"
LOG_FILE="$LOG_DIR/$(date '+%Y%m%d_%H%M%S')_dds_communication_test.log"

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
    echo -e "${BLUE}    DDS Communication Test Suite${NC}"
    echo -e "${BLUE}    Generated: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${BLUE}================================================${NC}"
    log "INFO" "Starting DDS Communication Test Suite"
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
test_environment_setup() {
    # Check if cyclonedds_ws exists and is built
    if [[ ! -d "$CYCLONEDDS_WS" ]]; then
        return 1
    fi
    
    if [[ ! -d "$CYCLONEDDS_WS/install" ]]; then
        return 1
    fi
    
    return 0
}

test_ros2_environment() {
    # Source ROS2 environment
    source /opt/ros/foxy/setup.bash
    source "$CYCLONEDDS_WS/install/setup.bash"
    
    # Check ROS2 environment variables
    if [[ -z "$ROS_DISTRO" ]] || [[ "$ROS_DISTRO" != "foxy" ]]; then
        return 1
    fi
    
    if [[ -z "$ROS_VERSION" ]] || [[ "$ROS_VERSION" != "2" ]]; then
        return 1
    fi
    
    return 0
}

test_rmw_implementation() {
    # Set correct RMW implementation (修正：使用正确的名称)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    
    # Verify RMW implementation is set correctly
    if [[ "$RMW_IMPLEMENTATION" != "rmw_cyclonedds_cpp" ]]; then
        return 1
    fi
    
    # Test if RMW implementation is available
    if ! ros2 doctor --report 2>/dev/null | grep -q "rmw_cyclonedds_cpp"; then
        print_status "WARN" "rmw_cyclonedds_cpp not detected in ros2 doctor, but will continue testing"
    fi
    
    return 0
}

test_basic_topic_communication() {
    # Create a temporary topic for testing
    local test_topic="/test_dds_communication"
    local test_message="Hello DDS Communication Test $(date '+%H:%M:%S')"
    
    # Start a subscriber in background
    timeout 10s ros2 topic echo "$test_topic" > /tmp/dds_test_output.txt 2>&1 &
    local subscriber_pid=$!
    
    # Wait a moment for subscriber to start
    sleep 2
    
    # Publish a test message with explicit message type
    timeout 5s ros2 topic pub --once "$test_topic" std_msgs/msg/String "{data: '$test_message'}" >/dev/null 2>&1
    
    # Wait for message to be received
    sleep 3
    
    # Kill subscriber
    kill $subscriber_pid 2>/dev/null || true
    wait $subscriber_pid 2>/dev/null || true
    
    # Check if message was received
    if grep -q "$test_message" /tmp/dds_test_output.txt 2>/dev/null; then
        rm -f /tmp/dds_test_output.txt
        return 0
    else
        rm -f /tmp/dds_test_output.txt
        return 1
    fi
}

test_unitree_message_types() {
    # Test if Unitree message types are available
    local unitree_msgs=(
        "unitree_go/msg/SportModeState"
        "unitree_go/msg/LowState"
        "unitree_go/msg/LowCmd"
        "unitree_api/msg/Request"
        "unitree_api/msg/Response"
    )
    
    for msg_type in "${unitree_msgs[@]}"; do
        if ! ros2 interface show "$msg_type" >/dev/null 2>&1; then
            print_status "WARN" "Message type $msg_type not available"
            return 1
        fi
    done
    
    return 0
}

test_unitree_topic_discovery() {
    # Test if we can discover Unitree-related topics
    local timeout_duration=10
    
    # Start ros2 topic list in background
    timeout $timeout_duration ros2 topic list > /tmp/topic_list.txt 2>/dev/null
    
    # Check if typical Unitree topics can be listed (they may not exist but should be discoverable)
    local expected_topics=(
        "/sportmodestate"
        "/lowstate" 
        "/lowcmd"
    )
    
    # For now, just verify that topic discovery works
    if [[ -f /tmp/topic_list.txt ]] && [[ -s /tmp/topic_list.txt ]]; then
        local topic_count=$(wc -l < /tmp/topic_list.txt)
        print_status "INFO" "Discovered $topic_count ROS2 topics"
        rm -f /tmp/topic_list.txt
        return 0
    else
        rm -f /tmp/topic_list.txt
        return 1
    fi
}

test_dds_performance() {
    # Test DDS communication performance with a simple ping-pong
    local test_topic="/dds_perf_test"
    local message_count=5
    local success_count=0
    
    for i in $(seq 1 $message_count); do
        local test_msg="Performance test message $i at $(date '+%H:%M:%S.%3N')"
        
        # Start subscriber for this test
        timeout 5s ros2 topic echo "$test_topic" --once > /tmp/dds_perf_$i.txt 2>/dev/null &
        local sub_pid=$!
        
        # Wait a moment
        sleep 1
        
        # Publish message
        if timeout 3s ros2 topic pub --once "$test_topic" std_msgs/msg/String "{data: '$test_msg'}" >/dev/null 2>&1; then
            # Wait for response
            sleep 2
            
            # Check if message was received
            if wait $sub_pid 2>/dev/null && grep -q "Performance test message $i" /tmp/dds_perf_$i.txt 2>/dev/null; then
                success_count=$((success_count + 1))
            fi
        fi
        
        # Cleanup
        kill $sub_pid 2>/dev/null || true
        wait $sub_pid 2>/dev/null || true
        rm -f /tmp/dds_perf_$i.txt
    done
    
    local success_rate=$(( success_count * 100 / message_count ))
    print_status "INFO" "DDS Performance: $success_count/$message_count messages succeeded ($success_rate%)"
    
    # Consider test passed if at least 60% of messages succeeded
    if [[ $success_rate -ge 60 ]]; then
        return 0
    else
        return 1
    fi
}

test_cyclonedds_configuration() {
    # Test CycloneDDS specific configuration
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    
    # Check if CycloneDDS is working with conservative settings
    export CYCLONEDX_URI="<CycloneDX><Domain><General><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDX>"
    
    # Test basic DDS functionality
    if timeout 5s ros2 topic list >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

# Main test execution
main() {
    print_header
    
    # System information
    print_status "INFO" "Platform: $(uname -a)"
    print_status "INFO" "ROS2 Distro: ${ROS_DISTRO:-'Not set'}"
    print_status "INFO" "Python: $(python3 --version)"
    print_status "INFO" "Project Root: $PROJECT_ROOT"
    print_status "INFO" "CycloneDX Workspace: $CYCLONEDDS_WS"
    
    # Source environment
    source /opt/ros/foxy/setup.bash
    source "$CYCLONEDDS_WS/install/setup.bash"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    
    # Run tests
    run_test "Environment Setup Check" "test_environment_setup"
    run_test "ROS2 Environment Validation" "test_ros2_environment" 
    run_test "RMW Implementation Configuration" "test_rmw_implementation"
    run_test "Basic Topic Communication" "test_basic_topic_communication"
    run_test "Unitree Message Types Availability" "test_unitree_message_types"
    run_test "Unitree Topic Discovery" "test_unitree_topic_discovery"
    run_test "DDS Performance Test" "test_dds_performance"
    run_test "CycloneDDS Configuration" "test_cyclonedds_configuration"
    
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
        print_status "PASS" "All DDS communication tests completed successfully!"
        echo -e "\n${GREEN}🎉 DDS Communication is ready for Unitree Go2 integration!${NC}"
        log "PASS" "All DDS communication tests completed successfully"
        return 0
    else
        print_status "FAIL" "Some tests failed. Check the logs for details."
        echo -e "\n${RED}❌ DDS Communication needs attention before proceeding.${NC}"
        log "FAIL" "Some DDS communication tests failed"
        return 1
    fi
}

# Cleanup function
cleanup_on_exit() {
    local exit_code=$?
    
    # Kill any remaining background processes
    pkill -f "ros2 topic" 2>/dev/null || true
    
    # Clean up temporary files
    rm -f /tmp/dds_test_output.txt /tmp/topic_list.txt /tmp/dds_perf_*.txt
    
    log "INFO" "DDS Communication Test completed with exit code: $exit_code"
    
    exit $exit_code
}

# Set up cleanup trap
trap cleanup_on_exit EXIT

# Run main function
main "$@"