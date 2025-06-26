#!/bin/bash
# 任务2
# Generated: 2025-06-26
# Purpose: Install and Verify Core Unitree SDKs (unitree_sdk2, unitree_ros2)
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
LOG_FILE="$LOG_DIR/$(date '+%Y%m%d_%H%M%S')_unitree_sdk_install.log"

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
    echo -e "${BLUE}    Unitree SDKs Installation & Verification${NC}"
    echo -e "${BLUE}    unitree_sdk2 + unitree_ros2${NC}"
    echo -e "${BLUE}    Generated: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${BLUE}================================================${NC}"
    log "INFO" "Starting Unitree SDKs Installation & Verification"
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

# Function to check prerequisites
check_prerequisites() {
    print_status "INFO" "Checking Prerequisites..."
    
    # Check ROS2 environment
    if [[ -z "$ROS_DISTRO" ]]; then
        source /opt/ros/foxy/setup.bash
    fi
    
    if [[ "$ROS_DISTRO" != "foxy" ]]; then
        print_status "FAIL" "ROS2 Foxy not detected. Current: ${ROS_DISTRO:-'none'}"
        return 1
    fi
    
    # Check cyclonedds_ws exists
    if [[ ! -d "$CYCLONEDDS_WS" ]]; then
        print_status "FAIL" "cyclonedds_ws workspace not found at $CYCLONEDDS_WS"
        return 1
    fi
    
    # Check if workspace is built
    if [[ ! -d "$CYCLONEDDS_WS/install" ]]; then
        print_status "WARN" "cyclonedds_ws not built yet"
        return 1
    fi
    
    print_status "PASS" "Prerequisites check completed"
    return 0
}

# Function to install system dependencies
install_system_dependencies() {
    print_status "INFO" "Installing System Dependencies..."
    
    # List of required packages
    local packages=(
        "ros-foxy-rmw-cyclonedds-cpp"
        "ros-foxy-rosidl-generator-dds-idl"
        "ros-foxy-cyclonedds"
        "build-essential"
        "cmake"
        "git"
    )
    
    # Check and install missing packages
    local missing_packages=()
    for pkg in "${packages[@]}"; do
        if ! dpkg -l | grep -q "^ii.*$pkg "; then
            missing_packages+=("$pkg")
        fi
    done
    
    if [[ ${#missing_packages[@]} -gt 0 ]]; then
        print_status "INFO" "Installing missing packages: ${missing_packages[*]}"
        sudo apt update
        sudo apt install -y "${missing_packages[@]}"
    else
        print_status "PASS" "All system dependencies already installed"
    fi
    
    return 0
}

# Function to verify SDK repositories
verify_sdk_repositories() {
    print_status "INFO" "Verifying SDK Repositories..."
    
    cd "$CYCLONEDDS_WS/src"
    
    # Check unitree_sdk2
    if [[ ! -d "unitree_sdk2" ]]; then
        print_status "WARN" "unitree_sdk2 not found, cloning..."
        git clone https://github.com/unitreerobotics/unitree_sdk2.git
    else
        print_status "PASS" "unitree_sdk2 repository found"
    fi
    
    # Check unitree_ros2
    if [[ ! -d "unitree_ros2" ]]; then
        print_status "WARN" "unitree_ros2 not found, cloning..."
        git clone https://github.com/unitreerobotics/unitree_ros2.git
    else
        print_status "PASS" "unitree_ros2 repository found"
    fi
    
    return 0
}

# Function to build workspace
build_workspace() {
    print_status "INFO" "Building cyclonedds_ws workspace..."
    
    cd "$CYCLONEDDS_WS"
    
    # Source ROS2 environment
    source /opt/ros/foxy/setup.bash
    
    # Clean build if requested
    if [[ "$1" == "clean" ]]; then
        print_status "INFO" "Cleaning previous build..."
        rm -rf build/ install/ log/
    fi
    
    # Build workspace
    print_status "INFO" "Running colcon build..."
    local build_start=$(date +%s)
    
    if colcon build --symlink-install 2>&1 | tee -a "$LOG_FILE"; then
        local build_end=$(date +%s)
        local build_duration=$((build_end - build_start))
        print_status "PASS" "Workspace built successfully in ${build_duration} seconds"
    else
        print_status "FAIL" "Workspace build failed"
        return 1
    fi
    
    return 0
}

# Function to verify installation
verify_installation() {
    print_status "INFO" "Verifying Installation..."
    
    # Source environments
    cd "$CYCLONEDDS_WS"
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    
    # Check ROS2 packages
    print_status "INFO" "Checking installed ROS2 packages..."
    local unitree_packages=$(ros2 pkg list | grep unitree | wc -l)
    if [[ $unitree_packages -gt 0 ]]; then
        print_status "PASS" "Found $unitree_packages Unitree ROS2 packages"
        ros2 pkg list | grep unitree | while read pkg; do
            print_status "INFO" "  - $pkg"
        done
    else
        print_status "FAIL" "No Unitree ROS2 packages found"
        return 1
    fi
    
    # Check message interfaces
    print_status "INFO" "Checking Unitree message interfaces..."
    local unitree_interfaces=$(ros2 interface list | grep unitree | wc -l)
    if [[ $unitree_interfaces -gt 0 ]]; then
        print_status "PASS" "Found $unitree_interfaces Unitree message interfaces"
    else
        print_status "FAIL" "No Unitree message interfaces found"
        return 1
    fi
    
    # Check critical message types
    local critical_msgs=(
        "unitree_go/msg/LowCmd"
        "unitree_go/msg/LowState"
        "unitree_go/msg/SportModeState"
        "unitree_api/msg/Request"
        "unitree_api/msg/Response"
    )
    
    for msg in "${critical_msgs[@]}"; do
        if ros2 interface show "$msg" >/dev/null 2>&1; then
            print_status "PASS" "$msg interface available"
        else
            print_status "FAIL" "$msg interface not available"
            return 1
        fi
    done
    
    return 0
}

# Function to verify C++ SDK libraries
verify_cpp_libraries() {
    print_status "INFO" "Verifying C++ SDK Libraries..."
    
    cd "$CYCLONEDDS_WS"
    
    # Check for built libraries
    local lib_count=$(find build/ -name "*.so" -o -name "*.a" | grep -E "(unitree|sdk)" | wc -l)
    if [[ $lib_count -gt 0 ]]; then
        print_status "PASS" "Found $lib_count Unitree SDK libraries"
    else
        print_status "WARN" "No Unitree SDK libraries found"
    fi
    
    # Check for Python bindings
    if find install/ -name "*unitree*" -type f | grep -q "python"; then
        print_status "PASS" "Python bindings available"
    else
        print_status "WARN" "Python bindings not found"
    fi
    
    return 0
}

# Function to test basic functionality
test_basic_functionality() {
    print_status "INFO" "Testing Basic Functionality..."
    
    # Source environments
    cd "$CYCLONEDDS_WS"
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    
    # Set RMW implementation
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    
    # Test topic listing
    if timeout 10s ros2 topic list >/dev/null 2>&1; then
        print_status "PASS" "ROS2 topic listing works"
    else
        print_status "FAIL" "ROS2 topic listing failed"
        return 1
    fi
    
    # Test if we can create a Unitree message
    print_status "INFO" "Testing Unitree message creation..."
    local test_script="/tmp/test_unitree_msg.py"
    cat > "$test_script" << 'EOF'
import sys
sys.path.append('/opt/ros/foxy/lib/python3.8/site-packages')
try:
    from unitree_go.msg import LowCmd, SportModeState
    from unitree_api.msg import Request, Response
    print("✅ All critical Unitree messages imported successfully")
except ImportError as e:
    print(f"❌ Failed to import Unitree messages: {e}")
    sys.exit(1)
EOF
    
    if python3 "$test_script"; then
        print_status "PASS" "Unitree message imports successful"
    else
        print_status "FAIL" "Unitree message imports failed"
        return 1
    fi
    
    rm -f "$test_script"
    return 0
}

# Function to create environment setup script
create_environment_script() {
    print_status "INFO" "Creating environment setup script..."
    
    local env_script="$PROJECT_ROOT/.env.unitree_sdk"
    cat > "$env_script" << EOF
#!/bin/bash
# Unitree SDK Environment Setup
# Generated: $(date '+%Y-%m-%d %H:%M:%S')

# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Source cyclonedx workspace
source $CYCLONEDDS_WS/install/setup.bash

# Set RMW implementation to CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set ROS domain ID (adjust as needed)
export ROS_DOMAIN_ID=0

# Optional: Set CYCLONEDX_URI for custom configuration
# export CYCLONEDX_URI="<CycloneDX><Domain><General><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDX>"

echo "🤖 Unitree SDK environment loaded!"
echo "📦 ROS2 Distro: \$ROS_DISTRO"
echo "🔧 RMW Implementation: \$RMW_IMPLEMENTATION"
echo "🌐 ROS Domain ID: \$ROS_DOMAIN_ID"
echo ""
echo "Usage examples:"
echo "  ros2 topic list | grep unitree"
echo "  ros2 interface list | grep unitree"
echo "  ros2 interface show unitree_go/msg/LowCmd"
EOF
    
    chmod +x "$env_script"
    print_status "PASS" "Environment script created: .env.unitree_sdk"
    
    return 0
}

# Function to generate installation report
generate_report() {
    print_status "INFO" "Generating Installation Report..."
    
    local report_file="$PROJECT_ROOT/logs/$(date '+%Y%m')/unitree_sdk_installation_report.md"
    mkdir -p "$(dirname "$report_file")"
    
    cat > "$report_file" << EOF
# Unitree SDK Installation Report

**Generated:** $(date '+%Y-%m-%d %H:%M:%S %Z')  
**Platform:** $(uname -a)  
**ROS2 Distro:** ${ROS_DISTRO:-'Not set'}  

## Installation Summary

### System Dependencies
$(dpkg -l | grep -E "(cyclone|unitree)" | awk '{print "- " $2 " (" $3 ")"}')

### ROS2 Packages
$(ros2 pkg list | grep unitree | awk '{print "- " $1}')

### Message Interfaces
$(ros2 interface list | grep unitree | head -10 | awk '{print "- " $1}')

### Workspace Structure
\`\`\`
$(ls -la $CYCLONEDDS_WS/ | head -10)
\`\`\`

### Environment Setup
- Workspace: $CYCLONEDDS_WS
- Environment Script: .env.unitree_sdk
- RMW Implementation: rmw_cyclonedds_cpp

## Next Steps
1. Source environment: \`source .env.unitree_sdk\`
2. Test communication: \`ros2 topic list | grep unitree\`
3. Run examples: Check \`cyclonedds_ws/src/unitree_ros2/example/\`

## Troubleshooting
- Ensure Go2 robot is connected and powered on
- Check network configuration and ROS_DOMAIN_ID
- Verify DDS communication with \`ros2 topic echo /lowstate\`
EOF
    
    print_status "PASS" "Installation report saved: $(basename "$report_file")"
    return 0
}

# Main execution
main() {
    print_header
    
    # System information
    print_status "INFO" "Platform: $(uname -a)"
    print_status "INFO" "ROS2 Distro: ${ROS_DISTRO:-'Not set'}"
    print_status "INFO" "Project Root: $PROJECT_ROOT"
    print_status "INFO" "Workspace: $CYCLONEDDS_WS"
    
    # Execute installation steps
    if ! check_prerequisites; then
        print_status "FAIL" "Prerequisites check failed"
        return 1
    fi
    
    if ! install_system_dependencies; then
        print_status "FAIL" "System dependencies installation failed"
        return 1
    fi
    
    if ! verify_sdk_repositories; then
        print_status "FAIL" "SDK repositories verification failed"
        return 1
    fi
    
    # Build only if needed or forced
    if [[ "$1" == "rebuild" ]] || [[ ! -d "$CYCLONEDDS_WS/install" ]]; then
        if ! build_workspace "$1"; then
            print_status "FAIL" "Workspace build failed"
            return 1
        fi
    else
        print_status "INFO" "Workspace already built, skipping build step"
    fi
    
    if ! verify_installation; then
        print_status "FAIL" "Installation verification failed"
        return 1
    fi
    
    if ! verify_cpp_libraries; then
        print_status "WARN" "Some C++ libraries verification issues"
    fi
    
    if ! test_basic_functionality; then
        print_status "FAIL" "Basic functionality test failed"
        return 1
    fi
    
    if ! create_environment_script; then
        print_status "FAIL" "Environment script creation failed"
        return 1
    fi
    
    if ! generate_report; then
        print_status "WARN" "Report generation failed"
    fi
    
    # Success summary
    echo -e "\n${BLUE}================================================${NC}"
    echo -e "${BLUE}           Installation Complete${NC}"
    echo -e "${BLUE}================================================${NC}"
    print_status "PASS" "Unitree SDKs installation completed successfully!"
    echo -e "\n${GREEN}🎉 Ready for Unitree Go2 development!${NC}"
    echo -e "\n${YELLOW}Quick Start:${NC}"
    echo -e "  ${BLUE}source .env.unitree_sdk${NC}"
    echo -e "  ${BLUE}ros2 topic list | grep unitree${NC}"
    echo -e "  ${BLUE}ros2 interface show unitree_go/msg/LowCmd${NC}"
    
    log "PASS" "Unitree SDKs installation completed successfully"
    return 0
}

# Cleanup function
cleanup_on_exit() {
    local exit_code=$?
    
    # Clean up temporary files
    rm -f /tmp/test_unitree_msg.py
    
    log "INFO" "Unitree SDK installation completed with exit code: $exit_code"
    exit $exit_code
}

# Set up cleanup trap
trap cleanup_on_exit EXIT

# Run main function with arguments
main "$@"