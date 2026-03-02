#!/bin/bash
# Test script for Task 2 metrics implementation
# This script builds the workspace, runs the node, plays the bag, and verifies all metrics publish
#
# USAGE:
#   On Linux (with ROS2 installed):
#     ./scripts/test_metrics.sh
#
#   On macOS (using Docker):
#     ./scripts/test_metrics_docker.sh [container_name]
#     OR manually: docker exec -it cav_container bash -c "cd /path/to/workspace && ./scripts/test_metrics.sh"

set -e  # Exit on error

# Determine repo root
# Check if REPO_ROOT was passed as environment variable (from Docker wrapper)
# But exclude root directory "/" as it's not a valid workspace
if [ -n "$REPO_ROOT" ] && [ "$REPO_ROOT" != "/" ] && [ -d "$REPO_ROOT" ]; then
    # Use provided REPO_ROOT
    :  # REPO_ROOT already set
else
    # When run from Docker wrapper, script is in /tmp but we cd to workspace first
    # So pwd should be the workspace root
    CURRENT_DIR="$(pwd)"
    
    # Start with current directory (should be workspace when called from Docker wrapper)
    REPO_ROOT="$CURRENT_DIR"
    
    # Verify we're in the right place - check for key files/directories
    if [ ! -f "${REPO_ROOT}/cavalier_take_home_26.mcap" ] && [ ! -d "${REPO_ROOT}/src/take_home_node" ]; then
        # Try common workspace locations
        for root in "/home/$USER/cav_take_home_fall_26" \
                    "/home/$(whoami)/cav_take_home_fall_26" \
                    "/root/cav_take_home_fall_26" \
                    "$HOME/cav_take_home_fall_26" \
                    "$CURRENT_DIR"; do
            if [ -n "$root" ] && [ "$root" != "/" ] && [ -d "$root" ]; then
                if [ -f "${root}/cavalier_take_home_26.mcap" ] || [ -d "${root}/src/take_home_node" ]; then
                    REPO_ROOT="$root"
                    break
                fi
            fi
        done
    fi
    
    # Sanity check - ensure REPO_ROOT is valid
    if [ -z "$REPO_ROOT" ] || [ "$REPO_ROOT" = "/" ] || [ ! -d "$REPO_ROOT" ]; then
        # Last resort: use current directory
        REPO_ROOT="$CURRENT_DIR"
    fi
fi

# Normalize path (remove trailing slashes, ensure it's not empty or just "/")
REPO_ROOT="${REPO_ROOT%/}"
if [ -z "$REPO_ROOT" ] || [ "$REPO_ROOT" = "/" ]; then
    # Try to get a valid path
    CURRENT_DIR="$(pwd)"
    if [ -n "$CURRENT_DIR" ] && [ "$CURRENT_DIR" != "/" ]; then
        REPO_ROOT="$CURRENT_DIR"
    else
        # Last resort fallbacks
        for root in "/home/$USER/cav_take_home_fall_26" "/home/$(whoami)/cav_take_home_fall_26" "/root/cav_take_home_fall_26" "$HOME/cav_take_home_fall_26"; do
            if [ -n "$root" ] && [ -d "$root" ]; then
                REPO_ROOT="$root"
                break
            fi
        done
    fi
fi

# Final safety check
if [ -z "$REPO_ROOT" ] || [ "$REPO_ROOT" = "/" ]; then
    echo "ERROR: Could not determine repository root directory"
    exit 1
fi

# Construct bag file path (avoid double slashes)
REPO_ROOT="${REPO_ROOT%/}"  # Remove trailing slash
BAG_FILE="${REPO_ROOT}/cavalier_take_home_26.mcap"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Task 2 Metrics Test Script ===${NC}"
echo ""

# Debug output (can be removed later)
if [ "${DEBUG:-0}" = "1" ]; then
    echo "DEBUG: REPO_ROOT=$REPO_ROOT"
    echo "DEBUG: BAG_FILE=$BAG_FILE"
    echo "DEBUG: pwd=$(pwd)"
    echo "DEBUG: ls -la $REPO_ROOT:"
    ls -la "$REPO_ROOT" 2>&1 | head -5 || true
fi

# Check if bag file exists
if [ ! -f "$BAG_FILE" ]; then
    echo -e "${RED}ERROR: Bag file not found at $BAG_FILE${NC}"
    echo -e "${YELLOW}Current directory: $(pwd)${NC}"
    echo -e "${YELLOW}Repo root: $REPO_ROOT${NC}"
    echo -e "${YELLOW}REPO_ROOT env var: ${REPO_ROOT:-<not set>}${NC}"
    if [ -d "$REPO_ROOT" ]; then
        echo -e "${YELLOW}Contents of $REPO_ROOT:${NC}"
        ls -la "$REPO_ROOT" 2>&1 | head -10 || true
    fi
    echo -e "${YELLOW}Please ensure the bag file 'cavalier_take_home_26.mcap' is in the repo root.${NC}"
    exit 1
fi

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
source /opt/ros/humble/setup.bash
echo -e "${GREEN}✓ Sourced ROS2 Humble${NC}"

# Disable FastDDS SHM transport to avoid shared memory errors
if [ -f "${REPO_ROOT}/disable_shm_fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${REPO_ROOT}/disable_shm_fastdds.xml"
    echo -e "${GREEN}✓ Disabled FastDDS SHM transport${NC}"
elif [ -f "/tmp/disable_shm_fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="/tmp/disable_shm_fastdds.xml"
    echo -e "${GREEN}✓ Disabled FastDDS SHM transport (using /tmp)${NC}"
fi
else
    echo -e "${RED}ERROR: /opt/ros/humble/setup.bash not found.${NC}"
    echo -e "${YELLOW}This script must be run inside a ROS2 environment.${NC}"
    echo -e "${YELLOW}On macOS, use: ./scripts/test_metrics_docker.sh${NC}"
    echo -e "${YELLOW}Or run inside Docker: docker exec -it <container> bash${NC}"
    exit 1
fi

# Change to repo root
cd "$REPO_ROOT"

# Setup debug log path (works in both Docker and native environments)
DEBUG_LOG="${REPO_ROOT}/.cursor/debug.log"
mkdir -p "${REPO_ROOT}/.cursor" 2>/dev/null || true

# #region agent log
# Instrumentation: Log build directory state before building
PROBLEMATIC_PATH="${REPO_ROOT}/build/deep_orange_msgs/ament_cmake_python/deep_orange_msgs/deep_orange_msgs"
if [ -e "$PROBLEMATIC_PATH" ]; then
    PATH_TYPE="unknown"
    if [ -L "$PROBLEMATIC_PATH" ]; then
        PATH_TYPE="symlink"
    elif [ -d "$PROBLEMATIC_PATH" ]; then
        PATH_TYPE="directory"
    elif [ -f "$PROBLEMATIC_PATH" ]; then
        PATH_TYPE="file"
    fi
    echo "{\"location\":\"test_metrics.sh:pre_build\",\"message\":\"Problematic path exists before build\",\"data\":{\"path\":\"$PROBLEMATIC_PATH\",\"type\":\"$PATH_TYPE\",\"hypothesisId\":\"A\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
fi
# #endregion

# Build the workspace
echo ""
echo -e "${GREEN}=== Building workspace ===${NC}"

# #region agent log
# Instrumentation: Check if build directory exists and log its state
if [ -d "${REPO_ROOT}/build" ]; then
    BUILD_DIR_SIZE=$(du -sh "${REPO_ROOT}/build" 2>/dev/null | cut -f1 || echo "unknown")
    echo "{\"location\":\"test_metrics.sh:pre_build\",\"message\":\"Build directory exists before build\",\"data\":{\"build_dir\":\"${REPO_ROOT}/build\",\"size\":\"$BUILD_DIR_SIZE\",\"hypothesisId\":\"B\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
fi

# Check for problematic symlink paths that might be directories
PROBLEMATIC_PATHS=(
    "${REPO_ROOT}/build/deep_orange_msgs/ament_cmake_python/deep_orange_msgs/deep_orange_msgs"
    "${REPO_ROOT}/build/vectornav_msgs/ament_cmake_python/vectornav_msgs/vectornav_msgs"
    "${REPO_ROOT}/build/raptor_dbw_msgs/ament_cmake_python/raptor_dbw_msgs/raptor_dbw_msgs"
    "${REPO_ROOT}/build/novatel_oem7_msgs/ament_cmake_python/novatel_oem7_msgs/novatel_oem7_msgs"
)

CLEANUP_NEEDED=0
for path in "${PROBLEMATIC_PATHS[@]}"; do
    if [ -d "$path" ]; then
        echo "{\"location\":\"test_metrics.sh:pre_build\",\"message\":\"Directory found where symlink should be\",\"data\":{\"path\":\"$path\",\"hypothesisId\":\"A\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
        CLEANUP_NEEDED=1
    fi
done
# #endregion

# Fix: Clean build directory if corrupted artifacts detected (Hypothesis A - CONFIRMED)
if [ $CLEANUP_NEEDED -eq 1 ] || [ ! -d "${REPO_ROOT}/build" ]; then
    echo -e "${YELLOW}Cleaning build artifacts (corrupted symlinks detected)...${NC}"
    # #region agent log
    echo "{\"location\":\"test_metrics.sh:pre_build\",\"message\":\"Cleaning build directory\",\"data\":{\"reason\":\"corrupted_symlinks\",\"hypothesisId\":\"A\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    # #endregion
    rm -rf "${REPO_ROOT}/build" "${REPO_ROOT}/install" "${REPO_ROOT}/log" 2>/dev/null || true
    echo -e "${GREEN}✓ Build directories cleaned${NC}"
fi

echo "Running colcon build (this may take a few minutes)..."
colcon build --symlink-install 2>&1 | tee /tmp/colcon_build.log
BUILD_EXIT_CODE=${PIPESTATUS[0]}

# #region agent log
# Instrumentation: Log build result
echo "{\"location\":\"test_metrics.sh:post_build\",\"message\":\"Build completed\",\"data\":{\"exit_code\":$BUILD_EXIT_CODE,\"hypothesisId\":\"A,B\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
# #endregion

if [ $BUILD_EXIT_CODE -ne 0 ]; then
    echo -e "${RED}ERROR: Build failed with exit code $BUILD_EXIT_CODE${NC}"
    echo ""
    
    # #region agent log
    # Instrumentation: Check for symlink error specifically
    if grep -q "failed to create symbolic link" /tmp/colcon_build.log; then
        SYMLINK_ERROR=$(grep "failed to create symbolic link" /tmp/colcon_build.log | head -1)
        echo "{\"location\":\"test_metrics.sh:build_error\",\"message\":\"Symlink creation error detected\",\"data\":{\"error\":\"$SYMLINK_ERROR\",\"hypothesisId\":\"A\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
        
        # Check the problematic path again
        if [ -e "$PROBLEMATIC_PATH" ]; then
            PATH_TYPE="unknown"
            if [ -L "$PROBLEMATIC_PATH" ]; then
                PATH_TYPE="symlink"
            elif [ -d "$PROBLEMATIC_PATH" ]; then
                PATH_TYPE="directory"
            elif [ -f "$PROBLEMATIC_PATH" ]; then
                PATH_TYPE="file"
            fi
            echo "{\"location\":\"test_metrics.sh:build_error\",\"message\":\"Problematic path state after build failure\",\"data\":{\"path\":\"$PROBLEMATIC_PATH\",\"type\":\"$PATH_TYPE\",\"hypothesisId\":\"A\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
        fi
    fi
    # #endregion
    
    echo -e "${YELLOW}Build errors summary:${NC}"
    tail -50 /tmp/colcon_build.log | grep -i "error\|failed\|aborted" | head -20 || tail -30 /tmp/colcon_build.log
    echo ""
    echo -e "${YELLOW}Common fixes:${NC}"
    echo "1. Clean build directory: rm -rf build install log"
    echo "2. Install missing dependencies: sudo apt install ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime"
    echo "3. Run rosdep: rosdep install --from-paths src --ignore-src -r -y"
    echo "4. Try building without tests: colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF"
    echo ""
    echo "Full build log saved to: /tmp/colcon_build.log"
    exit 1
fi
echo -e "${GREEN}✓ Build successful${NC}"

# Source the install folder
source install/setup.bash
echo -e "${GREEN}✓ Sourced install/setup.bash${NC}"

# Define expected metric topics
EXPECTED_TOPICS=(
    "metrics_output"
    "slip/long/rr"
    "slip/long/rl"
    "slip/long/fr"
    "slip/long/fl"
    "imu_top/jitter"
    "imu_bottom/jitter"
    "imu_vectornav/jitter"
    "lap_time"
)

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up...${NC}"
    # Kill background processes
    if [ ! -z "$NODE_PID" ]; then
        kill $NODE_PID 2>/dev/null || true
    fi
    if [ ! -z "$BAG_PID" ]; then
        kill $BAG_PID 2>/dev/null || true
    fi
    # Wait a bit for cleanup
    sleep 1
    # Force kill if still running
    pkill -f "take_home_node" 2>/dev/null || true
    pkill -f "ros2 bag play" 2>/dev/null || true
    echo -e "${GREEN}✓ Cleanup complete${NC}"
}

# Set trap to cleanup on exit
trap cleanup EXIT INT TERM

# Launch the node in background
echo ""
echo -e "${GREEN}=== Launching take_home_node ===${NC}"
NODE_LOG="/tmp/take_home_node.log"
ros2 run take_home take_home_node > "$NODE_LOG" 2>&1 &
NODE_PID=$!
echo "Node PID: $NODE_PID"
sleep 3  # Give node time to start

# #region agent log
# Instrumentation: Check node status and log initial state
if kill -0 $NODE_PID 2>/dev/null; then
    echo "{\"location\":\"test_metrics.sh:node_start\",\"message\":\"Node started successfully\",\"data\":{\"pid\":$NODE_PID,\"hypothesisId\":\"C\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    # Check for errors in node log
    if [ -f "$NODE_LOG" ] && [ -s "$NODE_LOG" ]; then
        NODE_ERRORS=$(grep -i "error\|fatal\|exception" "$NODE_LOG" 2>/dev/null | head -5 || echo "")
        if [ -n "$NODE_ERRORS" ]; then
            echo "{\"location\":\"test_metrics.sh:node_start\",\"message\":\"Node log contains errors\",\"data\":{\"errors\":\"$NODE_ERRORS\",\"hypothesisId\":\"D\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
        fi
    fi
else
    echo "{\"location\":\"test_metrics.sh:node_start\",\"message\":\"Node failed to start\",\"data\":{\"pid\":$NODE_PID,\"hypothesisId\":\"C\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    if [ -f "$NODE_LOG" ]; then
        echo "{\"location\":\"test_metrics.sh:node_start\",\"message\":\"Node log contents\",\"data\":{\"log\":\"$(head -20 $NODE_LOG | tr '\n' ';')\",\"hypothesisId\":\"D\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    fi
fi
# #endregion

# Check if node is still running
if ! kill -0 $NODE_PID 2>/dev/null; then
    echo -e "${RED}ERROR: Node failed to start${NC}"
    if [ -f "$NODE_LOG" ]; then
        echo -e "${YELLOW}Node log (last 20 lines):${NC}"
        tail -20 "$NODE_LOG"
    fi
    exit 1
fi
echo -e "${GREEN}✓ Node is running${NC}"

# Wait longer for topics to be advertised (executor needs time to process publishes)
echo ""
echo -e "${YELLOW}Waiting for topics to be advertised (checking multiple times)...${NC}"
for i in {1..5}; do
    sleep 1
    INITIAL_TOPICS=$(ros2 topic list 2>/dev/null || echo "")
    INITIAL_SLIP_COUNT=$(echo "$INITIAL_TOPICS" | grep -c "slip/long" || echo "0")
    INITIAL_IMU_COUNT=$(echo "$INITIAL_TOPICS" | grep -c "imu.*jitter" || echo "0")
    INITIAL_LAP_COUNT=$(echo "$INITIAL_TOPICS" | grep -c "lap_time" || echo "0")
    echo "  Attempt $i: Slip=$INITIAL_SLIP_COUNT, IMU=$INITIAL_IMU_COUNT, Lap=$INITIAL_LAP_COUNT"
    
    # #region agent log
    echo "{\"location\":\"test_metrics.sh:initial_topic_check\",\"message\":\"Topics check attempt $i\",\"data\":{\"attempt\":$i,\"slip_count\":$INITIAL_SLIP_COUNT,\"imu_count\":$INITIAL_IMU_COUNT,\"lap_count\":$INITIAL_LAP_COUNT,\"hypothesisId\":\"E\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    # #endregion
    
    if [ $INITIAL_SLIP_COUNT -eq 4 ] && [ $INITIAL_IMU_COUNT -eq 3 ]; then
        echo -e "${GREEN}  ✓ All topics advertised!${NC}"
        break
    fi
done

# Check node info to see what publishers it has
if command -v ros2 &> /dev/null; then
    echo ""
    echo -e "${YELLOW}Node publisher info:${NC}"
    NODE_INFO=$(ros2 node info /take_home_metrics 2>/dev/null || echo "")
    if [ -n "$NODE_INFO" ]; then
        echo "$NODE_INFO" | grep -A 30 "Publishers:" || echo "  (No Publishers section found)"
        # #region agent log
        echo "{\"location\":\"test_metrics.sh:node_info\",\"message\":\"Node publisher info\",\"data\":{\"info\":\"$(echo $NODE_INFO | tr '\n' ';')\",\"hypothesisId\":\"F\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
        # #endregion
    else
        echo "  (Node not found or not responding)"
        # #region agent log
        echo "{\"location\":\"test_metrics.sh:node_info\",\"message\":\"Node not found\",\"data\":{\"hypothesisId\":\"F\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
        # #endregion
    fi
fi

# Check node log for any errors or info messages
if [ -f "$NODE_LOG" ] && [ -s "$NODE_LOG" ]; then
    echo ""
    echo -e "${YELLOW}Recent node log messages:${NC}"
    tail -10 "$NODE_LOG" | head -5 || true
    # #region agent log
    echo "{\"location\":\"test_metrics.sh:node_log\",\"message\":\"Node log contents\",\"data\":{\"log\":\"$(tail -10 $NODE_LOG | tr '\n' ';')\",\"hypothesisId\":\"D\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    # #endregion
fi

# Play the bag file (limited duration for testing - 30 seconds)
echo ""
echo -e "${GREEN}=== Playing bag file (30 seconds) ===${NC}"
timeout 30 ros2 bag play -s mcap "$BAG_FILE" &
BAG_PID=$!
echo "Bag play PID: $BAG_PID"

# Wait for bag to start playing
sleep 5

# #region agent log
# Instrumentation: Check node status and topics before verification
if kill -0 $NODE_PID 2>/dev/null; then
    echo "{\"location\":\"test_metrics.sh:pre_topic_check\",\"message\":\"Node still running before topic check\",\"data\":{\"pid\":$NODE_PID,\"hypothesisId\":\"C\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
else
    echo "{\"location\":\"test_metrics.sh:pre_topic_check\",\"message\":\"Node died before topic check\",\"data\":{\"pid\":$NODE_PID,\"hypothesisId\":\"C\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
fi
# #endregion

# Check available topics
echo ""
echo -e "${GREEN}=== Checking available topics ===${NC}"
TOPICS=$(ros2 topic list 2>/dev/null || echo "")

# #region agent log
# Instrumentation: Log all available topics
TOPIC_COUNT=$(echo "$TOPICS" | grep -c "^/" || echo "0")
echo "{\"location\":\"test_metrics.sh:topic_check\",\"message\":\"Topics available\",\"data\":{\"count\":$TOPIC_COUNT,\"topics\":\"$(echo $TOPICS | tr '\n' ',')\",\"hypothesisId\":\"E\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
# #endregion

echo "$TOPICS"
echo ""

# Verify each expected topic exists
echo -e "${GREEN}=== Verifying metric topics ===${NC}"
MISSING_TOPICS=()
for topic in "${EXPECTED_TOPICS[@]}"; do
    if echo "$TOPICS" | grep -q "^/$topic$"; then
        echo -e "${GREEN}✓ Topic /$topic exists${NC}"
    else
        echo -e "${RED}✗ Topic /$topic NOT FOUND${NC}"
        MISSING_TOPICS+=("$topic")
    fi
done

# Wait for bag to finish or timeout
wait $BAG_PID 2>/dev/null || true

# #region agent log
# Instrumentation: Check node status after bag playback
if kill -0 $NODE_PID 2>/dev/null; then
    echo "{\"location\":\"test_metrics.sh:post_bag\",\"message\":\"Node still running after bag playback\",\"data\":{\"pid\":$NODE_PID,\"hypothesisId\":\"C\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    # Re-check topics after bag playback
    TOPICS_AFTER=$(ros2 topic list 2>/dev/null || echo "")
    TOPIC_COUNT_AFTER=$(echo "$TOPICS_AFTER" | grep -c "^/" || echo "0")
    echo "{\"location\":\"test_metrics.sh:post_bag\",\"message\":\"Topics after bag playback\",\"data\":{\"count\":$TOPIC_COUNT_AFTER,\"hypothesisId\":\"E\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
else
    echo "{\"location\":\"test_metrics.sh:post_bag\",\"message\":\"Node died during/after bag playback\",\"data\":{\"pid\":$NODE_PID,\"hypothesisId\":\"C\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    if [ -f "$NODE_LOG" ]; then
        echo "{\"location\":\"test_metrics.sh:post_bag\",\"message\":\"Node log after death\",\"data\":{\"log\":\"$(tail -30 $NODE_LOG | tr '\n' ';')\",\"hypothesisId\":\"D\"},\"timestamp\":$(date +%s%3N)}" >> "$DEBUG_LOG" 2>/dev/null || true
    fi
fi
# #endregion

# Give node time to process remaining messages
sleep 2

# Sample messages from each topic
echo ""
echo -e "${GREEN}=== Sampling messages from metric topics ===${NC}"
for topic in "${EXPECTED_TOPICS[@]}"; do
    if echo "$TOPICS" | grep -q "^/$topic$"; then
        echo ""
        echo -e "${YELLOW}--- /$topic ---${NC}"
        # Try to echo 3 messages with timeout
        timeout 3 ros2 topic echo /$topic -n 3 2>/dev/null || echo "  (No messages received or timeout)"
    fi
done

# Final status
echo ""
echo -e "${GREEN}=== Test Summary ===${NC}"
if [ ${#MISSING_TOPICS[@]} -eq 0 ]; then
    echo -e "${GREEN}✓ All expected topics are present${NC}"
    echo -e "${GREEN}✓ Test completed successfully${NC}"
    exit 0
else
    echo -e "${RED}✗ Missing topics: ${MISSING_TOPICS[*]}${NC}"
    echo -e "${RED}✗ Test failed${NC}"
    exit 1
fi
