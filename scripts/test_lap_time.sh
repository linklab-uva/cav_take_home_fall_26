#!/bin/bash
# Test script for Lap Time metric (Metric C)
# Verifies that /lap_time topic exists and publishes at least one message during bag playback

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
        echo "ERROR: Could not determine repository root"
        exit 1
    fi
fi

# Change to repo root
cd "$REPO_ROOT" || exit 1

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check for ROS2
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}ERROR: /opt/ros/humble/setup.bash not found. Make sure ROS2 is installed.${NC}"
    echo "This script must be run inside a ROS2 Humble Docker container."
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Disable FastDDS SHM transport to avoid shared memory errors
if [ -f "${REPO_ROOT}/disable_shm_fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${REPO_ROOT}/disable_shm_fastdds.xml"
    echo -e "${GREEN}✓ Disabled FastDDS SHM transport${NC}"
elif [ -f "/tmp/disable_shm_fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="/tmp/disable_shm_fastdds.xml"
    echo -e "${GREEN}✓ Disabled FastDDS SHM transport (using /tmp)${NC}"
fi

# Bag file path
BAG_FILE="${REPO_ROOT}/cavalier_take_home_26.mcap"

# Check bag file exists
if [ ! -f "$BAG_FILE" ]; then
    echo -e "${RED}ERROR: Bag file not found at $BAG_FILE${NC}"
    exit 1
fi

# Build the workspace
echo ""
echo -e "${GREEN}=== Building workspace ===${NC}"
if ! colcon build --symlink-install; then
    echo -e "${RED}ERROR: Build failed${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Build successful${NC}"

# Source the install folder
source install/setup.bash
echo -e "${GREEN}✓ Sourced install/setup.bash${NC}"

# Expected topic
EXPECTED_TOPIC="lap_time"

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up...${NC}"
    if [ ! -z "$NODE_PID" ]; then
        kill $NODE_PID 2>/dev/null || true
    fi
    if [ ! -z "$BAG_PID" ]; then
        kill $BAG_PID 2>/dev/null || true
    fi
    sleep 1
    pkill -f "take_home_node" 2>/dev/null || true
    pkill -f "ros2 bag play" 2>/dev/null || true
    echo -e "${GREEN}✓ Cleanup complete${NC}"
}

# Kill any existing nodes before starting (prevent duplicates)
echo -e "${YELLOW}Killing any existing take_home_node instances...${NC}"
pkill -f "take_home_node" 2>/dev/null || true
sleep 1

# Set trap to cleanup on exit
trap cleanup EXIT INT TERM

# Launch the node in background
echo ""
echo -e "${GREEN}=== Launching take_home_node ===${NC}"
ros2 run take_home take_home_node &
NODE_PID=$!
echo "Node PID: $NODE_PID"
sleep 2  # Give node time to start

# Check if node is still running
if ! kill -0 $NODE_PID 2>/dev/null; then
    echo -e "${RED}ERROR: Node failed to start${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Node is running${NC}"

# Wait for discovery - topics need time to be advertised
echo ""
echo -e "${YELLOW}Waiting for topic discovery (2 seconds)...${NC}"
sleep 2

# Check if topic is advertised (it should appear after initial publish)
echo ""
echo -e "${YELLOW}Checking topic before bag playback...${NC}"
INITIAL_TOPICS=$(ros2 topic list 2>/dev/null || echo "")
INITIAL_LAP_TIME=$(echo "$INITIAL_TOPICS" | grep -c "lap_time" || echo "0")
echo "  lap_time found: $INITIAL_LAP_TIME"
echo ""

# Play the bag file with increased read-ahead queue to avoid starvation warnings
echo ""
echo -e "${GREEN}=== Playing bag file ===${NC}"
echo "Note: Lap time may take time to compute (waiting for lap completion)..."
ros2 bag play -s mcap "$BAG_FILE" --read-ahead-queue-size 5000 &
BAG_PID=$!
echo "Bag play PID: $BAG_PID"

# Wait for bag to start and allow discovery
echo "Waiting for bag to start and topics to be discovered..."
sleep 2

# Wait for bag to finish
echo "Waiting for bag playback to complete..."
wait $BAG_PID 2>/dev/null || true

# Give node time to process remaining messages and for final discovery
sleep 2

# Diagnostic information
echo ""
echo -e "${GREEN}=== Diagnostic Information ===${NC}"
echo "Current directory: $(pwd)"
echo "ROS2 package path: $ROS_PACKAGE_PATH"
echo "Take home package prefix: $(ros2 pkg prefix take_home 2>/dev/null || echo 'NOT FOUND')"
echo ""

# Check available topics
echo ""
echo -e "${GREEN}=== Checking available topics ===${NC}"
TOPICS=$(ros2 topic list 2>/dev/null || echo "")
echo "$TOPICS"
echo ""

# Check for lap_time topic specifically
echo -e "${YELLOW}Checking for lap_time topic in topic list:${NC}"
echo "$TOPICS" | grep -E "lap_time" || echo "  (No lap_time topic found)"
echo ""

# Verify expected topic exists
echo -e "${GREEN}=== Verifying expected topic ===${NC}"
if echo "$TOPICS" | grep -q "^/$EXPECTED_TOPIC$"; then
    echo -e "${GREEN}✓ Topic /$EXPECTED_TOPIC exists${NC}"
else
    echo -e "${RED}✗ Topic /$EXPECTED_TOPIC NOT FOUND${NC}"
    echo -e "${RED}✗ Test failed: topic does not exist${NC}"
    exit 1
fi

# Sample message from topic (use longer timeout since lap may take time)
echo ""
echo -e "${GREEN}=== Sampling message from /$EXPECTED_TOPIC ===${NC}"
echo -e "${YELLOW}Waiting up to 20 seconds for a lap time message...${NC}"
SAMPLE_OUTPUT=$(timeout 20 ros2 topic echo /$EXPECTED_TOPIC -n 1 2>&1 || echo "TIMEOUT_OR_ERROR")
if echo "$SAMPLE_OUTPUT" | grep -q "data:"; then
    echo ""
    echo "$SAMPLE_OUTPUT" | head -10
    echo ""
    echo -e "${GREEN}  ✓ Topic is publishing${NC}"
    
    # Extract the lap time value for display
    LAP_TIME_VALUE=$(echo "$SAMPLE_OUTPUT" | grep "data:" | head -1 | awk '{print $2}')
    if [ -n "$LAP_TIME_VALUE" ]; then
        echo -e "${GREEN}  Lap time value: ${LAP_TIME_VALUE} seconds${NC}"
    fi
else
    echo ""
    echo "  (No messages received or timeout)"
    echo -e "${RED}  ✗ Topic exists but not publishing${NC}"
    echo -e "${YELLOW}  Note: This may be normal if no lap was completed during bag playback${NC}"
    echo -e "${YELLOW}  Check node logs for lap detection messages${NC}"
    exit 1
fi

# Optional: Debug command section
echo ""
echo -e "${GREEN}=== Debug: Topic frequencies ===${NC}"
echo "Input curvilinear_distance topic frequency:"
timeout 3 ros2 topic hz /curvilinear_distance 2>&1 | head -3 || echo "  (Could not measure)"
echo ""
echo "Output lap_time topic frequency:"
timeout 3 ros2 topic hz /lap_time 2>&1 | head -3 || echo "  (Could not measure)"

# Final status
echo ""
echo -e "${GREEN}=== Test Summary ===${NC}"
echo -e "${GREEN}✓ Topic /$EXPECTED_TOPIC exists${NC}"
echo -e "${GREEN}✓ Topic is publishing messages${NC}"
echo -e "${GREEN}✓ Test completed successfully${NC}"
exit 0
