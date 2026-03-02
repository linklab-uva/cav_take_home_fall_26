#!/bin/bash
# macOS wrapper script to run test_slip_metrics.sh inside Docker container
# Usage: ./scripts/test_slip_metrics_docker.sh [container_name]

set -e

CONTAINER_NAME="${1:-cav_container}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== Docker Test Runner for Wheel Slip Metrics (macOS) ==="
echo ""

# Check if container exists and is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '${CONTAINER_NAME}' is not running."
    echo "Attempting to start it..."
    docker container start "${CONTAINER_NAME}" 2>/dev/null || {
        echo "ERROR: Container '${CONTAINER_NAME}' not found or could not be started."
        echo ""
        echo "Please either:"
        echo "  1. Start your container: docker container start ${CONTAINER_NAME}"
        echo "  2. Or create a new container following MacOS-Docker.md instructions"
        echo "  3. Or specify a different container name: $0 <container_name>"
        exit 1
    }
    sleep 2
fi

echo "✓ Container '${CONTAINER_NAME}' is running"
echo ""

# Get the username in the container first
CONTAINER_USER=$(docker exec "${CONTAINER_NAME}" whoami 2>/dev/null || echo "root")

# Find the workspace path in the container
WORKSPACE_PATH=""
for path in "/home/${CONTAINER_USER}/cav_take_home_fall_26" \
            "/home/$USER/cav_take_home_fall_26" \
            "/root/cav_take_home_fall_26" \
            "$(docker exec "${CONTAINER_NAME}" bash -c 'find /home -maxdepth 3 -name "cav_take_home_fall_26" -type d 2>/dev/null | head -1' 2>/dev/null)"; do
    if [ -n "$path" ] && docker exec "${CONTAINER_NAME}" test -d "${path}" 2>/dev/null; then
        WORKSPACE_PATH="${path}"
        break
    fi
done

if [ -z "$WORKSPACE_PATH" ]; then
    echo "WARNING: Could not find workspace in container. Trying to detect..."
    WORKSPACE_PATH=$(docker exec "${CONTAINER_NAME}" bash -c 'find /home -maxdepth 3 -name "cav_take_home_fall_26" -type d 2>/dev/null | head -1' 2>/dev/null)
    if [ -z "$WORKSPACE_PATH" ]; then
        WORKSPACE_PATH="/home/${CONTAINER_USER}/cav_take_home_fall_26"
        echo "Using default path: $WORKSPACE_PATH"
    fi
fi

echo "✓ Workspace path: $WORKSPACE_PATH"
echo ""

# Validate workspace path
if [ -z "$WORKSPACE_PATH" ] || [ "$WORKSPACE_PATH" = "/" ]; then
    echo "ERROR: Invalid workspace path: $WORKSPACE_PATH"
    exit 1
fi

# Copy the test script and bag file into the container
echo "Copying test script into container..."
docker cp "${SCRIPT_DIR}/test_slip_metrics.sh" "${CONTAINER_NAME}:/tmp/test_slip_metrics.sh" 2>/dev/null || true

# Check if bag file exists on host and copy it to container (only if not already there)
BAG_FILE_HOST="${REPO_ROOT}/cavalier_take_home_26.mcap"
BAG_FILE_CONTAINER="${WORKSPACE_PATH}/cavalier_take_home_26.mcap"

if docker exec "${CONTAINER_NAME}" test -f "${BAG_FILE_CONTAINER}" 2>/dev/null; then
    echo "✓ Bag file already exists in container (skipping copy)"
else
    if [ -f "$BAG_FILE_HOST" ]; then
        echo "Copying bag file into container (this may take 10-30 seconds for 1.3GB file)..."
        docker cp "$BAG_FILE_HOST" "${CONTAINER_NAME}:${BAG_FILE_CONTAINER}" 2>/dev/null || {
            echo "WARNING: Could not copy bag file. Make sure it exists in the container at ${BAG_FILE_CONTAINER}"
        }
        echo "✓ Bag file copied"
    else
        echo "WARNING: Bag file not found on host at $BAG_FILE_HOST"
        echo "Make sure the bag file exists in the container at ${BAG_FILE_CONTAINER}"
    fi
fi
echo ""

echo "Running as user: $CONTAINER_USER"
echo ""

# Execute the test script inside the container
echo "=== Running wheel slip metrics test inside Docker container ==="
echo ""

docker exec -it --user "${CONTAINER_USER}" "${CONTAINER_NAME}" bash -c "
    cd '${WORKSPACE_PATH}' || { echo 'ERROR: Failed to cd to ${WORKSPACE_PATH}'; exit 1; } && \
    export REPO_ROOT='${WORKSPACE_PATH}' && \
    chmod +x /tmp/test_slip_metrics.sh && \
    bash /tmp/test_slip_metrics.sh
"

EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "✓ Test completed successfully!"
else
    echo "✗ Test failed with exit code $EXIT_CODE"
fi

exit $EXIT_CODE
