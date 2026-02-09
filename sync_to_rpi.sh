#!/bin/bash
# Sync project to Raspberry Pi over WiFi
# Usage: ./sync_to_rpi.sh [--dry-run]

RPI_HOST="aruconav.local"
RPI_HOST_IP="10.59.24.251"  # Fallback IP
RPI_USER="aruconav"
RPI_PATH="/home/aruconav/aruco_drone_nav"
LOCAL_PATH="$(dirname "$(realpath "$0")")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Parse args
DRY_RUN=""
if [[ "$1" == "--dry-run" ]]; then
    DRY_RUN="--dry-run"
    echo -e "${YELLOW}DRY RUN MODE - no changes will be made${NC}"
fi

echo -e "${GREEN}Syncing to RPi at ${RPI_HOST}...${NC}"

# Check connectivity (try hostname first, then IP)
if ping -c 1 -W 2 "$RPI_HOST" &>/dev/null; then
    TARGET_HOST="$RPI_HOST"
elif ping -c 1 -W 2 "$RPI_HOST_IP" &>/dev/null; then
    echo -e "${YELLOW}Using fallback IP ${RPI_HOST_IP}${NC}"
    TARGET_HOST="$RPI_HOST_IP"
else
    echo -e "${RED}Error: Cannot reach RPi at ${RPI_HOST} or ${RPI_HOST_IP}${NC}"
    echo "Make sure RPi is powered on and connected to WiFi"
    exit 1
fi

# Rsync with exclusions
rsync -avz --progress $DRY_RUN \
    --exclude='.git' \
    --exclude='venv' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.pytest_cache' \
    --exclude='recordings/*.json' \
    --exclude='*.bag' \
    --exclude='buildroot' \
    --exclude='.claude' \
    "$LOCAL_PATH/" "${RPI_USER}@${TARGET_HOST}:${RPI_PATH}/"

if [[ $? -eq 0 ]]; then
    echo -e "${GREEN}Sync complete!${NC}"
    if [[ -z "$DRY_RUN" ]]; then
        echo -e "Run on RPi: ${YELLOW}cd ${RPI_PATH} && python3 -m src --mode test${NC}"
    fi
else
    echo -e "${RED}Sync failed${NC}"
    exit 1
fi
