#!/bin/bash
# Sync project to Raspberry Pi over WiFi
# Usage: ./sync_to_rpi.sh [--dry-run]

RPI_HOST="pi.local"
RPI_HOST_IP="10.253.175.251"  # Fallback IP
RPI_USER="mtj"
RPI_PATH="/home/mtj/aruco_drone_nav"
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

# Check connectivity (try hostname first, then IP; use ssh instead of ping)
if ssh -o ConnectTimeout=3 -o BatchMode=yes "${RPI_USER}@${RPI_HOST}" true &>/dev/null; then
    TARGET_HOST="$RPI_HOST"
elif ssh -o ConnectTimeout=3 -o BatchMode=yes "${RPI_USER}@${RPI_HOST_IP}" true &>/dev/null; then
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
    --exclude='*.glb' \
    --exclude='*.gltf' \
    --exclude='*.zip' \
    --exclude='*.tar.gz' \
    --exclude='*.img.xz' \
    --exclude='*.csv' \
    --exclude='*.jpg' \
    --exclude='*.svg' \
    --exclude='Copter-*' \
    --exclude='docs/' \
    --exclude='markers/' \
    --exclude='viewer/' \
    --exclude='missions/' \
    --exclude='sdcard_backup/' \
    --exclude='*.tar' \
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
