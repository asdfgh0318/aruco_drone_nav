#!/bin/bash
# Run command on RPi over SSH
# Usage: ./rpi_cmd.sh <command>
# Examples:
#   ./rpi_cmd.sh "python3 -m src.main --mode test"
#   ./rpi_cmd.sh "systemctl status aruco-nav"
#   ./rpi_cmd.sh  # interactive shell

RPI_HOST="aruconav.local"
RPI_HOST_IP="10.59.24.251"
RPI_USER="aruconav"
RPI_PATH="/home/aruconav/aruco_drone_nav"

# Resolve host (try hostname first, then IP)
if ping -c 1 -W 1 "$RPI_HOST" &>/dev/null; then
    TARGET="${RPI_USER}@${RPI_HOST}"
else
    TARGET="${RPI_USER}@${RPI_HOST_IP}"
fi

if [[ -z "$1" ]]; then
    # Interactive shell
    ssh -t "$TARGET" "cd ${RPI_PATH} && bash"
else
    # Run command
    ssh "$TARGET" "cd ${RPI_PATH} && $*"
fi
