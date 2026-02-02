#!/bin/bash
#
# ArUco Drone Navigation - Raspberry Pi Zero Setup Script
#
# This script configures a fresh Raspberry Pi OS installation for
# the ArUco drone navigation system.
#
# Usage:
#   1. Flash Raspberry Pi OS Lite (64-bit recommended for RPi Zero 2W)
#   2. Enable SSH and configure WiFi in Raspberry Pi Imager
#   3. Boot the Pi and SSH into it
#   4. Run: curl -sSL https://raw.githubusercontent.com/asdfgh0318/aruco_drone_nav/main/rpi_setup/setup_rpi.sh | bash
#   Or copy this script and run: sudo bash setup_rpi.sh
#

set -e

echo "=============================================="
echo "  ArUco Drone Navigation - RPi Setup"
echo "=============================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (sudo bash setup_rpi.sh)"
    exit 1
fi

# Get the actual user (not root)
ACTUAL_USER=${SUDO_USER:-pi}
ACTUAL_HOME=$(eval echo ~$ACTUAL_USER)

echo "Setting up for user: $ACTUAL_USER"
echo "Home directory: $ACTUAL_HOME"
echo ""

# Update system
echo "[1/8] Updating system packages..."
apt-get update
apt-get upgrade -y

# Install system dependencies
echo "[2/8] Installing system dependencies..."
apt-get install -y \
    python3-pip \
    python3-venv \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    python3-tk \
    libopencv-dev \
    git \
    v4l-utils \
    i2c-tools \
    screen \
    htop

# Enable camera and serial
echo "[3/8] Configuring hardware interfaces..."

# Enable camera
if ! grep -q "^start_x=1" /boot/config.txt 2>/dev/null && ! grep -q "^camera_auto_detect=1" /boot/firmware/config.txt 2>/dev/null; then
    # For newer Pi OS (bookworm+)
    if [ -f /boot/firmware/config.txt ]; then
        echo "camera_auto_detect=1" >> /boot/firmware/config.txt
    else
        echo "start_x=1" >> /boot/config.txt
        echo "gpu_mem=128" >> /boot/config.txt
    fi
fi

# Enable serial port (disable console, enable UART)
if [ -f /boot/firmware/config.txt ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
    CMDLINE_FILE="/boot/firmware/cmdline.txt"
else
    CONFIG_FILE="/boot/config.txt"
    CMDLINE_FILE="/boot/cmdline.txt"
fi

if ! grep -q "^enable_uart=1" "$CONFIG_FILE"; then
    echo "enable_uart=1" >> "$CONFIG_FILE"
fi

# Disable serial console (so we can use it for MAVLink)
if [ -f "$CMDLINE_FILE" ]; then
    sed -i 's/console=serial0,[0-9]* //g' "$CMDLINE_FILE"
    sed -i 's/console=ttyAMA0,[0-9]* //g' "$CMDLINE_FILE"
fi

# Disable serial console service
systemctl disable serial-getty@ttyS0.service 2>/dev/null || true
systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true

# Add user to dialout group for serial access
usermod -a -G dialout $ACTUAL_USER
usermod -a -G video $ACTUAL_USER

# Clone or update the project
echo "[4/8] Setting up project..."
PROJECT_DIR="$ACTUAL_HOME/aruco_drone_nav"

if [ -d "$PROJECT_DIR" ]; then
    echo "Project directory exists, updating..."
    cd "$PROJECT_DIR"
    sudo -u $ACTUAL_USER git pull || true
else
    echo "Cloning project..."
    sudo -u $ACTUAL_USER git clone https://github.com/asdfgh0318/aruco_drone_nav.git "$PROJECT_DIR" || {
        echo "Git clone failed, creating directory for manual copy..."
        sudo -u $ACTUAL_USER mkdir -p "$PROJECT_DIR"
    }
fi

# Create virtual environment and install dependencies
echo "[5/8] Setting up Python environment..."
cd "$PROJECT_DIR"

# Create venv
sudo -u $ACTUAL_USER python3 -m venv venv --system-site-packages

# Install Python packages (use headless opencv for RPi)
sudo -u $ACTUAL_USER bash -c "source venv/bin/activate && pip install --upgrade pip"
sudo -u $ACTUAL_USER bash -c "source venv/bin/activate && pip install \
    pymavlink>=2.4.0 \
    PyYAML>=6.0 \
    reportlab>=4.0.0 \
    opencv-contrib-python-headless>=4.5.0"

# Create launcher script
echo "[6/8] Creating launcher scripts..."

cat > "$PROJECT_DIR/launch.sh" << 'LAUNCHER'
#!/bin/bash
# ArUco Drone Navigation Launcher
cd "$(dirname "$0")"
source venv/bin/activate
python3 -m src.main "$@"
LAUNCHER
chmod +x "$PROJECT_DIR/launch.sh"

cat > "$PROJECT_DIR/launch_gui.sh" << 'GUILAUNCHER'
#!/bin/bash
# ArUco Drone Navigation GUI Launcher
cd "$(dirname "$0")"
source venv/bin/activate
python3 tools/configurator_gui.py "$@"
GUILAUNCHER
chmod +x "$PROJECT_DIR/launch_gui.sh"

# Create systemd service for auto-start (optional)
echo "[7/8] Creating systemd service..."

cat > /etc/systemd/system/aruco-drone.service << SYSTEMD
[Unit]
Description=ArUco Vision GPS
After=network.target

[Service]
Type=simple
User=$ACTUAL_USER
WorkingDirectory=$PROJECT_DIR
ExecStart=$PROJECT_DIR/venv/bin/python3 -m src.main --config config/system_config.yaml --mode run
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
SYSTEMD

# Don't enable by default - user can enable manually
systemctl daemon-reload

# Create desktop shortcut (if desktop environment exists)
echo "[8/8] Creating shortcuts..."

if [ -d "$ACTUAL_HOME/Desktop" ]; then
    cat > "$ACTUAL_HOME/Desktop/ArUco-Drone.desktop" << DESKTOP
[Desktop Entry]
Name=ArUco Drone Nav
Comment=ArUco Drone Navigation System
Exec=$PROJECT_DIR/launch_gui.sh
Icon=applications-science
Terminal=false
Type=Application
Categories=Utility;
DESKTOP
    chmod +x "$ACTUAL_HOME/Desktop/ArUco-Drone.desktop"
    chown $ACTUAL_USER:$ACTUAL_USER "$ACTUAL_HOME/Desktop/ArUco-Drone.desktop"
fi

# Fix ownership
chown -R $ACTUAL_USER:$ACTUAL_USER "$PROJECT_DIR"

echo ""
echo "=============================================="
echo "  Setup Complete!"
echo "=============================================="
echo ""
echo "Project installed to: $PROJECT_DIR"
echo ""
echo "Quick commands:"
echo "  cd $PROJECT_DIR"
echo "  source venv/bin/activate"
echo ""
echo "  # Run GUI configurator:"
echo "  ./launch_gui.sh"
echo ""
echo "  # Run camera calibration:"
echo "  python3 tools/calibrate_camera.py"
echo ""
echo "  # Run ArUco detection test:"
echo "  python3 tools/test_aruco_detection.py"
echo ""
echo "  # Run bench test:"
echo "  python3 tools/bench_test.py"
echo ""
echo "  # Enable auto-start service:"
echo "  sudo systemctl enable aruco-drone.service"
echo ""
echo "IMPORTANT: Reboot required for hardware changes!"
echo "Run: sudo reboot"
echo ""
