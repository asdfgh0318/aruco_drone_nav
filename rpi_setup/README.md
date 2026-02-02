# Raspberry Pi Zero Setup

## Quick Start

### Option 1: Fresh Installation

1. **Flash Raspberry Pi OS** using Raspberry Pi Imager
   - Choose: Raspberry Pi OS Lite (64-bit) for RPi Zero 2W
   - Or: Raspberry Pi OS Lite (32-bit) for RPi Zero W
   - Configure WiFi and enable SSH in the imager settings

2. **Boot and SSH into the Pi**
   ```bash
   ssh pi@raspberrypi.local
   ```

3. **Run the setup script**
   ```bash
   curl -sSL https://raw.githubusercontent.com/asdfgh0318/aruco_drone_nav/main/rpi_setup/setup_rpi.sh | sudo bash
   ```

4. **Reboot**
   ```bash
   sudo reboot
   ```

### Option 2: Manual Installation

```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install dependencies
sudo apt-get install -y \
    python3-pip python3-venv python3-opencv python3-numpy \
    python3-yaml python3-tk git v4l-utils

# Clone project
git clone https://github.com/asdfgh0318/aruco_drone_nav.git
cd aruco_drone_nav

# Create venv with system packages (for opencv, tkinter)
python3 -m venv venv --system-site-packages

# Install additional Python packages
source venv/bin/activate
pip install pymavlink PyYAML reportlab

# Enable serial port for MAVLink
sudo raspi-config
# -> Interface Options -> Serial Port
# -> Login shell: No
# -> Hardware enabled: Yes

# Reboot
sudo reboot
```

## Hardware Connections

### Serial Connection (RPi to Flight Controller)

| RPi Pin | FC Pin | Description |
|---------|--------|-------------|
| GPIO 14 (TXD) | RX | RPi transmit → FC receive |
| GPIO 15 (RXD) | TX | RPi receive ← FC transmit |
| GND | GND | Common ground |

**Note:** Use a logic level converter if FC uses 5V logic!

### Camera

- USB camera: Just plug into USB port
- CSI camera: Connect to camera connector, enable in raspi-config

## Running the System

### GUI Configurator (if display connected)
```bash
cd ~/aruco_drone_nav
./launch_gui.sh
```

### Command Line Tools
```bash
cd ~/aruco_drone_nav
source venv/bin/activate

# Camera calibration
python3 tools/calibrate_camera.py

# ArUco detection test
python3 tools/test_aruco_detection.py

# Bench test (position + commands)
python3 tools/bench_test.py

# MAVLink test
python3 tools/test_mavlink.py

# Vision GPS test (prints position, no MAVLink)
python3 -m src.main --mode test

# Vision GPS run (sends position to FC via MAVLink)
python3 -m src.main --mode run
```

### Headless Operation (SSH)

For testing over SSH without display:

```bash
# Test camera detection (outputs to terminal)
python3 tools/test_aruco_detection.py 2>&1 | head -50

# Run with X11 forwarding (slow but works)
ssh -X pi@raspberrypi.local
python3 tools/bench_test.py
```

## Auto-Start Service

To run automatically on boot:

```bash
# Enable service
sudo systemctl enable aruco-drone.service

# Start now
sudo systemctl start aruco-drone.service

# Check status
sudo systemctl status aruco-drone.service

# View logs
journalctl -u aruco-drone.service -f
```

## Troubleshooting

### Camera not detected
```bash
# Check USB camera
v4l2-ctl --list-devices

# Check CSI camera
vcgencmd get_camera
```

### Serial port issues
```bash
# Check serial port
ls -la /dev/serial*
ls -la /dev/ttyS0 /dev/ttyAMA0

# Test serial (loopback)
# Short TX and RX pins, then:
echo "test" > /dev/serial0
cat /dev/serial0
```

### Permission denied
```bash
# Add user to groups
sudo usermod -a -G dialout,video $USER
# Then logout and login again
```

## Files

| File | Description |
|------|-------------|
| `setup_rpi.sh` | Automated setup script |
| `README.md` | This file |

## Tested On

- Raspberry Pi Zero 2W + Raspberry Pi OS Bookworm (64-bit)
- Raspberry Pi Zero W + Raspberry Pi OS Bullseye (32-bit)
- Raspberry Pi 4 + Raspberry Pi OS Bookworm (64-bit)
