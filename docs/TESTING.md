# Testing Guide

Complete guide for testing the ArUco Drone Navigation System.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Desktop Testing](#desktop-testing)
3. [Raspberry Pi Testing](#raspberry-pi-testing)
4. [Hardware Testing](#hardware-testing)
5. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Print Test Targets

Before any testing, print these files from the `markers/` folder:

| File | What it is | Print settings |
|------|------------|----------------|
| `markers_DICT_6X6_250.pdf` | ArUco markers (IDs 0-4) | **100% scale**, no fit-to-page |
| `chessboard_9x6.pdf` | Calibration pattern | **100% scale**, no fit-to-page |

**Important:** Measure the printed markers to verify size:
- ArUco markers should be **20cm × 20cm**
- Chessboard squares should be **~19mm × 19mm**

If sizes don't match, adjust `--marker-size` parameter accordingly.

### Hardware Checklist

- [ ] USB camera (any webcam works for testing)
- [ ] Printed ArUco markers (at least 1)
- [ ] Printed chessboard pattern
- [ ] Good lighting (avoid direct sunlight/shadows on markers)

---

## Desktop Testing

Test everything on your development machine before deploying to RPi.

### Step 1: Setup Environment

```bash
cd /home/adam-koszalka/ŻYCIE/PRACA/aruco_drone_nav

# Create virtual environment (first time only)
python3 -m venv venv

# Activate environment
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Install tkinter for GUI (system package)
sudo apt-get install python3-tk
```

### Step 2: Generate Test Targets (if not already done)

```bash
source venv/bin/activate

# Generate ArUco markers
python3 tools/generate_markers.py --ids 0,1,2,3,4 --size 20 --output markers/

# Generate chessboard
python3 tools/generate_chessboard.py --width 9 --height 6 --square-size 25 --output markers/
```

### Step 3: Camera Calibration

Calibration improves pose estimation accuracy. Do this once per camera.

```bash
source venv/bin/activate
python3 tools/calibrate_camera.py --camera 0
```

**Instructions:**
1. Hold the printed chessboard in front of the camera
2. Press **SPACE** to capture when corners are detected (green overlay)
3. Move the chessboard to different angles/distances between captures
4. Capture **15 images** from various positions
5. Calibration file saves to `config/camera_params.yaml`

**Tips for good calibration:**
- Cover the entire camera frame
- Include tilted angles (not just flat)
- Vary the distance (close and far)
- Ensure even lighting

### Step 4: ArUco Detection Test

Test that markers are detected correctly.

```bash
source venv/bin/activate
python3 tools/test_aruco_detection.py --camera 0 --marker-size 0.20
```

**What to verify:**
- [ ] Marker is detected (green border drawn)
- [ ] Marker ID is correct
- [ ] Position values change when you move the marker
- [ ] FPS is reasonable (>15 FPS)

**Controls:**
- `Q` - Quit
- `S` - Save screenshot
- `R` - Reset statistics

### Step 5: Bench Test (Position + Commands)

Full test showing position error and velocity commands.

```bash
source venv/bin/activate
python3 tools/bench_test.py --camera 0 --marker-size 0.20
```

**What to verify:**
- [ ] Position offset shows (Fwd, Right, Alt)
- [ ] Velocity commands respond to marker movement
- [ ] Visual velocity indicator works (circle + bar)
- [ ] Moving marker left → positive Vy command
- [ ] Moving marker up → positive Vx command
- [ ] Moving marker closer → negative Vz command

**Understanding the display:**
```
┌─────────────────────────────┐
│ Marker ID: 0                │
│ Position Offset:            │
│   Fwd:   +0.150 m          │  ← Marker is 15cm ahead
│   Right: -0.080 m          │  ← Marker is 8cm to left
│   Alt:   1.200 m           │  ← Distance to marker
│   Yaw:   +5.2 deg          │  ← Rotation angle
│                             │
│ Velocity Commands:          │
│   Vx: -0.075 m/s           │  ← Command to move backward
│   Vy: +0.040 m/s           │  ← Command to move right
│   Vz: -0.100 m/s           │  ← Command to descend
└─────────────────────────────┘
```

### Step 6: GUI Configurator (Optional)

Launch the graphical interface.

```bash
source venv/bin/activate
python3 tools/configurator_gui.py
```

**Features:**
- Adjust camera/marker settings
- One-click launch of all test tools
- Save/load configuration
- View output logs

---

## Raspberry Pi Testing

### Option A: Standard Raspbian Setup

For quick testing with full Raspbian OS.

#### On Your PC:

1. Flash **Raspberry Pi OS Lite** using Raspberry Pi Imager
2. Configure WiFi and enable SSH in the imager
3. Insert SD card into RPi and boot

#### On the RPi (via SSH):

```bash
# Connect
ssh pi@raspberrypi.local

# Run setup script
curl -sSL https://raw.githubusercontent.com/asdfgh0318/aruco_drone_nav/main/rpi_setup/setup_rpi.sh | sudo bash

# Reboot
sudo reboot

# After reboot, reconnect and test
ssh pi@raspberrypi.local
cd ~/aruco_drone_nav
source venv/bin/activate

# Test camera
python3 tools/test_aruco_detection.py --camera 0
```

### Option B: Minimal Buildroot Image

For production-ready minimal system.

#### Build the Image (on your PC):

```bash
cd buildroot

# Install build dependencies
sudo apt-get install build-essential wget cpio unzip rsync bc libncurses-dev

# Configure WiFi BEFORE building
nano overlay/etc/wpa_supplicant/wpa_supplicant.conf
# Add your network:
# network={
#     ssid="YourWiFi"
#     psk="YourPassword"
# }

# Build (takes 30-60 minutes first time)
./build.sh
```

#### Flash and Boot:

```bash
# Find SD card device
lsblk

# Flash (CAREFUL: replace sdX with actual device like sdb)
sudo dd if=build/buildroot-*/output/images/sdcard.img of=/dev/sdX bs=4M status=progress conv=fsync
sync

# Eject and insert into RPi
```

#### Test on Buildroot:

```bash
# SSH into the system
ssh root@aruco-drone.local
# Password: aruco

# Check application status
/etc/init.d/S99aruco status

# View logs
cat /var/log/aruco/aruco.log

# Stop auto-start to run manually
/etc/init.d/S99aruco stop

# Run manually
cd /opt/aruco
python3 -m src.main --mode ground_test
```

---

## Hardware Testing

### Camera Test

```bash
# List available cameras
v4l2-ctl --list-devices

# Test camera capture
v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=10
```

### Serial Port Test (MAVLink)

```bash
# Check serial port exists
ls -la /dev/serial0 /dev/ttyS0 /dev/ttyAMA0

# Simple loopback test (connect TX to RX)
echo "test" > /dev/serial0 &
cat /dev/serial0
```

### MAVLink Connection Test

```bash
source venv/bin/activate
python3 tools/test_mavlink.py --port /dev/serial0 --baud 921600
```

**Without flight controller**, use SITL:
```bash
# On another terminal, run ArduCopter SITL
sim_vehicle.py -v ArduCopter --console

# Then connect
python3 tools/test_mavlink.py --port udp:127.0.0.1:14550
```

### Full System Test (Ground Test Mode)

Test everything together without sending actual flight commands.

```bash
source venv/bin/activate
python3 -m src.main --mode ground_test --config config/system_config.yaml
```

**What to verify:**
- [ ] Camera initializes
- [ ] Markers are detected
- [ ] Position is calculated
- [ ] Velocity commands are computed (displayed, not sent)
- [ ] No errors in console

---

## Test Checklist

### Desktop Tests
- [ ] Virtual environment created
- [ ] Dependencies installed
- [ ] Markers printed and measured
- [ ] Camera calibration completed
- [ ] ArUco detection working
- [ ] Bench test showing correct commands
- [ ] GUI configurator launches

### RPi Tests
- [ ] RPi boots successfully
- [ ] WiFi connects
- [ ] SSH access works
- [ ] Camera detected (`/dev/video0`)
- [ ] Serial port available (`/dev/serial0`)
- [ ] ArUco detection works on RPi
- [ ] Application auto-starts (Buildroot only)

### Integration Tests
- [ ] MAVLink connects to FC/SITL
- [ ] Telemetry received
- [ ] Ground test mode runs without errors
- [ ] Watchdog recovers from crash (Buildroot only)

---

## Troubleshooting

### Camera Issues

**"No camera found"**
```bash
# Check if camera is detected
ls /dev/video*
v4l2-ctl --list-devices

# Check permissions
sudo usermod -a -G video $USER
# Logout and login again
```

**"Camera busy"**
```bash
# Find process using camera
fuser /dev/video0

# Kill it
sudo fuser -k /dev/video0
```

**Low FPS**
- Reduce resolution: `--width 320 --height 240`
- Check CPU usage: `htop`
- Close other applications

### Detection Issues

**Markers not detected**
- Check lighting (avoid shadows, reflections)
- Verify marker size parameter matches actual size
- Try closer/farther distance
- Check marker is not damaged/wrinkled
- Verify correct dictionary (`DICT_6X6_250`)

**Wrong position values**
- Recalibrate camera
- Verify marker size parameter
- Check marker is flat (not bent)

### Serial/MAVLink Issues

**"Permission denied"**
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

**"Connection timeout"**
- Check wiring (TX→RX, RX→TX)
- Verify baud rate matches FC setting
- Check FC is powered and running

### Buildroot Issues

**Won't boot**
- Reflash SD card
- Try different SD card
- Check power supply (needs stable 5V 2A)

**Application crashes**
```bash
# Check logs
cat /var/log/aruco/aruco.log
cat /var/log/aruco/error.log

# Check watchdog
dmesg | grep watchdog
```

**Need to modify files on read-only system**
```bash
# Temporarily make writable
mount -o remount,rw /

# Make changes...

# Make read-only again
mount -o remount,ro /
```

---

## Next Steps

After successful testing:

1. **Tune PID gains** in `config/system_config.yaml`
2. **Create marker map** for your ceiling layout
3. **Test with SITL** before real flight
4. **Tethered flight test** with safety line
5. **Free flight test** in safe environment
