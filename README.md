# ArUco Vision GPS

**Vision-based GPS emulator for indoor drones using ceiling-mounted ArUco markers**

A Python system for Raspberry Pi Zero that detects ArUco markers, calculates world-frame position, and sends it to an ArduCopter flight controller via MAVLink. The FC handles all navigation, PID control, missions, and failsafes natively.

## Architecture

```
Ceiling-mounted       USB Camera          RPi Zero 2W              Flight Controller
ArUco Markers    -->  (facing up)    -->  Vision GPS          -->  ArduCopter EKF
                                          - Detect markers         - Position estimation
                                          - Estimate position      - PID control
                                          - VISION_POSITION_       - Navigation
                                            ESTIMATE via MAVLink   - Missions & failsafes
```

## Hardware Requirements

- **Drone**: ArduCopter-compatible flight controller (Pixhawk, etc.)
- **Companion Computer**: Raspberry Pi Zero W/2W
- **Camera**: USB camera (facing upward toward ceiling)
- **Markers**: Printed ArUco markers (20cm recommended) mounted on ceiling
- **Connection**: UART serial between RPi and flight controller

## Quick Start

### Desktop Testing (Development Machine)

```bash
# Clone and setup
git clone https://github.com/asdfgh0318/aruco_drone_nav.git
cd aruco_drone_nav
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Generate test targets
python3 tools/generate_markers.py --ids 0,1,2,3,4 --size 20 --output markers/
python3 tools/generate_chessboard.py --output markers/

# Print markers/markers_DICT_6X6_250.pdf at 100% scale

# Calibrate camera
python3 tools/calibrate_camera.py --camera 0

# Test detection
python3 tools/test_aruco_detection.py --camera 0 --marker-size 0.20

# Run bench test (shows position + velocity commands)
python3 tools/bench_test.py --camera 0 --marker-size 0.20

# Launch GUI configurator
sudo apt-get install python3-tk  # if needed
python3 tools/configurator_gui.py
```

### Raspberry Pi Deployment

Two options available:

| Option | Best for | Image size | Boot time |
|--------|----------|------------|-----------|
| [Raspbian Setup](rpi_setup/README.md) | Quick testing | ~2GB | ~60s |
| [Buildroot Image](buildroot/README.md) | Production | ~100MB | ~5s |

**Option A: Standard Raspbian** (easy, for development)
```bash
# On fresh Raspberry Pi OS, run:
curl -sSL https://raw.githubusercontent.com/asdfgh0318/aruco_drone_nav/main/rpi_setup/setup_rpi.sh | sudo bash
sudo reboot
```

**Option B: Minimal Buildroot** (bulletproof, for production)
```bash
cd buildroot
./build.sh  # Takes 30-60 min first time
# Flash output image to SD card
```

See [docs/TESTING.md](docs/TESTING.md) for complete testing guide.

## Running the System

### Test Mode (no MAVLink, prints position to console)
```bash
python3 -m src.main --mode test --config config/system_config.yaml
```

### Run Mode (sends VISION_POSITION_ESTIMATE to FC)
```bash
python3 -m src.main --mode run --config config/system_config.yaml
```

## Tools

| Tool | Description | Usage |
|------|-------------|-------|
| `debug_gui.py` | **Debug GUI with live video, telemetry, marker map** | `python3 tools/debug_gui.py` |
| `camera_server.py` | MJPEG streaming server (runs on RPi) | `python3 tools/camera_server.py` |
| `calibrate_remote.py` | Network-based camera calibration | `python3 tools/calibrate_remote.py` |
| `calibrate_camera.py` | Local camera intrinsic calibration | `python3 tools/calibrate_camera.py` |
| `test_aruco_detection.py` | Live marker detection test | `python3 tools/test_aruco_detection.py` |
| `bench_test.py` | Position error + velocity commands | `python3 tools/bench_test.py` |
| `test_mavlink.py` | MAVLink connection test | `python3 tools/test_mavlink.py` |
| `generate_markers.py` | Create printable ArUco markers | `python3 tools/generate_markers.py` |
| `generate_chessboard.py` | Create calibration pattern | `python3 tools/generate_chessboard.py` |
| `configurator_gui.py` | GUI for testing/configuration | `python3 tools/configurator_gui.py` |

## Debug GUI (Network Streaming)

```
RPi (10.156.64.251)                    Local Machine
+------------------+                   +------------------+
| camera_server.py | ---- MJPEG ---->  | debug_gui.py     |
| - Capture frames |      Port 8000    | - Display video  |
| - JPEG compress  |                   | - Run detection  |
+------------------+                   | - Show telemetry |
                                       +------------------+
```

```bash
# On RPi: Start camera server
python3 tools/camera_server.py --port 8000

# On local machine: Launch debug GUI
python3 tools/debug_gui.py --host 10.156.64.251 --port 8000

# Remote calibration
python3 tools/calibrate_remote.py --host 10.156.64.251 --port 8000
```

## Configuration

### System Config (`config/system_config.yaml`)
```yaml
serial:
  port: "/dev/serial0"
  baud: 921600

camera:
  device_id: 0
  width: 640
  height: 480

aruco:
  dictionary: "DICT_6X6_250"
  marker_size_m: 0.20

control:
  loop_rate_hz: 20
```

### Marker Map (`config/marker_map.yaml`)
```yaml
markers:
  - id: 0
    position: [0.0, 0.0, 3.0]   # X, Y, Z (ceiling height)
    orientation: 0
  - id: 1
    position: [2.0, 0.0, 3.0]
    orientation: 0
```

## Project Structure

```
aruco_drone_nav/
+-- config/                     # Configuration files
+-- src/                        # Core system
|   +-- aruco_detector.py       # Marker detection + pose estimation
|   +-- position_estimator.py   # World position from markers
|   +-- mavlink_interface.py    # MAVLink (VISION_POSITION_ESTIMATE)
|   +-- camera_calibration.py   # Camera intrinsic calibration
|   +-- main.py                 # Vision GPS main loop
|   +-- deprecated/             # Old PID/mission/failsafe code
+-- tools/                      # Testing and utility tools
+-- buildroot/                  # Minimal Linux image builder
+-- rpi_setup/                  # Raspbian setup scripts
+-- markers/                    # Generated marker PDFs
+-- docs/                       # Documentation
```

## FC Configuration (ArduCopter)

To use vision position data, configure the flight controller:

```
VISO_TYPE = 1          # MAVLink vision
EK3_SRC1_POSXY = 6     # ExternalNav
EK3_SRC1_POSZ = 6      # ExternalNav
EK3_SRC1_YAW = 6       # ExternalNav (or 1 for compass)
GPS_TYPE = 0            # Disable GPS (indoor)
```

## Documentation

- **[TESTING.md](docs/TESTING.md)** - Complete testing guide
- **[TECHNICAL.md](docs/TECHNICAL.md)** - Technical details and algorithms
- **[CURRENT_PLAN.md](CURRENT_PLAN.md)** - Project roadmap
- **[rpi_setup/README.md](rpi_setup/README.md)** - Raspbian setup guide

## License

Proprietary - Warsaw University of Technology

## Authors

- Adam Koszalka
- Claude Code (AI Assistant)

---

*Last updated: 2026-02-02*
