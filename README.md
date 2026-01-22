# ArUco Drone Navigation System

**Vision-based indoor drone navigation using ceiling-mounted ArUco markers**

A Python-based navigation system for Raspberry Pi Zero that enables autonomous indoor drone flight using ArUco marker detection, MAVLink communication with ArduCopter, and mission execution from VR-generated waypoints.

## Overview

This system provides:
- **Phase 1**: Single marker hover control (position hold under one marker)
- **Phase 2**: Multi-marker waypoint navigation with JSON mission files
- **VR Integration**: Accepts waypoints from VR planning software, records flight paths for iteration
- **Safety Features**: Marker loss prediction, failsafe landing, battery monitoring

### System Architecture

```
┌─────────────────┐     JSON Mission      ┌─────────────────┐
│   VR Planning   │ ──────────────────►   │                 │
│     Software    │                       │    RPi Zero     │
│                 │ ◄──────────────────   │   (onboard)     │
└─────────────────┘   Flight Recording    └────────┬────────┘
                                                   │ UART/Serial
                                                   │ MAVLink
┌─────────────────┐                       ┌────────▼────────┐
│  Ceiling-mounted│     USB Camera        │   ArduCopter    │
│  ArUco Markers  │ ◄──── (facing up) ────│      FC         │
└─────────────────┘                       └─────────────────┘
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

The Debug GUI provides real-time visualization when testing with the RPi remotely.

### Architecture
```
RPi (10.156.64.251)                    Local Machine
┌─────────────────┐                    ┌─────────────────┐
│ camera_server.py│ ──── MJPEG ─────>  │ debug_gui.py    │
│ - Capture frames│      Port 8000     │ - Display video │
│ - JPEG compress │                    │ - Run detection │
└─────────────────┘                    │ - Show telemetry│
                                       └─────────────────┘
```

### Usage
```bash
# On RPi: Start camera server
python3 tools/camera_server.py --port 8000

# On local machine: Launch debug GUI
python3 tools/debug_gui.py --host 10.156.64.251 --port 8000
```

### Features
- **Live Video**: Real-time stream with marker detection overlay
- **Telemetry Panel**: Marker IDs, distances, positions, yaw angles
- **Marker Map**: Top-down visualization of detected markers
- **Recording**: Save trajectory and marker positions to JSON

### Remote Calibration
```bash
# Calibrate camera over network (show chessboard to camera)
python3 tools/calibrate_remote.py --host 10.156.64.251 --port 8000
```

## Running the System

### Ground Test Mode (No Flight)
Test detection and control calculations without arming:
```bash
python3 -m src.main --mode ground_test --config config/system_config.yaml
```

### Phase 1: Single Marker Hover
Hover under a single marker with position hold:
```bash
python3 -m src.main --mode hover --altitude 1.5
```

### Phase 2: Mission Execution
Execute a mission from JSON file:
```bash
python3 -m src.main --mode mission --mission missions/sample_square.json
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

## Mission File Format

JSON format compatible with VR planning software:
```json
{
  "mission_id": "flight_001",
  "waypoints": [
    {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0, "hold_time": 2.0},
    {"x": 1.5, "y": 0.0, "z": 1.5, "yaw": 0, "hold_time": 1.0}
  ],
  "settings": {
    "max_speed": 0.3,
    "position_tolerance": 0.15
  }
}
```

## Project Structure

```
aruco_drone_nav/
├── config/                     # Configuration files
├── src/                        # Core navigation code
│   ├── aruco_detector.py       # Marker detection
│   ├── position_estimator.py   # World position estimation
│   ├── mavlink_interface.py    # Flight controller communication
│   ├── mission_executor.py     # Waypoint navigation
│   └── main.py                 # Main control loop
├── tools/                      # Testing and utility tools
│   ├── debug_gui.py            # Debug GUI (video, telemetry, map)
│   ├── camera_server.py        # RPi MJPEG streaming server
│   ├── calibrate_remote.py     # Network-based calibration
│   ├── bench_test.py           # Position/command visualization
│   ├── configurator_gui.py     # GUI configurator
│   └── ...
├── buildroot/                  # Minimal Linux image builder
├── rpi_setup/                  # Raspbian setup scripts
├── markers/                    # Generated marker PDFs
├── missions/                   # Mission JSON files
├── recordings/                 # Flight recordings
└── docs/                       # Documentation
    ├── TESTING.md              # Complete testing guide
    ├── TECHNICAL.md            # Technical documentation
    └── PLAN_v1.md              # Implementation plan
```

## Documentation

- **[TESTING.md](docs/TESTING.md)** - Complete testing guide with checklists
- **[TECHNICAL.md](docs/TECHNICAL.md)** - Technical details and algorithms
- **[CLAUDE.md](CLAUDE.md)** - AI session resume instructions
- **[buildroot/README.md](buildroot/README.md)** - Minimal Linux image
- **[rpi_setup/README.md](rpi_setup/README.md)** - Raspbian setup guide

## Safety Considerations

1. **Always test in SITL first** - Use ArduCopter Software-In-The-Loop simulator
2. **Tethered tests** - Use safety tether for initial real hardware flights
3. **Marker loss failsafe** - System lands if markers lost for >2 seconds
4. **Battery monitoring** - Auto-land on low battery
5. **RC override** - Always have RC transmitter ready for manual control
6. **Watchdog** - Buildroot image includes hardware watchdog for auto-recovery

## Troubleshooting

See [docs/TESTING.md](docs/TESTING.md) for detailed troubleshooting guide.

### Quick Fixes

```bash
# Camera not detected
ls /dev/video*
sudo usermod -a -G video $USER

# Serial permission denied
sudo usermod -a -G dialout $USER

# Poor marker detection
# - Ensure markers are flat and well-lit
# - Verify --marker-size matches actual printed size
# - Run camera calibration
```

## License

Proprietary - Warsaw University of Technology

## Authors

- Adam Koszałka
- Claude Code (AI Assistant)

---

*Last updated: 2026-01-23*
