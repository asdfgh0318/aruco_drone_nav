# ArUco Vision GPS

**Vision-based GPS emulator for indoor drones using ceiling-mounted ChArUco Diamond markers**

A Python system for Raspberry Pi Zero 2W that detects ChArUco Diamond markers, calculates world-frame position, and sends `VISION_POSITION_ESTIMATE` to an ArduCopter flight controller via MAVLink. The FC handles all navigation, PID control, missions, and failsafes natively.

Diamond markers provide better robustness than single ArUco markers due to redundancy (4 markers per diamond) and partial occlusion tolerance.

## Architecture

```
Ceiling-mounted       USB Camera          RPi Zero 2W              Flight Controller
ArUco Markers    -->  (facing up)    -->  Vision GPS          -->  ArduCopter EKF
                                          - Detect markers         - Position estimation
                                          - Estimate position      - PID control
                                          - ENU→NED conversion     - Navigation
                                          - VISION_POSITION_       - Missions & failsafes
                                            ESTIMATE via MAVLink
```

## Hardware Requirements

- **Drone**: ArduCopter-compatible flight controller (Pixhawk, etc.)
- **Companion Computer**: Raspberry Pi Zero W/2W
- **Camera**: USB camera (facing upward toward ceiling)
- **Markers**: Printed ChArUco Diamond markers (~64cm) mounted on ceiling
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

# Generate diamond markers and calibration board
python3 tools/generate_diamonds.py --ids "0_1_2_3,4_5_6_7" -o markers/
python3 tools/generate_charuco.py --squares 7x5 -o markers/

# Print at 100% scale (no fit-to-page)

# Calibrate camera (use ChArUco board)
python3 tools/calibrate_camera.py --camera 0

# Test diamond detection
python3 tools/test_aruco_detection.py --diamond --camera 0

# Run bench test (shows position + velocity commands)
python3 tools/bench_test.py --camera 0 --marker-size 0.20

# Launch GUI configurator
sudo apt-get install python3-tk  # if needed
python3 tools/configurator_gui.py
```

### SITL Testing (No Hardware Needed)

```bash
# Build ArduCopter SITL (one-time)
cd ~/ardupilot && ./waf configure --board sitl && ./waf build --target bin/arducopter

# Start SITL with vision params
cd /tmp && ~/ardupilot/build/sitl/bin/arducopter --model + --speedup 1 -I0 \
    --home 52.2297,21.0122,100,0 \
    --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm,config/sitl_params.parm

# Run validation (in another terminal)
python3 tools/test_sitl.py -v --skip-params           # Basic: stream + EKF convergence
python3 tools/test_sitl.py -v --skip-params --arm      # Arm, takeoff, hover, land
python3 tools/test_sitl.py -v --skip-params -p circle   # Circle pattern tracking
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
| `test_sitl.py` | **SITL validation** (params, EKF, arm, flight) | `python3 tools/test_sitl.py -v --skip-params` |
| `debug_gui.py` | **Debug GUI** with live video, telemetry, marker map | `python3 tools/debug_gui.py` |
| `bench_test.py` | Position error + velocity visualization | `python3 tools/bench_test.py` |
| `camera_server.py` | MJPEG streaming server (runs on RPi) | `python3 tools/camera_server.py` |
| `calibrate_remote.py` | Network-based camera calibration | `python3 tools/calibrate_remote.py` |
| `calibrate_camera.py` | Local camera intrinsic calibration | `python3 tools/calibrate_camera.py` |
| `test_aruco_detection.py` | Live marker detection test | `python3 tools/test_aruco_detection.py` |
| `test_mavlink.py` | MAVLink connection test | `python3 tools/test_mavlink.py` |
| `generate_diamonds.py` | Create ChArUco Diamond markers | `python3 tools/generate_diamonds.py --ids "0_1_2_3"` |
| `generate_charuco.py` | Create ChArUco calibration boards | `python3 tools/generate_charuco.py --squares 7x5` |
| `generate_markers.py` | Create single ArUco markers | `python3 tools/generate_markers.py` |
| `generate_chessboard.py` | Create chessboard calibration pattern | `python3 tools/generate_chessboard.py` |
| `marker_spacing.py` | Calculate marker spacing for room/camera | `python3 tools/marker_spacing.py` |
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
  dictionary: "DICT_4X4_50"

diamond:
  square_size_m: 0.20
  marker_size_m: 0.15

control:
  loop_rate_hz: 20
```

### SITL Config (`config/sitl_config.yaml`)
```yaml
serial:
  port: "tcp:127.0.0.1:5760"
  baud: 115200
```

### Marker Map (`config/marker_map.yaml`)
```yaml
diamonds:
  - id: "0_1_2_3"
    position: [0.0, 0.0, 3.0]   # X, Y, Z (ceiling height)
    orientation: 0
  - id: "4_5_6_7"
    position: [2.0, 0.0, 3.0]
    orientation: 0
```

## Project Structure

```
aruco_drone_nav/
+-- config/                     # Configuration files
|   +-- system_config.yaml      # Main system config (serial, camera, aruco)
|   +-- camera_params.yaml      # Camera intrinsic calibration
|   +-- marker_map.yaml         # Marker world positions
|   +-- sitl_config.yaml        # SITL testing config
|   +-- sitl_params.parm        # ArduCopter params for vision flight
+-- src/                        # Core system
|   +-- main.py                 # Vision GPS main loop (detect→estimate→send)
|   +-- aruco_detector.py       # Diamond/ArUco detection + pose estimation
|   +-- position_estimator.py   # World position from markers (multi-marker fusion)
|   +-- mavlink_interface.py    # MAVLink (VISION_POSITION_ESTIMATE)
|   +-- camera_calibration.py   # Camera intrinsic calibration
|   +-- deprecated/             # Old PID/mission/failsafe code
+-- tools/                      # Testing and utility tools
+-- buildroot/                  # Minimal Linux image builder
+-- rpi_setup/                  # Raspbian setup scripts
+-- markers/                    # Generated marker PDFs
+-- docs/                       # Documentation (HTML site + markdown)
```

## FC Configuration (ArduCopter)

To use vision position data, configure the flight controller:

```
AHRS_EKF_TYPE = 3      # Use EKF3
EK3_SRC1_POSXY = 6     # ExternalNav for XY position
EK3_SRC1_POSZ = 1      # Baro for altitude (safer indoors)
EK3_SRC1_YAW = 6       # ExternalNav for yaw
VISO_TYPE = 1           # MAVLink vision
GPS_TYPE = 0            # Disable GPS (indoor)
COMPASS_ENABLE = 0      # Disable compass (indoor)
```

See [docs/FC_CONFIG.md](docs/FC_CONFIG.md) for full parameter list and explanation.

A ready-to-use parameter file is provided: `config/sitl_params.parm`

## SITL Validation Results

The full vision-to-FC pipeline has been validated in ArduCopter SITL:

| Test | Result | Position Error |
|------|--------|---------------|
| Basic (params + EKF convergence) | 6/6 PASS | 0.004m |
| Arm + guided flight (takeoff, hover, land) | 5/5 PASS | 0.003m |
| Circle pattern (1.5m radius, 20s) | 4/4 PASS | 0.189m |

See [docs/SITL_RESULTS.md](docs/SITL_RESULTS.md) for full results and reproduction steps.

## Documentation

- **[HTML Docs](docs/index.html)** - Open `docs/index.html` in a browser (`xdg-open docs/index.html`)
- **[FC_CONFIG.md](docs/FC_CONFIG.md)** - Flight controller configuration guide
- **[SITL_RESULTS.md](docs/SITL_RESULTS.md)** - SITL test results and reproduction
- **[PAWEL_SUMMARY.md](docs/PAWEL_SUMMARY.md)** - Technical architecture summary
- **[TESTING.md](docs/TESTING.md)** - Complete testing guide
- **[TECHNICAL.md](docs/TECHNICAL.md)** - Technical details and algorithms
- **[CURRENT_PLAN.md](CURRENT_PLAN.md)** - Project roadmap

## License

Proprietary - Warsaw University of Technology

## Authors

- Adam Koszalka
- Claude Code (AI Assistant)

---

*Last updated: 2026-02-03*
