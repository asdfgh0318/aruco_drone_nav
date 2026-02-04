# ArUco Vision GPS

**Vision-based GPS emulator for indoor drones using ceiling-mounted ArUco markers**

A Python system for Raspberry Pi Zero 2W that detects ArUco markers on the ceiling, calculates world-frame position, and sends `VISION_POSITION_ESTIMATE` to an ArduCopter flight controller via MAVLink. The FC handles all navigation, PID control, missions, and failsafes natively.

## Architecture

```
Ceiling-mounted       USB Camera          RPi Zero 2W              Flight Controller
ArUco Markers    -->  (facing up)    -->  Vision GPS          -->  ArduCopter EKF
                                          - CLAHE preprocessing    - Position estimation
                                          - Detect markers         - PID control
                                          - Estimate position      - Navigation
                                          - ENU->NED conversion    - Missions & failsafes
                                          - VISION_POSITION_
                                            ESTIMATE via MAVLink
```

## Current Performance (RPi Zero 2W)

| Metric | Value |
|--------|-------|
| Resolution | 1280x720 (MJPG) |
| Detection Rate | 95-100% |
| Processing Time | ~270ms/frame |
| FPS | ~3.7 |
| Marker Size | 18cm (A4 printable) |
| Working Distance | 1.5-2.5m |

### Timing Breakdown
```
grab:0ms  gray:3ms  CLAHE:20ms  bgr:2ms  detect:250ms  total:275ms
```

## Hardware Requirements

- **Drone**: ArduCopter-compatible flight controller (Pixhawk, etc.)
- **Companion Computer**: Raspberry Pi Zero 2W
- **Camera**: USB camera (facing upward toward ceiling), MJPG capable
- **Markers**: Printed ArUco markers (18cm, DICT_4X4_50) on A4 paper
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

# Generate ArUco markers (A4 size, 18cm)
python3 tools/generate_markers.py --ids 0,1,2,3 --size 180 -o markers/

# Print at 100% scale (no fit-to-page)

# Calibrate camera (use ChArUco board)
python3 tools/calibrate_camera.py --camera 0

# Test marker detection
python3 tools/test_aruco_detection.py --camera 0
```

### Raspberry Pi Deployment

```bash
# Sync code to RPi
./sync_to_rpi.sh

# SSH to RPi
ssh aruconav@aruconav.local

# Run test mode (prints position to console)
cd /home/aruconav/aruco_drone_nav
python3 -m src.main --mode test

# Run stream mode (HTTP server for debug viewer)
python3 -m src.main --mode stream
```

### Debug Viewer (Local Machine)

```bash
# Start debug viewer (connects to RPi stream)
python3 tools/debug_viewer.py --host aruconav.local --port 8001

# Press SPACE to capture frames
# Shows: position, detection rate, timing breakdown
```

## Running Modes

### Test Mode (no MAVLink, console output)
```bash
python3 -m src.main --mode test --config config/system_config.yaml
```

### Stream Mode (HTTP server for debug viewer)
```bash
python3 -m src.main --mode stream --port 8001
```

### Run Mode (sends VISION_POSITION_ESTIMATE to FC)
```bash
python3 -m src.main --mode run --config config/system_config.yaml
```

## Tools

| Tool | Description | Usage |
|------|-------------|-------|
| `debug_viewer.py` | Manual frame capture with timing | `python3 tools/debug_viewer.py` |
| `generate_markers.py` | Create ArUco marker PDFs | `python3 tools/generate_markers.py --ids 0,1,2` |
| `calibrate_camera.py` | Local camera calibration | `python3 tools/calibrate_camera.py` |
| `calibrate_remote.py` | Network-based calibration | `python3 tools/calibrate_remote.py --host rpi` |
| `camera_server.py` | MJPEG streaming server | `python3 tools/camera_server.py` |
| `test_aruco_detection.py` | Live detection test | `python3 tools/test_aruco_detection.py` |
| `test_mavlink.py` | MAVLink connection test | `python3 tools/test_mavlink.py` |

## Configuration

### System Config (`config/system_config.yaml`)
```yaml
camera:
  device_id: 0
  width: 1280
  height: 720
  fps: 30
  # exposure_time_us: 5000  # Uncomment for manual exposure

aruco:
  dictionary: "DICT_4X4_50"
  marker_size_m: 0.18      # 18cm for A4 with margins

control:
  loop_rate_hz: 20
```

### Marker Map (`config/marker_map.yaml`)
```yaml
markers:
  - id: 0
    position: [0.0, 0.0, 3.0]   # X, Y, Z (ceiling height)
    orientation: 0
    description: "Origin marker - above takeoff"
```

## Detection Pipeline

1. **Frame Capture**: Threaded capture from USB camera (MJPG format)
2. **CLAHE Preprocessing**: Adaptive histogram equalization for varying lighting
3. **ArUco Detection**: OpenCV ArUco detector with tuned parameters
4. **Pose Estimation**: solvePnP for 6-DOF marker pose
5. **Position Estimation**: Transform marker pose to world coordinates
6. **MAVLink Output**: Send VISION_POSITION_ESTIMATE to FC

### Detection Parameters (tuned for ceiling markers)
```python
adaptiveThreshWinSizeMin = 3
adaptiveThreshWinSizeMax = 23
adaptiveThreshWinSizeStep = 10
minMarkerPerimeterRate = 0.01   # Detect distant markers
cornerRefinementMethod = CORNER_REFINE_CONTOUR
```

## Project Structure

```
aruco_drone_nav/
+-- config/                     # Configuration files
|   +-- system_config.yaml      # Camera, ArUco, control settings
|   +-- camera_params.yaml      # Camera calibration (720p)
|   +-- marker_map.yaml         # Marker world positions
+-- src/                        # Core system
|   +-- main.py                 # Vision GPS main loop
|   +-- aruco_detector.py       # Detection + CLAHE + timing
|   +-- position_estimator.py   # World position calculation
|   +-- mavlink_interface.py    # MAVLink communication
|   +-- camera_calibration.py   # Calibration utilities
+-- tools/                      # Development tools
|   +-- debug_viewer.py         # Manual frame capture + timing
|   +-- generate_markers.py     # ArUco PDF generator
|   +-- calibrate_remote.py     # Network calibration
+-- markers/                    # Generated marker PDFs
+-- docs/                       # Documentation
```

## FC Configuration (ArduCopter)

```
AHRS_EKF_TYPE = 3      # Use EKF3
EK3_SRC1_POSXY = 6     # ExternalNav for XY position
EK3_SRC1_POSZ = 1      # Baro for altitude (safer indoors)
EK3_SRC1_YAW = 6       # ExternalNav for yaw
VISO_TYPE = 1          # MAVLink vision
GPS_TYPE = 0           # Disable GPS (indoor)
COMPASS_ENABLE = 0     # Disable compass (indoor)
```

See [docs/FC_CONFIG.md](docs/FC_CONFIG.md) for full parameter list.

## Known Issues

- **OpenCV CORNER_REFINE_CONTOUR crash**: Rare assertion error in OpenCV's contour refinement. Handled gracefully by skipping bad frames.
- **Processing speed**: ~3.7 FPS on RPi Zero 2W at 1280x720. Consider 640x480 for higher FPS.

## Documentation

- **[FC_CONFIG.md](docs/FC_CONFIG.md)** - Flight controller configuration
- **[TECHNICAL.md](docs/TECHNICAL.md)** - Technical details and algorithms
- **[TESTING.md](docs/TESTING.md)** - Testing guide
- **[CURRENT_PLAN.md](CURRENT_PLAN.md)** - Project roadmap

## License

Proprietary - Warsaw University of Technology

## Authors

- Adam Koszalka
- Claude Code (AI Assistant)

---

*Last updated: 2026-02-04*
