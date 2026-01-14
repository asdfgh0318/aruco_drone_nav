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

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/asdfgh0318/aruco_drone_nav.git
cd aruco_drone_nav
```

### 2. Install Dependencies
```bash
# On development machine
pip install -r requirements.txt

# On Raspberry Pi Zero (use headless OpenCV)
pip install opencv-python-headless opencv-contrib-python-headless
pip install pymavlink numpy PyYAML reportlab
```

### 3. Configure System
Edit `config/system_config.yaml`:
```yaml
serial:
  port: "/dev/serial0"    # RPi UART
  baud: 921600

camera:
  device_id: 0
  width: 640
  height: 480

aruco:
  dictionary: "DICT_6X6_250"
  marker_size_m: 0.20     # 20cm markers
```

## Quick Start

### Step 1: Calibrate Camera
```bash
python tools/calibrate_camera.py --camera 0 --output config/camera_params.yaml
```
Present a chessboard pattern at various angles. Minimum 15 images recommended.

### Step 2: Generate Markers
```bash
python tools/generate_markers.py --ids 0,1,2,3,4 --size 20 --output markers/
```
Print the PDF at 100% scale (20cm x 20cm) and mount flat on ceiling.

### Step 3: Configure Marker Map
Edit `config/marker_map.yaml` with actual marker positions:
```yaml
markers:
  - id: 0
    position: [0.0, 0.0, 3.0]   # X, Y, Z in meters (ceiling height)
    orientation: 0
  - id: 1
    position: [2.0, 0.0, 3.0]
    orientation: 0
```

### Step 4: Test Detection
```bash
python tools/test_aruco_detection.py
```

### Step 5: Test MAVLink Connection
```bash
python tools/test_mavlink.py --port /dev/serial0 --baud 921600
```

## Usage

### Ground Test Mode (No Flight)
Test detection and control calculations without arming:
```bash
python -m src.main --mode ground_test --config config/system_config.yaml
```

### Phase 1: Single Marker Hover
Hover under a single marker with position hold:
```bash
python -m src.main --mode hover --altitude 1.5
```

### Phase 2: Mission Execution
Execute a mission from JSON file:
```bash
python -m src.main --mode mission --mission missions/sample_square.json
```

With flight recording disabled:
```bash
python -m src.main --mode mission --mission missions/sample_square.json --no-record
```

## Mission File Format

JSON format compatible with VR planning software:
```json
{
  "mission_id": "flight_001",
  "waypoints": [
    {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0, "hold_time": 2.0, "name": "Start"},
    {"x": 1.5, "y": 0.0, "z": 1.5, "yaw": 0, "hold_time": 1.0, "name": "Point A"},
    {"x": 1.5, "y": 1.5, "z": 1.5, "yaw": 90, "hold_time": 1.0, "name": "Point B"}
  ],
  "settings": {
    "max_speed": 0.3,
    "position_tolerance": 0.15,
    "takeoff_altitude": 1.0
  }
}
```

## Flight Recording Output

Recorded flights are saved to `recordings/` in JSON format:
```json
{
  "recording_id": "flight_20240114_103000",
  "mission_id": "sample_square_001",
  "samples": [
    {"t": 0.0, "x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0, "markers": [0]},
    {"t": 0.1, "x": 0.05, "y": 0.01, "z": 1.52, "yaw": 1, "markers": [0]}
  ]
}
```

This format can be reimported to VR software for flight path iteration.

## Project Structure

```
aruco_drone_nav/
├── config/
│   ├── camera_params.yaml       # Camera calibration
│   ├── marker_map.yaml          # Marker world positions
│   └── system_config.yaml       # System configuration
├── src/
│   ├── __init__.py
│   ├── camera_calibration.py    # Chessboard calibration
│   ├── aruco_detector.py        # Marker detection + pose
│   ├── position_estimator.py    # World position estimation
│   ├── mavlink_interface.py     # ArduCopter communication
│   ├── mission_executor.py      # Waypoint navigation
│   ├── flight_recorder.py       # VR-compatible recording
│   ├── position_predictor.py    # Dead reckoning
│   └── main.py                  # Main control loop
├── tools/
│   ├── calibrate_camera.py      # Camera calibration CLI
│   ├── test_aruco_detection.py  # Detection test
│   ├── test_mavlink.py          # FC connection test
│   └── generate_markers.py      # Marker PDF generator
├── missions/                    # Mission JSON files
├── recordings/                  # Flight recordings
├── docs/                        # Documentation
└── requirements.txt
```

## Safety Considerations

1. **Always test in SITL first** - Use ArduCopter Software-In-The-Loop simulator
2. **Tethered tests** - Use safety tether for initial real hardware flights
3. **Marker loss failsafe** - System lands if markers lost for >2 seconds
4. **Battery monitoring** - Auto-land on low battery
5. **RC override** - Always have RC transmitter ready for manual control
6. **Geofence** - Software limits maximum distance from origin

## Troubleshooting

### Camera not detected
```bash
# List video devices
ls /dev/video*
# Test camera
python -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

### MAVLink connection failed
- Check serial port permissions: `sudo usermod -a -G dialout $USER`
- Verify baud rate matches flight controller
- Check UART is enabled on RPi: `sudo raspi-config`

### Poor marker detection
- Ensure markers are flat and well-lit
- Check camera focus
- Verify marker size in config matches physical size
- Run calibration with more images

## Contributing

This is a private repository for drone navigation research. Contact the maintainers for access.

## License

Proprietary - Warsaw University of Technology

## Authors

- Adam Koszałka
- Claude Code (AI Assistant)

---

*Generated: 2024-01-14*
