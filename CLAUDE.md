# Claude Code Resume Instructions

This document provides context for AI assistants continuing work on this project.

## Project Overview

**ArUco Vision GPS** - Vision-based GPS emulator for indoor drones using ceiling-mounted ArUco markers on Raspberry Pi Zero 2W. The RPi detects markers, calculates world-frame position, and sends VISION_POSITION_ESTIMATE to the flight controller via MAVLink. The FC handles all navigation, PID, missions, and failsafes.

**Project Location**: `/home/adam/ŻYCIE/PRACA/aruco_drone_nav`

## Architecture

```
RPi Zero 2W (companion)              Flight Controller
+---------------------------+        +------------------+
| USB Camera (facing up)    |        |                  |
|     |                     |        | ArduCopter EKF   |
|     v                     |        | - Position est.  |
| ArUco Detection           |  UART  | - PID control    |
|     |                     | -----> | - Navigation     |
|     v                     | MAVLink| - Missions       |
| Position Estimation       |        | - Failsafes      |
|     |                     |        |                  |
|     v                     |        |                  |
| VISION_POSITION_ESTIMATE  |        |                  |
+---------------------------+        +------------------+
```

## Key Files

| File | Purpose |
|------|---------|
| `src/main.py` | Vision GPS main loop (detect → estimate → send) |
| `src/aruco_detector.py` | Marker detection with OpenCV |
| `src/position_estimator.py` | World-frame position from markers |
| `src/mavlink_interface.py` | MAVLink communication (VISION_POSITION_ESTIMATE) |
| `src/camera_calibration.py` | Camera intrinsic calibration |
| `tools/debug_gui.py` | Debug GUI (runs on local machine) |
| `tools/camera_server.py` | MJPEG streaming (runs on RPi) |
| `tools/calibrate_remote.py` | Network calibration tool |
| `config/system_config.yaml` | System configuration |
| `config/camera_params.yaml` | Camera calibration data |
| `config/marker_map.yaml` | Marker world positions |
| `CURRENT_PLAN.md` | Living project roadmap |

## Quick Commands

### Run Vision GPS (on RPi)
```bash
# Test mode (prints position, no MAVLink)
python3 -m src.main --mode test --config config/system_config.yaml

# Run mode (sends to FC)
python3 -m src.main --mode run --config config/system_config.yaml
```

### Connect to RPi
```bash
ssh pi@10.156.64.251  # password: raspberry
```

### Debug GUI (on local machine)
```bash
python3 tools/debug_gui.py --host 10.156.64.251 --port 8000
```

## Hardware Setup
- **RPi Address**: `10.156.64.251`
- **Camera**: USB camera at `/dev/video0`, 640x480 @ 15 FPS
- **ArUco**: DICT_6X6_250 dictionary, 20cm markers
- **Serial**: `/dev/serial0` at 921600 baud

## FC Configuration (ArduCopter)
```
VISO_TYPE = 1          # MAVLink vision
EK3_SRC1_POSXY = 6     # ExternalNav
EK3_SRC1_POSZ = 6      # ExternalNav
EK3_SRC1_YAW = 6       # ExternalNav
GPS_TYPE = 0            # Disable GPS (indoor)
```

## Session History

### Session 1 (2025-01-22)
- Initial RPi deployment, camera and detection testing

### Session 2 (2026-01-23)
- Debug GUI, camera server, remote calibration, camera calibrated

### Session 3 (2026-02-02)
- Architecture change: RPi is now GPS emulator, FC handles navigation
- Removed PID, mission executor, dead reckoning, failsafes from RPi code
- Added VISION_POSITION_ESTIMATE to MAVLink interface
- Simplified main.py to detect → estimate → send loop
- Old modules archived in src/deprecated/
- Created CURRENT_PLAN.md as living roadmap
