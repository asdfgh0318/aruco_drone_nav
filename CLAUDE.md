# Claude Code Resume Instructions

This document provides context for AI assistants continuing work on this project.

## Project Overview

**ArUco Drone Navigation System** - Vision-based indoor drone navigation using ceiling-mounted ArUco markers on Raspberry Pi Zero 2W.

**Project Location**: `/home/adam-koszalka/ŻYCIE/PRACA/aruco_drone_nav`

## Current Status (2026-01-23)

### Completed
- Full navigation system implemented (detection, position estimation, MAVLink, missions)
- RPi Zero 2W deployment working at `10.156.64.251`
- Camera calibrated (fx=417, fy=417, cx=318, cy=275)
- Debug GUI with video streaming, telemetry, marker map
- Network-based calibration tool

### Hardware Setup
- **RPi Address**: `10.156.64.251`
- **Credentials**: user=`pi`, password=`raspberry`
- **Camera**: USB camera at `/dev/video0`, 640x480 @ 15 FPS
- **ArUco**: DICT_6X6_250 dictionary, 20cm markers

## Key Files

| File | Purpose |
|------|---------|
| `src/aruco_detector.py` | Marker detection with OpenCV |
| `src/position_estimator.py` | Position calculation from markers |
| `src/mavlink_interface.py` | MAVLink communication |
| `tools/debug_gui.py` | Debug GUI (runs on local machine) |
| `tools/camera_server.py` | MJPEG streaming (runs on RPi) |
| `tools/calibrate_remote.py` | Network calibration tool |
| `config/camera_params.yaml` | Camera calibration data |
| `config/marker_map.yaml` | Marker world positions |
| `config/system_config.yaml` | System configuration |

## Quick Commands

### Connect to RPi
```bash
ssh pi@10.156.64.251  # password: raspberry
```

### Start Camera Server (on RPi)
```bash
cd /home/pi/aruco_drone_nav
python3 tools/camera_server.py --port 8000
```

### Launch Debug GUI (on local machine)
```bash
python3 tools/debug_gui.py --host 10.156.64.251 --port 8000
```

### Calibrate Camera
```bash
python3 tools/calibrate_remote.py --host 10.156.64.251 --port 8000
```

### Deploy to RPi
```bash
# Create tarball and SCP to RPi
tar -czvf /tmp/deploy.tar.gz --exclude='buildroot' --exclude='venv' --exclude='.git' .
scp /tmp/deploy.tar.gz pi@10.156.64.251:/home/pi/
ssh pi@10.156.64.251 "cd /home/pi && tar -xzvf deploy.tar.gz -C aruco_drone_nav"
```

## Architecture

```
RPi (10.156.64.251)                    Local Machine
+------------------+                   +------------------+
| camera_server.py | ---- MJPEG ---->  | debug_gui.py     |
| - Capture frames |      Port 8000    | - Display video  |
| - JPEG compress  |                   | - Run detection  |
+------------------+                   | - Show telemetry |
                                       +------------------+
```

## Development Notes

### Detection Rate
- Low detection rate (~3%) in dim lighting
- Flat marker surface improves detection significantly
- Good lighting essential for reliable operation

### Calibration
- Reprojection error: 3.41 pixels (acceptable, could be improved)
- Calibration uses 9x6 chessboard pattern
- Print `markers/chessboard_9x6.pdf` at 100% scale

### Known Issues
- Camera calibration could be improved (reprojection error > 1.0)
- Detection rate drops in poor lighting conditions

## Future Work
See `TODO.md` for full list. Priority items:
- LiDAR integration for altitude sensing
- SITL testing before real flight
- Marker spacing calculator tool

## Session History

### Session 1 (2025-01-22)
- Initial RPi deployment
- Camera and detection testing
- Position estimation verification

### Session 2 (2026-01-23)
- Debug GUI implementation
- Camera server for network streaming
- Remote calibration tool
- Camera calibration completed
- Documentation updates
