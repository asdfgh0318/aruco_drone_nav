# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## IMPORTANT RULES

### Documentation with Every Commit
**ALWAYS update documentation when making code changes.** Never commit code without updating:
- `README.md` - If user-facing features change
- `CURRENT_PLAN.md` - Status, metrics, what's done/pending
- `TODO.md` - Completed items, new tasks
- `docs/TECHNICAL.md` - If implementation details change

This is mandatory. Code changes without doc updates are incomplete.

## Project Overview

**ArUco Vision GPS** — Vision-based GPS emulator for indoor drones. RPi Zero 2W detects ceiling-mounted ArUco markers, calculates world position, and sends `GPS_INPUT` to an ArduCopter FC via MAVLink. Barometer handles altitude; vision handles XY + yaw.

**Status**: GPS emulation with IMU-corrected position estimation. Althold tested. NixOS RPi. VISO approach preserved as git tag `viso-experimental`.

## Commands

```bash
# Run on RPi (sudo required for UART access)
sudo python3 -m src --mode run       # Flight: vision → GPS_INPUT → FC
sudo python3 -m src --mode test      # Console: prints position, no FC
sudo python3 -m src --mode stream    # HTTP server on port 8001

# Deploy to RPi
./sync_to_rpi.sh                     # rsync to aruconav@aruconav.local

# Debug tools (from dev laptop)
python3 tools/debug_viewer.py        # Remote frame capture (SPACE to grab)
python3 tools/terminal_map.py        # Curses live position map
curl http://aruconav.local:8001/position   # JSON position data
```

No build step, no tests, no linter. The system is ~700 lines across 2 source files.

## Architecture

```
Frame (1280x720 MJPG)
  → Gray → CLAHE → BGR → ArUco detectMarkers (~110ms)
  → solvePnP per marker (with temporal consistency)
  → Camera→Body→World transform → ENU position + yaw
  → GPS_INPUT via MAVLink UART → FC
```

### Core Files

| File | Purpose |
|------|---------|
| `src/vision_gps.py` (~500 lines) | Camera capture thread, CLAHE, ArUco detection, solvePnP, position estimation, coordinate transforms, HTTP server, main loop, CSV logging |
| `src/mavlink_bridge.py` (~200 lines) | Serial connection with PL011 bug workaround, heartbeat handshake, `send_gps_input()` (active), `send_vision_position()` (VISO fallback), EKF origin |
| `src/__main__.py` | Entry point: `from .vision_gps import main; main()` |

### Config Files

| File | Purpose |
|------|---------|
| `config/system_config.yaml` | Serial port/baud, camera resolution, ArUco params, `gps_emulation: true/false`, EKF origin lat/lon |
| `config/marker_map.yaml` | Marker ID → world position (ENU) + orientation |
| `config/camera_params.yaml` | Intrinsics calibrated at 640×480, auto-scaled to runtime resolution |

## Critical Architecture Details

### Two MAVLink Modes (controlled by `gps_emulation` in config)

**GPS_INPUT (active, `gps_emulation: true`):**
- Converts ENU → lat/lon relative to origin, sends `GPS_INPUT` message
- FC params: `GPS_TYPE=14`, `EK3_SRC1_POSXY=3`, `EK3_SRC1_POSZ=1` (baro)
- Works on any ArduCopter firmware with `AP_GPS_MAV_ENABLED`

**VISION_POSITION_ESTIMATE (`gps_emulation: false`):**
- Sends NED pose directly. Requires custom firmware + VISO FC params
- Tagged as `viso-experimental` — ArduPilot doesn't support Z axis properly via VISO

### Position Estimation (Angle-Based with IMU Correction)

Position uses tvec angles + IMU pitch/roll (NOT R_cm — too noisy at perpendicular viewing):

```
cam_ax = atan2(tvec[0], tvec[2])              # Camera-frame angle along X
cam_ay = atan2(tvec[1], tvec[2])              # Camera-frame angle along Y
world_ax = imu_pitch + (cam_ax - level_ax)    # IMU-corrected, static tilt removed
world_ay = imu_roll + (cam_ay - level_ay)     # Same for roll axis
body_fwd = -height * tan(world_ax)            # Body offset (cam_X = -body_fwd)
body_right = height * tan(world_ay)           # Body offset (cam_Y = body_right)
→ rotate by yaw → ENU world position
```

Yaw from vision: `R_bw = R_mw @ R_cm^T @ R_CB_NOMINAL^T`, `yaw = atan2(R_bw[1,0], R_bw[0,0])`

### solvePnP Disambiguation

IPPE_SQUARE returns two solutions for flat markers. At near-perpendicular viewing (our case), both have similar reprojection error. Fix: `solvePnPGeneric()` returns both, pick the one with most vertical marker Z in camera frame (`min(mZ_x² + mZ_y²)`).

### OpenCV API Compatibility

Bookworm ships OpenCV 4.6.0 with the **old** ArUco API. The code handles both:
- Old: `DetectorParameters_create()` + `cv2.aruco.detectMarkers(frame, dict, parameters=params)`
- New: `DetectorParameters()` + `ArucoDetector(dict, params).detectMarkers(frame)`

`create_detector()` returns either an `ArucoDetector` object (new) or a `(dict, params)` tuple (old). `detect()` checks `isinstance(detector, tuple)` to dispatch.

### solvePnP Temporal Consistency

- First detection: `SOLVEPNP_IPPE_SQUARE` (no initial guess)
- Subsequent frames: `SOLVEPNP_ITERATIVE` with previous frame's rvec/tvec
- Sanity check: discard if distance > 10m or rvec > 20π, reset temporal state for that marker

### MAVLink2 Requirement

Connection uses `mavlink20=True` — required because `GPS_INPUT.yaw` is a MAVLink2 extension field. Without it, yaw is silently truncated. The receive loop also captures `GLOBAL_POSITION_INT` for FC position feedback.

### ArduPilot Heartbeat Requirement

ArduPilot does NOT stream telemetry on non-primary UARTs until it receives heartbeats. `MAVLinkBridge.connect()` sends heartbeats BEFORE `wait_heartbeat()` and the receive loop continues at 1Hz.

### RPi PL011 UART Bug

`/dev/ttyAMA0` on RPi throws `SerialException: device reports readiness to read but returned no data`. Workaround: monkey-patch `serial.Serial.read` to catch and return `b''`. Applied in `mavlink_bridge.py connect()`.

## Hardware

- **RPi**: `pi.local` / `10.40.41.251`, user `mtj`, NixOS arm64
- **Camera**: USB, MJPG 1280×720, must lock autofocus: `v4l2-ctl --set-ctrl focus_automatic_continuous=0,focus_absolute=0`
- **FC**: SpeedyBee F405 V3, custom firmware, UART at 115200 baud
- **Markers**: 18cm ArUco (DICT_4X4_50) on A4 paper, ceiling-mounted

## Key Tools

| Tool | Purpose |
|------|---------|
| `tools/debug_viewer.py` | Remote frame capture with timing overlay |
| `tools/terminal_map.py` | Curses live position map (works over SSH) |
| `tools/live_map.html` | Browser live map with marker weights, FC arrow, YAML import |
| `tools/calibrate_level.py` | Camera mounting tilt calibration (records tvec under marker) |
| `tools/glb_viewer.html` | Browser 3D viewer for GLB models + flight paths + marker placement |
| `tools/vr_to_waypoints.py` | VR planner JSON → ArduPilot `.waypoints` |
| `tools/tlog_to_vr_json.py` | Mission Planner `.tlog` → VR JSON |
| `tools/generate_markers.py` | Generate printable ArUco marker PDFs |
| `tools/calibrate_camera.py` | Interactive chessboard camera calibration |

## Known Issues

- `CORNER_REFINE_CONTOUR` assertion crash on some frames (caught by try-except)
- Detection bottleneck: ArUco detect takes ~110ms (79% of frame time)
- Camera detector must be created BEFORE camera.start() (OpenCV thread safety segfault on Bookworm)
- solvePnP R_cm is unreliable at near-perpendicular viewing (IPPE ambiguity) — position uses tvec+IMU instead
- R_CB_NOMINAL has det=-1 (reflection from upward camera) — cannot be used with rotation matrices for IMU correction
