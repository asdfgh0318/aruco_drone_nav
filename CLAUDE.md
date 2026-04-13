# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## IMPORTANT RULES

### Documentation with Every Commit
**ALWAYS update documentation when making code changes.** Never commit code without updating:
- `README.md` - If user-facing features change
- `CLAUDE.md` - If architecture or implementation details change
- `docs/site.html` - The single comprehensive docs site (if relevant)

This is mandatory. Code changes without doc updates are incomplete.

## Project Overview

**ArUco Vision GPS** — Vision-based GPS emulator for indoor drones. RPi Zero 2W detects ceiling-mounted ArUco markers, calculates world position, and sends `GPS_INPUT` to an ArduCopter FC via MAVLink. Barometer handles altitude; vision handles XY + yaw.

**Status**: GPS emulation with IMU-corrected position estimation, ~20 FPS, 47 markers deployed. NixOS RPi.

## Commands

```bash
# Run on RPi (sudo required for UART access)
sudo python3 -m src --mode run       # Flight: vision → GPS_INPUT → FC
sudo python3 -m src --mode test      # Console: prints position, no FC
sudo python3 -m src --mode stream    # HTTP server on port 8001

# Deploy to RPi
./sync_to_rpi.sh                     # rsync to mtj@pi.local (fallback IP in script)

# Debug tools (from dev laptop)
python3 tools/debug_viewer.py        # Remote frame capture (SPACE to grab)
python3 tools/terminal_map.py        # Curses live position map
curl http://<rpi-ip>:8001/position   # JSON position data
```

No build step, no tests, no linter. The system is ~920 lines across 2 source files.

## Architecture

```
Frame (1280x720 MJPG)
  → Gray → CLAHE → ArUco detectMarkers (grayscale, ~50ms)
  → solvePnPGeneric per marker (IPPE disambiguation)
  → Angle-based position: tvec angles + IMU pitch/roll → body offset → rotate by yaw → NED
  → GPS_INPUT via MAVLink2 UART → FC
```

### Core Files

| File | Purpose |
|------|---------|
| `src/vision_gps.py` (~750 lines) | Camera capture thread, CLAHE, ArUco detection, solvePnP, angle-based position estimation with IMU correction, multi-marker fusion, HTTP server, main loop, CSV logging |
| `src/mavlink_bridge.py` (~165 lines) | Serial connection with PL011 bug workaround, heartbeat handshake, `send_gps_input(north, east, down, yaw_deg, ...)` with dynamic `horiz_accuracy` |
| `src/__main__.py` | Entry point: `from .vision_gps import main; main()` |

### Config Files

| File | Purpose |
|------|---------|
| `config/system_config.yaml` | Serial port/baud, camera resolution, ArUco params, EKF origin lat/lon/alt |
| `config/marker_map.yaml` | Marker ID → world position (NED) + orientation + size. 47 markers deployed. |
| `config/camera_params.yaml` | Intrinsics calibrated natively at 1280×720, level_tvec for tilt correction |

## Critical Architecture Details

### MAVLink GPS Emulation

- Converts NED → lat/lon relative to EKF origin, sends `GPS_INPUT` message
- FC params: `GPS_TYPE=14`, `EK3_SRC1_POSXY=3` (GPS), `EK3_SRC1_POSZ=1` (baro)
- `horiz_accuracy` scales dynamically with detection confidence (0.05m at conf=1.0, 0.50m at conf=0.1)
- Works on any ArduCopter firmware with `AP_GPS_MAV_ENABLED`

### Position Estimation (Angle-Based with IMU Correction)

Position uses tvec angles + IMU pitch/roll (NOT R_cm — too noisy at perpendicular viewing):

```
cam_ax = atan2(tvec[0], tvec[2])              # Camera-frame angle along X
cam_ay = atan2(tvec[1], tvec[2])              # Camera-frame angle along Y
world_ax = imu_pitch + (cam_ax - level_ax)    # IMU-corrected, static tilt removed
world_ay = imu_roll + (cam_ay - level_ay)     # Same for roll axis
body_fwd = -height * tan(world_ax)            # Body offset
body_right = height * tan(world_ay)           # Body offset
body_down = -height                           # Marker above = negative down
pos = marker_pos - rot(yaw) @ [body_fwd, body_right, body_down]
```

Yaw from vision: `R_bw = R_mw @ R_cm^T @ R_CB_NOMINAL^T`, `yaw = atan2(R_bw[1,0], R_bw[0,0])`

### Multi-Marker Fusion

When multiple markers visible, positions are weighted by inverse horizontal distance:
`w = 1.0 / (horiz_dist + 0.1)`. Markers directly overhead get strongest weight (best solvePnP accuracy, least tilt-correction error). Yaw uses circular mean to avoid wraparound.

### R_CB_NOMINAL

```python
R_CB_NOMINAL = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]  # det=+1, proper rotation (180° around Y)
```
Maps camera frame to body frame for upward-facing camera. det=+1 (proper rotation, not reflection).

### solvePnP Disambiguation

IPPE_SQUARE returns two solutions for flat markers. At near-perpendicular viewing (our case), both have similar reprojection error. Fix: `solvePnPGeneric()` returns both, pick the one with most vertical marker Z in camera frame (`min(mZ_x² + mZ_y²)`).

### Per-Marker Size Support

Each marker in `marker_map.yaml` can have an optional `size` field (meters). The detect() function creates per-marker object points. Default size from `system_config.yaml` (0.20m).

### OpenCV API Compatibility

NixOS ships OpenCV 4.9.0. The code handles both old and new ArUco APIs:
- Old: `DetectorParameters_create()` + `cv2.aruco.detectMarkers(frame, dict, parameters=params)`
- New: `DetectorParameters()` + `ArucoDetector(dict, params).detectMarkers(frame)`

### MAVLink2 Requirement

Connection uses `mavlink20=True` — required because `GPS_INPUT.yaw` is a MAVLink2 extension field. Without it, yaw is silently truncated. The receive loop also captures `GLOBAL_POSITION_INT` for FC position feedback and `ATTITUDE` for IMU roll/pitch/yaw + rates.

### ArduPilot Heartbeat Requirement

ArduPilot does NOT stream telemetry on non-primary UARTs until it receives heartbeats. `MAVLinkBridge.connect()` sends heartbeats BEFORE `wait_heartbeat()` and the receive loop continues at 1Hz.

### RPi PL011 UART Bug

`/dev/ttyAMA0` on RPi throws `SerialException: device reports readiness to read but returned no data`. Workaround: monkey-patch `serial.Serial.read` to catch and return `b''`. Applied in `mavlink_bridge.py connect()`.

### Detection Optimizations

- Grayscale-only detection (no Gray→BGR reconversion)
- CLAHE always on (critical for dark environments)
- Adaptive threshold: windows [3, 17, 31, 45] (step=14, max=53) for wide lighting range
- `minOtsuStdDev=3.0` for very dark scenes
- V4L2 explicit backend + buffer drain (grab before read)
- V4L2 buffer size = 1
- Frame age check (skip frames older than 500ms)

## Hardware

- **RPi**: `pi.local`, user `mtj`, NixOS arm64 (IP changes per network — check `sync_to_rpi.sh`)
- **Camera**: USB, MJPG 1280×720, focus locked at infinity on startup
- **FC**: SpeedyBee F405 V3, custom firmware (`AP_GPS_MAV_ENABLED=1`), UART at 115200 baud
- **Markers**: 20cm ArUco (DICT_4X4_50) on A4 paper, ceiling-mounted, 47 deployed

## Key Tools

| Tool | Purpose |
|------|---------|
| `tools/live_map.html` | Browser live 2D map with marker weights, FC arrow, YAML import, camera config, debug frame |
| `tools/glb_viewer.html` | Browser 3D viewer for GLB models + flight paths + marker placement (M key) |
| `tools/debug_viewer.py` | Remote frame capture with timing overlay |
| `tools/terminal_map.py` | Curses live position map (works over SSH) |
| `tools/vr_to_waypoints.py` | VR planner JSON → ArduPilot `.waypoints` (supports `--frame global` for MSL altitudes) |
| `tools/tlog_to_vr_json.py` | Mission Planner `.tlog` → VR JSON |
| `tools/generate_markers.py` | Generate printable ArUco marker PDFs |
| `tools/calibrate_camera.py` | Interactive chessboard camera calibration |
| `tools/calibrate_level.py` | Camera mounting tilt calibration (records tvec under marker) |
| `tools/orientation_check.py` | Interactive NED axis / marker orientation verification |

## HTTP API (port 8001)

| Endpoint | Method | Returns |
|----------|--------|---------|
| `/position` | GET | JSON: `{n, e, d, yaw, marker_ids, confidence, marker_weights, timing, imu_roll, imu_pitch, ...}` |
| `/markers` | GET | JSON: marker map from YAML |
| `/debug-frame` | GET | JPEG: latest camera frame |
| `/camera-config` | GET/POST | JSON: camera controls (brightness, contrast, exposure, etc.) |

## Known Issues

- `CORNER_REFINE_CONTOUR` assertion crash on some frames (caught by try-except)
- Camera detector must be created BEFORE `camera.start()` (OpenCV thread safety segfault)
- solvePnP R_cm is unreliable at near-perpendicular viewing (IPPE ambiguity) — position uses tvec+IMU instead
- `YAW_DEBUG` log line still in `estimate_single()` — per-frame debug matrix work, remove when yaw is verified
- Optical flow interpolation implemented but disabled (preserved in git `827ac84`) — future: Option A (OPTICAL_FLOW_RAD to EKF) is the proper approach

## Documentation

- **`docs/site.html`** — Single comprehensive HTML documentation site
- **`docs/pipeline_debug.html`** — Position pipeline debug reference
- **`docs/position_pipeline.html`** — Pipeline visualization
- **`docs/GLB_VIEWER.md`** — GLB viewer usage guide
- **`docs/archive/`** — Historical docs (VISO era, old plans, session notes)
- **`docs/images/`** — Screenshots for documentation

## FC Parameters (GPS_INPUT Mode)

```
GPS_TYPE = 14           # MAVLink GPS
EK3_SRC1_POSXY = 3      # GPS
EK3_SRC1_POSZ = 1       # Baro
EK3_SRC1_YAW = 2        # GPS yaw
SERIAL4_PROTOCOL = 2    # MAVLink2
SERIAL4_BAUD = 115       # 115200
```

Custom firmware required: stock SpeedyBee F405 V3 has `AP_GPS_MAV` compiled out. Built via custom.ardupilot.org with `AP_GPS_MAV_ENABLED=1`.
