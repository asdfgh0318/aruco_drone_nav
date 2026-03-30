# ArUco Drone Navigation - Current Plan

## Architecture
RPi + Camera = Visual GPS emulator. Detects markers, calculates position, sends GPS_INPUT to FC. FC handles everything else — barometer for altitude, vision for XY and yaw.

## Current Status (2026-03-30)

### System: Multi-Marker GPS Emulation with MAVLink2
GPS_INPUT mode with angle-based position estimation. IMU pitch/roll corrects for drone tilt,
vision provides yaw and tvec. Camera level calibration removes static mounting tilt.
solvePnP IPPE ambiguity resolved with marker Z verticality check.

**18 markers deployed** (IDs 0-17) across faculty building corridor (L-shaped layout).
MAVLink2 enabled for GPS yaw support. FC position feedback via GLOBAL_POSITION_INT.
Per-marker distance-weighted position fusion with confidence scoring.

**Position tested stable** when tilting drone ±15°. ArduPilot map matches physical movement.

### Performance on RPi Zero 2W (NixOS)
| Metric | Value |
|--------|-------|
| Resolution | 1280x720 (native calibration) |
| FPS | ~7 |
| Detection Rate | 80-100% |
| XY Precision | ~10mm (stationary), stable under tilt |
| GPS_INPUT | hdop=0.3, horiz_acc=0.1m, vert_acc=0.1m |

### Custom Firmware
SpeedyBee F405 V3 stock firmware has AP_GPS_MAV compiled out.
Custom build via custom.ardupilot.org with:
- AP_GPS_MAV_ENABLED=1
- HAL_VISUALODOM_ENABLED=1
- EK3_FEATURE_EXTERNAL_NAV=1
- AP_MOTORS_FRAME_QUAD_ENABLED=1

### FC Parameters (GPS Emulation Mode)
```
GPS_TYPE = 14           # MAVLink GPS
EK3_SRC1_POSXY = 3     # GPS for XY
EK3_SRC1_POSZ = 1      # Baro for altitude
EK3_SRC1_YAW = 2       # GPS yaw (or 0)
VISO_TYPE = 0           # Disabled
```

### RPi Setup (NixOS)
- NixOS arm64, IP: 192.168.213.251, user: mtj
- OpenCV 4.9.0, Python 3.12.8
- UART: /dev/ttyS0, baud 115200
- Deps via Nix: python3-opencv, numpy, yaml, serial, pymavlink

## Completed

### Vision System
- [x] ArUco marker detection (DICT_4X4_50)
- [x] CLAHE preprocessing for robust detection
- [x] 6-DOF pose estimation (solvePnP with temporal consistency)
- [x] Camera calibration (640x480, auto-scaled to 1280x720)
- [x] Corner refinement (CORNER_REFINE_CONTOUR)
- [x] Timing instrumentation
- [x] Codebase minimization (3,329 -> ~500 lines)
- [x] Bookworm compatibility (old + new ArUco API)
- [x] solvePnP sanity check (discard >10m / diverging rvec)

### Position Estimation
- [x] Correct solvePnP inverse: `cam_in_marker = -R^T @ tvec`, world pos from marker pose
- [x] Vision yaw from full rotation chain: `R_bw = R_mw @ R_cm^T @ R_CB^T`
- [x] Position is now IMU-independent (only solvePnP + known marker pose)
- [x] Multi-marker weighted fusion
- [x] Low-pass position filtering (alpha=0.3)
- [x] Debug CSV logging (always on)

### GPS Emulation (MAVLink)
- [x] GPS_INPUT to FC (GPS emulation mode) — **active**
- [x] VISION_POSITION_ESTIMATE to FC (preserved as `viso-experimental` tag)
- [x] Custom firmware with AP_GPS_MAV enabled
- [x] Heartbeat handshake (send before waiting)
- [x] Pyserial PL011 workaround
- [x] Althold flight test — successful hover

### Infrastructure
- [x] RPi Zero 2W deployment (Bookworm)
- [x] HTTP position server (JSON + debug JPEG)
- [x] RPi wired to FC via UART (115200 baud)

### Mission & Telemetry Tools
- [x] `tools/vr_to_waypoints.py` — VR planner JSON / missions JSON → ArduPilot `.waypoints`
- [x] `tools/tlog_to_vr_json.py` — Mission Planner `.tlog` → VR planner JSON for path comparison

### 3D Mission Viewer & Marker Placement
- [x] `tools/glb_viewer.html` — Browser-based GLB model + flight path + waypoints viewer
- [x] Marker placement mode (M key) — click ceiling to place markers, export to marker_map.yaml
- [x] YAML import/export with per-marker height from click point
- [x] Interactive tutorial wizard (8 steps, auto-shows on first visit, `?` to replay)

## Next: Reliability + Extended Flight Testing

### Immediate
1. [ ] **Investigate althold timeout** — althold worked then stopped after ~7 min (vibrations? heat? detection loss?)
2. [ ] **Fix camera mounting** — camera needs proper fixed position relative to drone frame
3. [ ] **Extended flight tests** — multiple flights to characterize reliability

### Performance
1. [ ] **Recalibrate camera at 1280x720** — fixes yaw-position coupling from non-uniform scaling
2. [ ] **FPS optimization** — conditional CLAHE, ROI detection
3. [ ] **Camera focus lock** — add to startup code

### Coverage
1. [x] **Multi-marker deployment** — 18 markers (IDs 0-17) deployed across faculty corridor
2. [ ] **Loiter mode testing** — after althold is reliable
3. [ ] **Marker orientation verification** — confirm 180° orientation matches physical mounting

## File Structure
```
src/vision_gps.py           (~500 lines) - Camera, detection, position, HTTP server, main loop
src/mavlink_bridge.py       (~200 lines) - MAVLink connection + GPS emulation + vision position
src/__main__.py             (3 lines)    - Entry point
tools/vr_to_waypoints.py   (~230 lines) - VR planner JSON → ArduPilot .waypoints
tools/tlog_to_vr_json.py   (~260 lines) - Mission Planner .tlog → VR planner JSON
tools/glb_viewer.html      (~1350 lines)- 3D viewer with marker placement + tutorial wizard
tools/live_map.html        (~430 lines) - 2D live map with marker weights + FC position
viewer/samples/                          - Sample mission files for testing
```

---

*Last updated: 2026-03-30*
