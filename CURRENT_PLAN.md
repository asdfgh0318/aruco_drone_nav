# ArUco Drone Navigation - Current Plan

## Architecture
RPi + Camera = Visual GPS emulator. Detects markers, calculates position, sends to FC. FC handles everything else.

## Current Status (2026-02-19)

### System: VISO End-to-End Working
Position visible in Mission Planner via VISION_POSITION_ESTIMATE. FC params set. Fresh RPi Trixie image. First althold flight test crashed (2026-02-11) — need to fix camera mount and reduce throttle before retrying.

### Performance on RPi Zero 2W
| Metric | Value |
|--------|-------|
| Resolution | 1280x720 (scaled from 640x480 calibration) |
| FPS | ~3.7 |
| Detection Rate | 95-100% |
| XY Precision | ~5mm (stationary) |
| Z Precision | ~1-3mm |
| Yaw-Pos Coupling | Present (non-uniform scaling) |

Note: Yaw-position coupling exists due to camera calibration at 640x480 being scaled non-uniformly to 1280x720 (sx=2.0, sy=1.5). Recalibrating at 1280x720 would fix this.

### Custom Firmware
SpeedyBee F405 V3 stock firmware has AP_GPS_MAV and HAL_VISUALODOM compiled out.
Custom build via custom.ardupilot.org with:
- AP_GPS_MAV_ENABLED=1
- HAL_VISUALODOM_ENABLED=1
- EK3_FEATURE_EXTERNAL_NAV=1
- AP_MOTORS_FRAME_QUAD_ENABLED=1

GPS_INPUT confirmed: fix=3, sats=12, correct lat/lon.

## Completed

### Vision System
- [x] ArUco marker detection (DICT_4X4_50)
- [x] CLAHE preprocessing for robust detection
- [x] 6-DOF pose estimation (solvePnP with temporal consistency)
- [x] Camera calibration (640x480, auto-scaled to 1280x720)
- [x] Corner refinement (CORNER_REFINE_CONTOUR)
- [x] Timing instrumentation
- [x] Codebase minimization (3,329 -> ~500 lines)

### Position Estimation
- [x] Correct solvePnP inverse: `cam_in_marker = -R^T @ tvec`, world pos from marker pose
- [x] Vision yaw from full rotation chain: `R_bw = R_mw @ R_cm^T @ R_CB^T`
- [x] Position is now IMU-independent (only solvePnP + known marker pose)
- [x] Multi-marker weighted fusion
- [x] Low-pass position filtering (alpha=0.3)
- [x] Debug CSV logging (always on)

### GPS Emulation (MAVLink)
- [x] GPS_INPUT to FC (GPS emulation mode)
- [x] VISION_POSITION_ESTIMATE to FC (vision mode)
- [x] Custom firmware with AP_GPS_MAV + VISUALODOM enabled
- [x] Heartbeat handshake (send before waiting)
- [x] Pyserial PL011 workaround

### Infrastructure
- [x] RPi Zero 2W deployment
- [x] HTTP position server (JSON + debug JPEG)
- [x] RPi wired to FC via UART (115200 baud)

### Mission & Telemetry Tools
- [x] `tools/vr_to_waypoints.py` — VR planner JSON / missions JSON → ArduPilot `.waypoints`
- [x] `tools/tlog_to_vr_json.py` — Mission Planner `.tlog` → VR planner JSON for path comparison

### 3D Mission Viewer
- [x] `tools/glb_viewer.html` — Browser-based GLB model + flight path + waypoints viewer
- [x] Interactive tutorial wizard (7 steps, auto-shows on first visit, `?` to replay)
- [x] Sample mission files in `viewer/samples/` for testing

## Next: Hardware Fixes + Flight Testing

### Immediate (before next flight)
1. [ ] **Fix camera mounting** — camera needs proper fixed position relative to drone frame
2. [ ] **Reduce throttle PWM** — drone is overpowered, adjust MOT_THST_HOVER or throttle range
3. [ ] **Retry althold** — after fixing camera mount and throttle

### Test Sequence
1. [x] **Bench test** — GPS_INPUT confirmed working (fix=3, sats=12)
2. [x] **EKF convergence** — VISO position displayed in Mission Planner
3. [ ] **Ground test** — Drone on ground under marker, verify stable position
4. [ ] **Tethered hover** — Safety tether, althold mode
5. [ ] **Free hover** — Remove tether, hover under single marker

## Future Improvements
1. Recalibrate camera at 1280x720 (fixes yaw-position coupling)
2. FPS optimization (conditional CLAHE, ROI detection)
3. Camera focus lock in startup code
4. Multi-marker deployment for larger area

## File Structure
```
src/vision_gps.py           (~500 lines) - Camera, detection, position, HTTP server, main loop
src/mavlink_bridge.py       (~183 lines) - MAVLink connection + GPS emulation + vision position
src/__main__.py             (3 lines)    - Entry point
tools/vr_to_waypoints.py   (~230 lines) - VR planner JSON → ArduPilot .waypoints
tools/tlog_to_vr_json.py   (~260 lines) - Mission Planner .tlog → VR planner JSON
tools/glb_viewer.html      (~1040 lines)- 3D viewer with tutorial wizard
viewer/samples/                          - Sample mission files for testing
```

---

*Last updated: 2026-02-23*
