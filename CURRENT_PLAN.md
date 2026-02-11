# ArUco Drone Navigation - Current Plan

## Architecture
RPi + Camera = Visual GPS emulator. Detects markers, calculates position, sends to FC. FC handles everything else.

## Current Status (2026-02-11)

### System: Visual Odometry Migration Done (Code Side)
Position formula fixed (correct solvePnP inverse). Visual odometry mode implemented (VISION_POSITION_ESTIMATE with auto EKF origin). FC parameters not yet set. First althold flight test crashed — needs investigation.

### Performance on RPi Zero 2W
| Metric | 640x480 | 1280x720 (scaled) |
|--------|---------|-------------------|
| FPS | ~20 | ~3.7 |
| Detection Rate | 95.6% | 95% |
| XY Precision | ~22mm | ~5mm |
| Z Precision | ~10mm | ~1-3mm |
| Yaw-Pos Coupling | Present* | Present* |

*Yaw-position coupling due to camera calibration at 640x480 being scaled non-uniformly to 1280x720. Need to recalibrate at native 1280x720.

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

## Next: Fix Calibration + Flight Testing

### Immediate
1. [ ] **Recalibrate camera at 1280x720** — fixes yaw-position coupling
2. [ ] **Set FC params for visual odometry** — VISO_TYPE=1, EK3_SRC1_POSXY/Z/YAW=6, COMPASS_USE=0, GPS_TYPE=0
3. [ ] **Investigate althold crash** — first flight test (2026-02-11) ended in crash

### Test Sequence
1. [x] **Bench test** - RPi + FC on bench, GPS_INPUT confirmed working
2. [ ] **EKF convergence** - Verify FC EKF accepts vision data
3. [ ] **Ground test** - Drone on ground under marker, verify stable position
4. [ ] **Tethered hover** - Safety tether, Loiter mode, verify position hold
5. [ ] **Free hover** - Remove tether, hover under single marker

## Future Improvements
1. FPS optimization at 1280x720 (conditional CLAHE, ROI detection)
2. Camera focus lock in startup code
3. Multi-marker deployment for larger area

## File Structure
```
src/vision_gps.py      (~500 lines) - Camera, detection, position, HTTP server, main loop
src/mavlink_bridge.py  (~183 lines) - MAVLink connection + GPS emulation + vision position
src/__main__.py        (3 lines)    - Entry point
```

---

*Last updated: 2026-02-11*
