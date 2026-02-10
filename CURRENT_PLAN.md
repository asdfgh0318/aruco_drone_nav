# ArUco Drone Navigation - Current Plan

## Architecture
RPi + Camera = Visual GPS emulator. Detects markers, calculates position, sends to FC. FC handles everything else.

## Current Status (2026-02-10)

### System: End-to-End Working
GPS emulation confirmed working with custom firmware. IMU fusion integrated. Ready for flight testing.

### Performance on RPi Zero 2W
| Metric | Value |
|--------|-------|
| Resolution | 1280x720 (MJPG) |
| Detection Rate | 95% |
| Processing Time | ~133ms/frame |
| FPS | ~7.5 |
| Stationary Precision | 3-9mm XY, 1-3mm Z |
| Marker Type | Single ArUco (DICT_4X4_50) |
| Marker Size | 18cm (A4 printable) |

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
- [x] Camera -> Body -> World coordinate transform (upward-facing camera)
- [x] IMU fusion: ZYX Euler body-to-world rotation from ATTITUDE messages
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

## Next: Flight Testing

### Pre-Flight Checklist
1. [x] Wire RPi to FC
2. [x] Mount camera facing up on drone frame
3. [x] Flash custom firmware
4. [x] Set FC parameters (GPS_TYPE=14)
5. [x] Mount marker(s) on ceiling above test area
6. [ ] Measure marker position(s) and update marker_map.yaml

### Test Sequence
1. [x] **Bench test** - RPi + FC on bench, GPS_INPUT confirmed working
2. [ ] **EKF convergence** - Verify FC EKF accepts GPS data
3. [ ] **Ground test** - Drone on ground under marker, verify stable position
4. [ ] **Tethered hover** - Safety tether, Loiter mode, verify position hold
5. [ ] **Free hover** - Remove tether, hover under single marker

## Future Improvements
1. IMU update rate (currently ~4Hz, should match camera ~7.5Hz)
2. FPS optimization (target 10+ FPS)
3. Camera focus lock in startup code
4. Recalibrate camera at 1280x720
5. Multi-marker deployment for larger area

## File Structure
```
src/vision_gps.py      (~500 lines) - Camera, detection, position, HTTP server, main loop
src/mavlink_bridge.py  (~183 lines) - MAVLink connection + GPS emulation + vision position
src/__main__.py        (3 lines)    - Entry point
```

---

*Last updated: 2026-02-10*
