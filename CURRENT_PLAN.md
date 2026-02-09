# ArUco Drone Navigation - Current Plan

## Architecture
RPi + Camera = Visual GPS emulator. Detects markers, calculates position, sends to FC. FC handles everything else.

## Current Status (2026-02-09)

### Codebase: Minimal & Working
Rewrote entire src/ from 3,329 lines to ~490 lines. Tested on RPi Zero 2W with live marker detection.

### Performance on RPi Zero 2W
| Metric | Value |
|--------|-------|
| Resolution | 1280x720 (MJPG) |
| Detection Rate | 99-100% |
| Processing Time | ~140ms/frame |
| FPS | ~6 |
| Marker Type | Single ArUco (DICT_4X4_50) |
| Marker Size | 18cm (A4 printable) |
| Source Lines | ~490 (2 files) |

### Timing Breakdown
```
gray:3ms  CLAHE:20ms  bgr:2ms  detect:110ms  total:135ms
```

## Completed

### Vision System
- [x] ArUco marker detection (DICT_4X4_50)
- [x] CLAHE preprocessing for robust detection
- [x] 6-DOF pose estimation (solvePnP)
- [x] Camera calibration (720p, 0.14 reprojection error)
- [x] Corner refinement (CORNER_REFINE_CONTOUR with crash handling)
- [x] Timing instrumentation
- [x] **Codebase minimization** (3,329 -> ~490 lines)

### Position Calculation
- [x] Camera -> Body -> World coordinate transform
- [x] Multi-marker weighted fusion
- [x] Low-pass position filtering (alpha=0.7)

### GPS Emulation (MAVLink)
- [x] VISION_POSITION_ESTIMATE to FC
- [x] Heading/yaw forwarding
- [x] Confidence-to-covariance mapping
- [x] ENU->NED conversion
- [x] SET_GPS_GLOBAL_ORIGIN for non-GPS arming

### Infrastructure
- [x] RPi Zero 2W deployment
- [x] HTTP position server (JSON + debug JPEG)
- [x] SITL validation (basic, guided flight, circle pattern)

## Next: Real Drone Testing

### Pre-Flight Checklist
1. [ ] Wire RPi to FC (see docs/WIRING.md - SpeedyBee F405 V3)
2. [x] Mount camera facing up on drone frame
3. [ ] Set FC parameters via Mission Planner (see docs/FC_CONFIG.md)
4. [x] Mount marker(s) on ceiling above test area
5. [ ] Measure marker position(s) and update marker_map.yaml

### Test Sequence
1. [ ] **Bench test** - RPi + FC powered on bench, verify VISION_POSITION_ESTIMATE received
2. [ ] **EKF convergence** - Verify FC EKF accepts vision data (check MAVLink Inspector)
3. [ ] **Ground test** - Drone on ground, verify position reading is stable
4. [ ] **Tethered hover** - Safety tether, take off in Loiter mode, verify position hold
5. [ ] **Free hover** - Remove tether, hover test under single marker
6. [ ] **Movement test** - Move between markers (if multiple deployed)

### What to Watch For
- EKF position error should converge to <0.5m
- No "toilet bowl" oscillation in Loiter (indicates yaw misalignment)
- Position should not jump when marker is lost/regained
- ~6 Hz update rate should be sufficient for stable hover

## File Structure
```
src/vision_gps.py      (~393 lines) - Camera, detection, position, HTTP server, main loop
src/mavlink_bridge.py  (~97 lines)  - MAVLink connection + send vision position
src/__main__.py        (3 lines)    - Entry point
```

---

*Last updated: 2026-02-09*
