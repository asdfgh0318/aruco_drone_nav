# Project TODO List

## Priority: Critical - Flight Testing

### Test Sequence
- [x] **Bench test** - RPi + FC on bench, GPS_INPUT confirmed (fix=3, sats=12)
- [ ] **EKF convergence** - Verify FC EKF accepts GPS data, position error <0.5m
- [ ] **Ground test** - Drone on ground under marker, verify stable position reading
- [ ] **Tethered hover** - Safety tether, Loiter mode, verify position hold
- [ ] **Free hover** - Remove tether, hover under single marker
- [ ] **Movement test** - Move between markers (if multiple deployed)

## Priority: Medium

### IMU Integration
- [x] **IMU fusion** - ZYX Euler body-to-world rotation from ATTITUDE messages
- [ ] **IMU update rate** - Currently ~4Hz vs camera 7.5Hz, causes yaw-position coupling
- [ ] **Static bias compensation** - Roll ~-4.7°, pitch ~-6.9° offset when level

### Performance
- [ ] **FPS optimization** - Currently ~7.5 FPS, target 10+
  - Reduce resolution to 640x480
  - Conditional CLAHE (skip when detection succeeds)
  - Detection parameter tuning

### Robustness
- [ ] **Camera focus lock** - Add to Camera.start() so autofocus is disabled automatically
- [ ] **Clean shutdown** - Fix OpenCV error on camera thread cleanup
- [ ] **Recalibrate camera at 1280x720** - Currently scaling from 640x480

### Coverage
- [ ] **Multi-marker deployment** - Multiple ceiling markers for larger area

## Priority: Low
- [ ] **ROI-based detection** - Search near last marker position first

## Completed

### Session 2026-02-10: GPS Emulation + IMU Fusion
- [x] Custom firmware build (AP_GPS_MAV, VISUALODOM, EK3_EXTERNAL_NAV)
- [x] GPS_INPUT end-to-end working (fix=3, sats=12)
- [x] Fresh RPi setup (UART, dependencies, code sync)
- [x] Camera autofocus lock (manual: focus_absolute=0)
- [x] Camera matrix auto-scaling (640x480 calibration → 1280x720 runtime)
- [x] solvePnP temporal consistency (IPPE_SQUARE → ITERATIVE with prev frame)
- [x] IMU body-to-world transform (ZYX Euler rotation)
- [x] CAM_TO_BODY z-fix for upward-facing camera
- [x] Debug CSV logging
- [x] Position precision: 3-9mm XY, 1-3mm Z (stationary)

### Session 2026-02-09: Codebase Minimization + RPi Testing
- [x] Rewrite src/ from 3,329 lines to ~490 lines (2 files)
- [x] Delete all legacy code (Diamond, ChArUco, unused MAVLink commands)
- [x] Test on RPi Zero 2W - 99-100% detection, ~6 FPS, ~140ms/frame

### Session 2026-02-04: Single ArUco + CLAHE + Timing
- [x] Switch from Diamond markers to single ArUco (DICT_4X4_50)
- [x] Add CLAHE preprocessing (robust 95-100% detection)
- [x] Add timing instrumentation

### Session 2026-02-03: Diamond Markers + Remote Calibration
- [x] Implement ChArUco Diamond detection
- [x] Remote camera calibration over network

### Session 2026-01-23: RPi Deployment & Debug Tools
- [x] Deploy system to RPi Zero 2W
- [x] MJPEG camera streaming, Debug GUI

---

*Last updated: 2026-02-10*
