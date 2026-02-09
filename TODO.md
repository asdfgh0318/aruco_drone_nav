# Project TODO List

## Priority: Critical - Real Drone Testing

### Hardware Setup
- [ ] **Wire RPi to FC** - UART serial (TX→RX, RX→TX), 921600 baud
- [ ] **Mount camera** - Facing up, centered on drone frame
- [ ] **Set FC parameters** - See docs/FC_CONFIG.md for full list
- [ ] **Mount ceiling markers** - Measure positions, update marker_map.yaml

### Test Sequence
- [ ] **Bench test** - RPi + FC on bench, verify VISION_POSITION_ESTIMATE received in MAVLink Inspector
- [ ] **EKF convergence** - Wait for "EKF3 IMU0 origin set", check position error <0.5m
- [ ] **Ground test** - Drone on ground under marker, verify stable position reading
- [ ] **Tethered hover** - Safety tether, Loiter mode, verify position hold
- [ ] **Free hover** - Remove tether, hover under single marker
- [ ] **Movement test** - Move between markers (if multiple deployed)

## Priority: Medium

### Performance
- [ ] **FPS optimization** - Currently ~6 FPS, target 10+
  - Reduce resolution to 640x480
  - Conditional CLAHE (skip when detection succeeds)
  - Detection parameter tuning

### Coverage
- [ ] **Multi-marker deployment** - Multiple ceiling markers for larger area
- [ ] **Wiring diagrams** - RPi-FC, camera, power

## Priority: Low
- [ ] **ROI-based detection** - Search near last marker position first
- [ ] **IMU integration** - Predict position between detections

## Completed

### Session 2026-02-09: Codebase Minimization + RPi Testing
- [x] Rewrite src/ from 3,329 lines to ~490 lines (2 files)
- [x] Delete all legacy code (Diamond, ChArUco, unused MAVLink commands)
- [x] Test on RPi Zero 2W - 99-100% detection, ~6 FPS, ~140ms/frame
- [x] Capture live camera images for documentation
- [x] Update all documentation with new performance data

### Session 2026-02-04: Single ArUco + CLAHE + Timing
- [x] Switch from Diamond markers to single ArUco (DICT_4X4_50)
- [x] Add CLAHE preprocessing (robust 95-100% detection)
- [x] Add timing instrumentation
- [x] Handle OpenCV CORNER_REFINE_CONTOUR crash gracefully

### Session 2026-02-03: Diamond Markers + Remote Calibration
- [x] Implement ChArUco Diamond detection
- [x] Remote camera calibration over network

### Session 2026-01-23: RPi Deployment & Debug Tools
- [x] Deploy system to RPi Zero 2W
- [x] MJPEG camera streaming, Debug GUI

---

*Last updated: 2026-02-09*
