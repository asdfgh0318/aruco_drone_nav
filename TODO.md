# Project TODO List

## Priority: High

### FPS Optimization (Current: 3.7 FPS, Target: 10+ FPS)
- [ ] **Reduce resolution to 640x480** - Quarter the pixel count
  - Update `config/system_config.yaml`
  - Test detection at ceiling distance (~1.7m)
  - Expected: 60-75% FPS improvement

- [ ] **Conditional CLAHE** - Skip when detection succeeds
  - Try detection on raw grayscale first
  - Only apply CLAHE on failed detection
  - Expected: 50-70% FPS gain when lighting is good

- [ ] **Detection parameter tuning**
  - Tighten `adaptiveThreshWinSizeMax` (23 -> 15)
  - Increase `minMarkerPerimeterRate` (0.01 -> 0.05)
  - Enable `aprilTagQuadDecimate = 2.0`

### Hardware Integration
- [ ] **Real FC testing** - Connect to actual flight controller
  - Verify VISION_POSITION_ESTIMATE reception
  - Check EKF convergence
  - Test position accuracy

- [ ] **Tethered hover test** - First real flight
  - Safety tether attached
  - Single marker hover
  - Verify position hold

## Priority: Medium

### Performance Monitoring
- [ ] **Add FPS counter to HTTP endpoint**
  - Include in JSON response
  - Track rolling average

- [ ] **ROI-based detection** - Search near last marker position
  - Track last detection location
  - Search expanded ROI first
  - Fall back to full frame

### Documentation
- [ ] **Wiring diagrams** - Create once final hardware selected
  - RPi Zero to FC serial connection
  - Camera connection
  - Power distribution

## Priority: Low

### Features
- [ ] **Multi-marker support** - Deploy multiple ceiling markers
- [ ] **Dynamic marker discovery** - Auto-register new markers
- [ ] **Web interface** - Real-time monitoring dashboard

### Future Optimization
- [ ] **Frame skipping** - Process every 2nd/3rd frame
- [ ] **IMU integration** - Predict position between detections

## Completed

### Session 2026-02-04: Single ArUco + CLAHE + Timing
- [x] Switch from Diamond markers to single ArUco (DICT_4X4_50)
- [x] Add CLAHE preprocessing (robust 95-100% detection)
- [x] Configure MJPG format for 30fps capture
- [x] Add timing instrumentation (grab/gray/CLAHE/bgr/detect)
- [x] Create debug_viewer.py with timing display
- [x] Add timing to HTTP JSON endpoint
- [x] Handle OpenCV CORNER_REFINE_CONTOUR crash gracefully
- [x] Update camera calibration (720p, 0.14 reprojection error)
- [x] Configure 18cm markers for A4 printing

### Session 2026-02-03: Diamond Markers + Remote Calibration
- [x] Implement ChArUco Diamond detection
- [x] Remote camera calibration over network
- [x] Wide-angle calibration support
- [x] Diamond marker generator

### Session 2026-01-23: RPi Deployment & Debug Tools
- [x] Deploy system to RPi Zero 2W
- [x] Test USB camera (640x480 @ 15 FPS)
- [x] Create MJPEG camera streaming server
- [x] Create Debug GUI with video/telemetry
- [x] Remote calibration tool
- [x] HTML documentation site

### Phase 1 & 2 - Core System
- [x] Camera calibration module
- [x] ArUco detection with pose estimation
- [x] Position estimator (marker-relative)
- [x] MAVLink interface
- [x] Mission executor (JSON parsing)
- [x] Flight recorder

---

## Performance Reference

### Current (2026-02-04)
| Metric | Value |
|--------|-------|
| Resolution | 1280x720 |
| Detection Rate | 95-100% |
| Total Time | ~270ms |
| FPS | ~3.7 |

### Timing Breakdown
| Step | Time |
|------|------|
| Frame grab | 0ms (buffered) |
| BGR->Gray | 3ms |
| CLAHE | 20ms |
| Gray->BGR | 2ms |
| **ArUco detect** | **250ms** |

### Target After Optimization
| Metric | Target |
|--------|--------|
| Resolution | 640x480 |
| FPS | 10+ |
| Detection Rate | >90% |

---

*Last updated: 2026-02-04*
