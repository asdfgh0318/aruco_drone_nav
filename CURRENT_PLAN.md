# ArUco Drone Navigation - Current Plan

## Architecture
RPi + Camera = Visual GPS emulator. Detects markers, calculates position, sends to FC. FC handles everything else.

## Current Status (2026-02-04)

### Performance on RPi Zero 2W
| Metric | Value |
|--------|-------|
| Resolution | 1280x720 (MJPG) |
| Detection Rate | 95-100% |
| Processing Time | ~270ms/frame |
| FPS | ~3.7 |
| Marker Type | Single ArUco (DICT_4X4_50) |
| Marker Size | 18cm (A4 printable) |

### Timing Breakdown
```
grab:0ms  gray:3ms  CLAHE:20ms  bgr:2ms  detect:250ms  total:275ms
```

## 1. Vision System
- [x] ArUco marker detection (DICT_4X4_50)
- [x] ~~ChArUco Diamond markers~~ (reverted to single markers for simplicity)
- [x] **Single ArUco markers** (18cm, A4 printable)
- [x] **CLAHE preprocessing** for robust detection in varying lighting
- [x] 6-DOF pose estimation (solvePnP)
- [x] Camera calibration (ChArUco board, 720p, 0.14 reprojection error)
- [x] Remote calibration over network
- [x] Corner refinement (CORNER_REFINE_CONTOUR with crash handling)
- [x] Timing instrumentation (grab/gray/CLAHE/bgr/detect breakdown)
- [ ] **FPS optimization** (target: 10+ FPS)
  - [ ] Resolution reduction (640x480)
  - [ ] Conditional CLAHE (skip when detection succeeds)
  - [ ] Detection parameter tuning

## 2. Position Calculation
- [x] Marker-to-world coordinate transform
- [x] Multi-marker weighted fusion
- [x] Low-pass position filtering

## 3. GPS Emulation (MAVLink)
- [x] Send position to FC via VISION_POSITION_ESTIMATE
- [x] Heading/yaw forwarding
- [x] Confidence/quality indicator to FC (covariance mapping)
- [x] Configure FC to use external vision as position source
- [x] ENU->NED coordinate frame conversion
- [x] SET_GPS_GLOBAL_ORIGIN for non-GPS arming

## 4. Hardware & Deployment
- [x] RPi Zero 2W deployment + setup script
- [x] MJPEG camera server (remote streaming)
- [x] USB camera at 1280x720 @ 30 FPS (MJPG)
- [x] HTTP position server with timing data
- [ ] Wiring diagrams (RPi-FC, camera, power)
- [ ] Performance optimization for RPi Zero (FPS improvement)

## 5. Tools
- [x] Debug viewer with timing display (`tools/debug_viewer.py`)
- [x] Marker PDF generator (`tools/generate_markers.py`)
- [x] Remote calibration (`tools/calibrate_remote.py`)
- [x] MAVLink test tool
- [x] Detection test tool
- [x] SITL validation tool

## 6. Testing
- [x] Desktop detection testing
- [x] RPi detection testing (95-100% rate)
- [x] Camera calibration verified (0.14 reprojection error)
- [x] SITL validation (ArduCopter + vision position input)
- [ ] Verify FC accepts vision position data
- [ ] Hover test with vision-based position
- [ ] Full autonomous flight test

## 7. Documentation
- [x] README (updated 2026-02-04)
- [x] Technical docs
- [x] Current plan (this file)
- [ ] Wiring diagrams
- [x] FC configuration guide

## Next Steps (Priority Order)

1. **FPS Optimization** - Improve from 3.7 to 10+ FPS
   - Reduce resolution to 640x480
   - Make CLAHE conditional
   - Tune detection parameters

2. **Real Hardware Testing**
   - Connect to actual flight controller
   - Verify VISION_POSITION_ESTIMATE reception
   - Tethered hover test

## Removed (FC handles natively)
- ~~PID controller~~
- ~~Mission executor~~
- ~~Dead reckoning / position predictor~~
- ~~Arm/disarm/mode control~~
- ~~Failsafes~~
- ~~Flight recorder~~

---

*Last updated: 2026-02-04*
