# ArUco Drone Navigation - Current Plan

## Architecture
RPi + Camera = Visual GPS emulator. Detects markers, calculates position, sends to FC. FC handles everything else.

## 1. Vision System
- [x] ArUco marker detection (DICT_4X4_50)
- [x] **ChArUco Diamond markers** (4 ArUco markers per diamond, better robustness)
- [x] 6-DOF pose estimation (solvePnP)
- [x] Camera calibration (ChArUco board)
- [x] Remote calibration over network
- [x] Wide-angle calibration (8-coefficient rational model)
- [x] ChArUco board detection (sub-pixel pose accuracy)
- [x] ChArUco board generator (DICT_4X4_50)
- [x] Diamond marker generator (`tools/generate_diamonds.py`)
- [ ] Adaptive detection parameters (lighting conditions)

## 2. Position Calculation
- [x] Marker-to-world coordinate transform
- [x] Multi-marker weighted fusion
- [x] Low-pass position filtering

## 3. GPS Emulation (MAVLink)
- [x] Send position to FC via VISION_POSITION_ESTIMATE
- [x] Heading/yaw forwarding
- [x] Confidence/quality indicator to FC (covariance mapping)
- [x] Configure FC to use external vision as position source
- [x] ENU→NED coordinate frame conversion
- [x] SET_GPS_GLOBAL_ORIGIN for non-GPS arming

## 4. Hardware & Deployment
- [x] RPi Zero 2W deployment + setup script
- [x] MJPEG camera server (remote streaming)
- [x] USB camera at 640x480 @ 15 FPS
- [ ] Wiring diagrams (RPi-FC, camera, power)
- [ ] Adjust code for final flight controller
- [ ] Performance optimization for RPi Zero

## 5. Tools
- [x] Debug GUI (video + telemetry + marker map)
- [x] Bench test (position/velocity visualization)
- [x] Marker PDF generator
- [x] Chessboard PDF generator
- [x] MAVLink test tool
- [x] Detection test tool
- [x] Marker spacing calculator
- [x] SITL validation tool (tools/test_sitl.py, config/sitl_params.parm)

## 6. Testing
- [x] Desktop detection testing
- [x] RPi detection testing
- [x] Camera calibration verified
- [x] SITL validation (ArduCopter + vision position input)
- [ ] Verify FC accepts vision position data
- [ ] Hover test with vision-based position
- [ ] Full autonomous flight test

## 7. Documentation
- [x] README
- [x] Technical docs
- [x] HTML documentation site
- [ ] Wiring diagrams
- [x] FC configuration guide (EKF, vision parameters)

## Removed (FC handles natively)
- ~~PID controller~~
- ~~Mission executor~~
- ~~Dead reckoning / position predictor~~
- ~~Arm/disarm/mode control~~
- ~~Failsafes~~
- ~~Flight recorder~~
