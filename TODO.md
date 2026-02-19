# Project TODO List

## Priority: Critical

### Immediate Next Steps (before next flight)
- [ ] **Fix camera mounting** — camera needs proper fixed position relative to drone frame
- [ ] **Reduce throttle PWM** — drone is overpowered, adjust MOT_THST_HOVER or throttle range
- [ ] **Retry althold** — after fixing camera mount and throttle

### Flight Test Sequence
- [x] **Bench test** — GPS_INPUT confirmed (fix=3, sats=12)
- [x] **EKF convergence** — VISO position displayed in Mission Planner (2026-02-13)
- [x] **FC params set** — VISO_TYPE=1, EK3_SRC1_POSXY/Z/YAW=6, COMPASS_USE=0, GPS_TYPE=0
- [ ] **Ground test** — Drone on ground under marker, verify stable position reading
- [ ] **Tethered hover** — Safety tether, althold mode
- [ ] **Free hover** — Remove tether, hover under single marker

## Priority: Medium

### Performance
- [ ] **Recalibrate camera at 1280x720** — fixes yaw-position coupling from non-uniform scaling
- [ ] **FPS optimization at 1280x720** — currently ~3.7 FPS, target 10+
  - Conditional CLAHE (skip when detection succeeds)
  - Detection parameter tuning
  - ROI-based detection near last marker position

### Robustness
- [ ] **Camera focus lock** — add to Camera.start() so autofocus is disabled automatically

### Coverage
- [ ] **Multi-marker deployment** - Multiple ceiling markers for larger area

## Completed

### Session 2026-02-19: Mission & Telemetry Converter Tools
- [x] `tools/vr_to_waypoints.py` — VR planner JSON → ArduPilot `.waypoints` (QGC WPL 110)
  - Auto-detects VR planner JSON vs missions JSON format
  - Unity→NED coordinate transform + fake lat/lon from EKF origin
  - Waypoint type mapping (FlyThrough, StopRotate, Record360)
  - Tested with both input formats
- [x] `tools/tlog_to_vr_json.py` — Mission Planner `.tlog` → VR planner JSON format
  - Parses LOCAL_POSITION_NED, ATTITUDE, HEARTBEAT messages
  - NED→Unity coordinate transform for side-by-side path comparison
  - Configurable downsampling, time filtering, attitude inclusion
- [x] Updated TECHNICAL.md with coordinate transform documentation


### Session 2026-02-13: VISO End-to-End + Fresh RPi Setup
- [x] Fresh RPi Trixie image (Debian 13, arm64) via RPi Imager
- [x] Full RPi setup: SSH key, UART, deps, pymavlink
- [x] VISO mode confirmed: position visible in Mission Planner
- [x] FC params set: VISO_TYPE=1, EK3_SRC1_POSXY/Z/YAW=6, COMPASS_USE=0, GPS_TYPE=0
- [x] EKF origin auto-set working

### Session 2026-02-11: Position Fix + VISO Migration + Terminal Map
- [x] Position formula fixed: correct solvePnP inverse (`-R^T @ tvec`)
- [x] Vision yaw from full rotation chain (`R_bw = R_mw @ R_cm^T @ R_CB^T`)
- [x] Position is now IMU-independent (solvePnP + marker pose only)
- [x] Visual odometry migration (code side): VISION_POSITION_ESTIMATE with auto EKF origin
- [x] Terminal mini map tool (`tools/terminal_map.py`) — curses, works over SSH
- [x] 640x480 resolution test: ~20 FPS, ~22mm XY precision
- [x] Confirmed yaw-position coupling from non-uniform camera matrix scaling
- [x] First althold flight test (crashed)

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

*Last updated: 2026-02-19 (mission tools added)*
