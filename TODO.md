# Project TODO List

## Priority: Critical

### Immediate Next Steps
- [ ] **Investigate althold timeout** — althold worked ~7 min then stopped (vibrations/heat/detection loss?)
  - Reproduce the issue in flight
  - Add auto-save frame on N consecutive detection failures (capture what camera sees when it fails)
  - Check RPi CPU temp during flight (`vcgencmd measure_temp`)
  - Based on captured frame: soft-mount camera if vibration blur, fix focus if shifted
- [x] **Calibrate camera level offset + IMU correction** — completed
  - Level calibration tool records tvec under marker → computes R_tilt
  - IMU pitch/roll corrects for dynamic drone tilt
  - Angle-based position estimation (avoids R_cm and improper R_CB)
  - solvePnPGeneric disambiguation (marker Z verticality check)
  - Tested: position stable when tilting drone ±15°
- [ ] **Fix camera mounting** — soft-mount with vibration dampening if blur confirmed
- [ ] **Extended flight tests** — multiple flights to characterize reliability

### Flight Test Sequence
- [x] **Bench test** — GPS_INPUT confirmed (fix=3, sats=12)
- [x] **EKF convergence** — Position displayed in Mission Planner
- [x] **FC params set** — GPS_TYPE=14, EK3_SRC1_POSXY=3, EK3_SRC1_POSZ=1 (baro)
- [x] **Throttle reduced** — MOT_THST_HOVER adjusted
- [x] **Althold hover** — Successful ~7 min hover with GPS emulation + baro altitude
- [ ] **Loiter mode** — GPS-based position hold
- [ ] **Extended hover** — 15+ min reliability test

## Priority: Medium

### Performance
- [x] **Recalibrate camera at 1280x720** — fixes yaw-position coupling from non-uniform scaling
- [ ] **FPS optimization at 1280x720** — currently ~3.7 FPS, target 10+
  - Conditional CLAHE (skip when detection succeeds)
  - Detection parameter tuning
  - ROI-based detection near last marker position

### Robustness
- [x] **Camera focus lock** — add to Camera.start() so autofocus is disabled automatically

### Coverage
- [x] **Multi-marker deployment** — 18 markers (IDs 0-17) deployed across faculty building
- [ ] **Full corridor flight test** — test navigation across all 18 markers
- [ ] **Marker orientation tuning** — verify 180° orientation produces correct yaw

## Completed

### Session 2026-03-30: Multi-Marker Deployment + MAVLink2 + Marker Placement Tool
- [x] Multi-marker map: 18 markers (IDs 0-17) placed via GLB viewer across faculty corridor
- [x] GLB viewer marker placement mode (M key): click ceiling to place, YAML import/export
- [x] Per-marker height from click point with optional fixed override
- [x] MAVLink2 enabled (`mavlink20=True`) — fixes GPS_INPUT yaw (was MAVLink1 extension, silently dropped)
- [x] FC position feedback: GLOBAL_POSITION_INT → ENU → second drone arrow on live map
- [x] Per-marker weights in position JSON and live map (green=active with %, gray=inactive)
- [x] Live map YAML import button for marker visualization
- [x] Live map plan view fix (North=up, East=right — was mirrored "from below" view)
- [x] Generated all 50 ArUco marker PDFs (DICT_4X4_50, 18cm on A4)
- [x] Fixed generate_markers.py broken import (aruco_detector module removed)
- [x] Updated RPi IP to 192.168.213.251

### Session 2026-03-26: IMU-Corrected Position + solvePnP Fix + Live Map
- [x] Discovered solvePnP IPPE ambiguity (two solutions flip randomly at perpendicular viewing)
- [x] Fixed with solvePnPGeneric + marker Z verticality disambiguator
- [x] Discovered R_cm is unreliable for position at near-perpendicular viewing
- [x] Implemented angle-based position estimation: tvec angles + IMU pitch/roll + level calibration
- [x] Camera level calibration tool (tools/calibrate_level.py) — records tvec for R_tilt computation
- [x] Browser live map (tools/live_map.html) with pitch/roll/SUM gauges
- [x] Position pipeline diagram (docs/position_pipeline.html)
- [x] GPS_INPUT accuracy tuned: hdop=0.3, horiz=0.1m, vert=0.1m
- [x] Migrated to NixOS RPi (user: mtj, host: pi.local)
- [x] Camera recalibrated at 1280x720, focus lock at infinity

### Session 2026-03-18: Bookworm Compatibility + GPS Emulation Switch + Althold Flight
- [x] Fresh RPi Bookworm image flashed and configured (IP: 10.40.41.251)
- [x] Fixed OpenCV 4.6.0 compatibility (old ArUco API: `DetectorParameters_create` + `detectMarkers`)
- [x] Fixed segfault: create detector before camera start (OpenCV thread safety)
- [x] Fixed pymavlink compatibility (fallback for `vision_position_estimate_send` without covariance)
- [x] Added solvePnP sanity check (discard >10m / diverging rvec, reset temporal state)
- [x] Switched from VISO to GPS emulation mode (ArduPilot doesn't support Z via visual odometry)
- [x] FC params: GPS_TYPE=14, EK3_SRC1_POSXY=3, EK3_SRC1_POSZ=1 (baro), VISO_TYPE=0
- [x] Throttle reduced, althold tested — successful ~7 min hover
- [x] VISO approach preserved as git tag `viso-experimental`

### Session 2026-02-23: GLB Viewer Tutorial + Sample Files
- [x] Interactive tutorial wizard for `tools/glb_viewer.html` (7 steps, spotlight highlights, Back/Next/Skip)
- [x] Auto-shows on first visit (`localStorage`), replay with `?` key or navbar button
- [x] Wrapped file buttons and coordinate inputs in groups for clean tutorial targeting
- [x] Sample mission files added to `viewer/samples/` (JSON, .waypoints, square mission)

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

*Last updated: 2026-03-30*
