# Technical Documentation

## System Overview

The ArUco Vision GPS system provides indoor positioning for drones using ceiling-mounted ArUco markers. It acts as a GPS substitute, sending position estimates to the flight controller via MAVLink.

## Architecture

```
Ceiling Markers ──► USB Camera ──► RPi Zero 2W ──► Flight Controller
                    (MJPG 720p)    - CLAHE preprocessing
                                   - ArUco detection
                                   - Pose estimation (solvePnP)
                                   - Position calculation
                                   - VISION_POSITION_ESTIMATE
```

### Source Files
| File | Lines | Purpose |
|------|-------|---------|
| `src/vision_gps.py` | ~393 | Camera, detection, position estimation, HTTP server, main loop |
| `src/mavlink_bridge.py` | ~97 | MAVLink connection, send vision position, set EKF origin |
| `src/__main__.py` | 3 | Entry point (`python3 -m src`) |

## Detection Pipeline

### 1. Frame Capture
- **Format**: MJPG (required for 30fps at 720p, YUYV limited to 10fps)
- **Resolution**: 1280x720
- **Buffer**: Single frame buffer (minimizes latency)
- **Threading**: Dedicated capture thread

### 2. CLAHE Preprocessing
```python
clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
enhanced = clahe.apply(gray)
```

### 3. ArUco Detection
```python
detector = cv2.aruco.ArucoDetector(dictionary, params)
corners, ids, rejected = detector.detectMarkers(frame)
```

**Detection Parameters** (tuned for ceiling markers at 1.5-2.5m):
```python
params.adaptiveThreshConstant = 7
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 23
params.adaptiveThreshWinSizeStep = 10
params.minMarkerPerimeterRate = 0.01   # Detect small/distant markers
params.maxMarkerPerimeterRate = 4.0
params.polygonalApproxAccuracyRate = 0.05
params.minCornerDistanceRate = 0.01
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
params.minOtsuStdDev = 5.0
```

### 4. Pose Estimation
```python
success, rvec, tvec = cv2.solvePnP(obj_points, marker_corners, camera_matrix, dist_coeffs)
```

### 5. Position Calculation
Camera → Body → World frame transforms:
```python
CAM_TO_BODY = [[0,1,0], [1,0,0], [0,0,-1]]     # Camera facing up
BODY_TO_WORLD_BASE = [[0,1,0], [1,0,0], [0,0,-1]]  # Drone faces North
drone_position = marker_world_position - (yaw_rotation @ BODY_TO_WORLD_BASE @ CAM_TO_BODY @ tvec)
```

Low-pass filter (alpha=0.7) with yaw wraparound handling.

## Performance (RPi Zero 2W)

| Metric | Value |
|--------|-------|
| Resolution | 1280x720 |
| Detection Rate | 99-100% |
| Total Time | ~140ms/frame |
| FPS | ~6 |

### Timing Breakdown
| Step | Time | % |
|------|------|---|
| BGR->Gray | 3ms | 2% |
| CLAHE | 20ms | 14% |
| Gray->BGR | 2ms | 1% |
| **ArUco detect** | **110ms** | **79%** |

## Coordinate Frames

### MAVLink Conversion (ENU to NED)
```python
mavlink_x = enu_y    # North = ENU Y
mavlink_y = enu_x    # East = ENU X
mavlink_z = -enu_z   # Down = -ENU Z
```

## HTTP API (Stream Mode)

**GET /position** - JSON with position and timing data
**GET /debug-frame** - JPEG image of current camera frame

## Mission & Telemetry Tools

### Coordinate Transforms (Unity ↔ NED)

The VR drone path planner uses Unity model-local coordinates:
- **Unity**: X=East, Y=Up, Z=North
- **ArduPilot NED**: X=North, Y=East, Z=Down

Verified by checking `detailedSegments`: `verticalChange == |dY|` and
`lengthHorizontal == sqrt(dX² + dZ²)` for all segments. Yaw is CW from +Z (North).

```
Unity → NED:
  NED.North = Unity.Z
  NED.East  = Unity.X
  NED.Down  = -Unity.Y
  alt_rel   = Unity.Y     (Y is up = altitude above model origin)

NED → Unity (reverse):
  Unity.X = NED.East       (NED.Y)
  Unity.Y = -NED.Down      (-NED.Z)
  Unity.Z = NED.North      (NED.X)
```

Yaw: both systems use clockwise-from-North (Unity measures CW from +Z) — no remapping needed.

### NED to fake lat/lon (indoor missions)
```python
lat = origin_lat + north_m / 111111.0
lon = origin_lon + east_m / (111111.0 * cos(origin_lat_rad))
```

### Tools

| Tool | Purpose |
|------|---------|
| `tools/vr_to_waypoints.py` | Convert VR planner JSON → ArduPilot `.waypoints` (QGC WPL 110) |
| `tools/tlog_to_vr_json.py` | Convert Mission Planner `.tlog` → VR planner JSON format |
| `tools/glb_viewer.html` | 3D mission viewer — GLB models, JSON paths, ArduPilot waypoints |

### 3D Mission Viewer (`tools/glb_viewer.html`)

Standalone HTML/JS viewer using Three.js. No build step required — open directly in a browser or serve via `server.py` for convert features.

**Features:**
- Load GLB 3D models, VR planner JSON missions, ArduPilot `.waypoints` files
- Drag & drop support, GPS origin + StartAlt coordinate conversion
- Layer toggle panel (flight path, waypoints, ArduPilot WPs, grid)
- Click for Unity/NED coordinates, hover for waypoint details
- Keyboard shortcuts: R (reset), G (grid), T (trajectory), W (waypoints), A (ArduPilot)
- Interactive tutorial wizard (7 steps, `?` to replay, auto-shows on first visit)
- Sample files in `viewer/samples/` for testing

## Known Issues

### OpenCV CORNER_REFINE_CONTOUR Crash
Rare assertion error handled by try-except, skipping bad frames.

### YUYV Format Limitation
YUYV at 720p limited to 10fps. Solution: MJPG format.

---

*Last updated: 2026-02-23*
