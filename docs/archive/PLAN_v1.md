# ArUco-Based Vision Navigation System for Drones

## Overview
A Python-based navigation system for RPi Zero that uses ceiling-mounted ArUco markers to provide position feedback to an ArduCopter flight controller via MAVLink. The system enables autonomous indoor flight with mission waypoints received from a VR planning tool.

## System Architecture

```
┌─────────────────┐     JSON Mission      ┌─────────────────┐
│   VR Planning   │ ──────────────────►   │                 │
│     Software    │                       │    RPi Zero     │
│                 │ ◄──────────────────   │   (onboard)     │
└─────────────────┘   Flight Recording    └────────┬────────┘
                                                   │ UART/Serial
                                                   │ MAVLink
┌─────────────────┐                       ┌────────▼────────┐
│  Ceiling-mounted│     USB Camera        │   ArduCopter    │
│  ArUco Markers  │ ◄──── (facing up) ────│      FC         │
└─────────────────┘                       └─────────────────┘
```

## Project Structure

```
aruco_drone_nav/
├── config/
│   ├── camera_params.yaml       # Camera intrinsics from calibration
│   ├── marker_map.yaml          # ArUco marker ID → world coordinates
│   └── system_config.yaml       # Serial port, marker size, etc.
├── src/
│   ├── __init__.py
│   ├── camera_calibration.py    # Chessboard calibration utility
│   ├── aruco_detector.py        # ArUco detection and pose estimation
│   ├── position_estimator.py    # Marker pose → world position transform
│   ├── mavlink_interface.py     # MAVLink communication with FC
│   ├── mission_executor.py      # JSON mission parsing and execution
│   ├── flight_recorder.py       # Record flight path for VR feedback
│   ├── position_predictor.py    # Predict position when markers lost
│   └── main.py                  # Main control loop
├── tools/
│   ├── calibrate_camera.py      # CLI tool to run calibration
│   ├── test_aruco_detection.py  # Test marker detection standalone
│   ├── generate_markers.py      # Generate ArUco marker PDFs
│   └── test_mavlink.py          # Test MAVLink connection
├── missions/                    # Mission JSON files from VR
├── recordings/                  # Recorded flight paths
├── requirements.txt
└── README.md
```

## Implementation Phases

### Phase 1: Single Marker Navigation (Core Foundation)

**Goal:** Drone can detect one ArUco marker, hover underneath it, and accept basic movement commands.

#### 1.1 Camera Calibration Module
**File:** `src/camera_calibration.py`
- Capture chessboard images from USB camera
- Compute camera matrix and distortion coefficients using OpenCV
- Save calibration to `config/camera_params.yaml`
- Support re-calibration without code changes

#### 1.2 ArUco Detection Module
**File:** `src/aruco_detector.py`
- Initialize USB camera (320x240 or 640x480 for RPi Zero performance)
- Detect ArUco markers using `cv2.aruco` module
- Use DICT_4X4_50 or DICT_6X6_250 dictionary
- Estimate marker pose (rvec, tvec) relative to camera
- Return marker ID, position (x, y, z), and orientation

#### 1.3 Position Estimator
**File:** `src/position_estimator.py`
- Transform marker-relative position to drone-relative position
- Account for camera mounting (facing up, fixed position on drone)
- Calculate drone's position relative to marker center
- Output: drone offset from marker in meters (x_offset, y_offset, altitude)

#### 1.4 MAVLink Interface
**File:** `src/mavlink_interface.py`
- Connect to ArduCopter via UART (`/dev/serial0` or `/dev/ttyAMA0`)
- Use pymavlink library
- Implement key functions:
  - `connect()` - Establish connection, wait for heartbeat
  - `arm()` / `disarm()` - Arm/disarm motors
  - `set_mode(mode)` - Switch to GUIDED mode
  - `takeoff(altitude)` - Command takeoff
  - `send_position_target(x, y, z, yaw)` - Send SET_POSITION_TARGET_LOCAL_NED
  - `send_velocity(vx, vy, vz, yaw_rate)` - Send velocity commands
  - `get_attitude()` - Read current attitude
  - `get_position()` - Read current local position
  - `land()` - Command landing

#### 1.5 Basic Control Loop
**File:** `src/main.py` (Phase 1 version)
```python
# Pseudocode for Phase 1
while running:
    # 1. Detect ArUco marker
    marker = detector.detect()

    # 2. If marker found, calculate position offset
    if marker:
        offset = estimator.get_drone_offset(marker)

        # 3. Simple P-controller to hover under marker center
        vx = -Kp * offset.x
        vy = -Kp * offset.y
        vz = Kp * (target_altitude - offset.z)

        # 4. Send velocity command to FC
        mavlink.send_velocity(vx, vy, vz, 0)
    else:
        # Hold position (zero velocity)
        mavlink.send_velocity(0, 0, 0, 0)
```

### Phase 2: Multi-Marker Navigation with Missions

**Goal:** Drone follows waypoints from JSON mission file, using multiple markers for position reference.

#### 2.1 Marker Map Configuration
**File:** `config/marker_map.yaml`
```yaml
markers:
  - id: 0
    position: [0.0, 0.0, 3.0]      # World coordinates (x, y, z) in meters
    orientation: 0                  # Marker rotation in degrees
  - id: 1
    position: [2.0, 0.0, 3.0]
    orientation: 0
  - id: 5
    position: [2.0, 2.0, 3.0]
    orientation: 0
```

#### 2.2 World Position Estimation
**File:** `src/position_estimator.py` (extended)
- Load marker map from config
- When marker detected, calculate drone's world position:
  ```
  drone_world_pos = marker_world_pos - marker_relative_offset
  ```
- Support multiple visible markers (weighted average for accuracy)
- Track which markers are visible for redundancy

#### 2.3 Mission Executor
**File:** `src/mission_executor.py`
- Parse JSON mission file from VR team:
  ```json
  {
    "mission_id": "flight_001",
    "waypoints": [
      {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0, "hold_time": 2.0},
      {"x": 1.0, "y": 0.0, "z": 1.5, "yaw": 0, "hold_time": 1.0},
      {"x": 1.0, "y": 1.0, "z": 1.5, "yaw": 90, "hold_time": 1.0}
    ],
    "settings": {
      "max_speed": 0.5,
      "position_tolerance": 0.1
    }
  }
  ```
- State machine: IDLE → TAKEOFF → NAVIGATING → WAYPOINT_HOLD → LANDING → COMPLETE
- Waypoint reached detection (position within tolerance)
- Smooth trajectory between waypoints

#### 2.4 Position Predictor (Marker Loss Handling)
**File:** `src/position_predictor.py`
- Track recent position history with timestamps
- When markers lost:
  1. Estimate velocity from recent positions
  2. Dead-reckon position based on velocity and time
  3. Predict where known markers should appear in camera frame
  4. Limit prediction time (e.g., max 2 seconds)
  5. If prediction timeout, hold last known position
- Re-acquire position when marker seen again

#### 2.5 Flight Recorder
**File:** `src/flight_recorder.py`
- Record timestamped position and attitude data:
  ```json
  {
    "mission_id": "flight_001",
    "start_time": "2024-01-14T10:30:00Z",
    "samples": [
      {"t": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0, "marker_id": 0},
      {"t": 0.1, "x": 0.05, "y": 0.01, "z": 0.5, "yaw": 0, "marker_id": 0}
    ]
  }
  ```
- Configurable sample rate (10-20 Hz typical)
- Save to file on mission complete
- Include marker visibility data for debugging

### Phase 3: Integration and Testing Tools

#### 3.1 Marker Generator
**File:** `tools/generate_markers.py`
- Generate printable ArUco marker PDFs
- Specify marker IDs and physical size
- Output ready for printing and ceiling mounting

#### 3.2 Calibration Tool
**File:** `tools/calibrate_camera.py`
- Interactive CLI for camera calibration
- Guide user through capturing chessboard images
- Validate calibration quality

#### 3.3 Detection Tester
**File:** `tools/test_aruco_detection.py`
- Live preview of marker detection
- Display detected marker IDs, positions, and orientations
- Useful for verifying marker placement and camera setup

#### 3.4 MAVLink Tester
**File:** `tools/test_mavlink.py`
- Test connection to flight controller
- Display telemetry data
- Send test commands (with safety checks)

## Key Technical Decisions

### MAVLink Protocol
- Use `SET_POSITION_TARGET_LOCAL_NED` for position/velocity commands
- Coordinate frame: `MAV_FRAME_LOCAL_NED` (North-East-Down)
- Send commands at 10-20 Hz for smooth control
- Monitor heartbeat for connection health

### Coordinate Transformations
```
Camera Frame (OpenCV)     →    Drone Body Frame    →    World Frame
      Z (forward)                   X (forward)            X (East)
      |                             |                      |
      |                             |                      |
      +---- X (right)               +---- Y (right)        +---- Y (North)
     /                             /                      /
    Y (down)                      Z (down)               Z (Up)
```

### Control Strategy
1. **Outer loop (10 Hz):** Vision-based position estimation
2. **Inner loop (handled by ArduCopter):** Attitude and rate control
3. **P/PD controller** for position tracking initially
4. Consider PID with feedforward for smoother trajectories later

### RPi Zero Performance Considerations
- Camera resolution: 320x240 or 640x480 max
- Target frame rate: 15-20 FPS for detection
- Use threading: separate threads for camera capture and MAVLink
- Consider grayscale processing to reduce CPU load

## Dependencies

```
# requirements.txt
opencv-python>=4.5.0          # Or opencv-python-headless for RPi
opencv-contrib-python>=4.5.0  # For ArUco module
pymavlink>=2.4.0
numpy>=1.20.0
PyYAML>=6.0
```

## Configuration Files

### system_config.yaml
```yaml
serial:
  port: "/dev/serial0"
  baud: 921600

camera:
  device_id: 0
  width: 640
  height: 480
  fps: 30

aruco:
  dictionary: "DICT_6X6_250"
  marker_size_m: 0.20          # 20cm markers

control:
  position_kp: 0.5
  velocity_kp: 0.3
  max_velocity: 0.5            # m/s
  position_tolerance: 0.10     # meters

safety:
  max_altitude: 2.5            # meters
  min_altitude: 0.3
  marker_loss_timeout: 2.0     # seconds
```

## Verification Plan

### Phase 1 Testing
1. **Camera calibration:** Verify reprojection error < 0.5 pixels
2. **ArUco detection:** Test detection at various distances and angles
3. **MAVLink connection:** Verify heartbeat and telemetry reception
4. **Ground test:** Run control loop with drone disarmed, verify velocity commands
5. **Tethered hover:** Test hovering under single marker with safety tether

### Phase 2 Testing
1. **Multi-marker detection:** Verify position consistency across markers
2. **Mission parsing:** Load and validate test mission files
3. **Waypoint navigation:** Navigate between 2-3 waypoints
4. **Marker loss recovery:** Test prediction and re-acquisition
5. **Flight recording:** Verify recording matches actual flight

## Safety Considerations

1. **Always test in SITL first** (ArduCopter Software-In-The-Loop)
2. **Use tether** for initial real hardware tests
3. **Implement failsafe:** Land if no markers seen for >2 seconds
4. **Geofence:** Software limits on maximum distance from origin
5. **Kill switch:** RC transmitter should always have manual override
6. **Low battery:** Monitor battery voltage, trigger RTL if low

## Files to Create

1. `aruco_drone_nav/src/__init__.py`
2. `aruco_drone_nav/src/camera_calibration.py`
3. `aruco_drone_nav/src/aruco_detector.py`
4. `aruco_drone_nav/src/position_estimator.py`
5. `aruco_drone_nav/src/mavlink_interface.py`
6. `aruco_drone_nav/src/mission_executor.py`
7. `aruco_drone_nav/src/flight_recorder.py`
8. `aruco_drone_nav/src/position_predictor.py`
9. `aruco_drone_nav/src/main.py`
10. `aruco_drone_nav/tools/calibrate_camera.py`
11. `aruco_drone_nav/tools/test_aruco_detection.py`
12. `aruco_drone_nav/tools/generate_markers.py`
13. `aruco_drone_nav/tools/test_mavlink.py`
14. `aruco_drone_nav/config/system_config.yaml`
15. `aruco_drone_nav/config/marker_map.yaml`
16. `aruco_drone_nav/requirements.txt`
