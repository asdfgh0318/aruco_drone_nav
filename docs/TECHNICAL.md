# Technical Documentation

## System Overview

The ArUco Drone Navigation System is designed for indoor autonomous flight using visual markers. This document covers the technical implementation details.

## Coordinate Frames

### Frame Definitions

```
Camera Frame (OpenCV)     →    Drone Body Frame    →    World Frame (ENU)
      Z (forward)                   X (forward)            X (East)
      |                             |                      |
      |                             |                      |
      +---- X (right)               +---- Y (right)        +---- Y (North)
     /                             /                      /
    Y (down)                      Z (down)               Z (Up)
```

### Transformations

1. **Camera to Body**: Accounts for camera mounting (facing up)
   - Camera Z → Body -Z
   - Camera X → Body Y
   - Camera Y → Body X

2. **Body to World**: Uses drone's yaw angle
   - Body X → World Y (when yaw=0, drone faces North)
   - Body Y → World X
   - Body Z → World -Z

## Module Details

### 1. ArUco Detector (`aruco_detector.py`)

**Purpose**: Detect ArUco markers and estimate 6-DOF pose

**Key Classes**:
- `MarkerDetection`: Dataclass containing marker ID, corners, rvec, tvec, distance
- `ArucoDetector`: Main detection class with threaded camera capture

**Detection Pipeline**:
1. Capture frame from USB camera (threaded)
2. Convert to grayscale
3. Detect markers using `cv2.aruco.ArucoDetector`
4. Estimate pose with `cv2.solvePnP` for each marker
5. Return list of `MarkerDetection` objects

**Configuration**:
```yaml
aruco:
  dictionary: "DICT_6X6_250"  # 6x6 bit pattern, 250 unique IDs
  marker_size_m: 0.20         # Physical size in meters
```

**Supported Dictionaries**:
- DICT_4X4_50/100/250/1000
- DICT_5X5_50/100/250/1000
- DICT_6X6_50/100/250/1000 (recommended)
- DICT_7X7_50/100/250/1000

### 2. Position Estimator (`position_estimator.py`)

**Purpose**: Transform marker poses to world-frame drone position

**Key Classes**:
- `MarkerConfig`: Marker ID with world position and orientation
- `DroneState`: Estimated position, yaw, confidence
- `PositionEstimator`: Multi-marker position estimation
- `SimplePositionEstimator`: Single-marker offset calculation (Phase 1)

**Algorithm**:
1. For each detected marker:
   - Look up marker's world position from map
   - Transform marker-relative pose to world frame
   - Calculate drone position: `drone_pos = marker_world_pos - relative_offset`

2. Multi-marker fusion:
   - Weight by inverse distance (closer markers more accurate)
   - Weighted average of positions
   - Handle yaw wraparound for averaging

**Filtering**:
- Low-pass filter on position (configurable alpha)
- Smooths noise while maintaining responsiveness

### 3. MAVLink Interface (`mavlink_interface.py`)

**Purpose**: Communication with ArduCopter flight controller

**Key Classes**:
- `FlightMode`: Enum of ArduCopter modes (GUIDED, LOITER, LAND, etc.)
- `TelemetryData`: Current drone state from FC
- `MAVLinkInterface`: Connection and command handling

**Connection**:
```python
mav = MAVLinkInterface("/dev/serial0", baud=921600)
mav.connect(timeout=30.0)  # Waits for heartbeat
```

**Commands**:
| Method | MAVLink Message | Description |
|--------|-----------------|-------------|
| `arm()` | MAV_CMD_COMPONENT_ARM_DISARM | Arm motors |
| `disarm()` | MAV_CMD_COMPONENT_ARM_DISARM | Disarm motors |
| `set_mode()` | SET_MODE | Change flight mode |
| `takeoff()` | MAV_CMD_NAV_TAKEOFF | Command takeoff |
| `land()` | MAV_CMD_NAV_LAND | Command landing |
| `send_velocity()` | SET_POSITION_TARGET_LOCAL_NED | Velocity command |
| `send_position_target()` | SET_POSITION_TARGET_LOCAL_NED | Position command |

**Velocity Command (Primary Control)**:
```python
# Body frame velocity command
mav.send_velocity(
    vx=0.5,      # Forward (m/s)
    vy=0.0,      # Right (m/s)
    vz=-0.1,     # Down (m/s), negative = climb
    yaw_rate=0.0 # Yaw rate (rad/s)
)
```

### 4. Mission Executor (`mission_executor.py`)

**Purpose**: Parse and execute waypoint missions

**State Machine**:
```
IDLE → LOADING → READY → TAKING_OFF → NAVIGATING ↔ AT_WAYPOINT → LANDING → COMPLETE
                              ↓
                          ABORTED/ERROR
```

**Mission Format**:
```json
{
  "mission_id": "string",
  "waypoints": [
    {"x": float, "y": float, "z": float, "yaw": float, "hold_time": float}
  ],
  "settings": {
    "max_speed": float,
    "position_tolerance": float,
    "yaw_tolerance": float
  }
}
```

**Navigation Algorithm**:
1. Calculate error vector to waypoint
2. Apply P-control: `velocity = Kp * error`
3. Limit to max_speed
4. Send velocity command
5. Check if within tolerance → advance to next waypoint

### 5. Position Predictor (`position_predictor.py`)

**Purpose**: Dead reckoning when markers temporarily lost

**Algorithm**:
1. Maintain rolling history of positions with timestamps
2. Estimate velocity using exponential smoothing
3. On marker loss:
   - Predict position: `pos = last_pos + velocity * dt`
   - Decay confidence over time
   - Timeout after max_prediction_time

**Velocity Estimation**:
```python
# Exponential smoothing
alpha = 0.3
new_velocity = (position - prev_position) / dt
velocity = alpha * new_velocity + (1 - alpha) * velocity
```

### 6. Flight Recorder (`flight_recorder.py`)

**Purpose**: Record flight path for VR feedback

**Recording Format**:
```json
{
  "recording_id": "flight_20240114_103000",
  "mission_id": "original_mission_id",
  "start_time": "2024-01-14T10:30:00Z",
  "samples": [
    {"t": 0.0, "x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0, "markers": [0], "conf": 1.0}
  ]
}
```

**Features**:
- Configurable sample rate (default 10 Hz)
- Rate-limited sampling with `add_sample()`
- Thread-safe recording
- Automatic timestamping

## Control Architecture

### Control Loop (20 Hz default)

```python
while running:
    # 1. Detect markers
    detections = detector.detect()

    # 2. Estimate position (or predict if lost)
    if detections:
        state = estimator.estimate(detections)
        predictor.update(state)
    else:
        state = predictor.predict()

    # 3. Calculate control output
    error = target - state.position
    velocity = pid.update(error)

    # 4. Send command
    mavlink.send_velocity(velocity)

    # 5. Record (if enabled)
    recorder.add_state(state)

    # 6. Rate limit
    sleep(loop_period - elapsed)
```

### PID Controller

Simple P/PD control for position:
- **Kp** = 0.5 (position proportional)
- **Kd** = 0.1 (derivative for damping)
- **Ki** = 0.0 (no integral, handled by FC)

Output limited to max_velocity (default 0.5 m/s horizontal, 0.3 m/s vertical)

## Performance Considerations

### RPi Zero Optimization

- Camera resolution: 640x480 max (320x240 for better performance)
- Target frame rate: 15-20 FPS
- Threaded camera capture to avoid blocking
- Grayscale detection (color not needed for ArUco)

### Latency Budget

| Component | Typical Latency |
|-----------|-----------------|
| Camera capture | 30-50 ms |
| ArUco detection | 10-20 ms |
| Pose estimation | 5-10 ms |
| MAVLink TX | 1-2 ms |
| **Total** | ~50-80 ms |

## Safety Systems

### Marker Loss Handling

1. **Detection**: No markers found for 1+ frames
2. **Prediction**: Use dead reckoning (up to 2 seconds)
3. **Timeout**: Trigger failsafe landing

### Failsafe Triggers

- Marker loss timeout exceeded
- Low battery voltage (< 10.5V for 3S)
- Signal handler (Ctrl+C) - graceful shutdown
- Exception in control loop

### Failsafe Actions

1. Log warning/error
2. Set mode to LAND
3. Stop control loop
4. Save recording (if active)

## Configuration Reference

### system_config.yaml

```yaml
serial:
  port: "/dev/serial0"          # Serial port
  baud: 921600                  # Baud rate

camera:
  device_id: 0                  # Camera index
  width: 640
  height: 480
  fps: 30

aruco:
  dictionary: "DICT_6X6_250"
  marker_size_m: 0.20

control:
  position_kp: 0.5
  position_ki: 0.0
  position_kd: 0.1
  max_velocity_xy: 0.5
  max_velocity_z: 0.3
  position_tolerance: 0.10
  loop_rate_hz: 20

safety:
  max_altitude: 2.5
  min_altitude: 0.3
  marker_loss_timeout: 2.0
  low_battery_voltage: 10.5
```

### marker_map.yaml

```yaml
markers:
  - id: 0
    position: [0.0, 0.0, 3.0]   # X, Y, Z (ceiling height)
    orientation: 0               # Rotation in degrees
    description: "Origin marker"
```

## Testing

### SITL Testing

ArduCopter Software-In-The-Loop simulation:
```bash
# Start SITL (in ArduPilot directory)
sim_vehicle.py -v ArduCopter --console --map

# Connect with UDP
python -m src.main --mode ground_test --config config/sitl_config.yaml
```

### Hardware Testing Sequence

1. **Camera test**: `tools/test_aruco_detection.py`
2. **MAVLink test**: `tools/test_mavlink.py`
3. **Ground test**: `--mode ground_test` (no flight)
4. **Tethered hover**: `--mode hover` with safety tether
5. **Free flight**: After successful tethered tests

---

*Last updated: 2024-01-14*
