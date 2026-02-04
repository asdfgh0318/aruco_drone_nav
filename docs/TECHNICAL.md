# Technical Documentation

## System Overview

The ArUco Vision GPS system provides indoor positioning for drones using ceiling-mounted ArUco markers. It acts as a GPS substitute, sending position estimates to the flight controller via MAVLink.

## Architecture

```
Ceiling Markers ──► USB Camera ──► RPi Zero 2W ──► Flight Controller
                    (MJPG 720p)    - CLAHE preprocessing
                                   - ArUco detection
                                   - Pose estimation
                                   - Position calculation
                                   - VISION_POSITION_ESTIMATE
```

## Detection Pipeline

### 1. Frame Capture
- **Format**: MJPG (required for 30fps at 720p, YUYV limited to 10fps)
- **Resolution**: 1280x720
- **Buffer**: Single frame buffer (minimizes latency)
- **Threading**: Dedicated capture thread

### 2. CLAHE Preprocessing
CLAHE (Contrast Limited Adaptive Histogram Equalization) improves detection in varying lighting.

```python
clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
gray_clahe = clahe.apply(gray)
```

**Parameters**:
- `clipLimit=2.5`: Contrast limiting threshold
- `tileGridSize=(8,8)`: 8x8 tiles for local adaptation

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
success, rvec, tvec = cv2.solvePnP(
    obj_points,      # 3D marker corners
    marker_corners,  # 2D image corners
    camera_matrix,
    dist_coeffs
)
```

### 5. Position Calculation
Transform marker-relative pose to world coordinates:
```python
drone_position = marker_world_position - relative_offset
```

## Performance (RPi Zero 2W)

### Current Metrics
| Metric | Value |
|--------|-------|
| Resolution | 1280x720 |
| Detection Rate | 95-100% |
| Total Time | ~270ms/frame |
| FPS | ~3.7 |

### Timing Breakdown
| Step | Time | % |
|------|------|---|
| Frame grab | 0ms | 0% (buffered) |
| BGR->Gray | 3ms | 1% |
| CLAHE | 20ms | 7% |
| Gray->BGR | 2ms | 1% |
| **ArUco detect** | **250ms** | **91%** |

### Bottleneck Analysis
The ArUco detector is the main bottleneck (91% of processing time). It performs:
1. Adaptive thresholding (3 iterations: win sizes 3, 13, 23)
2. Contour detection
3. Quadrilateral approximation
4. Corner refinement
5. Bit pattern extraction and matching

## Coordinate Frames

### Frame Definitions
```
Camera Frame (OpenCV)     →    World Frame (ENU)
      Z (forward/up)               X (East)
      |                            |
      |                            |
      +---- X (right)              +---- Y (North)
     /                            /
    Y (down)                     Z (Up)
```

### MAVLink Conversion (ENU to NED)
```python
# ENU (estimator) to NED (MAVLink)
mavlink_x = enu_y    # North = ENU Y
mavlink_y = enu_x    # East = ENU X
mavlink_z = -enu_z   # Down = -ENU Z
```

## Configuration

### system_config.yaml
```yaml
camera:
  device_id: 0
  width: 1280
  height: 720
  fps: 30
  # exposure_time_us: 5000  # Manual exposure (uncomment if needed)

aruco:
  dictionary: "DICT_4X4_50"
  marker_size_m: 0.18       # 18cm for A4 paper

control:
  loop_rate_hz: 20
```

### marker_map.yaml
```yaml
markers:
  - id: 0
    position: [0.0, 0.0, 3.0]  # X, Y, Z (ceiling height in meters)
    orientation: 0              # Rotation in degrees
    description: "Origin marker"
```

### camera_params.yaml
```yaml
camera_matrix:
  - [458.18, 0.0, 323.35]
  - [0.0, 458.35, 243.95]
  - [0.0, 0.0, 1.0]
distortion_coefficients:
  - [-0.427, 0.179, 0.001, -0.0001, 0.113]
image_width: 640
image_height: 480
reprojection_error: 0.142
```

## Known Issues

### OpenCV CORNER_REFINE_CONTOUR Crash
Rare assertion error in OpenCV's contour refinement:
```
cv2.error: (-215:Assertion failed) nContours.size() >= 2 in function '_interpolate2Dline'
```

**Solution**: Wrap detection in try-except, skip bad frames:
```python
try:
    corners, ids, rejected = detector.detectMarkers(frame)
except cv2.error as e:
    logger.warning(f"Detection error (skipping frame): {e}")
    corners, ids, rejected = [], None, []
```

### YUYV Format Limitation
YUYV format at 720p is limited to 10fps. Solution: Use MJPG format.
```python
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
```

## HTTP API

### Endpoints

**GET /position**
Returns JSON with position and timing:
```json
{
  "timestamp": 1707057600.123,
  "x": 0.35,
  "y": -1.06,
  "z": 1.75,
  "yaw": 83.5,
  "marker_ids": [0],
  "confidence": 1.0,
  "detection_rate": 0.98,
  "timing": {
    "grab": 0,
    "gray": 3,
    "clahe": 20,
    "bgr": 2,
    "detect": 250,
    "total": 275
  }
}
```

**GET /debug-frame**
Returns JPEG image with detection overlay.

## Optimization Opportunities

### Priority 1: Resolution Reduction
- Change from 1280x720 to 640x480
- Reduces pixels by 75%
- Expected: 3-4x faster detection

### Priority 2: Conditional CLAHE
- Try detection on raw grayscale first
- Only apply CLAHE if detection fails
- Saves ~20ms when lighting is good

### Priority 3: Detection Parameter Tuning
- Reduce `adaptiveThreshWinSizeMax` (23 -> 15)
- Increase `minMarkerPerimeterRate` (0.01 -> 0.05)
- Enable `aprilTagQuadDecimate = 2.0`

---

*Last updated: 2026-02-04*
