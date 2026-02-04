# Claude Session Resume File

> **Purpose**: This file helps Claude maintain context across sessions. Read this first when resuming work on this project.

## Project Identity

- **Name**: ArUco Drone Navigation System (Vision GPS)
- **Location**: `/home/adam/ŻYCIE/PRACA/aruco_drone_nav`
- **GitHub**: `github.com/asdfgh0318/aruco_drone_nav` (private)
- **Primary User**: Adam Koszalka
- **Institution**: Warsaw University of Technology

## Project Context

### What This Project Does
Indoor drone navigation system using ceiling-mounted ArUco markers. The system acts as a "Vision GPS" - detecting markers on the ceiling, calculating world-frame position, and sending VISION_POSITION_ESTIMATE to ArduCopter. The flight controller handles all navigation, PID, and failsafes.

### Hardware Setup
- **Drone**: ArduCopter-based with Pixhawk or similar
- **Companion Computer**: Raspberry Pi Zero 2W (`aruconav.local`)
- **Camera**: USB camera facing UP toward ceiling (MJPG, 1280x720)
- **Markers**: 18cm ArUco markers (DICT_4X4_50) on A4 paper
- **Communication**: UART serial (RPi -> FC)

### Architecture
```
Ceiling Markers ──► USB Camera ──► RPi Zero 2W ──► ArduCopter FC
                                   - CLAHE preprocessing
                                   - ArUco detection
                                   - Position estimation
                                   - VISION_POSITION_ESTIMATE
```

## Current State (2026-02-04)

### Performance
| Metric | Value |
|--------|-------|
| Resolution | 1280x720 (MJPG) |
| Detection Rate | 95-100% |
| Processing Time | ~270ms/frame |
| FPS | ~3.7 |
| Marker Size | 18cm |

### Timing Breakdown
```
grab:0ms  gray:3ms  CLAHE:20ms  bgr:2ms  detect:250ms  total:275ms
```

### What's Working
- [x] Single ArUco marker detection (DICT_4X4_50)
- [x] CLAHE preprocessing for robust detection
- [x] Position estimation and world-frame conversion
- [x] HTTP streaming with debug viewer
- [x] Timing instrumentation
- [x] OpenCV crash handling (CORNER_REFINE_CONTOUR bug)

### Next Priority
- [ ] FPS optimization (target: 10+ FPS)
- [ ] Real flight controller testing
- [ ] Tethered hover test

## Key Files Reference

| File | Purpose |
|------|---------|
| `src/main.py` | Main entry point, stream/test/run modes |
| `src/aruco_detector.py` | Detection + CLAHE + timing |
| `src/position_estimator.py` | World position calculation |
| `src/mavlink_interface.py` | ArduCopter communication |
| `tools/debug_viewer.py` | Manual frame capture + timing |
| `config/system_config.yaml` | Main configuration |
| `config/marker_map.yaml` | Marker world positions |
| `config/camera_params.yaml` | Camera calibration (720p) |

## How I Work With This User

### Communication Style
- Direct, technical, no fluff
- Appreciates proactive problem-solving
- Polish native speaker (paths may have Polish characters)

### Workflow Preferences
1. **Testing**: Always test on RPi via `./sync_to_rpi.sh`
2. **Documentation**: Keep docs updated
3. **Git**: Commit frequently with descriptive messages
4. **Session End**: Log progress to vibecoding logger

### Important Commands
```bash
# Sync to RPi
./sync_to_rpi.sh

# SSH to RPi
ssh aruconav@aruconav.local

# Run on RPi
cd /home/aruconav/aruco_drone_nav
python3 -m src.main --mode stream  # HTTP server
python3 -m src.main --mode test    # Console output

# Debug viewer (local)
python3 tools/debug_viewer.py
```

## Technical Decisions

1. **Marker Type**: Single ArUco (DICT_4X4_50) - simpler than Diamond
2. **Marker Size**: 18cm (fits A4 with margins)
3. **Preprocessing**: CLAHE (clipLimit=2.5, tileGridSize=8x8)
4. **Camera Format**: MJPG (30fps at 720p, YUYV limited to 10fps)
5. **Corner Refinement**: CORNER_REFINE_CONTOUR (with crash handling)
6. **Coordinate System**: ENU (East-North-Up) for world frame

## Iteration History

### v3 (2026-02-04) - Single ArUco + CLAHE + Timing
- Switched from Diamond to single ArUco markers
- Added CLAHE preprocessing
- Added timing instrumentation
- Created debug_viewer.py
- 95-100% detection rate achieved

### v2 (2026-02-03) - Diamond Markers
- Implemented ChArUco Diamond detection
- Remote calibration support
- Diamond marker generator

### v1 (2026-01-23) - Initial RPi Deployment
- Basic ArUco detection
- MJPEG streaming
- Debug GUI

## Notes for Next Session

When resuming:
1. Read this file first
2. Check TODO.md for priorities
3. Check if RPi stream is running: `ssh aruconav@aruconav.local "pgrep -f src.main"`
4. Main focus: FPS optimization (3.7 -> 10+ FPS)

---

*Last updated: 2026-02-04 by Claude*
