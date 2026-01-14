# Claude Session Resume File

> **Purpose**: This file helps Claude maintain context across sessions. Read this first when resuming work on this project.

## Project Identity

- **Name**: ArUco Drone Navigation System
- **Location**: `/home/adam-koszalka/ŻYCIE/PRACA/aruco_drone_nav`
- **GitHub**: `github.com/asdfgh0318/aruco_drone_nav` (private)
- **Primary User**: Adam Koszałka
- **Institution**: Warsaw University of Technology

## Project Context

### What This Project Does
Indoor drone navigation system using ceiling-mounted ArUco markers. A VR team creates flight paths in VR, exports waypoints as JSON. This system translates those waypoints to MAVLink commands for an ArduCopter drone. The drone uses a camera facing UP to detect markers on the ceiling for position feedback. Flight paths are recorded and sent back to VR for iteration.

### Hardware Setup
- **Drone**: ArduCopter-based with Pixhawk or similar
- **Companion Computer**: Raspberry Pi Zero W/2W
- **Camera**: USB camera facing UP toward ceiling
- **Markers**: 20cm ArUco markers glued to ceiling
- **Communication**: UART serial (RPi → FC)

### Architecture
```
VR Software ──JSON──► RPi Zero ──MAVLink──► ArduCopter FC
                ▲         │
                │         ▼
           Recording   USB Camera ───► Ceiling Markers
```

## Current State (2024-01-14)

### Completed
- [x] Phase 1: Single marker hover control
- [x] Phase 2: Multi-marker waypoint navigation
- [x] Camera calibration module
- [x] ArUco detection with pose estimation
- [x] MAVLink interface (velocity commands)
- [x] Mission executor (JSON → waypoints)
- [x] Flight recorder (VR-compatible output)
- [x] Position predictor (dead reckoning on marker loss)
- [x] All testing tools (calibration, detection, MAVLink)
- [x] Marker generator (PDFs for printing)
- [x] Documentation (README, TECHNICAL)

### Not Yet Done (See TODO.md)
- [ ] Wiring diagrams (waiting for final parts)
- [ ] Code adjustment to final hardware
- [ ] Marker spacing calculator (FOV-based)
- [ ] LiDAR integration
- [ ] Real flight testing

## How I Work With This User

### Communication Style
- Direct, technical, no fluff
- Uses "ultrathink" for complex planning
- Appreciates proactive problem-solving
- Polish native speaker (file paths may have Polish characters)

### Workflow Preferences
1. **Planning**: Always create TODO lists for multi-step tasks
2. **Documentation**: Maintain docs thoroughly, save plans
3. **Logging**: Use vibecoding logger (`/home/adam-koszalka/ŻYCIE/VIBECODING/vibecoding-logger/`)
4. **Git**: Commit frequently, push to GitHub
5. **Session End**: Log progress with "we're done" pattern

### Important Paths
- **Vibecoding Logger**: `/home/adam-koszalka/ŻYCIE/VIBECODING/vibecoding-logger/`
- **Log Command**: `/home/adam-koszalka/ŻYCIE/VIBECODING/vibecoding-logger/log.sh`
- **Work Logs**: `/home/adam-koszalka/ŻYCIE/VIBECODING/vibecoding-logger/WORK_LOGS`
- **This Project**: `/home/adam-koszalka/ŻYCIE/PRACA/aruco_drone_nav`

### End of Session Protocol
1. Ensure all changes committed to git
2. Push to GitHub
3. Log progress: `/home/adam-koszalka/ŻYCIE/VIBECODING/vibecoding-logger/log.sh "description"`
4. Update this file if significant changes made

## Key Files Reference

| File | Purpose |
|------|---------|
| `src/main.py` | Main entry point, control loops |
| `src/aruco_detector.py` | Marker detection + pose |
| `src/mavlink_interface.py` | ArduCopter communication |
| `src/mission_executor.py` | JSON mission handling |
| `src/position_predictor.py` | Dead reckoning |
| `config/system_config.yaml` | Main configuration |
| `config/marker_map.yaml` | Marker world positions |
| `TODO.md` | Outstanding tasks |
| `docs/PLAN_v1.md` | Original implementation plan |

## Technical Decisions Made

1. **ArUco Dictionary**: DICT_6X6_250 (good balance of size/detection)
2. **Marker Size**: 20cm (visible at 3-5m)
3. **Control Method**: Velocity commands via MAVLink (not position)
4. **Frame Rate**: 20 Hz control loop target
5. **Coordinate System**: ENU (East-North-Up) for world frame
6. **Failsafe**: Land on marker loss >2 seconds

## Iteration History

### v1 (2024-01-14) - Initial Implementation
- Full Phase 1 and Phase 2 implementation
- All core modules created
- Testing tools complete
- Documentation created

## Notes for Next Session

When resuming:
1. Read this file first
2. Check TODO.md for outstanding tasks
3. Check git status for any uncommitted work
4. Ask user what they want to focus on

---

*Last updated: 2024-01-14 by Claude*
