# Claude Code Instructions

## IMPORTANT RULES

### Documentation with Every Commit
**ALWAYS update documentation when making code changes.** Never commit code without updating:
- `README.md` - If user-facing features change
- `CURRENT_PLAN.md` - Status, metrics, what's done/pending
- `TODO.md` - Completed items, new tasks
- `docs/TECHNICAL.md` - If implementation details change

This is mandatory. Code changes without doc updates are incomplete.

---

## Project Overview

**ArUco Vision GPS** - Vision-based indoor positioning for drones using ceiling-mounted ArUco markers on Raspberry Pi Zero 2W.

**Current Status (2026-02-04):**
- Single ArUco markers (DICT_4X4_50, 18cm)
- CLAHE preprocessing for robust detection
- 95-100% detection rate at ~3.7 FPS
- HTTP streaming with timing data

## Architecture

```
Ceiling Markers ──► USB Camera ──► RPi Zero 2W ──► Flight Controller
                    (MJPG 720p)    - CLAHE preprocessing
                                   - ArUco detection (~250ms)
                                   - Position estimation
                                   - VISION_POSITION_ESTIMATE
```

## Key Files

| File | Purpose |
|------|---------|
| `src/main.py` | Vision GPS main loop (stream/test/run modes) |
| `src/aruco_detector.py` | Detection + CLAHE + timing instrumentation |
| `src/position_estimator.py` | World-frame position calculation |
| `src/mavlink_interface.py` | MAVLink (VISION_POSITION_ESTIMATE) |
| `tools/debug_viewer.py` | Manual frame capture with timing |
| `config/system_config.yaml` | Camera, ArUco, control settings |
| `config/marker_map.yaml` | Marker world positions |

## Quick Commands

### Sync and Run on RPi
```bash
./sync_to_rpi.sh                    # Sync code
ssh aruconav@aruconav.local         # Connect

# On RPi:
cd /home/aruconav/aruco_drone_nav
python3 -m src.main --mode stream   # HTTP server (port 8001)
python3 -m src.main --mode test     # Console output
```

### Debug Viewer (local machine)
```bash
python3 tools/debug_viewer.py       # Press SPACE to capture
```

### Check RPi Stream Status
```bash
ssh aruconav@aruconav.local "pgrep -f 'src.main' && tail -5 /tmp/stream.log"
```

## Hardware

- **RPi**: `aruconav.local` (user: `aruconav`)
- **Camera**: USB, MJPG 1280x720 @ 30fps
- **Markers**: 18cm ArUco (DICT_4X4_50) on A4 paper
- **Ceiling Height**: ~1.7m (working distance)

## Current Performance

| Metric | Value |
|--------|-------|
| Resolution | 1280x720 |
| Detection | 95-100% |
| Processing | ~270ms/frame |
| FPS | ~3.7 |

### Timing Breakdown
```
grab:0ms  gray:3ms  CLAHE:20ms  bgr:2ms  detect:250ms
```

## Session History

### 2026-02-04: Single ArUco + CLAHE + Timing
- Switched from Diamond to single ArUco markers
- Added CLAHE preprocessing (robust detection)
- Added timing instrumentation
- Created debug_viewer.py with timing display
- Handled OpenCV CORNER_REFINE_CONTOUR crash
- Updated all documentation

### 2026-02-03: Diamond Markers + Remote Calibration
- Implemented ChArUco Diamond detection
- Remote camera calibration
- Diamond marker generator

### 2026-01-23: RPi Deployment
- Initial RPi Zero 2W setup
- MJPEG streaming
- Debug GUI

## Next Priority

1. **FPS Optimization** - Target 10+ FPS
   - Reduce resolution (640x480)
   - Conditional CLAHE
   - Detection parameter tuning

2. **Real Hardware Testing**
   - Connect to flight controller
   - Tethered hover test
