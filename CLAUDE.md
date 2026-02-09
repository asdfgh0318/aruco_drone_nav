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

**Current Status (2026-02-09):**
- Minimal codebase (~490 lines in 2 source files)
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
| `src/vision_gps.py` | Camera, detection, position estimation, HTTP server, main loop |
| `src/mavlink_bridge.py` | MAVLink connection + send vision position to FC |
| `src/__main__.py` | Entry point (`python3 -m src`) |
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
python3 -m src --mode stream   # HTTP server (port 8001)
python3 -m src --mode test     # Console output
```

### Debug Viewer (local machine)
```bash
python3 tools/debug_viewer.py       # Press SPACE to capture
```

### Check RPi Stream Status
```bash
ssh aruconav@aruconav.local "pgrep -f 'src' && tail -5 /tmp/stream.log"
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

### 2026-02-09: Codebase Minimization
- Rewrote src/ from 3,329 lines (5 files) to ~490 lines (2 files)
- Deleted legacy Diamond/ChArUco/SimplePositionEstimator code
- Deleted unused MAVLink commands (arm, disarm, takeoff, land, velocity)
- Unified 3 run methods into single loop

### 2026-02-04: Single ArUco + CLAHE + Timing
- Switched from Diamond to single ArUco markers
- Added CLAHE preprocessing (robust detection)
- Added timing instrumentation
- Created debug_viewer.py with timing display

### 2026-02-03: Diamond Markers + Remote Calibration
- Implemented ChArUco Diamond detection
- Remote camera calibration

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
