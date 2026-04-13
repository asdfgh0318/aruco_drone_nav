# Repo Audit & Cleanup — 2026-04-13

## Source Code Changes

### `src/vision_gps.py`
- Removed unused `t_start = time.time()` — loop rate sleep was removed earlier, variable was orphaned
- Removed `cam_matrix` param from `estimate_position()` — passed in but never used, leftover from old reprojection design
- Removed `confidence=state.confidence` from `send_gps_input()` call — param was accepted but never used in mavlink_bridge

### `src/mavlink_bridge.py`
- Default port `/dev/serial0` → `/dev/ttyS0` — NixOS doesn't have `/dev/serial0` symlink
- Default baud `921600` → `115200` — matches actual FC config
- Removed `confidence` param from `send_gps_input()` — never used in the function body

### `config/system_config.yaml`
- Removed `control.loop_rate_hz` section — code no longer reads it (runs flat out)
- Removed `gps_emulation: true/false` toggle — VISO mode removed, GPS_INPUT is unconditional
- Removed `logging.level/file` section — never read by code (logging from CLI flag only)
- Changed `marker_size_m` from 0.18 to 0.20 — physical markers are 20cm now
- Removed Raspbian comment from serial port — system is NixOS only

### `config/marker_map.yaml`
- Added `# id: 19 — intentionally skipped` comment — gap in IDs was unexplained

### `.gitignore`
- Added `*.glb`, `*.zip`, `*.BIN`, `*.csv` — large data files sitting in repo dir
- Added `sdcard_backup/`, `buildroot/`, `Copter-*/` — multi-GB dirs not in git
- Added `markers/**/*.png`, `markers/**/*.pdf` — recursive marker subdirs
- Added `docs/mavproxy.html` — saved reference doc, not project content

### `requirements.txt`
- Added `pyserial>=3.5` — used by mavlink_bridge but was missing

## Deleted Files

### Dead tools (import from deleted src modules — crash on import)
- `tools/bench_test.py`
- `tools/debug_ceiling.py`
- `tools/debug_gui.py`
- `tools/configurator_gui.py`
- `tools/generate_charuco.py`
- `tools/generate_diamonds.py`
- `tools/test_aruco_detection.py`
- `tools/test_mavlink.py`

### Stale tools (superseded or broken with current code)
- `tools/camera_server.py` — superseded by vision_gps.py HTTP server
- `tools/calibrate_remote.py` — old user `aruconav`, broken OpenCV 4.9 API
- `tools/position_viewer.py` — reads `diamonds[]` key, replaced by live_map.html
- `tools/marker_spacing.py` — reads old camera YAML format

### Stale docs (describe VISO mode, wrong params/formulas)
- `docs/FC_CONFIG.md`
- `docs/WIRING.md`
- `docs/TESTING.md`
- `docs/TECHNICAL.md`

## Moved Files
- `1_czlowiek.json` → `missions/`
- `1_czlowiek.waypoints` → `missions/`
- `P01_System2_Planned path_20260219_085424.json` → `missions/`
- `P01_System2_Planned path_20260219_085424.waypoints` → `missions/`

## Added Files
- `docs/images/live_map_full.png` — live map with 30+ markers, drone + FC arrows
- `docs/images/glb_viewer_3d.png` — 3D building model with marker pins and flight path
- `docs/images/dev_workspace.png` — full development workspace overview
- `docs/images/vision_vs_fc.png` — vision vs FC position comparison
- `docs/images/corridor_tracking.png` — drone tracking through marker corridor
- `docs/images/camera_calibration.png` — camera calibration tool interface

## Remaining work
- CLAUDE.md rewrite (root of truth for the project)
- site.html update (single HTML docs site with screenshots)
- README.md update
- Source: YAW_DEBUG log line still in production (vision_gps.py ~line 346)
- Source: `Detection.distance` and `.timestamp` fields set but never read
- Source: `attitude_rate` in mavlink_bridge stored but not consumed (kept for future Option A flow)
- Source: `is_connected` property never called externally
- Source: `estimate_single._debug_body` function attribute — unconventional but functional
