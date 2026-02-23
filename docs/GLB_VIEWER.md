# GLB Viewer — Workflow Manual

Browser-based 3D viewer for inspecting Unity VR path planner exports against the building model. Use it to verify waypoint placement and debug the `vr_to_waypoints.py` converter.

## Quick Start

```bash
xdg-open tools/glb_viewer.html
# or just open tools/glb_viewer.html in any browser
```

No install, no build. Single HTML file using Three.js from CDN.

## Input Files

| Button | File | Source | Description |
|--------|------|--------|-------------|
| **GLB** | `*.glb` | Unity export | 3D building/environment model |
| **JSON** | `*.json` | VR path planner | Waypoints + trajectory samples (`detailedWaypoints`, `dronePlannedMovement`) |
| **WPL** | `*.waypoints` | `vr_to_waypoints.py` output | ArduPilot QGC WPL 110 mission file |

All three can also be loaded via **drag-and-drop** onto the viewport.

## Typical Workflows

### 1. Inspect a planned path on the 3D model

1. Click **GLB** → select the `.glb` model (camera auto-fits)
2. Click **JSON** → select the VR planner `.json`
3. Orbit around to verify waypoints are placed where intended
4. Click on model surfaces to read exact Unity coordinates
5. Hover waypoints for ID, type, yaw, position

### 2. Debug the waypoint converter

1. Load **GLB** + **JSON** as above (the ground truth)
2. Run the converter:
   ```bash
   python3 tools/vr_to_waypoints.py path.json -o path.waypoints --land-unity-y -1.117 -v
   ```
3. Click **WPL** → select the generated `.waypoints` file
4. Compare visually: purple ArduPilot diamonds should overlap the JSON colored spheres
5. Toggle layers on/off with checkboxes to isolate differences
6. Hover both JSON and ArduPilot markers to compare coordinates

### 3. Verify coordinate conversions

1. Load the model and click on a known point (e.g. a corner)
2. The info panel shows both **Unity (X,Y,Z)** and **NED (N,E,D)** coordinates
3. Cross-reference with your waypoint positions to validate the mapping

## UI Reference

### Nav Bar

| Element | Purpose |
|---------|---------|
| **GLB** / **JSON** / **WPL** | File load buttons |
| **Origin** fields | EKF origin lat/lon for WPL reverse conversion (default: 52.2297, 21.0122) |

> **Important:** Set the origin fields **before** loading a `.waypoints` file. These must match the `--origin-lat` and `--origin-lon` used when generating the waypoints.

### Panels

| Panel | Location | Content |
|-------|----------|---------|
| **Layers** (toggle) | Top-left | Checkboxes to show/hide each data layer |
| **Clicked Point** | Top-right | Unity + NED coordinates of last clicked surface point |
| **Waypoint Types** | Top-right | Color legend |
| **Stats** | Bottom-left | Model name, bbox, WP counts, path length, sample count |

### Tooltip (hover)

Hover any waypoint marker to see a popup with full details:

- **JSON waypoints:** WP order, ID, type name, yaw, Unity position, NED position
- **ArduPilot waypoints:** Mission index, MAV_CMD name, yaw, hold time, lat/lon, relative altitude, reverse-computed Unity position

## Visual Guide

### JSON Layer (from VR planner)

| Element | Appearance | Meaning |
|---------|------------|---------|
| Green sphere | ![#00ff88](https://via.placeholder.com/12/00ff88/00ff88) | FlyThrough waypoint |
| Yellow sphere | ![#ffdd00](https://via.placeholder.com/12/ffdd00/ffdd00) | StopRotate waypoint |
| Red sphere | ![#ff4444](https://via.placeholder.com/12/ff4444/ff4444) | Record360 waypoint |
| Blue fat line | ![#00aaff](https://via.placeholder.com/12/00aaff/00aaff) | Interpolated flight path (from `dronePlannedMovement.samples`) |
| White dashed line | — | Waypoint traversal order |
| Cone arrow | Same color as sphere | Yaw direction |
| Number label | White on black | Traversal order (1, 2, 3...) |

### ArduPilot Layer (from .waypoints)

| Element | Appearance | Meaning |
|---------|------------|---------|
| Purple diamond | ![#cc44ff](https://via.placeholder.com/12/cc44ff/cc44ff) | NAV_WAYPOINT or NAV_LOITER_TIME |
| Orange diamond | ![#ff8800](https://via.placeholder.com/12/ff8800/ff8800) | NAV_TAKEOFF |
| Red diamond | ![#ff0000](https://via.placeholder.com/12/ff0000/ff0000) | NAV_LAND |
| Purple dashed line | — | ArduPilot mission order |
| Purple cone | — | Yaw direction |
| Purple label | `A1`, `A2`... or `T/O`, `LND`, `30.0s` | Index or special marker |

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `R` | Reset camera to fit model |
| `G` | Toggle grid + axes |
| `T` | Toggle JSON flight path |
| `W` | Toggle JSON waypoints |
| `A` | Toggle ArduPilot layers (WPs + path) |

Mouse: **Left-drag** = orbit, **Right-drag** = pan, **Scroll** = zoom, **Click** = read coordinates.

## Coordinate Systems

The viewer handles two coordinate conversions:

### Unity → Scene (GLB display)

The Unity GLB exporter negates X when converting from left-handed to right-handed:

```
scene_x = -unity_x
scene_y =  unity_y
scene_z =  unity_z
```

This is applied automatically when loading JSON/WPL data.

### Unity → NED (for display)

```
North =  unity_z
East  =  unity_x
Down  = -unity_y
Alt   =  unity_y
```

### WPL reverse conversion (lat/lon → Unity)

```
north = (lat - origin_lat) * 111111
east  = (lon - origin_lon) * 111111 * cos(origin_lat)
unity_x = east
unity_z = north
unity_y = alt_relative + start_altitude
```

`start_altitude` is taken from the JSON's first segment start position. If no JSON is loaded, it defaults to 0 (horizontal positions still correct, altitude may be offset).

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| ArduPilot WPs far from model | Wrong origin lat/lon | Set origin fields to match `vr_to_waypoints.py --origin-lat/lon` values |
| ArduPilot WPs correct XZ but wrong Y | JSON not loaded first | Load the JSON **before** the WPL so `start_altitude` is known |
| Path mirrored/rotated | Coordinate flip mismatch | Should not happen — the X-negate is hardcoded. Report if seen. |
| Nothing appears after loading | File format issue | Check browser console (F12) for errors |
| Tooltips not showing | Hovering wrong element | Hover the sphere/diamond mesh directly, not the label or cone |
