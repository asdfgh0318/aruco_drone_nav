#!/usr/bin/env python3
"""Convert Unity VR drone path planner JSON to ArduPilot QGC WPL 110 waypoints file."""

import argparse
import json
import math
import sys
from pathlib import Path


# MAV_CMD constants
NAV_WAYPOINT = 16
NAV_TAKEOFF = 22
NAV_LAND = 21
NAV_LOITER_TIME = 19

# Frame constants
FRAME_GLOBAL = 0
FRAME_GLOBAL_REL_ALT = 3

# Waypoint type mapping (VR planner)
TYPE_MAP = {
    0: ("FlyThrough",   NAV_WAYPOINT),
    1: ("StopRotate",   NAV_WAYPOINT),
    2: ("Record360",    NAV_LOITER_TIME),
}


def ned_to_latlon(north, east, origin_lat, origin_lon):
    lat = origin_lat + north / 111111.0
    lon = origin_lon + east / (111111.0 * math.cos(math.radians(origin_lat)))
    return lat, lon


def get_traversal_order(data):
    """Extract waypoint visit order from detailedSegments.

    Segments form a chain: fromId=-1 (start) → wp → wp → ... → toId=-2 (end).
    Returns list of waypoint IDs in traversal order.
    """
    segments = data.get("detailedSegments", [])
    if not segments:
        return None

    # Build adjacency: fromId → toId
    next_wp = {}
    for seg in segments:
        next_wp[seg["fromId"]] = seg["toId"]

    # Walk the chain from start (-1)
    order = []
    current = next_wp.get(-1)
    visited = set()
    while current is not None and current != -2 and current not in visited:
        order.append(current)
        visited.add(current)
        current = next_wp.get(current)

    return order


def parse_vr_planner(data, args):
    """Parse detailedWaypoints format (Unity VR planner).

    Uses detailedSegments to determine path traversal order (not creation order).
    """
    # Build lookup by waypoint ID
    wp_by_id = {}
    for wp in data["detailedWaypoints"]:
        if wp.get("wasDeleted", False):
            continue
        wp_by_id[wp["id"]] = wp

    # Get traversal order from segments, fall back to creation order
    traversal = get_traversal_order(data)
    if traversal:
        # Filter to only IDs that exist (non-deleted)
        ordered_ids = [wid for wid in traversal if wid in wp_by_id]
        if len(ordered_ids) != len(wp_by_id):
            missing = set(wp_by_id.keys()) - set(ordered_ids)
            print(f"  Note: {len(missing)} waypoint(s) not in segment chain: {missing}")
    else:
        ordered_ids = list(wp_by_id.keys())
        print("  Note: No detailedSegments — using creation order")

    # Get start position altitude for zero-referencing
    segments = data.get("detailedSegments", [])
    if segments:
        start_alt = segments[0]["fromPos"]["y"]
    else:
        first_wp = wp_by_id[ordered_ids[0]] if ordered_ids else None
        start_alt = first_wp["position"]["y"] if first_wp else 0.0

    # Extract path start/end positions from segments for takeoff/landing
    start_pos = None
    end_pos = None
    if segments:
        start_pos = segments[0]["fromPos"]
        # Walk to last segment to get end position
        for seg in segments:
            if seg["toId"] == -2:
                end_pos = seg["toPos"]
                break
        if end_pos is None:
            end_pos = segments[-1]["toPos"]

    waypoints = []
    for wid in ordered_ids:
        wp = wp_by_id[wid]
        pos = wp["position"]
        north = pos["z"]   # Unity Z = North (yaw reference axis)
        east  = pos["x"]   # Unity X = East
        alt   = pos["y"] - start_alt  # Zero-referenced to path start position

        if args.fixed_alt is not None:
            alt = args.fixed_alt
        alt += args.alt_offset

        lat, lon = ned_to_latlon(north, east, args.origin_lat, args.origin_lon)
        yaw  = wp.get("yaw", 0.0)
        wtype = wp.get("type", 0)

        name, cmd = TYPE_MAP.get(wtype, ("FlyThrough", NAV_WAYPOINT))

        if wtype == 1:      # StopRotate
            hold = 2.0
        elif wtype == 2:    # Record360
            hold = args.record360_time
        else:               # FlyThrough
            hold = args.hold_default

        waypoints.append({
            "id": wid,
            "lat": lat, "lon": lon, "alt": alt,
            "cmd": cmd, "hold": hold, "yaw": yaw,
            "name": name,
        })

    return {"waypoints": waypoints, "start_pos": start_pos, "end_pos": end_pos,
            "start_alt": start_alt}


def parse_missions_json(data, args):
    """Parse waypoints + settings format (missions JSON)."""
    waypoints = []
    settings = data.get("settings", {})
    for wp in data["waypoints"]:
        north = wp["x"]    # already NED-aligned
        east  = wp["y"]
        alt   = wp["z"]    # positive up

        if args.fixed_alt is not None:
            alt = args.fixed_alt
        alt += args.alt_offset

        lat, lon = ned_to_latlon(north, east, args.origin_lat, args.origin_lon)
        yaw      = wp.get("yaw", 0.0)
        hold     = wp.get("hold_time", args.hold_default)

        waypoints.append({
            "lat": lat, "lon": lon, "alt": alt,
            "cmd": NAV_WAYPOINT, "hold": hold, "yaw": yaw,
            "name": "Waypoint",
        })
    return {"waypoints": waypoints, "start_pos": None, "end_pos": None,
            "start_alt": 0.0}


def fmt_row(index, current, frame, cmd, p1, p2, p3, p4, lat, lon, alt, autocont=1):
    return (
        f"{index}\t{current}\t{frame}\t{cmd}\t"
        f"{p1:.6f}\t{p2:.6f}\t{p3:.6f}\t{p4:.6f}\t"
        f"{lat:.7f}\t{lon:.7f}\t{alt:.3f}\t{autocont}"
    )


def build_mission(waypoints, args, start_pos=None, end_pos=None, start_alt=None):
    lines = ["QGC WPL 110"]

    # Metadata comment — viewer and tools can parse this to reverse altitude conversion
    if start_alt is not None:
        lines.append(f"# ARUCO_NAV start_alt={start_alt:.4f}")

    # Home (index 0)
    lines.append(fmt_row(
        0, 1, FRAME_GLOBAL, NAV_WAYPOINT,
        0, 0, 0, 0,
        args.origin_lat, args.origin_lon, args.origin_alt,
    ))

    idx = 1

    # Takeoff at path start position (where drone is physically placed)
    if not args.no_takeoff:
        takeoff_alt = args.takeoff_alt if args.takeoff_alt is not None else (
            waypoints[0]["alt"] if waypoints else 1.5
        )
        if start_pos is not None:
            to_lat, to_lon = ned_to_latlon(
                start_pos["z"], start_pos["x"],  # Unity Z=North, X=East
                args.origin_lat, args.origin_lon,
            )
        else:
            to_lat, to_lon = args.origin_lat, args.origin_lon
        lines.append(fmt_row(
            idx, 0, FRAME_GLOBAL_REL_ALT, NAV_TAKEOFF,
            0, 0, 0, 0,
            to_lat, to_lon, takeoff_alt,
        ))
        idx += 1

    # Waypoints
    for wp in waypoints:
        lines.append(fmt_row(
            idx, 0, FRAME_GLOBAL_REL_ALT, wp["cmd"],
            wp["hold"], 0, 0, wp["yaw"],
            wp["lat"], wp["lon"], wp["alt"],
        ))
        idx += 1

    # Land at path end position
    if not args.no_land:
        if end_pos is not None:
            land_lat, land_lon = ned_to_latlon(
                end_pos["z"], end_pos["x"],  # Unity Z=North, X=East
                args.origin_lat, args.origin_lon,
            )
        else:
            land_lat, land_lon = args.origin_lat, args.origin_lon
        lines.append(fmt_row(
            idx, 0, FRAME_GLOBAL_REL_ALT, NAV_LAND,
            0, 0, 0, 0,
            land_lat, land_lon, 0,
        ))

    return lines


def main():
    parser = argparse.ArgumentParser(
        description="Convert Unity VR drone path planner JSON to ArduPilot QGC WPL 110 waypoints."
    )
    parser.add_argument("input_json", help="Path to VR planner JSON or missions/*.json file")
    parser.add_argument("-o", "--output", metavar="PATH",
                        help="Output .waypoints file (default: <input_stem>.waypoints)")
    parser.add_argument("--origin-lat",      type=float, default=52.2297,  metavar="FLOAT",
                        help="EKF origin latitude (default: 52.2297)")
    parser.add_argument("--origin-lon",      type=float, default=21.0122,  metavar="FLOAT",
                        help="EKF origin longitude (default: 21.0122)")
    parser.add_argument("--origin-alt",      type=float, default=100.0,    metavar="FLOAT",
                        help="EKF origin altitude MSL (default: 100.0)")
    parser.add_argument("--fixed-alt",       type=float, default=None,     metavar="FLOAT",
                        help="Override all waypoint altitudes to this value (meters relative)")
    parser.add_argument("--alt-offset",      type=float, default=0.0,      metavar="FLOAT",
                        help="Add to all computed altitudes (default: 0.0)")
    parser.add_argument("--takeoff-alt",     type=float, default=None,     metavar="FLOAT",
                        help="Takeoff altitude (default: first waypoint altitude)")
    parser.add_argument("--hold-default",    type=float, default=0.0,      metavar="FLOAT",
                        help="Default hold time for FlyThrough waypoints (default: 0.0)")
    parser.add_argument("--record360-time",  type=float, default=30.0,     metavar="FLOAT",
                        help="Loiter time for Record360 waypoints (default: 30.0)")
    parser.add_argument("--no-takeoff",      action="store_true",
                        help="Skip TAKEOFF command")
    parser.add_argument("--no-land",         action="store_true",
                        help="Skip LAND command")
    parser.add_argument("-v", "--verbose",   action="store_true",
                        help="Print each waypoint")
    args = parser.parse_args()

    input_path = Path(args.input_json)
    if not input_path.exists():
        print(f"ERROR: File not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    with open(input_path) as f:
        data = json.load(f)

    # Auto-detect format
    if "detailedWaypoints" in data:
        fmt = "VR planner"
        result = parse_vr_planner(data, args)
    elif "waypoints" in data and "settings" in data:
        fmt = "missions JSON"
        result = parse_missions_json(data, args)
    else:
        print("ERROR: Unrecognised JSON format (expected 'detailedWaypoints' or 'waypoints'+'settings')",
              file=sys.stderr)
        sys.exit(1)

    waypoints = result["waypoints"]
    start_pos = result["start_pos"]
    end_pos = result["end_pos"]
    start_alt = result["start_alt"]

    print(f"Format detected: {fmt}")
    print(f"Waypoints loaded: {len(waypoints)}")
    if start_pos:
        print(f"Path start (Unity): X={start_pos['x']:.3f} Y={start_pos['y']:.3f} Z={start_pos['z']:.3f}")
    if end_pos:
        print(f"Path end   (Unity): X={end_pos['x']:.3f} Y={end_pos['y']:.3f} Z={end_pos['z']:.3f}")

    if not waypoints:
        print("WARNING: No waypoints after filtering.", file=sys.stderr)

    # Verbose listing
    if args.verbose:
        for i, wp in enumerate(waypoints):
            cmd_name = {NAV_WAYPOINT: "NAV_WAYPOINT", NAV_LOITER_TIME: "NAV_LOITER_TIME"}.get(wp["cmd"], str(wp["cmd"]))
            wp_id = f"WP{wp['id']}" if "id" in wp else f"#{i}"
            print(f"  [{i:3d}] {wp_id:5s} {wp['name']:12s} {cmd_name:18s} "
                  f"lat={wp['lat']:.6f} lon={wp['lon']:.6f} alt={wp['alt']:.2f}m "
                  f"hold={wp['hold']:.1f}s yaw={wp['yaw']:.1f}")

    # Summary stats
    if waypoints:
        alts = [wp["alt"] for wp in waypoints]
        lats = [wp["lat"] for wp in waypoints]
        lons = [wp["lon"] for wp in waypoints]
        print(f"Altitude range : {min(alts):.2f} .. {max(alts):.2f} m (relative)")
        print(f"Lat range      : {min(lats):.6f} .. {max(lats):.6f}")
        print(f"Lon range      : {min(lons):.6f} .. {max(lons):.6f}")

        negative_alts = [wp["alt"] for wp in waypoints if wp["alt"] < 0]
        if negative_alts:
            print(f"WARNING: {len(negative_alts)} waypoint(s) have negative altitude. "
                  f"Use --alt-offset or --fixed-alt to correct.", file=sys.stderr)

    # Build and write output
    lines = build_mission(waypoints, args, start_pos=start_pos, end_pos=end_pos,
                          start_alt=start_alt)

    output_path = Path(args.output) if args.output else input_path.with_suffix(".waypoints")
    output_path.write_text("\n".join(lines) + "\n")
    print(f"Written: {output_path}  ({len(lines) - 1} mission items incl. home)")


if __name__ == "__main__":
    main()
