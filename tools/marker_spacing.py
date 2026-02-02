#!/usr/bin/env python3
"""
Marker Spacing Calculator

Calculates optimal ceiling marker placement based on camera parameters
and flight altitude. Outputs recommended spacing, grid layout, and
marker count for a given room size.

Usage:
    python marker_spacing.py
    python marker_spacing.py --altitude 1.5 --room 5x4
    python marker_spacing.py --calibration config/camera_params.yaml --altitude 2.0
"""

import argparse
import math
import sys
import yaml
from pathlib import Path


def load_camera_fov(calibration_path: str) -> tuple:
    """
    Calculate camera FOV from calibration file.

    Returns:
        (fov_h_deg, fov_v_deg) horizontal and vertical FOV in degrees
    """
    with open(calibration_path, 'r') as f:
        data = yaml.safe_load(f)

    matrix = data['camera_matrix']['data']
    fx = matrix[0]
    fy = matrix[4]
    width = data['calibration_info']['image_width']
    height = data['calibration_info']['image_height']

    fov_h = 2 * math.degrees(math.atan(width / (2 * fx)))
    fov_v = 2 * math.degrees(math.atan(height / (2 * fy)))

    return fov_h, fov_v


def calculate_visibility(fov_h_deg: float, fov_v_deg: float,
                         ceiling_height: float, flight_altitude: float) -> tuple:
    """
    Calculate visible area on ceiling from a given flight altitude.

    Args:
        fov_h_deg: Horizontal FOV in degrees
        fov_v_deg: Vertical FOV in degrees
        ceiling_height: Ceiling height in meters
        flight_altitude: Drone flight altitude in meters

    Returns:
        (visible_width, visible_height) in meters
    """
    distance_to_ceiling = ceiling_height - flight_altitude
    if distance_to_ceiling <= 0:
        return 0.0, 0.0

    visible_w = 2 * distance_to_ceiling * math.tan(math.radians(fov_h_deg / 2))
    visible_h = 2 * distance_to_ceiling * math.tan(math.radians(fov_v_deg / 2))

    return visible_w, visible_h


def calculate_spacing(visible_width: float, visible_height: float,
                      marker_size: float, overlap: float) -> tuple:
    """
    Calculate marker spacing for continuous coverage.

    Args:
        visible_width: Visible ceiling width in meters
        visible_height: Visible ceiling height in meters
        marker_size: Physical marker size in meters
        overlap: Desired overlap fraction (0.0-1.0). 0.3 means 30% overlap.

    Returns:
        (spacing_x, spacing_y) in meters
    """
    effective_w = visible_width - marker_size
    effective_h = visible_height - marker_size

    spacing_x = effective_w * (1 - overlap)
    spacing_y = effective_h * (1 - overlap)

    return max(spacing_x, marker_size), max(spacing_y, marker_size)


def calculate_grid(room_width: float, room_depth: float,
                   spacing_x: float, spacing_y: float,
                   margin: float = 0.5) -> tuple:
    """
    Calculate marker grid for a room.

    Args:
        room_width: Room width in meters (X direction)
        room_depth: Room depth in meters (Y direction)
        spacing_x: Marker spacing in X
        spacing_y: Marker spacing in Y
        margin: Distance from wall to first marker

    Returns:
        (cols, rows, positions) where positions is list of (x, y)
    """
    usable_w = room_width - 2 * margin
    usable_d = room_depth - 2 * margin

    cols = max(1, int(usable_w / spacing_x) + 1)
    rows = max(1, int(usable_d / spacing_y) + 1)

    # Recalculate actual spacing to center the grid
    actual_sx = usable_w / max(1, cols - 1) if cols > 1 else 0
    actual_sy = usable_d / max(1, rows - 1) if rows > 1 else 0

    positions = []
    for r in range(rows):
        for c in range(cols):
            x = margin + c * actual_sx if cols > 1 else room_width / 2
            y = margin + r * actual_sy if rows > 1 else room_depth / 2
            positions.append((x, y))

    return cols, rows, positions


def draw_layout(room_width: float, room_depth: float,
                positions: list, marker_size: float,
                visible_width: float, visible_height: float):
    """Print ASCII layout of marker placement."""
    scale = 40 / max(room_width, room_depth)
    w = int(room_width * scale)
    d = int(room_depth * scale)

    grid = [['.' for _ in range(w + 1)] for _ in range(d + 1)]

    # Draw room border
    for x in range(w + 1):
        grid[0][x] = '-'
        grid[d][x] = '-'
    for y in range(d + 1):
        grid[y][0] = '|'
        grid[y][w] = '|'
    grid[0][0] = grid[0][w] = grid[d][0] = grid[d][w] = '+'

    # Draw markers
    for i, (mx, my) in enumerate(positions):
        gx = int(mx * scale)
        gy = int(my * scale)
        if 0 <= gx <= w and 0 <= gy <= d:
            grid[gy][gx] = '#'

    # Print
    print(f"\n  Ceiling Layout ({room_width}m x {room_depth}m)")
    print(f"  # = marker position\n")
    for row in grid:
        print("  " + "".join(row))


def main():
    parser = argparse.ArgumentParser(
        description="Calculate optimal ArUco marker spacing for ceiling coverage"
    )
    parser.add_argument(
        '--calibration', '-c',
        type=str,
        default=None,
        help='Path to camera_params.yaml (auto-calculates FOV)'
    )
    parser.add_argument(
        '--fov',
        type=float,
        nargs=2,
        metavar=('H', 'V'),
        default=None,
        help='Camera FOV in degrees (horizontal vertical). Overrides --calibration.'
    )
    parser.add_argument(
        '--altitude', '-a',
        type=float,
        default=1.5,
        help='Flight altitude in meters (default: 1.5)'
    )
    parser.add_argument(
        '--ceiling',
        type=float,
        default=3.0,
        help='Ceiling height in meters (default: 3.0)'
    )
    parser.add_argument(
        '--marker-size', '-s',
        type=float,
        default=0.20,
        help='Marker size in meters (default: 0.20)'
    )
    parser.add_argument(
        '--overlap', '-o',
        type=float,
        default=0.3,
        help='Desired overlap fraction 0.0-1.0 (default: 0.3 = 30%%)'
    )
    parser.add_argument(
        '--room', '-r',
        type=str,
        default=None,
        help='Room dimensions WxD in meters (e.g. 5x4). Generates grid layout.'
    )
    parser.add_argument(
        '--margin',
        type=float,
        default=0.5,
        help='Distance from wall to first marker in meters (default: 0.5)'
    )

    args = parser.parse_args()

    # Determine FOV
    if args.fov:
        fov_h, fov_v = args.fov
    elif args.calibration:
        cal_path = Path(args.calibration)
        if not cal_path.is_absolute():
            cal_path = Path(__file__).parent.parent / cal_path
        if not cal_path.exists():
            print(f"Error: calibration file not found: {cal_path}")
            sys.exit(1)
        fov_h, fov_v = load_camera_fov(str(cal_path))
    else:
        # Try default calibration file
        default_cal = Path(__file__).parent.parent / "config" / "camera_params.yaml"
        if default_cal.exists():
            fov_h, fov_v = load_camera_fov(str(default_cal))
        else:
            # Fallback: typical USB camera FOV
            fov_h, fov_v = 75.0, 60.0
            print(f"No calibration found, using default FOV: {fov_h}° x {fov_v}°")

    # Calculate
    distance = args.ceiling - args.altitude
    vis_w, vis_h = calculate_visibility(fov_h, fov_v, args.ceiling, args.altitude)
    sp_x, sp_y = calculate_spacing(vis_w, vis_h, args.marker_size, args.overlap)

    # Output
    print("=" * 50)
    print("  MARKER SPACING CALCULATOR")
    print("=" * 50)
    print(f"\n  Camera FOV:        {fov_h:.1f}° x {fov_v:.1f}°")
    print(f"  Flight altitude:   {args.altitude:.1f}m")
    print(f"  Ceiling height:    {args.ceiling:.1f}m")
    print(f"  Distance to ceil:  {distance:.1f}m")
    print(f"  Marker size:       {args.marker_size*100:.0f}cm")
    print(f"  Overlap:           {args.overlap*100:.0f}%")
    print(f"\n  Visible area:      {vis_w:.2f}m x {vis_h:.2f}m")
    print(f"  Recommended spacing: {sp_x:.2f}m x {sp_y:.2f}m")

    # Room layout
    if args.room:
        try:
            room_w, room_d = [float(x) for x in args.room.split('x')]
        except ValueError:
            print(f"\nError: invalid room format '{args.room}', use WxD (e.g. 5x4)")
            sys.exit(1)

        cols, rows, positions = calculate_grid(room_w, room_d, sp_x, sp_y, args.margin)
        total = cols * rows

        print(f"\n  Room size:         {room_w}m x {room_d}m")
        print(f"  Grid:              {cols} x {rows} = {total} markers")
        print(f"  Marker IDs needed: 0 - {total - 1}")

        draw_layout(room_w, room_d, positions, args.marker_size, vis_w, vis_h)

        # Print marker_map.yaml snippet
        print(f"\n  marker_map.yaml snippet:")
        print(f"  ---")
        print(f"  ceiling_height: {args.ceiling}")
        print(f"  markers:")
        for i, (x, y) in enumerate(positions):
            print(f"    - id: {i}")
            print(f"      position: [{x:.2f}, {y:.2f}, {args.ceiling}]")
            print(f"      orientation: 0")

    print()


if __name__ == "__main__":
    main()
