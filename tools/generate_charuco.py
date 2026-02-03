#!/usr/bin/env python3
"""
ChArUco Board Generator

Generates printable ChArUco boards (ArUco markers + chessboard corners)
for ceiling mounting. ChArUco provides sub-pixel pose accuracy compared
to plain ArUco markers.

Usage:
    python generate_charuco.py --id 0 --output markers/
    python generate_charuco.py --id 0 --squares 5x5 --marker-size 15 --square-size 20
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import cv2
import numpy as np

from src.aruco_detector import ARUCO_DICTIONARIES


def generate_charuco_board(
    board_id: int = 0,
    squares_x: int = 5,
    squares_y: int = 5,
    square_size_cm: float = 4.0,
    marker_size_cm: float = 3.0,
    dictionary: str = "DICT_4X4_50",
    output_dir: str = "markers",
    dpi: int = 300
):
    """
    Generate a ChArUco board image.

    Each board uses a unique range of ArUco marker IDs based on board_id,
    so multiple boards can be used simultaneously without ID conflicts.

    Args:
        board_id: Board identifier (determines which marker IDs are used)
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_size_cm: Chessboard square size in cm
        marker_size_cm: ArUco marker size in cm (must be < square_size)
        dictionary: ArUco dictionary name
        output_dir: Output directory
        dpi: Image resolution
    """
    if dictionary not in ARUCO_DICTIONARIES:
        print(f"Error: unknown dictionary '{dictionary}'")
        print(f"Available: {', '.join(ARUCO_DICTIONARIES.keys())}")
        return None

    if marker_size_cm >= square_size_cm:
        print(f"Error: marker size ({marker_size_cm}cm) must be smaller than square size ({square_size_cm}cm)")
        return None

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dictionary])

    # Calculate marker ID offset so each board uses unique IDs
    markers_per_board = (squares_x * squares_y) // 2
    id_offset = board_id * markers_per_board

    # Set marker IDs based on board_id offset
    ids = np.arange(id_offset, id_offset + markers_per_board, dtype=np.int32)

    # Create board with custom IDs (5th parameter)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_size_cm / 100.0,   # Convert to meters
        marker_size_cm / 100.0,
        aruco_dict,
        ids
    )
    board.setLegacyPattern(False)

    # Generate image
    pixels_per_cm = dpi / 2.54
    img_width = int(squares_x * square_size_cm * pixels_per_cm)
    img_height = int(squares_y * square_size_cm * pixels_per_cm)

    # Add border
    border_px = int(2.0 * pixels_per_cm)  # 2cm border
    total_width = img_width + 2 * border_px
    total_height = img_height + 2 * border_px

    board_img = board.generateImage((img_width, img_height))

    # Add white border
    output_img = np.ones((total_height, total_width), dtype=np.uint8) * 255
    output_img[border_px:border_px + img_height, border_px:border_px + img_width] = board_img

    # Add label at bottom
    label = f"ChArUco Board {board_id} | {squares_x}x{squares_y} | IDs: {id_offset}-{id_offset + markers_per_board - 1} | {dictionary}"
    font_scale = total_width / 2000
    cv2.putText(
        output_img, label,
        (border_px, total_height - border_px // 3),
        cv2.FONT_HERSHEY_SIMPLEX, max(0.4, font_scale), 0, max(1, int(font_scale * 2))
    )

    # Save
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    filename = f"charuco_board_{board_id}_{squares_x}x{squares_y}_{dictionary}.png"
    filepath = out_path / filename
    cv2.imwrite(str(filepath), output_img)

    print(f"Generated: {filepath}")
    print(f"  Board ID:     {board_id}")
    print(f"  Grid:         {squares_x} x {squares_y} squares")
    print(f"  Square size:  {square_size_cm} cm")
    print(f"  Marker size:  {marker_size_cm} cm")
    print(f"  Marker IDs:   {id_offset} - {id_offset + markers_per_board - 1}")
    print(f"  Image size:   {total_width} x {total_height} px")
    print(f"  Print size:   {squares_x * square_size_cm + 4:.1f} x {squares_y * square_size_cm + 4:.1f} cm")

    return filepath, board, ids


def main():
    parser = argparse.ArgumentParser(
        description="Generate printable ChArUco boards for ceiling mounting"
    )
    parser.add_argument(
        '--id',
        type=int,
        default=0,
        help='Board ID (determines marker ID range, default: 0)'
    )
    parser.add_argument(
        '--count',
        type=int,
        default=1,
        help='Number of boards to generate (default: 1)'
    )
    parser.add_argument(
        '--squares',
        type=str,
        default='5x5',
        help='Board grid size WxH (default: 5x5)'
    )
    parser.add_argument(
        '--square-size',
        type=float,
        default=4.0,
        help='Chessboard square size in cm (default: 4.0)'
    )
    parser.add_argument(
        '--marker-size',
        type=float,
        default=3.0,
        help='ArUco marker size in cm (default: 3.0)'
    )
    parser.add_argument(
        '--dictionary', '-d',
        type=str,
        default='DICT_4X4_50',
        help='ArUco dictionary (default: DICT_4X4_50)'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='markers',
        help='Output directory (default: markers/)'
    )
    parser.add_argument(
        '--dpi',
        type=int,
        default=300,
        help='Image DPI (default: 300)'
    )

    args = parser.parse_args()

    try:
        sq_x, sq_y = [int(x) for x in args.squares.split('x')]
    except ValueError:
        print(f"Error: invalid squares format '{args.squares}', use WxH (e.g. 5x5)")
        sys.exit(1)

    # Resolve output path
    output_dir = Path(args.output)
    if not output_dir.is_absolute():
        output_dir = Path(__file__).parent.parent / output_dir

    print("=" * 50)
    print("  CHARUCO BOARD GENERATOR")
    print("=" * 50)

    for i in range(args.count):
        board_id = args.id + i
        print(f"\n--- Board {board_id} ---")
        generate_charuco_board(
            board_id=board_id,
            squares_x=sq_x,
            squares_y=sq_y,
            square_size_cm=args.square_size,
            marker_size_cm=args.marker_size,
            dictionary=args.dictionary,
            output_dir=str(output_dir),
            dpi=args.dpi
        )

    print(f"\nDone. Print at 100% scale (no fit-to-page).")


if __name__ == "__main__":
    main()
