#!/usr/bin/env python3
"""
ArUco Diamond Marker Generator

Generates printable ChArUco Diamond markers for ceiling mounting.
Uses OpenCV's drawCharucoDiamond for proper marker structure.

Diamond structure (ChArUco pattern with 4 ArUco markers):
        [id0]
     [id3]  [id1]
        [id2]

Usage:
    python generate_diamonds.py --ids "0_1_2_3,4_5_6_7" -o markers/
    python generate_diamonds.py --ids "0_1_2_3" --square-size 20 --marker-size 15
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import cv2
import numpy as np

from src.aruco_detector import ARUCO_DICTIONARIES


def generate_diamond_marker(
    ids: tuple,
    square_size_cm: float = 20.0,
    marker_size_cm: float = 15.0,
    dictionary: str = "DICT_4X4_50",
    output_dir: str = "markers",
    dpi: int = 300
) -> Path:
    """
    Generate a ChArUco Diamond marker image using OpenCV.

    Args:
        ids: Tuple of 4 marker IDs (id0, id1, id2, id3)
        square_size_cm: Size of chessboard squares in cm
        marker_size_cm: Size of ArUco markers in cm (must be < square_size)
        dictionary: ArUco dictionary name
        output_dir: Output directory
        dpi: Image resolution

    Returns:
        Path to generated image
    """
    if len(ids) != 4:
        raise ValueError("Diamond requires exactly 4 marker IDs")

    if marker_size_cm >= square_size_cm:
        print(f"Error: marker size ({marker_size_cm}cm) must be smaller than square size ({square_size_cm}cm)")
        return None

    if dictionary not in ARUCO_DICTIONARIES:
        print(f"Error: unknown dictionary '{dictionary}'")
        print(f"Available: {', '.join(ARUCO_DICTIONARIES.keys())}")
        return None

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dictionary])

    # Convert to pixels
    pixels_per_cm = dpi / 2.54
    square_length_px = int(square_size_cm * pixels_per_cm)
    marker_length_px = int(marker_size_cm * pixels_per_cm)
    margin_px = int(2.0 * pixels_per_cm)  # 2cm margin

    # Generate diamond using OpenCV's built-in function
    ids_array = np.array(ids, dtype=np.int32)
    diamond_img = cv2.aruco.drawCharucoDiamond(
        aruco_dict,
        ids_array,
        square_length_px,
        marker_length_px,
        marginSize=margin_px
    )

    # Add label at bottom
    id_string = "_".join(str(i) for i in ids)

    # Extend image for label
    label_height = int(1.5 * pixels_per_cm)
    h, w = diamond_img.shape[:2]
    output_img = np.ones((h + label_height, w), dtype=np.uint8) * 255
    output_img[:h, :] = diamond_img

    label = f"Diamond {id_string} | {dictionary} | sq={square_size_cm}cm mk={marker_size_cm}cm"
    font_scale = w / 2000
    cv2.putText(
        output_img, label,
        (margin_px, h + label_height - int(0.3 * pixels_per_cm)),
        cv2.FONT_HERSHEY_SIMPLEX, max(0.4, font_scale), 0, max(1, int(font_scale * 2))
    )

    # Save
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    filename = f"diamond_{id_string}_{dictionary}.png"
    filepath = out_path / filename
    cv2.imwrite(str(filepath), output_img)

    # Calculate total size
    # OpenCV diamond size = 3*squareLength + 2*marginSize
    total_size_px = 3 * square_length_px + 2 * margin_px
    total_size_cm = total_size_px / pixels_per_cm

    print(f"Generated: {filepath}")
    print(f"  Diamond ID:    {id_string}")
    print(f"  Marker IDs:    {ids}")
    print(f"  Square size:   {square_size_cm} cm")
    print(f"  Marker size:   {marker_size_cm} cm")
    print(f"  Total size:    {total_size_cm:.1f} x {total_size_cm:.1f} cm")
    print(f"  Image size:    {output_img.shape[1]} x {output_img.shape[0]} px")

    return filepath


def parse_diamond_ids(ids_str: str) -> list:
    """
    Parse diamond IDs from string.

    Formats:
        "0_1_2_3" -> [(0, 1, 2, 3)]
        "0_1_2_3,4_5_6_7" -> [(0, 1, 2, 3), (4, 5, 6, 7)]
        "0,1,2,3" -> [(0, 1, 2, 3)] (single diamond, comma-separated)

    Args:
        ids_str: String of IDs

    Returns:
        List of tuples, each with 4 marker IDs
    """
    diamonds = []

    # Check if multiple diamonds (contains underscore groups separated by comma)
    if "_" in ids_str:
        # Format: "0_1_2_3,4_5_6_7"
        for diamond_str in ids_str.split(","):
            parts = diamond_str.strip().split("_")
            if len(parts) != 4:
                raise ValueError(f"Invalid diamond ID '{diamond_str}', need exactly 4 IDs")
            diamonds.append(tuple(int(p) for p in parts))
    else:
        # Format: "0,1,2,3" (single diamond)
        parts = ids_str.split(",")
        if len(parts) != 4:
            raise ValueError(f"Invalid IDs '{ids_str}', need exactly 4 IDs per diamond")
        diamonds.append(tuple(int(p.strip()) for p in parts))

    return diamonds


def main():
    parser = argparse.ArgumentParser(
        description="Generate printable ChArUco Diamond markers"
    )
    parser.add_argument(
        '--ids',
        type=str,
        required=True,
        help='Diamond IDs: "0_1_2_3" or "0_1_2_3,4_5_6_7" for multiple'
    )
    parser.add_argument(
        '--square-size',
        type=float,
        default=20.0,
        help='Chessboard square size in cm (default: 20.0)'
    )
    parser.add_argument(
        '--marker-size',
        type=float,
        default=15.0,
        help='ArUco marker size in cm, must be < square-size (default: 15.0)'
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

    # Validate marker size < square size
    if args.marker_size >= args.square_size:
        print(f"Error: marker-size ({args.marker_size}) must be smaller than square-size ({args.square_size})")
        sys.exit(1)

    # Parse IDs
    try:
        diamonds = parse_diamond_ids(args.ids)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Resolve output path
    output_dir = Path(args.output)
    if not output_dir.is_absolute():
        output_dir = Path(__file__).parent.parent / output_dir

    print("=" * 50)
    print("  CHARUCO DIAMOND MARKER GENERATOR")
    print("=" * 50)
    print(f"\nDictionary: {args.dictionary}")
    print(f"Square size: {args.square_size} cm")
    print(f"Marker size: {args.marker_size} cm")
    # Total = 3*square + 2*margin (margin ~2cm)
    total = 3 * args.square_size + 4
    print(f"Approx total size: {total:.1f} x {total:.1f} cm")
    print()

    for ids in diamonds:
        print(f"\n--- Diamond {ids} ---")
        generate_diamond_marker(
            ids=ids,
            square_size_cm=args.square_size,
            marker_size_cm=args.marker_size,
            dictionary=args.dictionary,
            output_dir=str(output_dir),
            dpi=args.dpi
        )

    print(f"\n{'=' * 50}")
    print("PRINTING INSTRUCTIONS")
    print("=" * 50)
    print("1. Print at 100% scale (no fit-to-page)")
    print("2. Use matte paper to reduce glare")
    print("3. Mount flat on ceiling, facing down")
    print()


if __name__ == "__main__":
    main()
