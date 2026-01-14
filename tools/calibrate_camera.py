#!/usr/bin/env python3
"""
Camera Calibration Tool

Interactive tool for calibrating camera intrinsic parameters
using a chessboard pattern.

Usage:
    python calibrate_camera.py [--camera 0] [--output config/camera_params.yaml]
"""

import argparse
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.camera_calibration import interactive_calibration


def main():
    parser = argparse.ArgumentParser(
        description="Camera calibration using chessboard pattern"
    )
    parser.add_argument(
        '--camera', '-c',
        type=int,
        default=0,
        help='Camera device ID (default: 0)'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='config/camera_params.yaml',
        help='Output file path'
    )
    parser.add_argument(
        '--images', '-n',
        type=int,
        default=15,
        help='Number of calibration images to capture (default: 15)'
    )
    parser.add_argument(
        '--board-width',
        type=int,
        default=9,
        help='Chessboard inner corners width (default: 9)'
    )
    parser.add_argument(
        '--board-height',
        type=int,
        default=6,
        help='Chessboard inner corners height (default: 6)'
    )
    parser.add_argument(
        '--square-size',
        type=float,
        default=0.025,
        help='Chessboard square size in meters (default: 0.025)'
    )

    args = parser.parse_args()

    # Resolve output path
    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = Path(__file__).parent.parent / output_path

    print("\n" + "=" * 50)
    print("        CAMERA CALIBRATION TOOL")
    print("=" * 50)
    print(f"\nCamera ID: {args.camera}")
    print(f"Output: {output_path}")
    print(f"Chessboard: {args.board_width}x{args.board_height} corners")
    print(f"Square size: {args.square_size * 100:.1f} cm")
    print(f"Images to capture: {args.images}")
    print("\nPrint a chessboard pattern and present it to the camera")
    print("at various angles and distances.\n")

    success = interactive_calibration(
        camera_id=args.camera,
        output_path=str(output_path),
        num_images=args.images,
        chessboard_size=(args.board_width, args.board_height),
        square_size_m=args.square_size
    )

    if success:
        print("\nCalibration completed successfully!")
        print(f"Parameters saved to: {output_path}")
    else:
        print("\nCalibration was not completed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
