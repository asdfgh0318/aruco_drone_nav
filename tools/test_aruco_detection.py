#!/usr/bin/env python3
"""
ArUco/Diamond Detection Test Tool

Live preview of ArUco marker or Diamond marker detection with pose estimation.
Useful for verifying marker detection and camera setup.

Usage:
    python test_aruco_detection.py [--camera 0] [--marker-size 0.20]
    python test_aruco_detection.py --diamond --square-size 0.20 --marker-size 0.10
"""

import argparse
import sys
import logging
import cv2
import numpy as np
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.aruco_detector import ArucoDetector, DiamondDetector, ARUCO_DICTIONARIES
from src.camera_calibration import CameraCalibration


def main():
    parser = argparse.ArgumentParser(
        description="Test ArUco/Diamond marker detection"
    )
    parser.add_argument(
        '--camera', '-c',
        type=int,
        default=0,
        help='Camera device ID (default: 0)'
    )
    parser.add_argument(
        '--calibration',
        type=str,
        default='config/camera_params.yaml',
        help='Camera calibration file'
    )
    parser.add_argument(
        '--dictionary', '-d',
        type=str,
        default='DICT_4X4_50',
        choices=list(ARUCO_DICTIONARIES.keys()),
        help='ArUco dictionary (default: DICT_4X4_50)'
    )
    parser.add_argument(
        '--marker-size', '-s',
        type=float,
        default=0.10,
        help='Marker size in meters (default: 0.10)'
    )
    parser.add_argument(
        '--diamond',
        action='store_true',
        help='Use Diamond marker detection instead of single ArUco'
    )
    parser.add_argument(
        '--square-size',
        type=float,
        default=0.20,
        help='Diamond center square size in meters (default: 0.20)'
    )
    parser.add_argument(
        '--width',
        type=int,
        default=640,
        help='Camera resolution width'
    )
    parser.add_argument(
        '--height',
        type=int,
        default=480,
        help='Camera resolution height'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Verbose output'
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )

    # Load calibration
    calib_path = Path(args.calibration)
    if not calib_path.is_absolute():
        calib_path = Path(__file__).parent.parent / calib_path

    camera_matrix, dist_coeffs = None, None
    if calib_path.exists():
        camera_matrix, dist_coeffs = CameraCalibration.load_calibration(
            str(calib_path)
        )
        print(f"Loaded calibration from {calib_path}")
    else:
        print(f"No calibration file found at {calib_path}")
        print("Using default camera parameters")

    # Create detector
    if args.diamond:
        detector = DiamondDetector(
            camera_id=args.camera,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=args.dictionary,
            square_size_m=args.square_size,
            marker_size_m=args.marker_size,
            resolution=(args.width, args.height)
        )
        mode_name = "DIAMOND"
    else:
        detector = ArucoDetector(
            camera_id=args.camera,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=args.dictionary,
            marker_size_m=args.marker_size,
            resolution=(args.width, args.height)
        )
        mode_name = "ArUco"

    print("\n" + "=" * 50)
    print(f"      {mode_name} DETECTION TEST")
    print("=" * 50)
    print(f"\nMode: {mode_name}")
    print(f"Dictionary: {args.dictionary}")
    if args.diamond:
        print(f"Square size: {args.square_size * 100:.1f} cm")
        print(f"Marker size: {args.marker_size * 100:.1f} cm")
        total = args.square_size + 2 * args.marker_size
        print(f"Total diamond size: {total * 100:.1f} cm")
    else:
        print(f"Marker size: {args.marker_size * 100:.1f} cm")
    print(f"Resolution: {args.width}x{args.height}")
    print("\nControls:")
    print("  Q - Quit")
    print("  S - Save screenshot")
    print("  R - Reset statistics")
    print()

    if not detector.start():
        print("Failed to start camera!")
        sys.exit(1)

    screenshot_count = 0

    try:
        while True:
            frame = detector.get_frame()
            if frame is None:
                continue

            detections = detector.detect(frame)
            display = detector.draw_detections(frame, detections)

            # Add detection info
            y_pos = 60
            for det in detections:
                x, y, z = det.position
                roll, pitch, yaw = det.get_euler_angles()

                # Handle both MarkerDetection and DiamondDetection
                if hasattr(det, 'id_string'):
                    marker_id = det.id_string
                else:
                    marker_id = str(det.marker_id)

                info = (
                    f"ID:{marker_id} "
                    f"pos=({x:+.3f}, {y:+.3f}, {z:.3f})m "
                    f"yaw={yaw:+.1f}deg"
                )
                cv2.putText(
                    display, info, (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
                )
                y_pos += 25

            # Statistics
            stats = (
                f"Frames: {detector.frames_processed} | "
                f"FPS: {detector.detection_rate:.1f}"
            )
            cv2.putText(
                display, stats, (10, display.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
            )

            cv2.imshow(f"{mode_name} Detection Test", display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"screenshot_{screenshot_count:03d}.png"
                cv2.imwrite(filename, display)
                print(f"Saved {filename}")
                screenshot_count += 1
            elif key == ord('r'):
                detector.frames_processed = 0
                print("Statistics reset")

    finally:
        detector.stop()
        cv2.destroyAllWindows()

    print(f"\nTotal frames processed: {detector.frames_processed}")


if __name__ == "__main__":
    main()
