#!/usr/bin/env python3
"""
Remote Camera Calibration Tool

Calibrates camera using ChArUco board pattern via network stream.
Connects to RPi camera server and captures calibration images.

Usage:
    python calibrate_remote.py --host aruconav.local --port 8000

Instructions:
    1. Display/print the ChArUco board (markers/charuco_board_*.png)
    2. Run this tool
    3. Show board to camera at various angles
    4. Press SPACE to capture when corners are detected (green)
    5. Capture at least 10 images from different angles
    6. Press 'c' to calibrate and save
"""

import argparse
import json
import logging
import sys
import threading
import time
import urllib.request
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

logger = logging.getLogger(__name__)


class MJPEGReader:
    """Simple MJPEG stream reader."""

    def __init__(self, url: str):
        self.url = url
        self.frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)

    def _read_loop(self):
        while self.running:
            try:
                stream = urllib.request.urlopen(self.url, timeout=5)
                buffer = b''
                while self.running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buffer += chunk
                    start = buffer.find(b'\xff\xd8')
                    end = buffer.find(b'\xff\xd9')
                    if start != -1 and end != -1 and end > start:
                        jpg = buffer[start:end + 2]
                        buffer = buffer[end + 2:]
                        img = np.frombuffer(jpg, dtype=np.uint8)
                        frame = cv2.imdecode(img, cv2.IMREAD_COLOR)
                        if frame is not None:
                            with self.lock:
                                self.frame = frame
            except Exception as e:
                logger.warning(f"Stream error: {e}")
                time.sleep(1)

    def get_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            return self.frame.copy() if self.frame is not None else None


class RemoteCalibrator:
    """Camera calibration using remote stream with ChArUco board."""

    def __init__(self, host: str, port: int,
                 board_size: Tuple[int, int] = (7, 5),
                 square_size_mm: float = 40.0,
                 marker_size_mm: float = 30.0,
                 dictionary: str = "DICT_4X4_50"):
        self.host = host
        self.port = port
        self.board_size = board_size  # Number of squares (width, height)
        self.square_size_mm = square_size_mm
        self.marker_size_mm = marker_size_mm

        # Stream
        self.stream_url = f"http://{host}:{port}/stream"
        self.reader = MJPEGReader(self.stream_url)

        # Setup ArUco dictionary and ChArUco board
        dict_id = getattr(cv2.aruco, dictionary)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # Create ChArUco board - IDs start from 0
        # Markers are on alternating squares (checkerboard pattern)
        num_markers = (board_size[0] * board_size[1]) // 2
        # For 7x5: 35//2 = 17 markers (IDs 0-16)
        ids = np.arange(num_markers, dtype=np.int32)
        self.charuco_board = cv2.aruco.CharucoBoard(
            board_size,
            square_size_mm / 1000.0,  # Convert to meters
            marker_size_mm / 1000.0,   # Convert to meters
            self.aruco_dict,
            ids
        )
        self.charuco_detector = cv2.aruco.CharucoDetector(self.charuco_board)

        # Calibration data
        self.captured_images: List[np.ndarray] = []
        self.all_charuco_corners: List[np.ndarray] = []
        self.all_charuco_ids: List[np.ndarray] = []
        self.image_size: Optional[Tuple[int, int]] = None

        # Calibration results
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.reprojection_error: float = 0.0

    def run(self):
        """Run the calibration process."""
        print("\n" + "=" * 60)
        print("  CHARUCO CAMERA CALIBRATION")
        print("=" * 60)
        print(f"\nConnecting to camera at {self.host}:{self.port}...")

        self.reader.start()
        time.sleep(1)

        print(f"\nChArUco board: {self.board_size[0]}x{self.board_size[1]} squares")
        print(f"Square size: {self.square_size_mm} mm")
        print(f"Marker size: {self.marker_size_mm} mm")
        print("\nInstructions:")
        print("  1. Hold the ChArUco board in view of the camera")
        print("  2. When corners are detected, they appear GREEN")
        print("  3. Press SPACE to capture (need at least 10 captures)")
        print("  4. Move the board to different positions and angles")
        print("  5. Press 'c' to calibrate when you have enough images")
        print("  6. Press 'q' to quit")
        print()

        cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
        current_corners = None
        current_ids = None

        try:
            while True:
                frame = self.reader.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                if self.image_size is None:
                    self.image_size = (frame.shape[1], frame.shape[0])

                display = frame.copy()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detect ChArUco board
                charuco_corners, charuco_ids, marker_corners, marker_ids = \
                    self.charuco_detector.detectBoard(gray)

                found = charuco_corners is not None and len(charuco_corners) >= 4

                if found:
                    current_corners = charuco_corners
                    current_ids = charuco_ids

                    # Draw detected markers
                    if marker_corners is not None and len(marker_corners) > 0:
                        cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)

                    # Draw ChArUco corners
                    cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids, (0, 255, 0))

                    cv2.putText(display, f"DETECTED {len(charuco_corners)} corners - Press SPACE to capture",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    current_corners = None
                    current_ids = None
                    # Still draw any detected markers for debugging
                    if marker_corners is not None and len(marker_corners) > 0:
                        cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
                        cv2.putText(display, f"Found {len(marker_corners)} markers, need ChArUco corners...",
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    else:
                        cv2.putText(display, "Looking for ChArUco board...",
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # Show capture count
                cv2.putText(display, f"Captured: {len(self.captured_images)}/10 (min)",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display, "SPACE=capture  C=calibrate  Q=quit",
                           (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                cv2.imshow("Calibration", display)
                key = cv2.waitKey(30) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord(' ') and found and current_corners is not None:
                    # Capture this frame
                    self.captured_images.append(frame.copy())
                    self.all_charuco_corners.append(current_corners)
                    self.all_charuco_ids.append(current_ids)
                    print(f"Captured image {len(self.captured_images)} ({len(current_corners)} corners)")
                elif key == ord('c'):
                    if len(self.captured_images) >= 10:
                        self._calibrate()
                        break
                    else:
                        print(f"Need at least 10 images, have {len(self.captured_images)}")

        finally:
            cv2.destroyAllWindows()
            self.reader.stop()

    def _calibrate(self):
        """Perform camera calibration using ChArUco."""
        print("\n" + "=" * 40)
        print("Calibrating with ChArUco...")
        print("=" * 40)

        # Run ChArUco calibration
        ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            self.all_charuco_corners,
            self.all_charuco_ids,
            self.charuco_board,
            self.image_size,
            None, None
        )

        self.camera_matrix = mtx
        self.dist_coeffs = dist
        self.reprojection_error = ret

        print(f"\nCalibration complete!")
        print(f"Reprojection error: {ret:.4f} pixels")
        print(f"  (Good if < 1.0, excellent if < 0.5)")

        print(f"\nCamera matrix:")
        print(f"  fx = {mtx[0, 0]:.2f}")
        print(f"  fy = {mtx[1, 1]:.2f}")
        print(f"  cx = {mtx[0, 2]:.2f}")
        print(f"  cy = {mtx[1, 2]:.2f}")

        print(f"\nDistortion coefficients:")
        print(f"  {dist.flatten()}")

        # Save calibration
        self._save_calibration()

    def _save_calibration(self):
        """Save calibration to YAML file."""
        output_path = Path(__file__).parent.parent / "config" / "camera_params.yaml"

        data = {
            'calibration_date': datetime.now().isoformat(),
            'image_width': self.image_size[0],
            'image_height': self.image_size[1],
            'camera_matrix': self.camera_matrix.tolist(),
            'distortion_coefficients': self.dist_coeffs.flatten().tolist(),
            'reprojection_error': float(self.reprojection_error),
            'num_images': len(self.captured_images),
            'calibration_method': 'charuco',
            'charuco_board_size': list(self.board_size),
            'square_size_mm': self.square_size_mm,
            'marker_size_mm': self.marker_size_mm
        }

        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        print(f"\nCalibration saved to: {output_path}")

        # Also deploy to RPi
        self._deploy_to_rpi(output_path)

    def _deploy_to_rpi(self, local_path: Path):
        """Deploy calibration file to RPi."""
        try:
            import subprocess

            print(f"\nDeploying calibration to RPi...")

            # Use scp with SSH key (no password needed)
            remote_path = f'aruconav@{self.host}:/home/aruconav/aruco_drone_nav/config/camera_params.yaml'
            result = subprocess.run(
                ['scp', str(local_path), remote_path],
                capture_output=True, text=True, timeout=30
            )

            if result.returncode == 0:
                print(f"Deployed to RPi: {remote_path}")
            else:
                print(f"Warning: scp failed: {result.stderr}")
                print(f"Please manually copy {local_path} to RPi")
        except Exception as e:
            print(f"Warning: Could not deploy to RPi: {e}")
            print(f"Please manually copy {local_path} to RPi")


def main():
    parser = argparse.ArgumentParser(description="Remote ChArUco Camera Calibration")
    parser.add_argument('--host', '-H', default='aruconav.local', help='Camera server host')
    parser.add_argument('--port', '-p', type=int, default=8000, help='Camera server port')
    parser.add_argument('--width', '-W', type=int, default=7, help='ChArUco board width (squares)')
    parser.add_argument('--height', '-L', type=int, default=5, help='ChArUco board height (squares)')
    parser.add_argument('--square-size', '-s', type=float, default=40.0, help='Square size in mm')
    parser.add_argument('--marker-size', '-m', type=float, default=30.0, help='Marker size in mm')
    parser.add_argument('--dict', '-d', default='DICT_4X4_50', help='ArUco dictionary')
    parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )

    calibrator = RemoteCalibrator(
        host=args.host,
        port=args.port,
        board_size=(args.width, args.height),
        square_size_mm=args.square_size,
        marker_size_mm=args.marker_size,
        dictionary=args.dict
    )
    calibrator.run()


if __name__ == '__main__':
    main()
