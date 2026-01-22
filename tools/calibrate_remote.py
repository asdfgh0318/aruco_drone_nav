#!/usr/bin/env python3
"""
Remote Camera Calibration Tool

Calibrates camera using chessboard pattern via network stream.
Connects to RPi camera server and captures calibration images.

Usage:
    python calibrate_remote.py --host 10.156.64.251 --port 8000

Instructions:
    1. Print the chessboard pattern (markers/chessboard_9x6.pdf)
    2. Run this tool
    3. Show chessboard to camera at various angles
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
    """Camera calibration using remote stream."""

    def __init__(self, host: str, port: int,
                 chessboard_size: Tuple[int, int] = (9, 6),
                 square_size_mm: float = 25.0):
        self.host = host
        self.port = port
        self.chessboard_size = chessboard_size  # Inner corners (width, height)
        self.square_size_mm = square_size_mm

        # Stream
        self.stream_url = f"http://{host}:{port}/stream"
        self.reader = MJPEGReader(self.stream_url)

        # Calibration data
        self.captured_images: List[np.ndarray] = []
        self.obj_points: List[np.ndarray] = []  # 3D points
        self.img_points: List[np.ndarray] = []  # 2D points
        self.image_size: Optional[Tuple[int, int]] = None

        # Prepare object points (chessboard corners in 3D)
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size_mm / 1000.0  # Convert to meters

        # Calibration results
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.reprojection_error: float = 0.0

    def run(self):
        """Run the calibration process."""
        print("\n" + "=" * 60)
        print("  CAMERA CALIBRATION")
        print("=" * 60)
        print(f"\nConnecting to camera at {self.host}:{self.port}...")

        self.reader.start()
        time.sleep(1)

        print(f"\nChessboard size: {self.chessboard_size[0]}x{self.chessboard_size[1]} inner corners")
        print(f"Square size: {self.square_size_mm} mm")
        print("\nInstructions:")
        print("  1. Hold the chessboard pattern in view of the camera")
        print("  2. When corners are detected, they appear GREEN")
        print("  3. Press SPACE to capture (need at least 10 captures)")
        print("  4. Move the board to different positions and angles")
        print("  5. Press 'c' to calibrate when you have enough images")
        print("  6. Press 'q' to quit")
        print()

        cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)

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

                # Find chessboard corners
                flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                found, corners = cv2.findChessboardCorners(gray, self.chessboard_size, flags)

                if found:
                    # Refine corners
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                    # Draw corners (green = detected)
                    cv2.drawChessboardCorners(display, self.chessboard_size, corners, found)
                    cv2.putText(display, "CORNERS DETECTED - Press SPACE to capture",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(display, "Looking for chessboard...",
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
                elif key == ord(' ') and found:
                    # Capture this frame
                    self.captured_images.append(frame.copy())
                    self.obj_points.append(self.objp)
                    self.img_points.append(corners)
                    print(f"Captured image {len(self.captured_images)}")
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
        """Perform camera calibration."""
        print("\n" + "=" * 40)
        print("Calibrating...")
        print("=" * 40)

        # Run calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, self.image_size, None, None
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
            'chessboard_size': list(self.chessboard_size),
            'square_size_mm': self.square_size_mm
        }

        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        print(f"\nCalibration saved to: {output_path}")

        # Also deploy to RPi
        self._deploy_to_rpi(output_path)

    def _deploy_to_rpi(self, local_path: Path):
        """Deploy calibration file to RPi."""
        try:
            import paramiko

            print(f"\nDeploying calibration to RPi...")

            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.host, username='pi', password='raspberry', timeout=10)

            sftp = ssh.open_sftp()
            remote_path = '/home/pi/aruco_drone_nav/config/camera_params.yaml'
            sftp.put(str(local_path), remote_path)
            sftp.close()
            ssh.close()

            print(f"Deployed to RPi: {remote_path}")
        except Exception as e:
            print(f"Warning: Could not deploy to RPi: {e}")
            print(f"Please manually copy {local_path} to RPi")


def main():
    parser = argparse.ArgumentParser(description="Remote Camera Calibration")
    parser.add_argument('--host', '-H', default='10.156.64.251', help='Camera server host')
    parser.add_argument('--port', '-p', type=int, default=8000, help='Camera server port')
    parser.add_argument('--width', '-W', type=int, default=9, help='Chessboard width (inner corners)')
    parser.add_argument('--height', '-L', type=int, default=6, help='Chessboard height (inner corners)')
    parser.add_argument('--square-size', '-s', type=float, default=25.0, help='Square size in mm')
    parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )

    calibrator = RemoteCalibrator(
        host=args.host,
        port=args.port,
        chessboard_size=(args.width, args.height),
        square_size_mm=args.square_size
    )
    calibrator.run()


if __name__ == '__main__':
    main()
