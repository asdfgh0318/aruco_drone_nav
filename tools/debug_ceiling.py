#!/usr/bin/env python3
"""
Ceiling Debug GUI

Local debug tool for testing ceiling-mounted diamond marker detection.
Connects to RPi camera stream, runs detection locally, and displays:
- Live camera feed with detection overlay
- Top-down position view with marker map
- Live stats (FPS, detection rate, distance, confidence)
- Position history trail

Usage:
    python tools/debug_ceiling.py --host aruconav.local --port 8000
"""

import argparse
import logging
import sys
import threading
import time
import urllib.request
from collections import deque
from pathlib import Path
from typing import Optional, List, Tuple

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.aruco_detector import DiamondDetector, DiamondDetection
from src.camera_calibration import CameraCalibration

logger = logging.getLogger(__name__)


class MJPEGReader:
    """MJPEG stream reader with reconnection."""

    def __init__(self, url: str):
        self.url = url
        self.frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.connected = False
        self.fps = 0.0
        self._frame_times = deque(maxlen=30)

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
                self.connected = False
                stream = urllib.request.urlopen(self.url, timeout=5)
                self.connected = True
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
                            now = time.time()
                            self._frame_times.append(now)
                            if len(self._frame_times) > 1:
                                dt = self._frame_times[-1] - self._frame_times[0]
                                if dt > 0:
                                    self.fps = (len(self._frame_times) - 1) / dt

                            with self.lock:
                                self.frame = frame

            except Exception as e:
                self.connected = False
                logger.debug(f"Stream error: {e}")
                time.sleep(1)

    def get_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            return self.frame.copy() if self.frame is not None else None


class CeilingDebugGUI:
    """Debug GUI for ceiling marker detection."""

    def __init__(
        self,
        host: str,
        port: int,
        config_path: str,
        calibration_path: str,
        marker_map_path: str
    ):
        self.host = host
        self.port = port

        # Load configuration
        with open(config_path) as f:
            self.config = yaml.safe_load(f)

        # Load marker map
        with open(marker_map_path) as f:
            self.marker_map = yaml.safe_load(f)

        # Stream reader
        self.stream_url = f"http://{host}:{port}/stream"
        self.reader = MJPEGReader(self.stream_url)

        # Diamond detector (uses local calibration)
        self.detector = self._create_detector(config_path, calibration_path)

        # Stats
        self.detection_count = 0
        self.frame_count = 0
        self.position_history: deque = deque(maxlen=100)
        self.last_position: Optional[Tuple[float, float, float]] = None
        self.last_yaw: float = 0.0
        self.last_detections: List[DiamondDetection] = []

        # Debug: ArUco marker detection (even without diamond)
        self.last_aruco_corners = None
        self.last_aruco_ids = None
        self.last_rejected = None

        # Log buffer for copy-paste
        self.log_lines: deque = deque(maxlen=20)
        self.log_lines.append("=== DEBUG LOG (press L to copy) ===")
        self.last_brightness = 128
        self.last_camera_matrix = None
        self.calib_width = 2592  # OV5693 5MP calibration resolution
        self.calib_height = 1944

        # Display settings (will adapt to actual camera resolution)
        self.camera_width = 640
        self.camera_height = 480
        self.map_size = 400
        self.stats_width = 250

    def _create_detector(self, config_path: str, calibration_path: str):
        """Create diamond detector with local processing."""
        with open(config_path) as f:
            config = yaml.safe_load(f)

        # Load calibration
        camera_matrix, dist_coeffs = CameraCalibration.load_calibration(calibration_path)
        if camera_matrix is None:
            logger.warning("No calibration, using defaults")
            camera_matrix = np.array([
                [500, 0, 320],
                [0, 500, 240],
                [0, 0, 1]
            ], dtype=np.float64)
            dist_coeffs = np.zeros(5)

        # Get ArUco settings
        aruco_cfg = config.get('aruco', {})
        diamond_cfg = config.get('diamond', {})

        dict_name = aruco_cfg.get('dictionary', 'DICT_4X4_50')
        dict_id = getattr(cv2.aruco, dict_name)
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        return {
            'aruco_dict': aruco_dict,
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs,
            'square_size': diamond_cfg.get('square_size_m', 0.04),
            'marker_size': diamond_cfg.get('marker_size_m', 0.02),
        }

    def log(self, msg: str):
        """Add message to log buffer."""
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {msg}"
        self.log_lines.append(line)
        print(line)  # Also print to console

    def detect_diamonds(self, frame: np.ndarray) -> List[DiamondDetection]:
        """Run diamond detection on a frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]

        # Scale camera matrix if resolution differs from calibration
        camera_matrix = self.detector['camera_matrix'].copy()
        if w != self.calib_width or h != self.calib_height:
            scale_x = w / self.calib_width
            scale_y = h / self.calib_height
            camera_matrix[0, 0] *= scale_x  # fx
            camera_matrix[1, 1] *= scale_y  # fy
            camera_matrix[0, 2] *= scale_x  # cx
            camera_matrix[1, 2] *= scale_y  # cy

        # Check image brightness
        brightness = gray.mean()
        if self.frame_count % 30 == 0:
            if brightness > 240:
                self.log(f"WARNING: Image overexposed! brightness={brightness:.0f}/255")
            elif brightness < 30:
                self.log(f"WARNING: Image too dark! brightness={brightness:.0f}/255")

        # Detect ArUco markers
        detector_params = cv2.aruco.DetectorParameters()
        aruco_detector = cv2.aruco.ArucoDetector(
            self.detector['aruco_dict'],
            detector_params
        )
        corners, ids, rejected = aruco_detector.detectMarkers(gray)

        # Store for debug display
        self.last_aruco_corners = corners
        self.last_aruco_ids = ids
        self.last_rejected = rejected
        self.last_brightness = brightness
        self.last_camera_matrix = camera_matrix  # Scaled for current resolution

        # Log every 30 frames
        if self.frame_count % 30 == 0:
            if ids is not None and len(ids) > 0:
                ids_list = ids.flatten().tolist()
                sizes = [f"{int(np.linalg.norm(c[0][0]-c[0][1]))}px" for c in corners]
                self.log(f"ArUco: {len(ids)} markers {ids_list} sizes={sizes}")
            else:
                self.log(f"No markers (rejected={len(rejected) if rejected else 0}, bright={brightness:.0f})")

        if ids is None or len(ids) < 4:
            return []

        # Detect diamonds
        sq_ratio = self.detector['square_size'] / self.detector['marker_size']
        diamond_corners, diamond_ids = cv2.aruco.detectCharucoDiamond(
            gray, corners, ids,
            sq_ratio,
            cameraMatrix=camera_matrix,
            distCoeffs=self.detector['dist_coeffs']
        )

        # Log diamond detection result
        if self.frame_count % 30 == 0:
            if diamond_ids is not None and len(diamond_ids) > 0:
                self.log(f"DIAMOND FOUND: {[tuple(d.flatten()) for d in diamond_ids]}")
            elif ids is not None and len(ids) >= 4:
                self.log(f"No diamond from {len(ids)} markers (ratio={sq_ratio:.2f})")

        if diamond_ids is None or len(diamond_ids) == 0:
            return []

        detections = []
        for i, (d_corners, d_id) in enumerate(zip(diamond_corners, diamond_ids)):
            # Estimate pose using scaled camera matrix
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [d_corners],
                self.detector['square_size'],
                camera_matrix,
                self.detector['dist_coeffs']
            )

            distance = np.linalg.norm(tvec[0][0])

            detection = DiamondDetection(
                diamond_id=tuple(d_id.flatten()),
                corners=d_corners,
                rvec=rvec[0][0],
                tvec=tvec[0][0],
                distance=distance,
                timestamp=time.time()
            )
            detections.append(detection)

        return detections

    def estimate_position(self, detections: List[DiamondDetection]) -> Optional[Tuple[float, float, float, float]]:
        """Estimate drone position from detections."""
        if not detections:
            return None

        # Use first detection for now
        det = detections[0]

        # Find marker in map
        marker_pos = None
        for diamond in self.marker_map.get('diamonds', []):
            if diamond['id'] == det.id_string:
                marker_pos = diamond['position']
                break

        if marker_pos is None:
            return None

        # Camera-to-marker vector (in camera frame)
        tvec = det.tvec

        # Convert rotation vector to matrix
        R_cam_to_marker, _ = cv2.Rodrigues(det.rvec)

        # Drone position = marker position - rotated tvec
        # (simplified - assumes marker faces down, camera faces up)
        drone_x = marker_pos[0] - tvec[0]
        drone_y = marker_pos[1] - tvec[1]
        drone_z = marker_pos[2] - tvec[2]

        # Extract yaw from rotation matrix (simplified)
        yaw = np.arctan2(R_cam_to_marker[1, 0], R_cam_to_marker[0, 0])
        yaw_deg = np.degrees(yaw)

        return (drone_x, drone_y, drone_z, yaw_deg)

    def draw_camera_view(self, frame: np.ndarray, detections: List[DiamondDetection]) -> np.ndarray:
        """Draw detection overlay on camera frame."""
        display = frame.copy()

        # Draw individual ArUco markers (yellow) - even if no diamond formed
        if self.last_aruco_corners is not None and self.last_aruco_ids is not None:
            cv2.aruco.drawDetectedMarkers(display, self.last_aruco_corners, self.last_aruco_ids, (0, 255, 255))

        # Draw rejected candidates (red, smaller)
        if self.last_rejected is not None and len(self.last_rejected) > 0:
            for rej in self.last_rejected:
                pts = rej.reshape(-1, 2).astype(int)
                cv2.polylines(display, [pts], True, (0, 0, 255), 1)

        # Draw detected diamond markers (green)
        if detections:
            for det in detections:
                # Draw diamond corners
                corners = det.corners.reshape(-1, 2).astype(int)
                cv2.polylines(display, [corners], True, (0, 255, 0), 3)

                # Draw axes
                cam_mtx = getattr(self, 'last_camera_matrix', self.detector['camera_matrix'])
                cv2.drawFrameAxes(
                    display,
                    cam_mtx,
                    self.detector['dist_coeffs'],
                    det.rvec, det.tvec,
                    self.detector['square_size'] * 0.5
                )

                # Draw ID and distance
                center = corners.mean(axis=0).astype(int)
                cv2.putText(
                    display, f"DIAMOND {det.id_string}",
                    (center[0] - 60, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
                cv2.putText(
                    display, f"{det.distance:.2f}m",
                    (center[0] - 30, center[1] + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
                )

        # Connection status
        status_color = (0, 255, 0) if self.reader.connected else (0, 0, 255)
        status_text = f"Connected ({self.reader.fps:.1f} FPS)" if self.reader.connected else "Disconnected"
        cv2.putText(display, status_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

        # ArUco marker count
        aruco_count = len(self.last_aruco_ids) if self.last_aruco_ids is not None else 0
        aruco_ids_str = ",".join(str(i) for i in self.last_aruco_ids.flatten()) if self.last_aruco_ids is not None else "none"
        cv2.putText(display, f"ArUco: {aruco_count} [{aruco_ids_str}]", (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Brightness indicator
        brightness = getattr(self, 'last_brightness', 128)
        if brightness > 240:
            bright_color = (0, 0, 255)  # Red - overexposed
            bright_text = f"OVEREXPOSED ({brightness:.0f})"
        elif brightness < 30:
            bright_color = (0, 0, 255)  # Red - too dark
            bright_text = f"TOO DARK ({brightness:.0f})"
        else:
            bright_color = (0, 255, 0)  # Green - ok
            bright_text = f"Brightness: {brightness:.0f}"
        cv2.putText(display, bright_text, (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, bright_color, 2)

        return display

    def draw_map_view(self) -> np.ndarray:
        """Draw top-down position map."""
        map_img = np.ones((self.map_size, self.map_size, 3), dtype=np.uint8) * 40

        # Map scale: 4m x 4m area
        scale = self.map_size / 4.0  # pixels per meter
        center_x = self.map_size // 2
        center_y = self.map_size // 2

        def world_to_map(x, y):
            mx = int(center_x + x * scale)
            my = int(center_y - y * scale)  # Y inverted
            return (mx, my)

        # Draw grid
        for i in range(-2, 3):
            # Vertical lines
            x = int(center_x + i * scale)
            cv2.line(map_img, (x, 0), (x, self.map_size), (60, 60, 60), 1)
            # Horizontal lines
            y = int(center_y - i * scale)
            cv2.line(map_img, (0, y), (self.map_size, y), (60, 60, 60), 1)

        # Draw origin
        cv2.circle(map_img, (center_x, center_y), 5, (100, 100, 100), -1)
        cv2.putText(map_img, "O", (center_x + 8, center_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

        # Draw markers from map
        for diamond in self.marker_map.get('diamonds', []):
            pos = diamond['position']
            mx, my = world_to_map(pos[0], pos[1])
            cv2.drawMarker(map_img, (mx, my), (0, 165, 255), cv2.MARKER_DIAMOND, 15, 2)
            cv2.putText(map_img, diamond['id'], (mx + 10, my), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 165, 255), 1)

        # Draw position history trail
        if len(self.position_history) > 1:
            points = [world_to_map(p[0], p[1]) for p in self.position_history]
            for i in range(1, len(points)):
                alpha = i / len(points)
                color = (int(100 * alpha), int(200 * alpha), int(100 * alpha))
                cv2.line(map_img, points[i-1], points[i], color, 1)

        # Draw current position
        if self.last_position:
            px, py = world_to_map(self.last_position[0], self.last_position[1])

            # Draw drone as triangle pointing in yaw direction
            yaw_rad = np.radians(self.last_yaw)
            size = 12
            pts = np.array([
                [px + size * np.cos(yaw_rad), py - size * np.sin(yaw_rad)],
                [px + size * np.cos(yaw_rad + 2.5), py - size * np.sin(yaw_rad + 2.5)],
                [px + size * np.cos(yaw_rad - 2.5), py - size * np.sin(yaw_rad - 2.5)],
            ], dtype=np.int32)
            cv2.fillPoly(map_img, [pts], (0, 255, 0))
            cv2.circle(map_img, (px, py), 4, (255, 255, 255), -1)

        # Title
        cv2.putText(map_img, "Top-Down View", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Scale indicator
        cv2.line(map_img, (10, self.map_size - 20), (10 + int(scale), self.map_size - 20), (200, 200, 200), 2)
        cv2.putText(map_img, "1m", (10, self.map_size - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        return map_img

    def draw_stats_panel(self) -> np.ndarray:
        """Draw statistics panel."""
        # Use fixed height for layout, will be resized later
        panel_height = 480
        panel = np.ones((panel_height, self.stats_width, 3), dtype=np.uint8) * 30

        y = 30
        line_height = 25

        def draw_stat(label, value, color=(200, 200, 200)):
            nonlocal y
            cv2.putText(panel, label, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(panel, str(value), (10, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            y += line_height + 15

        # Title
        cv2.putText(panel, "STATS", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y += 35

        # Detection rate
        rate = (self.detection_count / max(1, self.frame_count)) * 100
        rate_color = (0, 255, 0) if rate > 80 else (0, 255, 255) if rate > 50 else (0, 0, 255)
        draw_stat("Detection Rate", f"{rate:.1f}%", rate_color)

        # Frame count
        draw_stat("Frames", f"{self.frame_count}")

        # Detections
        draw_stat("Detections", f"{self.detection_count}")

        # ArUco markers detected
        aruco_count = len(self.last_aruco_ids) if self.last_aruco_ids is not None else 0
        aruco_color = (0, 255, 0) if aruco_count >= 4 else (0, 255, 255) if aruco_count > 0 else (100, 100, 100)
        draw_stat("ArUco Markers", f"{aruco_count}", aruco_color)

        # Current detections
        if self.last_detections:
            det = self.last_detections[0]
            draw_stat("Diamond ID", det.id_string, (0, 255, 0))
            draw_stat("Distance", f"{det.distance:.3f} m", (0, 255, 255))
        else:
            draw_stat("Diamond ID", "None", (100, 100, 100))
            draw_stat("Distance", "---", (100, 100, 100))

        # Position
        y += 10
        cv2.line(panel, (10, y), (self.stats_width - 10, y), (80, 80, 80), 1)
        y += 20
        cv2.putText(panel, "POSITION", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        y += 25

        if self.last_position:
            x, y_pos, z = self.last_position
            cv2.putText(panel, f"X: {x:+.3f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 255), 1)
            y += 22
            cv2.putText(panel, f"Y: {y_pos:+.3f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
            y += 22
            cv2.putText(panel, f"Z: {z:+.3f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
            y += 22
            cv2.putText(panel, f"Yaw: {self.last_yaw:+.1f} deg", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 1)
        else:
            cv2.putText(panel, "No position", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)

        # Log section
        y += 10
        cv2.line(panel, (10, y), (self.stats_width - 10, y), (80, 80, 80), 1)
        y += 15
        cv2.putText(panel, "LOG (L=copy)", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        y += 12
        for line in list(self.log_lines)[-6:]:  # Show last 6 lines
            # Truncate long lines
            display_line = line[-35:] if len(line) > 35 else line
            cv2.putText(panel, display_line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (180, 180, 180), 1)
            y += 11

        # Controls
        y = panel_height - 60
        cv2.line(panel, (10, y), (self.stats_width - 10, y), (80, 80, 80), 1)
        y += 20
        cv2.putText(panel, "Q:Quit C:Clear R:Reset", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
        y += 15
        cv2.putText(panel, "L: Copy log to terminal", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        return panel

    def run(self):
        """Run the debug GUI."""
        print("\n" + "=" * 50)
        print("  CEILING DEBUG GUI")
        print("=" * 50)
        print(f"\nConnecting to {self.host}:{self.port}...")
        print(f"\nDetection parameters:")
        print(f"  Square size: {self.detector['square_size']*100:.1f} cm")
        print(f"  Marker size: {self.detector['marker_size']*100:.1f} cm")
        print(f"  Size ratio:  {self.detector['square_size']/self.detector['marker_size']:.2f}")
        print(f"\nExpected diamond marker on ceiling: 4_5_6_7")
        print("\nControls:")
        print("  Q - Quit")
        print("  C - Clear position trail")
        print("  R - Reset statistics")
        print("\n--- Detection log (every 30 frames) ---")

        self.reader.start()

        # Create window
        window_name = "Ceiling Debug"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        # Initial size - will adapt when first frame arrives
        cv2.resizeWindow(window_name, 1200, 600)

        try:
            while True:
                frame = self.reader.get_frame()

                if frame is not None:
                    # Update dimensions from actual frame
                    orig_h, orig_w = frame.shape[:2]
                    self.frame_count += 1

                    # Detect diamonds on full resolution
                    detections = self.detect_diamonds(frame)
                    self.last_detections = detections

                    if detections:
                        self.detection_count += 1

                        # Estimate position
                        pos = self.estimate_position(detections)
                        if pos:
                            self.last_position = (pos[0], pos[1], pos[2])
                            self.last_yaw = pos[3]
                            self.position_history.append(self.last_position)

                    # Draw camera view then resize for display
                    camera_view = self.draw_camera_view(frame, detections)

                    # Scale down large frames for display (max 800px wide)
                    max_cam_width = 800
                    if orig_w > max_cam_width:
                        scale = max_cam_width / orig_w
                        new_w = int(orig_w * scale)
                        new_h = int(orig_h * scale)
                        camera_view = cv2.resize(camera_view, (new_w, new_h))

                    self.camera_width = camera_view.shape[1]
                    self.camera_height = camera_view.shape[0]
                else:
                    # No frame - show placeholder
                    camera_view = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
                    cv2.putText(camera_view, "Connecting...", (200, 240),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)

                # Draw map and stats - resize to match camera height
                map_view = self.draw_map_view()
                stats_panel = self.draw_stats_panel()

                # Resize map and stats to match camera height
                map_view = cv2.resize(map_view, (self.map_size, self.camera_height))
                stats_panel = cv2.resize(stats_panel, (self.stats_width, self.camera_height))

                # Combine views
                display = np.hstack([camera_view, map_view, stats_panel])

                cv2.imshow(window_name, display)

                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    self.position_history.clear()
                    self.log("Position trail cleared")
                elif key == ord('r'):
                    self.frame_count = 0
                    self.detection_count = 0
                    self.position_history.clear()
                    self.log("Statistics reset")
                elif key == ord('l'):
                    # Print full log for copy-paste
                    print("\n" + "="*50)
                    print("FULL DEBUG LOG - COPY BELOW THIS LINE")
                    print("="*50)
                    for line in self.log_lines:
                        print(line)
                    print("="*50)
                    self.log("Log printed to console (copy from terminal)")

        finally:
            cv2.destroyAllWindows()
            self.reader.stop()
            print("\nGUI closed")


def main():
    parser = argparse.ArgumentParser(description="Ceiling Debug GUI")
    parser.add_argument('--host', '-H', default='aruconav.local', help='Camera server host')
    parser.add_argument('--port', '-p', type=int, default=8000, help='Camera server port')
    parser.add_argument('--config', '-c', default='config/system_config.yaml', help='System config')
    parser.add_argument('--calibration', default='config/camera_params.yaml', help='Camera calibration')
    parser.add_argument('--marker-map', '-m', default='config/marker_map.yaml', help='Marker map')
    parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )

    # Resolve paths
    base_dir = Path(__file__).parent.parent
    config_path = base_dir / args.config
    calibration_path = base_dir / args.calibration
    marker_map_path = base_dir / args.marker_map

    gui = CeilingDebugGUI(
        host=args.host,
        port=args.port,
        config_path=str(config_path),
        calibration_path=str(calibration_path),
        marker_map_path=str(marker_map_path)
    )
    gui.run()


if __name__ == '__main__':
    main()
