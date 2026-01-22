#!/usr/bin/env python3
"""
Bench Test Tool for ArUco Drone Navigation

Comprehensive testing tool that shows:
- ArUco marker detection with pose estimation
- Position/offset calculations
- Computed velocity commands (what would be sent to autopilot)
- Real-time statistics and benchmarks

No MAVLink connection required - perfect for desktop testing.

Usage:
    python bench_test.py [--camera 0] [--marker-size 0.20]
"""

import argparse
import sys
import time
import logging
from pathlib import Path
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.aruco_detector import ArucoDetector, MarkerDetection, ARUCO_DICTIONARIES
from src.camera_calibration import CameraCalibration
from src.position_estimator import SimplePositionEstimator

logger = logging.getLogger(__name__)


@dataclass
class BenchStats:
    """Benchmark statistics."""
    frames_total: int = 0
    frames_with_detection: int = 0
    detection_times: deque = None
    loop_times: deque = None

    def __post_init__(self):
        if self.detection_times is None:
            self.detection_times = deque(maxlen=100)
        if self.loop_times is None:
            self.loop_times = deque(maxlen=100)

    @property
    def detection_rate(self) -> float:
        if self.frames_total == 0:
            return 0.0
        return self.frames_with_detection / self.frames_total * 100

    @property
    def avg_detection_ms(self) -> float:
        if not self.detection_times:
            return 0.0
        return sum(self.detection_times) / len(self.detection_times) * 1000

    @property
    def avg_loop_ms(self) -> float:
        if not self.loop_times:
            return 0.0
        return sum(self.loop_times) / len(self.loop_times) * 1000

    @property
    def fps(self) -> float:
        if not self.loop_times or self.avg_loop_ms == 0:
            return 0.0
        return 1000 / self.avg_loop_ms


class SimplePIDController:
    """Simple PID controller for bench testing."""

    def __init__(self, kp: float = 0.5, ki: float = 0.0, kd: float = 0.1, limit: float = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    def update(self, error: float) -> float:
        now = time.time()
        dt = 0.05 if self._last_time is None else (now - self._last_time)
        self._last_time = now

        # P
        p_term = self.kp * error

        # I with anti-windup
        self._integral += error * dt
        self._integral = np.clip(self._integral, -self.limit / (self.ki + 1e-6), self.limit / (self.ki + 1e-6))
        i_term = self.ki * self._integral

        # D
        d_term = self.kd * (error - self._last_error) / dt if dt > 0 else 0
        self._last_error = error

        return np.clip(p_term + i_term + d_term, -self.limit, self.limit)

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None


class BenchTest:
    """Bench test runner."""

    def __init__(
        self,
        camera_id: int = 0,
        calibration_path: Optional[str] = None,
        dictionary: str = "DICT_6X6_250",
        marker_size_m: float = 0.20,
        resolution: Tuple[int, int] = (640, 480),
        target_altitude_m: float = 1.5
    ):
        self.camera_id = camera_id
        self.marker_size_m = marker_size_m
        self.resolution = resolution
        self.target_altitude_m = target_altitude_m

        # Load calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        if calibration_path and Path(calibration_path).exists():
            self.camera_matrix, self.dist_coeffs = CameraCalibration.load_calibration(calibration_path)
            logger.info(f"Loaded calibration from {calibration_path}")

        # Create detector
        self.detector = ArucoDetector(
            camera_id=camera_id,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            dictionary=dictionary,
            marker_size_m=marker_size_m,
            resolution=resolution
        )

        # Position estimator
        self.estimator = SimplePositionEstimator()

        # PID controllers
        self.pid_x = SimplePIDController(kp=0.5, ki=0.0, kd=0.1, limit=0.5)
        self.pid_y = SimplePIDController(kp=0.5, ki=0.0, kd=0.1, limit=0.5)
        self.pid_z = SimplePIDController(kp=0.3, ki=0.0, kd=0.05, limit=0.3)

        # Statistics
        self.stats = BenchStats()

        # State
        self.running = False
        self.show_help = True

    def start(self) -> bool:
        """Start the camera."""
        if not self.detector.start():
            logger.error("Failed to start camera")
            return False
        self.running = True
        return True

    def stop(self):
        """Stop the camera."""
        self.running = False
        self.detector.stop()

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Optional[dict]]:
        """
        Process a single frame.

        Returns:
            Tuple of (display_frame, data_dict)
        """
        loop_start = time.time()

        # Detect markers
        detect_start = time.time()
        detections = self.detector.detect(frame)
        detect_time = time.time() - detect_start

        # Update stats
        self.stats.frames_total += 1
        self.stats.detection_times.append(detect_time)

        # Draw base detections
        display = self.detector.draw_detections(frame, detections)

        data = None

        if detections:
            self.stats.frames_with_detection += 1

            # Use first/closest marker
            detection = detections[0]

            # Get offset from marker
            fwd, right, alt, yaw = self.estimator.get_offset_from_marker(detection)

            # Calculate altitude error (assuming marker is on ceiling at 3m)
            ceiling_height = 3.0  # meters
            desired_distance = ceiling_height - self.target_altitude_m
            alt_error = alt - desired_distance

            # Compute velocity commands (what would be sent)
            vx = -self.pid_x.update(fwd)
            vy = -self.pid_y.update(right)
            vz = self.pid_z.update(alt_error)

            data = {
                'marker_id': detection.marker_id,
                'position': (fwd, right, alt),
                'yaw': yaw,
                'velocity_cmd': (vx, vy, vz),
                'alt_error': alt_error,
                'distance': detection.distance
            }

            # Draw position info panel
            self._draw_info_panel(display, data)
        else:
            # Reset PIDs when no detection
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()

            # Draw no detection message
            cv2.putText(
                display, "NO MARKER DETECTED", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
            )

        # Draw statistics bar
        self._draw_stats_bar(display)

        # Draw help if enabled
        if self.show_help:
            self._draw_help(display)

        loop_time = time.time() - loop_start
        self.stats.loop_times.append(loop_time)

        return display, data

    def _draw_info_panel(self, frame: np.ndarray, data: dict):
        """Draw position and command information panel."""
        h, w = frame.shape[:2]

        # Semi-transparent background
        overlay = frame.copy()
        panel_x, panel_y = 10, 50
        panel_w, panel_h = 300, 200
        cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        # Text settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (255, 255, 255)
        y = panel_y + 25
        line_h = 22

        # Marker ID
        cv2.putText(frame, f"Marker ID: {data['marker_id']}", (panel_x + 10, y), font, 0.6, (0, 255, 0), 2)
        y += line_h + 5

        # Position offset
        fwd, right, alt = data['position']
        cv2.putText(frame, "Position Offset:", (panel_x + 10, y), font, 0.5, (200, 200, 200), 1)
        y += line_h
        cv2.putText(frame, f"  Fwd:   {fwd:+.3f} m", (panel_x + 10, y), font, 0.5, color, 1)
        y += line_h
        cv2.putText(frame, f"  Right: {right:+.3f} m", (panel_x + 10, y), font, 0.5, color, 1)
        y += line_h
        cv2.putText(frame, f"  Alt:   {alt:.3f} m (err: {data['alt_error']:+.2f})", (panel_x + 10, y), font, 0.5, color, 1)
        y += line_h
        cv2.putText(frame, f"  Yaw:   {data['yaw']:+.1f} deg", (panel_x + 10, y), font, 0.5, color, 1)
        y += line_h + 5

        # Velocity commands
        vx, vy, vz = data['velocity_cmd']
        cv2.putText(frame, "Velocity Commands:", (panel_x + 10, y), font, 0.5, (200, 200, 200), 1)
        y += line_h

        # Color code based on magnitude
        def vel_color(v, limit=0.3):
            mag = abs(v) / limit
            if mag < 0.3:
                return (0, 255, 0)  # Green
            elif mag < 0.7:
                return (0, 255, 255)  # Yellow
            else:
                return (0, 128, 255)  # Orange

        cv2.putText(frame, f"  Vx: {vx:+.3f} m/s", (panel_x + 10, y), font, 0.5, vel_color(vx, 0.5), 1)
        y += line_h
        cv2.putText(frame, f"  Vy: {vy:+.3f} m/s", (panel_x + 10, y), font, 0.5, vel_color(vy, 0.5), 1)
        y += line_h
        cv2.putText(frame, f"  Vz: {vz:+.3f} m/s", (panel_x + 10, y), font, 0.5, vel_color(vz, 0.3), 1)

        # Draw velocity vector visualization
        self._draw_velocity_indicator(frame, vx, vy, vz, w - 100, 150)

    def _draw_velocity_indicator(self, frame: np.ndarray, vx: float, vy: float, vz: float, cx: int, cy: int):
        """Draw a visual velocity indicator."""
        radius = 60

        # Background circle
        cv2.circle(frame, (cx, cy), radius, (50, 50, 50), -1)
        cv2.circle(frame, (cx, cy), radius, (100, 100, 100), 2)

        # Cross hairs
        cv2.line(frame, (cx - radius, cy), (cx + radius, cy), (80, 80, 80), 1)
        cv2.line(frame, (cx, cy - radius), (cx, cy + radius), (80, 80, 80), 1)

        # Scale velocity to pixels (0.5 m/s = full radius)
        scale = radius / 0.5
        dx = int(vy * scale)  # Right = positive screen X
        dy = int(-vx * scale)  # Forward = negative screen Y (up)

        # Limit to circle
        mag = np.sqrt(dx*dx + dy*dy)
        if mag > radius:
            dx = int(dx * radius / mag)
            dy = int(dy * radius / mag)

        # Draw velocity vector
        end_x, end_y = cx + dx, cy + dy
        cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 0), 2, tipLength=0.3)

        # Draw vertical indicator bar
        bar_x = cx + radius + 20
        bar_h = 80
        bar_top = cy - bar_h // 2

        cv2.rectangle(frame, (bar_x, bar_top), (bar_x + 15, bar_top + bar_h), (50, 50, 50), -1)
        cv2.rectangle(frame, (bar_x, bar_top), (bar_x + 15, bar_top + bar_h), (100, 100, 100), 1)
        cv2.line(frame, (bar_x, cy), (bar_x + 15, cy), (80, 80, 80), 1)

        # Vz indicator (scaled: 0.3 m/s = full bar)
        vz_scale = bar_h / 2 / 0.3
        vz_px = int(-vz * vz_scale)  # Down is positive Vz, but we draw up as positive
        vz_px = np.clip(vz_px, -bar_h // 2, bar_h // 2)

        if vz_px != 0:
            if vz_px > 0:
                cv2.rectangle(frame, (bar_x + 2, cy), (bar_x + 13, cy - vz_px), (0, 255, 0), -1)
            else:
                cv2.rectangle(frame, (bar_x + 2, cy), (bar_x + 13, cy - vz_px), (0, 128, 255), -1)

        # Labels
        cv2.putText(frame, "XY", (cx - 10, cy + radius + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        cv2.putText(frame, "Z", (bar_x + 3, bar_top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

    def _draw_stats_bar(self, frame: np.ndarray):
        """Draw statistics bar at bottom."""
        h, w = frame.shape[:2]
        bar_h = 25

        # Background
        cv2.rectangle(frame, (0, h - bar_h), (w, h), (40, 40, 40), -1)

        # Stats text
        stats_text = (
            f"FPS: {self.stats.fps:.1f} | "
            f"Detection: {self.stats.avg_detection_ms:.1f}ms | "
            f"Rate: {self.stats.detection_rate:.1f}% | "
            f"Frames: {self.stats.frames_total}"
        )

        cv2.putText(
            frame, stats_text, (10, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1
        )

    def _draw_help(self, frame: np.ndarray):
        """Draw help text."""
        h, w = frame.shape[:2]
        help_text = "[Q]uit  [S]creenshot  [R]eset stats  [H]elp toggle"

        cv2.putText(
            frame, help_text, (w - 350, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1
        )

    def run(self):
        """Run the bench test loop."""
        if not self.start():
            return

        print("\n" + "=" * 60)
        print("          ArUco DRONE NAVIGATION - BENCH TEST")
        print("=" * 60)
        print(f"\nCamera: {self.camera_id}")
        print(f"Resolution: {self.resolution[0]}x{self.resolution[1]}")
        print(f"Marker size: {self.marker_size_m * 100:.1f} cm")
        print(f"Target altitude: {self.target_altitude_m} m")
        print(f"Calibration: {'Loaded' if self.camera_matrix is not None else 'Not loaded (using defaults)'}")
        print("\nControls: Q=Quit, S=Screenshot, R=Reset, H=Help toggle")
        print()

        screenshot_count = 0

        try:
            while self.running:
                frame = self.detector.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                display, data = self.process_frame(frame)

                cv2.imshow("ArUco Bench Test", display)

                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"bench_screenshot_{screenshot_count:03d}.png"
                    cv2.imwrite(filename, display)
                    print(f"Saved: {filename}")
                    screenshot_count += 1
                elif key == ord('r'):
                    self.stats = BenchStats()
                    print("Stats reset")
                elif key == ord('h'):
                    self.show_help = not self.show_help

        except KeyboardInterrupt:
            print("\nInterrupted")

        finally:
            self.stop()
            cv2.destroyAllWindows()

        # Print final stats
        print("\n" + "=" * 40)
        print("Final Statistics:")
        print(f"  Total frames: {self.stats.frames_total}")
        print(f"  Detections: {self.stats.frames_with_detection} ({self.stats.detection_rate:.1f}%)")
        print(f"  Avg FPS: {self.stats.fps:.1f}")
        print(f"  Avg detection time: {self.stats.avg_detection_ms:.2f} ms")
        print("=" * 40)


def main():
    parser = argparse.ArgumentParser(
        description="Bench test for ArUco drone navigation"
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
        default='DICT_6X6_250',
        choices=list(ARUCO_DICTIONARIES.keys()),
        help='ArUco dictionary (default: DICT_6X6_250)'
    )
    parser.add_argument(
        '--marker-size', '-s',
        type=float,
        default=0.20,
        help='Marker size in meters (default: 0.20)'
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
        '--altitude', '-a',
        type=float,
        default=1.5,
        help='Target hover altitude in meters (default: 1.5)'
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

    # Resolve calibration path
    calib_path = Path(args.calibration)
    if not calib_path.is_absolute():
        calib_path = Path(__file__).parent.parent / calib_path

    bench = BenchTest(
        camera_id=args.camera,
        calibration_path=str(calib_path) if calib_path.exists() else None,
        dictionary=args.dictionary,
        marker_size_m=args.marker_size,
        resolution=(args.width, args.height),
        target_altitude_m=args.altitude
    )

    bench.run()


if __name__ == "__main__":
    main()
