#!/usr/bin/env python3
"""
Position Viewer - Visualizes drone position from RPi Vision GPS stream.

Connects to the HTTP endpoint on the RPi and displays a top-down map
with drone position, yaw, and position history trail.

Usage:
    python3 tools/position_viewer.py --host aruconav.local --port 8001
"""

import argparse
import time
import json
import threading
import yaml
import numpy as np
from pathlib import Path
from typing import Optional, List, Tuple
from collections import deque
from urllib.request import urlopen
from urllib.error import URLError

import cv2


class PositionViewer:
    """
    OpenCV-based position visualizer that polls RPi for position data.
    """

    def __init__(
        self,
        host: str,
        port: int = 8001,
        marker_map_path: Optional[str] = None,
        poll_rate: float = 20.0
    ):
        self.host = host
        self.port = port
        self.url = f"http://{host}:{port}/position"
        self.poll_rate = poll_rate

        # Latest position data
        self.latest_data: Optional[dict] = None
        self.data_lock = threading.Lock()
        self._running = False

        # Position history trail (max 500 points)
        self.position_history: deque = deque(maxlen=500)

        # Marker map
        self.markers: List[dict] = []
        self.ceiling_height = 3.0
        if marker_map_path:
            self._load_marker_map(marker_map_path)

        # Visualization settings
        self.window_size = (900, 900)
        self.world_bounds = (-2.0, 2.0, -2.0, 2.0)  # xmin, xmax, ymin, ymax
        self.background_color = (20, 20, 20)
        self.grid_color = (50, 50, 50)
        self.marker_color = (80, 80, 200)  # Red-ish
        self.drone_color = (0, 220, 0)  # Green
        self.trail_color = (0, 180, 0)  # Darker green
        self.text_color = (200, 200, 200)
        self.warning_color = (0, 140, 255)  # Orange
        self.good_color = (0, 200, 100)  # Green

        # Connection state
        self.connected = False
        self.last_update_time = 0.0
        self.last_position_time = 0.0
        self.connection_error = ""
        self.poll_count = 0
        self.poll_success = 0

    def _load_marker_map(self, filepath: str):
        """Load marker positions from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            self.ceiling_height = data.get('ceiling_height', 3.0)

            # Load diamond markers
            for diamond in data.get('diamonds', []):
                self.markers.append({
                    'id': diamond['id'],
                    'position': diamond['position'],
                    'orientation': diamond.get('orientation', 0),
                    'description': diamond.get('description', '')
                })

            # Also load legacy markers
            for marker in data.get('markers', []):
                self.markers.append({
                    'id': str(marker['id']),
                    'position': marker['position'],
                    'orientation': marker.get('orientation', 0),
                    'description': marker.get('description', '')
                })

            print(f"Loaded {len(self.markers)} markers from {filepath}")

            # Auto-calculate world bounds from markers
            if self.markers:
                xs = [m['position'][0] for m in self.markers]
                ys = [m['position'][1] for m in self.markers]
                margin = 2.0
                self.world_bounds = (
                    min(xs) - margin, max(xs) + margin,
                    min(ys) - margin, max(ys) + margin
                )

        except Exception as e:
            print(f"Failed to load marker map: {e}")

    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen pixel coordinates."""
        xmin, xmax, ymin, ymax = self.world_bounds
        w, h = self.window_size

        # Normalize to 0-1
        nx = (x - xmin) / (xmax - xmin)
        ny = (y - ymin) / (ymax - ymin)

        # Convert to pixels (flip Y for screen coordinates)
        px = int(nx * w)
        py = int((1 - ny) * h)

        return (px, py)

    def _poll_position(self):
        """Background thread to poll position from RPi."""
        while self._running:
            self.poll_count += 1
            try:
                response = urlopen(self.url, timeout=0.5)
                data = json.loads(response.read().decode())

                with self.data_lock:
                    self.latest_data = data
                    self.connected = True
                    self.last_update_time = time.time()
                    self.connection_error = ""
                    self.poll_success += 1

                    # Add to history if we have valid position
                    if data.get('x') is not None:
                        self.last_position_time = time.time()
                        # Only add if position changed significantly
                        if len(self.position_history) == 0:
                            self.position_history.append((data['x'], data['y'], time.time()))
                        else:
                            last_x, last_y, _ = self.position_history[-1]
                            dist = ((data['x'] - last_x)**2 + (data['y'] - last_y)**2)**0.5
                            if dist > 0.01:  # Only add if moved > 1cm
                                self.position_history.append((data['x'], data['y'], time.time()))

            except URLError as e:
                with self.data_lock:
                    self.connected = False
                    self.connection_error = f"Connection failed"

            except Exception as e:
                with self.data_lock:
                    self.connected = False
                    self.connection_error = str(e)[:30]

            time.sleep(1.0 / self.poll_rate)

    def _draw_grid(self, frame: np.ndarray):
        """Draw coordinate grid."""
        xmin, xmax, ymin, ymax = self.world_bounds

        # Draw grid lines every 0.5m
        for x in np.arange(int(xmin) - 1, int(xmax) + 2, 0.5):
            p1 = self.world_to_screen(x, ymin)
            p2 = self.world_to_screen(x, ymax)
            thickness = 2 if abs(x - round(x)) < 0.01 else 1
            cv2.line(frame, p1, p2, self.grid_color, thickness)

        for y in np.arange(int(ymin) - 1, int(ymax) + 2, 0.5):
            p1 = self.world_to_screen(xmin, y)
            p2 = self.world_to_screen(xmax, y)
            thickness = 2 if abs(y - round(y)) < 0.01 else 1
            cv2.line(frame, p1, p2, self.grid_color, thickness)

        # Draw origin axes (brighter)
        origin = self.world_to_screen(0, 0)
        cv2.line(frame, self.world_to_screen(xmin, 0), self.world_to_screen(xmax, 0), (100, 100, 100), 2)
        cv2.line(frame, self.world_to_screen(0, ymin), self.world_to_screen(0, ymax), (100, 100, 100), 2)

        # Origin marker
        cv2.circle(frame, origin, 5, (150, 150, 150), -1)

        # Label axes
        cv2.putText(frame, "+X East", (self.window_size[0] - 80, origin[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.text_color, 1)
        cv2.putText(frame, "+Y North", (origin[0] + 10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.text_color, 1)

    def _draw_markers(self, frame: np.ndarray):
        """Draw marker positions on the map."""
        for marker in self.markers:
            x, y, z = marker['position']
            pos = self.world_to_screen(x, y)

            # Draw marker as diamond shape
            size = 20
            pts = np.array([
                [pos[0], pos[1] - size],
                [pos[0] + size, pos[1]],
                [pos[0], pos[1] + size],
                [pos[0] - size, pos[1]]
            ], np.int32)
            cv2.fillPoly(frame, [pts], (40, 40, 80))
            cv2.polylines(frame, [pts], True, self.marker_color, 2)

            # Label
            label = marker['id']
            cv2.putText(frame, label, (pos[0] - 30, pos[1] + size + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.marker_color, 1)

    def _draw_drone(self, frame: np.ndarray, data: dict):
        """Draw drone position with yaw arrow."""
        x, y = data['x'], data['y']
        yaw = data.get('yaw', 0)

        pos = self.world_to_screen(x, y)

        # Draw position trail
        trail_points = list(self.position_history)
        if len(trail_points) >= 2:
            for i in range(len(trail_points) - 1):
                tx1, ty1, _ = trail_points[i]
                tx2, ty2, _ = trail_points[i + 1]
                p1 = self.world_to_screen(tx1, ty1)
                p2 = self.world_to_screen(tx2, ty2)
                # Fade from dark to bright
                alpha = (i + 1) / len(trail_points)
                color = tuple(int(c * (0.3 + 0.7 * alpha)) for c in self.trail_color)
                cv2.line(frame, p1, p2, color, 2)

        # Draw drone body (filled circle with border)
        cv2.circle(frame, pos, 15, self.drone_color, -1)
        cv2.circle(frame, pos, 15, (255, 255, 255), 2)

        # Draw yaw arrow
        arrow_len = 35
        yaw_rad = np.radians(yaw)
        # yaw=0 is North (+Y), positive is clockwise
        dx = arrow_len * np.sin(yaw_rad)
        dy = -arrow_len * np.cos(yaw_rad)  # Negative because screen Y is inverted
        arrow_end = (int(pos[0] + dx), int(pos[1] + dy))
        cv2.arrowedLine(frame, pos, arrow_end, (255, 255, 255), 3, tipLength=0.35)

        # Draw position text near drone
        pos_text = f"({x:.2f}, {y:.2f})"
        cv2.putText(frame, pos_text, (pos[0] + 20, pos[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def _draw_stats(self, frame: np.ndarray, data: Optional[dict]):
        """Draw stats panel."""
        # Stats panel background
        cv2.rectangle(frame, (5, 5), (250, 220), (40, 40, 40), -1)
        cv2.rectangle(frame, (5, 5), (250, 220), (80, 80, 80), 1)

        y_offset = 25
        line_height = 22

        def draw_text(text: str, color=None):
            nonlocal y_offset
            cv2.putText(frame, text, (15, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color or self.text_color, 1)
            y_offset += line_height

        # Connection status
        if self.connected:
            draw_text(f"CONNECTED", self.good_color)
        else:
            draw_text(f"DISCONNECTED", self.warning_color)
            if self.connection_error:
                draw_text(f"  {self.connection_error}", self.warning_color)
            return

        if data is None:
            draw_text("Waiting for data...", self.warning_color)
            return

        # Position (large)
        if data.get('x') is not None:
            z = data.get('z', 0)
            draw_text(f"Alt: {z:.2f}m", self.good_color)
            yaw = data.get('yaw', 0)
            draw_text(f"Yaw: {yaw:.1f} deg")
        else:
            draw_text("NO POSITION FIX", self.warning_color)

        # Marker info
        markers = data.get('marker_ids', [])
        if markers:
            draw_text(f"Marker: {markers[0]}")
        else:
            draw_text("Marker: none", self.warning_color)

        # Detection stats
        det_rate = data.get('detection_rate', 0)
        color = self.good_color if det_rate > 0.3 else self.warning_color
        draw_text(f"Detection: {det_rate:.0%}", color)

        # Confidence
        conf = data.get('confidence', 0)
        draw_text(f"Confidence: {conf:.2f}")

        # Data freshness
        age = time.time() - self.last_position_time if self.last_position_time else 999
        if age < 1.0:
            draw_text(f"Data: LIVE", self.good_color)
        elif age < 5.0:
            draw_text(f"Data: {age:.1f}s ago", self.warning_color)
        else:
            draw_text(f"Data: STALE ({age:.0f}s)", self.warning_color)

    def _draw_help(self, frame: np.ndarray):
        """Draw help text at bottom."""
        h = self.window_size[1]
        cv2.putText(frame, "Q=quit  C=clear trail  +/-=zoom", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

    def run(self):
        """Main visualization loop."""
        self._running = True

        # Start polling thread
        poll_thread = threading.Thread(target=self._poll_position, daemon=True)
        poll_thread.start()

        print(f"Position Viewer started")
        print(f"  Connecting to: {self.url}")
        print(f"  Press 'q' to quit, 'c' to clear trail, +/- to zoom")

        cv2.namedWindow('Position Viewer', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Position Viewer', *self.window_size)

        while self._running:
            # Create frame
            frame = np.full((*self.window_size[::-1], 3), self.background_color, dtype=np.uint8)

            # Draw elements
            self._draw_grid(frame)
            self._draw_markers(frame)

            # Get latest data
            with self.data_lock:
                data = self.latest_data.copy() if self.latest_data else None

            if data and data.get('x') is not None:
                self._draw_drone(frame, data)

            self._draw_stats(frame, data)
            self._draw_help(frame)

            # Show frame
            cv2.imshow('Position Viewer', frame)

            # Handle keys
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                with self.data_lock:
                    self.position_history.clear()
                print("Trail cleared")
            elif key == ord('+') or key == ord('='):
                # Zoom in
                xmin, xmax, ymin, ymax = self.world_bounds
                cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
                w, h = (xmax - xmin) * 0.8, (ymax - ymin) * 0.8
                self.world_bounds = (cx - w/2, cx + w/2, cy - h/2, cy + h/2)
                print(f"Zoomed in: {self.world_bounds}")
            elif key == ord('-'):
                # Zoom out
                xmin, xmax, ymin, ymax = self.world_bounds
                cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
                w, h = (xmax - xmin) * 1.25, (ymax - ymin) * 1.25
                self.world_bounds = (cx - w/2, cx + w/2, cy - h/2, cy + h/2)
                print(f"Zoomed out: {self.world_bounds}")

        self._running = False
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Position Viewer - visualize drone position from RPi stream"
    )
    parser.add_argument(
        '--host',
        type=str,
        default='aruconav.local',
        help='RPi hostname or IP (default: aruconav.local)'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=8001,
        help='HTTP server port (default: 8001)'
    )
    parser.add_argument(
        '--marker-map',
        type=str,
        default=None,
        help='Path to marker_map.yaml (default: config/marker_map.yaml)'
    )
    parser.add_argument(
        '--poll-rate',
        type=float,
        default=30.0,
        help='Position poll rate in Hz (default: 30)'
    )

    args = parser.parse_args()

    # Find marker map
    marker_map_path = args.marker_map
    if marker_map_path is None:
        # Try default location
        script_dir = Path(__file__).parent.parent
        default_path = script_dir / 'config' / 'marker_map.yaml'
        if default_path.exists():
            marker_map_path = str(default_path)

    viewer = PositionViewer(
        host=args.host,
        port=args.port,
        marker_map_path=marker_map_path,
        poll_rate=args.poll_rate
    )

    try:
        viewer.run()
    except KeyboardInterrupt:
        print("\nStopped by user")


if __name__ == "__main__":
    main()
