#!/usr/bin/env python3
"""
ArUco Debug GUI - Visual Navigation Debugger

Real-time visualization tool for ArUco marker detection and position estimation.
Connects to RPi camera server and displays:
- Live video with marker detection overlay
- Telemetry (position, distance, yaw)
- Marker map (top-down view with trajectory)
- Recording and playback

Usage:
    python debug_gui.py [--host 10.156.64.251] [--port 8000]
"""

import argparse
import json
import logging
import os
import sys
import threading
import time
import urllib.request
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Add parent directory for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.aruco_detector import ArucoDetector, MarkerDetection, ARUCO_DICTIONARIES
from src.position_estimator import SimplePositionEstimator
from src.camera_calibration import CameraCalibration

# Tkinter imports
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk

logger = logging.getLogger(__name__)


@dataclass
class TrajectoryPoint:
    """Single point in trajectory."""
    timestamp: float
    x: float
    y: float
    z: float
    yaw: float
    marker_ids: List[int] = field(default_factory=list)


@dataclass
class MarkerRecord:
    """Recorded marker information."""
    marker_id: int
    world_pos: Tuple[float, float, float]  # x, y, z in world frame
    first_seen: float
    last_seen: float
    detection_count: int = 0


class MJPEGStreamReader:
    """Read MJPEG stream from HTTP server."""

    def __init__(self, url: str):
        self.url = url
        self.frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.fps = 0.0
        self.connected = False
        self.error_msg = ""

    def start(self):
        """Start reading stream in background thread."""
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop reading stream."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)

    def _read_loop(self):
        """Main stream reading loop."""
        fps_times = deque(maxlen=30)

        while self.running:
            try:
                # Open stream
                stream = urllib.request.urlopen(self.url, timeout=5)
                self.connected = True
                self.error_msg = ""
                buffer = b''

                while self.running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break

                    buffer += chunk

                    # Find JPEG boundaries
                    start = buffer.find(b'\xff\xd8')
                    end = buffer.find(b'\xff\xd9')

                    if start != -1 and end != -1 and end > start:
                        jpg = buffer[start:end + 2]
                        buffer = buffer[end + 2:]

                        # Decode JPEG
                        img_array = np.frombuffer(jpg, dtype=np.uint8)
                        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                        if frame is not None:
                            with self.frame_lock:
                                self.frame = frame

                            # Calculate FPS
                            now = time.time()
                            fps_times.append(now)
                            if len(fps_times) > 1:
                                self.fps = len(fps_times) / (fps_times[-1] - fps_times[0])

            except Exception as e:
                self.connected = False
                self.error_msg = str(e)
                logger.warning(f"Stream error: {e}")
                time.sleep(1)  # Retry delay

    def get_frame(self) -> Optional[np.ndarray]:
        """Get latest frame."""
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None


class DebugGUI:
    """Main debug GUI application."""

    def __init__(self, host: str = "10.156.64.251", port: int = 8000,
                 marker_size: float = 0.20, dictionary: str = "DICT_6X6_250"):
        self.host = host
        self.port = port
        self.marker_size = marker_size
        self.dictionary = dictionary

        # Stream reader
        self.stream_url = f"http://{host}:{port}/stream"
        self.stream = MJPEGStreamReader(self.stream_url)

        # ArUco detector (runs locally on received frames)
        self.detector = ArucoDetector(
            camera_id=-1,  # No local camera
            dictionary=dictionary,
            marker_size_m=marker_size
        )

        # Position estimator
        self.estimator = SimplePositionEstimator()

        # Load calibration if available
        self.camera_matrix = None
        self.dist_coeffs = None
        calib_path = Path(__file__).parent.parent / "config" / "camera_params.yaml"
        if calib_path.exists():
            try:
                self.camera_matrix, self.dist_coeffs = CameraCalibration.load_calibration(str(calib_path))
                self.detector.camera_matrix = self.camera_matrix
                self.detector.dist_coeffs = self.dist_coeffs
                logger.info("Loaded camera calibration")
            except:
                logger.warning("Failed to load calibration")

        # Recording state
        self.recording = False
        self.record_start_time = 0.0
        self.trajectory: List[TrajectoryPoint] = []
        self.markers_discovered: Dict[int, MarkerRecord] = {}

        # Current detection state
        self.current_detections: List[MarkerDetection] = []
        self.current_position = (0.0, 0.0, 0.0, 0.0)  # fwd, right, alt, yaw

        # Statistics
        self.frame_count = 0
        self.detection_count = 0

        # GUI state
        self.running = True
        self.root: Optional[tk.Tk] = None

        # Map settings
        self.map_scale = 100  # pixels per meter
        self.map_center = (0.0, 0.0)  # world coordinates at center of map

    def run(self):
        """Run the GUI application."""
        # Start stream reader
        self.stream.start()

        # Create main window
        self.root = tk.Tk()
        self.root.title("ArUco Debug GUI")
        self.root.geometry("1200x800")
        self.root.configure(bg='#1a1a1a')

        # Create UI
        self._create_ui()

        # Start update loop
        self._update_loop()

        # Run main loop
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _create_ui(self):
        """Create the user interface."""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Configure style
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#1a1a1a')
        style.configure('TLabel', background='#1a1a1a', foreground='#ffffff')
        style.configure('TButton', background='#333333')
        style.configure('Header.TLabel', font=('Arial', 12, 'bold'), foreground='#4CAF50')

        # Top row: Video + Telemetry
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.BOTH, expand=True)

        # Video panel (left)
        video_frame = ttk.LabelFrame(top_frame, text=" Live Video ", padding=5)
        video_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        self.video_canvas = tk.Canvas(video_frame, width=640, height=480, bg='#000000', highlightthickness=0)
        self.video_canvas.pack()

        self.video_status = ttk.Label(video_frame, text="Connecting...")
        self.video_status.pack(pady=2)

        # Telemetry panel (right)
        telemetry_frame = ttk.LabelFrame(top_frame, text=" Telemetry ", padding=10)
        telemetry_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))

        # Connection status
        ttk.Label(telemetry_frame, text="CONNECTION", style='Header.TLabel').pack(anchor=tk.W)
        self.conn_label = ttk.Label(telemetry_frame, text="Disconnected", foreground='#ff5555')
        self.conn_label.pack(anchor=tk.W, pady=(0, 10))

        # Markers detected
        ttk.Label(telemetry_frame, text="MARKERS", style='Header.TLabel').pack(anchor=tk.W)
        self.markers_label = ttk.Label(telemetry_frame, text="None detected")
        self.markers_label.pack(anchor=tk.W, pady=(0, 10))

        # Position
        ttk.Label(telemetry_frame, text="POSITION", style='Header.TLabel').pack(anchor=tk.W)
        self.pos_forward = ttk.Label(telemetry_frame, text="Forward:  ---")
        self.pos_forward.pack(anchor=tk.W)
        self.pos_right = ttk.Label(telemetry_frame, text="Right:    ---")
        self.pos_right.pack(anchor=tk.W)
        self.pos_alt = ttk.Label(telemetry_frame, text="Altitude: ---")
        self.pos_alt.pack(anchor=tk.W)
        self.pos_yaw = ttk.Label(telemetry_frame, text="Yaw:      ---")
        self.pos_yaw.pack(anchor=tk.W, pady=(0, 10))

        # Statistics
        ttk.Label(telemetry_frame, text="STATISTICS", style='Header.TLabel').pack(anchor=tk.W)
        self.stats_fps = ttk.Label(telemetry_frame, text="FPS: ---")
        self.stats_fps.pack(anchor=tk.W)
        self.stats_rate = ttk.Label(telemetry_frame, text="Detection: ---")
        self.stats_rate.pack(anchor=tk.W)
        self.stats_frames = ttk.Label(telemetry_frame, text="Frames: 0")
        self.stats_frames.pack(anchor=tk.W, pady=(0, 10))

        # Recording controls
        ttk.Label(telemetry_frame, text="RECORDING", style='Header.TLabel').pack(anchor=tk.W)
        self.record_btn = ttk.Button(telemetry_frame, text="Start Recording", command=self._toggle_recording)
        self.record_btn.pack(fill=tk.X, pady=2)
        self.record_status = ttk.Label(telemetry_frame, text="Not recording")
        self.record_status.pack(anchor=tk.W)

        ttk.Button(telemetry_frame, text="Save Map", command=self._save_map).pack(fill=tk.X, pady=2)
        ttk.Button(telemetry_frame, text="Load Map", command=self._load_map).pack(fill=tk.X, pady=2)

        # Bottom: Marker map
        map_frame = ttk.LabelFrame(main_frame, text=" Marker Map (Top-Down View) ", padding=5)
        map_frame.pack(fill=tk.BOTH, expand=True, pady=(5, 0))

        self.map_canvas = tk.Canvas(map_frame, height=200, bg='#0a0a0a', highlightthickness=1, highlightbackground='#333')
        self.map_canvas.pack(fill=tk.BOTH, expand=True)

        # Map controls
        map_controls = ttk.Frame(map_frame)
        map_controls.pack(fill=tk.X)

        ttk.Label(map_controls, text="Scale:").pack(side=tk.LEFT)
        self.scale_var = tk.StringVar(value="100")
        scale_spin = ttk.Spinbox(map_controls, from_=20, to=500, width=5, textvariable=self.scale_var,
                                  command=self._update_scale)
        scale_spin.pack(side=tk.LEFT, padx=5)
        ttk.Label(map_controls, text="px/m").pack(side=tk.LEFT)

        ttk.Button(map_controls, text="Clear Trajectory", command=self._clear_trajectory).pack(side=tk.RIGHT, padx=5)
        ttk.Button(map_controls, text="Center View", command=self._center_view).pack(side=tk.RIGHT, padx=5)

    def _update_loop(self):
        """Main update loop (called from Tkinter)."""
        if not self.running:
            return

        # Get frame from stream
        frame = self.stream.get_frame()

        if frame is not None:
            self.frame_count += 1

            # Run ArUco detection
            detections = self.detector.detect(frame)
            self.current_detections = detections

            if detections:
                self.detection_count += 1

                # Get position from first marker
                det = detections[0]
                fwd, right, alt, yaw = self.estimator.get_offset_from_marker(det)
                self.current_position = (fwd, right, alt, yaw)

                # Record if recording
                if self.recording:
                    self._record_point(detections)

            # Draw detections on frame
            display_frame = self.detector.draw_detections(frame, detections)

            # Add position overlay
            if detections:
                fwd, right, alt, yaw = self.current_position
                cv2.putText(display_frame, f"Fwd:{fwd:+.2f}m Right:{right:+.2f}m Alt:{alt:.2f}m Yaw:{yaw:+.1f}deg",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Convert to Tkinter image
            display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(display_frame)
            img = img.resize((640, 480), Image.Resampling.LANCZOS)
            self.photo = ImageTk.PhotoImage(img)

            self.video_canvas.delete("all")
            self.video_canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)

        # Update connection status
        if self.stream.connected:
            self.conn_label.configure(text=f"Connected to {self.host}", foreground='#55ff55')
            self.video_status.configure(text=f"FPS: {self.stream.fps:.1f}")
        else:
            self.conn_label.configure(text=f"Disconnected: {self.stream.error_msg[:30]}", foreground='#ff5555')
            self.video_status.configure(text="Reconnecting...")

        # Update telemetry
        self._update_telemetry()

        # Update map
        self._update_map()

        # Schedule next update
        self.root.after(33, self._update_loop)  # ~30 FPS

    def _update_telemetry(self):
        """Update telemetry panel."""
        # Markers
        if self.current_detections:
            ids = [d.marker_id for d in self.current_detections]
            dists = [f"{d.distance:.2f}m" for d in self.current_detections]
            self.markers_label.configure(text=f"IDs: {ids}\nDist: {dists}")
        else:
            self.markers_label.configure(text="None detected")

        # Position
        if self.current_detections:
            fwd, right, alt, yaw = self.current_position
            self.pos_forward.configure(text=f"Forward:  {fwd:+.3f} m")
            self.pos_right.configure(text=f"Right:    {right:+.3f} m")
            self.pos_alt.configure(text=f"Altitude: {alt:.3f} m")
            self.pos_yaw.configure(text=f"Yaw:      {yaw:+.1f} deg")
        else:
            self.pos_forward.configure(text="Forward:  ---")
            self.pos_right.configure(text="Right:    ---")
            self.pos_alt.configure(text="Altitude: ---")
            self.pos_yaw.configure(text="Yaw:      ---")

        # Statistics
        self.stats_fps.configure(text=f"FPS: {self.stream.fps:.1f}")
        rate = (self.detection_count / self.frame_count * 100) if self.frame_count > 0 else 0
        self.stats_rate.configure(text=f"Detection: {rate:.1f}%")
        self.stats_frames.configure(text=f"Frames: {self.frame_count}")

        # Recording status
        if self.recording:
            duration = time.time() - self.record_start_time
            self.record_status.configure(text=f"Recording: {duration:.1f}s, {len(self.trajectory)} pts")

    def _update_map(self):
        """Update marker map visualization."""
        canvas = self.map_canvas
        canvas.delete("all")

        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 10 or h < 10:
            return

        cx, cy = w // 2, h // 2
        scale = self.map_scale

        # Draw grid
        grid_spacing = scale  # 1 meter
        for i in range(-10, 11):
            x = cx + i * grid_spacing
            y = cy + i * grid_spacing
            if 0 <= x <= w:
                canvas.create_line(x, 0, x, h, fill='#222222')
            if 0 <= y <= h:
                canvas.create_line(0, y, w, y, fill='#222222')

        # Draw axes
        canvas.create_line(cx, 0, cx, h, fill='#444444', width=2)  # Y axis
        canvas.create_line(0, cy, w, cy, fill='#444444', width=2)  # X axis
        canvas.create_text(cx + 10, 10, text="Forward", fill='#666666', anchor=tk.NW)
        canvas.create_text(w - 50, cy + 10, text="Right", fill='#666666', anchor=tk.NW)

        # Draw trajectory
        if len(self.trajectory) > 1:
            points = []
            for pt in self.trajectory:
                px = cx + int(pt.y * scale)  # right -> screen X
                py = cy - int(pt.x * scale)  # forward -> screen -Y
                points.extend([px, py])
            if len(points) >= 4:
                canvas.create_line(points, fill='#4CAF50', width=2, smooth=True)

        # Draw discovered markers
        for marker_id, marker in self.markers_discovered.items():
            mx = cx + int(marker.world_pos[1] * scale)
            my = cy - int(marker.world_pos[0] * scale)
            # Draw marker square
            size = 15
            canvas.create_rectangle(mx - size, my - size, mx + size, my + size,
                                   fill='#2196F3', outline='#64B5F6', width=2)
            canvas.create_text(mx, my, text=str(marker_id), fill='white', font=('Arial', 10, 'bold'))

        # Draw current position (camera/drone icon)
        if self.current_detections:
            fwd, right, alt, yaw = self.current_position
            px = cx + int(right * scale)
            py = cy - int(fwd * scale)

            # Draw camera as triangle pointing in yaw direction
            yaw_rad = np.radians(-yaw)  # Convert to standard math angle
            size = 12
            pts = [
                (px + size * np.sin(yaw_rad), py - size * np.cos(yaw_rad)),
                (px + size * np.sin(yaw_rad + 2.5), py - size * np.cos(yaw_rad + 2.5)),
                (px + size * np.sin(yaw_rad - 2.5), py - size * np.cos(yaw_rad - 2.5)),
            ]
            canvas.create_polygon(pts, fill='#FF5722', outline='#FFAB91', width=2)

        # Scale indicator
        canvas.create_line(10, h - 20, 10 + scale, h - 20, fill='#888888', width=2)
        canvas.create_text(10 + scale // 2, h - 30, text="1m", fill='#888888')

    def _record_point(self, detections: List[MarkerDetection]):
        """Record current position to trajectory."""
        now = time.time()
        fwd, right, alt, yaw = self.current_position

        # Add trajectory point
        pt = TrajectoryPoint(
            timestamp=now - self.record_start_time,
            x=fwd, y=right, z=alt, yaw=yaw,
            marker_ids=[d.marker_id for d in detections]
        )
        self.trajectory.append(pt)

        # Update discovered markers
        for det in detections:
            if det.marker_id not in self.markers_discovered:
                # First time seeing this marker
                self.markers_discovered[det.marker_id] = MarkerRecord(
                    marker_id=det.marker_id,
                    world_pos=(fwd, right, alt),  # Approximate world pos from current detection
                    first_seen=pt.timestamp,
                    last_seen=pt.timestamp,
                    detection_count=1
                )
            else:
                # Update existing marker
                marker = self.markers_discovered[det.marker_id]
                marker.last_seen = pt.timestamp
                marker.detection_count += 1

    def _toggle_recording(self):
        """Toggle recording state."""
        if self.recording:
            self.recording = False
            self.record_btn.configure(text="Start Recording")
            self.record_status.configure(text=f"Stopped: {len(self.trajectory)} points")
        else:
            self.recording = True
            self.record_start_time = time.time()
            self.record_btn.configure(text="Stop Recording")
            self.record_status.configure(text="Recording...")

    def _save_map(self):
        """Save recorded data to file."""
        if not self.trajectory and not self.markers_discovered:
            messagebox.showwarning("Nothing to Save", "No data recorded yet.")
            return

        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"session_{timestamp}.json"

        filepath = filedialog.asksaveasfilename(
            initialfile=default_name,
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )

        if not filepath:
            return

        # Build save data
        data = {
            "metadata": {
                "start_time": datetime.now().isoformat(),
                "duration_sec": len(self.trajectory) * 0.033 if self.trajectory else 0,
                "total_points": len(self.trajectory),
                "markers_found": len(self.markers_discovered),
                "host": self.host,
                "marker_size": self.marker_size
            },
            "markers_discovered": {
                str(m.marker_id): {
                    "world_pos": list(m.world_pos),
                    "first_seen": m.first_seen,
                    "last_seen": m.last_seen,
                    "detection_count": m.detection_count
                }
                for m in self.markers_discovered.values()
            },
            "trajectory": [
                {
                    "t": pt.timestamp,
                    "x": pt.x, "y": pt.y, "z": pt.z,
                    "yaw": pt.yaw,
                    "markers": pt.marker_ids
                }
                for pt in self.trajectory
            ]
        }

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        messagebox.showinfo("Saved", f"Saved to {filepath}")

    def _load_map(self):
        """Load marker map from file."""
        filepath = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("YAML files", "*.yaml"), ("All files", "*.*")]
        )

        if not filepath:
            return

        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            # Load markers
            if "markers_discovered" in data:
                self.markers_discovered.clear()
                for mid, mdata in data["markers_discovered"].items():
                    self.markers_discovered[int(mid)] = MarkerRecord(
                        marker_id=int(mid),
                        world_pos=tuple(mdata["world_pos"]),
                        first_seen=mdata.get("first_seen", 0),
                        last_seen=mdata.get("last_seen", 0),
                        detection_count=mdata.get("detection_count", 0)
                    )

            # Load trajectory
            if "trajectory" in data:
                self.trajectory.clear()
                for pt in data["trajectory"]:
                    self.trajectory.append(TrajectoryPoint(
                        timestamp=pt["t"],
                        x=pt["x"], y=pt["y"], z=pt["z"],
                        yaw=pt["yaw"],
                        marker_ids=pt.get("markers", [])
                    ))

            messagebox.showinfo("Loaded", f"Loaded {len(self.markers_discovered)} markers, {len(self.trajectory)} trajectory points")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to load: {e}")

    def _update_scale(self):
        """Update map scale from spinbox."""
        try:
            self.map_scale = int(self.scale_var.get())
        except:
            pass

    def _clear_trajectory(self):
        """Clear recorded trajectory."""
        self.trajectory.clear()

    def _center_view(self):
        """Center map view on current position."""
        if self.current_detections:
            fwd, right, _, _ = self.current_position
            self.map_center = (fwd, right)

    def _on_close(self):
        """Handle window close."""
        self.running = False
        self.stream.stop()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="ArUco Debug GUI")
    parser.add_argument('--host', '-H', default='10.156.64.251', help='RPi camera server host')
    parser.add_argument('--port', '-p', type=int, default=8000, help='RPi camera server port')
    parser.add_argument('--marker-size', '-s', type=float, default=0.20, help='Marker size in meters')
    parser.add_argument('--dictionary', '-d', default='DICT_6X6_250', choices=list(ARUCO_DICTIONARIES.keys()))
    parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )

    gui = DebugGUI(
        host=args.host,
        port=args.port,
        marker_size=args.marker_size,
        dictionary=args.dictionary
    )
    gui.run()


if __name__ == '__main__':
    main()
