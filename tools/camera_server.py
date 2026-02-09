#!/usr/bin/env python3
"""
MJPEG Camera Streaming Server for RPi

Streams camera frames over HTTP for remote viewing and processing.
Designed to run on RPi Zero 2W with minimal overhead.

Usage:
    python camera_server.py [--port 8000] [--camera 0] [--width 640] [--height 480]

Access stream at:
    http://<rpi-ip>:8000/stream  - MJPEG video stream
    http://<rpi-ip>:8000/frame   - Single JPEG frame
    http://<rpi-ip>:8000/status  - JSON status info
"""

import argparse
import json
import logging
import socket
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class CameraCapture:
    """Thread-safe camera capture with frame buffering."""

    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480,
                 fps: int = 15, jpeg_quality: int = 80):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality

        self.cap: Optional[cv2.VideoCapture] = None
        self.frame: Optional[bytes] = None
        self.frame_lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None

        # Statistics
        self.frames_captured = 0
        self.last_frame_time = 0
        self.actual_fps = 0.0
        self.start_time = 0

    def start(self) -> bool:
        """Start the camera capture thread."""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            logger.error(f"Failed to open camera {self.camera_id}")
            return False

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Get actual settings
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(f"Camera opened: {actual_w}x{actual_h}")

        self.running = True
        self.start_time = time.time()
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        """Stop the camera capture."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()

    def _capture_loop(self):
        """Main capture loop running in separate thread."""
        fps_window = []

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # Encode as JPEG
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            _, jpeg = cv2.imencode('.jpg', frame, encode_param)

            # Update frame buffer
            with self.frame_lock:
                self.frame = jpeg.tobytes()
                self.frames_captured += 1

                # Calculate FPS
                now = time.time()
                fps_window.append(now)
                # Keep last 30 frame timestamps
                fps_window = [t for t in fps_window if now - t < 1.0]
                self.actual_fps = len(fps_window)
                self.last_frame_time = now

    def get_frame(self) -> Optional[bytes]:
        """Get the latest JPEG frame."""
        with self.frame_lock:
            return self.frame

    def get_status(self) -> dict:
        """Get camera status."""
        with self.frame_lock:
            return {
                'camera_id': self.camera_id,
                'resolution': f'{self.width}x{self.height}',
                'target_fps': self.fps,
                'actual_fps': round(self.actual_fps, 1),
                'frames_captured': self.frames_captured,
                'uptime_sec': round(time.time() - self.start_time, 1),
                'jpeg_quality': self.jpeg_quality,
                'running': self.running
            }


class StreamingHandler(BaseHTTPRequestHandler):
    """HTTP request handler for camera streaming."""

    camera: CameraCapture = None  # Set by server

    def log_message(self, format, *args):
        """Override to use our logger."""
        logger.debug(f"{self.address_string()} - {format % args}")

    def do_GET(self):
        if self.path == '/stream':
            self._handle_stream()
        elif self.path == '/frame':
            self._handle_frame()
        elif self.path == '/status':
            self._handle_status()
        elif self.path == '/':
            self._handle_index()
        else:
            self.send_error(404)

    def _handle_index(self):
        """Serve simple HTML page with stream embed."""
        html = '''<!DOCTYPE html>
<html>
<head>
    <title>ArUco Camera Stream</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #1a1a1a; color: #fff; }
        h1 { color: #4CAF50; }
        img { border: 2px solid #4CAF50; max-width: 100%; }
        .status { background: #2a2a2a; padding: 10px; border-radius: 5px; margin-top: 10px; }
        pre { color: #8BC34A; }
    </style>
</head>
<body>
    <h1>ArUco Drone Navigation - Camera Stream</h1>
    <img src="/stream" alt="Camera Stream">
    <div class="status">
        <h3>Status: <a href="/status" style="color:#4CAF50">/status</a></h3>
        <pre id="status"></pre>
    </div>
    <script>
        function updateStatus() {
            fetch('/status').then(r => r.json()).then(data => {
                document.getElementById('status').textContent = JSON.stringify(data, null, 2);
            });
        }
        updateStatus();
        setInterval(updateStatus, 2000);
    </script>
</body>
</html>'''
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.send_header('Content-Length', len(html))
        self.end_headers()
        self.wfile.write(html.encode())

    def _handle_stream(self):
        """Handle MJPEG stream request."""
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        try:
            while True:
                frame = self.camera.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                # Send frame
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')

                # Limit frame rate to reduce CPU
                time.sleep(0.033)  # ~30 FPS max

        except (BrokenPipeError, ConnectionResetError):
            pass  # Client disconnected

    def _handle_frame(self):
        """Handle single frame request."""
        frame = self.camera.get_frame()
        if frame is None:
            self.send_error(503, 'No frame available')
            return

        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', len(frame))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(frame)

    def _handle_status(self):
        """Handle status request."""
        status = self.camera.get_status()
        data = json.dumps(status, indent=2)

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(data))
        self.end_headers()
        self.wfile.write(data.encode())


class CameraServer:
    """MJPEG camera streaming server."""

    def __init__(self, port: int = 8000, camera_id: int = 0,
                 width: int = 640, height: int = 480, fps: int = 15,
                 jpeg_quality: int = 80):
        self.port = port
        self.camera = CameraCapture(camera_id, width, height, fps, jpeg_quality)
        self.server: Optional[HTTPServer] = None

    def start(self):
        """Start the camera and HTTP server."""
        if not self.camera.start():
            raise RuntimeError("Failed to start camera")

        # Set camera reference for handler
        StreamingHandler.camera = self.camera

        # Create server
        self.server = HTTPServer(('0.0.0.0', self.port), StreamingHandler)

        # Get local IP
        hostname = socket.gethostname()
        try:
            local_ip = socket.gethostbyname(hostname)
        except:
            local_ip = '0.0.0.0'

        print("\n" + "=" * 60)
        print("  ArUco Camera Server")
        print("=" * 60)
        print(f"\nServer running on port {self.port}")
        print(f"\nAccess URLs:")
        print(f"  Stream:  http://{local_ip}:{self.port}/stream")
        print(f"  Frame:   http://{local_ip}:{self.port}/frame")
        print(f"  Status:  http://{local_ip}:{self.port}/status")
        print(f"  Web UI:  http://{local_ip}:{self.port}/")
        print(f"\nPress Ctrl+C to stop\n")

        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()

    def stop(self):
        """Stop the server and camera."""
        if self.server:
            self.server.shutdown()
        self.camera.stop()


def main():
    parser = argparse.ArgumentParser(
        description='MJPEG Camera Streaming Server'
    )
    parser.add_argument(
        '--port', '-p',
        type=int,
        default=8000,
        help='HTTP server port (default: 8000)'
    )
    parser.add_argument(
        '--camera', '-c',
        type=int,
        default=0,
        help='Camera device ID (default: 0)'
    )
    parser.add_argument(
        '--width', '-W',
        type=int,
        default=2592,
        help='Frame width (default: 2592 for OV5693 5MP)'
    )
    parser.add_argument(
        '--height', '-H',
        type=int,
        default=1944,
        help='Frame height (default: 1944 for OV5693 5MP)'
    )
    parser.add_argument(
        '--fps', '-f',
        type=int,
        default=15,
        help='Target FPS (default: 15)'
    )
    parser.add_argument(
        '--quality', '-q',
        type=int,
        default=80,
        help='JPEG quality 0-100 (default: 80)'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Verbose logging'
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )

    server = CameraServer(
        port=args.port,
        camera_id=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        jpeg_quality=args.quality
    )
    server.start()


if __name__ == '__main__':
    main()
