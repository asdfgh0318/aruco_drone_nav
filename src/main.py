"""
ArUco Drone Navigation - Vision GPS Emulator

Detects ceiling-mounted ArUco markers, calculates world-frame position,
and sends it to the flight controller as vision position estimates.
The FC handles all navigation, PID control, missions, and failsafes.
"""

import time
import signal
import sys
import logging
import subprocess
import yaml
import argparse
import json
import threading
import numpy as np
import cv2
from pathlib import Path
from typing import Optional
from http.server import HTTPServer, BaseHTTPRequestHandler

from .aruco_detector import ArucoDetector
from .position_estimator import PositionEstimator, DroneState

logger = logging.getLogger(__name__)


class PositionServer:
    """
    Simple HTTP server for streaming position data to remote viewers.

    Runs in a background thread and serves:
    - /position - JSON position data
    - /debug-frame - JPEG with detection boxes drawn
    """

    def __init__(self, port: int = 8001):
        self.port = port
        self.latest_state: Optional[DroneState] = None
        self.latest_debug_frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self._server: Optional[HTTPServer] = None
        self._thread: Optional[threading.Thread] = None

        # Stats
        self.frame_count = 0
        self.detection_count = 0
        self._start_time = time.time()

        # Timing data
        self.timing = {}

    def update(self, state: Optional[DroneState], frame_count: int, detection_count: int,
                debug_frame: Optional[np.ndarray] = None, timing: Optional[dict] = None):
        """Update the latest position state and debug frame (called from main loop)."""
        with self.lock:
            if state is not None:
                self.latest_state = state
            self.frame_count = frame_count
            self.detection_count = detection_count
            if debug_frame is not None:
                self.latest_debug_frame = debug_frame
            if timing is not None:
                self.timing = timing

    def get_debug_frame_jpeg(self) -> Optional[bytes]:
        """Get latest debug frame as JPEG bytes."""
        with self.lock:
            if self.latest_debug_frame is None:
                return None
            _, jpeg = cv2.imencode('.jpg', self.latest_debug_frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
            return jpeg.tobytes()

    def get_json(self) -> str:
        """Get current state as JSON string."""
        with self.lock:
            if self.latest_state is None:
                return json.dumps({
                    "timestamp": time.time(),
                    "x": None,
                    "y": None,
                    "z": None,
                    "yaw": None,
                    "marker_ids": [],
                    "confidence": 0.0,
                    "frame_count": self.frame_count,
                    "detection_count": self.detection_count,
                    "detection_rate": self.detection_count / max(1, self.frame_count),
                    "uptime": time.time() - self._start_time,
                    "timing": self.timing
                })

            state = self.latest_state
            return json.dumps({
                "timestamp": state.timestamp,
                "x": round(state.x, 3),
                "y": round(state.y, 3),
                "z": round(state.z, 3),
                "yaw": round(state.yaw, 1),
                "marker_ids": state.marker_ids,
                "confidence": round(state.confidence, 2),
                "frame_count": self.frame_count,
                "detection_count": self.detection_count,
                "detection_rate": round(self.detection_count / max(1, self.frame_count), 2),
                "uptime": round(time.time() - self._start_time, 1),
                "timing": self.timing
            })

    def start(self):
        """Start HTTP server in background thread."""
        server_ref = self

        class PositionHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                # Suppress default logging
                pass

            def do_GET(self):
                if self.path == '/position' or self.path == '/':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(server_ref.get_json().encode())
                elif self.path == '/debug-frame':
                    jpeg_data = server_ref.get_debug_frame_jpeg()
                    if jpeg_data is not None:
                        self.send_response(200)
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                        self.end_headers()
                        self.wfile.write(jpeg_data)
                    else:
                        self.send_error(503, 'No frame available yet')
                else:
                    self.send_error(404)

        self._server = HTTPServer(('0.0.0.0', self.port), PositionHandler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        logger.info(f"Position server started on port {self.port}")

    def stop(self):
        """Stop the HTTP server."""
        if self._server:
            self._server.shutdown()
            logger.info("Position server stopped")


def init_camera_settings(device: str = "/dev/video0", exposure_us: Optional[int] = None) -> bool:
    """
    Initialize camera settings via v4l2-ctl.

    Args:
        device: Video device path (default: /dev/video0)
        exposure_us: Manual exposure time in microseconds. If None, uses auto exposure.
                     Lower values (e.g., 5000) reduce motion blur but need good lighting.

    Returns:
        True if settings were applied successfully
    """
    settings = [
        ("focus_automatic_continuous", 1),  # Auto focus enabled
        ("white_balance_automatic", 1),     # Auto white balance
    ]

    # Exposure settings
    if exposure_us is not None:
        # Manual exposure mode (1) with specific exposure time
        settings.insert(0, ("auto_exposure", 1))  # Manual mode
        settings.append(("exposure_time_absolute", exposure_us))
        logger.info(f"Setting manual exposure: {exposure_us}us ({exposure_us/1000:.1f}ms)")
    else:
        settings.insert(0, ("auto_exposure", 3))  # Aperture Priority (auto)

    success = True
    for ctrl, val in settings:
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", device, f"--set-ctrl={ctrl}={val}"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                logger.debug(f"Set {ctrl}={val}")
            else:
                logger.warning(f"Failed to set {ctrl}: {result.stderr.strip()}")
                success = False
        except FileNotFoundError:
            logger.warning("v4l2-ctl not found, skipping camera settings")
            return False
        except subprocess.TimeoutExpired:
            logger.warning(f"Timeout setting {ctrl}")
            success = False

    if success:
        mode = f"manual {exposure_us}us" if exposure_us else "auto"
        logger.info(f"Camera settings applied (exposure: {mode}, auto focus, auto WB)")
    return success


class VisionGPS:
    """
    Vision-based GPS emulator.

    Detects ArUco markers, estimates world position, and sends
    VISION_POSITION_ESTIMATE to the flight controller via MAVLink.
    """

    def __init__(self, config_path: str):
        self.config = self._load_config(config_path)
        self.config_dir = Path(config_path).parent

        self._running = False
        self._shutdown_requested = False

        # Components (initialized in start())
        self.detector: Optional[ArucoDetector] = None
        self.estimator: Optional[PositionEstimator] = None
        self.mavlink = None
        self.position_server: Optional[PositionServer] = None

        # Timing
        loop_rate = self.config.get('control', {}).get('loop_rate_hz', 20)
        self.loop_period = 1.0 / loop_rate

        # Statistics
        self.frames_processed = 0
        self.detections_count = 0
        self.positions_sent = 0
        self._start_time = 0.0

    def _load_config(self, path: str) -> dict:
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def start(self, use_mavlink: bool = True) -> bool:
        """
        Initialize all components.

        Args:
            use_mavlink: If False, skip MAVLink connection (for testing)

        Returns:
            True if started successfully
        """
        logger.info("Starting Vision GPS...")

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Initialize camera settings (exposure, focus, white balance)
        camera_device = f"/dev/video{self.config.get('camera', {}).get('device_id', 0)}"
        exposure_us = self.config.get('camera', {}).get('exposure_time_us')
        init_camera_settings(camera_device, exposure_us=exposure_us)

        # Initialize ArUco detector (single markers)
        camera_params_path = self.config_dir / "camera_params.yaml"
        self.detector = ArucoDetector.from_config(
            self.config,
            str(camera_params_path)
        )

        if not self.detector.start():
            logger.error("Failed to start camera")
            return False

        # Initialize position estimator
        marker_map_path = self.config_dir / "marker_map.yaml"
        self.estimator = PositionEstimator(str(marker_map_path))

        # Initialize MAVLink
        if use_mavlink:
            from .mavlink_interface import MAVLinkInterface

            serial_config = self.config.get('serial', {})
            self.mavlink = MAVLinkInterface(
                connection_string=serial_config.get('port', '/dev/serial0'),
                baud=serial_config.get('baud', 921600)
            )

            if not self.mavlink.connect(timeout=30.0):
                logger.error("Failed to connect to flight controller")
                return False

            # Set EKF origin (required for non-GPS flight)
            self.mavlink.set_ekf_origin()

        logger.info("All components initialized")
        self._running = True
        self._start_time = time.time()
        return True

    def stop(self):
        logger.info("Stopping Vision GPS...")
        self._running = False

        if self.detector:
            self.detector.stop()

        if self.mavlink:
            self.mavlink.disconnect()

        if self.position_server:
            self.position_server.stop()

        # Print final stats
        elapsed = time.time() - self._start_time if self._start_time else 0
        if elapsed > 0:
            logger.info(
                f"Stats: {self.frames_processed} frames, "
                f"{self.detections_count} detections "
                f"({100*self.detections_count/max(1,self.frames_processed):.1f}%), "
                f"{self.positions_sent} positions sent, "
                f"{elapsed:.1f}s runtime"
            )

        logger.info("Vision GPS stopped")

    def _signal_handler(self, signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        self._shutdown_requested = True

    def run(self):
        """
        Main loop: detect markers, estimate position, send to FC.
        """
        logger.info("Running Vision GPS (Ctrl+C to stop)")

        while self._running and not self._shutdown_requested:
            iteration_start = time.time()

            frame = self.detector.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            detections = self.detector.detect(frame)
            self.frames_processed += 1

            if detections:
                self.detections_count += 1
                state = self.estimator.estimate(detections)

                if state and self.mavlink:
                    # Convert ENU (estimator) to NED (MAVLink)
                    # ENU: X=East, Y=North, Z=Up
                    # NED: X=North, Y=East, Z=Down
                    self.mavlink.send_vision_position_estimate(
                        x=state.y,       # North = ENU Y
                        y=state.x,       # East  = ENU X
                        z=-state.z,      # Down  = -ENU Z
                        roll=0.0,
                        pitch=0.0,
                        yaw=np.radians(state.yaw),
                        confidence=state.confidence
                    )
                    self.positions_sent += 1

                # Log periodically
                if self.frames_processed % 40 == 0 and state:
                    logger.info(
                        f"Position: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})m "
                        f"yaw={state.yaw:.1f}° "
                        f"markers={state.marker_ids} "
                        f"conf={state.confidence:.2f}"
                    )

            # Rate limiting
            elapsed = time.time() - iteration_start
            sleep_time = self.loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("Main loop ended")

    def run_test(self):
        """
        Run without MAVLink -- prints position to console for testing.
        """
        logger.info("Running Vision GPS test mode (no MAVLink)")
        logger.info("Press Ctrl+C to stop")

        while self._running and not self._shutdown_requested:
            frame = self.detector.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            detections = self.detector.detect(frame)
            self.frames_processed += 1

            if detections:
                self.detections_count += 1
                state = self.estimator.estimate(detections)

                if state:
                    print(
                        f"\rPos: ({state.x:+.2f}, {state.y:+.2f}, {state.z:.2f})m | "
                        f"Yaw: {state.yaw:+.1f}° | "
                        f"Markers: {state.marker_ids} | "
                        f"Conf: {state.confidence:.2f}",
                        end=""
                    )
                else:
                    print("\rMarkers detected but position estimation failed" + " " * 20, end="")
            else:
                print("\rNo markers detected" + " " * 50, end="")

            time.sleep(self.loop_period)

        print()
        logger.info("Test mode ended")

    def run_stream(self, port: int = 8001):
        """
        Run with HTTP position streaming (no MAVLink).

        Starts an HTTP server that serves position data at /position endpoint.
        Useful for remote visualization and flight simulation testing.

        Args:
            port: HTTP server port (default: 8001)
        """
        self.position_server = PositionServer(port=port)
        self.position_server.start()

        logger.info(f"Running Vision GPS stream mode (HTTP on port {port})")
        logger.info("Press Ctrl+C to stop")

        while self._running and not self._shutdown_requested:
            iteration_start = time.time()

            frame = self.detector.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            detections = self.detector.detect(frame)
            self.frames_processed += 1

            state = None
            if detections:
                self.detections_count += 1
                state = self.estimator.estimate(detections)

            # Draw debug frame with detection overlay
            debug_frame = self.detector.draw_detections(frame, detections)
            # Add position info to debug frame
            if state:
                cv2.putText(
                    debug_frame,
                    f"Pos: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})m  Yaw: {state.yaw:.1f}°",
                    (10, debug_frame.shape[0] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
            detection_rate = self.detections_count / max(1, self.frames_processed)
            cv2.putText(
                debug_frame,
                f"Detection: {detection_rate:.0%} ({self.detections_count}/{self.frames_processed})",
                (10, debug_frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
            )

            # Update position server with latest state, debug frame, and timing
            self.position_server.update(state, self.frames_processed, self.detections_count, debug_frame, self.detector._timing)

            # Log periodically
            if self.frames_processed % 100 == 0:
                rate = self.detections_count / max(1, self.frames_processed)
                if state:
                    logger.info(
                        f"Position: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})m | "
                        f"Detection rate: {rate:.0%}"
                    )
                else:
                    logger.info(f"No position | Detection rate: {rate:.0%}")

            # Rate limiting
            elapsed = time.time() - iteration_start
            sleep_time = self.loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("Stream mode ended")


def main():
    parser = argparse.ArgumentParser(
        description="ArUco Vision GPS - sends position to flight controller"
    )
    parser.add_argument(
        '--config',
        type=str,
        default='config/system_config.yaml',
        help='Path to configuration file'
    )
    parser.add_argument(
        '--mode',
        type=str,
        choices=['run', 'test', 'stream'],
        default='test',
        help='run = send to FC, test = print to console, stream = HTTP server'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=8001,
        help='HTTP server port for stream mode (default: 8001)'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Enable verbose logging'
    )

    args = parser.parse_args()

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
    )

    config_path = Path(args.config)
    if not config_path.is_absolute():
        script_dir = Path(__file__).parent.parent
        config_path = script_dir / config_path

    if not config_path.exists():
        logger.error(f"Configuration file not found: {config_path}")
        sys.exit(1)

    gps = VisionGPS(str(config_path))

    try:
        use_mavlink = (args.mode == 'run')

        if not gps.start(use_mavlink=use_mavlink):
            logger.error("Failed to start Vision GPS")
            sys.exit(1)

        if args.mode == 'run':
            gps.run()
        elif args.mode == 'stream':
            gps.run_stream(port=args.port)
        else:
            gps.run_test()

    except Exception as e:
        logger.exception(f"Error: {e}")

    finally:
        gps.stop()


if __name__ == "__main__":
    main()
