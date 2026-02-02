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
import yaml
import argparse
import numpy as np
from pathlib import Path
from typing import Optional

from .aruco_detector import ArucoDetector
from .position_estimator import PositionEstimator, DroneState

logger = logging.getLogger(__name__)


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

        # Initialize ArUco detector
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
                    self.mavlink.send_vision_position_estimate(
                        x=state.x,
                        y=state.y,
                        z=state.z,
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
        choices=['run', 'test'],
        default='test',
        help='run = send to FC via MAVLink, test = print to console'
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
        else:
            gps.run_test()

    except Exception as e:
        logger.exception(f"Error: {e}")

    finally:
        gps.stop()


if __name__ == "__main__":
    main()
