"""
ArUco Drone Navigation - Main Control Loop

Phase 1: Single marker hover control
Phase 2: Multi-marker waypoint navigation (with mission support)

This is the main entry point for the navigation system.
"""

import time
import signal
import sys
import logging
import yaml
import argparse
import numpy as np
from pathlib import Path
from enum import Enum, auto
from typing import Optional

from .aruco_detector import ArucoDetector, MarkerDetection
from .position_estimator import PositionEstimator, SimplePositionEstimator, DroneState
from .mavlink_interface import MAVLinkInterface, FlightMode, TelemetryData
from .mission_executor import MissionExecutor, Mission, MissionState
from .flight_recorder import FlightRecorder
from .position_predictor import PositionPredictor

logger = logging.getLogger(__name__)


class NavigationState(Enum):
    """Navigation state machine states."""
    INIT = auto()
    CONNECTING = auto()
    IDLE = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    HOVERING = auto()
    NAVIGATING = auto()
    LANDING = auto()
    LANDED = auto()
    ERROR = auto()
    EMERGENCY = auto()


class PIDController:
    """Simple PID controller for position control."""

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limit: float = 1.0
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    def update(self, error: float, dt: Optional[float] = None) -> float:
        """
        Update PID controller with new error.

        Args:
            error: Current error (setpoint - measurement)
            dt: Time step (auto-calculated if None)

        Returns:
            Control output
        """
        current_time = time.time()

        if dt is None:
            if self._last_time is None:
                dt = 0.0
            else:
                dt = current_time - self._last_time
        self._last_time = current_time

        # Proportional
        p_term = self.kp * error

        # Integral (with anti-windup)
        self._integral += error * dt
        self._integral = np.clip(
            self._integral,
            -self.output_limit / (self.ki + 1e-6),
            self.output_limit / (self.ki + 1e-6)
        )
        i_term = self.ki * self._integral

        # Derivative
        if dt > 0:
            derivative = (error - self._last_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        self._last_error = error

        # Combined output with limiting
        output = p_term + i_term + d_term
        return np.clip(output, -self.output_limit, self.output_limit)

    def reset(self):
        """Reset controller state."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None


class DroneNavigator:
    """
    Main drone navigation controller.

    Phase 1: Hover under single ArUco marker
    Phase 2: Follow waypoints using multiple markers
    """

    def __init__(self, config_path: str):
        """
        Initialize navigator.

        Args:
            config_path: Path to system configuration YAML
        """
        self.config = self._load_config(config_path)
        self.config_dir = Path(config_path).parent

        # State
        self.state = NavigationState.INIT
        self._running = False
        self._shutdown_requested = False

        # Components (initialized in start())
        self.detector: Optional[ArucoDetector] = None
        self.estimator: Optional[PositionEstimator] = None
        self.simple_estimator = SimplePositionEstimator()
        self.mavlink: Optional[MAVLinkInterface] = None

        # Controllers
        control_config = self.config.get('control', {})
        self.pid_x = PIDController(
            kp=control_config.get('position_kp', 0.5),
            ki=control_config.get('position_ki', 0.0),
            kd=control_config.get('position_kd', 0.1),
            output_limit=control_config.get('max_velocity_xy', 0.5)
        )
        self.pid_y = PIDController(
            kp=control_config.get('position_kp', 0.5),
            ki=control_config.get('position_ki', 0.0),
            kd=control_config.get('position_kd', 0.1),
            output_limit=control_config.get('max_velocity_xy', 0.5)
        )
        self.pid_z = PIDController(
            kp=control_config.get('position_kp', 0.3),
            ki=0.0,
            kd=control_config.get('position_kd', 0.05),
            output_limit=control_config.get('max_velocity_z', 0.3)
        )

        # Target position (for Phase 1, just hover at marker center)
        self.target_altitude = 1.5  # meters from ground
        self.target_offset = np.array([0.0, 0.0])  # XY offset from marker

        # Safety
        self.safety_config = self.config.get('safety', {})
        self._marker_loss_start: Optional[float] = None
        self._last_detection_time = 0.0

        # Timing
        self.loop_rate = control_config.get('loop_rate_hz', 20)
        self.loop_period = 1.0 / self.loop_rate

        # Statistics
        self.frames_processed = 0
        self.detections_count = 0

        # Phase 2 components
        self.predictor: Optional[PositionPredictor] = None
        self.recorder: Optional[FlightRecorder] = None
        self.mission_executor: Optional[MissionExecutor] = None
        self._current_state: Optional[DroneState] = None

    def _load_config(self, path: str) -> dict:
        """Load configuration from YAML file."""
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def start(self) -> bool:
        """
        Initialize all components and start the system.

        Returns:
            True if started successfully
        """
        logger.info("Starting ArUco Drone Navigator...")

        # Setup signal handlers
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
        serial_config = self.config.get('serial', {})
        self.mavlink = MAVLinkInterface(
            connection_string=serial_config.get('port', '/dev/serial0'),
            baud=serial_config.get('baud', 921600)
        )

        self.state = NavigationState.CONNECTING

        if not self.mavlink.connect(timeout=30.0):
            logger.error("Failed to connect to flight controller")
            self.state = NavigationState.ERROR
            return False

        logger.info("All components initialized")
        self.state = NavigationState.IDLE
        self._running = True

        return True

    def stop(self):
        """Stop all components and clean up."""
        logger.info("Stopping navigator...")
        self._running = False

        if self.detector:
            self.detector.stop()

        if self.mavlink:
            self.mavlink.disconnect()

        self.state = NavigationState.LANDED
        logger.info("Navigator stopped")

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum}, initiating shutdown...")
        self._shutdown_requested = True

    def run_phase1_hover(self):
        """
        Phase 1: Simple hover control under single marker.

        The drone will:
        1. Detect the ArUco marker above
        2. Calculate offset from marker center
        3. Send velocity commands to center under marker
        4. Maintain target altitude
        """
        logger.info("Starting Phase 1: Single Marker Hover Control")
        logger.info(f"Target altitude: {self.target_altitude}m")
        logger.info("Press Ctrl+C to stop")

        self.state = NavigationState.HOVERING
        loop_start = time.time()

        while self._running and not self._shutdown_requested:
            iteration_start = time.time()

            # Get latest detection
            frame = self.detector.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            detections = self.detector.detect(frame)
            self.frames_processed += 1

            # Get telemetry
            telem = self.mavlink.get_telemetry()

            if detections:
                self.detections_count += 1
                self._last_detection_time = time.time()
                self._marker_loss_start = None

                # Use closest/first marker for Phase 1
                detection = detections[0]

                # Get offset from marker
                fwd, right, alt, yaw = self.simple_estimator.get_offset_from_marker(
                    detection
                )

                # Calculate desired altitude error
                # alt is distance to marker, we want to be at target_altitude from ground
                # Marker is on ceiling at ceiling_height, so:
                # desired_distance_to_marker = ceiling_height - target_altitude
                ceiling_height = self.estimator.markers.get(
                    detection.marker_id,
                    type('obj', (object,), {'world_position': np.array([0, 0, 3.0])})()
                ).world_position[2]

                desired_marker_distance = ceiling_height - self.target_altitude
                alt_error = alt - desired_marker_distance

                # PID control
                # Negative because we want to move opposite to offset
                vx = -self.pid_x.update(fwd)
                vy = -self.pid_y.update(right)
                vz = self.pid_z.update(alt_error)  # Positive vz = down in NED

                # Send velocity command (body frame)
                self.mavlink.send_velocity(vx, vy, vz, 0.0)

                # Log status periodically
                if self.frames_processed % 20 == 0:
                    logger.info(
                        f"Marker {detection.marker_id}: "
                        f"offset=({fwd:.2f}, {right:.2f})m, "
                        f"alt={alt:.2f}m, "
                        f"vel=({vx:.2f}, {vy:.2f}, {vz:.2f})"
                    )

            else:
                # No marker detected
                if self._marker_loss_start is None:
                    self._marker_loss_start = time.time()
                    logger.warning("Marker lost!")

                marker_loss_duration = time.time() - self._marker_loss_start
                timeout = self.safety_config.get('marker_loss_timeout', 2.0)

                if marker_loss_duration > timeout:
                    logger.error(
                        f"Marker lost for {marker_loss_duration:.1f}s, "
                        "triggering failsafe!"
                    )
                    self._trigger_failsafe()
                    break

                # Hold position (zero velocity)
                self.mavlink.send_velocity(0, 0, 0, 0)

            # Check for low battery
            if telem.battery_voltage > 0:
                low_batt = self.safety_config.get('low_battery_voltage', 10.5)
                if telem.battery_voltage < low_batt:
                    logger.warning(
                        f"Low battery: {telem.battery_voltage:.2f}V, landing!"
                    )
                    self._trigger_failsafe()
                    break

            # Rate limiting
            elapsed = time.time() - iteration_start
            sleep_time = self.loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.state = NavigationState.IDLE
        logger.info("Phase 1 hover control ended")

    def _trigger_failsafe(self):
        """Trigger failsafe landing."""
        logger.warning("FAILSAFE TRIGGERED - Landing!")
        self.state = NavigationState.EMERGENCY
        self.mavlink.set_mode(FlightMode.LAND)

    def run_ground_test(self):
        """
        Run control loop without sending commands (for testing).

        Useful for verifying detection and control calculations
        without actually flying.
        """
        logger.info("Starting ground test mode (no commands sent)")
        logger.info("Press Ctrl+C to stop")

        while self._running and not self._shutdown_requested:
            frame = self.detector.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            detections = self.detector.detect(frame)
            self.frames_processed += 1

            if detections:
                detection = detections[0]
                fwd, right, alt, yaw = self.simple_estimator.get_offset_from_marker(
                    detection
                )

                # Calculate what commands would be sent
                vx = -self.pid_x.update(fwd)
                vy = -self.pid_y.update(right)
                vz = self.pid_z.update(alt - 2.0)  # Assume 2m desired distance

                print(
                    f"\rMarker {detection.marker_id}: "
                    f"pos=({fwd:+.2f}, {right:+.2f}, {alt:.2f})m | "
                    f"cmd=({vx:+.2f}, {vy:+.2f}, {vz:+.2f})m/s | "
                    f"yaw={yaw:+.1f}°",
                    end=""
                )
            else:
                print("\rNo marker detected" + " " * 50, end="")

            time.sleep(self.loop_period)

        print()  # Newline after loop
        logger.info("Ground test ended")

    def run_mission(self, mission_path: str, record: bool = True):
        """
        Phase 2: Execute a mission from JSON file with multi-marker navigation.

        Args:
            mission_path: Path to mission JSON file
            record: Whether to record flight path
        """
        logger.info(f"Starting Phase 2: Mission Execution")
        logger.info(f"Mission file: {mission_path}")

        # Initialize Phase 2 components
        self.predictor = PositionPredictor(
            max_prediction_time=self.safety_config.get('marker_loss_timeout', 2.0)
        )

        recording_config = self.config.get('logging', {})
        self.recorder = FlightRecorder(
            sample_rate_hz=recording_config.get('record_rate_hz', 10),
            output_dir=str(self.config_dir.parent / "recordings")
        )

        # Load mission
        mission = Mission.from_json_file(mission_path)
        logger.info(f"Loaded mission: {mission.mission_id}")
        logger.info(f"Waypoints: {mission.num_waypoints}")
        logger.info(f"Total distance: {mission.total_distance:.2f}m")

        # Setup mission executor with callbacks
        self.mission_executor = MissionExecutor(
            position_callback=self._get_current_position,
            yaw_callback=self._get_current_yaw,
            velocity_command_callback=self._send_velocity_command,
            arm_callback=lambda: self.mavlink.arm(),
            disarm_callback=lambda: self.mavlink.disarm(),
            takeoff_callback=lambda alt: self.mavlink.takeoff(alt),
            land_callback=lambda: self.mavlink.land()
        )

        if not self.mission_executor.load_mission_dict(mission.to_dict()):
            logger.error("Failed to load mission into executor")
            return

        # Start recording
        if record:
            self.recorder.start_recording(
                mission_id=mission.mission_id,
                metadata={'config': str(self.config_dir)}
            )

        self.state = NavigationState.NAVIGATING
        logger.info("Press Ctrl+C to abort mission")

        try:
            # Start mission
            if not self.mission_executor.start():
                logger.error("Failed to start mission")
                return

            # Main control loop
            while self._running and not self._shutdown_requested:
                iteration_start = time.time()

                # Get frame and detect markers
                frame = self.detector.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                detections = self.detector.detect(frame)
                self.frames_processed += 1

                # Estimate position
                if detections:
                    self.detections_count += 1
                    self._last_detection_time = time.time()
                    self._marker_loss_start = None

                    # Get world position from markers
                    self._current_state = self.estimator.estimate(detections)

                    if self._current_state:
                        # Update predictor with good measurement
                        self.predictor.update(
                            self._current_state.position,
                            self._current_state.yaw,
                            self._current_state.marker_ids,
                            self._current_state.confidence
                        )

                        # Record flight data
                        if record and self.recorder.is_recording:
                            self.recorder.add_state(
                                self._current_state.position,
                                self._current_state.yaw,
                                marker_ids=self._current_state.marker_ids,
                                confidence=self._current_state.confidence
                            )

                else:
                    # No markers - use prediction
                    if self._marker_loss_start is None:
                        self._marker_loss_start = time.time()
                        logger.warning("Markers lost, using prediction...")

                    prediction = self.predictor.predict()
                    if prediction:
                        pos, yaw, conf = prediction
                        self._current_state = DroneState(
                            position=pos,
                            yaw=yaw,
                            timestamp=time.time(),
                            marker_ids=[],
                            confidence=conf
                        )

                        # Record predicted position
                        if record and self.recorder.is_recording:
                            self.recorder.add_state(
                                pos, yaw,
                                marker_ids=[],
                                confidence=conf
                            )
                    else:
                        # Prediction failed/timed out
                        logger.error("Position prediction failed!")
                        self._trigger_failsafe()
                        break

                # Update mission executor
                if not self.mission_executor.update():
                    # Mission complete or aborted
                    break

                # Check mission state
                if self.mission_executor.state == MissionState.COMPLETE:
                    logger.info("Mission completed successfully!")
                    break
                elif self.mission_executor.state in [MissionState.ABORTED, MissionState.ERROR]:
                    logger.error(f"Mission ended: {self.mission_executor.state.name}")
                    break

                # Check battery
                telem = self.mavlink.get_telemetry()
                if telem.battery_voltage > 0:
                    low_batt = self.safety_config.get('low_battery_voltage', 10.5)
                    if telem.battery_voltage < low_batt:
                        logger.warning(f"Low battery: {telem.battery_voltage:.2f}V")
                        self.mission_executor.abort()
                        break

                # Log progress periodically
                if self.frames_processed % 50 == 0:
                    wp = self.mission_executor.current_waypoint
                    if wp and self._current_state:
                        dist = self._current_state.distance_to(wp.position)
                        logger.info(
                            f"WP {self.mission_executor.current_waypoint_index}: "
                            f"dist={dist:.2f}m, "
                            f"state={self.mission_executor.state.name}"
                        )

                # Rate limiting
                elapsed = time.time() - iteration_start
                sleep_time = self.loop_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            logger.exception(f"Mission error: {e}")
            self.mission_executor.abort()

        finally:
            # Stop recording and save
            if record and self.recorder.is_recording:
                filepath = self.recorder.stop_recording(save=True)
                if filepath:
                    logger.info(f"Flight recording saved to: {filepath}")
                    stats = self.recorder.get_statistics()
                    logger.info(f"Recording stats: {stats}")

            self.state = NavigationState.IDLE
            logger.info("Mission execution ended")

    def _get_current_position(self) -> Optional[np.ndarray]:
        """Callback: Get current position for mission executor."""
        if self._current_state:
            return self._current_state.position
        return None

    def _get_current_yaw(self) -> float:
        """Callback: Get current yaw for mission executor."""
        if self._current_state:
            return self._current_state.yaw
        return 0.0

    def _send_velocity_command(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Callback: Send velocity command for mission executor."""
        self.mavlink.send_velocity(vx, vy, vz, yaw_rate)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="ArUco-based drone navigation system"
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
        choices=['hover', 'ground_test', 'mission'],
        default='ground_test',
        help='Operation mode'
    )
    parser.add_argument(
        '--altitude',
        type=float,
        default=1.5,
        help='Target hover altitude in meters'
    )
    parser.add_argument(
        '--mission', '-m',
        type=str,
        default=None,
        help='Path to mission JSON file (required for mission mode)'
    )
    parser.add_argument(
        '--no-record',
        action='store_true',
        help='Disable flight recording during mission'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Enable verbose logging'
    )

    args = parser.parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
    )

    # Find config file
    config_path = Path(args.config)
    if not config_path.is_absolute():
        # Try relative to script location
        script_dir = Path(__file__).parent.parent
        config_path = script_dir / config_path

    if not config_path.exists():
        logger.error(f"Configuration file not found: {config_path}")
        sys.exit(1)

    # Create and start navigator
    navigator = DroneNavigator(str(config_path))

    try:
        if not navigator.start():
            logger.error("Failed to start navigator")
            sys.exit(1)

        navigator.target_altitude = args.altitude

        if args.mode == 'ground_test':
            navigator.run_ground_test()
        elif args.mode == 'hover':
            navigator.run_phase1_hover()
        elif args.mode == 'mission':
            if args.mission is None:
                logger.error("Mission file required for mission mode (--mission)")
                sys.exit(1)

            mission_path = Path(args.mission)
            if not mission_path.is_absolute():
                mission_path = Path(__file__).parent.parent / mission_path

            if not mission_path.exists():
                logger.error(f"Mission file not found: {mission_path}")
                sys.exit(1)

            navigator.run_mission(
                str(mission_path),
                record=not args.no_record
            )

    except Exception as e:
        logger.exception(f"Error: {e}")

    finally:
        navigator.stop()


if __name__ == "__main__":
    main()
