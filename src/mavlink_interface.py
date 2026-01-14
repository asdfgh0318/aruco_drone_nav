"""
MAVLink Interface Module

Provides communication with ArduCopter flight controller via MAVLink protocol.
Handles connection, telemetry, and command sending for drone control.
"""

import time
import threading
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Callable, Dict, Any
import logging

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

logger = logging.getLogger(__name__)


class FlightMode(IntEnum):
    """ArduCopter flight modes."""
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    AVOID_ADSB = 19
    GUIDED_NOGPS = 20
    SMART_RTL = 21
    FLOWHOLD = 22
    FOLLOW = 23
    ZIGZAG = 24
    SYSTEMID = 25
    AUTOROTATE = 26
    AUTO_RTL = 27


@dataclass
class TelemetryData:
    """Container for drone telemetry data."""

    # Position (local NED frame)
    x: float = 0.0               # North (meters)
    y: float = 0.0               # East (meters)
    z: float = 0.0               # Down (meters, negative is up)

    # Velocity
    vx: float = 0.0              # North velocity (m/s)
    vy: float = 0.0              # East velocity (m/s)
    vz: float = 0.0              # Down velocity (m/s)

    # Attitude (radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # System state
    armed: bool = False
    mode: int = 0
    mode_name: str = "UNKNOWN"
    battery_voltage: float = 0.0
    battery_remaining: int = 100

    # GPS
    gps_fix: int = 0
    satellites: int = 0

    # Timestamps
    timestamp: float = 0.0
    heartbeat_time: float = 0.0

    @property
    def altitude(self) -> float:
        """Get altitude (positive up) in meters."""
        return -self.z


class MAVLinkInterface:
    """
    Interface for MAVLink communication with ArduCopter.

    Provides methods for:
    - Connection management
    - Telemetry reading
    - Flight mode control
    - Arming/disarming
    - Position and velocity commands
    """

    def __init__(
        self,
        connection_string: str = "/dev/serial0",
        baud: int = 921600,
        source_system: int = 255,
        source_component: int = 0
    ):
        """
        Initialize MAVLink interface.

        Args:
            connection_string: Serial port or UDP connection string
            baud: Baud rate for serial connection
            source_system: MAVLink system ID for this component
            source_component: MAVLink component ID
        """
        self.connection_string = connection_string
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component

        self.connection: Optional[mavutil.mavlink_connection] = None
        self.telemetry = TelemetryData()

        self._connected = False
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._telemetry_lock = threading.Lock()

        # Callbacks for telemetry updates
        self._callbacks: Dict[str, Callable] = {}

        # Target system/component (will be set on connection)
        self.target_system = 1
        self.target_component = 1

    def connect(self, timeout: float = 10.0) -> bool:
        """
        Establish MAVLink connection and wait for heartbeat.

        Args:
            timeout: Maximum time to wait for heartbeat

        Returns:
            True if connected successfully
        """
        try:
            logger.info(
                f"Connecting to {self.connection_string} at {self.baud} baud..."
            )

            # Create connection
            if self.connection_string.startswith("/dev"):
                # Serial connection
                self.connection = mavutil.mavlink_connection(
                    self.connection_string,
                    baud=self.baud,
                    source_system=self.source_system,
                    source_component=self.source_component
                )
            else:
                # UDP or TCP connection
                self.connection = mavutil.mavlink_connection(
                    self.connection_string,
                    source_system=self.source_system,
                    source_component=self.source_component
                )

            # Wait for heartbeat
            logger.info("Waiting for heartbeat...")
            msg = self.connection.wait_heartbeat(timeout=timeout)

            if msg is None:
                logger.error("No heartbeat received")
                return False

            self.target_system = msg.get_srcSystem()
            self.target_component = msg.get_srcComponent()

            logger.info(
                f"Connected to system {self.target_system}, "
                f"component {self.target_component}"
            )

            # Request data streams
            self._request_data_streams()

            # Start receive thread
            self._running = True
            self._recv_thread = threading.Thread(
                target=self._receive_loop,
                daemon=True
            )
            self._recv_thread.start()

            self._connected = True
            return True

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Close MAVLink connection."""
        self._running = False

        if self._recv_thread is not None:
            self._recv_thread.join(timeout=2.0)
            self._recv_thread = None

        if self.connection is not None:
            self.connection.close()
            self.connection = None

        self._connected = False
        logger.info("Disconnected")

    def _request_data_streams(self):
        """Request telemetry data streams from flight controller."""
        if self.connection is None:
            return

        # Request all data streams at reasonable rates
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_ALL, 4),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2),
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),  # Attitude
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 2),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2),
        ]

        for stream_id, rate in streams:
            self.connection.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                stream_id,
                rate,
                1  # Start sending
            )

    def _receive_loop(self):
        """Background thread for receiving MAVLink messages."""
        while self._running and self.connection is not None:
            try:
                msg = self.connection.recv_match(blocking=True, timeout=0.1)
                if msg is not None:
                    self._handle_message(msg)
            except Exception as e:
                if self._running:
                    logger.error(f"Receive error: {e}")

    def _handle_message(self, msg):
        """Process received MAVLink message."""
        msg_type = msg.get_type()

        with self._telemetry_lock:
            if msg_type == "HEARTBEAT":
                self.telemetry.heartbeat_time = time.time()
                self.telemetry.mode = msg.custom_mode
                self.telemetry.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

                # Get mode name
                try:
                    self.telemetry.mode_name = FlightMode(msg.custom_mode).name
                except ValueError:
                    self.telemetry.mode_name = f"MODE_{msg.custom_mode}"

            elif msg_type == "LOCAL_POSITION_NED":
                self.telemetry.x = msg.x
                self.telemetry.y = msg.y
                self.telemetry.z = msg.z
                self.telemetry.vx = msg.vx
                self.telemetry.vy = msg.vy
                self.telemetry.vz = msg.vz
                self.telemetry.timestamp = time.time()

            elif msg_type == "ATTITUDE":
                self.telemetry.roll = msg.roll
                self.telemetry.pitch = msg.pitch
                self.telemetry.yaw = msg.yaw

            elif msg_type == "SYS_STATUS":
                self.telemetry.battery_voltage = msg.voltage_battery / 1000.0
                self.telemetry.battery_remaining = msg.battery_remaining

            elif msg_type == "GPS_RAW_INT":
                self.telemetry.gps_fix = msg.fix_type
                self.telemetry.satellites = msg.satellites_visible

        # Call registered callbacks
        if msg_type in self._callbacks:
            self._callbacks[msg_type](msg)

    def get_telemetry(self) -> TelemetryData:
        """Get copy of current telemetry data."""
        with self._telemetry_lock:
            return TelemetryData(
                x=self.telemetry.x,
                y=self.telemetry.y,
                z=self.telemetry.z,
                vx=self.telemetry.vx,
                vy=self.telemetry.vy,
                vz=self.telemetry.vz,
                roll=self.telemetry.roll,
                pitch=self.telemetry.pitch,
                yaw=self.telemetry.yaw,
                armed=self.telemetry.armed,
                mode=self.telemetry.mode,
                mode_name=self.telemetry.mode_name,
                battery_voltage=self.telemetry.battery_voltage,
                battery_remaining=self.telemetry.battery_remaining,
                gps_fix=self.telemetry.gps_fix,
                satellites=self.telemetry.satellites,
                timestamp=self.telemetry.timestamp,
                heartbeat_time=self.telemetry.heartbeat_time
            )

    def register_callback(self, msg_type: str, callback: Callable):
        """Register callback for specific message type."""
        self._callbacks[msg_type] = callback

    @property
    def is_connected(self) -> bool:
        """Check if connection is active (heartbeat within last 3 seconds)."""
        if not self._connected:
            return False

        with self._telemetry_lock:
            return (time.time() - self.telemetry.heartbeat_time) < 3.0

    @property
    def is_armed(self) -> bool:
        """Check if drone is armed."""
        with self._telemetry_lock:
            return self.telemetry.armed

    # ==================== COMMANDS ====================

    def arm(self, force: bool = False) -> bool:
        """
        Arm the drone motors.

        Args:
            force: Force arming even if pre-arm checks fail

        Returns:
            True if arm command sent successfully
        """
        if self.connection is None:
            return False

        param = 21196 if force else 0  # Magic number for force arm

        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,      # Arm
            param,  # Force parameter
            0, 0, 0, 0, 0
        )

        logger.info("Arm command sent")
        return True

    def disarm(self, force: bool = False) -> bool:
        """
        Disarm the drone motors.

        Args:
            force: Force disarm even while flying (DANGEROUS!)

        Returns:
            True if disarm command sent
        """
        if self.connection is None:
            return False

        param = 21196 if force else 0

        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,      # Disarm
            param,
            0, 0, 0, 0, 0
        )

        logger.info("Disarm command sent")
        return True

    def set_mode(self, mode: FlightMode) -> bool:
        """
        Set flight mode.

        Args:
            mode: Target flight mode

        Returns:
            True if mode change command sent
        """
        if self.connection is None:
            return False

        self.connection.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode
        )

        logger.info(f"Mode change to {mode.name} requested")
        return True

    def takeoff(self, altitude: float) -> bool:
        """
        Command takeoff to specified altitude.

        Args:
            altitude: Target altitude in meters

        Returns:
            True if takeoff command sent
        """
        if self.connection is None:
            return False

        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            altitude
        )

        logger.info(f"Takeoff to {altitude}m commanded")
        return True

    def land(self) -> bool:
        """
        Command landing at current position.

        Returns:
            True if land command sent
        """
        if self.connection is None:
            return False

        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0
        )

        logger.info("Land command sent")
        return True

    def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
        coordinate_frame: int = mavutil.mavlink.MAV_FRAME_BODY_NED
    ):
        """
        Send velocity command.

        Args:
            vx: Forward velocity (m/s) - body frame
            vy: Right velocity (m/s) - body frame
            vz: Down velocity (m/s) - body frame (negative = up)
            yaw_rate: Yaw rate (rad/s)
            coordinate_frame: MAVLink coordinate frame
        """
        if self.connection is None:
            return

        # Type mask: ignore position, use velocity
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        self.connection.mav.set_position_target_local_ned_send(
            0,                      # time_boot_ms (not used)
            self.target_system,
            self.target_component,
            coordinate_frame,
            type_mask,
            0, 0, 0,               # position (ignored)
            vx, vy, vz,            # velocity
            0, 0, 0,               # acceleration (ignored)
            0,                     # yaw (ignored)
            yaw_rate               # yaw_rate
        )

    def send_velocity_ned(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rate: float = 0.0
    ):
        """
        Send velocity command in NED frame.

        Args:
            vn: North velocity (m/s)
            ve: East velocity (m/s)
            vd: Down velocity (m/s)
            yaw_rate: Yaw rate (rad/s)
        """
        self.send_velocity(
            vn, ve, vd, yaw_rate,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED
        )

    def send_position_target(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float = 0.0,
        coordinate_frame: int = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    ):
        """
        Send position target command.

        Args:
            x: X position (meters)
            y: Y position (meters)
            z: Z position (meters, negative = up in NED)
            yaw: Yaw angle (radians)
            coordinate_frame: MAVLink coordinate frame
        """
        if self.connection is None:
            return

        # Type mask: use position and yaw, ignore velocity and acceleration
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.connection.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            coordinate_frame,
            type_mask,
            x, y, z,               # position
            0, 0, 0,               # velocity (ignored)
            0, 0, 0,               # acceleration (ignored)
            yaw,                   # yaw
            0                      # yaw_rate (ignored)
        )

    def send_position_velocity(
        self,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
        yaw: float = 0.0,
        coordinate_frame: int = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    ):
        """
        Send combined position and velocity target (feedforward control).

        Args:
            x, y, z: Position target (meters)
            vx, vy, vz: Velocity feedforward (m/s)
            yaw: Yaw angle (radians)
            coordinate_frame: MAVLink coordinate frame
        """
        if self.connection is None:
            return

        # Use position, velocity, and yaw
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.connection.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            coordinate_frame,
            type_mask,
            x, y, z,
            vx, vy, vz,
            0, 0, 0,
            yaw,
            0
        )

    def set_home_position(self):
        """Set current position as home."""
        if self.connection is None:
            return

        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            1,  # Use current location
            0, 0, 0, 0, 0, 0
        )
        logger.info("Set home position command sent")

    def reboot_flight_controller(self):
        """Reboot the flight controller (use with caution!)."""
        if self.connection is None:
            return

        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1,  # Reboot autopilot
            0, 0, 0, 0, 0, 0
        )
        logger.warning("Flight controller reboot commanded")


def test_connection(connection_string: str = "/dev/serial0", baud: int = 921600):
    """Test MAVLink connection."""
    logging.basicConfig(level=logging.INFO)

    mav = MAVLinkInterface(connection_string, baud)

    if not mav.connect():
        print("Failed to connect")
        return

    print("\nConnection established! Monitoring telemetry...")
    print("Press Ctrl+C to exit\n")

    try:
        while True:
            telem = mav.get_telemetry()
            print(
                f"Mode: {telem.mode_name:12} | "
                f"Armed: {telem.armed} | "
                f"Alt: {telem.altitude:6.2f}m | "
                f"Batt: {telem.battery_voltage:5.2f}V ({telem.battery_remaining}%)"
            )
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        mav.disconnect()


if __name__ == "__main__":
    import sys
    conn = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
    test_connection(conn, baud)
