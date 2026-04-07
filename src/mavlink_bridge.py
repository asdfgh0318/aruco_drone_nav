"""Minimal MAVLink bridge - sends GPS_INPUT to flight controller."""

import math
import time
import threading
import logging
from pymavlink import mavutil

log = logging.getLogger(__name__)


class MAVLinkBridge:
    def __init__(self, port="/dev/serial0", baud=921600):
        self.port = port
        self.baud = baud
        self.conn = None
        self.target_system = 1
        self.target_component = 1
        self._running = False
        self._thread = None
        self._last_heartbeat = 0.0
        self.attitude = None  # (roll, pitch, yaw) in radians
        self.attitude_rate = None  # (rollspeed, pitchspeed, yawspeed) in rad/s
        self.fc_position = None  # (lat, lon, alt, yaw_deg) from FC EKF

    def connect(self, timeout=10.0):
        try:
            log.info(f"Connecting to {self.port}...")
            kwargs = {"source_system": 255, "source_component": 0}
            if self.port.startswith("/dev"):
                kwargs["baud"] = self.baud
                # Workaround: pyserial on RPi PL011 (ttyAMA0) throws
                # "device reports readiness to read but returned no data"
                try:
                    import serial
                    _orig_read = serial.Serial.read
                    def _safe_read(self_ser, size=1):
                        try:
                            return _orig_read(self_ser, size)
                        except serial.SerialException:
                            return b''
                    serial.Serial.read = _safe_read
                except ImportError:
                    pass
            self.conn = mavutil.mavlink_connection(self.port, mavlink20=True, **kwargs)

            # ArduPilot requires heartbeats before it streams on non-primary UARTs
            log.info("Sending heartbeats to wake up FC...")
            for _ in range(5):
                self._send_heartbeat()
                time.sleep(0.3)

            log.info("Waiting for FC heartbeat...")
            deadline = time.time() + timeout
            msg = None
            while time.time() < deadline:
                m = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                if m and m.get_srcSystem() != 255:  # Ignore our own heartbeats
                    msg = m
                    break
                # Keep sending heartbeats while waiting
                self._send_heartbeat()
            if msg is None:
                log.error("No heartbeat received")
                return False

            self.target_system = msg.get_srcSystem()
            self.target_component = msg.get_srcComponent()
            self._last_heartbeat = time.time()
            log.info(f"Connected to system {self.target_system}")

            self._running = True
            self._thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._thread.start()
            return True
        except Exception as e:
            log.error(f"Connection failed: {e}")
            return False

    def _send_heartbeat(self):
        if self.conn:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.conn:
            self.conn.close()
            self.conn = None
        log.info("Disconnected")

    def _receive_loop(self):
        last_hb_sent = 0.0
        while self._running and self.conn:
            try:
                # Send heartbeat at 1Hz to keep FC streaming
                now = time.time()
                if now - last_hb_sent >= 1.0:
                    self._send_heartbeat()
                    last_hb_sent = now
                msg = self.conn.recv_match(blocking=True, timeout=0.1)
                if msg:
                    mtype = msg.get_type()
                    if mtype == "HEARTBEAT" and msg.get_srcSystem() != 255:
                        self._last_heartbeat = time.time()
                    elif mtype == "ATTITUDE":
                        self.attitude = (msg.roll, msg.pitch, msg.yaw)
                        self.attitude_rate = (msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                    elif mtype == "GLOBAL_POSITION_INT":
                        self.fc_position = (
                            msg.lat / 1e7, msg.lon / 1e7,
                            msg.relative_alt / 1000.0,
                            msg.hdg / 100.0 if msg.hdg != 65535 else 0.0
                        )
            except Exception:
                pass

    @property
    def is_connected(self):
        return self.conn is not None and (time.time() - self._last_heartbeat) < 3.0

    def send_gps_input(self, north, east, down, yaw_deg, origin_lat, origin_lon, origin_alt,
                       confidence=1.0, horiz_accuracy=0.1):
        """Send GPS_INPUT emulating a GPS fix from NED position relative to origin."""
        if not self.conn:
            return
        lat = origin_lat + (north / 111111.0)
        lon = origin_lon + (east / (111111.0 * math.cos(math.radians(origin_lat))))
        alt = origin_alt - down
        hdop = 0.3
        vdop = 0.5
        # yaw: GPS_INPUT expects centidegrees, 0=North, CW positive, 0-36000
        yaw_cd = int(((yaw_deg % 360) + 360) % 360 * 100)
        # GPS epoch: Jan 6 1980 00:00:00 UTC
        gps_epoch = 315964800.0
        gps_seconds = time.time() - gps_epoch
        gps_week = int(gps_seconds / 604800)
        gps_week_ms = int((gps_seconds % 604800) * 1000)
        self.conn.mav.gps_input_send(
            int(time.time() * 1e6),   # time_usec
            0,                        # gps_id
            (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
             mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY),
            gps_week_ms,              # time_week_ms (uint32)
            gps_week,                 # time_week (uint16)
            3,                        # fix_type: 3D fix
            int(lat * 1e7),           # lat (degE7)
            int(lon * 1e7),           # lon (degE7)
            alt,                      # alt (m)
            hdop,                     # hdop
            vdop,                     # vdop
            0.0,                      # vn (m/s)
            0.0,                      # ve (m/s)
            0.0,                      # vd (m/s)
            0.01,                     # speed_accuracy (m/s)
            horiz_accuracy,               # horiz_accuracy (m)
            0.1,                      # vert_accuracy (m)
            12,                       # satellites_visible
            yaw_cd,                   # yaw (centidegrees)
        )

