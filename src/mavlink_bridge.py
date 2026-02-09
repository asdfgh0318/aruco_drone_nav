"""Minimal MAVLink bridge - sends vision position estimates to flight controller."""

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

    def connect(self, timeout=10.0):
        try:
            log.info(f"Connecting to {self.port}...")
            kwargs = {"source_system": 255, "source_component": 0}
            if self.port.startswith("/dev"):
                kwargs["baud"] = self.baud
            self.conn = mavutil.mavlink_connection(self.port, **kwargs)

            log.info("Waiting for heartbeat...")
            msg = self.conn.wait_heartbeat(timeout=timeout)
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

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.conn:
            self.conn.close()
            self.conn = None
        log.info("Disconnected")

    def _receive_loop(self):
        while self._running and self.conn:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.1)
                if msg and msg.get_type() == "HEARTBEAT":
                    self._last_heartbeat = time.time()
            except Exception:
                pass

    @property
    def is_connected(self):
        return self.conn is not None and (time.time() - self._last_heartbeat) < 3.0

    def send_vision_position(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, confidence=1.0):
        if not self.conn:
            return
        usec = int(time.time() * 1e6)
        pos_cov = max(0.01, 0.01 / max(confidence, 0.01))
        ang_cov = pos_cov * 2.0
        covariance = [
            pos_cov, 0, 0, 0, 0, 0,
            pos_cov, 0, 0, 0, 0,
            pos_cov, 0, 0, 0,
            ang_cov, 0, 0,
            ang_cov, 0,
            ang_cov,
        ]
        self.conn.mav.vision_position_estimate_send(
            usec, x, y, z, roll, pitch, yaw, covariance, 0
        )

    def set_ekf_origin(self, lat, lon, alt):
        if not self.conn:
            return
        self.conn.mav.set_gps_global_origin_send(
            self.target_system,
            int(lat * 1e7), int(lon * 1e7), int(alt * 1000),
            int(time.time() * 1e6),
        )
        log.info(f"EKF origin set to ({lat:.4f}, {lon:.4f}, {alt:.0f}m)")
