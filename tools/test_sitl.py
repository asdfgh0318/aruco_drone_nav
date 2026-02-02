#!/usr/bin/env python3
"""
SITL Validation Tool

Verifies the full vision-to-FC MAVLink pipeline works in ArduCopter SITL.
Connects to SITL, configures FC parameters, sends synthetic vision positions,
and verifies the FC accepts them via EKF convergence.

Prerequisites:
    Build SITL:
        cd ~/ardupilot && ./waf configure --board sitl && ./waf build --target bin/arducopter

    Start SITL with vision params (recommended):
        cd /tmp && ~/ardupilot/build/sitl/bin/arducopter --model + --speedup 1 -I0 \
            --home 52.2297,21.0122,100,0 \
            --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm,config/sitl_params.parm

    Or with sim_vehicle.py (if MAVProxy installed):
        sim_vehicle.py -v ArduCopter --add-param-file config/sitl_params.parm --console --map

Usage:
    python3 tools/test_sitl.py -v --skip-params        # Pre-configured SITL (recommended)
    python3 tools/test_sitl.py -v                       # Set params + reboot (UDP/sim_vehicle only)
    python3 tools/test_sitl.py -v --skip-params --arm   # Include arm + guided flight
    python3 tools/test_sitl.py -p circle -d 20 -v --skip-params  # Circle pattern for 20s
"""

import argparse
import math
import sys
import time
import threading
import logging
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.mavlink_interface import MAVLinkInterface, FlightMode

logger = logging.getLogger(__name__)

# FC parameters for vision-based navigation in SITL
SITL_PARAMS = {
    "AHRS_EKF_TYPE": 3,
    "EK3_ENABLE": 1,
    "EK2_ENABLE": 0,
    "EK3_SRC1_POSXY": 6,
    "EK3_SRC1_POSZ": 1,       # Baro for altitude (safer)
    "EK3_SRC1_VELXY": 0,
    "EK3_SRC1_VELZ": 0,
    "EK3_SRC1_YAW": 6,
    "VISO_TYPE": 1,
    "VISO_POS_M_NSE": 0.2,
    "VISO_YAW_M_NSE": 0.3,
    "VISO_DELAY_MS": 50,
    "GPS_TYPE": 0,
    "COMPASS_ENABLE": 0,
    "COMPASS_USE": 0,
    "COMPASS_USE2": 0,
    "COMPASS_USE3": 0,
}


class SITLValidator:
    """Validates the vision-to-FC MAVLink pipeline against ArduCopter SITL."""

    def __init__(self, connection_string: str, pattern: str, duration: float,
                 attempt_arm: bool, skip_params: bool):
        self.connection_string = connection_string
        self.pattern = pattern
        self.duration = duration
        self.attempt_arm = attempt_arm
        self.skip_params = skip_params

        self.mav: MAVLinkInterface = None
        self.results = []
        self.status_messages = []

    def run(self) -> bool:
        """Run all validation steps. Returns True if all passed."""
        steps = [
            ("Connect to SITL", self._step_connect),
        ]

        if not self.skip_params:
            steps.append(("Set FC parameters", self._step_set_params))
            steps.append(("Reboot & reconnect", self._step_reboot))

        steps.extend([
            ("Set EKF origin", self._step_set_origin),
            (f"Stream vision data ({self.duration:.0f}s)", self._step_stream),
            ("Check EKF convergence", self._step_check_convergence),
        ])

        if self.attempt_arm:
            steps.append(("Arm + guided flight", self._step_arm_flight))

        print()
        for name, func in steps:
            print(f"  {name:35s}", end="", flush=True)
            try:
                success = func()
            except Exception as e:
                logger.debug(f"Step '{name}' exception: {e}", exc_info=True)
                success = False
            tag = "[PASS]" if success else "[FAIL]"
            self.results.append((name, success))
            print(f" {tag}")
            if not success:
                break

        # Summary
        passed = sum(1 for _, s in self.results if s)
        total = len(self.results)
        print(f"\n  Result: {passed}/{total} PASSED\n")

        # Cleanup
        if self.mav is not None:
            self.mav.disconnect()

        return passed == total

    # ======================== STEPS ========================

    def _step_connect(self) -> bool:
        """Step 1: Connect to SITL and wait for heartbeat."""
        self.mav = MAVLinkInterface(
            connection_string=self.connection_string,
            source_system=255,
            source_component=0,
        )
        if not self.mav.connect(timeout=15.0):
            logger.error("No heartbeat from SITL")
            return False

        # Register STATUSTEXT callback to capture FC messages
        self.mav.register_callback("STATUSTEXT", self._on_statustext)
        return True

    def _step_set_params(self) -> bool:
        """Step 2: Set FC parameters for vision navigation."""
        conn = self.mav.connection
        if conn is None:
            return False

        for name, value in SITL_PARAMS.items():
            logger.debug(f"Setting {name} = {value}")

            # Send param set
            conn.mav.param_set_send(
                self.mav.target_system,
                self.mav.target_component,
                name.encode('utf-8'),
                float(value),
                0,  # MAV_PARAM_TYPE_REAL32 (SITL uses float for all)
            )

            # Wait for acknowledgement via PARAM_VALUE
            ack_event = threading.Event()
            ack_value = [None]

            def on_param_value(msg, _name=name, _ev=ack_event, _val=ack_value):
                param_id = msg.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode('utf-8')
                param_id = param_id.rstrip('\x00')
                if param_id == _name:
                    _val[0] = msg.param_value
                    _ev.set()

            self.mav.register_callback("PARAM_VALUE", on_param_value)

            if not ack_event.wait(timeout=3.0):
                logger.warning(f"No ack for {name}, continuing anyway")
            else:
                logger.debug(f"  {name} confirmed = {ack_value[0]}")

        # Remove param callback
        self.mav._callbacks.pop("PARAM_VALUE", None)
        return True

    def _step_reboot(self) -> bool:
        """Step 3: Reboot FC and reconnect."""
        logger.debug("Commanding reboot...")
        self.mav.reboot_flight_controller()

        # Disconnect old connection
        self.mav.disconnect()
        self.mav = None

        # Wait for FC to restart
        logger.debug("Waiting for FC restart...")
        time.sleep(5)

        # Retry connection — SITL may take variable time to re-bind TCP
        for attempt in range(5):
            logger.debug(f"Reconnect attempt {attempt + 1}/5...")
            self.mav = MAVLinkInterface(
                connection_string=self.connection_string,
                source_system=255,
                source_component=0,
            )
            if self.mav.connect(timeout=10.0):
                break
            self.mav.disconnect()
            self.mav = None
            time.sleep(3)

        if self.mav is None:
            logger.error("Failed to reconnect after reboot")
            logger.error(
                "SITL TCP doesn't survive soft-reboot. "
                "Restart SITL with pre-configured params instead:\n"
                "  sim_vehicle.py -v ArduCopter --add-param-file config/sitl_params.parm\n"
                "  OR: arducopter --model + ... "
                "--defaults copter.parm,config/sitl_params.parm\n"
                "Then re-run with: test_sitl.py --skip-params"
            )
            return False

        self.mav.register_callback("STATUSTEXT", self._on_statustext)

        # Wait for FC to fully initialize
        time.sleep(3)
        return True

    def _step_set_origin(self) -> bool:
        """Step 4: Set EKF origin (Warsaw coords)."""
        self.mav.set_ekf_origin(lat=52.2297, lon=21.0122, alt=100.0)
        time.sleep(1)

        # Send it twice to be sure
        self.mav.set_ekf_origin(lat=52.2297, lon=21.0122, alt=100.0)
        time.sleep(1)
        return True

    def _step_stream(self) -> bool:
        """Step 5: Stream VISION_POSITION_ESTIMATE at 20 Hz."""
        interval = 1.0 / 20.0
        t_start = time.time()
        count = 0

        while (time.time() - t_start) < self.duration:
            t = time.time() - t_start
            x, y, z, yaw = self._get_pattern_position(t)

            self.mav.send_vision_position_estimate(
                x=x, y=y, z=z,
                roll=0.0, pitch=0.0, yaw=yaw,
                confidence=1.0,
            )
            count += 1

            # Sleep to maintain rate
            elapsed = time.time() - t_start
            expected = count * interval
            sleep_time = expected - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.debug(f"Sent {count} vision messages in {self.duration:.1f}s "
                     f"({count / self.duration:.1f} Hz)")
        # Store last sent position for convergence check
        t_final = self.duration
        self._last_sent_x, self._last_sent_y, self._last_sent_z, _ = \
            self._get_pattern_position(t_final)
        return True

    def _step_check_convergence(self) -> bool:
        """Step 6: Verify FC's LOCAL_POSITION_NED matches sent vision data."""
        # Give EKF a moment to converge
        time.sleep(1)

        # Keep sending vision data while checking, otherwise EKF may diverge
        check_start = time.time()
        interval = 1.0 / 20.0
        telem = None

        while (time.time() - check_start) < 5.0:
            t = self.duration + (time.time() - check_start)
            x, y, z, yaw = self._get_pattern_position(t)
            self.mav.send_vision_position_estimate(
                x=x, y=y, z=z,
                roll=0.0, pitch=0.0, yaw=yaw,
                confidence=1.0,
            )
            time.sleep(interval)
            telem = self.mav.get_telemetry()

        if telem is None or telem.timestamp == 0:
            logger.error("No LOCAL_POSITION_NED data received from FC")
            return False

        # Final sent position
        t_final = self.duration + 5.0
        exp_x, exp_y, _, _ = self._get_pattern_position(t_final)

        dx = abs(telem.x - exp_x)
        dy = abs(telem.y - exp_y)
        error = math.sqrt(dx * dx + dy * dy)

        logger.debug(f"FC position:   x={telem.x:.3f}  y={telem.y:.3f}")
        logger.debug(f"Sent position: x={exp_x:.3f}  y={exp_y:.3f}")
        logger.debug(f"XY error: {error:.3f} m")

        # Threshold: 2m is generous for SITL EKF convergence
        if error > 2.0:
            logger.error(f"EKF diverged: error={error:.2f}m > 2.0m threshold")
            return False

        return True

    def _step_arm_flight(self) -> bool:
        """Step 7 (optional): Arm in GUIDED, takeoff, hover, land."""
        # Keep streaming vision in background
        stop_event = threading.Event()
        stream_thread = threading.Thread(
            target=self._background_stream,
            args=(stop_event,),
            daemon=True,
        )
        stream_thread.start()

        try:
            # Set GUIDED mode
            logger.debug("Setting GUIDED mode...")
            self.mav.set_mode(FlightMode.GUIDED)
            time.sleep(2)

            telem = self.mav.get_telemetry()
            if telem.mode_name != "GUIDED":
                logger.error(f"Failed to set GUIDED mode, current: {telem.mode_name}")
                return False

            # Arm
            logger.debug("Arming...")
            self.mav.arm(force=False)

            # Wait for arm
            armed = False
            for _ in range(30):
                time.sleep(0.5)
                if self.mav.is_armed:
                    armed = True
                    break
            if not armed:
                logger.error("Failed to arm (check pre-arm messages)")
                for msg in self.status_messages[-10:]:
                    logger.error(f"  FC: {msg}")
                return False

            logger.debug("Armed! Commanding takeoff to 2m...")
            self.mav.takeoff(2.0)

            # Wait for altitude
            at_altitude = False
            for _ in range(60):
                time.sleep(0.5)
                telem = self.mav.get_telemetry()
                if telem.altitude > 1.5:
                    at_altitude = True
                    break

            if not at_altitude:
                logger.warning(f"Takeoff may have failed, alt={telem.altitude:.2f}m")

            # Hover for a few seconds
            logger.debug(f"Hovering at {telem.altitude:.2f}m...")
            time.sleep(5)

            # Land
            logger.debug("Landing...")
            self.mav.set_mode(FlightMode.LAND)
            time.sleep(10)

            # Disarm
            self.mav.disarm(force=True)
            time.sleep(2)

            return True

        finally:
            stop_event.set()
            stream_thread.join(timeout=3.0)

    # ======================== HELPERS ========================

    def _get_pattern_position(self, t: float):
        """Get synthetic position for time t based on selected pattern.

        Returns (x, y, z, yaw) in NED frame.
        z=0 means ground level (no altitude from vision in this test).
        """
        if self.pattern == "hover":
            return 0.0, 0.0, 0.0, 0.0

        elif self.pattern == "square":
            # 2m square, 8s per lap
            cycle = t % 8.0
            if cycle < 2.0:
                frac = cycle / 2.0
                x = 1.0 * frac
                y = 0.0
            elif cycle < 4.0:
                frac = (cycle - 2.0) / 2.0
                x = 1.0
                y = 1.0 * frac
            elif cycle < 6.0:
                frac = (cycle - 4.0) / 2.0
                x = 1.0 * (1.0 - frac)
                y = 1.0
            else:
                frac = (cycle - 6.0) / 2.0
                x = 0.0
                y = 1.0 * (1.0 - frac)
            return x, y, 0.0, 0.0

        elif self.pattern == "circle":
            # 1.5m radius, 10s per lap
            omega = 2.0 * math.pi / 10.0
            x = 1.5 * math.cos(omega * t)
            y = 1.5 * math.sin(omega * t)
            yaw = omega * t
            return x, y, 0.0, yaw

        else:
            return 0.0, 0.0, 0.0, 0.0

    def _background_stream(self, stop_event: threading.Event):
        """Stream vision data in background (for arm/flight step)."""
        interval = 1.0 / 20.0
        t_start = time.time()

        while not stop_event.is_set():
            t = time.time() - t_start
            x, y, z, yaw = self._get_pattern_position(t)
            self.mav.send_vision_position_estimate(
                x=x, y=y, z=z,
                roll=0.0, pitch=0.0, yaw=yaw,
                confidence=1.0,
            )
            time.sleep(interval)

    def _on_statustext(self, msg):
        """Capture FC status messages."""
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode('utf-8', errors='replace')
        text = text.rstrip('\x00')
        self.status_messages.append(text)
        logger.debug(f"FC: {text}")


def main():
    parser = argparse.ArgumentParser(
        description="Validate vision-to-FC pipeline against ArduCopter SITL"
    )
    parser.add_argument(
        '-c', '--connection',
        type=str,
        default='tcp:127.0.0.1:5760',
        help='MAVLink connection string (default: tcp:127.0.0.1:5760)',
    )
    parser.add_argument(
        '-p', '--pattern',
        type=str,
        choices=['hover', 'square', 'circle'],
        default='hover',
        help='Position pattern to send (default: hover)',
    )
    parser.add_argument(
        '-d', '--duration',
        type=float,
        default=15.0,
        help='Duration to stream vision data in seconds (default: 15)',
    )
    parser.add_argument(
        '--arm',
        action='store_true',
        help='Attempt arm + guided flight after streaming',
    )
    parser.add_argument(
        '--skip-params',
        action='store_true',
        help='Skip parameter setting and reboot (for repeat runs)',
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Verbose logging',
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    print("\n" + "=" * 50)
    print("       SITL VALIDATION")
    print("=" * 50)
    print(f"  Connection:  {args.connection}")
    print(f"  Pattern:     {args.pattern}")
    print(f"  Duration:    {args.duration}s")
    print(f"  Skip params: {args.skip_params}")
    print(f"  Arm flight:  {args.arm}")

    validator = SITLValidator(
        connection_string=args.connection,
        pattern=args.pattern,
        duration=args.duration,
        attempt_arm=args.arm,
        skip_params=args.skip_params,
    )

    success = validator.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
