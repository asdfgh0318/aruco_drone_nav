#!/usr/bin/env python3
"""
MAVLink Connection Test Tool

Tests MAVLink connection to ArduCopter flight controller.
Displays telemetry data and allows basic command testing.

Usage:
    python test_mavlink.py [--port /dev/serial0] [--baud 921600]
"""

import argparse
import sys
import time
import logging
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.mavlink_interface import MAVLinkInterface, FlightMode


def print_telemetry(telem, clear=True):
    """Print telemetry data."""
    if clear:
        print("\033[H\033[J", end="")  # Clear screen

    print("=" * 60)
    print("              MAVLink TELEMETRY")
    print("=" * 60)
    print()
    print(f"  Mode:     {telem.mode_name:15} Armed: {telem.armed}")
    print()
    print("  Position (NED):")
    print(f"    North: {telem.x:+8.2f} m")
    print(f"    East:  {telem.y:+8.2f} m")
    print(f"    Down:  {telem.z:+8.2f} m  (Alt: {telem.altitude:.2f} m)")
    print()
    print("  Velocity:")
    print(f"    Vn: {telem.vx:+6.2f} m/s")
    print(f"    Ve: {telem.vy:+6.2f} m/s")
    print(f"    Vd: {telem.vz:+6.2f} m/s")
    print()
    print("  Attitude:")
    print(f"    Roll:  {np.degrees(telem.roll):+7.2f} deg")
    print(f"    Pitch: {np.degrees(telem.pitch):+7.2f} deg")
    print(f"    Yaw:   {np.degrees(telem.yaw):+7.2f} deg")
    print()
    print("  Battery:")
    print(f"    Voltage: {telem.battery_voltage:5.2f} V")
    print(f"    Remaining: {telem.battery_remaining:3d} %")
    print()
    print("  GPS:")
    print(f"    Fix: {telem.gps_fix}  Sats: {telem.satellites}")
    print()
    print("-" * 60)
    print("Commands: [G]uided [L]oiter [S]tabilize [A]rm [D]isarm [Q]uit")
    print("-" * 60)


def interactive_mode(mav: MAVLinkInterface):
    """Interactive command mode."""
    import select
    import numpy as np

    print("\nEntering interactive mode...")
    print("Press keys to send commands (Q to quit)\n")

    try:
        import termios
        import tty

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        interactive = True
    except:
        print("Interactive input not available, display-only mode")
        interactive = False

    try:
        while True:
            telem = mav.get_telemetry()
            print_telemetry(telem)

            if interactive:
                # Check for input with timeout
                if select.select([sys.stdin], [], [], 0.5)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'q':
                        print("\nExiting...")
                        break
                    elif key == 'g':
                        print("\nSetting GUIDED mode...")
                        mav.set_mode(FlightMode.GUIDED)
                    elif key == 'l':
                        print("\nSetting LOITER mode...")
                        mav.set_mode(FlightMode.LOITER)
                    elif key == 's':
                        print("\nSetting STABILIZE mode...")
                        mav.set_mode(FlightMode.STABILIZE)
                    elif key == 'a':
                        print("\nArming...")
                        mav.arm()
                    elif key == 'd':
                        print("\nDisarming...")
                        mav.disarm()
                    elif key == 'h':
                        print("\nSetting ALT_HOLD mode...")
                        mav.set_mode(FlightMode.ALT_HOLD)
            else:
                time.sleep(0.5)

    finally:
        if interactive:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    import numpy as np

    parser = argparse.ArgumentParser(
        description="Test MAVLink connection to flight controller"
    )
    parser.add_argument(
        '--port', '-p',
        type=str,
        default='/dev/serial0',
        help='Serial port or connection string'
    )
    parser.add_argument(
        '--baud', '-b',
        type=int,
        default=921600,
        help='Baud rate for serial connection'
    )
    parser.add_argument(
        '--timeout', '-t',
        type=float,
        default=30.0,
        help='Connection timeout in seconds'
    )
    parser.add_argument(
        '--simple',
        action='store_true',
        help='Simple display mode (no interactive commands)'
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

    print("\n" + "=" * 50)
    print("        MAVLink CONNECTION TEST")
    print("=" * 50)
    print(f"\nConnecting to: {args.port}")
    print(f"Baud rate: {args.baud}")
    print()

    mav = MAVLinkInterface(args.port, args.baud)

    if not mav.connect(timeout=args.timeout):
        print("\nFailed to connect!")
        print("\nTroubleshooting:")
        print("  - Check that the serial port is correct")
        print("  - Verify baud rate matches flight controller")
        print("  - Ensure flight controller is powered on")
        print("  - Check cable connections")
        sys.exit(1)

    print("\nConnected successfully!")

    try:
        if args.simple:
            # Simple display mode
            print("Press Ctrl+C to exit\n")
            while True:
                telem = mav.get_telemetry()
                print(
                    f"\rMode: {telem.mode_name:12} | "
                    f"Armed: {str(telem.armed):5} | "
                    f"Alt: {telem.altitude:6.2f}m | "
                    f"Batt: {telem.battery_voltage:5.2f}V",
                    end=""
                )
                time.sleep(0.2)
        else:
            interactive_mode(mav)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        mav.disconnect()
        print("Disconnected")


if __name__ == "__main__":
    main()
