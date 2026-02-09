#!/usr/bin/env python3
"""MAVLink debug tool - diagnose connection issues with flight controller."""

import sys
import time
import argparse

def test_raw_serial(port, baud, duration=3.0):
    """Read raw bytes to verify serial connection."""
    import serial
    print(f"\n--- Raw serial test: {port} @ {baud} ---")
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(0.5)
        start = time.time()
        total = 0
        mavlink2_headers = 0
        mavlink1_headers = 0
        while time.time() - start < duration:
            data = ser.read(256)
            if data:
                total += len(data)
                mavlink2_headers += data.count(b'\xfd')
                mavlink1_headers += data.count(b'\xfe')
        ser.close()
        print(f"  Bytes received: {total}")
        print(f"  MAVLink2 headers (0xfd): {mavlink2_headers}")
        print(f"  MAVLink1 headers (0xfe): {mavlink1_headers}")
        if total > 0:
            print(f"  Byte rate: {total/duration:.0f} bytes/sec")
        return total > 0
    except Exception as e:
        print(f"  ERROR: {e}")
        return False


def test_mavlink(port, baud, timeout=10.0):
    """Test MAVLink connection with pymavlink (no monkey-patching)."""
    from pymavlink import mavutil
    print(f"\n--- MAVLink test: {port} @ {baud} ---")
    try:
        conn = mavutil.mavlink_connection(
            port, baud=baud,
            source_system=255, source_component=0
        )
        print(f"  Waiting for heartbeat (timeout={timeout}s)...")
        msg = conn.wait_heartbeat(timeout=timeout)
        if msg:
            print(f"  HEARTBEAT received!")
            print(f"  System: {msg.get_srcSystem()}, Component: {msg.get_srcComponent()}")
            print(f"  Type: {msg.type}, Autopilot: {msg.autopilot}")

            # Read a few more messages
            print("  Reading messages for 3s...")
            start = time.time()
            counts = {}
            while time.time() - start < 3.0:
                m = conn.recv_match(blocking=True, timeout=0.5)
                if m:
                    t = m.get_type()
                    counts[t] = counts.get(t, 0) + 1
            for t, c in sorted(counts.items(), key=lambda x: -x[1]):
                print(f"    {t}: {c}")
            conn.close()
            return True
        else:
            # Read raw to see what's coming
            print("  No heartbeat. Reading raw messages for 3s...")
            start = time.time()
            counts = {}
            while time.time() - start < 3.0:
                m = conn.recv_match(blocking=True, timeout=0.5)
                if m:
                    t = m.get_type()
                    counts[t] = counts.get(t, 0) + 1
            for t, c in sorted(counts.items(), key=lambda x: -x[1]):
                print(f"    {t}: {c}")
            conn.close()
            return False
    except Exception as e:
        print(f"  ERROR: {e}")
        return False


def test_baud_rates(port, bauds=None):
    """Try multiple baud rates to find the right one."""
    if bauds is None:
        bauds = [115200, 57600, 921600, 460800, 230400, 38400, 19200, 9600]

    print(f"\n=== Auto-detect baud rate on {port} ===")
    for baud in bauds:
        print(f"\nTrying {baud}...")
        has_data = test_raw_serial(port, baud, duration=2.0)
        if has_data:
            success = test_mavlink(port, baud, timeout=5.0)
            if success:
                print(f"\n*** SUCCESS at {baud} baud ***")
                return baud
            else:
                print(f"  Data flows at {baud} but no valid MAVLink")
        else:
            print(f"  No data at {baud}")

    print("\nNo working baud rate found")
    return None


def main():
    parser = argparse.ArgumentParser(description="MAVLink debug tool")
    parser.add_argument("--port", default="/dev/serial0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--auto", action="store_true", help="Auto-detect baud rate")
    parser.add_argument("--raw", action="store_true", help="Raw serial test only")
    parser.add_argument("--timeout", type=float, default=10.0, help="Heartbeat timeout")
    args = parser.parse_args()

    if args.auto:
        test_baud_rates(args.port)
    elif args.raw:
        test_raw_serial(args.port, args.baud, duration=5.0)
    else:
        test_raw_serial(args.port, args.baud, duration=3.0)
        test_mavlink(args.port, args.baud, timeout=args.timeout)


if __name__ == "__main__":
    main()
