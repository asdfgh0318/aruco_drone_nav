#!/usr/bin/env python3
"""Quick test: send GPS_INPUT to FC and check if it gets a fix."""

import time
import math
import sys

def main():
    port = "/dev/serial0"
    baud = 115200

    # Workaround for RPi PL011 pyserial bug
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

    from pymavlink import mavutil

    print(f"Connecting to {port} @ {baud}...")
    conn = mavutil.mavlink_connection(port, baud=baud, source_system=255, source_component=0)

    # Send heartbeats to wake FC
    print("Sending heartbeats...")
    for _ in range(5):
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)
        time.sleep(0.3)

    print("Waiting for FC heartbeat...")
    while True:
        m = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if m and m.get_srcSystem() != 255:
            print(f"FC connected: system {m.get_srcSystem()}")
            break
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

    # GPS origin (Warsaw)
    origin_lat = 52.2297
    origin_lon = 21.0122
    origin_alt = 100.0
    gps_epoch = 315964800.0

    print(f"\nSending GPS_INPUT at ({origin_lat}, {origin_lon}, {origin_alt}m)...")
    print("Check Mission Planner for GPS fix. Press Ctrl+C to stop.\n")

    count = 0
    last_hb = time.time()
    last_status = time.time()

    try:
        while True:
            now = time.time()

            # Heartbeat at 1Hz
            if now - last_hb >= 1.0:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0)
                last_hb = now

            # GPS_INPUT at 5Hz
            gps_seconds = now - gps_epoch
            gps_week = int(gps_seconds / 604800)
            gps_week_ms = int((gps_seconds % 604800) * 1000)

            conn.mav.gps_input_send(
                int(now * 1e6),       # time_usec
                0,                    # gps_id
                (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                 mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY),
                gps_week_ms,          # time_week_ms
                gps_week,             # time_week
                3,                    # fix_type: 3D fix
                int(origin_lat * 1e7),
                int(origin_lon * 1e7),
                origin_alt,
                0.8,                  # hdop
                1.2,                  # vdop
                0.0, 0.0, 0.0,       # vn, ve, vd
                0.01,                 # speed_accuracy
                0.3,                  # horiz_accuracy
                0.5,                  # vert_accuracy
                12,                   # satellites_visible
                0,                    # yaw (0 = north)
            )
            count += 1

            # Check for GPS_RAW_INT from FC every 2s
            if now - last_status >= 2.0:
                print(f"Sent {count} GPS_INPUT messages")
                # Drain and check messages
                while True:
                    m = conn.recv_match(blocking=False)
                    if not m:
                        break
                    if m.get_type() == 'GPS_RAW_INT':
                        fix = m.fix_type
                        sats = m.satellites_visible
                        lat = m.lat / 1e7
                        lon = m.lon / 1e7
                        print(f"  >> GPS_RAW_INT: fix={fix}, sats={sats}, lat={lat:.7f}, lon={lon:.7f}")
                last_status = now

            time.sleep(0.2)  # 5Hz

    except KeyboardInterrupt:
        print(f"\nDone. Sent {count} GPS_INPUT messages.")
    finally:
        conn.close()

if __name__ == "__main__":
    main()
