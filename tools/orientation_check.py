#!/usr/bin/env python3
"""
Orientation Check Tool

Determines correct axis mapping and yaw orientation by measuring
the drone's position at two known points (M0 and toward M1).

Usage:
    python3 tools/orientation_check.py [--host 192.168.213.251:8001]
"""

import sys
import json
import time
try:
    from urllib.request import urlopen
except ImportError:
    from urllib2 import urlopen


def fetch(host):
    """Fetch position from RPi."""
    try:
        r = urlopen(f"http://{host}/position", timeout=2)
        return json.loads(r.read())
    except Exception as e:
        return None


def avg_position(host, samples=20, delay=0.15):
    """Average position over multiple samples for stability."""
    ns, es, yaws = [], [], []
    print(f"  Sampling {samples} readings...", end="", flush=True)
    for i in range(samples):
        d = fetch(host)
        if d and "n" in d:
            ns.append(d["n"])
            es.append(d["e"])
            yaws.append(d["yaw"])
            print(".", end="", flush=True)
        else:
            print("x", end="", flush=True)
        time.sleep(delay)
    print()
    if not ns:
        return None
    return {
        "n": sum(ns) / len(ns),
        "e": sum(es) / len(es),
        "yaw": sum(yaws) / len(yaws),
        "count": len(ns),
    }


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.213.251:8001"

    print("=" * 60)
    print("  ORIENTATION CHECK TOOL")
    print("=" * 60)
    print(f"\n  Host: {host}")
    print()

    # Check connection
    d = fetch(host)
    if not d:
        print("  ERROR: Cannot connect to RPi. Is the service running?")
        sys.exit(1)
    if "n" not in d:
        print("  ERROR: No position fix. Is a marker visible?")
        sys.exit(1)
    print(f"  Connected. Detection rate: {d.get('detection_rate', '?')}")
    print(f"  Current: N={d['n']:.3f} E={d['e']:.3f} yaw={d['yaw']:.1f}")

    # Marker positions from YAML (M0 and M1)
    print()
    print("-" * 60)
    print("  STEP 1: Place drone directly UNDER marker M0")
    print("  Make sure M0 is detected (check live map)")
    input("  Press ENTER when ready... ")

    pos_a = avg_position(host)
    if not pos_a:
        print("  ERROR: No readings. Check detection.")
        sys.exit(1)
    print(f"  Position A (under M0): N={pos_a['n']:.3f} E={pos_a['e']:.3f} yaw={pos_a['yaw']:.1f}° ({pos_a['count']} samples)")

    print()
    print("-" * 60)
    print("  STEP 2: Move drone ~1m TOWARD marker M1")
    print("  (along the corridor, away from M0)")
    print("  Keep the drone facing the SAME direction")
    input("  Press ENTER when ready... ")

    pos_b = avg_position(host)
    if not pos_b:
        print("  ERROR: No readings. Check detection.")
        sys.exit(1)
    print(f"  Position B (toward M1): N={pos_b['n']:.3f} E={pos_b['e']:.3f} yaw={pos_b['yaw']:.1f}° ({pos_b['count']} samples)")

    # Analyze movement
    dn = pos_b["n"] - pos_a["n"]
    de = pos_b["e"] - pos_a["e"]
    dist = (dn**2 + de**2)**0.5

    print()
    print("=" * 60)
    print("  RESULTS")
    print("=" * 60)
    print(f"  Delta: dN={dn:+.3f} (North)  dE={de:+.3f} (East)")
    print(f"  Distance moved: {dist:.3f}m")

    if dist < 0.2:
        print("  WARNING: Moved less than 20cm — not enough to determine direction reliably.")
        print("  Move further and try again.")
        sys.exit(1)

    # Expected: M0(N=4.85, E=7.14) → M1(N=3.68, E=7.09) in NED
    # Moving M0→M1 should show: dN≈-1.17 (big negative = South), dE≈-0.05 (tiny)
    import math
    measured_angle = math.degrees(math.atan2(de, dn))  # atan2(East, North) = compass heading
    expected_angle = math.degrees(math.atan2(7.09 - 7.14, 3.68 - 4.85))  # M0→M1

    print()
    print(f"  Measured movement direction: {measured_angle:.1f}° (0=N, 90=E, -90=W)")
    print(f"  Expected M0→M1 direction:   {expected_angle:.1f}°")
    angle_error = measured_angle - expected_angle
    # Normalize to -180..180
    while angle_error > 180: angle_error -= 360
    while angle_error < -180: angle_error += 360
    print(f"  Angle error: {angle_error:.1f}°")

    if abs(angle_error) < 20:
        print("  ✓ XY AXES CORRECT — position matches physical movement")
    elif abs(abs(angle_error) - 180) < 20:
        print("  ✗ XY AXES INVERTED — 180° off, likely an axis is negated")
    elif abs(abs(angle_error) - 90) < 20:
        print("  ✗ XY AXES ROTATED — 90° off, axes may be swapped")
    else:
        print(f"  ? Unexpected angle error — check marker positions in YAML")

    # Yaw check
    print()
    print("-" * 60)
    print("  STEP 3: Point the drone's NOSE toward M1")
    print("  (toward the next marker in the corridor)")
    input("  Press ENTER when ready... ")

    pos_c = avg_position(host)
    if not pos_c:
        print("  ERROR: No readings.")
        sys.exit(1)

    # Facing M0→M1 = approximately South = yaw should be ~180°
    # In NED, yaw is compass heading: 0=North, 90=East, CW positive
    actual_yaw = pos_c["yaw"]
    # Expected heading: same as measured_angle (already compass heading in NED)
    expected_compass = (expected_angle + 360) % 360

    print(f"  Measured yaw:  {actual_yaw:.1f}°")
    print(f"  Expected yaw:  {expected_compass:.1f}° (compass, 0=N, CW)")
    yaw_error = actual_yaw - expected_compass
    while yaw_error > 180: yaw_error -= 360
    while yaw_error < -180: yaw_error += 360
    print(f"  Yaw error: {yaw_error:.1f}°")

    if abs(yaw_error) < 20:
        print("  ✓ YAW CORRECT — orientation value in YAML is right")
    else:
        # Suggest orientation correction
        # Current orientation adds to the yaw. To fix, subtract the error.
        current_ori = 0  # current YAML value
        suggested_ori = (current_ori - yaw_error) % 360
        print(f"  ✗ YAW OFF by {yaw_error:.0f}°")
        print(f"  → Suggested YAML orientation: {suggested_ori:.0f}°")
        print(f"    (current: {current_ori}°, correction: {-yaw_error:.0f}°)")

    print()
    print("=" * 60)
    print("  DONE")
    print("=" * 60)


if __name__ == "__main__":
    main()
