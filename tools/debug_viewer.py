#!/usr/bin/env python3
"""
Debug Frame Viewer (Manual Capture)

Captures debug frames from RPi on demand for analyzing:
- Camera issues (focus, exposure, framing)
- ArUco detection issues (parameters, marker quality)

Press SPACE to capture a new frame. Lightweight - no continuous streaming.

Usage:
    python3 tools/debug_viewer.py --host aruconav.local --port 8001
"""

import argparse
import cv2
import numpy as np
import urllib.request
import time
import sys
import json
from typing import Optional


def fetch_frame(url: str, timeout: float = 5.0) -> Optional[np.ndarray]:
    """Fetch a debug frame from the server."""
    try:
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=timeout) as response:
            data = response.read()
            arr = np.frombuffer(data, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"Error fetching frame: {e}")
        return None


def fetch_position(url: str, timeout: float = 2.0) -> Optional[dict]:
    """Fetch position JSON from the server."""
    try:
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.loads(response.read().decode())
    except:
        return None


def create_idle_frame(shape=(480, 640, 3)) -> np.ndarray:
    """Create the idle screen."""
    frame = np.zeros(shape, dtype=np.uint8)

    cv2.putText(frame, "ArUco Debug Viewer", (shape[1]//2 - 150, shape[0]//2 - 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.putText(frame, "Press SPACE to capture frame", (shape[1]//2 - 180, shape[0]//2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, "S = save  |  Q = quit", (shape[1]//2 - 120, shape[0]//2 + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 1)

    return frame


def main():
    parser = argparse.ArgumentParser(description="Debug frame viewer (manual capture)")
    parser.add_argument('--host', type=str, default='aruconav.local',
                        help='RPi hostname or IP (default: aruconav.local)')
    parser.add_argument('--port', type=int, default=8001,
                        help='HTTP server port (default: 8001)')
    args = parser.parse_args()

    frame_url = f"http://{args.host}:{args.port}/debug-frame"
    position_url = f"http://{args.host}:{args.port}/position"

    print(f"Debug Viewer - {args.host}:{args.port}")
    print("Controls: SPACE=capture  S=save  Q=quit")

    window_name = "ArUco Debug Viewer"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)

    current_frame = create_idle_frame()
    capture_count = 0

    while True:
        cv2.imshow(window_name, current_frame)
        key = cv2.waitKey(100) & 0xFF

        if key == ord('q') or key == 27:
            break

        elif key == ord(' '):
            print("Capturing...", end=' ', flush=True)
            frame = fetch_frame(frame_url)

            if frame is not None:
                current_frame = frame
                capture_count += 1

                # Also fetch position for console output
                pos = fetch_position(position_url)
                if pos and pos.get('x') is not None:
                    t = pos.get('timing', {})
                    timing_str = f"grab:{t.get('grab',0):.0f} gray:{t.get('gray',0):.0f} CLAHE:{t.get('clahe',0):.0f} bgr:{t.get('bgr',0):.0f} detect:{t.get('detect',0):.0f} total:{t.get('total',0):.0f}ms"
                    print(f"OK - Pos: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f}) "
                          f"Detection: {pos.get('detection_rate', 0):.0%}")
                    print(f"    [TIMING] {timing_str}")
                else:
                    print(f"OK - No position (detection rate: {pos.get('detection_rate', 0) if pos else 'N/A'})")
            else:
                print("FAILED - check connection")

        elif key == ord('s'):
            if capture_count > 0:
                filename = f"debug_{int(time.time())}.jpg"
                cv2.imwrite(filename, current_frame)
                print(f"Saved: {filename}")
            else:
                print("Capture a frame first (SPACE)")

    cv2.destroyAllWindows()
    print(f"Done. {capture_count} frames captured.")


if __name__ == "__main__":
    main()
