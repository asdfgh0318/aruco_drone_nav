#!/usr/bin/env python3
"""
Camera Level Calibration — records marker center offset for skewed camera mounting.

Place drone directly under an ArUco marker, perfectly level.
The tool detects the marker, records where its center appears in the image,
and saves the pixel offset from the image center. This offset is used during
flight to correct for imperfect camera mounting.

Usage:
    python3 tools/calibrate_level.py [--camera 0] [--samples 30]

Controls:
    SPACE  - capture offset (averages over --samples frames when marker visible)
    Q/ESC  - quit without saving

Runs on RPi (headless, prints to terminal) or laptop (OpenCV window).
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

# Add parent for imports
sys.path.insert(0, str(Path(__file__).parent.parent))


def main():
    parser = argparse.ArgumentParser(description="Calibrate camera mounting offset")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--samples", type=int, default=30, help="Frames to average")
    parser.add_argument("--config", type=str, default="config/system_config.yaml")
    parser.add_argument("--output", type=str, default="config/camera_params.yaml")
    parser.add_argument("--headless", action="store_true", help="No GUI, auto-capture")
    args = parser.parse_args()

    root = Path(__file__).parent.parent
    config_path = root / args.config
    output_path = root / args.output

    # Load system config for ArUco params
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    aruco_dict_name = cfg.get("aruco", {}).get("dictionary", "DICT_4X4_50")
    marker_size = cfg.get("aruco", {}).get("marker_size_m", 0.18)

    # Load camera params
    with open(output_path) as f:
        cam_cfg = yaml.safe_load(f)
    cam_matrix = np.array(cam_cfg["camera_matrix"])
    dist_coeffs = np.array(cam_cfg["distortion_coefficients"])

    # Setup ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, aruco_dict_name))
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()

    # Open camera
    res_w = cfg.get("camera", {}).get("width", 1280)
    res_h = cfg.get("camera", {}).get("height", 720)
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
    # Lock focus at infinity
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)

    if not cap.isOpened():
        print("Cannot open camera")
        sys.exit(1)

    ret, frame = cap.read()
    actual_w, actual_h = frame.shape[1], frame.shape[0]
    img_center = np.array([actual_w / 2.0, actual_h / 2.0])

    print(f"Camera: {actual_w}x{actual_h}")
    print(f"Image center: ({img_center[0]:.1f}, {img_center[1]:.1f})")
    print(f"\nPlace drone DIRECTLY UNDER the marker, perfectly level.")
    if args.headless:
        print(f"Auto-capturing {args.samples} samples...")
    else:
        print(f"Press SPACE to capture {args.samples} samples. Q to quit.\n")

    collected = []
    capturing = args.headless  # Auto-start in headless mode

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

        marker_center = None
        marker_tvec = None
        if ids is not None and len(ids) > 0:
            # Use first detected marker
            c = corners[0].reshape(-1, 2)
            marker_center = c.mean(axis=0)

            # solvePnP to get tvec (needed for tilt correction)
            half = marker_size / 2
            obj_pts = np.array([[-half, half, 0], [half, half, 0],
                                [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
            ok, rvec, tvec = cv2.solvePnP(obj_pts, c, cam_matrix, dist_coeffs,
                                           flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                marker_tvec = tvec.flatten()

            if capturing and marker_tvec is not None:
                collected.append(marker_tvec.copy())
                if not args.headless:
                    print(f"\r  Sample {len(collected)}/{args.samples} "
                          f"center=({marker_center[0]:.1f}, {marker_center[1]:.1f})", end="")

        # Display (if not headless)
        if not args.headless:
            display = frame.copy()
            # Draw image center crosshair (blue)
            cv2.drawMarker(display, (int(img_center[0]), int(img_center[1])),
                          (255, 100, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(display, "IMG", (int(img_center[0]) + 15, int(img_center[1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)

            if marker_center is not None:
                # Draw marker center (green)
                mc = (int(marker_center[0]), int(marker_center[1]))
                cv2.drawMarker(display, mc, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
                cv2.putText(display, "CODE", (mc[0] + 15, mc[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                # Draw offset line (yellow)
                cv2.line(display, (int(img_center[0]), int(img_center[1])), mc, (0, 255, 255), 1)
                offset = marker_center - img_center
                cv2.putText(display, f"offset: ({offset[0]:+.1f}, {offset[1]:+.1f}) px",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if ids is not None and corners is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

            status = f"Samples: {len(collected)}/{args.samples}" if capturing else "SPACE to capture"
            color = (0, 255, 0) if capturing else (200, 200, 200)
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(display, "SPACE=capture Q=quit", (10, actual_h - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.imshow("Level Calibration", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                print("\nAborted.")
                break
            elif key == ord(" ") and not capturing:
                capturing = True
                collected = []
                print("Capturing...")

        # Check if done
        if len(collected) >= args.samples:
            tvecs = np.array(collected)
            avg_tvec = tvecs.mean(axis=0)
            std = tvecs.std(axis=0)

            # Compute tilt angle
            v = avg_tvec / np.linalg.norm(avg_tvec)
            tilt_deg = np.degrees(np.arccos(np.clip(v[2], -1, 1)))

            print(f"\n\nAvg tvec: ({avg_tvec[0]:+.4f}, {avg_tvec[1]:+.4f}, {avg_tvec[2]:+.4f}) m")
            print(f"Std dev:  ({std[0]:.4f}, {std[1]:.4f}, {std[2]:.4f}) m")
            print(f"Camera tilt from vertical: {tilt_deg:.1f} deg")
            print(f"Height to marker: {np.linalg.norm(avg_tvec):.3f} m")

            # Save to camera_params.yaml
            cam_cfg["level_tvec"] = [float(avg_tvec[0]), float(avg_tvec[1]), float(avg_tvec[2])]

            with open(output_path, "w") as f:
                yaml.dump(cam_cfg, f, default_flow_style=None)

            print(f"\nSaved to {output_path}")
            print(f"  level_tvec: [{avg_tvec[0]:+.4f}, {avg_tvec[1]:+.4f}, {avg_tvec[2]:+.4f}]")
            print(f"  level_offset_px: [{offset[0]:+.1f}, {offset[1]:+.1f}]")
            print(f"  level_center_px: [{avg_center[0]:.1f}, {avg_center[1]:.1f}]")
            break

    cap.release()
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass


if __name__ == "__main__":
    main()
