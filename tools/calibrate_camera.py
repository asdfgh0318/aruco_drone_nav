#!/usr/bin/env python3
"""
Camera Calibration Tool — captures chessboard images and computes intrinsics.

Usage:
    python3 tools/calibrate_camera.py [--camera 0] [--width 9] [--height 6] [--square-size 0.019]

Controls:
    SPACE  - capture frame (must detect chessboard)
    C      - run calibration with captured frames
    Q/ESC  - quit
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description="Calibrate camera using chessboard")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=9, help="Inner corners width")
    parser.add_argument("--height", type=int, default=6, help="Inner corners height")
    parser.add_argument("--square-size", type=float, default=0.019, help="Square size in meters")
    parser.add_argument("--output", type=str, default="config/camera_params.yaml")
    parser.add_argument("--res-w", type=int, default=1280, help="Camera width")
    parser.add_argument("--res-h", type=int, default=720, help="Camera height")
    args = parser.parse_args()

    board = (args.width, args.height)
    obj_p = np.zeros((board[0] * board[1], 3), np.float32)
    obj_p[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2) * args.square_size

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.res_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.res_h)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    if not cap.isOpened():
        print("Cannot open camera")
        sys.exit(1)

    ret, frame = cap.read()
    actual_w, actual_h = frame.shape[1], frame.shape[0]
    print(f"Camera opened at {actual_w}x{actual_h}")
    print(f"Chessboard: {board[0]}x{board[1]} corners, {args.square_size*1000:.1f}mm squares")
    print(f"\nSPACE=capture  C=calibrate  Q=quit\n")

    obj_points = []
    img_points = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    found = False
    corners2 = None
    search_frame = None
    import threading

    # Run chessboard detection in background thread
    detect_lock = threading.Lock()
    detect_result = [False, None, None]  # found, corners2, frame

    def detect_worker():
        while True:
            if search_frame is None:
                time.sleep(0.01)
                continue
            with detect_lock:
                sf = search_frame
            if sf is None:
                time.sleep(0.01)
                continue
            gray = cv2.cvtColor(sf, cv2.COLOR_BGR2GRAY)
            f, c = cv2.findChessboardCorners(gray, board,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
            c2 = None
            if f:
                c2 = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), criteria)
            with detect_lock:
                detect_result[0] = f
                detect_result[1] = c2
                detect_result[2] = sf

    t = threading.Thread(target=detect_worker, daemon=True)
    t.start()

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        with detect_lock:
            search_frame = frame.copy()
            found = detect_result[0]
            corners2 = detect_result[1]

        display = frame.copy()
        if found and corners2 is not None:
            cv2.drawChessboardCorners(display, board, corners2, found)
            status = f"FOUND | {len(obj_points)} captured"
            color = (0, 255, 0)
        else:
            status = f"No board | {len(obj_points)} captured"
            color = (0, 0, 255)

        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(display, "SPACE=capture C=calibrate Q=quit", (10, actual_h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.imshow("Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif key == ord(" ") and found and corners2 is not None:
            obj_points.append(obj_p)
            img_points.append(corners2)
            print(f"  Captured #{len(obj_points)}")
        elif key == ord("c"):
            if len(obj_points) < 5:
                print(f"  Need at least 5 images, have {len(obj_points)}")
                continue

            print(f"\nCalibrating with {len(obj_points)} images...")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, (actual_w, actual_h), None, None
            )

            # Compute reprojection error
            total_err = 0
            for i in range(len(obj_points)):
                proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
                err = cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
                total_err += err
            mean_err = total_err / len(obj_points)

            print(f"\nCamera matrix:\n{mtx}")
            print(f"\nDistortion: {dist.flatten()}")
            print(f"Reprojection error: {mean_err:.4f} px (< 0.5 is good)")

            # Save
            output = Path(args.output)
            if not output.is_absolute():
                output = Path(__file__).parent.parent / output

            data = {
                "image_width": actual_w,
                "image_height": actual_h,
                "camera_matrix": mtx.tolist(),
                "distortion_coefficients": dist.flatten().tolist(),
                "reprojection_error": float(mean_err),
                "calibration_images": len(obj_points),
                "chessboard_size": [board[0], board[1]],
                "square_size_m": args.square_size,
            }

            with open(output, "w") as f:
                yaml.dump(data, f, default_flow_style=None)

            print(f"\nSaved to {output}")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
